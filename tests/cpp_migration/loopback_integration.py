#!/usr/bin/env python3
"""Exercise actual g1_deploy over local DDS only; never accepts a robot NIC/domain."""
import argparse
import csv
import json
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--only', nargs='+', help='run selected named cases')
    args = parser.parse_args()
    root, binary, fixture = args.root.resolve(), args.binary.resolve(), args.fixture.resolve()
    # The executable also hardcodes lo and rejects domain zero. Disjoint domains
    # isolate each case and the PID offset reduces collisions with other tests.
    first_domain = 90 + os.getpid() % 30
    reports = []

    def stop_process(process):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

    with tempfile.TemporaryDirectory(prefix='g1_local_dds_') as work:
        work = Path(work)
        for index, case in enumerate(('readonly_100', 'readonly_reference_goal', 'wait_stop', 'takeover_stop', 'auto_start_count')):
            if args.only and case not in args.only:
                continue
            directory = work/case
            directory.mkdir()
            domain = first_domain+index
            capture = directory/'commands.csv'
            fake = subprocess.Popen([str(fixture), str(domain), str(capture)], stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1)
            process = None
            try:
                ready_lines = queue.Queue()
                def drain_fake(stream):
                    for text in stream:
                        ready_lines.put(text)
                threading.Thread(target=drain_fake, args=(fake.stdout,), daemon=True).start()
                start = time.monotonic()
                while True:
                    try:
                        line = ready_lines.get(timeout=.1)
                    except queue.Empty:
                        line = ''
                    if 'LOOPBACK_READY' in line:
                        break
                    if fake.poll() is not None or time.monotonic()-start > 10:
                        raise RuntimeError('local DDS fixture did not become ready: '+line)
                policy = 'kungfubot_horse_stance_pose' if case == 'readonly_reference_goal' else 'straight'
                mode = 'readonly' if case.startswith('readonly') else 'real'
                command = [str(binary), '--root', str(root), '--mode', mode, '--policy', policy,
                           '--iface', 'lo', '--domain', str(domain), '--output', str(directory/'run')]
                if case == 'readonly_reference_goal': command += ['--steps', '2']
                if case == 'auto_start_count': command += ['--auto-start', '--steps', '5', '--pre-s', '.04']
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=subprocess.STDOUT, text=True, bufsize=1)
                lines, events = [], queue.Queue()
                def drain():
                    for text in process.stdout:
                        lines.append(text)
                        events.put(text)
                reader = threading.Thread(target=drain, daemon=True)
                reader.start()
                def until(marker, timeout=30):
                    end = time.monotonic()+timeout
                    while time.monotonic()<end:
                        try: text = events.get(timeout=.1)
                        except queue.Empty:
                            if process.poll() is not None: break
                            continue
                        if marker in text: return
                    raise RuntimeError(f'{case}: missing {marker}\n'+''.join(lines))
                if case in ('wait_stop', 'takeover_stop'):
                    until('WAIT_TAKEOVER')
                    if case == 'takeover_stop':
                        process.stdin.write('TAKEOVER\n');process.stdin.flush()
                        end = time.monotonic()+10
                        while time.monotonic() < end:
                            captured = list(csv.DictReader(capture.open()))
                            if any(float(row['kp_max']) > 0 for row in captured):
                                break
                            if process.poll() is not None:
                                raise RuntimeError('runner exited before takeover: '+''.join(lines))
                            time.sleep(.01)
                        else:
                            raise RuntimeError('no active command appeared after TAKEOVER')
                        time.sleep(.08)
                    process.stdin.write('STOP\n');process.stdin.flush()
                status = process.wait(timeout=40)
                reader.join(timeout=2)
                assert status == 0, f'{case}: exit={status}\n'+''.join(lines)
                # Let the final sample reach the independent subscriber before teardown.
                time.sleep(.05)
                stop_process(fake)
                rows = list(csv.DictReader(capture.open()))
                report = json.loads((directory/'run/summary.json').read_text())
                trace = np.load(directory/'run/trace.npz')
                assert not report['error'], (case, report)
                assert all(row['crc_ok'] == '1' for row in rows), 'bad lowcmd CRC'
                if case.startswith('readonly'):
                    expected = 100 if case == 'readonly_100' else 2
                    assert report['phase_ticks'] == {'READONLY': expected}, report
                    assert not rows and report['robot_commands_published'] == 0, 'readonly published lowcmd'
                    assert report.get('bridge_publisher_created') is False, 'readonly created a publisher'
                    obs = trace['observation']
                    width = 1720 if case == 'readonly_reference_goal' else 1012
                    assert obs.shape == (expected, width) and np.isfinite(obs).all(), obs.shape
                    if case == 'readonly_reference_goal':
                        assert np.any(np.abs(obs[:, 990:]) > 0), 'Reference goal missing from trace'
                elif case == 'wait_stop':
                    assert not rows and report['robot_commands_published'] == 0, 'WAIT_TAKEOVER STOP published lowcmd'
                elif case == 'takeover_stop':
                    active = [r for r in rows if float(r['kp_max'])>0]
                    damp = [r for r in rows if float(r['kp_max'])==0]
                    assert active and damp, 'takeover or damping not observed'
                    assert report['phase_ticks'] == {'DAMP': 50}, 'STOP was not processed during TAKEOVER'
                    assert report['phase_ticks'].get('DAMP') == 50, report
                    elapsed = float(damp[-1]['time'])-float(damp[0]['time'])
                    assert .94 <= elapsed <= 1.15, f'damping was not exactly one second: {elapsed}'
                    assert all(float(r['kd_min']) == 3 and float(r['kd_max']) == 3 for r in damp)
                    assert float(damp[0]['time']) >= float(active[-1]['time']), 'active command after STOP'
                else:
                    assert report['phase_ticks'] == {'STAND_PRE': 2, 'PREPARE': 3}, report
                    assert len(trace['phase']) == 5, '200 takeover ticks consumed --steps'
                    assert any(float(r['kp_max'])>0 for r in rows), 'auto-start never took over'
                item = dict(case=case, domain=domain, iface='lo', phase_ticks=report['phase_ticks'],
                            captured_commands=len(rows), runner_published=report['robot_commands_published'])
                if case == 'takeover_stop': item['damping_seconds'] = elapsed
                if case.startswith('readonly'): item['publisher_created'] = report['bridge_publisher_created']
                reports.append(item)
                print(json.dumps(item), flush=True)
            finally:
                if process is not None: stop_process(process)
                stop_process(fake)
    result = dict(status='passed', cases=reports, scope='Actual local DDS transport, positive test domains, hardcoded loopback interface; no physical robot.')
    if args.report: args.report.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
