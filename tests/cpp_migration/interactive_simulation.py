"""Exercise the actual C++ interactive command path, without opening DDS."""
import argparse
import csv
import pathlib
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--binary', required=True)
parser.add_argument('--root', required=True)
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix='g1_sim2sim_') as work:
    work = pathlib.Path(work)
    script, trace = work/'commands.yaml', work/'trace.csv'
    script.write_text('''- {tick: 0, command: TAKEOVER}
- {tick: 1, command: START}
- {tick: 120, command: PAUSE}
- {tick: 121, command: START}
- {tick: 122, command: NEXT VELOCITY}
- {tick: 123, command: TAKEOVER}
- {tick: 130, command: RESUME}
- {tick: 135, command: START}
- {tick: 180, command: PAUSE}
- {tick: 185, command: RESUME}
- {tick: 190, command: NEXT VELOCITY}
- {tick: 260, command: START}
- {tick: 260, command: "  StOp  "}
''')
    result = subprocess.run([args.binary, '--root', args.root, '--mode', 'sim2sim',
        '--policy', 'DaNcE', '--headless', '--fast', '--steps', '300',
        '--script', str(script), '--trace', str(trace)], text=True, capture_output=True, timeout=45)
    assert result.returncode == 0, result.stdout+result.stderr
    for expected in ('RL/Active/Mimic_Dance1_subject2', 'RL/Active/Velocity',
                     'Stand interpolation is not complete', 'Finished: Passive'):
        assert expected in result.stdout, (expected, result.stdout)
    assert result.stdout.count('Simulation is paused') >= 3, result.stdout
    with trace.open() as source:
        rows = {int(row['tick']): row for row in csv.DictReader(source)}
    assert max(rows) == 260, 'STOP did not end the session'
    assert rows[260]['state'] == 'Passive'
    assert rows[135]['selected'] == 'Mimic_Dance1_subject2', 'paused NEXT changed selection'
    for begin, end in ((120, 129), (180, 184)):
        for tick in range(begin, end+1):
            assert rows[tick]['time'] == rows[begin]['time'], 'physics advanced while paused'
            assert rows[tick]['policy_frame'] == rows[begin]['policy_frame'], 'policy advanced while paused'
            assert rows[tick]['paused'] == '1'
    for tick in range(186, 190):
        assert abs(float(rows[tick]['time'])-float(rows[tick-1]['time'])-.02) < 1e-8
        assert int(rows[tick]['policy_frame']) == int(rows[tick-1]['policy_frame'])+1
    # Reversing simultaneous input order must still let STOP win.
    script.write_text('- {tick: 0, command: " STOP "}\n- {tick: 0, command: TAKEOVER}\n')
    result = subprocess.run([args.binary, '--root', args.root, '--mode', 'sim2sim',
        '--headless', '--fast', '--steps', '2', '--script', str(script)],
        text=True, capture_output=True, timeout=30)
    assert result.returncode == 0 and 'Finished: Passive' in result.stdout, result.stdout+result.stderr
    assert 'tick 0: Stand' not in result.stdout
    # Preserve the final non-newline command and terminate pure terminal sessions at EOF.
    for tail in ('  sToP  ', ''):
        tail_args = ['--steps', '2', '--trace', str(trace)] if tail else []
        result = subprocess.run([args.binary, '--root', args.root, '--mode', 'sim2sim',
            '--headless', *tail_args], input=tail, text=True, capture_output=True, timeout=30)
        assert result.returncode == 0 and 'Finished: Passive' in result.stdout, result.stdout+result.stderr
        if tail:
            with trace.open() as source:
                assert len(list(csv.DictReader(source))) == 1, 'Final non-newline STOP was lost'
print('Interactive sim2sim: aliases, selected actor, pause/frozen frames, resume cadence, STOP priority and EOF passed')
