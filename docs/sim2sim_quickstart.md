# G1 sim2sim 课堂操作速查卡

目标：在电脑里的 G1 虚拟机器人上完成一次马步冲拳。无需连接真机。

## 1. 打开终端，进入项目

```bash
cd ~/unitree_rl_mjlab_cpp_deploy_kungfubot
./run_policy.sh --list
```

确认列表里有 `kungfubot_horse_stance_punch`。第一次使用、尚未克隆或编译的同学，先完成[完整培训教程第 7 节](sim2sim_training.md#7-首次准备环境与重新编译)的环境准备与编译；克隆命令见该教程第 2 节。

## 2. 启动仿真

```bash
./run_sim2sim.sh --policy kungfubot_horse_stance_punch
```

出现仿真窗口和 `WAIT_TAKEOVER`：程序在等待接管。

## 3. 按顺序完成操作

**在启动程序的同一个终端中输入，每条后按回车。逐步操作，不要一次粘贴全部命令。**

| 顺序 | 输入 | 看见什么再继续 |
|---|---|---|
| ① 接管 | `TAKEOVER` | 等约 4 秒出现 `STAND_PRE`，再观察站稳约 2 秒 |
| ② 开始动作 | `START` | 自动准备、播放、返回；约 7.5 秒后出现 `STAND_POST` |
| ③ 检查结果 | `STATUS` | 状态含 `STAND_POST`，进度为 `332/332`，再观察站立 1～2 秒 |
| ④ 退出 | `STOP` | 进入 `DAMP`，约 1 秒后程序退出并打印日志路径 |

记住：**先接管 → 等站稳 → 再开始 → 做完回站立 → 最后停止。**

## 4. 暂停和看画面

- 终端输入 `PAUSE`：暂停；输入 `RESUME`：恢复。
- 鼠标左键拖动旋转视角，右键拖动平移，滚轮缩放。
- 点击仿真窗口后：`T` 接管，`S` 开始，Space 暂停/恢复，`Q` 停止退出。
- 需要提前结束时，输入 `STOP`。

## 5. 检查是否完成

退出后，到终端打印的 `Logs` 目录中查看 `summary.json`：

```text
error                          false
motion_complete                true
policy_ticks                   332
expected_policy_ticks          332
standing_after_motion          true
standing_recovered_last_1s     true
```

`stop_reason` 是 `STOP requested` 可以是正常结束。只看到窗口或程序退出，不代表动作完整完成。

## 6. 再练一次 / 换成勾拳

先 `STOP` 退出，再重新启动。勾拳用下面的命令，后续步骤相同：

```bash
./run_sim2sim.sh --policy kungfubot_hooks_punch
```

注意：勾拳的完整帧数是 `291/291`。动作结束后，再次输入 `START` 不会自动重播。

遇到报错，记录**动作名、阶段、报错、日志路径**，查看[完整培训教程](sim2sim_training.md)中的环境准备和常见问题。

动作数据来自 [KungfuBot / PBHC](https://github.com/TeleHuman/PBHC)，感谢原作者团队的贡献。论文引用与 BibTeX 见[数据来源、引用与致谢](acknowledgements.md)。
