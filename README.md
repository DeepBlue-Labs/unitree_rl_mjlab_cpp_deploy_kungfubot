# G1 C++ 部署

仅保留 G1 29 自由度部署。主程序、ONNX 推理、状态机、MuJoCo 仿真与 DDS 通信均为 C++。
当前注册表包含 7 个 KungfuBot 动作，另保留原 C++ Velocity、舞蹈、Stand、Passive。
训练代码和其他机器人未恢复。开发检查使用离线回放/仿真，不代表完成实机验收。

**动作数据来源与致谢：**当前 7 个动作使用了 [KungfuBot / PBHC](https://github.com/TeleHuman/PBHC) 项目提供的动作数据，策略使用 SuperMimic 训练。感谢 KungfuBot 原作者团队的工作与开放贡献。官方项目、论文和 BibTeX 见[数据来源、引用与致谢](docs/acknowledgements.md)。

**用于培训：请从 [sim2sim 零基础培训教程](docs/sim2sim_training.md) 开始。** 教程按当前 7 个 KungfuBot 动作编写，包含逐步操作、预期现象、流程图、日志检查、常见问题与课堂练习。动手时可使用[课堂操作速查卡](docs/sim2sim_quickstart.md)。

## 获取工程与培训材料

```bash
git clone https://github.com/DeepBlue-Labs/unitree_rl_mjlab_cpp_deploy_kungfubot.git ~/unitree_rl_mjlab_cpp_deploy_kungfubot
cd ~/unitree_rl_mjlab_cpp_deploy_kungfubot
```

- [完整培训教程](docs/sim2sim_training.md)：从检查文件到播放动作、回站立、退出与日志验收。
- [课堂操作速查卡](docs/sim2sim_quickstart.md)：学员动手时逐步对照。

仓库包含源码、7 个 KungfuBot 动作部署文件、G1 模型及编译所需的附带库。编译目录和运行日志由每台电脑自行生成。

## 目录

```text
unitree_rl_mjlab_cpp_deploy_kungfubot/
├── build_cpp.sh                 编译并运行本地检查
├── run_policy.sh / run_cpp.sh   同一个 C++ 部署入口
├── run_sim2sim.sh               交互仿真快捷入口
├── prepare_robot.sh             C++ 网卡/控制权准备入口
├── site.json                    网卡、模型、状态超时、温度等现场参数
├── deploy/robots/g1/
│   ├── cpp_main.cpp             主程序
│   ├── prepare_main.cpp         连接准备工具
│   ├── include/control/         接口
│   ├── src/control/             策略、Session 衔接、顶层状态机、仿真、通信
│   ├── config/                  原 C++ Stand、Velocity、舞蹈
│   └── main.cpp、src/State_*    保留的原版 C++ 控制器
├── policies/
│   ├── registry.json            全部动作、别名、衔接参数与来源
│   ├── standing/                原 Python 站立配置；共享原 ONNX 权重
│   └── kungfubot_…/             7 个动作所需权重、配置、参考轨迹
├── assets/supermimic_g1/         G1 MuJoCo 模型
├── vendor/unitree_sdk2/          SDK
├── deploy/thirdparty/            ONNX Runtime、cnpy
├── simulate/mujoco/              MuJoCo C 库
├── tests/cpp_migration/          离线测试、Python 对照工具与结果
├── results/                     每次运行的日志
└── build/                       C++ 可执行文件
```

## 编译

Ubuntu 22.04 / x86_64。首次克隆后没有 `build` 目录，安装缺少的依赖后编译：

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libyaml-cpp-dev \
  libboost-program-options-dev libfmt-dev libspdlog-dev zlib1g-dev \
  libssl-dev libjsoncpp-dev libglfw3-dev libgl1-mesa-dev libegl1-mesa-dev \
  ffmpeg iproute2 python3 python3-numpy
cd ~/unitree_rl_mjlab_cpp_deploy_kungfubot
cmake -S deploy/robots/g1 -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./run_policy.sh --list
./run_policy.sh --policy kungfubot_horse_stance_punch --inspect
```

若 `build` 缓存来自其他目录，重新编译前先保留旧目录并重新配置，详见[培训教程第 7 节](docs/sim2sim_training.md#7-首次准备环境与重新编译)。`./build_cpp.sh` 会编译全部目标并运行整套历史测试；部分历史场景依赖原完整工程的动作注册表。

部署不需要 Conda 或 Python 推理环境；Python 仅用于部分离线测试驱动。
`--inspect` 只验证文件并输出 JSON，不连接 DDS。当前附带的 MuJoCo 为 x86_64；ARM 需换用 ARM MuJoCo 后重新编译。

## 交互式 sim2sim

以马步冲拳为例，先在 Linux 终端启动：

```bash
./run_sim2sim.sh --policy kungfubot_horse_stance_punch
# 等价：./run_policy.sh --policy kungfubot_horse_stance_punch --mode sim2sim
```

打开窗口后，回到**启动程序的同一个终端**，按下面顺序逐条输入，每条后按回车。不要一次性粘贴所有命令。

| 操作 | 等待并确认 |
|---|---|
| 输入 `TAKEOVER` | 默认约 4 秒，出现 `STAND_PRE`，再观察机器人是否站稳 |
| 输入 `START` | 自动经过 `PREPARE → POLICY → RETURN → STAND_POST`，马步冲拳约需 7.5 秒 |
| 输入 `STATUS` | 动作后状态为 `STAND_POST`，进度为 `332/332`；继续观察站立至少 1～2 秒 |
| 输入 `STOP` | 进入 `DAMP`，默认约 1 秒后退出，打印日志路径 |

当前 7 个 KungfuBot 动作使用单次播放流程：

```text
WAIT_TAKEOVER → TAKEOVER → STAND_PRE → PREPARE → POLICY → RETURN → STAND_POST
               ↑ TAKEOVER             ↑ START                           ↓ STOP
                                                                       DAMP → 退出
```

`START` 不会重复播放已完成的动作。初学时 `STOP` 后重新启动即可重播或更换动作；熟练后可在
`STAND_POST` 输入 `policy kungfubot_hooks_punch` 直接开始勾拳的准备阶段。
`HELP` 查看当前会话帮助，`STATUS` 查看动作进度。当前注册表不提供旧版 `entry_star`、`straight`、`hook` 格斗会话示例。

| 窗口键（先点击仿真窗口） | 功能 |
|---|---|
| T | TAKEOVER |
| S / Enter | START |
| Space | 暂停/恢复，物理和策略帧都停止推进 |
| Q / Esc | STOP，阻尼后退出 |
| P | 切到原 C++ Passive；与 STOP 退出不同 |
| [ / ] 或左右键 | 选择策略；初学保持默认选择 |

鼠标左拖旋转、右拖平移、滚轮缩放；双击选中部位，Ctrl+右拖施力、Ctrl+左拖施加扭转，松开即清除。暂停时不施力。终端也支持 `PAUSE`（切换暂停/恢复）、`RESUME`（恢复）。
虚拟吊架只在 `WAIT_TAKEOVER`、`TAKEOVER`、`DAMP` 启用；站立与动作运行时释放。
`--headless` 可只用终端交互；关闭窗口、EOF、Ctrl+C 均按停止流程处理。本地 sim2sim 不连接机器人或 DDS，无需配置网卡。

## 自动仿真、录像、日志

```bash
./run_policy.sh --policy kungfubot_horse_stance_punch --mode sim
./run_policy.sh --policy kungfubot_horse_stance_punch --mode sim --render
```

`sim` 自动运行前置站立（默认 5 秒）、完整动作及后置站立/保持（默认 10 秒）。
`sim2sim` 等待人工交互，两者不同。`--render` 保存 25 fps MP4，自动模式不打开交互窗口；`--push` 使用原固定随机种子的三次扰动。`--render` / `--push` 只用于自动 `sim`。

每次生成独立 `results/…/`：`run.json`（配置和 SHA256）、`trace.npz`、`summary.json`；
具备相应衔接配置的会话另有 `session.json`；当前 7 个单动作无需该文件。录像为 `handoff.mp4`。`--output 新目录` 可指定位置，已有目录会拒绝覆盖。
仿真报告包含逐阶段高度/倾斜、动作完整性、最终站立/保持恢复指标。

## 首次连接机器人

1. 可靠吊装机器人，上电并完成初始化。按该机型官方流程进入零力矩/调试模式；原工程使用遥控器 **L2+R2** 进入调试。
2. 网线连接电脑与机器人，确认网卡、机器人实际 IP 和现场网段。
3. 准备网络及控制权，看到 `READY` 后执行只读检查，再启动控制。

```bash
ip -br link
ip -br addr
ROBOT_IFACE=enp7s0                # 换成实际有线网卡
ROBOT_IP=192.168.123.161          # 示例；必须换成现场确认的机器人地址

# 先查看计划，不连接 DDS、不改网络
./prepare_robot.sh --iface "$ROBOT_IFACE" --robot-ip "$ROBOT_IP" --dry-run

# 配置电脑地址，检查/释放 AI 控制，验证 lowstate 和 lowcmd 静默
./prepare_robot.sh --iface "$ROBOT_IFACE" --robot-ip "$ROBOT_IP" \
  --address 192.168.123.222/24

./run_policy.sh --policy kungfubot_horse_stance_punch --mode readonly --iface "$ROBOT_IFACE" --steps 100
./run_policy.sh --policy kungfubot_horse_stance_punch --mode real --iface "$ROBOT_IFACE"
```

电脑 IP 与机器人 IP 必须不同；现场不是上述网段时同步修改地址。
网络修改可能需要相应权限。准备工具只在明确识别的内置 AI 模式下调用 ReleaseMode，
`--check-only` 不改网络、不释放控制权。它不发送电机命令。

`readonly` 不创建电机命令发布器。`real` 等待 `TAKEOVER`，之前不发电机命令；随后使用与仿真相同的
`START`、`STOP` 和站立阶段的 `policy <完整名称>`。接管和退出时保留物理支撑；阻尼不能保持站立。
要求完整腰部 `mode_machine=5`，不要同时运行其他低层控制器。
默认状态过期 20 ms、主循环超时 40 ms、独立命令看门狗 100 ms、温度停止 65°C、退出阻尼 1 秒，见 `site.json`。
实机 domain 0 禁止自动启动/步数限制；测试用自动 DDS 仅允许 `--iface lo --domain 正整数`。

## 原 C++ 状态与内置官方站立

在终端输入 `stand` 进入原 C++ FixStand（约 2 秒插值），完成后 `rl Velocity` 进入速度策略，
`policy dance` 切到原舞蹈，`passive` 回到阻尼。`policy 名称` 在站立阶段可进入对应部署会话。
当前注册表的 7 个 KungfuBot 动作为单次播放，按前述站立阶段切换流程操作。
原 C++ 遥控操作保留：L2+上键 Stand、R2+A Velocity、R1+A 舞蹈；L2+B 在实机入口请求停止。

机器人**内置官方站立**是单独的 SDK `StandUp()`：先退出所有低层控制程序，按官方流程恢复内置运动服务，再执行：

```bash
./run_cpp.sh --mode official-stand --iface "$ROBOT_IFACE"
```

该命令只向已运行的内置服务发送 `StandUp()`，不创建低层命令发布器，也不负责恢复已经释放的内置控制权。原完整 C++ 控制器仍可单独使用：
`deploy/robots/g1/build/g1_ctrl --network "$ROBOT_IFACE"`，不要与新程序同时控制机器人。

完整功能与验证对应关系见 [Python 功能对照](docs/python_feature_parity.md)。

## 数据来源、引用与致谢

感谢 **KungfuBot / PBHC 团队**提供本项目所使用的动作数据和相关研究基础。使用这些动作数据、参考轨迹或相关成果开展研究时，请同时注明 KungfuBot 的贡献并引用原论文。

- [KungfuBot 官方项目页](https://kungfubot.github.io/)
- [官方代码与动作资源：TeleHuman/PBHC](https://github.com/TeleHuman/PBHC)
- [原论文：NeurIPS 2025](https://proceedings.neurips.cc/paper_files/paper/2025/hash/5a0e51901cff2b42d379ec7869603e91-Abstract-Conference.html)
- [完整来源说明与可复制的 BibTeX](docs/acknowledgements.md)
