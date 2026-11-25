# Python 部署功能对照

迁移基准：未裁剪工程 `/home/pfy/unitree_rl_mjlab_supermimic` 的公开统一部署入口
`run_policy.sh` / `unified_deploy`，以及它调用的 Hybrid、Reference、UFO、standing、plant 和 DDS 接口。
原 C++ 基准独立保留；没有把训练器、数据导出工具或其他机器人重新放进部署工程。

| 原功能 | C++ 位置 | 验证 |
|---|---|---|
| 28 动作、别名、中文来源分组、默认与逐动作参数、SHA256 | Registry / policies/registry.json | registry_parity.py |
| Hybrid / Reference / UFO 观测、历史、朝向、目标映射 | MotionPolicy | motion_parity.py、policy_transition_parity.py |
| 原 Python zero-command standing | LegacyPolicy Balance / policies/standing | policy_transition_parity.py |
| 原 C++ Velocity / Mimic / Stand / Passive | LegacyPolicy / Controller / 原 g1_ctrl | legacy_parity.py、hierarchy、simulation_switching |
| 4 秒测量起点接管、STAND_PRE、准备、3 帧入场混合、完整动作、返回、STAND_POST | Session | session_parity.py 全策略回放 |
| ReadyHold 各候选、末帧静态目标、NEXT、站立后重入 | Session | 原 Python ReadyHold 全路线回放 |
| entry → home 保持、7 种格斗动作、exit → 站立 → entry | Session | 真实 actor 回放 + MuJoCo full_session_integration |
| 连续稳定门槛、请求替换/超时、0.5 秒双 actor 五次多项式混合、实际目标反馈、末帧稳定等待 | Session / SwitchGate | phase/frame/gate/target/observation 逐 tick 对照 |
| 各阶段 STOP、暂停、EOF、Ctrl+C、关窗、无自动重播 | cpp_main / Session / Viewer | 12 阶段 STOP 回放 + 实际主程序测试 |
| 自动 sim、交互 sim2sim、readonly、real | cpp_main | 主程序集成与本地 DDS 测试 |
| 吊架、上身重力补偿、足底运动学估计、参考相对高度/倾斜筛查 | Physics / RunLog | physics_parity.py、仿真指标 |
| 固定种子扰动、MP4 | Physics / VideoRecorder | EGL 离屏录制及实际自动仿真 |
| 视角、选体、Ctrl 拖拽扰动、暂停不改物理 | Viewer | viewer_interaction |
| site / registry 覆盖、inspect、独立输出目录、run/session/summary JSON、Unicode NPZ | Registry / RunLog / cpp_main | full_session_integration |
| 20 ms 状态时效、40 ms 主循环期限、100 ms 发布看门狗、65℃门槛、1 秒阻尼 | Robot / cpp_main | robot_packets_offline + loopback_integration |
| 只读不创建 publisher；接管前 STOP 不发布；接管后停止阻尼只执行一次 | Robot / cpp_main | 独立 loopback LowCmd 抓取 |
| 网卡选择、地址计划、受限 ReleaseMode、静默检查与 READY、check-only/dry-run | g1_prepare | prepare_offline、dry-run |

`policies/standing` 使用原 Python 配置（waist yaw action scale 0.55）；原 C++ Velocity 继续使用
原始配置（0.35）。两者 ONNX 完全相同，通过相对链接共享，不复制权重。
切换所需 actor、运动学模型及 gate 在连接机器人前建立，控制周期内不读取模型或重新建会话。

数值对照结果见 `tests/cpp_migration/policy_transition_validation.json` 与
`session_parity_report.json`。Session 对照采用真实原 Python actor 和运动学计算，不以 mock 状态机代替。
回放覆盖 45 个场景、38,111 tick；相位、参考帧、完成列表、请求与 gate 状态逐步一致。
浮点推理允许数值容差，报告记录实际误差。

常规检查：`./build_cpp.sh`。需要复跑 Python 数值 oracle 时，使用装有 NumPy、PyYAML、ONNX Runtime、
MuJoCo 的验证环境运行对应 `*_parity.py --help`；部署可执行文件不调用这些 Python 工具。

验证范围是离线真实输入回放、MuJoCo、隔离的 `lo` 正 domain DDS 和内存故障注入。
未向实机发送控制命令，实机性能、现场网络时序及动作安全仍须现场验收。
