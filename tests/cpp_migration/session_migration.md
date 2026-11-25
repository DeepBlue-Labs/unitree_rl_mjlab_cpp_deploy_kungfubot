# 完整部署会话的 C++ 迁移

`Session` 直接迁移原工程的 `unified_deploy/sequence.py`、`ready_hold.py`、
`fighting.py` 和 `switch_gate.py`。它是顶层 RL 状态内部的会话，拥有策略帧推进；
调用者不能再次调用其活动策略的 `advance()`。

## 保留的流程

- Interactive：WAIT_TAKEOVER → TAKEOVER（默认 4 秒原 cubic 插值、Standing 增益）
  → STAND_PRE，等待 START。STOP 从任意阶段进入 DAMP，默认 1 秒后 finished。
  `startStanding()` 提供原自动仿真的 STAND_PRE 入口，不运行 TAKEOVER 或动作 actor。
- One-shot：STAND_PRE → PREPARE → POLICY → RETURN → STAND_POST。
  STAND_PRE/POST 与 RETURN 都调用原 Python 独立 Standing actor；RETURN 将实际混合
  目标反馈到 standing.last_action。START 只在 STAND_PRE 生效，不重播完成动作。
- ReadyHold：严格使用 registry 的 hold_policies 和 next_from_hold。entry/home 保持
  最后一帧；NEXT exit 回 standing；STAND_POST 可以进入 entry/guard/home/exit。
  guard 保留单次动作语义，不因通用 PolicyInfo.hold_final 标志被错误冻结。
- Fighting：entry 完成后切 home actor，双 actor 五次多项式权重融合并获得稳定 HOLD；
  bare 动作名和 NEXT 都可请求已验证动作。保持 ready actor，等待连续 25 个合格测量；
  请求可替换，4 秒仍不合格取消。动作切入在 frame 0 冻结 26 次推理（原 0..25 混合索引），
  两个 actor 每步推理，并向双方动作历史反馈实际混合目标。动作执行完毕进入 SETTLE，
  冻结最后目标且 future dq 清零；连续 5 个合格测量后回 home，再经过 acquire gate 进入 HOLD。
  exit 单独走 RETURN → STAND_POST，之后 entry 可重新进入；START 不触发重播。
- acquire/settle 超时保留为异常，交给顶层控制器停止并进入 DAMP。

Gate 使用与原 Python 相同的 MuJoCo 关节 FK、脚体 Jacobian、角速度和 IMU gravity。
输入不包含 root 平移或线速度；脚相对高度和相对速度仍是运动学代理，不是接触力估计。
准备阶段继续使用原上身重力补偿，不修改 actor、增益、权重或动作映射。

## 接口与时序

`SessionOutput.phase` 是本 tick 真正执行的控制阶段；`Session::phase()` 是本 tick
完成后的状态。动作末帧输出仍为 POLICY，但下一状态可能已经是 RETURN、READY_BLEND
或 SETTLE。原 Python 的计数和日志也采用这种区分。

`SessionOutput.support` 只在 WAIT_TAKEOVER/TAKEOVER/DAMP 为 true。暂停应同时停止
Session 和物理推进。停止时仍应调用 step，直到 `finished()`，不能仅看状态为 DAMP 就退出。

Session 依赖预加载的 Catalog；构造期间验证候选策略的 joint order、观测宽度、50 Hz、
proprio 类型以及 kp/kd/effort 一致性，并创建全部 gate/gravity MuJoCo 对象。控制周期内
不加载文件或创建 ONNX 会话。每个原策略的 ready_hold/fighting 元数据来自完整 registry。

## 原 Python 数值对照

```bash
python tests/cpp_migration/session_parity.py \
  --probe build/session_probe \
  --original /home/pfy/unitree_rl_mjlab_supermimic \
  --report tests/cpp_migration/session_parity_report.json
```

测试直接导入未修改的原 Python 部署模块，使用原 ONNX actor 和原 MuJoCo gate，
生成确定性实测状态输入与预期输出，再由 C++ 逐 tick 重放。比对状态、实际控制阶段、
动作帧、策略累计帧数、pending 请求、HOLD 计数、完成动作列表、目标/增益、观测和 gate 数值。
测试没有 mock actor、替代 gate、缩短参考动作或更改模型。传感器轨迹使用参考姿势而非闭环
物理轨迹，因此本测试证明会话和数值语义一致；机器人动态稳定性应由独立仿真/实机验证。

`--quick` 运行两个 one-shot、ReadyHold 全路线和 Fighting 七动作完整往返。
完整测试还覆盖全部 27 个非 entry 策略的完整 one-shot、自动 STAND_PRE、各阶段 STOP、
请求替换与超时、acquire 和 settle 超时。各模型主体推理保持 C++，Python 仅用于离线对照。

## 本地 DDS 实际集成

```bash
python tests/cpp_migration/loopback_integration.py \
  --root "$PWD" --binary build/g1_deploy --fixture build/loopback_g1 \
  --report tests/cpp_migration/loopback_report.json
```

fixture 的网卡硬编码为 `lo`，拒绝 domain 0，只发布模拟 LowState 并独立采集 LowCmd。
测试逐个使用正数隔离 domain，验证：readonly 默认 100 帧且不创建 publisher、Reference
观测日志包含 730 维 goal、WAIT_TAKEOVER 中 STOP 不发命令、接管中 STOP 只有 50 个
DAMP tick 和约 1 秒阻尼消息，以及自动 DDS 仿真的 steps 不包含 200 个 TAKEOVER tick。
测试没有连接实体机器人；传感器按命令直接跟随，因此它验证 DDS/命令生命周期，不是物理稳定性。
