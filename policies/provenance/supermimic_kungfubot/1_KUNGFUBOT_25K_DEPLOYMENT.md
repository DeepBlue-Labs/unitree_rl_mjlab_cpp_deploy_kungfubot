# KungfuBot 25k SuperMimic 部署记录

日期：2026-11-10

本记录延续 `SUPERMIMIC_REFERENCE.md` 的原版 bridge 部署流程。KungfuBot 只提供动作数据；策略由 SuperMimic 在远端 4096 环境、每条动作 25,000 次迭代训练。本次部署不修改原有七个 SuperMimic 策略。

## 隔离目录

新策略安装到：

```text
/home/pfy/unitree_rl_mjlab_supermimic/supermimic_reference/deploy_artifacts/kungfubot_25k/
```

每个策略目录包含：

- `policy.onnx`
- `policy.ts.pt`
- `deploy_config.json`
- 原始 `deployment.json`
- 原始 `reference.npz`
- 原始 `manifest.json` / `result.json` / `README.txt`
- 策略级 `SHA256SUMS`

转换后的原版 runner 参考轨迹位于：

```text
deploy_artifacts/kungfubot_25k/clips/<policy_id>.npz
```

命名空间根目录还有 `manifest.json`、`index.json`、`README.md` 和总的 `SHA256SUMS`。

安装前对参考工程七个旧策略、旧 clips 和旧 mjlab 策略配置共 56 个文件生成 SHA256；安装完成后全部校验通过。安装器也会拒绝覆盖任何同名策略或 clip。

## 已安装策略

| 动作 | 最佳迭代 | 归一化回报 | policy id |
|---|---:|---:|---|
| Bruce_Lee_pose | 20,000 | 0.961880 | `bruce_lee_pose_iter20000` |
| Charleston_dance | 17,500 | 0.974855 | `charleston_dance_iter17500` |
| Hooks_punch | 20,000 | 0.973023 | `hooks_punch_iter20000` |
| Horse-stance_pose | 22,500 | 0.989798 | `horse_stance_pose_iter22500` |
| Horse-stance_punch | 25,000 | 0.976211 | `horse_stance_punch_iter25000` |

远端当时仍剩 `Roundhouse_kick` 训练中和 `Side_kick` 排队。这两条完成后将对应 bundle 放入同一个 bundle 目录，再运行安装器即可增量加入，不需要修改已有目录。

## 数据与策略合同

- 动作数据来源：KungfuBot 数据集。
- 训练框架：SuperMimic。
- 机器人：G1 EDU 29 DoF。
- ONNX 输入：`obs_hist [1,10,99]`、`goal [1,730]`。
- ONNX 输出：`action [1,29]`。
- 控制频率：50 Hz。
- 关节顺序：Unitree G1 motor 0..28。
- `deploy_config.json` 直接使用 bundle 内的 `default_q`、Kp/Kd、action scale、位置/速度/力矩限制和 50 Hz 控制周期。
- 原版 runner 直接消费 730 维预计算目标，不在部署侧再次归一化。

## 验证结果

### 接口与数据

- 新增安装与数据合同测试：6 项通过。
- 原 SuperMimic-vibed 适配测试：15 项通过。
- 5 个 ONNX 均成功加载，输入输出尺寸和 action 有限性检查通过。
- 转换后的 clip goal 与从 `policy_reference` 重新计算的 730 维 goal 在 `2e-6` 绝对误差内一致。
- 安装器重复运行无覆盖，输出为 `already installed (identical)`。

### Unitree MuJoCo G2 地面 nominal

条件：命令延迟 10 ms、训练 Kp/Kd、无附加质量、摩擦系数 1.0。

| 动作 | 结果 | 完成步数 | 最大倾角跟踪误差 |
|---|---|---:|---:|
| Bruce_Lee_pose | 第 190 步跌倒 | 190/548 | 6.11° |
| Charleston_dance | 完成 | 1015/1015 | 4.24° |
| Hooks_punch | 完成 | 290/290 | 5.10° |
| Horse-stance_pose | 完成 | 348/348 | 18.27° |
| Horse-stance_punch | 完成 | 331/331 | 2.30° |

因此地面 nominal 为 4/5 完成，Bruce_Lee_pose 未通过。

### G3 悬挂 air 过渡（参考训练 Kp/Kd）

该测试包含默认站姿到动作首帧的插值、悬挂策略运行和结束过渡。任何 `DeployCore` abort 在真实 bridge 中都会触发保护停机，所以“机器人仍直立”不算通过。

| 动作 | 完整执行 | 安全中止 | 首次中止 |
|---|---|---|---|
| Bruce_Lee_pose | 是 | 75 步 | 第 153 步，倾角误差 21° |
| Charleston_dance | 是 | 101 步 | 第 383 步，倾角误差 21° |
| Hooks_punch | 是 | 0 | 无 |
| Horse-stance_pose | 是 | 0 | 无 |
| Horse-stance_punch | 是 | 5 步 | 第 151 步，倾角误差 21° |

按原版 20° 安全阈值，air 过渡只有 `Hooks_punch` 和 `Horse-stance_pose` 通过。没有放宽保护阈值来消除这些失败。

逐次结果位于：

```text
supermimic_reference/deploy_artifacts/kungfubot_25k/reports/
```

### 本地 DDS bridge 全链路

对 `hooks_punch_iter20000` 使用原版 `g1_runner_bridge.py`、真实 `g1_bridge` 和 Unitree MuJoCo 在 `lo/domain 1` 跑了两轮，分别是 `air` 和 `ground` 安全配置。两轮都确认：

- ONNX 和 clip 成功加载。
- 50 Hz 控制线程正常运行，`lowcmd` 约为 512 Hz。
- bridge 发布失败、看门狗、通信丢失均为 0。
- 策略计算耗时 p99 小于 7 ms。

但两轮都在进入 `STANDBY_PRE` 后约 2 秒触发原版倾角保护并 DAMP；其中 air 首次故障是 `tilt error 22 deg`。因此当前只能确认“部署文件及 bridge 链路可启动”，不能确认状态机能进入并完成动作。

日志：

```text
supermimic_reference/deploy_logs/sim_e2e/kungfubot_25k_hooks_air/
supermimic_reference/deploy_logs/sim_e2e/kungfubot_25k_hooks_ground/
```

## 启动命令

```bash
cd /home/pfy/unitree_rl_mjlab_supermimic/supermimic_reference
source deploy_env/env.sh

python scripts/deploy/bridge/g1_runner_bridge.py \
  --iface <机器人网卡> --domain 0 --stage air \
  --artifacts deploy_artifacts/kungfubot_25k/<policy_id> \
  --clip deploy_artifacts/kungfubot_25k/clips/<policy_id>.npz \
  --log-dir deploy_logs/real
```

每个策略只能搭配自己的同名 clip，不能把不同权重当作同一个 actor 的多个动作。

## 安全边界

- 只能先在吊架悬挂、`--stage air` 条件下做首次尝试。
- 遥控器急停 `L2+B` 必须由旁站人员随时准备触发。
- 这些策略尚未在真机验证。
- Bruce_Lee_pose 地面已经跌倒；Charleston_dance、Bruce_Lee_pose 和 Horse-stance_punch 的 air 测试会触发原版倾角保护。
- 推荐优先验证 `Hooks_punch` 和 `Horse-stance_pose`；其他策略应先修正动作跟踪或训练分布，再重新导出。

## 增量安装

```bash
cd /home/pfy/unitree_rl_mjlab_supermimic/supermimic_reference
source deploy_env/env.sh

python scripts/deploy/install_kungfubot_bundle.py \
  --bundle-dir /home/pfy/Documents/Codex/2026-11-09/new-chat/outputs/kungfubot_25k/policies \
  --output deploy_artifacts/kungfubot_25k

python scripts/deploy/smoke_test_kungfubot_25k.py --mode g2
python scripts/deploy/smoke_test_kungfubot_25k.py --mode g3-air
```
