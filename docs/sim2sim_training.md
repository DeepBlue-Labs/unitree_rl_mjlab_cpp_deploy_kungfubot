# G1 KungfuBot sim2sim 培训教程

适用项目：[`DeepBlue-Labs/unitree_rl_mjlab_cpp_deploy_kungfubot`](https://github.com/DeepBlue-Labs/unitree_rl_mjlab_cpp_deploy_kungfubot)  
面向学员：第一次接触本项目、能打开终端并输入命令的同学。  
本次目标：独立完成一次“启动仿真 → 站起来 → 做动作 → 回到站立 → 退出”，并能检查动作是否完整完成。

讲解时按本文逐节进行；学员动手时可使用[课堂操作速查卡](sim2sim_quickstart.md)。

## 1. 先理解：这次到底在做什么？

**sim2sim，就是把训练好的策略放到部署用的仿真环境中运行，检查机器人能不能完成动作。**

本项目的主线是：

```text
已经训练好的动作策略
        ↓
C++ 程序加载策略和动作文件
        ↓
控制 MuJoCo 中的 G1 虚拟机器人
        ↓
观察动作，检查结果，保存日志
```

这里使用的是 **G1 29 自由度**模型。7 个 KungfuBot 动作的训练框架是 **SuperMimic**，动作数据来自 **KungfuBot**；具体名称以本项目的 `--list` 输出为准。

本教程从“已有部署文件”开始。训练与导出工具不在这个部署目录中。运行时，C++ 程序通过 ONNX Runtime 做策略推理，通过 MuJoCo 做物理仿真。

**本次只在电脑里操作虚拟机器人，无需连接机器人、配置网卡或运行 `prepare_robot.sh`。**

### 两个运行模式，先分清楚

| 模式 | 谁决定什么时候开始？ | 画面和用途 |
|---|---|---|
| `sim2sim` | 学员输入命令 | 默认打开交互窗口，适合学习接管、开始、暂停和退出 |
| `sim` | 程序自动开始和结束 | 自动仿真，适合检查结果；加 `--render` 可保存视频 |

**课堂先使用 `sim2sim`，跑通之后再学习自动仿真和录像。**

## 2. 第一次上手：完整跑一遍马步冲拳

下面以“马步冲拳”为例。主流程只需要启动命令，以及 `TAKEOVER`、`START`、`STOP` 三个交互命令。

> **输入位置很重要：**启动前的命令输在 Linux 终端；启动后的 `TAKEOVER`、`START`、`STATUS`、`STOP` 输在**启动程序的同一个终端**，每条输入后按回车。不要另开终端直接执行这些交互命令，也不要把全部命令一次性粘贴进去。

### 第 1 步：进入项目，确认有这个动作

第一次使用时，先将仓库克隆到自己的用户目录：

```bash
git clone https://github.com/DeepBlue-Labs/unitree_rl_mjlab_cpp_deploy_kungfubot.git ~/unitree_rl_mjlab_cpp_deploy_kungfubot
```

已经克隆过就不要重复克隆。以下按项目位于 `~/unitree_rl_mjlab_cpp_deploy_kungfubot` 编写；如果你放在其他位置，进入实际目录即可。新克隆还未编译时，先完成第 7 节。

在终端执行：

```bash
cd ~/unitree_rl_mjlab_cpp_deploy_kungfubot
./run_policy.sh --list
```

**你应该看到：**已注册 7 个策略，列表中包含：

```text
kungfubot_horse_stance_punch | 动作：马步冲拳
```

再检查部署文件能否加载：

```bash
./run_policy.sh --policy kungfubot_horse_stance_punch --inspect
```

**成功标志：**输出 JSON，没有报错，其中 `frame_count` 为 `332`，`control_dt` 为 `0.02`。这表示动作共有 332 帧，每帧 0.02 秒，动作本体约 6.64 秒。`--inspect` 不播放动作，也不连接机器人。

如果提示 `Run ./build_cpp.sh first.`，先按第 7 节准备环境并编译，再回来继续。

### 第 2 步：打开交互仿真

```bash
./run_sim2sim.sh --policy kungfubot_horse_stance_punch
```

**你应该看到：**弹出 G1 仿真窗口，终端出现：

```text
Interactive C++ sim2sim ready. No robot/DDS connection.
```

状态中包含：

```text
WAIT_TAKEOVER
```

含义是“程序已准备好，正在等待接管”。**打开窗口并不等于开始做动作。**此时默认有虚拟吊架支撑机器人。

### 第 3 步：接管，让机器人站起来

点击回到启动程序的终端，输入下面这一行，然后按回车：

```text
TAKEOVER
```

**程序会做什么：**用默认约 4 秒，从当前姿态逐渐过渡到正常站姿。

等待终端出现：

```text
STAND_PRE: virtual hoist OFF; zero-command standing. Type START when ready.
```

含义是“动作前站立”：接管结束，虚拟吊架已释放，站立策略正在控制机器人。

**先看机器人是否站稳。课堂可以在出现 `STAND_PRE` 后再观察约 2 秒，然后继续。**这 2 秒是课堂观察时间，不是程序强制倒计时。不要在 `TAKEOVER` 尚未结束时输入 `START`；过早输入会被忽略，需要等到站立后重新输入。

### 第 4 步：播放一次动作

仍在同一个终端输入：

```text
START
```

接下来程序自动完成三个阶段，无需再输入命令：

| 终端状态中出现 | 通俗解释 | 马步冲拳的默认用时 |
|---|---|---|
| `PREPARE` | 从站姿过渡到动作起始姿态 | 约 0.4 秒 |
| `POLICY` | 策略控制机器人完成马步冲拳 | 约 6.64 秒 |
| `RETURN` | 从动作结束姿态过渡回站立控制 | 约 0.5 秒 |

**正常情况下，输入 `START` 后约 7.5 秒会进入 `STAND_POST`。**如果暂停或电脑运行较慢，实际等待时间会更长。

### 第 5 步：确认动作结束，并保持站立

等待终端出现：

```text
STAND_POST: zero-command velocity standing
```

含义是“动作后站立”：本次动作已播完，站立策略继续控制机器人。

观察至少 1～2 秒，确认没有跌倒或持续失稳，然后输入：

```text
STATUS
```

**成功标志：**状态包含 `STAND_POST`，动作进度显示 `332/332`。

此时程序仍在运行。**再次输入 `START` 不会重播这次动作。**初次练习时，先退出，再重新启动一轮，最容易掌握。

### 第 6 步：正常退出

输入：

```text
STOP
```

程序进入 `DAMP`（阻尼停止），默认约 1 秒后退出，终端重新出现 Linux 命令提示符。最后会打印本次日志目录：

```text
Logs: "……/results/kungfubot_horse_stance_punch_sim2sim_……"
```

至此，一次完整练习结束。`STOP` 结束的是这次运行；要再次练习，重新执行第 2 步即可。

## 3. 用一张流程图记住操作顺序

```text
启动程序
   ↓
WAIT_TAKEOVER       等待学员操作
   │ 输入 TAKEOVER
   ↓
TAKEOVER            约 4 秒，逐渐调整到站姿
   ↓ 自动进入
STAND_PRE           动作前站立；学员观察是否站稳
   │ 输入 START
   ↓
PREPARE             过渡到动作起始姿态
   ↓ 自动进入
POLICY              播放完整动作
   ↓ 自动进入
RETURN              过渡回站立控制
   ↓ 自动进入
STAND_POST          动作后站立；检查动作完成情况
   │ 输入 STOP
   ↓
DAMP                阻尼停止，约 1 秒
   ↓
程序退出，保存日志
```

记忆口诀：**先接管，等站稳，再开始；做完回站立，最后停止。**

正常练习从 `STAND_POST` 退出；需要提前结束时，也可以在其他阶段输入 `STOP`。

## 4. 换一个动作怎么做？

先 `STOP` 退出当前运行，然后替换启动命令中 `--policy` 后面的名称即可。其他步骤相同。

例如，改成勾拳：

```bash
./run_sim2sim.sh --policy kungfubot_hooks_punch
```

随后依次执行：输入 `TAKEOVER` → 等待 `STAND_PRE` 并观察站稳 → 输入 `START` → 等待 `STAND_POST` → 输入 `STOP`。

当前 7 个动作如下。表中的时间只计算动作本体，不包含接管、准备、返回和人工等待。

| 动作 | `--policy` 后填写的完整名称 | 动作帧数 | 动作本体约时 |
|---|---|---:|---:|
| 李小龙姿势 | `kungfubot_bruce_lee_pose` | 549 | 10.98 秒 |
| 查尔斯顿舞 | `kungfubot_charleston_dance` | 1016 | 20.32 秒 |
| 勾拳 | `kungfubot_hooks_punch` | 291 | 5.82 秒 |
| 马步 | `kungfubot_horse_stance_pose` | 349 | 6.98 秒 |
| 马步冲拳 | `kungfubot_horse_stance_punch` | 332 | 6.64 秒 |
| 回旋踢 | `kungfubot_roundhouse_kick` | 262 | 5.24 秒 |
| 侧踢 | `kungfubot_side_kick` | 297 | 5.94 秒 |

这些是当前文件中的帧数；更新部署文件后，用 `--inspect` 重新确认。**能加载一个动作，不代表它在每次仿真中都能稳定完成**，需要逐次查看动作与结果。

本教程统一使用完整名称。`--list` 也会列出可用别名。旧版材料中的 `entry_star`、`straight`、`hook` 不在本目录当前的 7 个动作列表中；本目录的勾拳请使用 `kungfubot_hooks_punch`。

熟练后，可以在 `STAND_POST` 输入 `policy kungfubot_hooks_punch`，在同一个进程中切到勾拳。**这条命令会直接开始新动作的准备阶段，不需要再输入 `START`**；初学阶段仍按“退出后重新启动”练习。

## 5. 暂停、看画面与快捷键

### 最容易分清的两种操作方式

| 想做什么 | 在启动终端输入，并按回车 | 点击仿真窗口后按键 |
|---|---|---|
| 接管 | `TAKEOVER` | `T` |
| 开始 | `START` | `S` 或 Enter |
| 暂停 / 恢复 | `PAUSE`（每次切换） | Space（每次切换） |
| 明确恢复 | `RESUME` | 再按一次 Space |
| 查看进度 | `STATUS` | 回到终端输入 |
| 查看帮助 | `HELP` | 回到终端输入 |
| 停止并退出 | `STOP` | `Q` 或 Esc |

**按键只在对应窗口获得焦点时生效。**例如，在终端里输入 `T` 并回车，不等于窗口里的 `T` 快捷键。

暂停会同时停止物理仿真和策略帧推进；暂停期间仍可调整视角。开始或切换动作前要先恢复。`STOP` 与暂停不同：它会结束本次运行。

### 观察画面

| 鼠标操作 | 效果 |
|---|---|
| 按住左键拖动 | 旋转视角 |
| 按住右键拖动 | 平移视角 |
| 滚轮 | 放大 / 缩小 |

第一轮练习只调整视角即可。进阶演示可双击选中身体部位，再用 Ctrl + 右键拖动施力，或 Ctrl + 左键拖动施加扭转；松开后清除扰动。暂停时不施力。

窗口中 `State` 表示当前控制状态，`Selected` 表示界面选中的策略。初学时重点看 `State`，保持默认选择，不使用左右键切换策略。

## 6. 如何确认“这次真的成功了”？

### 先看过程，再看日志

课堂练习需要同时满足：

1. 接管后进入 `STAND_PRE`，机器人可以站立。
2. 开始后经过 `PREPARE → POLICY → RETURN → STAND_POST`，中途没有故障退出。
3. `STATUS` 显示完整帧数；马步冲拳是 `332/332`。
4. 动作后可以稳定站立，再用 `STOP` 退出。

只看到窗口、动作动了一下、或者程序正常退出，都不足以说明完整动作已经成功。

### 日志保存在哪里？

默认每次运行新建一个目录：

```text
results/动作完整名称_sim2sim_时间戳/
├── run.json       本次运行参数、文件校验值
├── trace.npz      每个控制周期的状态、目标和动作数据
└── summary.json   本次运行结果摘要
```

退出后，按终端最后打印的 `Logs` 路径找到该目录，打开 `summary.json`。也可在终端用 `cat` 加该文件路径查看。

| 字段 | 正常完成一次马步冲拳时重点检查 |
|---|---|
| `error` | `false`，没有记录故障 |
| `motion_complete` | `true`，动作帧播放完整 |
| `policy_ticks` / `expected_policy_ticks` | 两者相等，示例中均为 `332` |
| `standing_after_motion` | `true`，进入过动作后站立 |
| `standing_recovered_last_1s` | `true`，最后采样的 1 秒动作后站立满足内置高度/倾斜判据 |
| `robot_commands_published` | `0`，本次本地仿真未发布实机电机命令 |

**正常 `STOP` 结束时，`stop_reason` 可以是 `STOP requested`，这本身不是故障。**如果动作没做完就 `STOP`，即使 `error` 是 `false`，也不能算动作完成。动作后站立不足 1 秒时，恢复指标可能为 `false`，所以课堂要留出观察时间。

这些结果用于本地仿真检查，不等于实机验收。

### 进阶：自动跑完并保存视频

先结束当前交互运行，再在终端执行：

```bash
./run_policy.sh --policy kungfubot_horse_stance_punch --mode sim --render
```

这次**不需要输入 `TAKEOVER` 或 `START`**。程序从预设站姿开始，默认先站立 5 秒，播放动作，再站立 10 秒，完成后自动退出。自动模式不打开交互窗口；视频写入本次日志目录下的 `handoff.mp4`。

`--render` 用于自动 `sim`，不能直接加在交互 `sim2sim` 命令后面。自动模式和交互模式的初始条件不同，应分别检查结果。

## 7. 首次准备环境与重新编译

同一台电脑上已经能执行第 2 节命令时，可以跳过本节。以下按项目原有的 Ubuntu 22.04 / x86_64 环境准备。

### 安装依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libyaml-cpp-dev \
  libboost-program-options-dev libfmt-dev libspdlog-dev zlib1g-dev \
  libssl-dev libjsoncpp-dev libglfw3-dev libgl1-mesa-dev libegl1-mesa-dev \
  ffmpeg iproute2 python3 python3-numpy
```

C++ 部署运行不需要 Conda。Python 用于部分检查工具；项目已附带 ONNX Runtime 和 MuJoCo 库。当前附带 MuJoCo 是 x86_64 版本，其他架构需要准备匹配的库。

### 从 GitHub 克隆时跳过本小节；复制过旧目录时检查缓存

仓库不包含 `build`。只有从其他电脑或路径复制了已有编译目录时，才可能遇到 `CMakeCache.txt` 中的源码路径不匹配。此时，先执行一次下面的命令，把旧 `build` 改名保留，再重新编译：

```bash
cd ~/unitree_rl_mjlab_cpp_deploy_kungfubot
mv build "build_backup_$(date +%Y%m%d_%H%M%S)"
```

只在旧缓存存在且路径不一致时执行这一步。新拷贝没有 `build` 目录，或已在当前路径重新配置过时，跳过改名。

### 编译本教程用到的主程序

```bash
cd ~/unitree_rl_mjlab_cpp_deploy_kungfubot
cmake -S deploy/robots/g1 -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target g1_deploy -j2
./run_policy.sh --list
./run_policy.sh --policy kungfubot_horse_stance_punch --inspect
```

编译后主程序为 `build/g1_deploy`。回到第 2 节开始练习。

项目也提供 `./build_cpp.sh`，它会编译全部目标并执行整套历史测试。历史测试中有针对原 28 动作工程的场景；检查这些测试时，应核对当前精简注册表是否提供它们要求的动作。本教程的最小编译命令只构建当前培训使用的主程序。

## 8. 常见问题：看到什么，就怎么处理

| 现象 | 常见原因 | 操作方法 |
|---|---|---|
| 提示 `Run ./build_cpp.sh first.` | 找不到可执行的 `build/g1_deploy` | 按第 7 节编译主程序 |
| CMake 提示源码目录与缓存不匹配 | `build` 从其他路径复制过来 | 按第 7 节保留旧目录，再重新配置 |
| `unknown policy` | 动作名写错，或用了旧项目名称 | 执行 `--list`，复制当前动作完整名称 |
| 打开窗口后机器人不做动作 | 还在等待接管或开始 | 先 `TAKEOVER`，等 `STAND_PRE`，再 `START` |
| `START ignored in TAKEOVER` | 接管没有结束 | 等到 `STAND_PRE`，观察站稳后重新输入 `START` |
| 做完后再输入 `START` 没反应 | 当前动作设计为单次播放 | `STOP` 后重新启动；熟练后用 `policy 完整名称` |
| 输入 `PAUSE` 后开始不了 | 仿真仍暂停 | 先输入 `RESUME`，再执行下一步 |
| 窗口快捷键没反应 | 焦点仍在终端 | 点击仿真窗口后按键，或直接使用终端交互命令 |
| 终端输入 `TAKEOVER` 显示“找不到命令” | 程序已退出，或输入到了另一个终端 | 重新启动仿真，在运行中的原终端输入 |
| GLFW 初始化失败或无法创建窗口 | 没有可用图形显示环境，或相关依赖缺失 | 有桌面时检查图形环境与 GLFW；服务器可加 `--headless`，按相同终端流程操作，但没有画面 |
| `entry not ready`、`reference guard` 或高度/倾角保护报错 | 起始姿态或动作过程未满足程序检查条件 | 保存日志，`STOP` 退出，重新从接管流程复现；比较动作名、部署文件和发生故障的阶段 |
| `--output` 指定目录报已存在 | 程序拒绝覆盖旧运行结果 | 不指定 `--output`，或换一个全新的目录名 |

发生故障时，先记录**动作名称、所处阶段、完整报错、日志路径**，再分析原因。不要只凭窗口里机器人还直立就判断通过，也不要通过放宽保护阈值来完成课堂验收。

## 9. 给讲师：一堂约 40 分钟的课怎么安排？

| 时间 | 讲师演示 / 学员任务 | 当场检查 |
|---|---|---|
| 5 分钟 | 讲清策略、虚拟机器人、`sim2sim` 和 `sim` | 学员能说出“当前运行无需连接真机” |
| 10 分钟 | 按第 2 节演示一次完整马步冲拳 | 学员能指出 `STAND_PRE` 与 `STAND_POST` 的区别 |
| 10 分钟 | 学员独立跑一次，再换成勾拳 | 动作进度完整，动作后回到站立 |
| 5 分钟 | 练习暂停、恢复和视角调整 | 能解释暂停与停止的区别 |
| 5 分钟 | 找到日志，读取结果字段 | 能用帧数与恢复指标说明是否完成 |
| 5 分钟 | 演示自动运行和录像 | 找到 `handoff.mp4` |

课堂结束前，让每位学员独立完成下面四项：

- [ ] 从 `--list` 找到动作，并正确启动。
- [ ] 按“接管 → 等站稳 → 开始 → 等动作后站立 → 停止”完成一次运行。
- [ ] 正确暂停、恢复，能调整观察视角。
- [ ] 找到本次 `summary.json`，说清动作是否完整、站立是否恢复。

## 附录：需要讲原理时，再展开这部分

### A. 三类动作文件各做什么？

以 `policies/kungfubot_horse_stance_punch/` 为例：

| 文件 | 可以怎样向学员解释 |
|---|---|
| `policy.onnx` | 学好的“大脑”：根据当前状态与动作目标，计算下一步控制输出 |
| `deploy_config.json` | 控制参数：关节顺序、输出缩放、默认角度、Kp/Kd 和限制等 |
| `clip.npz` | 动作参考：各帧目标、参考关节姿态和身体姿态等 |

这三个文件需要成套使用。`policies/registry.json` 是动作目录，负责把动作名指向对应文件，并设置接管、准备、返回时间。

`site.json` 指向 G1 MuJoCo 模型和站立策略。站立由 `policies/standing/` 提供；动作执行前后都要依靠它控制站立。

### B. 为什么叫“策略控制”，而不只是播放动画？

部署程序每 0.02 秒做一轮控制，也就是 **50 Hz**：

```text
读取 MuJoCo 当前机器人状态
          ↓
整理观测历史，取当前动作参考目标
          ↓
ONNX 策略计算 29 个关节的动作输出
          ↓
结合默认角度、输出缩放和力矩约束，得到关节目标
          ↓
MuJoCo 计算关节运动、重力和接触，产生下一轮状态
          └──────────────────→ 回到第一步
```

因此，机器人偏离目标后，下一轮输入会变化，策略也会重新计算输出。这是一个持续反馈的过程。

进阶讲解可补充：当前 Reference 策略输入为 `obs_hist [1,10,99]` 与 `goal [1,730]`，输出为 `action [1,29]`。学员完成基本操作不需要记住这些维度。

### C. 从启动脚本看到哪里？

```text
run_sim2sim.sh
  → run_cpp.sh，传入 --mode sim2sim 和当前项目 --root
  → build/g1_deploy
  → cpp_main.cpp：加载文件、创建仿真、读取命令、保存日志
  → Controller.cpp：处理控制请求
  → Session.cpp：推进接管、站立、准备、动作、返回、停止阶段
  → MotionPolicy.cpp / LegacyPolicy.cpp：动作与站立推理
  → Physics.cpp：推进 MuJoCo 物理仿真
```

相关 C++ 源码位于 `deploy/robots/g1/`。要修改培训里的操作流程，先看 `Session.cpp`；要理解策略输入输出，再看 `MotionPolicy.cpp`。

### D. 本教程核验记录

2026-11-25，使用当前目录已有可执行文件核验：

- `--list` 确认当前 7 个 KungfuBot 动作；逐个 `--inspect` 成功，并核对上表帧数。
- 用 `sim2sim --headless --fast --script` 驱动真实 C++ 主程序，完成马步冲拳的接管、站立、动作、返回与停止。命令分别在第 0、300、1200 个控制周期发送 `TAKEOVER`、`START`、`STOP`。
- 该次结果：`error=false`、`motion_complete=true`、`policy_ticks=expected_policy_ticks=332`、`standing_after_motion=true`、`standing_recovered_last_1s=true`、`robot_commands_published=0`。
- 另行核验 `STATUS` 显示 `332/332`、重复 `START` 不重播；在动作后站立时用 `policy kungfubot_hooks_punch` 切换，自动播放完整 291 帧并恢复站立。
- 自动 `sim --render` 核验成功，`complete=true`、`screen_passed=true`，生成非空 `handoff.mp4`。
- 按教程使用的 CMake 配置与主程序构建方式，在独立临时编译目录重新编译当前源码成功；新编译程序再次完成 332 帧马步冲拳并恢复站立。原项目已有 `build` 目录保留未改动。

上述记录验证马步冲拳主线和切换勾拳的流程；其余 5 个动作仅完成文件检查，不代表已逐个完成动态仿真。图形窗口按键和鼠标说明依据当前代码核对，未在此次核验中逐项操作图形界面。
