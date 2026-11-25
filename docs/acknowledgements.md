# 动作数据来源、引用与致谢

## KungfuBot 动作数据

本项目的 7 个 KungfuBot 动作使用了 **KungfuBot / PBHC 项目提供的动作数据**：李小龙姿势、查尔斯顿舞、勾拳、马步、马步冲拳、回旋踢和侧踢。

感谢 Weiji Xie、Jinrui Han、Jiakun Zheng、Huanyu Li、Xinzhe Liu、Jiyuan Shi、Weinan Zhang、Chenjia Bai、Xuelong Li 及 KungfuBot 团队在动作数据、运动处理与人形机器人控制研究方面的贡献。

本项目的数据与策略来源关系为：

```text
KungfuBot 动作数据
        ↓
SuperMimic 策略训练与导出
        ↓
本仓库的 C++ 部署、MuJoCo sim2sim 与培训材料
```

原始动作数据的贡献归 KungfuBot 原作者团队。当前部署文件的训练与导出记录见[KungfuBot 25k SuperMimic 部署记录](../policies/provenance/supermimic_kungfubot/1_KUNGFUBOT_25K_DEPLOYMENT.md)。

## 官方来源

- 项目页：[KungfuBot](https://kungfubot.github.io/)。
- 官方仓库：[TeleHuman/PBHC](https://github.com/TeleHuman/PBHC)。
- 上游动作来源与处理说明：[motion_source](https://github.com/TeleHuman/PBHC/tree/main/motion_source)。
- 论文：[KungfuBot: Physics-Based Humanoid Whole-Body Control for Learning Highly-Dynamic Skills](https://proceedings.neurips.cc/paper_files/paper/2025/hash/5a0e51901cff2b42d379ec7869603e91-Abstract-Conference.html)，NeurIPS 2025。

以上官方链接与引用条目于 2026-11-25 核对。

## 论文引用

使用本项目中的 KungfuBot 动作数据、参考轨迹或相关成果开展研究时，请注明数据来源并引用原论文。以下 BibTeX 采用 [KungfuBot 官方项目页](https://kungfubot.github.io/)提供的会议论文条目：

```bibtex
@inproceedings{xie2025kungfubot,
  title={KungfuBot: Physics-Based Humanoid Whole-Body Control for Learning Highly-Dynamic Skills},
  author={Weiji Xie and Jinrui Han and Jiakun Zheng and Huanyu Li and Xinzhe Liu and Jiyuan Shi and Weinan Zhang and Chenjia Bai and Xuelong Li},
  booktitle={The Thirty-ninth Annual Conference on Neural Information Processing Systems},
  year={2025},
  url={https://openreview.net/forum?id=LCPoXt0pzm}
}
```

## 署名与许可说明

请保留原作者署名，并遵循对应材料发布时适用的许可条款。上游许可说明见[官方仓库 License 小节](https://github.com/TeleHuman/PBHC#license)；本仓库的 `LICENCE` 不替代上游动作数据和第三方材料的许可条款。
