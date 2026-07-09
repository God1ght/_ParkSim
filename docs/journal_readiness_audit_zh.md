# ParkSim-Qwen-VLA 期刊就绪性审计与联动优化清单

本文档面向高水平期刊论文标准，审计当前 ParkSim-Qwen-VLA 工作在代码、实验和论文三条线上的不足，并给出联动优化。当前不考虑 LoRA/DPO 或任何模型权重微调。

## 总体判断

当前工作已经具备可运行原型、长时段混驾仿真、候选 `spot-route-wait` 动作包、强规则 baseline、oracle 上界 baseline、决策审计和中文 IEEE 报告草稿。距离高水平期刊论文仍缺少真实 Qwen 大规模实验、视频证据、统计显著性、真实交通分布校准和完整论文结果图表。

## 代码不足与优化

| 不足 | 风险 | 已完成/建议优化 |
| --- | --- | --- |
| paper gate 只覆盖 paper 级最低要求 | 无法区分 pilot、paper 与 journal 证据等级 | 已新增 `journal` profile，要求更高种子数、密度覆盖、强 baseline、真实 Qwen、视频证据和论文源文件 |
| 强 baseline 与上界检查未机器化 | 论文比较可能只停留在弱规则对比 | 已将 `oracle_intent_bundle`、`centralized_min_cost` 纳入 gate 必选项 |
| 论文源文件与实验结果未绑定 | 文稿可能脱离可复现实验 | 已要求 `docs/qwen_vla_ieee_report_zh.tex/.md` 存在并纳入 gate |
| 视觉证据未纳入最终验收 | 难以证明混驾交互行为 | `journal` profile 默认要求 `video_manifest.json` |
| Qwen 决策审计可能被遗漏 | 不能复盘大模型动作是否合法 | gate 要求 `qwen_vla_decisions.jsonl` 和 `decision_audit.json` |

## 实验不足与优化

| 不足 | 期刊审稿风险 | 联动优化 |
| --- | --- | --- |
| 当前多数结果仍是 smoke/pilot | 不能证明统计稳定性 | 使用 `scripts/run_vla_journal_suite.sh` 运行 5 seeds、3 background modes、5 density levels 的真实 Qwen suite；默认 replay 仅保留 empty 密度，因此期刊矩阵为 55 个 paired scenarios、605 行指标 |
| mock Qwen 不能代表模型能力 | 审稿人会质疑 VLA 贡献 | journal gate 默认要求 real Qwen；mock 只能用于 CI |
| 缺少视频/动图证据 | 难以展示冲突、让行、泊位竞争 | 使用 visualizer pipeline 生成 `video_manifest.json`，再运行 journal gate |
| 缺少真实分布校准 | 混驾场景真实性不足 | 后续需补进出场分布、泊位偏好、驾驶风格、replay 轨迹统计对齐 |
| centralized baseline 仍是代价代理 | 理论上界不足 | 后续可实现真正多车 Hungarian/ILP/时空预约优化器 |

## 论文不足与优化

| 不足 | 联动优化 |
| --- | --- |
| 当前 IEEE 报告仍是方法型草稿 | 后续将 `paper_report` 输出的表格、统计检验和视频案例写入结果章节 |
| 缺少真实结果图 | 使用 `paper_summary.csv`、`paper_paired_deltas.csv`、`paper_statistical_tests.csv` 生成表格和图 |
| 贡献边界需更清晰 | 明确 Qwen-VLA 是冻结高层策略，不做低层控制、不做 LoRA/DPO |
| 安全性贡献需强化 | 将 safety shield、valid action set、task semantic gating 作为方法贡献 |
| 缺少失败案例分析 | 从 `decision_audit_failures.jsonl` 和视频中抽取典型错误案例 |

## 推荐执行顺序

1. 运行 dry-run 检查期刊矩阵：

```bash
PARKSIM_SUITE_DRY_RUN=1 ./scripts/run_vla_journal_suite.sh
```

2. 确认模型服务、显存和预计时长后运行真实 Qwen journal suite：

```bash
./scripts/run_vla_journal_suite.sh
```

3. 生成或补齐可视化视频证据：

```bash
PARKSIM_VIS_QWEN_MODE=real ./scripts/run_visualizer_policy_videos.sh
```

4. 运行最终期刊 gate：

```bash
./scripts/run_vla_journal_gate.sh <suite_dir>
```

5. 将通过 gate 的 `paper_report` 表格、统计检验和视频案例写入 `docs/qwen_vla_ieee_report_zh.tex` 的实验结果章节。

## 当前不可宣称的结论

- 不能宣称 Qwen-VLA 已在 paper-scale 上显著优于所有强 baseline。
- 不能宣称真实混驾分布已完成外部数据校准。
- 不能宣称 centralized baseline 是严格全局最优。
- 不能将 oracle baseline 作为可部署策略。

## 当前可以宣称的结论

- 已建立冻结 Qwen-VLA 的受约束高层决策框架。
- 已实现候选 `spot-route-wait` 动作包、安全盾、任务语义 gating 和决策审计。
- 已建立包含强规则、预约式、集中式代价代理和 oracle 上界的 baseline 体系。
- 已建立 journal profile gate，用于检查高水平期刊所需的实验覆盖、真实 Qwen、视频证据和论文源文件。
