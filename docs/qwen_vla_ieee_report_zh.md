# 面向人机混驾停车场的 ParkSim-Qwen-VLA 高层泊位-路径联合决策框架

IEEEtran 中文报告源文件见 `docs/qwen_vla_ieee_report_zh.tex`。本文档保留同一报告的阅读摘要。

## 核心结论

当前工作应被定位为冻结 Qwen-VLA 的受约束高层决策框架：Qwen 只在合法 `spot-route-wait` 候选动作包中选择 `action_id`，不输出低层控制，也不采用 LoRA/DPO。

## 已补强的 baseline

- `reservation_bundle`：近似时间窗预约策略。
- `rolling_horizon_bundle`：近似滚动时域重规划策略。
- `centralized_min_cost`：集中式最小系统代价代理。
- `oracle_intent_bundle`：真实背景意图可见的上界 baseline。

## 论文实验重点

高质量论文应重点比较 Qwen-VLA 与 `bundle_risk_aware`、`reservation_bundle`、`centralized_min_cost`、`oracle_intent_bundle`，而不是只比较最近泊位规则。

## 不做的内容

当前报告和实现不包含 LoRA、DPO 或模型权重微调。Qwen-VLA 优化仅限于状态输入、动作包生成、prompt contract、安全盾和实验协议。
