# ParkSim webviz（浏览器实时可视化）· v3

双主题 / 渲染升级（插值动画、参考路径、尾迹）/ 组件化 UI。独立桥进程 + WebSocket 帧流，不侵入仿真节点。

## 用法

```bash
source /media/step/data/Yccc7/ParkSim-JTH/env_ros.sh
cd /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python
python/parksim/webviz/run_web_sim.sh [端口，默认 8099]   # 一键：桥 + 仿真（无 dpg 窗口）
```

- 访问：<http://172.16.1.167:8099>（校园网）· <http://100.72.121.40:8099>（Tailscale）· <http://localhost:8099>（本机）
- 停止：`Ctrl+C`（脚本会一并退出仿真）；重跑同一命令即可开新一轮（自动清理旧进程）

## 主题（v3）

- 默认**深色「夜间演示」**；浅色「工程纸」风：`?theme=light` 或点「主题」按钮切换；选择记忆在浏览器（localStorage）
- 车辆/地图配色由前端主题调色板决定（`static/app.js` 中 `THEMES`）；`visualization.yaml` 的颜色不再影响 webviz
- 主题切换无需刷新；URL 参数：`?theme=dark|light`

## 交互

| 操作 | 效果 |
|---|---|
| 悬停车辆 | 信息卡（编号 / 状态 / 速度） |
| 单击车辆 | 跟随（相机锁定）+ 显示该车参考路径；单击空白或 Esc 取消 |
| 滚轮 / 拖拽 | 缩放 / 平移（拖拽自动退出跟随） |
| `空格` | 暂停 / 恢复（需 `--control`，一键脚本默认开启） |
| `H` | 演示模式（隐藏全部界面元素，仅留地图） |
| `T` / `R` | 切换主题 / 重置视图 |
| 「图层」按钮 | 抽屉：底图 / 网格 / 车位 / 航点 / 障碍车辆 / 数据智能体 / 参考路径 / 尾迹 / 编号（记忆在浏览器） |

## 数据协议（v3 增量，向后兼容）

- `init`：不变（地图 / 车位 / 航点 / 障碍 / 颜色 / options）
- `frame` 增量：
  - `vehicles[].v`：纵向速度 m/s（来自 `state.v.v_long`；vehicle_node 已加一行镜像 `v.v → v.v_long` 供可视化读取）
  - `fpath`：当前聚焦车辆的参考路径（≤64 点降采样；仅聚焦时下发，其余时刻为 null）
- 上行消息：`{type:'focus', id}` 设定/取消聚焦；`{type:'pause', value}`、`{type:'ping'}` 同前

## 方案面板与网页重启（v4）

- `run_web_sim.sh` 以「托管模式」启动（桥负责起停仿真）：
  - 页面底部控制坞「方案」按钮 → 面板：场景 / 泊位分配 / 路由规划 / 参考路径 / 机动方案 + 「应用并重启仿真」
  - 重启流程：桥优雅停止仿真（SIGINT → ≤12s → SIGTERM/SIGKILL + 残留清理）→ 带新参数重新 launch
  - 重启期间页面显示「正在重启仿真…」；完成后自动衔接新一轮（桥进程不重启，WebSocket 不断线）
- 只读模式（server.py 未加 `--manage-sim`）下按钮禁用，不影响观看；桥退出（Ctrl+C）会一并停止仿真
- 协议：上行 `{"type":"restart","config":{...}}`；下行 `{"type":"status","value":"restarting|started|error|busy",...}`
- 实测：页面切换 `example_custom + linear + online_rs` 重启成功（车辆按 custom 时间表生成）

## 方案面板细参数（v5）

- 面板在原有五项方案下拉基础上新增**模式相关参数区**（随所选场景模式显示/隐藏）：
  - `random`：入库数 / 出库数 / 到达间隔 / 随机种子 / 入口门控 y / 附加封锁车位 / 附加预占用车位
  - `replay`：时间倍率 / 最大智能体数
- **参数预填**：选择场景预设后自动带入该预设的现有值（含分配/路由等方案键），所见即所得；
- **应用机制**：桥把「预设/默认场景文件 + 网页覆盖」合成运行时文件 `$PARKSIM_ROOT/webviz_runtime.yaml` 后再重启；
  原配置文件不被改写（改动为会话性质，重启桥后回到文件值）；
- 注意：封锁/预占用为「追加」语义（在 simulator.yaml blocked_spots 之外追加）。

## 场地数据（数据场景）与初始化模式（v6）

「方案」面板按用途拆分为两块：

- **初始化**（场景初始化模式）：
  - `经验初始化`（replay）：按所选场地数据的**真实占用/障碍**初始化场景，并**回放该录制里的智能体**（agents_data_XXXX）
  - `随机生成`（random）：**车位占用随机重掷**（默认与数据集占用数量一致，可指定「占用数量」；「占用随机化」可关）+ 随机到发
  - `自定义时间表`（custom）：按 custom.file 时间表生成
- **场地数据**：桥启动时**扫描** `priorFiles/data/` 中所有可用的 DJI_XXXX 数据集（当前 30 个：DJI_0001–DJI_0030）并全部列出；选择后随重启应用 `map:=DJI_XXXX`

场景切换流程（不需要重启桥）：

1. 桥在重启线程中**重载 DLP 数据集**（约 3–7 秒），向所有客户端重推新的 `init`（障碍/车位/航点随之更新，页面自动重绘并重置视图）；
2. 仿真实例以 `map:=<场地>` 重启；智能体数据按场景自动解析（`agents_data_XXXX.pickle`，缺失时告警并跳过回放）；
3. 随机模式下仿真日志打印 `Random occupancy: N/364 spots occupied (dataset: M, seed=…)`。

停放车辆渲染（两态）：

- `dataset` 模式（经验初始化/自定义）：绘制数据集真实障碍多边形（`init.obstacles`）；
- `occupancy` 模式（随机生成）：按仿真发布的 `/occupancy` 车位占用绘制停放车辆（桥订阅后转发 `{type:'occupancy', data:[…]}`，仅在变化时下发）。

配套工具：`tools/build_agent_data.py`（DLP JSON → `agents_data_XXXX.pickle`，可批处理全场景；`--out-dir/--force/--data-dir` 可选）。

## 备注

- 本模块为独立脚本 + 静态文件，直接由仓库路径提供，**无需 install 树符号链接同步**
- `--record` 帧流录制 JSONL（`$PARKSIM_ROOT/webviz_recordings/`）；不加 `--control` 时为只读观看（暂停按钮禁用）
- 帧率参考：60 辆车 ~51–57 FPS（1080 桌面火狐，1920×1080）；单用户工具，focus 为全局单一
