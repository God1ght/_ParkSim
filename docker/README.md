# ParkSim-JTH 一体化镜像（parksim-jth:v2）

自带 **ROS 2 Foxy + 仿真器 + webviz 前端 + jth_b1 地图资产**的单文件 Docker 镜像。
目标机**不需要** ROS、不需要 conda、不需要任何源码——只要装了 Docker，一条命令就能起服，
浏览器打开即用。

---

## 0. 本版版本与校验（交付/迁移时请核对）

| 项 | 值 |
|---|---|
| 源码分支 / HEAD | `JTH/vision-html` / **`ef4f62d`** |
| 相对上一交付点（`f7223a7`）的 3 笔 | `6df928d`（修复入口放行门三处缺陷）/ `cf16dee`（持续生成 + 占用率门控 + 可选上限 + `spawn_stuck` 误报修复）/ `ef4f62d`（不限并发时 `self.vehicles` 句柄无界增长） |
| `webviz/server.py` | md5 `bf13bddb4edfa72e7df7c160698a3646` |
| `webviz/static/app.js` | md5 `a22dfa78ff0da2186dc6c8942fc81f51` |
| `webviz/static/index.html` | md5 `0a81750fd2abb61a98ab144bfb543f4c`（内含 `app.js?v=31-panels`、`style.css?v=10`、`charts.js?v=4`、`charts.css?v=3`） |
| `webviz/static/style.css` | md5 `c9f9b6c2956deb88d6100c0859474d06` |
| `webviz/static/charts.js` | md5 `5c2b3c2e7e051d3746647db01a5b36e8` |
| `webviz/static/charts.css` | md5 `6c4ba2293641c6e813b374d59cd777a0` |
| `workspace/src/parksim/src/simulator_node.py` | md5 `02ec820c5bc4151517c6225de2b67344` |

- 上表源码文件的 md5 **在仓库与镜像内逐字节一致**（镜像里是构建时 COPY 进去的实体文件，不是符号链接）。

**构建产物侧（每次构建都不同，因此本表刻意不写死数值）**

| 项 | 权威来源 |
|---|---|
| 镜像 tag | `parksim-jth:v2` |
| 镜像 id / 构建时间 / 源码 commit | `docker image inspect -f '{{.Id}}' parksim-jth:v2`；`./install.sh --version`（读镜像内 `/opt/parksim/BUILDINFO`） |
| tar 体积 / 校验值 | `ls -l parksim-jth-v2.tar.gz`；`sha256sum -c parksim-jth-v2.tar.gz.sha256` |

> **为什么这里不再写死镜像 id / 构建时间 / tar 字节数与 md5**：本文件自身会被 `COPY` 进它正在
> 描述的那个镜像，写死构建产物数值必然在下一次构建后变成错值——本表曾因此长期停留在
> `724fd75` / 镜像 `b4587e3e` / tar `376,727,675 B` / md5 `ce2f1cab`，**照它核对会把正确的
> 新包判成错包**。构建产物一律以 `.sha256` 与 `BUILDINFO` 为准。
- tar 已做**装载回验**：`docker load -i parksim-jth-v2.tar.gz` 后 `docker image inspect` 得到的
  id 与该次构建的源镜像 id **完全一致**，即"打包—迁移—装载"链路无失真
  （该流程由 `docker/install.sh` 与冒烟脚本共同覆盖）。
- 本轮相对上一版的实质改动：**换基线 + 换构建方式 + 修交付链**。
  ① 交付基线从 `f7223a7` 升到 `ef4f62d`（补上持续生成 / 占用率门控 / 可选上限 / 句柄泄漏修复）；
  ② 镜像改为**多阶段瘦身构建**（构建工具链不进最终镜像）：**1,203,652,520 B → 930,247,825 B（−22.7%）**，
  并新增 8 条**构建期**运行期断言（`C1a/C1b/C1c/C2a/C2b/C3/C4a/C4b`，见 `docker/assert_runtime_deps.sh`）；
  ③ `build_image.sh` 新增 `--dockerfile` 并默认走多阶段（此前它**无法**选择 Dockerfile，多阶段只能手动 `docker build`）；
  ④ `install.sh` 改为**按交付目录里的 tar 文件名自动推导镜像标签**，以后换版本号不必改脚本；
  ⑤ 修正本表——它此前停在 `a3e8fd6`（一个早已作废的历史版本），照它核对会把正确的新包判成错包。

---

## 1. 目标机要求

| 项 | 要求 |
|---|---|
| Docker | >= 19.03（需要 `docker load` / `docker run`） |
| 架构 | linux/amd64（镜像基于 ubuntu:20.04，内含 x86_64 二进制） |
| 磁盘 | 镜像展开后约 **0.93 GB**，加 tar 包（约 0.4 GB）共需 **≥ 1.8 GB** |
| 内存 | 建议 **≥ 8 GB**（实测 23 车时容器峰值内存 **3.95 GiB**；旧文写「30+ 车峰值 ~1 GB」是不实数据） |
| CPU | 建议 **≥ 8 核**（实测 23 车时峰值约 **13.5 / 16 核**，load average 42.54） |
| 网络 | 运行阶段**不需要外网**；仅安装阶段需要能 `docker load` 本地文件 |
| 其它 | **无需** ROS / conda / python / 编译工具 / 源代码 |

---

## 2. 一条命令起服

```bash
# 解压后的目录里（含 parksim-jth-v2.tar.gz、install.sh、docker-compose.yml、README.md）
chmod +x install.sh
./install.sh
```

`install.sh` 会：`docker load` 镜像 → 后台启动容器（宿主 **8098** → 容器 8099）
→ 轮询 `/` 直到 HTTP 就绪，最后打印访问地址。

然后浏览器打开：

```
http://<目标机IP>:8098/
```

页面会自动加载 jth_b1 底图，但**默认停在「已停止」态**——点页面上的「开启仿真」才发车。
需要容器一启动就自动发车时，加 `-e PARKSIM_AUTOSTART=1`（见 §3）。

手动等价命令（不想用脚本时）：

```bash
docker load -i parksim-jth-v2.tar.gz
docker run -d --name parksim-jth --restart unless-stopped \
    -p 8098:8099 -e PORT=8099 -e PARKSIM_MAP=jth_b1 \
    -e PARKSIM_AUTOSTART=0 \
    -e ROS_LOCALHOST_ONLY=1 \
    parksim-jth:v2
```

或用 compose：

```bash
docker load -i parksim-jth-v2.tar.gz
docker compose up -d          # 读同目录 docker-compose.yml
```

常用操作：

```bash
docker logs -f parksim-jth        # 看日志（仿真 + webviz 全部走 stdout）
./install.sh --stop               # 停止并删除容器
./install.sh --port 9000          # 换宿主端口
./install.sh --autostart          # 开机即自动发车（等价 -e PARKSIM_AUTOSTART=1）
./install.sh --uninstall          # 删容器 + 删镜像
./install.sh --fg                 # 前台跑（Ctrl+C 退出）
```

---

## 3. 运行时环境变量（`docker run -e`）

| 变量 | 默认 | 说明 |
|---|---|---|
| `PORT` | `8099` | 容器内监听端口；改这个要同步改 `-p` 的右侧 |
| `PARKSIM_MAP` | `jth_b1` | 启动时加载的地图名（对应 `priorFiles/maps/<name>`） |
| `PARKSIM_AUTOSTART` | `0` | 是否开机即自动发车。`0`（默认）= 就绪后停在「已停止」态，由页面「开启仿真」触发；`1`/`true`/`yes`/`on` = 容器一起来就开跑 |
| `ROS_DOMAIN_ID` | `0` | DDS 域 |
| `ROS_LOCALHOST_ONLY` | `1` | **已在镜像内固化**；置 `1` 时 DDS 只走容器自身 `lo` |
| `PARKSIM_ALLOW_FOREIGN_VEHICLES` | 空（=关） | 逃生开关。置真值（`1`/`true`/`yes`/`on`，大小写不敏感）时**整路跳过**外来节点检查：两个启动检查点 + 运行期晚期告警都不做，且启动期会打印 `skipping foreign node check` |
| `PARKSIM_FOREIGN_GUARD_WINDOW` | `3.0` | **只**控制「启动期最小观察窗」（秒）。设 `0` = 关闭该窗；与上一行**相互独立**：关窗**不会**关掉晚期告警。⚠️ 实测安全边界：该值**会被夹到 ≤ `_FOREIGN_CHECK_MAX_WAIT`(10 s)**；设 `3600` 只会等 10 s（早期版本会**无限挂死、一辆车都不发**）；`inf` / `nan` / 非法值一律**回落默认 3.0** |
| `PARKSIM_ASSET_ROOT` | 空 | ⚠️ **只影响 webviz 桥，不影响仿真节点** —— 详见 §8.3，别拿它当"让仿真换资产目录"的旋钮 |

### 3.2 外来节点守护：启动期检查点 + 运行期晚期告警

仿真侧的 `simulator_node` 有两道机制处理"本域里出现了别人的 vehicle/simulator 节点"：

1. **启动期两个检查点**（startup / post-dataset-load）：发现外来 vehicle 会直接
   `ERROR … Some vehicle nodes are not shut down cleanly` → `RuntimeError: simulator exited early`
   → **进程退出**。这是**硬拦**，行为未变。
2. **运行期晚期告警**（本版新增）：启动窗口**之后**才被本域发现的外来节点，
   旧实现会**完全静默**（仿真照跑、操作者不知情）。现在会打一条 WARN：

   ```
   Late cross-talk detected: foreign node(s) [foreign vehicle=['/vehicle_999/vehicle']] appeared
   after startup checkpoints; this instance and another are sharing one DDS domain.
   Isolate with ROS_LOCALHOST_ONLY=1 or set PARKSIM_ALLOW_FOREIGN_VEHICLES=1 to silence.
   ```

   性质：**只报不杀** —— 不 raise、不杀进程、仿真继续跑。设计上是"可见化"而非"硬拦"，
   避免把一个还能用的仿真直接打死。

   触发条件（四条同时满足）：仿真在跑（`sim_is_running`）→ 未超过观察窗 → 距上次扫描 ≥5 s 节流
   → 真的扫到外来节点。命中后由 `_late_cross_talk_warned` 置位，**整个进程生命周期只打一次**
   （防刷屏）。

   ⚠️ **观察窗只有 60 s**：起点是**两个启动检查点都过完的时刻**（`_late_window_t0`，即
   post-dataset-load 之后）。实测首车约在 T+12 s 出现，因此首车之后的有效覆盖约 **56 s**；
   **超过窗口之后才出现的串扰仍旧静默** —— 这条告警不是全程哨兵。
   需要全程保障请依赖 `ROS_LOCALHOST_ONLY=1` 隔离本身（§3.3）。

   **判定口径（本版修过一个真盲点）**：运行期扫描要排除本实例自己的车，早期实现是"按命名空间
   精确匹配"（`/vehicle_1 … /vehicle_<num_vehicles>` 都算自己的），结果**外来实例只要 id 落在自己
   区间内就会被静默排除** —— 而两个实例都用从 1 开始的同一套 id 生成器，晚启动那家的 id 必然与
   我方重叠，**正是最该报警的场景反而报不出来**。实测对照：注入 `/vehicle_999` 报警、注入
   `/vehicle_1` 完全静默。
   现改为按「同名同命名空间出现次数 − 自身应占条数」判定（DDS 图 API 对同名同 ns 会返回多条）：
   复现同一实验，注入 `/vehicle_1` 现在**稳定报 1 条**，且 WARN 文本带完整节点名
   （`/vehicle_1/vehicle`，早期只打裸 `vehicle`，看不出是哪个节点）。
   干净路径已验证零误报：跑满 210 s / 累计发车 46 辆、车正常停好退出、节点数从 24 降到 13，全程 0 条。

### ⚠️ 3.3 DDS 隔离的正确姿势（重要，早期文档写错了）

**错误说法**（本 README 早期版本曾这样写）：默认 bridge + 端口映射下，DDS 在容器自己的
网络命名空间里，不会与宿主/其它 ParkSim 实例互相发现。

**实测事实**：**DDS 发现会穿透 bridge 网络**——双向。容器内建的节点，宿主与其它容器能看到；
宿主建的节点，容器也能看到。直接后果是可复现的：当宿主/其它容器已有 ParkSim 实例在跑
（里面有 vehicle 节点）时，用默认参数再起一个容器，`simulator_node` 会发现这些**外来 vehicle**，
报 `ERROR ...Some vehicle nodes are not shut down cleanly`，进而
`RuntimeError: simulator exited early`，**容器直接退出(1)**。且这是**竞态**：同一命令在
「那一刻外来实例没有 vehicle 节点」时会侥幸成功，于是表现为「时好时坏」。

**本镜像的处置**：在 `Dockerfile` 里用 `ENV ROS_LOCALHOST_ONLY=1` **固化**隔离，
`run_app.sh` 再兜一层 `export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"`；
`install.sh` / `docker-compose.yml` / 上面的手动 `docker run` 示例也默认带上
`-e ROS_LOCALHOST_ONLY=1`。置 1 后容器内 DDS 只在本机 `lo` 上发现，**与宿主及其它容器完全互不可见**
（已用唯一命名节点交叉探针验证：新容器互相 `NONE`、也看不到 8098 demo 的探针）。

> 只在需要与外部 ROS 节点互相发现（多机联调）时才显式改为 `0`（如
> `-e ROS_LOCALHOST_ONLY=0` 或 `ROS_LOCALHOST_ONLY=0 ./install.sh`），并自行承担上述发现串扰风险。

---

## 4. 页面控制行为与状态显示（本版新增 / 变更）

这一节写的是**人在浏览器里能直接看到的行为**，全部经真实 Chromium 实测，不是"代码里有所以算实现了"。

### 4.1 暂停状态灯（新）

旧版点「暂停」后，右上角状态灯**仍显示「仿真运行中 · N 车」**，只有车不动 ——
灯和行为不一致，排查时极易误判成"暂停没生效"。

本版新增 `sim-paused` 状态：暂停时灯显示 **「已暂停 · N 车」**（`--warn` 色），恢复后回到
「仿真运行中 · N 车」。同步发生在 `updateSimChip()`，并在**两处**调用：收到 `{type:'paused'}`
的 WS 推送时、以及点暂停按钮的本地回执时 —— 所以不管是自己点的还是服务端推的，灯都会跟上。

> 一个容易误报的旁证：暂停瞬间车数可能 +1（如 16→17）。这不是"暂停还在发车"，而是暂停前
> 已进 spawn 队列的车在这一帧落了地；`timer_callback` 本身有 `if self.sim_is_running:` 门禁。
> 实测干净暂停 60 s：仿真时钟冻结在 40.3 s、车数稳定 12 辆不变。**已确认为非缺陷，未改代码。**

### 4.2 确认框改为页面内非阻塞弹窗（真实缺陷修复）

旧版用浏览器原生 `window.confirm()`。实测：**模态期间整个页面的 JS 事件循环被冻住** ——
在确认框上停留 45 s，心跳出现 **92.5 s 的空洞**；更糟的是「停止」指令**确实发出去了**，
但服务端这段时间处理不到、前端也收不到回执，表现为**静默失败**（点了没反应、也不报错）。

本版改为页面内自绘的 `#confirmModal`（`askConfirm(text)` 返回 Promise），不阻塞事件循环。
同条件复测：停留 40 s，心跳**最大间隔 0 ms**，停止指令正常送达并成功；
取消路径（Esc / 点「取消」）实测**不会**发出停止指令。

### 4.3 控制指令的 ack 超时与重发（新）

控制指令（停止 / 暂停）现在有确认机制：

- 发出后等 **12 s** ack（`CONTROL_ACK_TIMEOUT_MS`）；
- 超时后**重发一次**；再超时就在页面上给出**可见的失败提示**（不再是"点了没反应"）；
- **重发用的是首次发送时冻结的那份载荷，逐字节相同**。这是刻意的：暂停是 toggle，
  若重发时按当前状态重新计算，可能把语义反转（本来要暂停 → 重发变成恢复）。
  已用抓帧证明首次与重发的 payload 完全一致。

失败矩阵实测（真实浏览器）：正常停止 / 确认框停留 40 s / Esc 取消 / 桥接无响应时的停止 /
桥接无响应时的暂停 / 失败后手工重试 —— **全部符合预期**；全流程交互后 JS 错误列表为 `[]`。

### 4.4 暂停不再被误报成「数据流停滞」（真实缺陷修复）

早期版本点「暂停」后约 20 s 就会弹红色横幅 `仿真数据流停滞（约 22 秒未推进）。点击「重启仿真」恢复。`
—— 因为 webviz 的 `sim_watchdog` 只看仿真时钟有没有推进，**对"暂停"无感知**；而暂停时时钟本来就该冻结。
更糟的是它引导用户「重启仿真」，重启会**直接毁掉这次暂停的运行**，且停止后横幅还残留。

本版：桥节点把 `sim_paused` 写入共享状态，看门狗在暂停期间**跳过停滞判定**并持续刷新基准
（否则恢复瞬间会把暂停时长一次算成停滞）；暂停时若已误报则主动解除；停止/未启动时不做停滞判定。

实测（真实浏览器）：暂停 **55~66 s** → 状态灯「已暂停 · N 车」、**红色横幅保持隐藏**；
恢复后仿真时钟继续推进、无即时误报；停止后横幅回落 idle 文案「底图已加载（仿真未启动）…」。
**反向验证**：用 `kill -STOP` 冻住仿真进程（不是暂停），停滞检测**仍然在 35 s 内正常报警**
（`status: stalled / 约 22 秒`）—— 说明真实停滞没被一起吞掉。

### 4.5 左右常驻监控面板（新）

页面两侧新增两块常驻面板，由控制坞的两个按钮开关：**「指标」**（左，`#btnMetrics`）/
**「事件」**（右，`#btnEvents`）。默认**收起**，点开**即刻出数**（`setSidePanel(..., true)` 会强制
立刻刷新一次，不等节流窗口），面板上的 `×` 收起。刷新由 `updateChips()` 驱动，固定 **400 ms**
（`PANEL_REFRESH_MS = 400`）一次，且**只刷当前可见的那块**（收起的不算）。

**左：运行指标**（`#panelMetrics`）—— 数据全部来自已有的 WS 推送（`frame` / `occupancy` /
`departing`），**不额外发请求**，因此开面板不会增加桥的负担：

| 分组 | 字段 |
|---|---|
| 仿真状态 | 生命周期、仿真时刻 |
| 车辆状态分布 | 行驶中 / 泊车机动 / 制动等待 / 已完成 / 在场合计 |
| 泊位占用 | 已占用 / 总数、占用率、入库呈现（去重）、正在出库 |
| 链路时效 | 前端 FPS、WS 延迟 |

**右：冲突事件 · 监管建议**（`#panelEvents`）

- **当前冲突**：让行对（`wait` 字段，一对车互相等待时升级为 `互等环（死锁前兆）`，
  且 A→B / B→A 只展示一条）、**最小车距**、车辆与停放车的最小距离。
- **生效告警**：`S.activeAlerts` 的全部告警码，外加比 alert 更严重的**横幅态**
  （`sim_dead` / `stalled` / `error` / `disconnected` / `assets_missing`）。
- **下一步监管建议**：**纯前端规则引擎**（`buildAdvice()`）——服务端**不**产出"建议"类数据，
  所以这一栏的内容完全由浏览器本地推导（如"泊位 N 被多车同时瞄准 → 检查泊位分配与互斥锁"），
  按 `bad > warn > info` 排序。

> ⚠️ **「最小车距」是按车辆中心点算的，不是车身净距**（车身约 4.6 × 1.85 m）。面板 meta 里已写明；
> 当净距看会系统性低估风险。
>
> ⚠️ **让行判定存在退化路径**：若对端服务端版本较旧 / `wait` 字段恒为 0，会退化到用状态色
> `c === 2`（制动等待）近似，并在 meta 里**明确标注"退化判定"** —— 看到这行字即说明当前冲突
> 信息的精度较低，不要据此下强结论。
>
> ℹ️ 面板是**只读可见化**：不改变仿真行为、不下发任何控制指令、不写任何状态。

---

## 5. 镜像里装了什么 / 怎么组织的

镜像刻意把载荷放在**与构建机完全相同的绝对路径**下——ROS 的 install-tree
（`setup.bash`、ament index）内部写死了绝对路径，重定位极易损坏，路径同构是最稳的做法：

```
/media/step/data/Yccc7/ParkSim-JTH/
├── deps/ros/foxy/          # 内置 ROS 2 Foxy install-tree（278M，非 /opt/ros）
├── deps/dlp-dataset/dlp/   # DLP 数据集 python 包（webviz/simulator 都 import）
└── _ParkSim/               # 源码（git archive HEAD，commit 见镜像内 /opt/parksim/BUILDINFO）+ 镜像内 colcon build 产物
    ├── python/parksim/
    │   ├── webviz/         # 浏览器前端 + WS 桥（server.py）
    │   ├── priorFiles/
    │   │   ├── maps/jth_b1/   # jth_b1 全套资产：有向图、逐泊位机动表、车位、障碍物、底图
    │   │   ├── parking_maneuvers.pickle 等（现已在库，见 §8）
    │   │   └── data/DJI_0012_*.json      # webviz 启动自检所需：scene/agents/obstacles
    │   │                                 # 为真实文件，frames/instances 为 {} 占位
    │   └── ...
    └── workspace/install/  # colcon 构建出的 parksim 包（15 msg + 1 srv）
```

入口脚本 `/usr/local/bin/run_app.sh` 做四件事：导出环境变量 → `source` 内置 foxy 树
→ `source` `workspace/install/setup.bash` → `exec python3 -u server.py --port 8099
--control --manage-sim [--auto-start] --launch-args map:=jth_b1`（`--auto-start` 仅在
`PARKSIM_AUTOSTART=1` 时追加，**默认不发车**）。
**所有日志走 stdout/stderr**，所以 `docker logs` 能看到 webviz 与仿真节点的全部输出。

---

## 6. 常见问题（FAQ）

**Q: 打开页面空白 / 连不上？**
先看日志：`docker logs parksim-jth`。正常启动会依次出现
`[webviz] repo root: ...` → `[webviz] static from map layout: jth_b1 (269 spots, ...)`
→ `[webviz] serving on http://0.0.0.0:8099/`。若只有前半段，多半是宿主端口被占用，
换 `--port` 重来。

**Q: 8098 被占用？**
`./install.sh --port 9000`（会自动映射到容器内 8099）。

**Q: 页面在转圈、车辆不动？**
正常——容器**默认不发车**，请在页面上点「开启仿真」。若希望容器一起来就自动跑，
用 `-e PARKSIM_AUTOSTART=1`（或 `./install.sh --autostart`）重启容器。

**Q: 只读模式（不带 `--manage-sim`）怎么识别？**
只读模式下**首页外观与正常版完全相同**——不会显示任何「只读」角标，差异是**行为级**的：
点「Restart」会返回 `status: error`、点「Stop」会返回 `stop_failed`（仿真不受控）。
运维排查时若发现按钮点了没反应，先确认容器是不是以只读方式起的（本镜像入口默认带
`--control --manage-sim`，正常部署不会出现只读）。

**Q: 点「停止」/「暂停」没反应，或页面提示失败？**
本版已把"静默失败"变成可见：指令 12 s 未 ack 会**自动重发一次**，再失败就在页面上明确提示
（§4.3）。看到失败提示时，先看 `docker logs` 里 bridge 侧是否还活着；桥活着而仍失败，
多半是仿真进程已处于 finished/starting 中间态，重试一次即可。
**旧版的原生 `window.confirm` 会冻住页面事件循环导致静默失败**的缺陷本版已修（§4.2）。

**Q: 日志里出现 `Late cross-talk detected: …` 是什么意思？**
说明本实例的 DDS 域里**出现了别人的 vehicle/simulator 节点**，且是在启动检查点之后才被发现的。
**这条只是告警，不会杀进程**，仿真仍在跑。处理：确认是否真的需要多实例共存，
不需要就用 `ROS_LOCALHOST_ONLY=1` 隔离（本镜像已默认固化）；确需联调就设
`PARKSIM_ALLOW_FOREIGN_VEHICLES=1` 静音。注意它**只打一次**且只有 60 s 观察窗（§8.9），
别把它当全程哨兵。

**Q: 页面提示"资产缺失"/底图降级是什么情况？**
webviz 启动时做资产自检，缺文件会在日志里逐条列出并进入**降级模式**：只能显示底图、
不能发车。本镜像自带完整 jth_b1 资产，正常部署不会出现；出现即说明资产目录被挂载覆盖了。
`PARKSIM_ASSET_ROOT` 只能影响**桥**的自检路径，不影响仿真（§8.8）。

**Q: 想和宿主上的 ROS 节点互相发现？**
默认**不**互相发现（镜像固化 `ROS_LOCALHOST_ONLY=1`，隔离是特性不是缺陷，见 §3）。
确需联调时，把 `docker-compose.yml` 里的 `ports` 换成 `network_mode: host`
（容器内监听 8099 会直接落在宿主上，注意端口冲突），并显式设 `ROS_LOCALHOST_ONLY=0`、
保证两边 `ROS_DOMAIN_ID` 一致。

**Q: 怎么进容器排查？**
```bash
docker exec -it parksim-jth bash
# 里面已可通过
source /media/step/data/Yccc7/ParkSim-JTH/deps/ros/foxy/setup.bash
source /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/workspace/install/setup.bash
ros2 topic list
```
镜像内已装 `netifaces`，`ros2 node list` / `ros2 topic list` 可直接用。

**Q: 仿真数据/日志写在哪？会丢吗？**
写在容器内 `/media/.../_ParkSim/vehicle_log`、`webviz_recordings` 等目录（容器可写层）。
`docker restart` 后回到镜像基线，不落盘到宿主。需要留存请自行加 `-v` 挂载。

**Q: `docker exec parksim-jth ps` 看到几个 `[ros2] <defunct>`？**
这是**上游应用既有行为，不是本镜像引入的**：`SimManager` 停仿真时先 SIGINT 整个
进程组，中间层 `ros2 launch` 先退出，导致它拉起的孙进程被 reparent 到 PID 1 而无人
`wait()`，成为僵尸（Z 态，不占 CPU/内存）。已核对：构建机上正在运行的宿主 8099 实例
有**完全相同的 9 个** `[ros2] <defunct>`。真正的仿真进程（`simulator_node.py` /
`vehicle_node.py`）在 stop 后**没有**残留——这一点已用探针 + `ps` 验证过。

**Q: 可以用哪些地图？**
镜像里完整可用的只有 **jth_b1**。7.5 GB 的 DJI 全量数据集**没有**打进去，
所以 DJI 选项只能显示底图、无法做经验回放。

---

## 7. 从源码重建镜像（可选）

构建机需要：Docker、能访问 pypi 镜像源（默认清华）、能 apt（默认 archive.ubuntu.com）。

```bash
# 1) 准备 payload（都放在构建暂存区，不进 git）
BUILD=/media/step/data/parksim-image-build
S=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/priorFiles
DJI=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/priorFiles/data
mkdir -p $BUILD/payload/{parksim,dlp,dji_subset,priorfiles_root}
cp -a /media/step/data/Yccc7/ParkSim-JTH/deps/ros/foxy            $BUILD/payload/foxy
cp -a /media/step/data/Yccc7/ParkSim-JTH/deps/dlp-dataset/dlp     $BUILD/payload/dlp/
$(cd /media/step/data/Yccc7/ParkSim-JTH/_ParkSim && git archive HEAD) \
    | tar -x -C $BUILD/payload/parksim                       # 源码（含 jth_b1 地图资产）
cp -a $S/parking_maneuvers.pickle $S/spots_data.pickle \
      $S/waypoints_graph.pickle   $S/agents_data_0012.pickle  $BUILD/payload/priorfiles_root/
cp -a $DJI/DJI_0012_{scene,agents,obstacles}.json             $BUILD/payload/dji_subset/
printf '{}' > $BUILD/payload/dji_subset/DJI_0012_frames.json      # 占位（内容不被使用）
printf '{}' > $BUILD/payload/dji_subset/DJI_0012_instances.json   # 占位（内容不被使用）

# 2) 构建（首次约 7 分钟；改 payload 后借助缓存增量构建 ~15 秒）
cd $BUILD && docker build -t parksim-jth:v2 .
```

构建期会**自动断言**关键资产存在且 DJI json 可解析（缺文件/坏 json 直接 build 失败，
不会拖到运行时才炸）。

打完镜像后导出制品：

```bash
mkdir -p /media/step/data/ParkSim-JTH-image
docker save parksim-jth:v2 | gzip -6 > /media/step/data/ParkSim-JTH-image/parksim-jth-v2.tar.gz
md5sum parksim-jth-v2.tar.gz        # 与 §0 的校验表对一下
```

导出后建议做一次**装载回验**（这是唯一能证明"包没坏、迁移等价"的手段）：

```bash
docker load -i parksim-jth-v2.tar.gz
docker image inspect parksim-jth:v2 --format '{{.Id}}'   # 应等于 §0 里的镜像 id
```

> 若镜像有更新，**必须先重新导出 tar 再发货** —— tar 不会自动跟着镜像变。
> 实测踩过一次：镜像已在 06:51 重建完毕，但目录里的 tar 仍是 05:33 的旧包（md5 `a5afe318…`），
> 两者相差两个提交。发货前请核对 §0 的 md5。

---

## 8. 已知限制

1. **只带 jth_b1**：为了体积，7.5 GB DJI 数据集未打包。DJI 地图只能显示底图。
2. **priorFiles 的 4 个 pickle（已入库）**：`parking_maneuvers.pickle`、`waypoints_graph.pickle`、
   `spots_data.pickle`、`agents_data_0012.pickle` **曾经**被根 `.gitignore` 的 `*.pickle`
   静默排除，导致 `git archive` 拿不到——这是早期 `vehicle_node` 启动即 `FileNotFoundError`
   的真凶。**自 `cc5c98f` 起这 4 个文件已正式入库**，`git archive HEAD` 现能正常拿到。
   因此 `Dockerfile` 第 6 步的 `COPY payload/priorfiles_root/` 现在**只是冗余兜底**
   （防止将来有人回退到未入库的提交或本机误删时镜像仍能构建成功），非必需：
   - `parking_maneuvers.pickle`（必需，`vehicle_node` 无条件打开，缺了车直接起不来）
   - `waypoints_graph.pickle`（`graph_cost` / `nearest_entrance` 分配器用）
   - `spots_data.pickle`（默认车位表回退）
   - `agents_data_0012.pickle`（replay 默认 agents）
3. **DJI_0012 只保留 3 个真实文件 + 2 个占位**：webviz 启动时 `Dataset.load()` 会
   **无条件**打开 `DJI_0012_{frames,agents,instances,obstacles,scene}.json`，
   jth_b1 下这些内容不被使用但文件必须存在。因此：
   - `scene.json`（15 KB，提供 `scene_token`，也是 DLP 回退分支的障碍物来源）
   - `obstacles.json`（58 KB）、`agents.json`（46 KB，幽灵层用）→ 保留真实文件
   - `frames.json` / `instances.json`（原 26 MB + 242 MB，仅幽灵层/经验回放读取）
     → 用 `{}` 占位，**已实测冒烟全过**（对比实验：真实文件 vs `{}`，两次探针 10/10
     PASS、车辆均正常出现并移动）。这一项当时让镜像从 1.54 GB 降到 1.28 GB
     （**历史数值**；当前体积见 §0「构建产物侧」的权威来源）。
   - 7.5 GB 的 DJI 全量数据集不带。
4. 镜像内 foxy 树、`_ParkSim` 路径与构建机**完全一致**；若要改路径，
   必须同时改 `setup.bash` 里的绝对路径，否则 ament index 会失效。
5. **bridge 网络不隔离 DDS**：这是本镜像把 `ROS_LOCALHOST_ONLY=1` 固化的原因，见 §3.3。
   若有人手动改成 `0`（多机联调），务必意识到多个实例之间会互相发现。
6. **跨隔离边界的 DDS 发现：>12 s 且不收敛，加观察窗没用**。实测 `--network host` +
   `ROS_LOCALHOST_ONLY=0` 的跨边界发现，等 12 s 仍未收敛，且**继续等也不会好** —— 这不是
   "起得慢"，是隔离模式下发现本身就不可靠。因此不要试图用"把等待时间调大"来绕开，
   **正确解法是隔离本身**（`ROS_LOCALHOST_ONLY=1`，本镜像已固化）。
7. **容器启动比宿主多约 3 s**：同样流程在宿主上是 0 增量，容器里约 +3 s（镜像层解压/文件系统开销）。
   排障时别把这 3 s 误当成"卡住了"。
8. **`PARKSIM_ASSET_ROOT` 只影响 webviz 桥，不影响仿真节点**（易踩）：
   `python/parksim/webviz/server.py` 里 `resolve_asset_root()`（L134-141）是**唯一**读这个变量的
   地方（全文件 4 处命中：L120/137/140/192）；仿真侧节点对它的引用次数是 **0**。
   含义：设了它只会让**桥接**去别的目录找资产（用来复现"资产缺失"降级场景很有用），
   **仿真节点仍然读真实资产**。所以"用 `PARKSIM_ASSET_ROOT` 让整体换一套资产"是错的用法；
   想让仿真也换，得改仿真侧的资产根（`PARKSIM_DATA_ROOT` 等）或挂载覆盖目录。
   - **实测证据**：用 `-e PARKSIM_ASSET_ROOT=/tmp/empty_assets2`（空目录）起容器，页面出现黄色横幅
     `关键资产缺失：… 等共 7 项`，但**点「开启仿真」后仿真照常发车**，
     车辆正常生成、寻路、入库，实测跑到 13 辆。
   - ⚠️ 因此横幅措辞**不再断言**"仿真将无法正常发车"（自 `263ca5f` 起改为：
     `桥侧资产自检未通过；若为 PARKSIM_ASSET_ROOT 覆盖所致，仿真节点仍读真实资产、可正常发车；
     若资产确实缺失，发车会失败——详见服务端日志`）。
     判读时仍建议看 `docker logs` 里仿真节点有没有真的报 `FileNotFoundError`：
     真正的资产缺失（仿真侧也读不到）才会导致发车失败。
9. **晚期串扰告警的三个边界**（见 §3.2，别把它当全程哨兵）：
   - **60 s 观察窗**：起点是两个启动检查点都过完的时刻，超过窗口后才出现的串扰**仍旧静默**；
   - **只打一次**：`_late_cross_talk_warned` 命中即置位，进程生命周期内不再重复。
     也就是说"报过一次"不代表"之后就干净了" —— 日志里**没有**新 WARN **不等于没有**串扰，
     且窗口内**第二批**（不同于第一次那批的）外来节点也不会再报；
   - **自己的车已退出、外来同名节点顶上**这类场景仍无法区分（`enter_procs/exit_procs` 只保留存活
     进程，两种情形在图 API 上表现相同）。主场景（两实例 id 重叠 → 同名同 ns 出现两条）已能检出。
10. **仓库里有一份陈旧的前端副本**：`python/parksim/webviz/index.html`（`style.css?v=6`，结构更旧）
    **不在服务路径上** —— 服务端只吃 `python/parksim/webviz/static/index.html`。改前端请改 `static/`
    下那份，改错文件不会生效、还会制造分歧。
11. **前端缓存串是"双保险"**：服务端 `index()` 会按 `app.js` 的 **mtime** 自动重写 `?v=`，且首页带
    `Cache-Control: no-store`，所以即使忘了升手写的缓存串，改过的 JS 一般也会重新拉取。但手写的
    `?v=` 在重写失败的回退分支（`web.FileResponse`）里才生效 —— **所以改了前端仍应同时升缓存串**。
