# ParkSim-JTH 一体化镜像（parksim-jth:v1）

自带 **ROS 2 Foxy + 仿真器 + webviz 前端 + jth_b1 地图资产**的单文件 Docker 镜像。
目标机**不需要** ROS、不需要 conda、不需要任何源码——只要装了 Docker，一条命令就能起服，
浏览器打开即用。

---

## 1. 目标机要求

| 项 | 要求 |
|---|---|
| Docker | >= 19.03（需要 `docker load` / `docker run`） |
| 架构 | linux/amd64（镜像基于 ubuntu:20.04，内含 x86_64 二进制） |
| 磁盘 | 镜像展开后约 **1.27 GB**，加 tar 包共需 ≥ 1.8 GB |
| 内存 | 建议 ≥ 4 GB（仿真 30+ 车时实测峰值 ~1 GB） |
| 网络 | 运行阶段**不需要外网**；仅安装阶段需要能 `docker load` 本地文件 |
| 其它 | **无需** ROS / conda / python / 编译工具 / 源代码 |

---

## 2. 一条命令起服

```bash
# 解压后的目录里（含 parksim-jth-v1.tar.gz、install.sh）
tar xzf parksim-jth-v1.tar.gz.bundle 2>/dev/null || true   # 若整体打包过，先解出下面两个文件
chmod +x install.sh
./install.sh
```

`install.sh` 会：`docker load` 镜像 → 后台启动容器（宿主 **8098** → 容器 8099）
→ 轮询 `/` 直到 HTTP 就绪，最后打印访问地址。

然后浏览器打开：

```
http://<目标机IP>:8098/
```

页面会自动加载 jth_b1 底图并**自动开始仿真**（容器入口带了 `--auto-start`）。

手动等价命令（不想用脚本时）：

```bash
docker load -i parksim-jth-v1.tar.gz
docker run -d --name parksim-jth --restart unless-stopped \
    -p 8098:8099 -e PORT=8099 -e PARKSIM_MAP=jth_b1 \
    parksim-jth:v1
```

或用 compose：

```bash
docker load -i parksim-jth-v1.tar.gz
docker compose up -d          # 读同目录 docker-compose.yml
```

常用操作：

```bash
docker logs -f parksim-jth        # 看日志（仿真 + webviz 全部走 stdout）
./install.sh --stop               # 停止并删除容器
./install.sh --port 9000          # 换宿主端口
./install.sh --uninstall          # 删容器 + 删镜像
./install.sh --fg                 # 前台跑（Ctrl+C 退出）
```

---

## 3. 运行时环境变量（`docker run -e`）

| 变量 | 默认 | 说明 |
|---|---|---|
| `PORT` | `8099` | 容器内监听端口；改这个要同步改 `-p` 的右侧 |
| `PARKSIM_MAP` | `jth_b1` | 启动时加载的地图名（对应 `priorFiles/maps/<name>`） |
| `ROS_DOMAIN_ID` | `0` | DDS 域。容器自带网络命名空间，与宿主 ROS 天然隔离 |
| `ROS_LOCALHOST_ONLY` | 未设 | 置 `1` 时 DDS 只走容器 `lo`（单容器部署更省发现开销） |

> 容器默认走 **bridge + 端口映射**：DDS 在容器自己的网络命名空间里，不会和宿主上
> 其它 ParkSim/ROS 实例（比如某个跑在 8099 的进程）互相发现或串扰。

---

## 4. 镜像里装了什么 / 怎么组织的

镜像刻意把载荷放在**与构建机完全相同的绝对路径**下——ROS 的 install-tree
（`setup.bash`、ament index）内部写死了绝对路径，重定位极易损坏，路径同构是最稳的做法：

```
/media/step/data/Yccc7/ParkSim-JTH/
├── deps/ros/foxy/          # 内置 ROS 2 Foxy install-tree（278M，非 /opt/ros）
├── deps/dlp-dataset/dlp/   # DLP 数据集 python 包（webviz/simulator 都 import）
└── _ParkSim/               # 源码（git archive HEAD=0272503）+ 镜像内 colcon build 产物
    ├── python/parksim/
    │   ├── webviz/         # 浏览器前端 + WS 桥（server.py）
    │   ├── priorFiles/
    │   │   ├── maps/jth_b1/   # jth_b1 全套资产：有向图、逐泊位机动表、车位、障碍物、底图
    │   │   ├── parking_maneuvers.pickle 等（见 §7「必须补的 4 个文件」）
    │   │   └── data/DJI_0012_*.json      # webviz 启动自检所需：scene/agents/obstacles
    │   │                                 # 为真实文件，frames/instances 为 {} 占位
    │   └── ...
    └── workspace/install/  # colcon 构建出的 parksim 包（15 msg + 1 srv）
```

入口脚本 `/usr/local/bin/run_app.sh` 做四件事：导出环境变量 → `source` 内置 foxy 树
→ `source` `workspace/install/setup.bash` → `exec python3 -u server.py --port 8099
--control --manage-sim --auto-start --launch-args map:=jth_b1`。
**所有日志走 stdout/stderr**，所以 `docker logs` 能看到 webviz 与仿真节点的全部输出。

---

## 5. 常见问题（FAQ）

**Q: 打开页面空白 / 连不上？**
先看日志：`docker logs parksim-jth`。正常启动会依次出现
`[webviz] repo root: ...` → `[webviz] static from map layout: jth_b1 (269 spots, ...)`
→ `[webviz] serving on http://0.0.0.0:8099/`。若只有前半段，多半是宿主端口被占用，
换 `--port` 重来。

**Q: 8098 被占用？**
`./install.sh --port 9000`（会自动映射到容器内 8099）。

**Q: 页面在转圈、车辆不动？**
点页面上的「Restart / 开启仿真」。容器已默认 `--auto-start`，正常情况无需手动点。

**Q: 想和宿主上的 ROS 节点互相发现？**
把 `docker-compose.yml` 里的 `ports` 换成 `network_mode: host`（容器内监听 8099 会直接
落在宿主上，注意端口冲突），并保证两边 `ROS_DOMAIN_ID` 一致。

**Q: 怎么进容器排查？**
```bash
docker exec -it parksim-jth bash
# 里面已可通过
source /media/step/data/Yccc7/ParkSim-JTH/deps/ros/foxy/setup.bash
source /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/workspace/install/setup.bash
ros2 topic list
```

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

## 6. 从源码重建镜像（可选）

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
cd $BUILD && docker build -t parksim-jth:v1 .
```

构建期会**自动断言**关键资产存在且 DJI json 可解析（缺文件/坏 json 直接 build 失败，
不会拖到运行时才炸）。

打完镜像后导出制品：

```bash
mkdir -p /media/step/data/ParkSim-JTH-image
docker save parksim-jth:v1 | gzip -1 > /media/step/data/ParkSim-JTH-image/parksim-jth-v1.tar.gz
# → 约 426 MB（406 MiB），耗时 ~50 秒
```

---

## 7. 已知限制

1. **只带 jth_b1**：为了体积，7.5 GB DJI 数据集未打包。DJI 地图只能显示底图。
2. **必须补的 4 个 priorFiles 文件**：根 `.gitignore` 的 `*.pickle` 会把它们静默排除，
   `git archive` 拿不到，因此构建时单独 `COPY` 进去：
   - `parking_maneuvers.pickle`（**必需**，`vehicle_node` 无条件打开，缺了车直接起不来）
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
     PASS、车辆均正常出现并移动）。这一项让镜像从 1.54 GB 降到 1.27 GB。
   - 7.5 GB 的 DJI 全量数据集不带。
4. 镜像内 foxy 树、`_ParkSim` 路径与构建机**完全一致**；若要改路径，
   必须同时改 `setup.bash` 里的绝对路径，否则 ament index 会失效。
