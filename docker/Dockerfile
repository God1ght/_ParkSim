# =============================================================================
# ParkSim-JTH 一体化镜像（方案 C）
#
# 目标：目标机「无 ROS、无 conda、无源码」——一条 docker run 即起服，
#       浏览器打开 http://<host>:8098 即可用。
#
# 关键设计（与构建机 STEP-1080 现网行为等价）：
#   * 内置 ROS 2 Foxy：把本机 install-tree（deps/ros/foxy，278M）原样复制进
#     镜像，并**保持同一绝对路径**。该树的 setup.bash / ament index 内含绝对
#     路径，重定位极易损坏，因此路径同构是成本最低、最可靠的策略。
#   * _ParkSim 源码同样放在 **同一绝对路径**，保证：
#       - config/*.yaml 里的 ${PARKSIM_ROOT} 展开与现网一致
#       - priorFiles/maps/jth_b1/map.yaml 的 ${PARKSIM_PRIORFILES} 展开一致
#       - 镜像内 colcon build 出的 workspace/install 与现网同布局
#   * 镜像内用**系统 python3.8 + pip3**（不装 conda），依赖只装最小集
#     （requirements-app.txt），不装 RL/训练侧依赖。
#   * jth_b1 地图资产随源码 git archive 自带（8.4M）；7.5G DJI 数据集不带，
#     只带 webviz 启动时 dlp.dataset.Dataset.load() 必读的 DJI_0012 最小文件子集。
#   * ROS_LOCALHOST_ONLY=1 已在镜像内固化（见下方 ENV）：**默认 bridge 网络并不能
#     隔离 DDS 发现**——实测容器内 DDS 与宿主及其它容器的 DDS 会双向互相发现，
#     多个实例并存时会出现「发现到外来 vehicle 节点 → 容器启动即退出(1)」的竞态。
#     置 1 让 DDS 只走容器自身的 lo，从根上消除跨实例串扰。要做多机联调时显式覆盖为 0。
# =============================================================================
FROM ubuntu:20.04

LABEL org.opencontainers.image.title="parksim-jth" \
      org.opencontainers.image.description="Self-contained ParkSim JTH simulator + webviz (bundled ROS 2 Foxy, no conda, no /opt/ros)" \
      org.opencontainers.image.version="v1"

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Asia/Shanghai \
    PARKSIM_ENV_ROOT=/media/step/data/Yccc7/ParkSim-JTH \
    PARKSIM_ROOT=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim \
    PARKSIM_DATA_ROOT=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/priorFiles \
    ROS_FOXY_ROOT=/media/step/data/Yccc7/ParkSim-JTH/deps/ros/foxy \
    DLP_DATASET_ROOT=/media/step/data/Yccc7/ParkSim-JTH/deps/dlp-dataset \
    PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    PIP_NO_CACHE_DIR=1 \
    ROS_LOCALHOST_ONLY=1

# -----------------------------------------------------------------------------
# 1) 系统依赖（python3.8 + 编译链；ps/pkill 供 SimManager 清理仿真进程）
#    - libssl-dev    ：colcon 构建时 foxy 的 fastrtps-config.cmake 会
#                      find_package(OpenSSL REQUIRED)
#    - libtinyxml2-dev：foxy 的 fastrtps 导出 target 里**写死了**
#                      /usr/lib/x86_64-linux-gnu/libtinyxml2.so 的绝对路径
#                      （构建机装了该包），故目标镜像必须提供同一路径的 so。
# -----------------------------------------------------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3 python3-dev python3-pip python3-setuptools \
        build-essential cmake pkg-config \
        libssl-dev libtinyxml2-dev \
        procps psmisc \
        ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 2) Python 依赖（最小集，走清华镜像）
# -----------------------------------------------------------------------------
COPY requirements-app.txt /tmp/requirements-app.txt
RUN python3 -m pip install --no-cache-dir "pip==24.2" "setuptools==75.1.0" "wheel==0.44.0" \
 && python3 -m pip install --no-cache-dir -r /tmp/requirements-app.txt \
 && rm -f /tmp/requirements-app.txt

# -----------------------------------------------------------------------------
# 3) 载荷 —— 全部放在与构建机相同的绝对路径
# -----------------------------------------------------------------------------
# 3a) ROS 2 Foxy install-tree
COPY payload/foxy/ /media/step/data/Yccc7/ParkSim-JTH/deps/ros/foxy/
# 3b) ParkSim 源码（git archive HEAD=cc5c98f，含 jth_b1 地图资产与 4 个运行必需 pickle）
COPY payload/parksim/ /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/
# 3c) dlp 数据集 python 包（webviz 与 simulator 均 import；只带包，不带 277M pickle）
COPY payload/dlp/ /media/step/data/Yccc7/ParkSim-JTH/deps/dlp-dataset/

# -----------------------------------------------------------------------------
# 4) colcon build 工作区（15 个 msg + 1 个 srv）
#    注意 1：不 source /opt/ros/*，只用镜像内自带的 foxy 树
#    注意 2：source ROS/ament 的 setup.bash 时**必须关掉 set -u**——
#            ament 的 setup 脚本会引用 AMENT_TRACE_SETUP_FILES 等未定义变量，
#            nounset 会直接让脚本退出（exit 1）。source 完成后再开回 -u。
# -----------------------------------------------------------------------------
RUN bash -c 'set -eo pipefail; \
      set +u; source ${ROS_FOXY_ROOT}/setup.bash; set -u; \
      cd ${PARKSIM_ROOT}/workspace; \
      colcon build --packages-select parksim \
          --cmake-args -DCMAKE_BUILD_TYPE=Release; \
      set +u; source ${PARKSIM_ROOT}/workspace/install/setup.bash; set -u; \
      python3 -c "from parksim.msg import VehicleStateMsg, VehicleInfoMsg; from parksim.srv import OccupancySrv; print(\"[build] OK: parksim.msg/parksim.srv importable\")"'

# -----------------------------------------------------------------------------
# 5) webviz 启动自检所需的 DJI_0012 最小子集
#    server.py 启动时**无条件**执行 resolve_dlp_prefix() -> dlp.dataset.Dataset.load()，
#    而 Dataset.load() 会打开 <prefix>_{frames,agents,instances,obstacles,scene}.json。
#    jth_b1 的静态图层走 layout_rotated.json 分支并提前 return，这些 DJI 文件
#    **内容不被使用**，但文件必须存在（否则 webviz 起不来）。故：
#      · scene / agents / obstacles 用真实文件（合计 120 KB）——scene 用于
#        ds.list_scenes() 取 scene_token，也是 DLP 回退分支的障碍物来源；
#      · frames / instances 是本数据集 269 MB 的两个大件（幽灵层/经验回放才用，
#        jth_b1 的 has_experience=False、且 7.5G 数据集本就不入镜像），
#        用 `{}` 占位，实测冒烟全过（见 README §7）。
# -----------------------------------------------------------------------------
COPY payload/dji_subset/ /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/priorFiles/data/

# -----------------------------------------------------------------------------
# 6) 运行必需资产的冗余兜底 COPY
#    cc5c98f 起，parking_maneuvers.pickle / spots_data.pickle / waypoints_graph.pickle /
#    agents_data_0012.pickle 这 4 个文件已正式入库（原先被根 .gitignore 的 `*.pickle`
#    静默排除，是导致 vehicle_node 启动即 FileNotFoundError 的真凶），因此它们**已经**
#    由第 3b 步的 `COPY payload/parksim/` 带进镜像。
#    这里再 COPY 一次，是刻意的**兜底**：万一将来有人回退到未入库的提交、
#    或在本机把这几个文件删掉导致 git archive 缺失，镜像仍能构建成功并跑通。
#    内容与 archive 一致（同一来源的同一份文件）。
#    vehicle_node.py:74  -> parking_maneuvers.pickle   ← 必需，OfflineManeuver
#                            在 __init__ 里**无条件 open**，缺了 vehicle_node 直接退出。
#    simulator_node.py:83 -> waypoints_graph.pickle    ← graph_cost / nearest_entrance 分配器
#    simulator.yaml       -> spots_data.pickle         ← 默认车位表回退
#    global_params.yaml   -> agents_data_0012.pickle   ← replay 默认 agents
# -----------------------------------------------------------------------------
COPY payload/priorfiles_root/ /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/priorFiles/

# 构建期资产断言：这些文件缺任何一个都会在运行时才炸，这里直接让镜像构建失败。
RUN set -e; \
    PF=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/priorFiles; \
    for f in parking_maneuvers.pickle spots_data.pickle waypoints_graph.pickle agents_data_0012.pickle; do \
        test -f "$PF/$f" || { echo "MISSING required priorFiles asset: $f"; exit 1; }; \
    done; \
    for f in DJI_0012_scene.json DJI_0012_agents.json DJI_0012_obstacles.json DJI_0012_frames.json DJI_0012_instances.json; do \
        test -f "$PF/data/$f" || { echo "MISSING required DLP file: $f"; exit 1; }; \
    done; \
    for f in map.yaml layout_rotated.json spots_data.pickle waypoints_graph.pickle \
             parking_maneuvers_per_spot.pickle base_map_clean.png obstacles.json zone_overview.json; do \
        test -f "$PF/maps/jth_b1/$f" || { echo "MISSING required jth_b1 asset: $f"; exit 1; }; \
    done; \
    python3 -c "import json; p='/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/priorFiles/data/DJI_0012_%s.json'; [json.load(open(p % n)) for n in ('scene','agents','obstacles','frames','instances')]; print('[assets] OK: priorFiles + DJI_0012 subset + jth_b1 assets present, DJI json all valid')"

# -----------------------------------------------------------------------------
# 7) 入口
# -----------------------------------------------------------------------------
COPY run_app.sh /usr/local/bin/run_app.sh
RUN chmod +x /usr/local/bin/run_app.sh

ENV PORT=8099
EXPOSE 8099
WORKDIR /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python

# 日志一律 stdout/stderr -> docker logs
ENTRYPOINT ["/usr/local/bin/run_app.sh"]
