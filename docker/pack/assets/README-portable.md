# ParkSim-JTH 便携封装包（Ubuntu 20.04 / WSL2 解压即用）

> ### 与 Docker 形态的关系（建议先读这段）
>
> **本包是 Docker 镜像 `parksim-jth:v1` 的同构替代品。** 二者同源：源码范围一致（同一个
> git HEAD `724fd75`）、内置的 ROS 2 Foxy 版本一致、`server.py` 的启动参数一致
> （`--host 0.0.0.0 --port 8099 --control --manage-sim --launch-args map:=jth_b1`）；
> 差别只是把「镜像里的文件系统」换成磁盘上的一棵目录树，由 `install.sh` 铺到
> `/media/step/data/Yccc7/ParkSim-JTH`。
>
> **本包不含任何纯静态离线页面。** `ParkSim.html` 只是 Windows 侧的**双击入口**
> （自探测 + 连不上时的排查指引），它自己不做任何可视化。想看仿真画面、想控制仿真启停，
> 必须有一个**正在运行的服务**：由 `run.sh` 拉起的 webviz，默认监听 `0.0.0.0:8099`。
> 服务没起来，双击 `ParkSim.html` / `ParkSim.bat` 只会拿到排查指引，看不到停车场。
>
> **解压 ≠ 能用。** 解压后还要两步：`install.sh`（建安装根、落盘、装 pip 依赖），
> 然后 `run.sh`（起服务）。**不是**"解压双击就能跑"。

一句话：**拿到包 → 解压 → `install.sh` → `run.sh` → Windows 浏览器开 `http://127.0.0.1:8099/`**。
目标机**不需要**装 ROS、**不需要** `colcon build`、**不需要** 7.5 GB 数据集。

---

## 0. 两种包格式，选一个用

同一份载荷提供两个格式，**任选其一，不要两个都解压到同一目录**：

| 格式 | 在哪用 | 解压命令 |
| --- | --- | --- |
| `parksim-jth-portable-<date>.zip` | **Windows 用户首选**，资源管理器里右键"全部解压缩"即可 | WSL 里手动解压：`unzip parksim-jth-portable-<date>.zip` |
| `parksim-jth-portable-<date>.tar.gz` | Linux / WSL 命令行习惯的用户 | `tar -xzf parksim-jth-portable-<date>.tar.gz` |

> **注意**：精简版 Ubuntu / 全新 WSL 发行版**默认不带 `unzip`**。若提示 `unzip: command not found`，
> 先装：`sudo apt-get install -y unzip`（`tar` 是系统自带的，一般不用装）。

两个格式的**文件内容完全一致**，只有一个差别：`.tar.gz` 里保留了少量指向源码的**符号链接**，
`.zip` 里把它们全部展开成了**真实文件**（Windows 的 ZIP 无法表达符号链接，展开后在任何系统上行为一致）。
两边都**不含**语法外的差异，运行结果相同。

---

## 1. 包里有什么

```
parksim-jth-portable-<date>/
├── _ParkSim/                    源码（git archive HEAD，按白名单剪过）
│   ├── python/parksim/          运行时代码 + priorFiles（jth_b1 地图资产 + 4 个必需 pickle）
│   │   └── webviz/              server.py + static/（可视化服务端与前端）
│   ├── workspace/               src（launch + config）+ install（colcon 产物，11 个 Linux .so）
│   └── docker/                  requirements-app.txt / run_app.sh（原样保留，作参考）
├── deps/
│   ├── ros/foxy/                ★ 预置 ROS 2 Foxy install-tree（278 MB）—— 不用 apt 装 ROS
│   └── dlp-dataset/dlp/         DLP 数据集 Python 包（132 KB，含 /base_map.png 路由依赖的底图）
├── install.sh                   安装：建根 → 落盘 → pip → 自检
├── run.sh                       启动：等效 docker/run_app.sh，绑 0.0.0.0:8099
├── requirements-app.txt         Python 依赖最小集
├── BUILDINFO.txt                构建元信息（git HEAD / 构建时间）
├── README-portable.md           本文件
├── ParkSim.bat                  Windows 一键入口（拉服务 + 轮询 + 开浏览器）
└── ParkSim.html                 Windows 双击入口页（自探测 + 排查指引，不依赖外网）
```

**没有**打进来的东西（省了约 7.6 GB）：

| 排除项 | 原始体积 | 为什么不需要 |
| --- | --- | --- |
| `priorFiles/data/`（DJI 全量数据集） | 7.5 GB | 只保留了 `DJI_0012` 的 3 个真实 json + 2 个 `{}` 占位（见 §6） |
| `python/parksim/trajectory_predict/` | 68 MB CSV | 训练/离线路径，运行链路不 import |
| `carla_PythonAPI/` | 17 MB | 与 CARLA 联合仿真相关，本链路不用 |
| `docs/` | 15 MB | 文档 |
| `workspace/build`、`workspace/log` | 16 MB | colcon 中间产物；`install/` 已足够 |
| `priorFiles/_archive_jth_old_20260915/` | 12 MB | 旧版地图归档 |
| 各种 `*.bak_*` / `webviz_backup_*` / `*.log` / `__pycache__` | ~20 MB | 备份与日志 |

---

## 2. 场景 A：Ubuntu 20.04（物理机 / 虚拟机 / 服务器）

```bash
# 0) 前置：python3 与 pip 会在 install.sh 里自动装；其余什么都不用准备

# 1) 解包
tar -xzf parksim-jth-portable-<date>.tar.gz
cd parksim-jth-portable-<date>

# 2) 安装（会建 /media/step/data/Yccc7/ParkSim-JTH 并铺进去；pip 走清华源）
sudo bash install.sh
#   只想落盘不装依赖：bash install.sh --no-apt --no-pip
#   换 pip 源：PIP_INDEX=https://pypi.org/simple bash install.sh

# 3) 启动
bash /media/step/data/Yccc7/ParkSim-JTH/run.sh
#   看到 serving on http://0.0.0.0:8099/ 就是好了

# 4) 浏览器打开
#    本机：http://127.0.0.1:8099/
#    别的机器：http://<本机IP>:8099/   （服务绑的是 0.0.0.0）
```

停止服务：`Ctrl-C`（`--manage-sim` 模式下桥退出会一并停仿真）。

## 3. 场景 B：Windows + WSL2（推荐给 Windows 用户）

### 3.1 装 WSL2（只做一次）

管理员 PowerShell：

```powershell
wsl --install -d Ubuntu-20.04
```

装完**重启电脑**。必须是 **Ubuntu 20.04**——包里预置的 ROS 2 Foxy 与 colcon 产物是按
`python3.8` + `libssl1.1` 构建的，22.04/24.04 会缺运行库。

### 3.2 在 WSL 里安装

把 tar.gz 放到 Windows 的某个目录（比如 `C:\Users\<你>\Downloads\`），然后在 Ubuntu 窗口里：

```bash
cd /mnt/c/Users/<你>/Downloads
tar -xzf parksim-jth-portable-<date>.tar.gz
cd parksim-jth-portable-<date>
sudo bash install.sh
```

`install.sh` 会在 WSL 里建 `/media/step/data/Yccc7/ParkSim-JTH` 并铺进去。
**这个路径是刻意的**（见 §5 第 1 条），在 WSL2 里建目录成本为零。

### 3.3 启动 + 开页面（三选一）

| 方式 | 操作 |
| --- | --- |
| **双击 `ParkSim.html`** | 页面自己探测 `http://127.0.0.1:8099/`，就绪自动跳转；没就绪给排查指引。**推荐先看这个** |
| **双击 `ParkSim.bat`** | 检测 WSL → 拉起 `run.sh`（另开窗口显示日志）→ 轮询到 200 → 自动开浏览器 |
| 手动 | Ubuntu 窗口里 `bash /media/step/data/Yccc7/ParkSim-JTH/run.sh`，然后浏览器开 `http://127.0.0.1:8099/` |

> `ParkSim.html` 与 `ParkSim.bat` 都不依赖任何外网 / CDN，离线可用。

---

## 4. 怎么确认真的成功了

```bash
# 1) 首页 200
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8099/
# → 200

# 2) 静态资源与底图 200
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8099/static/app.js
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8099/base_map.png
# → 200 / 200

# 3) 服务端日志里应当出现这两行
#    [webviz] asset check: OK（map=jth_b1，资产根=/…/python/parksim）
#    serving on http://0.0.0.0:8099/

# 4) 打开页面后点「开启仿真」发车；车辆数从 0 变正即全链路通
```

页面是**停在「已停止」态**的（符合「打开页面按键控制」的需求），
由用户在网页上点「开启仿真」发车，而不是起来就跑。
想开机即发车：`PARKSIM_AUTOSTART=1 bash run.sh`。

---

## 5. 已知坑（实测，按踩到的概率排序）

1. **安装根必须是 `/media/step/data/Yccc7/ParkSim-JTH`**
   包里预置的 ROS 2 Foxy 树（`deps/ros/foxy`）的 `setup.bash` / ament index **内含编译期
   的绝对路径**。放对路径 = 100% 可用；放错路径 = ament 找不到包。
   `install.sh` 支持 `--root <其他路径>` 并会做一次文本改写，但那是 **best-effort**，
   出问题请换回默认根。默认根在 WSL2 / Ubuntu 里就是建个目录，成本为零。

2. **Ubuntu 20.04 / ROS 2 Foxy 都已 EOL —— 不要 `apt install ros-foxy-*`**
   现在大概率拉不到包。这正是我们**直接预置整棵 foxy 树**的原因：解压即用，不联网装 ROS。
   `apt` 只用来装 `python3 / python3-pip / build-essential / libssl-dev / libtinyxml2-dev /
   procps / psmisc`。若 `apt-get update` 失败（源失效），把 `/etc/apt/sources.list` 里的
   `archive.ubuntu.com` 换成 `old-releases.ubuntu.com` 再试。

3. **WSL2 的 `localhostForwarding` 默认开启，但快速启动 / 休眠之后可能失效**
   症状：服务明明在跑、日志正常，Windows 侧就是连不上。
   处理：Windows PowerShell 里 `wsl --shutdown`，然后重新起服务。
   根治：在 Windows 的「电源选项」里关掉「启用快速启动」。

4. **IPv4 / IPv6 不一致时用 `http://127.0.0.1:8099` 强制 IPv4**
   `localhost` 可能被解析到 `::1`，而服务只监听了 IPv4 的 `0.0.0.0`。

5. **服务必须绑 `0.0.0.0`，不能绑 `127.0.0.1`**
   WSL2 是 NAT 模式，绑 `127.0.0.1` 时 Windows 侧访问不到。`run.sh` 已经写死 `--host 0.0.0.0`，
   **不要改**。

6. **Win11 22H2+ 的 Hyper-V 虚拟防火墙可能拦 WSL 的入站**
   管理员 PowerShell：
   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   ```
   或在 `C:\Users\<你>\.wslconfig` 里写：
   ```ini
   [wsl2]
   firewall=false
   ```
   然后 `wsl --shutdown` 生效。

7. **兜底：直接用 WSL 的 IP**
   ```bash
   ip -4 addr show eth0      # 或 hostname -I
   ```
   拿到例如 `172.20.10.5`，浏览器开 `http://172.20.10.5:8099/`。
   `ParkSim.html` 里也有输入框可以试这个地址。

8. **建议显式 `export MPLBACKEND=Agg`**
   运行链路会 `import matplotlib.pyplot`（不会真的弹窗），但目标机若没装 GUI 后端库，
   没有 Agg 时会在 import 阶段报后端相关错误。`run.sh` 已经默认设成 `Agg`。

9. **不要在目标机上 `colcon build`**
   包里带的是**构建好的** `workspace/install`（含 11 个 Linux `.so`，cp38）。
   这些 `.so` 只能在 Linux / WSL2 里用，**不能**用于原生 Windows。
   需要重新构建消息包才要 build，届时请回构建机。

10. **python 必须是 3.8**
    `.so` 是 `cpython-38` 的。Ubuntu 20.04 自带 python3.8，正好；
    其它版本会 `ImportError: ... cpython-38 ... cannot open shared object file`。

11. **同机跑多个实例会互相串扰 DDS**
    `run.sh` 默认 `ROS_LOCALHOST_ONLY=1`（DDS 只走本机 lo）。
    若你把它改成 0 且与别的 ROS 2 机器同网段，可能发现到外来 vehicle 节点导致启动即退出。

---

## 6. 两个「看起来奇怪但是故意的」设计

### 6.1 `priorFiles/data/DJI_0012_frames.json` / `_instances.json` 是 `{}`

`webviz/server.py` 启动时**无条件**执行
`resolve_dlp_prefix() → dlp.dataset.Dataset.load()`，而 `Dataset.load()` 会去打开
`<prefix>_{frames,agents,instances,obstacles,scene}.json`。

但 `jth_b1` 的静态图层走 `layout_rotated.json` 分支并提前 return，这些 DJI 文件
**内容根本不被使用**，只是**文件必须存在**（缺了 webviz 起不来）。而这两个文件在原数据集里
是 **26 MB + 242 MB**。所以：

* `scene` / `agents` / `obstacles` —— 真实文件（合计 120 KB，`scene` 用于 `ds.list_scenes()`）
* `frames` / `instances` —— `{}` 占位

如果你确实要跑「幽灵层 / 经验回放」，把原数据集里这两个真文件拷回去覆盖即可。

### 6.2 `deps/ros/foxy` 有 278 MB，为什么不裁

它的 `setup.bash` / ament index / cmake 导出 target 互相引用，裁剪 demo 包与 `include/`
理论可行但风险不低（例如 fastrtps 的导出 target 里写死了
`/usr/lib/x86_64-linux-gnu/libtinyxml2.so` 的绝对路径）。
278 MB 换「绝对能跑起来」是划算的，故**整棵原样带**。

---

## 7. 目录约定（装完之后）

```
/media/step/data/Yccc7/ParkSim-JTH/          ← PARKSIM_ENV_ROOT（安装根）
├── _ParkSim/                                ← PARKSIM_ROOT
│   ├── python/                              ← server.py 的工作目录
│   └── workspace/install/                   ← colcon 产物
├── deps/ros/foxy/                           ← ROS_FOXY_ROOT（≠ /opt/ros）
├── deps/dlp-dataset/                        ← DLP_DATASET_ROOT
├── run.sh
├── requirements-app.txt
└── README-portable.md
```

`run.sh` 会用**自己所在目录**推导 `PARKSIM_ENV_ROOT`（该目录下有 `_ParkSim` 就用它，
否则回退默认根），所以即使你把整个目录搬到别处，也能配合 `install.sh --root` 的改写工作。

---

## 8. 卸载

```bash
# 停服务（Ctrl-C 或 kill 掉 run.sh 起的 python3）
sudo rm -rf /media/step/data/Yccc7/ParkSim-JTH
```

`install.sh` 遇到已存在的 `_ParkSim` / `deps` 会先改名成 `*.bak_<时间戳>`，不会静默覆盖。

---

## 9. 环境变量速查

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `PARKSIM_ENV_ROOT` | run.sh 所在目录 / 默认根 | 环境根 |
| `PORT` | `8099` | 监听端口 |
| `PARKSIM_MAP` | `jth_b1` | 地图名 |
| `PARKSIM_AUTOSTART` | `0` | `1` 时开机即发车 |
| `ROS_DOMAIN_ID` | `0` | DDS 域 |
| `ROS_LOCALHOST_ONLY` | `1` | DDS 只走 lo，防串扰 |
| `MPLBACKEND` | `Agg` | matplotlib 无 GUI 后端 |
| `PIP_INDEX` | 清华源 | 仅 `install.sh` 用 |
| `PARKSIM_INSTALL_ROOT` | 默认根 | 仅 `install.sh` 用（等价 `--root`） |
