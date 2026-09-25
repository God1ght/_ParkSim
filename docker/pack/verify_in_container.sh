#!/usr/bin/env bash
# =============================================================================
# verify_in_container.sh —— 在**临时** ubuntu:20.04 容器里验证便携封装包
#   仅作验证用，不作为交付形态；跑完会删容器。
#   注意：不映射宿主 8099，避免碰现网服务。
# =============================================================================
set -eo pipefail

DATE="${1:-$(date +%Y%m%d)}"
PKG="parksim-jth-portable-${DATE}"
TARBALL="/media/step/data/ParkSim-JTH-portable/${PKG}.tar.gz"
CNAME="portable-verify"
ROOT="/media/step/data/Yccc7/ParkSim-JTH"

[ -f "${TARBALL}" ] || { echo "FATAL 找不到 ${TARBALL}"; exit 1; }

echo "################ 0) 清理 + 起临时容器 ################"
docker rm -f "${CNAME}" >/dev/null 2>&1 || true
docker run -d --name "${CNAME}" ubuntu:20.04 sleep infinity >/dev/null
docker exec "${CNAME}" bash -c 'cat /etc/os-release | head -2; echo "python3: $(command -v python3 || echo MISSING)"; echo "/opt/ros: $(ls -d /opt/ros 2>/dev/null || echo ABSENT)"'

echo
echo "################ 1) 拷包进容器并解包 ################"
docker cp "${TARBALL}" "${CNAME}:/tmp/pkg.tar.gz"
docker exec "${CNAME}" bash -c 'cd /tmp && tar -xzf pkg.tar.gz && ls -la /tmp/'"${PKG}"

echo
echo "################ 2) 包内容体检（白名单逐项 + 排除项确认没混进来）################"
docker exec "${CNAME}" bash -c "D=/tmp/${PKG}; set -e
echo '--- 必备项 ---'
for p in _ParkSim/python/parksim/webviz/server.py \
         _ParkSim/python/parksim/webviz/static/app.js \
         _ParkSim/workspace/install/setup.bash \
         _ParkSim/workspace/src/parksim/config/global_params.yaml \
         deps/ros/foxy/setup.bash \
         deps/dlp-dataset/dlp/base_map.png \
         requirements-app.txt install.sh run.sh README-portable.md ParkSim.bat ParkSim.html BUILDINFO.txt; do
  if [ -e \"\$D/\$p\" ]; then echo \"OK      \$p\"; else echo \"MISSING \$p\"; fi
done
echo '--- jth_b1 资产 ---'
for p in map.yaml layout_rotated.json spots_data.pickle waypoints_graph.pickle \
         parking_maneuvers_per_spot.pickle base_map_clean.png obstacles.json zone_overview.json; do
  if [ -e \"\$D/_ParkSim/python/parksim/priorFiles/maps/jth_b1/\$p\" ]; then echo \"OK      \$p\"; else echo \"MISSING \$p\"; fi
done
echo '--- priorFiles 根 pickle ---'
for p in parking_maneuvers.pickle spots_data.pickle waypoints_graph.pickle agents_data_0012.pickle; do
  if [ -e \"\$D/_ParkSim/python/parksim/priorFiles/\$p\" ]; then echo \"OK      \$p\"; else echo \"MISSING \$p\"; fi
done
echo '--- DJI_0012 子集 ---'
ls -la \"\$D/_ParkSim/python/parksim/priorFiles/data/\"
echo '--- 排除项（应全部 ABSENT）---'
for p in _ParkSim/carla_PythonAPI _ParkSim/docs \
         _ParkSim/python/parksim/trajectory_predict \
         _ParkSim/python/parksim/priorFiles/_archive_jth_old_20260915 \
         _ParkSim/python/parksim/webviz_backup_20260912_150903 \
         _ParkSim/workspace/build _ParkSim/workspace/log; do
  if [ -e \"\$D/\$p\" ]; then echo \"LEAKED  \$p\"; else echo \"ABSENT  \$p\"; fi
done
echo \"large-data check (priorFiles/data 体积): \$(du -sh \$D/_ParkSim/python/parksim/priorFiles/data | cut -f1)\"
echo \"big-file scan (>20MB, 排除 foxy 的 .so 与自带库):\"
find \$D/_ParkSim \$D/deps/dlp-dataset -type f -size +20M -printf '%s\t%p\n' 2>/dev/null | sort -rn | head -10 || true
echo '--- 逐组体积 ---'
du -sh \$D/_ParkSim \$D/deps/ros/foxy \$D/deps/dlp-dataset \$D 2>/dev/null
echo \"--- webviz_backup / *.bak_ 残留计数 ---\"
find \$D -name '*_bak_*' -o -name '*.bak_*' -o -name 'webviz_backup_*' | wc -l
"

echo
echo "################ 3) install.sh ################"
docker exec "${CNAME}" bash -c "cd /tmp/${PKG} && bash install.sh --no-sudo" 2>&1 | tail -60

echo
echo "################ 4) foxy 自包含性（不依赖 /opt/ros）################"
docker exec "${CNAME}" bash -c "ls -d /opt/ros 2>/dev/null && echo 'FAIL /opt/ros 存在' || echo 'OK /opt/ros 不存在'
bash -c 'set +u; source ${ROOT}/deps/ros/foxy/setup.bash; set -u
echo ROS_DISTRO=\${ROS_DISTRO}
echo AMENT_PREFIX_PATH_0=\$(echo \${AMENT_PREFIX_PATH} | cut -d: -f1)
command -v ros2 && ros2 --version
python3 -c \"import rclpy, std_msgs; print(\\\"rclpy OK\\\", rclpy.__file__)\"'"

echo
echo "################ 5) run.sh 起服务 ################"
docker exec -d "${CNAME}" bash -c "cd ${ROOT} && bash run.sh > /tmp/serve.log 2>&1"
for i in $(seq 1 60); do
  sleep 3
  if docker exec "${CNAME}" bash -c 'curl -s -o /dev/null -w "%{http_code}" http://127.0.0.1:8099/ 2>/dev/null' | grep -q 200; then
    echo "服务就绪（第 ${i} 次探测，约 $((i*3)) 秒）"; break
  fi
done

echo
echo "################ 6) HTTP 验收 ################"
docker exec "${CNAME}" bash -c '
for u in / /static/app.js /base_map.png; do
  code=$(curl -s -o /dev/null -w "%{http_code}" "http://127.0.0.1:8099${u}")
  echo "GET ${u} -> ${code}"
done'

echo
echo "################ 7) 服务端日志关键行 ################"
docker exec "${CNAME}" bash -c 'grep -n "asset check\|serving on\|PARKSIM_ROOT\|listen=" /tmp/serve.log | head -20'

echo
echo "################ 8) WS restart 发车验证 ################"
docker exec "${CNAME}" bash -c "cd ${ROOT}/_ParkSim/docker && timeout 240 python3 ws_probe.py http://127.0.0.1:8099 12345 40" 2>&1 | tail -40

echo
echo "################ 9) 服务端日志尾 ################"
docker exec "${CNAME}" bash -c 'tail -30 /tmp/serve.log'

echo
echo "################ 10) 清理 ################"
docker rm -f "${CNAME}" >/dev/null 2>&1 && echo "已删除容器 ${CNAME}"
docker ps --format '{{.Names}}\t{{.Image}}'
