/* ParkSim webviz v3 — 双主题 / 平滑插值 / 路径图层 / 组件化 UI
 * 数据协议（与 server.py 配合）：
 *   init  : {type:'init', map_size, scale, spots, waypoints, obstacles, colors, options}
 *   frame : {type:'frame', seq, t, vehicles:[{id,x,y,psi,c,label,v}], ghosts, fpath, running}
 *   ws 上行: {type:'pause', value} | {type:'ping', t} | {type:'focus', id}
 */
(() => {
'use strict';

/* ================= 主题调色板（与 style.css 变量保持同步） ================= */
const THEMES = {
  dark: {
    canvasBg: '#0d1219',
    grid: 'rgba(130,160,200,0.07)',
    gridText: 'rgba(140,170,205,0.45)',
    rowFill: 'rgba(22,30,41,0.5)',
    rowStroke: 'rgba(120,155,195,0.12)',
    slotLine: 'rgba(140,185,235,0.30)',
    waypoint: 'rgba(140,180,225,0.40)',
    obstacle: 'rgba(58,74,95,0.88)',
    obstacleStroke: 'rgba(122,156,196,0.25)',
    departing: 'rgba(224,168,32,0.85)',
    departingStroke: 'rgba(255,214,102,0.95)',
    rampFill: { entry: 'rgba(46,208,126,0.13)', exit: 'rgba(235,150,60,0.13)',
                layer_down: 'rgba(70,140,220,0.13)', layer_up: 'rgba(170,110,230,0.13)' },
    rampStroke: { entry: 'rgba(46,208,126,0.55)', exit: 'rgba(235,150,60,0.55)',
                  layer_down: 'rgba(90,160,240,0.55)', layer_up: 'rgba(185,130,240,0.55)' },
    rampHatch: 'rgba(150,200,255,0.22)',
    rampArrow: 'rgba(110,190,255,0.85)',
    rampLabel: 'rgba(205,228,248,0.92)',
    entranceFill: 'rgba(46,208,126,0.16)',
    entranceStroke: 'rgba(46,208,126,0.75)',
    exitFill: 'rgba(235,90,60,0.16)',
    exitStroke: 'rgba(235,110,80,0.80)',
    gateLabel: 'rgba(225,240,252,0.95)',
    gateLabelBg: 'rgba(13,18,25,0.72)',
    ghost: 'rgba(165,198,232,0.13)',
    ghostStroke: 'rgba(165,198,232,0.10)',
    veh: { driving:'#38d6c0', parking:'#f5b04d', braking:'#e06060', done:'rgba(112,126,144,0.8)' },
    vehStroke: 'rgba(0,0,0,0.55)',
    cabin: 'rgba(9,18,28,0.55)',
    wedge:  'rgba(255,255,255,0.55)',
    label: 'rgba(214,230,246,0.95)',
    path: 'rgba(76,194,255,0.9)',
    pathGlow: 'rgba(76,194,255,0.16)',
    trail: '76,194,255',
    halo: 'rgba(76,194,255,0.30)',
    scale: 'rgba(205,222,240,0.65)',
    badgeBg: 'rgba(13,18,25,0.85)',
    spotReserved: '#2fd07e',
  },
  light: {
    canvasBg: '#f6f3ed',
    grid: 'rgba(40,60,80,0.06)',
    gridText: 'rgba(50,66,82,0.5)',
    rowFill: 'rgba(234,230,222,0.6)',
    rowStroke: 'rgba(70,90,110,0.10)',
    slotLine: 'rgba(255,255,255,0.96)',
    waypoint: 'rgba(70,90,110,0.42)',
    obstacle: 'rgba(196,208,220,0.92)',
    obstacleStroke: 'rgba(90,110,135,0.28)',
    departing: 'rgba(214,152,26,0.80)',
    departingStroke: 'rgba(146,96,8,0.95)',
    rampFill: { entry: 'rgba(38,166,102,0.12)', exit: 'rgba(214,130,40,0.12)',
                layer_down: 'rgba(52,116,190,0.12)', layer_up: 'rgba(140,86,200,0.12)' },
    rampStroke: { entry: 'rgba(38,166,102,0.6)', exit: 'rgba(214,130,40,0.6)',
                  layer_down: 'rgba(52,116,190,0.6)', layer_up: 'rgba(140,86,200,0.6)' },
    rampHatch: 'rgba(70,100,130,0.25)',
    rampArrow: 'rgba(52,116,190,0.85)',
    rampLabel: 'rgba(50,66,82,0.92)',
    entranceFill: 'rgba(38,166,102,0.14)',
    entranceStroke: 'rgba(22,118,70,0.75)',
    exitFill: 'rgba(214,80,50,0.14)',
    exitStroke: 'rgba(168,54,28,0.80)',
    gateLabel: 'rgba(45,58,72,0.95)',
    gateLabelBg: 'rgba(246,243,237,0.78)',
    ghost: 'rgba(45,70,95,0.11)',
    ghostStroke: 'rgba(45,70,95,0.08)',
    veh: { driving:'#2f9e6e', parking:'#e08a2c', braking:'#cf4b4b', done:'rgba(138,146,158,0.85)' },
    vehStroke: 'rgba(255,255,255,0.85)',
    cabin: 'rgba(255,255,255,0.40)',
    wedge:  'rgba(255,255,255,0.9)',
    label: 'rgba(45,60,75,0.92)',
    path: 'rgba(38,132,150,0.9)',
    pathGlow: 'rgba(38,132,150,0.15)',
    trail: '38,132,150',
    halo: 'rgba(47,158,110,0.28)',
    scale: 'rgba(55,75,95,0.6)',
    badgeBg: 'rgba(255,255,255,0.9)',
    spotReserved: '#1f9e4f',
  },
};

const STATE_NAMES = ['行驶中', '泊车机动', '制动等待', '已完成'];

/* ================= 全局状态 ================= */
const S = {
  init: null,
  frame: null,
  prevFrame: null,
  frameWallT: 0,
  ws: null,
  connected: false,
  control: false,
  paused: false,
  latency: null,
  follow: 0,
  focusSent: -1,
  view: { zoom: 1, x: 0, y: 0 },
  dragging: false,
  dragMoved: false,
  lastMouse: null,
  downMouse: null,
  mouse: { x: 0, y: 0 },
  layers: null,
  theme: 'dark',
  trailsMap: new Map(),
  fps: 0, _fpsCount: 0, _fpsT: performance.now(), _chipTick: 0,
  mapW: 140, mapH: 80, scale: 10,
  dpr: 1,
  hover: 0,
  presentation: false,
  occupancy: null,
  departing: null,
  obstacleMode: 'dataset',
  sysBannerKind: null,
};

const P = { rows: null, slots: null, obstacles: null, waypoints: null };

/* ================= DOM ================= */
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const bgImg = document.getElementById('bg');
const btnPause = document.getElementById('btnPause');
const btnReset = document.getElementById('btnReset');
const btnTheme = document.getElementById('btnTheme');
const btnLayers = document.getElementById('btnLayers');
const layersEl = document.getElementById('layers');
const btnScheme = document.getElementById('btnScheme');
const btnMap = document.getElementById('btnMap');
const mapPanelEl = document.getElementById('mapPanel');
const btnLoadMap = document.getElementById('btnLoadMap');
const schemeEl = document.getElementById('scheme');
const scMap = document.getElementById('scMap');
const scInitMode = document.getElementById('scInitMode');
const scCustomFile = document.getElementById('scCustomFile');
const secCustom = document.getElementById('secCustom');
const scAlloc = document.getElementById('scAlloc');
const scRoute = document.getElementById('scRoute');
const scRef = document.getElementById('scRef');
const scManeuver = document.getElementById('scManeuver');
const btnApplyScheme = document.getElementById('btnApplyScheme');
const schemeStatus = document.getElementById('schemeStatus');
const sysBanner = document.getElementById('sysBanner');
const sysBannerText = document.getElementById('sysBannerText');
const btnSysRestart = document.getElementById('btnSysRestart');
const levelIndicatorEl = document.getElementById('levelIndicator');
const secRandom = document.getElementById('secRandom');
const secReplay = document.getElementById('secReplay');
const scEntering = document.getElementById('scEntering');
const scExiting = document.getElementById('scExiting');
const scInterval = document.getElementById('scInterval');
const scSeed = document.getElementById('scSeed');
const scYBound = document.getElementById('scYBound');
const scBlocked = document.getElementById('scBlocked');
const scOccupied = document.getElementById('scOccupied');
const scOccRandom = document.getElementById('scOccRandom');
const scOccCount = document.getElementById('scOccCount');
const scTimeScale = document.getElementById('scTimeScale');
const scMaxAgents = document.getElementById('scMaxAgents');
const tooltipEl = document.getElementById('tooltip');
const msgEl = document.getElementById('msg');
const hintEl = document.getElementById('hint');
const chipConn = document.getElementById('chipConn');
const vTime = document.getElementById('vTime');
const vVeh = document.getElementById('vVeh');
const vFps = document.getElementById('vFps');
const vLat = document.getElementById('vLat');
const vConn = document.getElementById('vConn');

let W = 0, H = 0;

/* ================= 主题 ================= */
function initTheme() {
  const q = new URLSearchParams(location.search).get('theme');
  let t = q === 'light' ? 'light' : q === 'dark' ? 'dark' : null;
  if (!t) { try { t = localStorage.getItem('parksim_theme'); } catch (e) {} }
  if (t !== 'light' && t !== 'dark') t = 'dark';
  setTheme(t, false);
}
function setTheme(t, persist = true) {
  S.theme = t;
  document.body.dataset.theme = t;
  if (persist) { try { localStorage.setItem('parksim_theme', t); } catch (e) {} }
}
function cycleTheme() { setTheme(S.theme === 'dark' ? 'light' : 'dark'); }

/* ================= 图层 ================= */
function defaultLayers() {
  return {
    baseMap: S.theme === 'light', grid: false, spots: true, waypoints: false,
    obstacles: false, ghosts: true, paths: true, trails: false, labels: true,
    ramps: true, entranceExit: true,
  };
}
function initLayers() {
  try {
    const raw = localStorage.getItem('parksim_layers_v5');
    if (raw) S.layers = Object.assign(defaultLayers(), JSON.parse(raw));
  } catch (e) {}
  if (!S.layers) S.layers = defaultLayers();
  document.querySelectorAll('#layers input').forEach(cb => {
    cb.checked = !!S.layers[cb.dataset.layer];
    cb.addEventListener('change', () => {
      S.layers[cb.dataset.layer] = cb.checked;
      try { localStorage.setItem('parksim_layers_v5', JSON.stringify(S.layers)); } catch (e) {}
    });
  });
}

/* ================= 画布 / 坐标 ================= */
function resize() {
  W = cv.clientWidth; H = cv.clientHeight;
  S.dpr = window.devicePixelRatio || 1;
  cv.width = Math.round(W * S.dpr);
  cv.height = Math.round(H * S.dpr);
}
window.addEventListener('resize', resize);

function w2s(x, y) {
  const v = S.view;
  return [x * S.scale * v.zoom + v.x, (S.mapH - y) * S.scale * v.zoom + v.y];
}
function s2w(sx, sy) {
  const v = S.view;
  return [(sx - v.x) / (S.scale * v.zoom), S.mapH - (sy - v.y) / (S.scale * v.zoom)];
}
function setScreen() { ctx.setTransform(S.dpr, 0, 0, S.dpr, 0, 0); }
function setWorld() {
  const v = S.view, k = S.dpr, a = k * S.scale * v.zoom;
  ctx.setTransform(a, 0, 0, -a, k * v.x, k * ((S.mapH * S.scale * v.zoom) + v.y));
}
function s2wEl(e) {
  const r = cv.getBoundingClientRect();
  return s2w(e.clientX - r.left, e.clientY - r.top);
}

/* ================= 静态图层构建（Path2D，世界坐标） ================= */
function rrectTo(p, x, y, w, h, r) {
  r = Math.min(r, Math.abs(w) / 2, Math.abs(h) / 2);
  p.moveTo(x + r, y);
  p.arcTo(x + w, y, x + w, y + h, r);
  p.arcTo(x + w, y + h, x, y + h, r);
  p.arcTo(x, y + h, x, y, r);
  p.arcTo(x, y, x + w, y, r);
  p.closePath();
}
function rrectCtx(x, y, w, h, r) {
  ctx.beginPath();
  rrectTo(ctx, x, y, w, h, r);
}

function computeRows(spots) {
  const cs = spots.map(sp => {
    const xs = [sp[0], sp[2], sp[4], sp[6]], ys = [sp[1], sp[3], sp[5], sp[7]];
    return {
      x: xs.reduce((a, b) => a + b, 0) / 4, y: ys.reduce((a, b) => a + b, 0) / 4,
      x0: Math.min.apply(null, xs), x1: Math.max.apply(null, xs),
      y0: Math.min.apply(null, ys), y1: Math.max.apply(null, ys),
    };
  }).sort((a, b) => a.y - b.y);
  const yClusters = []; let cur = [];
  for (const c of cs) {
    if (cur.length && c.y - cur[cur.length - 1].y > 3.0) { yClusters.push(cur); cur = []; }
    cur.push(c);
  }
  if (cur.length) yClusters.push(cur);
  const rows = [];
  for (const cl of yClusters) {
    cl.sort((a, b) => a.x - b.x);
    let group = [];
    const flush = () => {
      if (group.length >= 4) {
        const x0 = Math.min.apply(null, group.map(c => c.x0)) - 0.8;
        const x1 = Math.max.apply(null, group.map(c => c.x1)) + 0.8;
        const y0 = Math.min.apply(null, group.map(c => c.y0)) - 0.7;
        const y1 = Math.max.apply(null, group.map(c => c.y1)) + 0.7;
        rows.push({ x: x0, y: y0, w: x1 - x0, h: y1 - y0 });
      }
      group = [];
    };
    for (const c of cl) {
      if (group.length && c.x0 - group[group.length - 1].x1 > 9) flush();
      group.push(c);
    }
    flush();
  }
  return rows;
}

function buildStatic() {
  const init = S.init;
  if (!init) return;
  P.rows = new Path2D();
  for (const r of computeRows(init.spots)) rrectTo(P.rows, r.x, r.y, r.w, r.h, 1.6);
  P.slots = new Path2D();
  for (const sp of init.spots) {
    P.slots.moveTo(sp[0], sp[1]);
    P.slots.lineTo(sp[2], sp[3]);
    P.slots.lineTo(sp[4], sp[5]);
    P.slots.lineTo(sp[6], sp[7]);
    P.slots.closePath();
  }
  P.obstacles = new Path2D();
  for (const ob of init.obstacles) {
    P.obstacles.moveTo(ob[0][0], ob[0][1]);
    for (let i = 1; i < ob.length; i++) P.obstacles.lineTo(ob[i][0], ob[i][1]);
    P.obstacles.closePath();
  }
  P.entrance = init.entrance || null;
  P.exit = init.exit || null;
  P.waypoints = new Path2D();
  for (const key in init.waypoints) {
    for (const p of init.waypoints[key]) {
      P.waypoints.moveTo(p[0] + 0.22, p[1]);
      P.waypoints.arc(p[0], p[1], 0.22, 0, Math.PI * 2);
    }
  }
  // 坡道（上下坡关系）：区域按类型分色 + 坡度线纹理 + 方向箭头
  P.rampsK = { entry: new Path2D(), exit: new Path2D(), layer_down: new Path2D(), layer_up: new Path2D() };
  P.rampHatch = new Path2D();
  P.rampArrows = new Path2D();
  for (const r of (init.ramps || [])) {
    const poly = r.poly || [];
    if (poly.length < 3) continue;
    const g = P.rampsK[r.kind] || P.rampsK.layer_down;
    g.moveTo(poly[0][0], poly[0][1]);
    for (let i = 1; i < poly.length; i++) g.lineTo(poly[i][0], poly[i][1]);
    g.closePath();
    const xs = poly.map(p => p[0]), ys = poly.map(p => p[1]);
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    // 坡度线纹理（与坡道长轴垂直的等距短线）
    if ((x1 - x0) >= (y1 - y0)) {
      const n = Math.max(3, Math.round((x1 - x0) / 2.5));
      for (let k = 1; k < n; k++) {
        const x = x0 + (x1 - x0) * k / n;
        P.rampHatch.moveTo(x, y0 + 0.6); P.rampHatch.lineTo(x, y1 - 0.6);
      }
    } else {
      const n = Math.max(3, Math.round((y1 - y0) / 2.5));
      for (let k = 1; k < n; k++) {
        const y = y0 + (y1 - y0) * k / n;
        P.rampHatch.moveTo(x0 + 0.6, y); P.rampHatch.lineTo(x1 - 0.6, y);
      }
    }
    // 方向箭头（down=下坡朝南/-y，up=上坡朝北/+y）
    const d = 2.0;
    if (r.dir === 'down') {
      P.rampArrows.moveTo(cx, cy - d); P.rampArrows.lineTo(cx, cy + d);
      P.rampArrows.moveTo(cx - 0.9, cy + d - 0.9); P.rampArrows.lineTo(cx, cy + d);
      P.rampArrows.lineTo(cx + 0.9, cy + d - 0.9);
    } else {
      P.rampArrows.moveTo(cx, cy + d); P.rampArrows.lineTo(cx, cy - d);
      P.rampArrows.moveTo(cx - 0.9, cy - d + 0.9); P.rampArrows.lineTo(cx, cy - d);
      P.rampArrows.lineTo(cx + 0.9, cy - d + 0.9);
    }
  }
  buildLevelIndicator();
}

/* ================= WebSocket ================= */
function setMsg(t) { msgEl.textContent = t || ''; msgEl.classList.toggle('show', !!t); }

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(proto + '://' + location.host + '/ws');
  S.ws = ws;
  ws.onopen = () => {
    S.connected = true;
    if (S.init) setMsg(S.control ? '' : '桥为只读模式（未启用控制，暂停按钮不可用）');
  };
  ws.onclose = () => {
    S.connected = false;
    S.focusSent = -1;
    if (S.init === null) setMsg('正在连接…');
    setTimeout(connect, 1500);
  };
  ws.onerror = () => { try { ws.close(); } catch (e) {} };
  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch (e) { return; }
    if (m.type === 'init') {
      S.init = m;
      S.scale = m.scale || 10;
      S.mapW = m.map_size.x; S.mapH = m.map_size.y;
      S.options = m.options || {};
      S.control = !!(m.options && m.options.control);
      S.obstacleMode = (m.options && m.options.obstacle_mode) || 'dataset';
      S.occupancy = null;
      hideSysBanner();
      btnPause.disabled = !S.control;
      setMsg(S.control ? '' : '桥为只读模式（未启用控制，暂停按钮不可用）');
      if (m.options && m.options.base_map_url) bgImg.src = m.options.base_map_url;
      S.bgT = m.base_map_transform || null;
      S.rowFill = (m.row_fill !== false);   // 地图包（JTH）默认不画车位行底色
      initSchemePanel();
      buildStatic();
      fitView();
    } else if (m.type === 'frame') {
      S.prevFrame = S.frame;
      S.frame = m;
      S.frameWallT = performance.now();
      S.paused = !m.running;
      updatePauseLabel();
      updateTrails();
    } else if (m.type === 'occupancy') {
      S.occupancy = m.data || null;
    } else if (m.type === 'departing') {
      S.departing = m.data || null;
    } else if (m.type === 'paused') {
      S.paused = !!m.value;
      updatePauseLabel();
    } else if (m.type === 'status') {
      onSchemeStatus(m);
    } else if (m.type === 'pong') {
      S.latency = Math.round(performance.now() - m.t);
    }
  };
}

function sendFocus(id) {
  if (S.focusSent === id) return;
  S.focusSent = id;
  if (S.ws && S.ws.readyState === 1) {
    S.ws.send(JSON.stringify({ type: 'focus', id: id || 0 }));
  }
}

/* ================= 方案面板（换方案并重启仿真） ================= */
const SCHEME_LABELS = {
  init_mode: {
    'replay': '经验初始化',
    'random': '随机生成',
    'custom': '自定义时间表',
  },
  allocation_method: {
    'random': '随机分配（默认）',
    'nearest_entrance': '最近入口优先',
    'graph_cost': '图距离最短',
    'balanced_rows': '按排均衡',
    'manual': '手动序列',
  },
  route_planner: {
    'astar': 'A*（默认）',
    'dijkstra': '迪杰斯特拉（对照）',
    'via': '途经点串联',
  },
  ref_path_generator: {
    'spline': '样条曲线（默认）',
    'linear': '折线插值',
  },
  maneuver_provider: {
    'offline': '离线机动库（默认）',
    'online_rs': '在线几何生成（实验）',
  },
};

function schemeLabel(kind, value) {
  const map = SCHEME_LABELS[kind] || {};
  if (Object.prototype.hasOwnProperty.call(map, value)) return map[value];
  return value || '--';
}

function spotQuadPath(sp, k) {
  const cx = (sp[0] + sp[2] + sp[4] + sp[6]) / 4;
  const cy = (sp[1] + sp[3] + sp[5] + sp[7]) / 4;
  ctx.beginPath();
  ctx.moveTo(sp[0] + (cx - sp[0]) * k, sp[1] + (cy - sp[1]) * k);
  ctx.lineTo(sp[2] + (cx - sp[2]) * k, sp[3] + (cy - sp[3]) * k);
  ctx.lineTo(sp[4] + (cx - sp[4]) * k, sp[5] + (cy - sp[5]) * k);
  ctx.lineTo(sp[6] + (cx - sp[6]) * k, sp[7] + (cy - sp[7]) * k);
  ctx.closePath();
}

function drawOccupancyCars(th, v) {
  const occ = S.occupancy;
  const spots = S.init && S.init.spots;
  if (!occ || !spots) return;
  const depSet = new Set((S.departing || []).map(String));
  ctx.fillStyle = th.obstacle;
  ctx.strokeStyle = th.obstacleStroke;
  ctx.lineWidth = 0.8 / (S.scale * v.zoom);
  const k = 0.16;
  for (let i = 0; i < occ.length && i < spots.length; i++) {
    if (!occ[i]) continue;
    if (depSet.has(String(i))) continue;   // 即将驶出：由 departing 绘制黄色标记
    const sp = spots[i];
    spotQuadPath(sp, k);
    ctx.fill();
    ctx.stroke();
  }
}

function drawDepartingSpots(th, v) {
  const dep = S.departing;
  const spots = S.init && S.init.spots;
  if (!dep || !dep.length || !spots) return;
  ctx.fillStyle = th.departing;
  ctx.strokeStyle = th.departingStroke;
  ctx.lineWidth = 0.9 / (S.scale * v.zoom);
  const k = 0.16;
  for (let q = 0; q < dep.length; q++) {
    const i = dep[q];
    if (i < 0 || i >= spots.length) continue;
    spotQuadPath(spots[i], k);
    ctx.fill();
    ctx.stroke();
  }
}

// 入口/出口区域可视化（地图规则数据：点位 + 朝向）
function drawEntranceExit(th, v) {
  const k = S.scale * v.zoom;
  const drawOne = (pt, kind) => {
    if (!pt || pt.x == null) return;
    const isIn = kind === 'in';
    const R = 4.0;                      // 区域半边长（8m 区域）
    const xs = pt.x - R, ys = pt.y - R, w = R * 2, h = R * 2;
    ctx.save();
    // 区域（圆角矩形）
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(xs, ys, w, h, 1.2);
    else ctx.rect(xs, ys, w, h);
    ctx.fillStyle = isIn ? th.entranceFill : th.exitFill;
    ctx.fill();
    ctx.strokeStyle = isIn ? th.entranceStroke : th.exitStroke;
    ctx.lineWidth = 1.4 / k;
    ctx.stroke();
    // 方向箭头（有 heading 时：从区域中心沿朝向）
    if (pt.heading != null) {
      const L = 5.5;
      const ex = pt.x + Math.cos(pt.heading) * L;
      const ey = pt.y + Math.sin(pt.heading) * L;
      const bx = pt.x - Math.cos(pt.heading) * L * 0.5;
      const by = pt.y - Math.sin(pt.heading) * L * 0.5;
      ctx.beginPath();
      ctx.moveTo(bx, by);
      ctx.lineTo(ex, ey);
      ctx.strokeStyle = isIn ? th.entranceStroke : th.exitStroke;
      ctx.lineWidth = 2.0 / k;
      ctx.stroke();
      // 箭头头部
      const a = pt.heading;
      const hl = 1.6;
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex - Math.cos(a - 0.45) * hl, ey - Math.sin(a - 0.45) * hl);
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex - Math.cos(a + 0.45) * hl, ey - Math.sin(a + 0.45) * hl);
      ctx.lineWidth = 2.0 / k;
      ctx.stroke();
    }
    // 标签（胶囊底 + 文字）
    const label = isIn ? '入口' : '出口';
    ctx.font = (11 / k) + 'px system-ui, -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const tw = ctx.measureText(label).width;
    const ly = pt.y - R - 1.8;
    ctx.fillStyle = th.gateLabelBg;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(pt.x - tw / 2 - 0.9, ly - 1.05, tw + 1.8, 2.1, 0.7);
    else ctx.rect(pt.x - tw / 2 - 0.9, ly - 1.05, tw + 1.8, 2.1);
    ctx.fill();
    ctx.fillStyle = th.gateLabel;
    ctx.fillText(label, pt.x, ly + 0.05);
    ctx.restore();
  };
  drawOne(P.entrance, 'in');
  drawOne(P.exit, 'out');
}

function drawRamps(th, v) {
  if (!S.init || !(S.init.ramps && S.init.ramps.length)) return;
  const k = S.scale * v.zoom;
  for (const kind in P.rampsK) {
    const g = P.rampsK[kind];
    if (!g) continue;
    ctx.fillStyle = th.rampFill[kind] || th.rampFill.default;
    ctx.fill(g);
    ctx.strokeStyle = th.rampStroke[kind] || th.rampStroke.default;
    ctx.lineWidth = 0.8 / k;
    ctx.stroke(g);
  }
  if (P.rampHatch) {
    ctx.strokeStyle = th.rampHatch;
    ctx.lineWidth = 0.5 / k;
    ctx.stroke(P.rampHatch);
  }
  if (P.rampArrows) {
    ctx.strokeStyle = th.rampArrow;
    ctx.lineWidth = 1.0 / k;
    ctx.stroke(P.rampArrows);
  }
  ctx.font = (10 / k) + 'px system-ui, -apple-system, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillStyle = th.rampLabel;
  for (const r of S.init.ramps) {
    const xs = r.poly.map(p => p[0]), ys = r.poly.map(p => p[1]);
    const cx = (Math.min(...xs) + Math.max(...xs)) / 2;
    const cy = (Math.min(...ys) + Math.max(...ys)) / 2;
    ctx.fillText(r.label || '', cx, cy - 2.6);
  }
}

// 层级指示器（地面/B1/B2 层间关系，HTML 组件）
function buildLevelIndicator() {
  if (!levelIndicatorEl || !S.init) return;
  const ramps = S.init.ramps || [];
  if (!ramps.length) { levelIndicatorEl.classList.add('hidden'); return; }
  const levels = [];
  for (const r of ramps) for (const lv of (r.levels || [])) {
    if (!levels.includes(lv)) levels.push(lv);
  }
  const cur = S.init.level || 'B1';
  let html = '';
  for (let i = 0; i < levels.length; i++) {
    if (i > 0) {
      const down = ramps.some(r => (r.levels || []).join(',') === levels[i-1] + ',' + levels[i] && r.dir === 'down');
      const up = ramps.some(r => (r.levels || []).join(',') === levels[i] + ',' + levels[i-1] && r.dir === 'up');
      html += '<div class="lv-conn">' + (down && up ? '↕' : down ? '↓' : up ? '↑' : '│') + '</div>';
    }
    html += '<div class="lv-item' + (levels[i] === cur ? ' active' : '') + '">' + levels[i] + '</div>';
  }
  levelIndicatorEl.innerHTML = html;
  levelIndicatorEl.classList.remove('hidden');
}

function fillSelect(sel, values, current, labels) {
  sel.innerHTML = '';
  for (const v of values) {
    const opt = document.createElement('option');
    opt.value = v;
    opt.textContent = (labels && labels[v] !== undefined) ? labels[v] : v;
    sel.appendChild(opt);
  }
  if (current !== undefined && current !== null) sel.value = current;
  if (sel.selectedIndex < 0 && sel.options.length) sel.selectedIndex = 0;
}

function initSchemePanel() {
  const schemes = (S.options && S.options.schemes) || {};
  const current = (S.options && S.options.current) || {};
  const fallback = {
    allocation_method: ['random', 'nearest_entrance', 'graph_cost', 'balanced_rows', 'manual'],
    route_planner: ['astar', 'dijkstra', 'via'],
    ref_path_generator: ['spline', 'linear'],
    maneuver_provider: ['offline', 'online_rs'],
  };
  const maps = (S.options && S.options.maps) || [];
  S.initDefaults = (S.options && S.options.init_defaults) || { mode: 'random', random: {}, replay: {}, custom_file: '' };
  // 保留用户当前选择的地图（修复：加载底图后 init 广播会把选择重置回服务器旧值）
  const _curMapSel = scMap.value;
  const _wantMap = (_curMapSel && maps.indexOf(_curMapSel) >= 0) ? _curMapSel
                 : (current.map || (S.options && S.options.map) || 'DJI_0012');
  fillSelect(scMap, maps.length ? maps : ['DJI_0012'], _wantMap);
  // 经验初始化依赖场景（agents）数据：无经验数据的地图（如 JTH）不提供该选项
  const hasExp = !(S.options && S.options.has_experience === false);
  const initModes = hasExp ? ['random', 'replay', 'custom'] : ['random', 'custom'];
  fillSelect(scInitMode, initModes,
             current.init_mode || S.initDefaults.mode || 'random', SCHEME_LABELS.init_mode);
  fillSelect(scAlloc, schemes.allocation_method || fallback.allocation_method, current.allocation_method || 'random', SCHEME_LABELS.allocation_method);
  fillSelect(scRoute, schemes.route_planner || fallback.route_planner, current.route_planner || 'astar', SCHEME_LABELS.route_planner);
  fillSelect(scRef, schemes.ref_path_generator || fallback.ref_path_generator, current.ref_path_generator || 'spline', SCHEME_LABELS.ref_path_generator);
  fillSelect(scManeuver, schemes.maneuver_provider || fallback.maneuver_provider, current.maneuver_provider || 'offline', SCHEME_LABELS.maneuver_provider);
  const managed = !!(S.options && S.options.manage_sim);
  btnApplyScheme.disabled = !managed;
  schemeStatus.textContent = managed ? '' : '当前桥为只读模式（未启用 --manage-sim），无法从网页重启仿真。';
  if (!S.schemeWired) {
    scInitMode.addEventListener('change', () => applyModePrefill());
    scMap.addEventListener('change', () => {
      setMsg('已选择场地：' + scMap.value + '，点击「加载底图」生效');
      if (btnLoadMap) btnLoadMap.textContent = '加载底图（' + scMap.value + '）';
    });
    S.schemeWired = true;
  }
  applyModePrefill();
  applyParamsToInputs(current.params);
}

function applyModePrefill() {
  const mode = scInitMode.value;
  secRandom.classList.toggle('hidden', mode !== 'random');
  secReplay.classList.toggle('hidden', mode !== 'replay');
  if (secCustom) secCustom.classList.toggle('hidden', mode !== 'custom');
  const d = S.initDefaults || {};
  if (mode === 'random') {
    const r = d.random || {};
    scEntering.value = (r.entering != null) ? r.entering : 30;
    scExiting.value = (r.exiting != null) ? r.exiting : 30;
    scInterval.value = (r.interval_mean != null) ? r.interval_mean : 5.0;
    scSeed.value = (r.seed != null) ? r.seed : 0;
    scYBound.value = (r.y_bound != null) ? r.y_bound : 72;
    const occ = r.occupancy || {};
    scBlocked.value = Array.isArray(occ.blocked) ? occ.blocked.join(',') : '';
    scOccupied.value = Array.isArray(occ.occupied) ? occ.occupied.join(',') : '';
    const ro = r.occupancy_random || {};
    if (scOccRandom) scOccRandom.checked = ro.enable !== false;
    if (scOccCount) scOccCount.value = (ro.count != null) ? ro.count : 0;
  }
  if (mode === 'replay') {
    const rp = d.replay || {};
    scTimeScale.value = (rp.time_scale != null) ? rp.time_scale : 1.0;
    scMaxAgents.value = (rp.max_agents != null) ? rp.max_agents : 0;
  }
  if (mode === 'custom' && scCustomFile) {
    scCustomFile.textContent = d.custom_file ? ('时间表文件：' + d.custom_file) : '未配置 custom.file（见 scenario.yaml）';
  }
}

function applyParamsToInputs(params) {
  if (!params) return;
  const r = params.random || {};
  if (r.entering != null) scEntering.value = r.entering;
  if (r.exiting != null) scExiting.value = r.exiting;
  if (r.interval_mean != null) scInterval.value = r.interval_mean;
  if (r.seed != null) scSeed.value = r.seed;
  if (r.y_bound != null) scYBound.value = r.y_bound;
  const occ = r.occupancy || {};
  if (Array.isArray(occ.blocked)) scBlocked.value = occ.blocked.join(',');
  if (Array.isArray(occ.occupied)) scOccupied.value = occ.occupied.join(',');
  const rp = params.replay || {};
  if (rp.time_scale != null) scTimeScale.value = rp.time_scale;
  if (rp.max_agents != null) scMaxAgents.value = rp.max_agents;
}

function parseSpotList(text) {
  const parts = String(text || '').split(/[,\s;]+/).filter(s => s.length > 0);
  const out = [];
  for (const p of parts) {
    const n = parseInt(p, 10);
    if (!isNaN(n)) out.push(n);
  }
  return out;
}

function collectParams() {
  const mode = scInitMode.value;
  const params = {};
  if (mode === 'random') {
    const random = {};
    const entering = parseInt(scEntering.value, 10);
    if (!isNaN(entering)) random.entering = entering;
    const exiting = parseInt(scExiting.value, 10);
    if (!isNaN(exiting)) random.exiting = exiting;
    const interval = parseFloat(scInterval.value);
    if (!isNaN(interval)) random.interval_mean = interval;
    const seed = parseInt(scSeed.value, 10);
    if (!isNaN(seed)) random.seed = seed;
    const yBound = parseFloat(scYBound.value);
    if (!isNaN(yBound)) random.y_bound = yBound;
    const blocked = parseSpotList(scBlocked.value);
    const occupied = parseSpotList(scOccupied.value);
    const occ = {};
    if (blocked.length) occ.blocked = blocked;
    if (occupied.length) occ.occupied = occupied;
    if (Object.keys(occ).length) random.occupancy = occ;
    const occRandom = {};
    if (scOccRandom) occRandom.enable = !!scOccRandom.checked;
    const occCount = parseInt(scOccCount && scOccCount.value, 10);
    if (!isNaN(occCount)) occRandom.count = occCount;
    if (Object.keys(occRandom).length) random.occupancy_random = occRandom;
    if (Object.keys(random).length) params.random = random;
  } else if (mode === 'replay') {
    const replay = {};
    const timeScale = parseFloat(scTimeScale.value);
    if (!isNaN(timeScale)) replay.time_scale = timeScale;
    const maxAgents = parseInt(scMaxAgents.value, 10);
    if (!isNaN(maxAgents)) replay.max_agents = maxAgents;
    if (Object.keys(replay).length) params.replay = replay;
  }
  return params;
}

function describeScheme(cfg) {
  cfg = cfg || {};
  const parts = [
    '初始化=' + schemeLabel('init_mode', cfg.init_mode || 'random'),
    '场地=' + (cfg.map || 'DJI_0012'),
    '分配=' + schemeLabel('allocation_method', cfg.allocation_method || 'random'),
    '路由=' + schemeLabel('route_planner', cfg.route_planner || 'astar'),
    '参考路径=' + schemeLabel('ref_path_generator', cfg.ref_path_generator || 'spline'),
    '机动=' + schemeLabel('maneuver_provider', cfg.maneuver_provider || 'offline'),
  ];
  if (cfg.params && Object.keys(cfg.params).length) parts.push('含细参数覆盖');
  return parts.join(' · ');
}

function collectSchemeConfig() {
  const config = {
    map: scMap.value,
    init_mode: scInitMode.value,
    allocation_method: scAlloc.value,
    route_planner: scRoute.value,
    ref_path_generator: scRef.value,
    maneuver_provider: scManeuver.value,
  };
  for (const k in config) { if (!config[k]) delete config[k]; }
  const params = collectParams();
  if (Object.keys(params).length) config.params = params;
  return config;
}

function applyScheme() {
  if (!S.options || !S.options.manage_sim) {
    schemeStatus.textContent = '当前桥未启用托管模式（--manage-sim）';
    return;
  }
  if (!S.ws || S.ws.readyState !== 1) {
    schemeStatus.textContent = '未连接，稍后再试';
    return;
  }
  btnApplyScheme.disabled = true;
  schemeStatus.textContent = '正在重启仿真…';
  S.ws.send(JSON.stringify({ type: 'restart', config: collectSchemeConfig() }));
}

/* ---------- 系统状态横幅（崩溃 / 停滞 / 断连） ---------- */
function showSysBanner(kind, text, restartEnabled) {
  S.sysBannerKind = kind;
  if (sysBannerText) sysBannerText.textContent = text || '';
  if (sysBanner) sysBanner.classList.remove('hidden');
  if (btnSysRestart) {
    const can = restartEnabled !== false && !!(S.options && S.options.manage_sim) && !!S.control;
    btnSysRestart.disabled = !can;
    btnSysRestart.title = can ? '' : '当前桥未启用托管模式（--manage-sim），无法从网页重启';
    btnSysRestart.textContent = (kind === 'idle') ? '开启仿真' : '重启仿真';
  }
}

function hideSysBanner() {
  S.sysBannerKind = null;
  if (sysBanner) sysBanner.classList.add('hidden');
}

function restartFromBanner() {
  if (!S.ws || S.ws.readyState !== 1) {
    showSysBanner('disconnected', '未连接到桥，无法重启（自动重连中）…', false);
    return;
  }
  if (!S.options || !S.options.manage_sim || !S.control) return;
  const starting = S.sysBannerKind === 'idle';
  btnSysRestart.disabled = true;
  showSysBanner('restarting', starting ? '正在启动仿真…' : '正在重启仿真…', false);
  S.ws.send(JSON.stringify({ type: 'restart', config: collectSchemeConfig() }));
}

function onSchemeStatus(m) {
  const v = m.value;
  if (v === 'restarting') {
    btnApplyScheme.disabled = true;
    schemeStatus.textContent = '正在重启仿真…';
    setMsg('正在重启仿真（切换方案）…');
    S.follow = 0;
    S.focusSent = -1;
    sendFocus(0);
    S.trailsMap.clear();
    S.paused = false;
    updatePauseLabel();
    showSysBanner('restarting', '正在重启仿真…', false);
  } else if (v === 'started') {
    S.options.current = m.config || {};
    if (m.config && m.config.init_mode) {
      S.obstacleMode = (m.config.init_mode === 'random') ? 'occupancy' : 'dataset';
    }
    btnApplyScheme.disabled = false;
    btnApplyScheme.textContent = '应用并重启仿真';
    hideSysBanner();
    schemeStatus.textContent = '已按新方案重启：' + describeScheme(m.config);
    setMsg('仿真已按新方案重启：' + describeScheme(m.config));
    setTimeout(() => { setMsg(''); }, 4500);
  } else if (v === 'error') {
    btnApplyScheme.disabled = false;
    schemeStatus.textContent = '重启失败：' + (m.message || '未知错误');
    setMsg('重启失败：' + (m.message || '未知错误'));
    showSysBanner('error', '重启失败：' + (m.message || '未知错误'), true);
  } else if (v === 'busy') {
    schemeStatus.textContent = '已有重启进行中，请稍候…';
  } else if (v === 'idle') {
    showSysBanner('idle', '底图已加载（仿真未启动）。点击「开启仿真」启动车辆仿真。', true);
    if (btnApplyScheme) btnApplyScheme.textContent = '开启仿真';
  } else if (v === 'map_switched') {
    if (btnLoadMap) { btnLoadMap.disabled = false; btnLoadMap.textContent = '加载底图'; }
    schemeStatus.textContent = (m.message || '底图已切换');
    setMsg(m.message || '底图已切换');
    setTimeout(() => { setMsg(''); }, 4500);
  } else if (v === 'sim_dead') {
    showSysBanner('dead', '仿真进程已退出（code ' + (m.code != null ? m.code : '?') + '）。点击「重启仿真」恢复。', true);
  } else if (v === 'stalled') {
    showSysBanner('stalled', '仿真数据流停滞（约 ' + (m.seconds || '?') + ' 秒未推进）。点击「重启仿真」恢复。', true);
  } else if (v === 'resumed') {
    if (S.sysBannerKind === 'stalled') hideSysBanner();
  }
}

let hoverTipVid = 0;
function updatePauseLabel() { btnPause.textContent = S.paused ? '恢复' : '暂停'; }

function fitView() {
  const z = Math.min(W / (S.mapW * S.scale), H / (S.mapH * S.scale)) * 0.94;
  S.view.zoom = z || 1;
  S.view.x = (W - S.mapW * S.scale * S.view.zoom) / 2;
  S.view.y = (H - S.mapH * S.scale * S.view.zoom) / 2;
}

/* ================= 尾迹 ================= */
function updateTrails() {
  const f = S.frame;
  if (!f || !f.vehicles) return;
  const now = performance.now();
  for (const v of f.vehicles) {
    let tr = S.trailsMap.get(v.id);
    if (!tr) { tr = []; S.trailsMap.set(v.id, tr); }
    const last = tr[tr.length - 1];
    if (!last || ((v.x - last.x) * (v.x - last.x) + (v.y - last.y) * (v.y - last.y)) > 0.16) {
      tr.push({ x: v.x, y: v.y, t: now });
      if (tr.length > 48) tr.shift();
    }
  }
  if (S.trailsMap.size) {
    const ids = new Set(f.vehicles.map(v => v.id));
    for (const id of Array.from(S.trailsMap.keys())) {
      if (!ids.has(id)) S.trailsMap.delete(id);
    }
  }
}

/* ================= 绘制 ================= */
function interpAlpha() {
  if (S.paused) return 1;
  return Math.min(1, (performance.now() - S.frameWallT) / 55);
}
function angDiff(a, b) {
  let d = b - a;
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return d;
}

function estimateSpeed(vh) {
  if (vh.v != null && vh.v >= 0) return vh.v;
  const pf = S.prevFrame;
  if (!pf || !pf.vehicles || !S.frame) return null;
  const pv = pf.vehicles.find(q => q.id === vh.id);
  if (!pv) return null;
  const dt = (S.frame.t || 0) - (pf.t || 0);
  if (dt < 0.02) return null;
  return Math.hypot(vh.x - pv.x, vh.y - pv.y) / dt;
}

function drawCarWorld(x, y, psi, color, opt) {
  opt = opt || {};
  const L = opt.L || 4.6, Wd = opt.W || 1.85;
  const th = THEMES[S.theme];
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(psi);
  if (opt.focus) {
    ctx.fillStyle = th.halo;
    rrectCtx(-L / 2 - 0.75, -Wd / 2 - 0.75, L + 1.5, Wd + 1.5, 1.1);
    ctx.fill();
  }
  if (!opt.ghost) { // 软阴影
    ctx.fillStyle = 'rgba(0,0,0,0.16)';
    rrectCtx(-L / 2 + 0.16, -Wd / 2 - 0.14, L, Wd, 0.6);
    ctx.fill();
  }
  ctx.fillStyle = color;
  rrectCtx(-L / 2, -Wd / 2, L, Wd, 0.6);
  ctx.fill();
  if (!opt.ghost) {
    ctx.lineWidth = 0.085;
    ctx.strokeStyle = th.vehStroke;
    ctx.stroke();
    ctx.fillStyle = th.cabin;
    rrectCtx(0.1, -Wd / 2 + 0.34, 1.55, Wd - 0.68, 0.3);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(L / 2 + 0.02, 0);
    ctx.lineTo(L / 2 - 0.55, -0.5);
    ctx.lineTo(L / 2 - 0.55, 0.5);
    ctx.closePath();
    ctx.fillStyle = th.wedge;
    ctx.fill();
  }
  ctx.restore();
}

function drawGrid(th) {
  const lw = 1 / (S.scale * S.view.zoom);
  ctx.strokeStyle = th.grid;
  ctx.lineWidth = lw;
  ctx.beginPath();
  for (let x = 0; x <= S.mapW; x += 10) { ctx.moveTo(x, 0); ctx.lineTo(x, S.mapH); }
  for (let y = 0; y <= S.mapH; y += 10) { ctx.moveTo(0, y); ctx.lineTo(S.mapW, y); }
  ctx.stroke();
  // 坐标文字（屏幕空间）
  setScreen();
  ctx.fillStyle = th.gridText;
  ctx.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
  for (let x = 0; x <= S.mapW; x += 10) {
    const [sx, sy] = w2s(x, 0);
    ctx.fillText(String(x), sx + 3, sy - 5);
  }
  for (let y = 10; y <= S.mapH; y += 10) {
    const [sx, sy] = w2s(0, y);
    ctx.fillText(String(y), sx + 3, sy - 4);
  }
  setWorld();
}

function drawHud(th) {
  setScreen();
  // 比例尺（左下）
  let mlen = 20, px = mlen * S.scale * S.view.zoom;
  if (px > 190) { mlen = 10; px = mlen * S.scale * S.view.zoom; }
  if (px < 34) { mlen = 50; px = mlen * S.scale * S.view.zoom; }
  const bx = 20, by = H - 22;
  ctx.strokeStyle = th.scale;
  ctx.fillStyle = th.scale;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(bx, by); ctx.lineTo(bx + px, by);
  ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by + 4);
  ctx.moveTo(bx + px, by - 4); ctx.lineTo(bx + px, by + 4);
  ctx.stroke();
  ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
  ctx.fillText(mlen + ' m', bx + px + 8, by + 4);
  // 指北针（右上，chips 下方）
  const nx = W - 34, ny = 92;
  ctx.beginPath();
  ctx.moveTo(nx, ny - 14); ctx.lineTo(nx - 6, ny + 6); ctx.lineTo(nx, ny + 2); ctx.lineTo(nx + 6, ny + 6);
  ctx.closePath();
  ctx.fillStyle = th.scale;
  ctx.fill();
  ctx.font = 'bold 10px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('N', nx, ny + 18);
  ctx.textAlign = 'start';
}

function updateChips() {
  const f = S.frame;
  vTime.textContent = f ? f.t.toFixed(1) + ' s' : '--';
  vVeh.textContent = (f && f.vehicles) ? String(f.vehicles.length) : '--';
  vFps.textContent = String(S.fps);
  vLat.textContent = S.latency != null ? S.latency + ' ms' : '--';
  vConn.textContent = S.connected ? '已连接' : '断开';
  chipConn.classList.toggle('ok', S.connected);
}

function draw() {
  requestAnimationFrame(draw);
  if (!W) return;
  S._fpsCount++;
  const now = performance.now();
  if (now - S._fpsT >= 500) {
    S.fps = Math.round(S._fpsCount * 1000 / (now - S._fpsT));
    S._fpsCount = 0; S._fpsT = now;
  }

  const th = THEMES[S.theme];
  setScreen();
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = th.canvasBg;
  ctx.fillRect(0, 0, W, H);

  const init = S.init;
  if (!init) { if (++S._chipTick % 15 === 0) updateChips(); return; }

  setWorld();
  const v = S.view;

  // 底图（世界空间；含变换时按「PNG 像素 → 世界坐标」矩阵绘制）
  if (S.layers.baseMap && bgImg.complete && bgImg.naturalWidth > 0) {
    ctx.save();
    ctx.globalAlpha = S.theme === 'dark' ? 0.42 : 0.72;
    if (S.bgT) {
      const t = S.bgT;
      // xr = -kx*py + cx ; yr = -ky*px + cy（canvas: x'=a*x+c*y+e, y'=b*x+d*y+f）
      ctx.transform(0, -t.ky, -t.kx, 0, t.cx, t.cy);
      ctx.drawImage(bgImg, 0, 0);
    } else {
      ctx.drawImage(bgImg, 0, 0, S.mapW, S.mapH);
    }
    ctx.globalAlpha = 1;
    ctx.restore();
  }

  // 网格
  if (S.layers.grid) drawGrid(th);

  // 停车排块（JTH 地图包已由底图提供车位详图 → row_fill 关闭时不绘制）
  if (P.rows && S.rowFill !== false) {
    ctx.fillStyle = th.rowFill;
    ctx.fill(P.rows);
    ctx.strokeStyle = th.rowStroke;
    ctx.lineWidth = 1 / (S.scale * v.zoom);
    ctx.stroke(P.rows);
  }
  // 车位线
  if (S.layers.spots && P.slots) {
    ctx.strokeStyle = th.slotLine;
    ctx.lineWidth = 1.1 / (S.scale * v.zoom);
    ctx.stroke(P.slots);
  }
  // 航点
  if (S.layers.waypoints && P.waypoints) {
    ctx.fillStyle = th.waypoint;
    ctx.fill(P.waypoints);
  }
  // 障碍车辆（dataset：数据集真实障碍；occupancy：按占用车位绘制停放车辆）
  if (S.layers.obstacles) {
    if (S.obstacleMode === 'occupancy') {
      drawOccupancyCars(th, v);
    }
    // 硬障碍物（地图包 obstacles.json）：任何模式都渲染
    if (P.obstacles) {
      ctx.fillStyle = th.obstacle;
      ctx.fill(P.obstacles);
      ctx.strokeStyle = th.obstacleStroke;
      ctx.lineWidth = 0.8 / (S.scale * v.zoom);
      ctx.stroke(P.obstacles);
    }
    drawDepartingSpots(th, v);
  }
  if (S.layers.entranceExit) drawEntranceExit(th, v);
  if (S.layers.ramps) {
    drawRamps(th, v);
  }

  const f = S.frame;

  // 参考路径（聚焦车辆）
  if (f && S.layers.paths && f.fpath && f.fpath.length > 1) {
    ctx.beginPath();
    ctx.moveTo(f.fpath[0][0], f.fpath[0][1]);
    for (let i = 1; i < f.fpath.length; i++) ctx.lineTo(f.fpath[i][0], f.fpath[i][1]);
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.strokeStyle = th.pathGlow;
    ctx.lineWidth = 2.2;
    ctx.stroke();
    ctx.strokeStyle = th.path;
    ctx.lineWidth = 0.3;
    ctx.stroke();
    const e = f.fpath[f.fpath.length - 1];
    ctx.beginPath();
    ctx.arc(e[0], e[1], 0.75, 0, Math.PI * 2);
    ctx.fillStyle = th.path;
    ctx.fill();
  }

  // 尾迹
  if (S.layers.trails) {
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.strokeStyle = 'rgba(' + th.trail + ',0.28)';
    ctx.lineWidth = 0.4;
    for (const tr of S.trailsMap.values()) {
      if (tr.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(tr[0].x, tr[0].y);
      for (let i = 1; i < tr.length; i++) ctx.lineTo(tr[i].x, tr[i].y);
      ctx.stroke();
    }
  }

  if (f) {
    // 幽灵层
    if (S.layers.ghosts && f.ghosts) {
      for (const g of f.ghosts) {
        drawCarWorld(g[0], g[1], g[2], th.ghost, { L: g[3] || 4.6, W: g[4] || 1.85, ghost: true });
      }
    }
    // 泊车目标车位高亮（未完成泊入的入库车辆；出库车不标记）
    const spotPolys = init.spots || [];
    const spotBadges = [];
    // 预约泊位固定绿色高亮（表示已被预约，不随车辆颜色/状态变化）
    const spotCol = th.spotReserved || '#2fd07e';
    for (const vh of f.vehicles) {
      const sp = vh.spot;
      if (sp == null || sp <= 0 || vh.c === 3) continue;
      const poly = spotPolys[sp];
      if (!poly || poly.length < 8) continue;
      ctx.beginPath();
      ctx.moveTo(poly[0], poly[1]);
      for (let i = 2; i < poly.length; i += 2) ctx.lineTo(poly[i], poly[i + 1]);
      ctx.closePath();
      ctx.globalAlpha = 0.18;
      ctx.fillStyle = spotCol;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = spotCol;
      ctx.lineWidth = 2.2 / (S.scale * v.zoom);
      ctx.stroke();
      let cx = 0, cy = 0;
      const np = poly.length / 2;
      for (let i = 0; i < poly.length; i += 2) { cx += poly[i]; cy += poly[i + 1]; }
      spotBadges.push([cx / np, cy / np, vh.id, spotCol]);
    }
    if (spotBadges.length) {
      setScreen();
      ctx.font = 'bold 10px ui-monospace, SFMono-Regular, Menlo, monospace';
      ctx.textAlign = 'center';
      for (const bd of spotBadges) {
        const pos = w2s(bd[0], bd[1]);
        const text = '#' + bd[2];
        const halfW = ctx.measureText(text).width / 2 + 5;
        rrectCtx(pos[0] - halfW, pos[1] - 8, halfW * 2, 16, 4);
        ctx.fillStyle = th.badgeBg;
        ctx.fill();
        ctx.fillStyle = bd[3];
        ctx.fillText(text, pos[0], pos[1] + 3.5);
      }
      ctx.textAlign = 'start';
      setWorld();
    }
    // 车辆（带插值）
    const alpha = interpAlpha();
    let prevMap = null;
    if (S.prevFrame && S.prevFrame.vehicles && !S.paused) {
      prevMap = new Map();
      for (const pv of S.prevFrame.vehicles) prevMap.set(pv.id, pv);
    }
    for (const vh of f.vehicles) {
      let x = vh.x, y = vh.y, psi = vh.psi;
      if (prevMap) {
        const pv = prevMap.get(vh.id);
        if (pv) {
          x = pv.x + (vh.x - pv.x) * alpha;
          y = pv.y + (vh.y - pv.y) * alpha;
          psi = pv.psi + angDiff(pv.psi, vh.psi) * alpha;
        }
      }
      const cols = [th.veh.driving, th.veh.parking, th.veh.braking, th.veh.done];
      drawCarWorld(x, y, psi, cols[vh.c] || cols[0], { focus: vh.id === S.follow });
      // 编号
      if (S.layers.labels && v.zoom > 0.5) {
        setScreen();
        const [tx, ty] = w2s(x, y);
        ctx.fillStyle = th.label;
        ctx.font = Math.min(16, Math.max(10, 11 * v.zoom)) + 'px sans-serif';
        ctx.fillText(vh.label || ('#' + vh.id), tx + 10, ty - 8);
        setWorld();
      }
    }
  }

  // 跟随
  if (S.follow && f) {
    const fv = f.vehicles.find(q => q.id === S.follow);
    if (fv) {
      v.x = W / 2 - fv.x * S.scale * v.zoom;
      v.y = H / 2 - (S.mapH - fv.y) * S.scale * v.zoom;
    } else {
      S.follow = 0;
    }
  }

  drawHud(th);

  if (++S._chipTick % 15 === 0) updateChips();
}

/* ================= 交互 ================= */
function pickVehicle(wx, wy, maxDist) {
  const f = S.frame;
  if (!f) return null;
  let best = null, bestD = maxDist * maxDist;
  for (const vh of f.vehicles) {
    const d = (vh.x - wx) * (vh.x - wx) + (vh.y - wy) * (vh.y - wy);
    if (d < bestD) { bestD = d; best = vh; }
  }
  return best;
}

function updateTooltip() {
  if (!S.hover) { tooltipEl.classList.add('hidden'); return; }
  const f = S.frame;
  if (!f) return;
  const vh = f.vehicles.find(q => q.id === S.hover);
  if (!vh) { S.hover = 0; tooltipEl.classList.add('hidden'); return; }
  const spd = estimateSpeed(vh);
  tooltipEl.innerHTML =
    '<div class="tt-title">车辆 #' + vh.id + '</div>' +
    '<div class="tt-sub">' + (STATE_NAMES[vh.c] || '') +
    ' · 速度 ' + (spd != null ? spd.toFixed(1) : '--') + ' m/s</div>';
  const mx = S.mouse.x, my = S.mouse.y;
  tooltipEl.style.left = Math.min(mx + 16, window.innerWidth - 250) + 'px';
  tooltipEl.style.top = Math.min(my + 14, window.innerHeight - 90) + 'px';
  tooltipEl.classList.remove('hidden');
}

cv.addEventListener('wheel', (e) => {
  e.preventDefault();
  const factor = Math.exp(-e.deltaY * 0.0012);
  const z1 = Math.min(8, Math.max(0.25, S.view.zoom * factor));
  const mx = e.offsetX, my = e.offsetY;
  const wpt = s2w(mx, my);
  S.view.zoom = z1;
  const np = w2s(wpt[0], wpt[1]);
  S.view.x += mx - np[0];
  S.view.y += my - np[1];
}, { passive: false });

cv.addEventListener('mousedown', (e) => {
  if (e.button !== 0) return;
  S.dragging = true;
  S.dragMoved = false;
  S.downMouse = [e.clientX, e.clientY];
  S.lastMouse = [e.clientX, e.clientY];
  cv.classList.add('dragging');
});
window.addEventListener('mousemove', (e) => {
  const r = cv.getBoundingClientRect();
  S.mouse.x = e.clientX - r.left; S.mouse.y = e.clientY - r.top;
  if (S.dragging && S.lastMouse) {
    const dx = e.clientX - S.lastMouse[0], dy = e.clientY - S.lastMouse[1];
    if (Math.abs(e.clientX - S.downMouse[0]) + Math.abs(e.clientY - S.downMouse[1]) > 4) S.dragMoved = true;
    if (S.dragMoved) {
      S.view.x += dx; S.view.y += dy;
      S.follow = 0; // 拖动即退出跟随
    }
    S.lastMouse = [e.clientX, e.clientY];
  } else if (S.frame && !S.presentation) {
    const wp = s2w(S.mouse.x, S.mouse.y);
    const hit = pickVehicle(wp[0], wp[1], 1.6);
    S.hover = hit ? hit.id : 0;
    updateTooltip();
  }
});
window.addEventListener('mouseup', (e) => {
  if (!S.dragging) return;
  S.dragging = false;
  cv.classList.remove('dragging');
  if (!S.dragMoved && S.frame) {
    const wp = s2w(S.mouse.x, S.mouse.y);
    const hit = pickVehicle(wp[0], wp[1], 2.2);
    if (hit) { S.follow = hit.id; sendFocus(hit.id); }
    else { S.follow = 0; sendFocus(0); }
  }
});
cv.addEventListener('mouseleave', () => { S.hover = 0; updateTooltip(); });

btnPause.addEventListener('click', () => {
  if (!S.control || !S.ws || S.ws.readyState !== 1) return;
  S.ws.send(JSON.stringify({ type: 'pause', value: !S.paused }));
});
btnReset.addEventListener('click', () => { S.follow = 0; sendFocus(0); fitView(); });
btnTheme.addEventListener('click', cycleTheme);
function closeFloatingPanels() {
  layersEl.classList.add('hidden');
  mapPanelEl.classList.add('hidden');
  schemeEl.classList.add('hidden');
}
btnLayers.addEventListener('click', (e) => {
  e.stopPropagation();
  closeFloatingPanels();
  layersEl.classList.toggle('hidden');
});
btnMap.addEventListener('click', (e) => {
  e.stopPropagation();
  closeFloatingPanels();
  mapPanelEl.classList.toggle('hidden');
});
btnScheme.addEventListener('click', (e) => {
  e.stopPropagation();
  closeFloatingPanels();
  schemeEl.classList.toggle('hidden');
});
btnLoadMap.addEventListener('click', () => {
  if (!S.ws || S.ws.readyState !== 1) {
    setMsg('未连接到桥，无法加载底图');
    return;
  }
  btnLoadMap.disabled = true;
  btnLoadMap.textContent = '正在加载底图…';
  setMsg('正在加载底图：' + scMap.value + ' …');
  S.ws.send(JSON.stringify({ type: 'switch_map', map: scMap.value }));
});
btnApplyScheme.addEventListener('click', applyScheme);
btnSysRestart.addEventListener('click', restartFromBanner);
document.addEventListener('click', (e) => {
  if (!layersEl.classList.contains('hidden') && !layersEl.contains(e.target) && e.target !== btnLayers) {
    layersEl.classList.add('hidden');
  }
  if (!mapPanelEl.classList.contains('hidden') && !mapPanelEl.contains(e.target) && e.target !== btnMap) {
    mapPanelEl.classList.add('hidden');
  }
  if (!schemeEl.classList.contains('hidden') && !schemeEl.contains(e.target) && e.target !== btnScheme) {
    schemeEl.classList.add('hidden');
  }
});

window.addEventListener('keydown', (e) => {
  if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA')) return;
  if (e.key === ' ') { e.preventDefault(); btnPause.click(); }
  else if (e.key === 'h' || e.key === 'H') {
    S.presentation = !S.presentation;
    document.body.classList.toggle('presentation', S.presentation);
  } else if (e.key === 'r' || e.key === 'R') { S.follow = 0; sendFocus(0); fitView(); }
  else if (e.key === 't' || e.key === 'T') cycleTheme();
  else if (e.key === 'Escape') {
    if (!schemeEl.classList.contains('hidden')) { schemeEl.classList.add('hidden'); }
    else if (S.presentation) { S.presentation = false; document.body.classList.remove('presentation'); }
    else { S.follow = 0; sendFocus(0); }
  }
});

// 延迟测量
setInterval(() => {
  if (S.ws && S.ws.readyState === 1) {
    S.ws.send(JSON.stringify({ type: 'ping', t: performance.now() }));
  }
}, 2000);

// 客户端侧故障兜底：断连 / 数据流停滞（桥侧监控之外的第二层）
setInterval(() => {
  if (!S.connected) {
    if (S.init && S.sysBannerKind !== 'dead') {
      showSysBanner('disconnected', '与桥的连接已断开（自动重连中）…', false);
    }
    return;
  }
  if (S.sysBannerKind === 'disconnected') hideSysBanner();
  if (S.paused || !S.frame || !S.frameWallT) return;
  const gap = performance.now() - S.frameWallT;
  if (S.sysBannerKind === null && gap > 15000) {
    showSysBanner('stalled', '数据流中断（约 ' + Math.round(gap / 1000) + ' 秒未收到更新）。', true);
  } else if (S.sysBannerKind === 'stalled' && gap <= 15000) {
    hideSysBanner();
  }
}, 3000);

// 提示条淡出
setTimeout(() => hintEl.classList.add('fade'), 7000);

/* ================= 启动 ================= */
initTheme();
initLayers();
resize();
connect();
draw();
})();
