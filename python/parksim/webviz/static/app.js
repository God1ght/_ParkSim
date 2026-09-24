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
    slotLine: 'rgba(72,200,255,0.85)',
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
    slotLine: 'rgba(30,140,220,0.85)',
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
  bgBase: null,
  theme: 'dark',
  trailsMap: new Map(),
  fps: 0, _fpsCount: 0, _fpsT: performance.now(), _chipTick: 0,
  mapW: 140, mapH: 80, scale: 10,
  dpr: 1,
  hover: 0,
  presentation: false,
  occupancy: null,
  occPath: null,
  departing: null,
  staticGarage: [],
  obstacleMode: 'dataset',
  sysBannerKind: null,
  simState: null,   // 仿真生命周期状态：null=未知（未收到 sim_state，保持旧行为）
  degraded: false,      // 关键资产缺失（启动自检降级标记，来自 init.degraded）
  assetsMissing: [],    // init.assets_missing：缺失资产绝对路径列表
  activeAlerts: {},     // 仍生效的黄色告警：code -> message（被更严重横幅占用时保留）
  // 控制指令（stop/pause）ack 超时管理：计时器句柄 + 是否已重发一次
  _ctlAck: { stop: null, pause: null },
  _ctlAckRetried: { stop: false, pause: false },
  // ack 超时重发用的「已序列化载荷」：首次发送时冻结，重发逐字节复用（防 toggle 反转）
  _ctlAckPayload: { stop: null, pause: null },
};

const P = { rows: null, slots: null, obstacles: null, waypoints: null };

/* ================= DOM ================= */
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const bgImg = document.getElementById('bg');
const btnPause = document.getElementById('btnPause');
const btnStart = document.getElementById('btnStart');
const btnStop = document.getElementById('btnStop');
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
const schemeCurrent = document.getElementById('schemeCurrent');
const sysBanner = document.getElementById('sysBanner');
const sysBannerText = document.getElementById('sysBannerText');
const btnSysRestart = document.getElementById('btnSysRestart');
const confirmModal = document.getElementById('confirmModal');
const confirmText = document.getElementById('confirmText');
const btnConfirmOk = document.getElementById('btnConfirmOk');
const btnConfirmCancel = document.getElementById('btnConfirmCancel');
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
const scOccDataset = document.getElementById('scOccDataset');
const tooltipEl = document.getElementById('tooltip');
const msgEl = document.getElementById('msg');
const hintEl = document.getElementById('hint');
const chipConn = document.getElementById('chipConn');
const chipSim = document.getElementById('chipSim');
const vSim = document.getElementById('vSim');
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
    baseMap: S.theme === 'light', zoneOverview: false, grid: false, spots: true, waypoints: false,
    obstacles: false, ghosts: true, paths: true, trails: false, labels: true,
    ramps: true, entranceExit: true,
  };
}
function applyLayerState() {
  document.querySelectorAll('#layers input').forEach(cb => {
    cb.checked = !!S.layers[cb.dataset.layer];
  });
}
function persistLayers() {
  try { localStorage.setItem('parksim_layers_v5', JSON.stringify(S.layers)); } catch (e) {}
}
function resetLayers() {
  S.layers = defaultLayers();
  applyLayerState();
  applyBaseMapVariant();
  persistLayers();
}
function initLayers() {
  try {
    const raw = localStorage.getItem('parksim_layers_v5');
    if (raw) S.layers = Object.assign(defaultLayers(), JSON.parse(raw));
  } catch (e) {}
  if (!S.layers) S.layers = defaultLayers();
  // 从持久态恢复时也要联动：「分区概览图」依赖「底图」总开关，
  // 否则 localStorage 里 zoneOverview=true / baseMap=false 时，图有 url 也不会绘制。
  if (S.layers.zoneOverview && !S.layers.baseMap) S.layers.baseMap = true;
  applyLayerState();
  persistLayers();
  document.querySelectorAll('#layers input').forEach(cb => {
    cb.addEventListener('change', () => {
      S.layers[cb.dataset.layer] = cb.checked;
      if (cb.dataset.layer === 'zoneOverview') {
        // 「底图」是底图绘制总开关：开概览图时若它关着，图不会画出来
        if (cb.checked && !S.layers.baseMap) {
          S.layers.baseMap = true;
          const bcb = document.querySelector('#layers input[data-layer="baseMap"]');
          if (bcb) bcb.checked = true;
        }
        applyBaseMapVariant();
      }
      persistLayers();
    });
  });
  const resetBtn = document.getElementById('btnLayersReset');
  if (resetBtn) resetBtn.addEventListener('click', resetLayers);
}
/* 图层可用性：数据智能体仅回放模式有效；坡道无数据时自动隐藏 */
function applyLayerAvailability() {
  const useExisting = !!(S.options && S.options.use_existing_agents);
  const ghostPill = document.querySelector('#layers label[data-layernote="ghosts"]');
  const ghostHint = document.getElementById('ghostHint');
  if (ghostPill) {
    const off = !useExisting;
    ghostPill.classList.toggle('pill-off', off);
    if (off) {
      const cb = ghostPill.querySelector('input');
      if (cb && S.layers.ghosts) { S.layers.ghosts = false; applyLayerState(); }
    }
    if (ghostHint) ghostHint.textContent = off ? '· 仅回放模式' : '';
  }
  const rampPill = document.querySelector('#layers label[data-layernote="ramps"]');
  if (rampPill) {
    const hasRamp = !!(S.init && S.init.ramps && S.init.ramps.length);
    rampPill.classList.toggle('hidden', !hasRamp);
    if (!hasRamp) {
      const cb = rampPill.querySelector('input');
      if (cb && S.layers.ramps) { S.layers.ramps = false; applyLayerState(); }
    }
  }
  // 分区概览图：服务端未下发图与变换时整条隐藏，避免打开了也没东西可画
  const zonePill = document.querySelector('#layers label[data-layernote="zoneOverview"]');
  if (zonePill) {
    const hasZone = !!(S.options && S.options.zone_overview_url && S.options.zone_overview_transform);
    zonePill.classList.toggle('hidden', !hasZone);
    if (!hasZone) {
      const cb = zonePill.querySelector('input');
      if (cb && S.layers.zoneOverview) { S.layers.zoneOverview = false; applyLayerState(); }
    }
  }
}
/* 底图变体：原底图 ↔ 分区概览图（只换 URL 与「像素→世界」变换，其余绘制逻辑不变） */
function applyBaseMapVariant() {
  const o = S.options || {};
  const useZone = !!(S.layers && S.layers.zoneOverview
    && o.zone_overview_url && o.zone_overview_transform);
  const base = S.bgBase || { url: o.base_map_url || '', t: null };
  const url = useZone ? o.zone_overview_url : base.url;
  const t = useZone ? o.zone_overview_transform : base.t;
  if (url && bgImg.getAttribute('src') !== url) bgImg.src = url;
  S.bgT = t || null;
}
/* 参考路径提示：未聚焦车辆时提醒“点击车辆查看”（聚焦后隐藏） */
function updatePathHint() {
  const ph = document.getElementById('pathHint');
  if (!ph) return;
  ph.classList.toggle('hidden', !!S.follow);
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
  P.gates = init.gates || [];      // 多出入口「门」；空 → 回退单点 entrance/exit
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
      // 是否具备经验/占用数据集：DJI 系列=true（场景障碍=车辆障碍，仅经验模式读取）；JTH=false（固定障碍，始终读取）
      S.hasExperience = !(m.options && m.options.has_experience === false);
      // 启动期关键资产自检结果（前向兼容：字段缺失视为未降级）
      S.degraded = !!m.degraded;
      S.assetsMissing = Array.isArray(m.assets_missing) ? m.assets_missing : [];
      S.activeAlerts = {};   // 新 init 后由服务端回放的 active_alerts 重建
      S.occupancy = null;
      S.occPath = null;
      hideSysBanner();
      btnPause.disabled = !S.control;
      setMsg(S.control ? '' : '桥为只读模式（未启用控制，暂停按钮不可用）');
      if (m.options && m.options.base_map_url) bgImg.src = m.options.base_map_url;
      S.bgT = m.base_map_transform || null;
      S.bgBase = { url: (m.options && m.options.base_map_url) || '', t: S.bgT };
      applyBaseMapVariant();
      S.rowFill = (m.row_fill !== false);   // 地图包（JTH）默认不画车位行底色
      applyLayerAvailability();
      initSchemePanel();
      // 仿真生命周期状态：init 顶层 sim_state 优先，其次 options.sim_state；缺省 = 未知（保持旧行为）
      const _simState = (m.sim_state !== undefined) ? m.sim_state
                      : (m.options && m.options.sim_state);
      S.simState = normalizeSimState(_simState);
      applySimStateUI();
      updateSimChip();
      // 启动期关键资产缺失 → 黄色横幅（桥侧自检未通过；不阻断启动，仅告知原因）
      if (S.degraded) showSysBanner('warn:degraded', degradedBannerText(), true);
      buildStatic();
      fitView();
    } else if (m.type === 'frame') {
      S.prevFrame = S.frame;
      S.frame = m;
      S.frameWallT = performance.now();
      S.staticGarage = m.static_obstacles || S.staticGarage || [];
      // 静态障碍集合增减时重建占用车位路径，保证与 staticGarage 去重保持同步
      if (S.staticGarage.length !== (S._garageN || 0)) {
        S._garageN = S.staticGarage.length;
        S.occPath = buildOccPath();
      }
      S.paused = !m.running;
      updatePauseLabel();
      updateTrails();
    } else if (m.type === 'occupancy') {
      S.occupancy = m.data || null;
      S.occPath = buildOccPath();
    } else if (m.type === 'departing') {
      S.departing = m.data || null;
      S.occPath = buildOccPath();
    } else if (m.type === 'paused') {
      S.paused = !!m.value;
      clearControlAck('pause');   // 收到暂停 ack → 清除超时计时器
      updatePauseLabel();
      updateSimChip();
    } else if (m.type === 'status') {
      onSchemeStatus(m);
    } else if (m.type === 'alert') {
      onSimAlert(m);
    } else if (m.type === 'stopped') {
      onSimStopped(m);
    } else if (m.type === 'stop_failed') {
      onSimStopFailed(m);
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

function ocQuadToPath(p2, sp, k) {
  const cx = (sp[0] + sp[2] + sp[4] + sp[6]) / 4;
  const cy = (sp[1] + sp[3] + sp[5] + sp[7]) / 4;
  p2.moveTo(sp[0] + (cx - sp[0]) * k, sp[1] + (cy - sp[1]) * k);
  p2.lineTo(sp[2] + (cx - sp[2]) * k, sp[3] + (cy - sp[3]) * k);
  p2.lineTo(sp[4] + (cx - sp[4]) * k, sp[5] + (cy - sp[5]) * k);
  p2.lineTo(sp[6] + (cx - sp[6]) * k, sp[7] + (cy - sp[7]) * k);
  p2.closePath();
}
/* 缓存占用车位车为一个 Path2D，仅占用/离场变化时重建；正在驶出的由 departing 层单独画 */
function buildOccPath() {
  const occ = S.occupancy;
  const spots = S.init && S.init.spots;
  if (!occ || !spots) return null;
  const depSet = new Set((S.departing || []).map(String));
  // 已由 staticGarage 以真实几何绘制的车位不再重复画占用方块（避免叠加/双影）
  const garageSet = new Set((S.staticGarage || []).map((ob) => String(ob && ob.spot)));
  const p2 = new Path2D();
  const k = 0.16;
  const n = Math.min(occ.length, spots.length);
  for (let i = 0; i < n; i++) {
    if (!occ[i]) continue;
    if (depSet.has(String(i))) continue;
    if (garageSet.has(String(i))) continue;
    ocQuadToPath(p2, spots[i], k);
  }
  return p2;
}
function drawStaticObstacles(th, v) {
  const list = S.staticGarage;
  if (!list || !list.length) return;
  const sw = 0.8 / (S.scale * v.zoom);
  for (const ob of list) {
    const L = ob.l || 4.6, W = ob.w || 1.85;
    ctx.save();
    ctx.translate(ob.x, ob.y);
    ctx.rotate(ob.psi || 0);
    // 静态停放车辆：实心车身（无遮挡阴影/驾驶舱细节），醒目标识障碍语义
    ctx.fillStyle = th.obstacle;
    rrectCtx(-L / 2, -W / 2, L, W, 0.6);
    ctx.fill();
    ctx.strokeStyle = th.obstacleStroke;
    ctx.lineWidth = sw;
    ctx.stroke();
    ctx.restore();
  }
}
function drawOccupancyCars(th, v) {
  const p2 = S.occPath;
  if (!p2) return;
  ctx.fillStyle = th.obstacle;
  ctx.fill(p2);
  ctx.strokeStyle = th.obstacleStroke;
  ctx.lineWidth = 0.8 / (S.scale * v.zoom);
  ctx.stroke(p2);
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

// ================= 出入口「门」可视化 =================
// 门 = 按朝向摆放的扁长矩形：长轴角 = heading + 90°，短轴（法向）= heading。
// 世界→屏幕变换是 Y 翻转的（见 setWorld），所以**不使用 ctx.rotate**：
// 一律在世界坐标里算出矩形角点，再用 w2s 逐个转成屏幕坐标后画多边形，
// 从根上规避旋转角度符号被 Y 翻转带偏的问题。
const GATE_THICK_DEFAULT = 2.0;   // 门厚（米，沿法向）
const GATE_LONG_DEFAULT = 3.5;    // 门长（米，沿长轴）

function gateGeom(g) {
  const h = (typeof g.heading === 'number') ? g.heading : 0;
  const ht = (g.thick || GATE_THICK_DEFAULT) / 2;
  let cx = g.x, cy = g.y;
  let hl = (g.long || GATE_LONG_DEFAULT) / 2;
  let tx = Math.cos(h + Math.PI / 2), ty = Math.sin(h + Math.PI / 2);  // 长轴
  let nx = Math.cos(h), ny = Math.sin(h);                              // 短轴（法向）
  // 权威门洞截面（portal 的 section_rotated_m）：直接拿两端点当长边，
  // 不再按 heading 反推长轴 —— heading 是过门行驶方向，未必与门洞严格垂直。
  const sec = g.section;
  if (sec && sec.length === 2 && isFinite(sec[0][0]) && isFinite(sec[1][0])) {
    const dx = sec[1][0] - sec[0][0], dy = sec[1][1] - sec[0][1];
    const L = Math.hypot(dx, dy);
    if (L > 1e-6) {
      cx = (sec[0][0] + sec[1][0]) / 2;
      cy = (sec[0][1] + sec[1][1]) / 2;
      hl = (g.long || L) / 2;      // 门长以 width_m 为准，方向/中心取截面
      tx = dx / L; ty = dy / L;
      nx = ty; ny = -tx;           // 短轴 = 截面法向
    }
  }
  return { x: cx, y: cy, hl: hl, ht: ht, h: h, tx: tx, ty: ty, nx: nx, ny: ny };
}

// 沿长轴 t∈[t0, t1] 截取的矩形（世界坐标 4 角点）
function gateQuad(q, t0, t1) {
  const ax = q.x + t0 * q.tx, ay = q.y + t0 * q.ty;
  const bx = q.x + t1 * q.tx, by = q.y + t1 * q.ty;
  return [
    [ax + q.ht * q.nx, ay + q.ht * q.ny],
    [bx + q.ht * q.nx, by + q.ht * q.ny],
    [bx - q.ht * q.nx, by - q.ht * q.ny],
    [ax - q.ht * q.nx, ay - q.ht * q.ny],
  ];
}

// 世界坐标角点 → 屏幕坐标多边形
function gatePoly(pts) {
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const s = w2s(pts[i][0], pts[i][1]);
    if (i === 0) ctx.moveTo(s[0], s[1]); else ctx.lineTo(s[0], s[1]);
  }
  ctx.closePath();
}

// 门内方向箭头：世界坐标起止点 → 屏幕坐标线段 + 箭头头部
function gateArrow(q, t, dirRad, color) {
  const cxw = q.x + t * q.tx, cyw = q.y + t * q.ty;
  const dx = Math.cos(dirRad), dy = Math.sin(dirRad);
  const L = Math.min(5.0, Math.max(2.4, q.hl * 0.9));
  const tipx = cxw + dx * L * 0.5, tipy = cyw + dy * L * 0.5;
  const a = w2s(cxw - dx * L * 0.5, cyw - dy * L * 0.5);
  const b = w2s(tipx, tipy);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2.0;
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.stroke();
  const hl = 1.4;
  const p1 = w2s(tipx - Math.cos(dirRad - 0.45) * hl, tipy - Math.sin(dirRad - 0.45) * hl);
  const p2 = w2s(tipx - Math.cos(dirRad + 0.45) * hl, tipy - Math.sin(dirRad + 0.45) * hl);
  ctx.beginPath();
  ctx.moveTo(b[0], b[1]); ctx.lineTo(p1[0], p1[1]);
  ctx.moveTo(b[0], b[1]); ctx.lineTo(p2[0], p2[1]);
  ctx.stroke();
}

// 标签：锚在矩形上方（屏幕坐标，跟随门口朝向自适应）
function gateLabel(pts, text, th) {
  let minY = Infinity, cx = 0;
  for (const p of pts) {
    const s = w2s(p[0], p[1]);
    if (s[1] < minY) minY = s[1];
    cx += s[0];
  }
  cx /= pts.length;
  ctx.font = '11px system-ui, -apple-system, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  const tw = ctx.measureText(text).width;
  const ly = minY - 13;
  ctx.fillStyle = th.gateLabelBg;
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(cx - tw / 2 - 5, ly - 9, tw + 10, 18, 6);
  else ctx.rect(cx - tw / 2 - 5, ly - 9, tw + 10, 18);
  ctx.fill();
  ctx.fillStyle = th.gateLabel;
  ctx.fillText(text, cx, ly);
}

function drawGate(g, th) {
  if (!g || g.x == null || g.y == null) return;
  const roles = g.roles || [];
  const isIn = roles.indexOf('entrance') >= 0;
  const isOut = roles.indexOf('exit') >= 0;
  if (!isIn && !isOut) return;
  const both = isIn && isOut;
  const q = gateGeom(g);
  const side = (g.entry_side === 1) ? 1 : -1;
  ctx.lineWidth = 1.4;
  if (both) {
    // 双向门：沿长轴劈成两半，进/出各一色（entry_side 决定哪半边是「进」）
    gatePoly(gateQuad(q, 0, side * q.hl));
    ctx.fillStyle = th.entranceFill; ctx.fill();
    ctx.strokeStyle = th.entranceStroke; ctx.stroke();
    gatePoly(gateQuad(q, 0, -side * q.hl));
    ctx.fillStyle = th.exitFill; ctx.fill();
    ctx.strokeStyle = th.exitStroke; ctx.stroke();
  } else {
    gatePoly(gateQuad(q, -q.hl, q.hl));
    ctx.fillStyle = isIn ? th.entranceFill : th.exitFill; ctx.fill();
    ctx.strokeStyle = isIn ? th.entranceStroke : th.exitStroke; ctx.stroke();
  }
  // 方向箭头：入口沿 heading 进场；单向出口亦沿 heading 离场；双向门出口取反向
  if (isIn) gateArrow(q, both ? side * q.hl * 0.5 : 0, q.h, th.entranceStroke);
  if (isOut) gateArrow(q, both ? -side * q.hl * 0.5 : 0, both ? q.h + Math.PI : q.h, th.exitStroke);
  const tag = both ? '进/出' : (isIn ? '入口' : '出口');
  gateLabel(gateQuad(q, -q.hl, q.hl), (g.id ? g.id + ' ' : '') + tag, th);
}

// 兼容旧数据：无 portals 的地图仍按原来的单点小方块画（entrance / exit）
function drawGatePoint(pt, kind, th, v) {
  if (!pt || pt.x == null) return;
  const k = S.scale * v.zoom;
  const isIn = kind === 'in';
  const R = 4.0;                      // 区域半边长（8m 区域）
  ctx.save();
  setWorld();                         // 单点画法沿用世界坐标
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(pt.x - R, pt.y - R, R * 2, R * 2, 1.2);
  else ctx.rect(pt.x - R, pt.y - R, R * 2, R * 2);
  ctx.fillStyle = isIn ? th.entranceFill : th.exitFill;
  ctx.fill();
  ctx.strokeStyle = isIn ? th.entranceStroke : th.exitStroke;
  ctx.lineWidth = 1.4 / k;
  ctx.stroke();
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
    const a = pt.heading, hl = 1.6;
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - Math.cos(a - 0.45) * hl, ey - Math.sin(a - 0.45) * hl);
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - Math.cos(a + 0.45) * hl, ey - Math.sin(a + 0.45) * hl);
    ctx.lineWidth = 2.0 / k;
    ctx.stroke();
  }
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
}

// 入口/出口区域可视化：优先画 portal「门」，无 portal 数据时回退单点画法
function drawEntranceExit(th, v) {
  const gates = P.gates || [];
  if (!gates.length) {
    drawGatePoint(P.entrance, 'in', th, v);
    drawGatePoint(P.exit, 'out', th, v);
    return;
  }
  ctx.save();                   // 保存世界变换（含 ctx 状态）
  setScreen();                  // 门按屏幕坐标绘制：角点已由 w2s 转换
  for (const g of gates) drawGate(g, th);
  ctx.restore();                // 还原世界变换
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

function _pickOccDataset(baseMap, currentMap, occDatasets) {
  // 底图 DJI 时，取当前加载的真实 DJI_XXXX 数据集作为占用；否则回退第一个。
  if (occDatasets && occDatasets.length) {
    const cur = (typeof currentMap === 'string' && occDatasets.indexOf(currentMap) >= 0)
      ? currentMap : null;
    return cur || occDatasets[0];
  }
  return 'DJI_0012';
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
  // 经验初始化的「初始占用」数据集（DJI 底图下可选 DJI_XXXX，对应不同泊位初始占用）
  const occDatasets = (S.options && S.options.occupancy_datasets) || [];
  const _curOcc = _pickOccDataset(scMap.value, current.map, occDatasets);
  fillSelect(scOccDataset, occDatasets.length ? occDatasets : ['DJI_0012'], _curOcc);
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
  refreshSchemeCurrent();
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
  // 底图为 DJI（共享几何）时，真实地图参数取选定的 DJI_XXXX 占用数据集：
  // DJI_XXXX 后缀只表示泊位初始占用状态，不改变底图几何。
  const _baseMap = scMap.value;
  const _mapVal = (_baseMap === 'DJI' && scOccDataset && scOccDataset.value)
    ? scOccDataset.value : _baseMap;
  const config = {
    map: _mapVal,
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

/* ---------- 系统状态横幅（崩溃 / 停滞 / 断连 / 降级 / 告警） ---------- */
function showSysBanner(kind, text, restartEnabled) {
  S.sysBannerKind = kind;
  if (sysBannerText) sysBannerText.textContent = text || '';
  if (sysBanner) {
    sysBanner.classList.remove('hidden');
    // 'warn' 前缀 = 黄色告警/降级横幅（区别于红色致命横幅），按钮区隐藏
    sysBanner.classList.toggle('warn', typeof kind === 'string' && kind.indexOf('warn') === 0);
  }
  if (btnSysRestart) {
    const can = restartEnabled !== false && !!(S.options && S.options.manage_sim) && !!S.control;
    btnSysRestart.disabled = !can;
    btnSysRestart.title = can ? '' : '当前桥未启用托管模式（--manage-sim），无法从网页重启';
    btnSysRestart.textContent = (kind === 'idle') ? '开启仿真' : '重启仿真';
  }
}

function hideSysBanner() {
  S.sysBannerKind = null;
  if (sysBanner) {
    sysBanner.classList.add('hidden');
    sysBanner.classList.remove('warn');
  }
}

/* 关键资产缺失横幅文案：列出缺失路径（最多 3 条，其余折叠计数）。
 * 注意措辞：该自检只覆盖「桥侧资产根」（PARKSIM_ASSET_ROOT / 默认根）；
 * 仿真节点读的是自己的资产根，不受此覆盖影响 —— 实测该场景下仿真照样发车，
 * 所以不能断言「仿真将无法正常发车」，只能提示可能失败并指路日志。 */
function degradedBannerText() {
  const miss = (S.assetsMissing || []).filter(Boolean);
  const tail = '（桥侧资产自检未通过；若为 PARKSIM_ASSET_ROOT 覆盖所致，仿真节点仍读真实资产、可正常发车；' +
    '若资产确实缺失，发车会失败——详见服务端日志）';
  if (!miss.length) return '关键资产缺失' + tail;
  const shown = miss.slice(0, 3).join('、');
  const more = miss.length > 3 ? (' 等共 ' + miss.length + ' 项') : '';
  return '关键资产缺失：' + shown + more + tail;
}

/* 告警解除/被更严重横幅占用后，回落到「基础横幅」：
 * 优先仍未解除的黄色告警 → 降级提示 → idle 提示 → 隐藏。 */
function restoreBaseBanner() {
  const codes = Object.keys(S.activeAlerts || {});
  if (codes.length) {
    const code = codes[codes.length - 1];
    showSysBanner('warn:' + code, S.activeAlerts[code] || '仿真告警', false);
    return;
  }
  if (S.degraded) {
    showSysBanner('warn:degraded', degradedBannerText(), true);
  } else if (S.simState === 'idle' || S.simState === 'stopped') {
    // stopped 与 idle 同属「仿真未运行」：回落到 idle 提示（带「开启仿真」按钮）
    showSysBanner('idle', '底图已加载（仿真未启动）。点击「开启仿真」启动车辆仿真。', true);
  } else {
    hideSysBanner();
  }
}

/* 服务端告警（{type:'alert',code,active,message}）：
 *   active=true  黄色横幅；不覆盖更严重的横幅（error/dead/disconnected/restarting/stalled）。
 *   active=false 条件恢复 → 记录失效；若当前正是该告警则回落到基础横幅。 */
function onSimAlert(m) {
  const code = (m && m.code) ? String(m.code) : 'unknown';
  const kind = 'warn:' + code;
  if (!m || !m.active) {
    delete S.activeAlerts[code];
    if (S.sysBannerKind === kind) restoreBaseBanner();
    return;
  }
  S.activeAlerts[code] = m.message || '仿真告警';
  // 更严重者优先：这些横幅弹出时黄色告警不覆盖（不改横幅，但记录为生效中，
  // 待其解除后经 restoreBaseBanner 重新浮现）
  if (S.sysBannerKind === 'error' || S.sysBannerKind === 'dead'
      || S.sysBannerKind === 'disconnected' || S.sysBannerKind === 'restarting'
      || S.sysBannerKind === 'stalled') {
    return;
  }
  showSysBanner(kind, S.activeAlerts[code], false);
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
    setSimState('starting');   // restarting 视同 starting
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
  } else if (v === 'started' || v === 'restarted') {
    S.options.current = m.config || {};
    if (m.config && m.config.init_mode) {
      S.obstacleMode = (m.config.init_mode === 'random') ? 'occupancy' : 'dataset';
    }
    setSimState('running');
    refreshSchemeCurrent();
    btnApplyScheme.disabled = false;
    btnApplyScheme.textContent = '应用并重启仿真';
    if (S.degraded) {
      showSysBanner('warn:degraded', degradedBannerText(), true);
    } else {
      hideSysBanner();
    }
    schemeStatus.textContent = '已按新方案重启：' + describeScheme(m.config);
    setMsg('仿真已按新方案重启：' + describeScheme(m.config));
    setTimeout(() => { setMsg(''); }, 4500);
  } else if (v === 'error' || v === 'restart_failed') {
    btnApplyScheme.disabled = false;
    schemeStatus.textContent = '重启失败：' + (m.message || '未知错误');
    setMsg('重启失败：' + (m.message || '未知错误'));
    showSysBanner('error', '重启失败：' + (m.message || '未知错误'), true);
  } else if (v === 'busy') {
    schemeStatus.textContent = '已有重启进行中，请稍候…';
  } else if (v === 'starting') {
    // 新协议：启动中（与 restarting 等效，但不清理跟随/尾迹——只是生命周期通知）
    setSimState('starting');
    if (btnApplyScheme) btnApplyScheme.disabled = true;
  } else if (v === 'running') {
    setSimState('running');
    if (S.sysBannerKind === 'restarting') {
      if (S.degraded) {
        showSysBanner('warn:degraded', degradedBannerText(), true);
      } else {
        hideSysBanner();
      }
    }
  } else if (v === 'finished') {
    setSimState('finished');
    schemeStatus.textContent = '本轮仿真已完成';
    setMsg('本轮仿真已完成，可调整设置后再次开启');
    setTimeout(() => { setMsg(''); }, 4500);
  } else if (v === 'stopped') {
    clearControlAck('stop');   // status 广播的 stopped 亦视作停止 ack
    setSimState('stopped');
    // 停止后不得残留红色停滞横幅（暂停/停止期间时钟冻结属正常，不是停滞）：
    // 'stalled' → 回落基础横幅；'null'（含被客户端兜底隐藏的 stalled）→ 显示 idle 提示
    if (S.sysBannerKind === 'stalled' || S.sysBannerKind === null) restoreBaseBanner();
  } else if (v === 'idle') {
    setSimState('idle');
    if (S.degraded) {
      // 关键资产缺失优先于普通 idle 提示（横幅持续告知原因）
      showSysBanner('warn:degraded', degradedBannerText(), true);
    } else {
      showSysBanner('idle', '底图已加载（仿真未启动）。点击「开启仿真」启动车辆仿真。', true);
    }
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
    // 既有 stalled 解除 → 回落到基础横幅（可能仍有 降级/黄色告警/idle 需要显示）
    if (S.sysBannerKind === 'stalled') restoreBaseBanner();
  }
}

/* ---------- 仿真生命周期状态（sim_state）驱动控制坞与顶栏状态灯 ----------
 * 状态来源：init 载荷的 sim_state（顶层或 options 内）+ 后续 status 广播 +
 * {type:'stopped'} 回复。未收到 sim_state 时 S.simState 保持 null，
 * applySimStateUI 走「未知」分支，维持接入前的旧按钮行为，不阻塞现有用户。 */
function normalizeSimState(v) {
  if (v === undefined || v === null) return null;
  const s = String(v);
  if (s === 'restarting') return 'starting';   // 旧值兼容：restarting 视同 starting
  if (s === 'starting' || s === 'running' || s === 'finished'
      || s === 'stopped' || s === 'idle') return s;
  return null;
}

function setSimState(v) {
  S.simState = normalizeSimState(v);
  applySimStateUI();
  updateSimChip();
}

/* 控制坞按钮可见性/禁用矩阵：
 *   running               → 显 btnPause+btnStop，隐藏 btnStart
 *   starting              → 三者皆显但全禁，btnStart 文案「启动中…」
 *   finished/stopped/idle → 只显 btnStart（「开启仿真」）
 *   未知                  → 隐藏 btnStart/btnStop，btnPause 维持旧逻辑（disabled=!control） */
function applySimStateUI() {
  if (!btnStart || !btnStop) return;
  const st = S.simState;
  const managed = !!(S.options && S.options.manage_sim);
  if (!st) {
    btnStart.classList.add('hidden');
    btnStop.classList.add('hidden');
    btnPause.classList.remove('hidden');
    btnPause.disabled = !S.control;
    return;
  }
  if (st === 'running') {
    btnStart.classList.add('hidden');
    btnPause.classList.remove('hidden');
    btnStop.classList.remove('hidden');
    btnStart.disabled = true;
    btnPause.disabled = !S.control;
    btnStop.disabled = !S.control;
    btnStart.textContent = '开启仿真';
  } else if (st === 'starting') {
    btnStart.classList.remove('hidden');
    btnPause.classList.remove('hidden');
    btnStop.classList.remove('hidden');
    btnStart.disabled = true;
    btnPause.disabled = true;
    btnStop.disabled = true;
    btnStart.textContent = '启动中…';
  } else { // finished / stopped / idle
    btnStart.classList.remove('hidden');
    btnPause.classList.add('hidden');
    btnStop.classList.add('hidden');
    btnStart.disabled = !(S.control && managed);
    btnStart.textContent = '开启仿真';
  }
}

/* 顶栏仿真状态灯：running=绿「仿真运行中 · N 车」/ starting=黄「启动中…」
 * finished=蓝「本轮已完成」/ stopped=灰「已停止」/ idle=灰「未启动」/ 未知=隐藏 */
function updateSimChip() {
  if (!chipSim || !vSim) return;
  const st = S.simState;
  if (!st) { chipSim.classList.add('hidden'); return; }
  chipSim.classList.remove('hidden');
  chipSim.classList.remove('sim-running', 'sim-starting', 'sim-finished', 'sim-stopped', 'sim-paused');
  const n = (S.frame && S.frame.vehicles) ? S.frame.vehicles.length : 0;
  if (st === 'running' && S.paused) {
    // 暂停态：仿真时间/车辆数已冻结，状态灯转警示黄并显式标注「已暂停」
    // （finished/stopped/idle/starting 不受 paused 影响，仍按各自分支渲染）
    chipSim.classList.add('sim-paused');
    vSim.textContent = '已暂停 · ' + n + ' 车';
  } else if (st === 'running') {
    chipSim.classList.add('sim-running');
    vSim.textContent = '仿真运行中 · ' + n + ' 车';
  } else if (st === 'starting') {
    chipSim.classList.add('sim-starting');
    vSim.textContent = '启动中…';
  } else if (st === 'finished') {
    chipSim.classList.add('sim-finished');
    vSim.textContent = '本轮已完成';
  } else if (st === 'stopped') {
    chipSim.classList.add('sim-stopped');
    vSim.textContent = '已停止';
  } else { // idle
    chipSim.classList.add('sim-stopped');
    vSim.textContent = '未启动';
  }
}

/* 方案面板顶部「当前生效」摘要：init 与 started/restarted 后刷新 */
function refreshSchemeCurrent() {
  if (!schemeCurrent) return;
  const cur = (S.options && S.options.current) || {};
  schemeCurrent.textContent = '当前生效：' + describeScheme(cur);
}

/* {type:'stopped'}：服务端确认停止 → 置状态并提示 */
function onSimStopped(m) {
  clearControlAck('stop');   // 收到停止 ack → 清除超时计时器
  setSimState('stopped');
  // 停止后不得残留红色停滞横幅（stalled → 回落；无横幅 → idle 提示）
  if (S.sysBannerKind === 'stalled' || S.sysBannerKind === null) restoreBaseBanner();
  S.paused = false;
  updatePauseLabel();
  const t = (m && m.message) || '仿真已停止（场内车辆已清空）';
  schemeStatus.textContent = t;
  setMsg(t);
  setTimeout(() => { setMsg(''); }, 4500);
}

/* {type:'stop_failed', message}：停止失败 → 提示并恢复按钮可用（状态未变） */
function onSimStopFailed(m) {
  clearControlAck('stop');   // 收到停止失败 ack → 清除超时计时器
  const t = '停止失败：' + ((m && m.message) || '未知错误');
  schemeStatus.textContent = t;
  setMsg(t);
  applySimStateUI();
}

/* ---------- 控制指令 ack 超时（stop/pause）：一次重发 + 可见反馈 ----------
 * 发出控制指令后启动计时；收到对应 ack 立即清除。超时未 ack → 重发一次并重新计时；
 * 第二次仍超时 → 恢复按钮可用 + setMsg 提示 + console.warn（不把按钮永久卡死）。 */
const CONTROL_ACK_TIMEOUT_MS = 12000;

function clearControlAck(kind) {
  if (S._ctlAck[kind]) { clearTimeout(S._ctlAck[kind]); S._ctlAck[kind] = null; }
  S._ctlAckPayload[kind] = null;   // 停表的同时丢弃冻结载荷，避免陈旧载荷被后续复用
}

function sendControl(kind) {
  if (!S.ws || S.ws.readyState !== 1) return false;
  // 首次发送即「冻结」已序列化的整条消息，供 ack 超时重发逐字节复用。
  // 重发必须逐字节复用首次载荷，否则 toggle 类指令（如 pause）可能在重发时反转语义。
  const payload = (kind === 'stop') ? '{"type":"stop"}'
                                    : JSON.stringify({ type: 'pause', value: !S.paused });
  S.ws.send(payload);
  S._ctlAckPayload[kind] = payload;
  return true;
}

function armControlAck(kind) {
  // 仅重置计时器（不清载荷——载荷需保留，供超时逐字节重发）
  if (S._ctlAck[kind]) { clearTimeout(S._ctlAck[kind]); S._ctlAck[kind] = null; }
  S._ctlAckRetried[kind] = false;
  S._ctlAck[kind] = setTimeout(() => onControlAckTimeout(kind), CONTROL_ACK_TIMEOUT_MS);
}

function onControlAckTimeout(kind) {
  S._ctlAck[kind] = null;
  if (!S._ctlAckRetried[kind]) {
    S._ctlAckRetried[kind] = true;
    console.warn('[webviz] 控制指令 ' + kind + ' ' + (CONTROL_ACK_TIMEOUT_MS / 1000) +
                 's 未收到 ack，重发一次');
    // 重发必须逐字节复用首次载荷，否则 toggle 类指令（如 pause）可能在重发时反转语义。
    // 因此这里只用冻结的字符串调 ws.send(...)，绝不调用 sendControl、也不读取 S.paused。
    const payload = S._ctlAckPayload[kind];
    if (S.ws && S.ws.readyState === 1 && payload) {
      S.ws.send(payload);
      S._ctlAck[kind] = setTimeout(() => onControlAckTimeout(kind), CONTROL_ACK_TIMEOUT_MS);
    } else {
      console.warn('[webviz] 控制指令 ' + kind + ' 重发失败：连接不可用或载荷缺失');
      S._ctlAckPayload[kind] = null;
      if (kind === 'stop') btnStop.disabled = false; else btnPause.disabled = false;
      setMsg('与桥的连接不可用，请等待重连后重试');
    }
    return;
  }
  // 第二次仍超时：放弃等待，恢复按钮可用并给出可见反馈
  console.warn('[webviz] 控制指令 ' + kind + ' 重发后仍无 ack，放弃等待');
  S._ctlAckPayload[kind] = null;   // 放弃等待：丢弃冻结载荷
  if (kind === 'stop') {
    btnStop.disabled = false;
    setMsg('桥未响应停止请求，请检查连接后重试');
  } else {
    btnPause.disabled = false;
    setMsg('桥未响应暂停请求，请检查连接后重试');
  }
}

/* ---------- 页内非阻塞确认框（替代原生阻塞式确认弹窗，避免阻塞事件循环） ----------
 * askConfirm(text) 返回 Promise<boolean>：确定=true，取消=false。Enter=确定，Esc=取消。 */
let _confirmResolve = null;

function askConfirm(text) {
  return new Promise((resolve) => {
    if (!confirmModal) { resolve(false); return; }
    _confirmResolve = resolve;
    if (confirmText) confirmText.textContent = text || '';
    confirmModal.classList.remove('hidden');
    if (btnConfirmOk) { try { btnConfirmOk.focus(); } catch (e) {} }
  });
}

function closeConfirm(result) {
  if (confirmModal) confirmModal.classList.add('hidden');
  const r = _confirmResolve;
  _confirmResolve = null;
  if (r) r(result);
}

let hoverTipVid = 0;
function updatePauseLabel() { btnPause.textContent = S.paused ? '恢复' : '暂停'; }

function fitView() {
  // 优先按「全内容边界」（服务端下发，含边距）适配——保证全场都在视图内
  const b = S.init && S.init.content_bounds;
  if (b && b.length === 4) {
    const w = Math.max(b[2] - b[0], 1), h = Math.max(b[3] - b[1], 1);
    const z = Math.min(W / (w * S.scale), H / (h * S.scale)) * 0.94;
    S.view.zoom = z || 1;
    const sz = S.scale * S.view.zoom;
    S.view.x = (W - w * sz) / 2 - b[0] * sz;
    S.view.y = (H - h * sz) / 2 - (S.mapH - b[3]) * sz;
    return;
  }
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
  const b = (S.init && S.init.content_bounds) || [0, 0, S.mapW, S.mapH];
  const gx0 = Math.floor(b[0] / 10) * 10 - 10, gx1 = Math.ceil(b[2] / 10) * 10 + 10;
  const gy0 = Math.floor(b[1] / 10) * 10 - 10, gy1 = Math.ceil(b[3] / 10) * 10 + 10;
  ctx.beginPath();
  for (let x = gx0; x <= gx1; x += 10) { ctx.moveTo(x, gy0); ctx.lineTo(x, gy1); }
  for (let y = gy0; y <= gy1; y += 10) { ctx.moveTo(gx0, y); ctx.lineTo(gx1, y); }
  ctx.stroke();
  // 坐标文字（屏幕空间）
  setScreen();
  ctx.fillStyle = th.gridText;
  ctx.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
  for (let x = gx0; x <= gx1; x += 10) {
    const [sx, sy] = w2s(x, gy0);
    ctx.fillText(String(x), sx + 3, sy - 5);
  }
  for (let y = gy0; y <= gy1; y += 10) {
    const [sx, sy] = w2s(gx0, y);
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
  updateSimChip();   // running 时「· N 车」随最新帧刷新
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
    ctx.globalAlpha = S.theme === 'dark' ? 0.6 : 0.85;
    if (S.bgT) {
      const t = S.bgT;
      if (t.affine) {
        // 完整仿射：保留图像自带的微小旋转交叉项
        //   x' = a*px + c*py + e ; y' = b*px + d*py + f（canvas transform 参数序为 a,b,c,d,e,f）
        const A = t.affine;
        ctx.transform(A.a, A.b, A.c, A.d, A.e, A.f);
      } else {
        // 退化形式：xr = -kx*py + cx ; yr = -ky*px + cy
        ctx.transform(0, -t.ky, -t.kx, 0, t.cx, t.cy);
      }
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
    ctx.lineWidth = 1.5 / (S.scale * v.zoom);
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
      // 两者叠加渲染：占用车位描法负责「初始随机占用」等占位车，
      // staticGarage 负责真实入库车辆的显式几何；同一车位不会重复绘制（buildOccPath 已去重）。
      // 旧写法是二选一，导致首辆车入库后 staticGarage 变非空，初始占用车整体消失。
      drawOccupancyCars(th, v);
      if (S.staticGarage && S.staticGarage.length) drawStaticObstacles(th, v);
    }
    // 硬障碍物：仅当「该障碍物确实被仿真读取」时才渲染。
    //   经验地图（DJI/车辆障碍，hasExperience=true）：随机模式不读取障碍 → 仅 dataset(经验) 显示；
    //   固定障碍图（JTH 等，hasExperience=false）：障碍始终为固定物，任何模式都显示。
    const _sceneObsActive = !(S.hasExperience && S.obstacleMode === 'occupancy');
    if (P.obstacles && _sceneObsActive) {
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
    updatePathHint();
  }
});
cv.addEventListener('mouseleave', () => { S.hover = 0; updateTooltip(); });

btnPause.addEventListener('click', () => {
  if (btnPause.classList.contains('hidden')) return;   // 状态驱动下隐藏时（含空格键）不触发
  if (!S.control || !S.ws || S.ws.readyState !== 1) return;
  sendControl('pause');     // {type:'pause', value:!S.paused}
  armControlAck('pause');   // 12s ack 超时 + 一次重发
  updateSimChip();          // 暂停态即将切换：立即刷新状态灯
});
btnStart.addEventListener('click', () => {
  // 与方案面板「开启仿真」同路径：按当前面板配置启动仿真（applyScheme 内含托管/连接守卫）
  applyScheme();
});
btnStop.addEventListener('click', async () => {
  if (!S.control || !S.ws || S.ws.readyState !== 1) return;
  // 页内非阻塞确认（替代原生阻塞式确认弹窗，避免阻塞事件循环导致 stop 指令丢失）
  if (!await askConfirm('将清空场内车辆并结束本轮仿真，确定停止？')) return;
  if (!S.control || !S.ws || S.ws.readyState !== 1) return;   // 确认期间连接可能已断
  btnStop.disabled = true;   // 等待 {type:'stopped'} / {type:'stop_failed'} / status 广播刷新
  sendControl('stop');
  armControlAck('stop');     // 12s ack 超时 + 一次重发；第二次超时恢复按钮可用
});
btnReset.addEventListener('click', () => { S.follow = 0; sendFocus(0); updatePathHint(); fitView(); });
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
  // 底图 DJI（共享）时，实际加载所选 DJI_XXXX 占用数据集；其余底图原样提交
  const _base = scMap.value;
  const _mapVal = (_base === 'DJI' && scOccDataset && scOccDataset.value)
    ? scOccDataset.value : _base;
  setMsg('正在加载底图：' + _mapVal + ' …');
  S.ws.send(JSON.stringify({ type: 'switch_map', map: _mapVal }));
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

function confirmModalOpen() {
  return !!(confirmModal && !confirmModal.classList.contains('hidden'));
}
if (btnConfirmOk) btnConfirmOk.addEventListener('click', () => closeConfirm(true));
if (btnConfirmCancel) btnConfirmCancel.addEventListener('click', () => closeConfirm(false));
if (confirmModal) {
  // 点击遮罩空白处 = 取消
  confirmModal.addEventListener('click', (e) => { if (e.target === confirmModal) closeConfirm(false); });
}

window.addEventListener('keydown', (e) => {
  // 确认框打开时优先处理 Enter/Esc，并吞掉其它按键（避免空格误触暂停）
  if (confirmModalOpen()) {
    if (e.key === 'Enter') { e.preventDefault(); closeConfirm(true); }
    else if (e.key === 'Escape') { e.preventDefault(); closeConfirm(false); }
    return;
  }
  if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA')) return;
  if (e.key === ' ') { e.preventDefault(); btnPause.click(); }
  else if (e.key === 'h' || e.key === 'H') {
    S.presentation = !S.presentation;
    document.body.classList.toggle('presentation', S.presentation);
  } else if (e.key === 'r' || e.key === 'R') { S.follow = 0; sendFocus(0); updatePathHint(); fitView(); }
  else if (e.key === 'b' || e.key === 'B') {
    // 底图对比：快速切换底图显隐，核对矢量与底图重合度
    S.layers.baseMap = !S.layers.baseMap;
    const cb = document.querySelector('#layers input[data-layer="baseMap"]');
    if (cb) cb.checked = S.layers.baseMap;
    try { localStorage.setItem('parksim_layers_v5', JSON.stringify(S.layers)); } catch (e2) {}
  }
  else if (e.key === 't' || e.key === 'T') cycleTheme();
  else if (e.key === 'Escape') {
    if (!schemeEl.classList.contains('hidden')) { schemeEl.classList.add('hidden'); }
    else if (S.presentation) { S.presentation = false; document.body.classList.remove('presentation'); }
    else { S.follow = 0; sendFocus(0); updatePathHint(); }
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
