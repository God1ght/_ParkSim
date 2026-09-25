/* =============================================================================
 * ParkSim · 指标 + 事件可视化模块（charts.js）
 * -----------------------------------------------------------------------------
 * 挂载点（由页面提供，本文件只往里建 DOM，不依赖任何外部 element id）：
 *   #metricsViz  —— 左侧「运行指标」面板内
 *   #eventsViz   —— 右侧「冲突事件」面板内
 *
 * 依赖：**零第三方依赖**。三张时序图（车辆状态堆叠面积 / 泊位占用率折线 /
 *       事件速率柱状）全部为原生 Canvas 2D 自绘，与页面主画布 #cv 同一套技术，
 *       风格统一；KPI sparkline 用内联 SVG，类型分布条用纯 CSS。
 *       不读 window.echarts，也不需要 index.html 引入任何 vendor 脚本。
 *
 * 对外接口（契约，不可改）：
 *   window.ParkSimCharts = { init, onFrame, onOccupancy, onAlert, resize, reset,
 *                            events, _dbg }
 *
 * 数据口径：
 *   采样 2Hz（仿真时刻 t 增量 >= 0.5s），环形缓冲 600 点（≈5 分钟）。
 *   绘制只在采样点更新时触发（2Hz），不跟 20Hz 帧重绘；悬停时按需重绘。
 *   暂停判定只看 frame.running 与 t（**不能用 seq**：暂停时 seq 仍在增长）。
 * ========================================================================== */
(function (global) {
  'use strict';

  /* ---------------------------------------------------------------- 常量 -- */
  var SAMPLE_STEP = 0.5;       // 采样间隔（仿真秒）
  var MAX_SAMPLES = 600;       // 采样环形缓冲长度
  var MAX_EVENTS = 200;        // 事件保留条数
  var SPARK_POINTS = 60;       // sparkline 取点数
  var BUCKET_SEC = 10;         // 事件速率分桶宽度（仿真秒）
  var MAX_BUCKETS = 24;        // 速率图最多显示桶数
  var SPOT_TOTAL = 269;        // 泊位总数：无 occupancy 时的兜底分母
  var TL_ROWS = 50;            // 时间轴最多渲染行数
  var ALERT_DEDUPE_SEC = 5;    // 同一告警去重窗口（仿真秒）
  var RENDER_MIN_MS = 250;     // 渲染节流（墙钟毫秒）

  // 自绘 Canvas 布局：与原先 ECharts grid 取值一致（紧凑）
  var PAD = { left: 34, right: 10, top: 10, bottom: 20 };
  var CHART_H = { state: 130, occ: 96, rate: 96 };
  var AXIS_FONT = '9px Inter, -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif';
  var MIN_TICK_GAP = 4;        // 相邻 x 轴标签之间至少留出的空白（px）
  var DPR_CAP = 3;             // devicePixelRatio 上限，避免超高分屏画布过大

  /* 颜色表 C / EV_COLOR 是**可变**的：refreshTheme() 会就地改写，实现主题切换。
   * 下面这套暗色 hex 是视觉规范值，同时作为取不到 CSS 变量时的兜底默认值。 */
  var C = {
    drive: '#38d6c0', park: '#f5b04d', brake: '#e06060', done: '#707e90',
    accent: '#4cc2ff', ok: '#3ddbb4', warn: '#e8b224', bad: '#e06060',
    text: '#d8e3ef', sub: '#8296ab',
    grid: 'rgba(255,255,255,.06)', axisPtr: 'rgba(255,255,255,.18)'
  };
  var DEFAULT_C = {
    drive: C.drive, park: C.park, brake: C.brake, done: C.done,
    accent: C.accent, ok: C.ok, warn: C.warn, bad: C.bad,
    text: C.text, sub: C.sub
  };

  var EV_COLOR = {
    enter: '#4cc2ff', parked: '#3ddbb4', exit: '#8296ab',
    alert: '#e8b224', conflict: '#e06060'
  };
  var DEFAULT_EV = {
    enter: EV_COLOR.enter, parked: EV_COLOR.parked, exit: EV_COLOR.exit,
    alert: EV_COLOR.alert, conflict: EV_COLOR.conflict
  };
  var EV_LABEL = {
    enter: '进场', parked: '泊车', exit: '出场', alert: '告警', conflict: '冲突'
  };
  var DIST_ORDER = ['enter', 'parked', 'exit', 'alert', 'conflict'];
  // 分布条图例：规格要求「进场/泊车/出场/告警」四项常驻，冲突仅在有计数时出现
  var DIST_ALWAYS = { enter: '进场', parked: '泊车', exit: '出场', alert: '告警' };

  // 三张图的系列定义（自绘图例 / tooltip 共用）：n 名称，c 颜色，d 小数位，u 单位
  var SERIES = {
    state: [
      { n: '行驶', c: C.drive, d: 0 },
      { n: '泊车', c: C.park, d: 0 },
      { n: '制动', c: C.brake, d: 0 },
      { n: '完成', c: C.done, d: 0 }
    ],
    occ: [{ n: '占用率', c: C.accent, d: 1, u: '%' }],
    rate: [{ n: '事件数', c: C.accent, d: 0 }]
  };
  var STACK_KEYS = ['drive', 'park', 'brake', 'done'];

  // cKey：运行时按 C[cKey] 取色（主题切换后自动跟随）；color 仅作建 DOM 时的初值
  var KPI_DEF = [
    { key: 'total',     label: '在场车辆',   field: 'total',     cKey: 'accent', color: C.accent,  unit: '辆',  digits: 0 },
    { key: 'occRate',   label: '泊位占用率', field: 'occRate',   cKey: 'drive',  color: '#38d6c0', unit: '%',   digits: 1 },
    { key: 'avgV',      label: '平均车速',   field: 'avgV',      cKey: 'park',   color: C.park,    unit: 'm/s', digits: 1 },
    { key: 'parkedCum', label: '已完成泊车', field: 'parkedCum', cKey: 'ok',     color: C.ok,      unit: '次',  digits: 0 }
  ];

  /* ---------------------------------------------------------------- 状态 -- */
  var S = {
    inited: false,
    samples: [],          // {t, drive, park, brake, done, total, occRate, avgV, parkedCum}
    events: [],           // 对外暴露，元素 {t, type, text}（保持同一数组引用）
    prevIds: {},          // { vehicleId: true }
    prevParked: {},       // { vehicleId: spot }
    conflictActive: {},   // { "minId:maxId": true }：当前处于互等的车辆对
    lastAlert: {},        // { key: t }：告警去重
    parkedCum: 0,         // 累计完成泊车次数
    primed: false,        // 首帧只建基线、不产事件
    lastSampleT: null,
    lastT: 0,
    lastOcc: null,
    occRate: 0,
    charts: { state: null, occ: null, rate: null },
    dom: {
      metricsHost: null, eventsHost: null,
      kpi: [],
      stateEl: null, occEl: null, rateEl: null,
      timeline: null, distBar: null, distLegend: null
    },
    dirty: { kpi: true, state: true, occ: true, timeline: true, rate: true, dist: true },
    lastRenderWall: 0,
    lastInitTry: 0,
    winBound: false,         // 是否已绑定 window resize（只绑一次）
    theme: 'dark'            // 当前主题（applyTheme 写入）
  };

  /* ------------------------------------------------------------ 小工具 -- */
  function num(v) { return typeof v === 'number' && isFinite(v) ? v : NaN; }

  function idOf(v) {
    var n = Number(v);
    return (typeof n === 'number' && isFinite(n)) ? n : null;
  }

  function esc(s) {
    return String(s == null ? '' : s).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  }

  function hasDom() {
    return typeof global.document !== 'undefined' && !!global.document;
  }

  function el(tag, cls) {
    var d = global.document.createElement(tag);
    if (cls) d.className = cls;
    return d;
  }

  function elText(tag, cls, txt) {
    var d = el(tag, cls);
    d.textContent = txt;
    return d;
  }

  function svgNode(tag) {
    return global.document.createElementNS('http://www.w3.org/2000/svg', tag);
  }

  function legendHtml(items) {
    var h = '';
    for (var i = 0; i < items.length; i++) {
      h += '<span class="ch-lg"><i style="background:' + items[i].color + '"></i>'
        + esc(items[i].label) + '</span>';
    }
    return h;
  }

  /* --------------------------------------------------------- 建内部 DOM -- */
  function buildMetricsDom(host) {
    var d = S.dom;
    host.innerHTML = '';
    var wrap = el('ch-wrap');

    /* 1) KPI 卡 2x2 */
    var sec1 = el('ch-sec');
    sec1.appendChild(elText('div', 'ch-sec-title', '关键指标 · 近 5 分钟'));
    var grid = el('ch-kpi-grid');
    d.kpi = [];
    for (var i = 0; i < KPI_DEF.length; i++) {
      var def = KPI_DEF[i];
      var card = el('ch-kpi');
      card.appendChild(elText('div', 'ch-kpi-label', def.label));
      var val = el('div', 'ch-kpi-value');
      var numEl = el('span', 'ch-kpi-num');
      val.appendChild(numEl);
      val.appendChild(elText('span', 'ch-kpi-unit', def.unit));
      card.appendChild(val);

      var poly = null;
      if (typeof global.document.createElementNS === 'function') {
        var svg = svgNode('svg');
        svg.setAttribute('class', 'ch-spark');
        svg.setAttribute('viewBox', '0 0 100 24');
        svg.setAttribute('preserveAspectRatio', 'none');
        poly = svgNode('polyline');
        poly.setAttribute('fill', 'none');
        poly.setAttribute('stroke', def.color);
        poly.setAttribute('stroke-width', '1.4');
        poly.setAttribute('stroke-linejoin', 'round');
        poly.setAttribute('stroke-linecap', 'round');
        poly.setAttribute('vector-effect', 'non-scaling-stroke');
        svg.appendChild(poly);
        card.appendChild(svg);
      }
      grid.appendChild(card);
      d.kpi.push({ def: def, numEl: numEl, poly: poly });
    }
    sec1.appendChild(grid);
    wrap.appendChild(sec1);

    /* 2) 车辆状态分布（堆叠面积图） */
    var sec2 = el('ch-sec');
    var t2 = el('div', 'ch-sec-title');
    t2.appendChild(elText('span', null, '车辆状态分布'));
    var lg2 = el('div', 'ch-legend');
    lg2.innerHTML = legendHtml([
      { color: C.drive, label: '行驶' }, { color: C.park, label: '泊车' },
      { color: C.brake, label: '制动' }, { color: C.done, label: '完成' }
    ]);
    t2.appendChild(lg2);
    sec2.appendChild(t2);
    d.stateEl = S.charts.state = makeChart('state', sec2, SERIES.state);
    wrap.appendChild(sec2);

    /* 3) 泊位占用率（折线图） */
    var sec3 = el('ch-sec');
    var t3 = el('div', 'ch-sec-title');
    t3.appendChild(elText('span', null, '泊位占用率'));
    var lg3 = el('div', 'ch-legend');
    lg3.innerHTML = legendHtml([{ color: C.accent, label: '占用率 %' }]);
    t3.appendChild(lg3);
    sec3.appendChild(t3);
    d.occEl = S.charts.occ = makeChart('occ', sec3, SERIES.occ);
    wrap.appendChild(sec3);

    host.appendChild(wrap);
  }

  function buildEventsDom(host) {
    var d = S.dom;
    host.innerHTML = '';
    var wrap = el('ch-wrap');

    /* 1) 事件时间轴 */
    var sec1 = el('ch-sec');
    sec1.appendChild(elText('div', 'ch-sec-title', '事件时间轴 · 最新在上'));
    d.timeline = el('div', 'ch-tl');
    sec1.appendChild(d.timeline);
    wrap.appendChild(sec1);

    /* 2) 事件速率（柱状图，10s/桶） */
    var sec2 = el('ch-sec');
    var t2 = el('div', 'ch-sec-title');
    t2.appendChild(elText('span', null, '事件速率 · 10s/桶'));
    var lg2 = el('div', 'ch-legend');
    lg2.innerHTML = legendHtml([{ color: C.accent, label: '事件数' }]);
    t2.appendChild(lg2);
    sec2.appendChild(t2);
    d.rateEl = S.charts.rate = makeChart('rate', sec2, SERIES.rate);
    wrap.appendChild(sec2);

    /* 3) 类型分布条 */
    var sec3 = el('ch-sec');
    sec3.appendChild(elText('div', 'ch-sec-title', '类型分布'));
    d.distBar = el('div', 'ch-bar');
    sec3.appendChild(d.distBar);
    d.distLegend = el('div', 'ch-legend ch-dist-legend');
    sec3.appendChild(d.distLegend);
    wrap.appendChild(sec3);

    host.appendChild(wrap);
  }

  /* ------------------------------------------- Canvas 2D 自绘图表引擎 -- */
  /* 每张图 = 一个独立 <canvas>（绝不画到页面主画布 #cv 上）+ 一个 HTML tooltip。
   * 零第三方依赖：刻度、网格、坐标轴、图例、tooltip 全部自己算、自己画。 */

  function makeChart(key, host, series) {
    var wrap = el('div', 'ch-chart ch-chart-' + key);
    var cv = el('canvas', 'ch-canvas');
    wrap.appendChild(cv);
    var tip = el('div', 'ch-tip hidden');
    wrap.appendChild(tip);
    host.appendChild(wrap);
    var ch = {
      key: key, series: series, wrap: wrap, canvas: cv, tip: tip,
      cssW: 0, cssH: CHART_H[key], dpr: 0, ctx: null,
      hoverIdx: -1, items: [], ticks: []
    };
    bindHover(ch);
    return ch;
  }

  /* devicePixelRatio：画布按物理像素开，CSS 尺寸仍按逻辑像素设。
   * 每次绘制前用 setTransform(dpr,0,0,dpr,0,0) 重置变换（幂等，不会累积缩放）。 */
  function sizeCanvas(ch) {
    var w = ch.wrap.clientWidth || 0;
    if (!(w > 0)) return false;                 // 面板隐藏（宽度 0）→ 跳过绘制
    var h = ch.cssH;
    var dpr = global.devicePixelRatio || 1;
    if (!isFinite(dpr) || dpr < 1) dpr = 1;
    if (dpr > DPR_CAP) dpr = DPR_CAP;
    var pw = Math.round(w * dpr), ph = Math.round(h * dpr);
    if (ch.canvas.width !== pw || ch.canvas.height !== ph) {
      ch.canvas.width = pw;
      ch.canvas.height = ph;
    }
    ch.canvas.style.width = w + 'px';
    ch.canvas.style.height = h + 'px';
    var ctx = null;
    try { ctx = ch.canvas.getContext('2d'); } catch (e) { ctx = null; }
    if (!ctx) return false;
    ch.cssW = w; ch.dpr = dpr; ch.ctx = ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.font = AXIS_FONT;
    return true;
  }

  function fmtT(v) { return Math.round(v) + 's'; }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }

  function plotBox(ch) {
    return {
      L: PAD.left, R: ch.cssW - PAD.right,
      T: PAD.top, B: ch.cssH - PAD.bottom
    };
  }

  /* 网格线 + 轴标签（对应 ECharts 的 splitLine / axisLabel） */
  function drawGrid(ch, box, yOf, yVals, yLabels, xTicks) {
    var ctx = ch.ctx, i, yy, x;
    ctx.lineWidth = 1;
    ctx.font = AXIS_FONT;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (i = 0; i < yVals.length; i++) {
      yy = Math.round(yOf(yVals[i])) + 0.5;
      ctx.strokeStyle = C.grid;
      ctx.beginPath();
      ctx.moveTo(box.L, yy);
      ctx.lineTo(box.R, yy);
      ctx.stroke();
      ctx.fillStyle = C.sub;
      ctx.fillText(yLabels[i], box.L - 5, yy);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (i = 0; i < xTicks.length; i++) {
      // fitTicks() 里已按同一公式夹过一次，这里再夹一次是幂等的（防御性）
      x = clamp(xTicks[i].x, box.L + 8, box.R - 8);
      ctx.fillStyle = C.sub;
      ctx.fillText(xTicks[i].label, x, box.B + 5);
    }
  }

  /* 标签宽度：优先 ctx.measureText，不可用时按 9px 字体粗估 */
  function labelWidth(ctx, text) {
    if (ctx && typeof ctx.measureText === 'function') {
      try {
        var m = ctx.measureText(text);
        if (m && isFinite(m.width) && m.width > 0) return m.width;
      } catch (e) { /* 忽略，走估算 */ }
    }
    return String(text).length * 5.6;
  }

  /* 按实测宽度剔除会重叠 / 出界的 x 轴刻度：
   * 任意两个相邻标签之间至少留 MIN_TICK_GAP 空白；画不下的直接丢弃。
   * prioLast=true 时，若「最后一个刻度」与前一个冲突，丢前一个保最后一个
   * （速率图最右侧是最新时刻，信息价值更高）。 */
  function fitTicks(ch, box, ticks, prioLast) {
    var out = [], i, t, x, w, last, prev2;
    for (i = 0; i < ticks.length; i++) {
      t = ticks[i];
      x = clamp(t.x, box.L + 8, box.R - 8);
      w = labelWidth(ch.ctx, t.label);
      if (x - w / 2 < 1 || x + w / 2 > ch.cssW - 1) continue;   // 出界 → 丢
      last = out.length ? out[out.length - 1] : null;
      if (!last || (x - w / 2) - (last.x + last.w / 2) >= MIN_TICK_GAP) {
        out.push({ x: x, label: t.label, w: w });
        continue;
      }
      if (prioLast && i === ticks.length - 1) {
        prev2 = out.length > 1 ? out[out.length - 2] : null;
        if (!prev2 || (x - w / 2) - (prev2.x + prev2.w / 2) >= MIN_TICK_GAP) {
          out.pop();
          out.push({ x: x, label: t.label, w: w });
        }
      }
    }
    ch.ticks = out;
    return out;
  }

  function drawHint(ch, box, text) {
    var ctx = ch.ctx;
    ctx.font = AXIS_FONT;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = C.sub;
    ctx.globalAlpha = 0.6;
    ctx.fillText(text, (box.L + box.R) / 2, (box.T + box.B) / 2);
    ctx.globalAlpha = 1;
  }

  function drawHoverLine(ch, box, x) {
    var ctx = ch.ctx;
    ctx.strokeStyle = C.axisPtr;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(Math.round(x) + 0.5, box.T);
    ctx.lineTo(Math.round(x) + 0.5, box.B);
    ctx.stroke();
  }

  // y 轴刻度取 0 / step / 2*step，step 至少为 1（整数计数图不出现小数刻度）
  function yStep(maxVal) { return Math.max(1, Math.ceil(maxVal / 2)); }

  function xTicksValue(box, n, tMin, span) {
    var out = [], w = box.R - box.L, i, f;
    if (n <= 0) return out;
    if (n === 1) return [{ x: (box.L + box.R) / 2, label: fmtT(tMin) }];
    var cnt = w > 150 ? 3 : 2;
    for (i = 0; i < cnt; i++) {
      f = i / (cnt - 1);
      out.push({ x: box.L + f * w, label: fmtT(tMin + f * span) });
    }
    return out;
  }

  /* 图 1：车辆状态堆叠面积图（4 系列，x 轴为仿真时刻，自底向上 行驶→完成） */
  function drawState(ch) {
    var box = plotBox(ch), ctx = ch.ctx, arr = S.samples, n = arr.length, i, k, v;
    var cum = [[], [], [], []], items = [], maxTotal = 0, acc, vals;
    for (i = 0; i < n; i++) {
      acc = 0; vals = [];
      for (k = 0; k < 4; k++) {
        v = Number(arr[i][STACK_KEYS[k]]);
        if (!isFinite(v) || v < 0) v = 0;
        acc += v;
        cum[k][i] = acc;
        vals.push(v);
      }
      if (acc > maxTotal) maxTotal = acc;
      items.push({ label: fmtT(arr[i].t), vals: vals, total: acc });
    }
    ch.items = items;

    var step = yStep(maxTotal), yMax = step * 2;
    var tMin = n ? arr[0].t : 0, tMax = n ? arr[n - 1].t : 0;
    var span = tMax - tMin;
    if (!(span > 0)) span = 1;
    var yOf = function (val) { return box.B - val / yMax * (box.B - box.T); };
    var xOf = function (idx) {
      return n > 1 ? box.L + (arr[idx].t - tMin) / span * (box.R - box.L) : (box.L + box.R) / 2;
    };

    drawGrid(ch, box, yOf, [0, step, yMax], ['0', String(step), String(yMax)],
      fitTicks(ch, box, xTicksValue(box, n, tMin, span), false));

    if (n < 2) { drawHint(ch, box, n ? '采集中…' : '暂无数据'); return; }

    for (k = 0; k < 4; k++) {
      ctx.beginPath();
      ctx.moveTo(xOf(0), yOf(cum[k][0]));
      for (i = 1; i < n; i++) ctx.lineTo(xOf(i), yOf(cum[k][i]));   // 上边界正向
      for (i = n - 1; i >= 0; i--) ctx.lineTo(xOf(i), yOf(k === 0 ? 0 : cum[k - 1][i])); // 下边界逆向
      ctx.closePath();
      ctx.fillStyle = SERIES.state[k].c;
      ctx.fill();
    }
    if (ch.hoverIdx >= 0) drawHoverLine(ch, box, xOf(ch.hoverIdx));
  }

  /* 图 2：泊位占用率折线图（y 轴固定 0–100%） */
  function drawOcc(ch) {
    var box = plotBox(ch), ctx = ch.ctx, arr = S.samples, n = arr.length, i, v;
    var items = [];
    for (i = 0; i < n; i++) {
      v = Number(arr[i].occRate);
      if (!isFinite(v) || v < 0) v = 0;
      if (v > 100) v = 100;
      items.push({ label: fmtT(arr[i].t), vals: [v] });
    }
    ch.items = items;

    var yOf = function (val) { return box.B - val / 100 * (box.B - box.T); };
    var tMin = n ? arr[0].t : 0, tMax = n ? arr[n - 1].t : 0;
    var span = tMax - tMin;
    if (!(span > 0)) span = 1;
    var xOf = function (idx) {
      return n > 1 ? box.L + (arr[idx].t - tMin) / span * (box.R - box.L) : (box.L + box.R) / 2;
    };

    drawGrid(ch, box, yOf, [0, 50, 100], ['0%', '50%', '100%'],
      fitTicks(ch, box, xTicksValue(box, n, tMin, span), false));

    if (n < 2) { drawHint(ch, box, n ? '采集中…' : '暂无数据'); return; }

    // 面积（低透明）+ 折线
    ctx.beginPath();
    ctx.moveTo(xOf(0), yOf(items[0].vals[0]));
    for (i = 1; i < n; i++) ctx.lineTo(xOf(i), yOf(items[i].vals[0]));
    ctx.lineTo(xOf(n - 1), box.B);
    ctx.lineTo(xOf(0), box.B);
    ctx.closePath();
    ctx.globalAlpha = 0.14;
    ctx.fillStyle = C.accent;
    ctx.fill();
    ctx.globalAlpha = 1;

    ctx.beginPath();
    ctx.moveTo(xOf(0), yOf(items[0].vals[0]));
    for (i = 1; i < n; i++) ctx.lineTo(xOf(i), yOf(items[i].vals[0]));
    ctx.strokeStyle = C.accent;
    ctx.lineWidth = 1.4;
    ctx.stroke();

    if (ch.hoverIdx >= 0) drawHoverLine(ch, box, xOf(ch.hoverIdx));
  }

  /* 图 3：事件速率柱状图（10 秒分桶，最多 24 桶） */
  function drawRate(ch) {
    var box = plotBox(ch), ctx = ch.ctx, i, b, cnt;
    var counts = {}, keys = [];
    for (i = 0; i < S.events.length; i++) {
      b = Math.floor(Number(S.events[i].t) / BUCKET_SEC);
      if (!isFinite(b)) continue;
      if (counts[b] === undefined) { counts[b] = 0; keys.push(b); }
      counts[b] += 1;
    }
    var items = [];
    if (keys.length) {
      keys.sort(function (a, c) { return a - c; });
      var hi = keys[keys.length - 1];
      var lo = Math.max(keys[0], hi - MAX_BUCKETS + 1);
      for (b = lo; b <= hi; b++) {
        cnt = counts[b] === undefined ? 0 : counts[b];
        items.push({ label: (b * BUCKET_SEC) + 's', vals: [cnt] });
      }
    }
    ch.items = items;

    var n = items.length, maxC = 0;
    for (i = 0; i < n; i++) if (items[i].vals[0] > maxC) maxC = items[i].vals[0];
    var step = yStep(maxC), yMax = step * 2;
    var yOf = function (val) { return box.B - val / yMax * (box.B - box.T); };

    // x 轴：刻度落在柱心。先按可用宽度粗抽稀，再交给 fitTicks 按实测文字宽度
    // 剔除仍会重叠 / 出界的刻度（粗抽稀 + 强制补最后一根柱会造成叠字，已修）
    var plotW = box.R - box.L;
    var slot = n ? plotW / n : 0;
    var maxLab = Math.max(2, Math.floor(plotW / 40));
    var every = n > maxLab ? Math.ceil(n / maxLab) : 1;
    var cand = [], lastIdx = -1;
    for (i = 0; i < n; i++) {
      if (i % every === 0) {
        cand.push({ x: box.L + slot * (i + 0.5), label: items[i].label });
        lastIdx = i;
      }
    }
    if (n && lastIdx !== n - 1) {
      cand.push({ x: box.L + slot * (n - 1 + 0.5), label: items[n - 1].label });
    }
    drawGrid(ch, box, yOf, [0, step, yMax], ['0', String(step), String(yMax)],
      fitTicks(ch, box, cand, true));

    if (!n) { drawHint(ch, box, '暂无事件'); return; }

    var bw = Math.min(12, Math.max(2, slot * 0.6));
    for (i = 0; i < n; i++) {
      var x = box.L + slot * (i + 0.5) - bw / 2;
      var y = yOf(items[i].vals[0]);
      ctx.fillStyle = (i === ch.hoverIdx) ? C.text : C.accent;
      ctx.globalAlpha = (ch.hoverIdx >= 0 && i !== ch.hoverIdx) ? 0.6 : 1;
      ctx.fillRect(x, y, bw, box.B - y);
      ctx.globalAlpha = 1;
    }
  }

  function drawChart(ch) {
    if (!ch) return;
    if (!sizeCanvas(ch)) return;     // 宽度为 0（面板隐藏）直接跳过
    try {
      if (ch.key === 'state') drawState(ch);
      else if (ch.key === 'occ') drawOcc(ch);
      else drawRate(ch);
    } catch (err) {
      /* 绘制异常 → 静默，绝不影响仿真主流程 */
    }
  }

  /* 悬停：命中最近的数据点 → 重绘（带竖直准星）+ 显示 HTML tooltip */
  function bindHover(ch) {
    var cv = ch.canvas;
    if (!cv || typeof cv.addEventListener !== 'function') return;
    cv.addEventListener('mousemove', function (ev) {
      var n = ch.items ? ch.items.length : 0;
      if (!n || !(ch.cssW > 0)) return;
      var rect = (typeof cv.getBoundingClientRect === 'function') ? cv.getBoundingClientRect() : null;
      var mx = ev.clientX - (rect ? rect.left : 0);
      var my = ev.clientY - (rect ? rect.top : 0);
      var box = plotBox(ch);
      var idx;
      if (ch.key === 'rate') {
        var slot = (box.R - box.L) / n;
        idx = slot > 0 ? Math.floor((mx - box.L) / slot) : 0;
      } else {
        var w = box.R - box.L;
        idx = w > 0 ? Math.round((mx - box.L) / w * (n - 1)) : 0;
      }
      idx = clamp(idx, 0, n - 1);
      if (idx !== ch.hoverIdx) { ch.hoverIdx = idx; drawChart(ch); }
      showTip(ch, mx, my);
    });
    cv.addEventListener('mouseleave', function () {
      if (ch.hoverIdx === -1) return;
      ch.hoverIdx = -1;
      hideTip(ch);
      drawChart(ch);
    });
  }

  function showTip(ch, mx, my) {
    var tip = ch.tip;
    var item = (ch.items && ch.hoverIdx >= 0) ? ch.items[ch.hoverIdx] : null;
    if (!item) { hideTip(ch); return; }
    var html = '<div class="ch-tip-t">' + esc(item.label) + '</div>';
    for (var i = 0; i < ch.series.length; i++) {
      var s = ch.series[i], v = item.vals[i];
      html += '<div class="ch-tip-r"><i style="background:' + s.c + '"></i>'
        + esc(s.n) + ' ' + esc(isFinite(v) ? v.toFixed(s.d) : '--') + esc(s.u || '') + '</div>';
    }
    if (item.total !== undefined && ch.series.length > 1) {
      html += '<div class="ch-tip-r ch-tip-sum">合计 ' + esc(String(item.total)) + '</div>';
    }
    tip.innerHTML = html;
    tip.className = 'ch-tip';
    var tw = tip.offsetWidth || 0, th = tip.offsetHeight || 0;
    var left = mx + 12;
    if (tw && left + tw > ch.cssW) left = mx - tw - 12;
    if (left < 0) left = 0;
    var top = my + 12;
    if (th && top + th > ch.cssH) top = my - th - 12;
    if (top < 0) top = 0;
    tip.style.left = left + 'px';
    tip.style.top = top + 'px';
  }

  function hideTip(ch) {
    if (ch.tip) ch.tip.className = 'ch-tip hidden';
  }

  /* -------------------------------------------------------------- 主题 -- */
  /* 主题切换：Canvas 画不出 CSS 变量，所以由 JS 实时从 style.css / charts.css
   * 读 --xxx 变量写回颜色表 C / EV_COLOR，再整屏重绘。
   * 读不到（老浏览器 / 无 getComputedStyle）时回落到上面的暗色默认值。 */

  function cssVar(el, name) {
    if (!el || typeof global.getComputedStyle !== 'function') return '';
    var v = '';
    try {
      v = global.getComputedStyle(el).getPropertyValue(name);
    } catch (e) { return ''; }
    return (typeof v === 'string') ? v.trim() : '';
  }

  function currentTheme() {
    var b = (global.document && global.document.body) || null;
    var th = (b && b.dataset) ? b.dataset.theme : '';
    return (th === 'light' || th === 'dark') ? th : 'dark';
  }

  function applyTheme() {
    var th = currentTheme();
    var body = (global.document && global.document.body) || null;
    var host = S.dom.metricsHost || S.dom.eventsHost || body;
    var i, v;

    // 车辆状态四色 + 语义色 + 文字色：直接取 style.css 的主题变量
    var map = [
      ['drive', '--veh-driving'], ['park', '--veh-parking'], ['brake', '--veh-braking'],
      ['done', '--veh-done'], ['accent', '--accent'], ['ok', '--ok'],
      ['warn', '--warn'], ['bad', '--bad'], ['text', '--text'], ['sub', '--sub']
    ];
    for (i = 0; i < map.length; i++) {
      v = cssVar(body, map[i][1]);
      C[map[i][0]] = v || DEFAULT_C[map[i][0]];
    }

    // 网格线 / 准星：style.css 没有对应变量（需要比 --border 更淡），按主题派生
    C.grid = (th === 'light') ? 'rgba(0,0,0,.08)' : 'rgba(255,255,255,.06)';
    C.axisPtr = (th === 'light') ? 'rgba(0,0,0,.18)' : 'rgba(255,255,255,.18)';

    // 事件类型色：charts.css 在挂载点上定义了 --ev-*（含浅色覆盖）
    var evMap = [
      ['enter', '--ev-enter'], ['parked', '--ev-parked'], ['exit', '--ev-exit'],
      ['alert', '--ev-alert'], ['conflict', '--ev-conflict']
    ];
    for (i = 0; i < evMap.length; i++) {
      v = cssVar(host, evMap[i][1]);
      EV_COLOR[evMap[i][0]] = v || DEFAULT_EV[evMap[i][0]];
    }

    // SERIES 在定义时已拷贝过一次颜色值，这里同步刷新
    SERIES.state[0].c = C.drive;
    SERIES.state[1].c = C.park;
    SERIES.state[2].c = C.brake;
    SERIES.state[3].c = C.done;
    SERIES.occ[0].c = C.accent;
    SERIES.rate[0].c = C.accent;

    S.theme = th;
  }

  /* 对外：主题切换后调用（app.js 的 setTheme() 内已预埋，带 typeof 守卫） */
  function refreshTheme() {
    applyTheme();
    S.dirty.kpi = true; S.dirty.state = true; S.dirty.occ = true;
    S.dirty.timeline = true; S.dirty.rate = true; S.dirty.dist = true;
    maybeRender(true);
  }

  /* ------------------------------------------------------------ 渲染层 -- */
  function sparkPoints(field) {
    var arr = S.samples;
    var n = Math.min(SPARK_POINTS, arr.length);
    if (n < 2) return '';
    var start = arr.length - n;
    var min = Infinity, max = -Infinity, i, v;
    for (i = start; i < arr.length; i++) {
      v = Number(arr[i][field]);
      if (!isFinite(v)) continue;
      if (v < min) min = v;
      if (v > max) max = v;
    }
    if (!isFinite(min) || !isFinite(max)) return '';
    var span = max - min;
    if (span <= 0) span = 1;
    var pts = [];
    for (i = start; i < arr.length; i++) {
      v = Number(arr[i][field]);
      if (!isFinite(v)) v = min;
      var x = (i - start) / (n - 1) * 100;
      var y = 23 - (v - min) / span * 21;   // 2 .. 23
      pts.push(x.toFixed(1) + ',' + y.toFixed(1));
    }
    return pts.join(' ');
  }

  function renderKpi() {
    var d = S.dom;
    var last = S.samples.length ? S.samples[S.samples.length - 1] : null;
    for (var i = 0; i < d.kpi.length; i++) {
      var k = d.kpi[i], def = k.def;
      var raw = last ? Number(last[def.field]) : NaN;
      k.numEl.textContent = isFinite(raw) ? raw.toFixed(def.digits) : '--';
      if (k.poly) {
        // 描边色每次渲染都从颜色表取，主题切换后 sparkline 会跟着变
        k.poly.setAttribute('stroke', C[def.cKey] || def.color);
        var p = sparkPoints(def.field);
        if (p) k.poly.setAttribute('points', p);
        else k.poly.removeAttribute('points');
      }
    }
  }

  function renderTimeline() {
    var host = S.dom.timeline;
    if (!host) return;
    var arr = S.events;
    if (!arr.length) {
      host.innerHTML = '<div class="ch-empty">暂无事件</div>';
      return;
    }
    var n = Math.min(TL_ROWS, arr.length);
    var html = '';
    for (var i = arr.length - 1; i >= arr.length - n; i--) {
      var e = arr[i];
      var t = Number(e.t);
      var type = EV_LABEL[e.type] ? e.type : 'enter';
      html += '<div class="ch-tl-row">'
        + '<span class="ch-tl-dot ev-' + esc(type) + '"></span>'
        + '<div class="ch-tl-main">'
        + '<div class="ch-tl-meta">' + (isFinite(t) ? t.toFixed(1) : '--') + ' s · '
        + esc(EV_LABEL[type]) + '</div>'
        + '<div class="ch-tl-text">' + esc(e.text) + '</div>'
        + '</div></div>';
    }
    host.innerHTML = html;
  }

  function renderDist() {
    var bar = S.dom.distBar, lg = S.dom.distLegend;
    if (!bar || !lg) return;
    var counts = { enter: 0, parked: 0, exit: 0, alert: 0, conflict: 0 };
    var total = 0, i;
    for (i = 0; i < S.events.length; i++) {
      var ty = S.events[i].type;
      if (counts[ty] === undefined) continue;
      counts[ty] += 1;
      total += 1;
    }
    if (!total) {
      bar.innerHTML = '<i class="ch-bar-empty"></i>';
      lg.innerHTML = '<span class="ch-lg">暂无事件</span>';
      return;
    }
    var seg = '';
    for (i = 0; i < DIST_ORDER.length; i++) {
      var k = DIST_ORDER[i];
      if (!counts[k]) continue;
      var pct = (counts[k] / total * 100).toFixed(2);
      seg += '<i style="flex:0 0 ' + pct + '%;background:' + EV_COLOR[k] + '"></i>';
    }
    bar.innerHTML = seg;

    var items = [];
    for (i = 0; i < DIST_ORDER.length; i++) {
      var key = DIST_ORDER[i];
      if (!DIST_ALWAYS[key] && !counts[key]) continue;
      items.push({ color: EV_COLOR[key], label: (DIST_ALWAYS[key] || EV_LABEL[key]) + ' ' + counts[key] });
    }
    lg.innerHTML = legendHtml(items);
  }

  function maybeRender(force) {
    var now = (typeof global.Date === 'function') ? new global.Date().getTime() : 0;
    if (!force && now - S.lastRenderWall < RENDER_MIN_MS) return;
    S.lastRenderWall = now;
    var any = false, k;
    for (k in S.dirty) { if (S.dirty[k]) { any = true; break; } }
    if (!any) return;
    if (S.dirty.kpi) renderKpi();
    if (S.dirty.state) drawChart(S.charts.state);
    if (S.dirty.occ) drawChart(S.charts.occ);
    if (S.dirty.timeline) renderTimeline();
    if (S.dirty.rate) drawChart(S.charts.rate);
    if (S.dirty.dist) renderDist();
    for (k in S.dirty) S.dirty[k] = false;
  }

  /* ------------------------------------------------------------ 数据采集 -- */
  function pushSample(t, cnt, total, occRate, avgV) {
    S.samples.push({
      t: t,
      drive: cnt[0], park: cnt[1], brake: cnt[2], done: cnt[3],
      total: total, occRate: occRate, avgV: avgV, parkedCum: S.parkedCum
    });
    if (S.samples.length > MAX_SAMPLES) S.samples.splice(0, S.samples.length - MAX_SAMPLES);
    S.dirty.kpi = true; S.dirty.state = true; S.dirty.occ = true;
  }

  function occRateOf(obstacles) {
    var arr = S.lastOcc;
    if (arr && arr.length) {
      var ones = 0;
      for (var i = 0; i < arr.length; i++) {
        var v = arr[i];
        if (v === 1 || v === true || v === '1') ones++;
      }
      return ones / arr.length * 100;
    }
    // 从未收到 occupancy：退回 static_obstacles.length / 269
    var n = Array.isArray(obstacles) ? obstacles.length : 0;
    return n / SPOT_TOTAL * 100;
  }

  /* ------------------------------------------------------------ 事件推导 -- */
  function addEvent(t, type, text) {
    S.events.push({ t: t, type: type, text: text });
    if (S.events.length > MAX_EVENTS) S.events.splice(0, S.events.length - MAX_EVENTS);
    S.dirty.timeline = true; S.dirty.rate = true; S.dirty.dist = true;
  }

  function deriveEvents(t, vehicles, obstacles) {
    var i, id, v;
    var curIds = {};
    var waitMap = {};
    var hasWait = false;

    for (i = 0; i < vehicles.length; i++) {
      v = vehicles[i];
      if (!v) continue;
      id = idOf(v.id);
      if (id === null) continue;
      curIds[id] = true;
      var w = idOf(v.wait);
      if (w !== null && w > 0 && w !== id) { waitMap[id] = w; hasWait = true; }
    }

    var curParked = {};
    for (i = 0; i < obstacles.length; i++) {
      var o = obstacles[i];
      if (!o) continue;
      var vid = idOf(o.vehicle_id);
      if (vid === null) continue;
      var spot = idOf(o.spot);
      curParked[vid] = (spot === null ? 0 : spot);
    }

    // 首帧（或 reset 后第一帧）只建基线：否则会把已有车辆全部记成「进场」
    if (!S.primed) {
      S.primed = true;
      S.prevIds = curIds;
      S.prevParked = curParked;
      return;
    }

    var prevIds = S.prevIds || {};
    var prevParked = S.prevParked || {};
    var k;

    /* enter：新出现的车辆 id */
    for (k in curIds) {
      if (!prevIds[k]) addEvent(t, 'enter', '车辆 #' + k + ' 由入口驶入');
    }

    /* parked：新进入 static_obstacles 的 vehicle_id */
    for (k in curParked) {
      if (prevParked[k] === undefined) {
        addEvent(t, 'parked', '车辆 #' + k + ' 停入车位 ' + curParked[k]);
        S.parkedCum += 1;
      }
    }

    /* exit：从 static_obstacles 消失的 vehicle_id（spot 用消失前记录的那个） */
    for (k in prevParked) {
      if (curParked[k] === undefined) {
        addEvent(t, 'exit', '车辆 #' + k + ' 离开车位 ' + prevParked[k]);
      }
    }

    /* conflict：A.wait === B.id && B.wait === A.id，按车辆对去重 */
    var curPairs = {};
    if (hasWait) {
      for (k in waitMap) {
        var a = Number(k);
        var b = waitMap[k];
        var back = waitMap[b];
        if (back === undefined || back !== a) continue;
        if (a > b) continue;                       // 每对只由较小 id 一侧处理
        curPairs[a + ':' + b] = true;
      }
    }
    for (k in curPairs) {
      if (!S.conflictActive[k]) {
        var parts = k.split(':');
        addEvent(t, 'conflict', '#' + parts[0] + ' 与 #' + parts[1] + ' 互等，死锁前兆');
      }
    }
    for (k in S.conflictActive) {
      if (!curPairs[k]) delete S.conflictActive[k];  // 该对解除 → 允许再次触发
    }
    for (k in curPairs) S.conflictActive[k] = true;

    S.prevIds = curIds;
    S.prevParked = curParked;
  }

  /* -------------------------------------------------------------- 接口 -- */
  function init() {
    if (!hasDom()) return;
    var d = S.dom;
    var mh = global.document.getElementById('metricsViz');
    var eh = global.document.getElementById('eventsViz');

    if (mh && d.metricsHost !== mh) {
      d.metricsHost = mh;
      d.kpi = [];
      buildMetricsDom(mh);
    }
    if (eh && d.eventsHost !== eh) {
      d.eventsHost = eh;
      buildEventsDom(eh);
    }
    S.inited = true;
    // 首屏：initTheme() 早于 init()，所以这里必须自己按当前主题取一次色
    applyTheme();
    // 浏览器窗口尺寸变化会让侧栏宽度变，除了宿主调 resize()，这里也兜一层
    if (!S.winBound && typeof global.addEventListener === 'function') {
      global.addEventListener('resize', function () { resize(); });
      S.winBound = true;
    }
    maybeRender(true);
  }

  /* 容错：若调用方尚未 init()（或挂载点在 init 之后才插入 DOM），自动重试一次。
   * 每次尝试间隔 >= 500ms，避免高频 getElementById。 */
  function ensureReady() {
    if (S.inited && (S.dom.metricsHost || S.dom.eventsHost)) return;
    if (!hasDom()) return;
    var now = new global.Date().getTime();
    if (now - S.lastInitTry < 500) return;
    S.lastInitTry = now;
    init();
  }

  function onFrame(frame) {
    if (!frame || typeof frame !== 'object') return;
    ensureReady();
    if (!S.inited) return;

    var t = num(frame.t);
    if (!isFinite(t)) return;
    S.lastT = t;

    var vehicles = Array.isArray(frame.vehicles) ? frame.vehicles : [];
    var obstacles = Array.isArray(frame.static_obstacles) ? frame.static_obstacles : [];
    var running = frame.running !== false;

    /* 事件推导：逐帧差分。空车帧（停止/清空瞬间）跳过，避免整批误报出场 */
    if (running && vehicles.length > 0) deriveEvents(t, vehicles, obstacles);

    /* 时序采样：running===false 或 t 未推进 → 不采样（不能用 seq 判断） */
    if (!running) { maybeRender(false); return; }
    if (S.lastSampleT === null || t < S.lastSampleT || t - S.lastSampleT >= SAMPLE_STEP) {
      var cnt = [0, 0, 0, 0], vSum = 0, vN = 0, i, v, c;
      for (i = 0; i < vehicles.length; i++) {
        v = vehicles[i];
        if (!v) continue;
        c = Number(v.c);
        if (c === 0) cnt[0] += 1;
        else if (c === 1) cnt[1] += 1;
        else if (c === 2) cnt[2] += 1;
        else if (c === 3) cnt[3] += 1;
        var sp = num(v.v);
        if (isFinite(sp)) { vSum += sp; vN += 1; }
      }
      pushSample(
        Math.round(t * 10) / 10,
        cnt,
        vehicles.length,
        occRateOf(obstacles),
        vN ? vSum / vN : 0
      );
      S.lastSampleT = t;
      maybeRender(true);
      return;
    }
    maybeRender(false);
  }

  function onOccupancy(payload) {
    ensureReady();
    if (!S.inited) return;
    var arr = Array.isArray(payload) ? payload
      : (payload && Array.isArray(payload.data) ? payload.data : null);
    if (!arr || !arr.length) return;
    S.lastOcc = arr;
    var ones = 0;
    for (var i = 0; i < arr.length; i++) {
      var v = arr[i];
      if (v === 1 || v === true || v === '1') ones++;
    }
    S.occRate = ones / arr.length * 100;
    S.dirty.kpi = true;
    maybeRender(false);
  }

  function onAlert(alert) {
    ensureReady();
    if (!S.inited) return;
    if (!alert || typeof alert !== 'object') return;
    if (alert.active !== true) return;
    var text = (typeof alert.message === 'string' && alert.message)
      ? alert.message
      : (alert.code ? String(alert.code) : '告警');
    var key = String(alert.code == null ? '' : alert.code) + '|' + text;
    var prev = S.lastAlert[key];
    // 同一告警 5 仿真秒内只记一次（后端可能逐帧重发 active:true）
    if (prev !== undefined && Math.abs(S.lastT - prev) < ALERT_DEDUPE_SEC) return;
    S.lastAlert[key] = S.lastT;
    addEvent(S.lastT, 'alert', text);
    maybeRender(false);
  }

  function resize() {
    if (!S.inited) return;
    // 自绘图表没有 chart.resize()：重新量一次 CSS 宽度 → 按 dpr 重建画布 → 重绘
    S.dirty.state = true; S.dirty.occ = true; S.dirty.rate = true;
    maybeRender(true);
  }

  function reset() {
    S.samples.length = 0;
    S.events.length = 0;                 // 保持数组引用不变
    S.prevIds = {};
    S.prevParked = {};
    S.conflictActive = {};
    S.lastAlert = {};
    S.parkedCum = 0;
    S.primed = false;
    S.lastSampleT = null;
    S.lastT = 0;
    S.lastOcc = null;
    S.occRate = 0;
    if (S.charts.state) { S.charts.state.hoverIdx = -1; hideTip(S.charts.state); }
    if (S.charts.occ) { S.charts.occ.hoverIdx = -1; hideTip(S.charts.occ); }
    if (S.charts.rate) { S.charts.rate.hoverIdx = -1; hideTip(S.charts.rate); }
    S.dirty.kpi = true; S.dirty.state = true; S.dirty.occ = true;
    S.dirty.timeline = true; S.dirty.rate = true; S.dirty.dist = true;
    maybeRender(true);
  }

  function canvasInfo(ch) {
    if (!ch) return null;
    return {
      w: ch.cssW, h: ch.cssH, dpr: ch.dpr,
      pw: ch.canvas.width, ph: ch.canvas.height,
      items: ch.items ? ch.items.length : 0,
      hoverIdx: ch.hoverIdx,
      ticks: (ch.ticks || []).map(function (t) {
        return { x: Math.round(t.x * 10) / 10, label: t.label, w: Math.round(t.w * 10) / 10 };
      })
    };
  }

  function dbg() {
    return {
      inited: S.inited,
      theme: S.theme,
      colors: {
        drive: C.drive, park: C.park, brake: C.brake, done: C.done,
        accent: C.accent, text: C.text, sub: C.sub, grid: C.grid
      },
      hasMetricsHost: !!S.dom.metricsHost,
      hasEventsHost: !!S.dom.eventsHost,
      charts: {
        state: !!(S.charts.state && S.charts.state.cssW > 0),
        occ: !!(S.charts.occ && S.charts.occ.cssW > 0),
        rate: !!(S.charts.rate && S.charts.rate.cssW > 0)
      },
      canvas: {
        state: canvasInfo(S.charts.state),
        occ: canvasInfo(S.charts.occ),
        rate: canvasInfo(S.charts.rate)
      },
      samples: S.samples.length,
      lastSample: S.samples.length ? S.samples[S.samples.length - 1] : null,
      events: S.events.length,
      lastEvents: S.events.slice(-5),
      byType: (function () {
        var m = { enter: 0, parked: 0, exit: 0, alert: 0, conflict: 0 };
        for (var i = 0; i < S.events.length; i++) {
          if (m[S.events[i].type] !== undefined) m[S.events[i].type] += 1;
        }
        return m;
      })(),
      lastSampleT: S.lastSampleT,
      lastT: S.lastT,
      occRate: S.occRate,
      occLen: S.lastOcc ? S.lastOcc.length : 0,
      parkedCum: S.parkedCum,
      primed: S.primed,
      prevIds: S.prevIds ? Object.keys(S.prevIds).length : 0,
      prevParked: S.prevParked ? Object.keys(S.prevParked).length : 0,
      conflictActive: S.conflictActive ? Object.keys(S.conflictActive).length : 0
    };
  }

  /* -------------------------------------------------------------- 导出 -- */
  global.ParkSimCharts = {
    init: init,
    onFrame: onFrame,
    onOccupancy: onOccupancy,
    onAlert: onAlert,
    resize: resize,
    reset: reset,
    refreshTheme: refreshTheme,
    events: S.events,
    _dbg: dbg
  };
})(typeof window !== 'undefined' ? window : this);
