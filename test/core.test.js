'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const core = require('../public/core.js');

const T = 12;

function approx(actual, expected, tol = 1e-6, label) {
  assert.ok(
    Math.abs(actual - expected) <= tol,
    `${label || '值'} 期望 ${expected}，实际 ${actual}`
  );
}

test('正面相撞：两人相向匀速，在 t=6 接触', () => {
  const a = { id: 'a', radius: 0, keys: [{ t: 0, x: 0, y: 0 }, { t: 12, x: 120, y: 0 }] };
  const b = { id: 'b', radius: 0, keys: [{ t: 0, x: 120, y: 0 }, { t: 12, x: 0, y: 0 }] };
  const r = core.analyzePair(a, b, T);
  assert.equal(r.collision, true);
  assert.equal(r.intervals.length, 1);
  approx(r.intervals[0][0], 6, 1e-7, '进入碰撞');
  approx(r.intervals[0][1], 6, 1e-7, '离开碰撞');
  approx(r.firstContact, 6, 1e-7);
  approx(r.minDistance, 0, 1e-7);
  approx(r.minDistanceT, 6, 1e-7);
});

test('半径产生有限宽度的碰撞区间（非点接触）', () => {
  const a = { id: 'a', radius: 10, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 100, y: 0 }, { t: 10, x: 0, y: 0 }] };
  const r = core.analyzePair(a, b, T);
  assert.equal(r.collision, true);
  // 相对速度 20/s，半径和 20 -> |100-20t|<=20 -> t∈[4,6]
  approx(r.intervals[0][0], 4, 1e-7);
  approx(r.intervals[0][1], 6, 1e-7);
});

test('相切：最近距离恰好等于半径和，区间退化为单点也算碰撞', () => {
  // A 水平经过 y=10，B 静止原点；rSum=10 -> 恰在 t=5 相切
  const a = { id: 'a', radius: 0, keys: [{ t: 0, x: 0, y: 10 }, { t: 10, x: 100, y: 10 }] };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 50, y: 0 }] };
  const r = core.analyzePair(a, b, T);
  assert.equal(r.collision, true);
  approx(r.intervals[0][0], 5, 1e-6);
  approx(r.intervals[0][1], 5, 1e-6);
  approx(r.minDistance, 10, 1e-6);
});

test('错峰交叉：路径相交但不同时到达，不得误报', () => {
  // A 水平匀速，t=5 经过交点 (50,0)；B 竖直疾行，t≈1 经过交点后远离
  const a = { id: 'a', radius: 2, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const b = { id: 'b', radius: 2, keys: [{ t: 0, x: 50, y: -100 }, { t: 2, x: 50, y: 100 }] };
  const r = core.analyzePair(a, b, T);
  assert.equal(r.collision, false);
  assert.deepEqual(r.intervals, []);
  assert.equal(r.firstContact, null);
  // 几何路径交点距离为 0 但时刻错开，全程最近距离仍远大于半径和 4
  assert.ok(r.minDistance > 30, `最近距离应 >30，实际 ${r.minDistance}`);
});

test('相对静止持续重叠：v=0 退化解覆盖整段', () => {
  const a = { id: 'a', radius: 30, keys: [{ t: 0, x: 0, y: 0 }, { t: 12, x: 120, y: 0 }] };
  // B 同速平行，间距恒为 40 <= 半径和 50
  const b = { id: 'b', radius: 20, keys: [{ t: 0, x: 0, y: 40 }, { t: 12, x: 120, y: 40 }] };
  const r = core.analyzePair(a, b, T);
  assert.equal(r.collision, true);
  assert.equal(r.intervals.length, 1);
  approx(r.intervals[0][0], 0, 1e-7);
  approx(r.intervals[0][1], 12, 1e-7);
  approx(r.minDistance, 40, 1e-7);
});

test('跨关键帧的碰撞区间在边界处正确合并', () => {
  // A 匀速直线（单段），B 用两个共线关键帧人为切成两段，结果应一致
  const a = { id: 'a', radius: 10, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const b1 = { id: 'b', radius: 10, keys: [{ t: 0, x: 100, y: 0 }, { t: 5, x: 50, y: 0 }, { t: 10, x: 0, y: 0 }] };
  const b2 = { id: 'b', radius: 10, keys: [{ t: 0, x: 100, y: 0 }, { t: 10, x: 0, y: 0 }] };
  const r1 = core.analyzePair(a, b1, T);
  const r2 = core.analyzePair(a, b2, T);
  assert.equal(r1.intervals.length, r2.intervals.length);
  approx(r1.intervals[0][0], r2.intervals[0][0]);
  approx(r1.intervals[0][1], r2.intervals[0][1]);
});

test('关键帧之外保持静止（开场即贴近静止者）', () => {
  const a = { id: 'a', radius: 15, keys: [{ t: 3, x: 0, y: 0 }, { t: 9, x: 100, y: 0 }] };
  const b = { id: 'b', radius: 15, keys: [{ t: 0, x: 5, y: 0 }] };
  const r = core.analyzePair(a, b, T);
  // 0..3 期间 A 静止在 (0,0)，与 B 相距 5 <= 30，必然碰撞
  assert.equal(r.collision, true);
  approx(r.intervals[0][0], 0, 1e-7);
});

test('analyzeScene：4 人产生 6 对，首次碰撞与全局最小间距正确', () => {
  const scene = core.builtinScene();
  const r = core.analyzeScene(scene);
  assert.equal(r.pairCount, 6);
  const colliding = r.pairs.filter((p) => p.collision);
  const names = colliding.map((p) => `${p.aId}-${p.bId}`).sort();
  assert.deepEqual(names, ['p1-p2', 'p3-p4']);

  const p12 = r.pairs.find((p) => p.aId === 'p1' && p.bId === 'p2');
  approx(p12.firstContact, 5.53, 0.02, '甲乙首碰时刻');
  approx(p12.minDistance, 0, 1e-6, '甲乙最小间距');
  approx(p12.minDistanceT, 6, 1e-6, '甲乙最小间距时刻');
  approx(p12.intervals[0][0], 5.53, 0.02);
  approx(p12.intervals[0][1], 6.47, 0.02);

  const p34 = r.pairs.find((p) => p.aId === 'p3' && p.bId === 'p4');
  assert.equal(p34.collision, true);
  approx(p34.firstContact, 2.60, 0.02, '丙丁首碰时刻');
  approx(p34.lastRelease, 12, 1e-6, '丙丁持续到收场');
  assert.ok(p34.minDistance < 40, '丙丁最近间距应小于半径和 40');

  // 几何相交但错峰的组合不得误报
  for (const id of ['p1-p3', 'p2-p3', 'p1-p4', 'p2-p4']) {
    const [x, y] = id.split('-');
    const p = r.pairs.find((q) => q.aId === x && q.bId === y);
    assert.equal(p.collision, false, `${id} 不应碰撞`);
  }

  approx(r.firstContact, p34.firstContact, 1e-6);
  approx(r.minDistance, 0, 1e-6);
});

test('positionAt 线性插值与端点静止', () => {
  const keys = core.normalizeKeys([{ t: 2, x: 0, y: 0 }, { t: 6, x: 40, y: 20 }]);
  assert.deepEqual(core.positionAt(keys, 0), { x: 0, y: 0 });
  const mid = core.positionAt(keys, 4);
  approx(mid.x, 20, 1e-9);
  approx(mid.y, 10, 1e-9);
  assert.deepEqual(core.positionAt(keys, 9), { x: 40, y: 20 });
});

test('radiusAt：缺省继承表演者半径，相邻关键帧线性插值，首尾之外保持端点值', () => {
  const keys = core.normalizeKeys([
    { t: 0, x: 0, y: 0 },          // 缺省 -> 继承 20
    { t: 4, x: 0, y: 0, r: 60 },
    { t: 8, x: 0, y: 0, r: 40 }
  ], 20);
  approx(core.radiusAt(keys, -1, 20), 20, 1e-9, '首帧之前保持首帧（继承）值');
  approx(core.radiusAt(keys, 0, 20), 20, 1e-9);
  approx(core.radiusAt(keys, 2, 20), 40, 1e-9, '段内线性插值');
  approx(core.radiusAt(keys, 4, 20), 60, 1e-9);
  approx(core.radiusAt(keys, 6, 20), 50, 1e-9);
  approx(core.radiusAt(keys, 8, 20), 40, 1e-9);
  approx(core.radiusAt(keys, 12, 20), 40, 1e-9, '末帧之后保持末帧值');

  const st = core.stateAt([{ t: 0, x: 10, y: 0 }, { t: 10, x: 90, y: 0, r: 40 }], 5, 20);
  approx(st.x, 50, 1e-9);
  approx(st.r, 30, 1e-9, 'stateAt 同步插值位置与半径');
});

test('可变半径：张开道具提前进入碰撞（半径和线性增大，解析二次式）', () => {
  // B 沿 x 轴匀速从 0 走到 100（r=10）；A 静止在 x=70，
  // t∈[0,4] 安全半径 10→50，之后保持 50
  const a = {
    id: 'a', radius: 10,
    keys: [{ t: 0, x: 70, y: 0, r: 10 }, { t: 4, x: 70, y: 0, r: 50 }]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const r = core.analyzePair(a, b, 10);
  assert.equal(r.collision, true);
  // 张开段：间距 70-10t，半径和 20+10t → 相等时 70-10t=20+10t → t=2.5
  approx(r.firstContact, 2.5, 1e-7, '张开导致首碰 t=2.5');
  // t≥4 后半径和恒 60、间距持续缩小；穿过中心后在 70-10t=-60 → t=13 才脱离（窗外）
  approx(r.lastRelease, 10, 1e-7, '窗内始终重叠到收场');
  // 几何最小间距仍为 0（半径无关）
  approx(r.minDistance, 0, 1e-7);
  // 半径和极值：20（起始）～60
  approx(r.radiusSumMin, 20, 1e-9);
  approx(r.radiusSumMax, 60, 1e-9);
});

test('可变半径：单帧张开再收拢，碰撞区间在同段精确起止（两点根）', () => {
  // B 从 100 走到 0（r=10）；A 静止原点，半径在 t∈[2,6] 由 10 线性升到 90
  const a = {
    id: 'a', radius: 10,
    keys: [{ t: 2, x: 0, y: 0, r: 10 }, { t: 6, x: 0, y: 0, r: 90 }]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 100, y: 0 }, { t: 10, x: 0, y: 0 }] };
  const r = core.analyzePair(a, b, 10);
  assert.equal(r.collision, true);
  // 张开段 t∈[2,6]：间距 100-10t，rA=20t-30，半径和 20t-20
  //   相等：100-10t=20t-20 → t=4 接触
  approx(r.intervals[0][0], 4, 1e-7, '张开段接触 t=4');
  // t≥6 后半径和恒 100，间距继续缩小；脱离需 100-10t=-100 → t=20（窗外）
  approx(r.intervals[r.intervals.length - 1][1], 10, 1e-7);
});

test('可变半径：相对静止下仅靠张开道具产生精确进入时刻', () => {
  // 两人完全静止，相距 30；A 半径 10 在 t∈[2,6] 线性升到 50，B 恒 10
  const a = {
    id: 'a', radius: 10,
    keys: [{ t: 0, x: 0, y: 0, r: 10 }, { t: 2, x: 0, y: 0, r: 10 }, { t: 6, x: 0, y: 0, r: 50 }]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 30, y: 0 }] };
  const r = core.analyzePair(a, b, 8);
  assert.equal(r.collision, true);
  // 半径和 = 20 + 10(t-2) = 10t（t∈[2,6]），等于 30 时 t=3
  approx(r.firstContact, 3, 1e-7);
  approx(r.intervals[0][0], 3, 1e-7);
  // t≥6 半径和 60 > 30，持续碰撞到窗末
  approx(r.lastRelease, 8, 1e-7);
  // 最小几何间距不受半径影响：恒为 30
  approx(r.minDistance, 30, 1e-7);
});

test('半径快速张开（二次式首项系数为负）：先碰后随合拢脱离', () => {
  // 一人匀速离开但半径张开更快：|v|² - d² < 0 分支
  // B 从 x=0 以 10/s 向 +x（r=0）；A 静止 x=0，r: 0→80 在 t∈[0,4]
  const a = {
    id: 'a', radius: 0,
    keys: [{ t: 0, x: 0, y: 0, r: 0 }, { t: 4, x: 0, y: 0, r: 80 }]
  };
  const b = { id: 'b', radius: 0, keys: [{ t: 0, x: 0, y: 0 }, { t: 8, x: 80, y: 0 }] };
  const r = core.analyzePair(a, b, 8);
  // t∈[0,4]：间距 10t，半径和 20t → 10t ≤ 20t 恒成立（t=0 重合）
  // t∈[4,8]：间距 10t，半径和恒 80 → 10t ≤ 80 → t ≤ 8（t=8 相切脱离）
  assert.equal(r.collision, true);
  approx(r.firstContact, 0, 1e-7);
  approx(r.lastRelease, 8, 1e-7, 't=8 恰好相切');
});

test('未设置关键帧半径时与固定半径结果完全一致（向后兼容）', () => {
  const a1 = { id: 'a', radius: 10, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const a2 = {
    id: 'a', radius: 10,
    keys: [{ t: 0, x: 0, y: 0, r: 10 }, { t: 10, x: 100, y: 0, r: 10 }]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 100, y: 0 }, { t: 10, x: 0, y: 0 }] };
  const r1 = core.analyzePair(a1, b, T);
  const r2 = core.analyzePair(a2, b, T);
  assert.equal(r1.intervals.length, r2.intervals.length);
  approx(r1.intervals[0][0], r2.intervals[0][0]);
  approx(r1.intervals[0][1], r2.intervals[0][1]);
  approx(r1.minDistance, r2.minDistance);
  approx(r2.radiusSumMin, 20, 1e-9);
  approx(r2.radiusSumMax, 20, 1e-9);
});

test('收拢道具在半径线性减小段精确脱离碰撞', () => {
  // 两人完全静止，相距 30；A 半径在 t∈[2,6] 由 50 线性收到 10，B 恒 10
  const a = {
    id: 'a', radius: 10,
    keys: [{ t: 0, x: 0, y: 0, r: 50 }, { t: 2, x: 0, y: 0, r: 50 }, { t: 6, x: 0, y: 0, r: 10 }]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 30, y: 0 }] };
  const r = core.analyzePair(a, b, 8);
  assert.equal(r.collision, true);
  // t<2 半径和 60 ≥ 30 全程碰撞；收拢段半径和 = 60-10(t-2) = 80-10t
  // 等于 30 时 t=5 脱离，之后半径和 20 < 30 安全
  assert.equal(r.intervals.length, 1);
  approx(r.intervals[0][0], 0, 1e-7);
  approx(r.intervals[0][1], 5, 1e-7);
});

test('跨关键帧半径分段：张开段进入与关键帧处相切合并', () => {
  // A 静止原点；t∈[0,4] r 10→50，t∈[4,8] r 50→10；B 以 10/s 从 100 走向 0（r=10）
  const a = {
    id: 'a', radius: 10,
    keys: [
      { t: 0, x: 0, y: 0, r: 10 },
      { t: 4, x: 0, y: 0, r: 50 },
      { t: 8, x: 0, y: 0, r: 10 }
    ]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 100, y: 0 }, { t: 10, x: 0, y: 0 }] };
  const r = core.analyzePair(a, b, 10);
  assert.equal(r.collision, true);
  // 张开段：间距 100-10t，半径和 20+10t，相等时 t=4（恰在关键帧处相切进入）
  // 合拢段半径和 100-10t，与间距恒等，整段重叠；t≥8 后半径和 20、间距继续缩小，
  // 在 t∈[8,10] 内间距 ≤20 → 直到 t=10。两段在 t=4 相接，合并为单一区间
  assert.equal(r.intervals.length, 1, '段边界相接区间应合并为一段');
  approx(r.intervals[0][0], 4, 1e-6);
  approx(r.intervals[0][1], 10, 1e-6);
});

test('内置样例含关键帧半径且不改变原有碰撞结论', () => {
  const s = core.builtinScene();
  const p3 = s.performers.find((p) => p.id === 'p3');
  assert.equal(p3.keys[2].r, 60, '丙 t=8 帧携带展开半径 60');
  const r = core.analyzeScene(s);
  const p13 = r.pairs.find((p) => p.aId === 'p1' && p.bId === 'p3');
  const p23 = r.pairs.find((p) => p.aId === 'p2' && p.bId === 'p3');
  assert.equal(p13.collision, false, '展开后与甲仍安全');
  assert.equal(p23.collision, false, '展开后与乙仍安全');
  const p34 = r.pairs.find((p) => p.aId === 'p3' && p.bId === 'p4');
  assert.equal(p34.collision, true);
  approx(p34.firstContact, 2.60, 0.02);
});

// ---------- 最小安全余量 minMargin = min distance(t)-[rA(t)+rB(t)] ----------

test('最小余量：相向贯穿且各有半径，余量最小值为负半径和，发生在相遇中点', () => {
  // A: 0→100，B: 100→0，各 r=10；t=5 重合：余量 0-20=-20
  const a = { id: 'a', radius: 10, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 100, y: 0 }, { t: 10, x: 0, y: 0 }] };
  const r = core.analyzePair(a, b, T);
  approx(r.minMargin, -20, 1e-7, '最大侵入量 = 半径和负值');
  approx(r.minMarginT, 5, 1e-7, '发生在相遇中点');
});

test('最小余量：相切时刻余量恰好为 0', () => {
  // A 水平经过 y=10，B 静止 (50,0) r=10，rA=0 → t=5 相切
  const a = { id: 'a', radius: 0, keys: [{ t: 0, x: 0, y: 10 }, { t: 10, x: 100, y: 10 }] };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 50, y: 0 }] };
  const r = core.analyzePair(a, b, T);
  approx(r.minMargin, 0, 1e-7, '相切余量为 0');
  approx(r.minMarginT, 5, 1e-7);
  assert.equal(r.collision, true, '余量 0 等价于碰撞（相切）');
});

test('最小余量：相对静止恒重叠，余量为常数且全程相等（取最早时刻 0）', () => {
  // 同速平行，距离恒 40，半径和恒 50 → 余量恒 -10
  const a = { id: 'a', radius: 30, keys: [{ t: 0, x: 0, y: 0 }, { t: 12, x: 120, y: 0 }] };
  const b = { id: 'b', radius: 20, keys: [{ t: 0, x: 0, y: 40 }, { t: 12, x: 120, y: 40 }] };
  const r = core.analyzePair(a, b, T);
  approx(r.minMargin, -10, 1e-7, '恒量余量 = 40-50');
  approx(r.minMarginT, 0, 1e-7, '余量全程相等取最早时刻');
});

test('最小余量：相对静止且距离为零（完全重合），余量为负半径和', () => {
  const a = { id: 'a', radius: 20, keys: [{ t: 0, x: 35, y: 12 }] };
  const b = { id: 'b', radius: 15, keys: [{ t: 0, x: 35, y: 12 }] };
  const r = core.analyzePair(a, b, T);
  approx(r.minDistance, 0, 1e-9);
  approx(r.minMargin, -35, 1e-9, '完全重合：0-(20+15)');
  approx(r.minMarginT, 0, 1e-9);
});

test('最小余量：保持净距不碰撞，余量为正且等于最近间距减半径和', () => {
  // A 水平，B 静止 (50,30)，各 r=10：最近距离 30，余量 10
  const a = { id: 'a', radius: 10, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 50, y: 30 }] };
  const r = core.analyzePair(a, b, T);
  assert.equal(r.collision, false);
  approx(r.minMargin, 10, 1e-7, '净空间隙 30-20');
  approx(r.minMarginT, 5, 1e-7);
});

test('最小余量：相对静止 + 可变半径线性张开，余量为一次函数，最小值在张满端点', () => {
  // 两人静止相距 30；A 半径 t∈[2,6] 由 10 升到 50，B 恒 10
  // 余量 g=30-(rA+10)：t≤2 恒 10，张开段 30-10t，t=6 时 30-60=-30 最小
  const a = {
    id: 'a', radius: 10,
    keys: [{ t: 0, x: 0, y: 0, r: 10 }, { t: 2, x: 0, y: 0, r: 10 }, { t: 6, x: 0, y: 0, r: 50 }]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 30, y: 0 }] };
  const r = core.analyzePair(a, b, 8);
  approx(r.minMargin, -30, 1e-7, '张满端点侵入最深');
  approx(r.minMarginT, 6, 1e-7);
  assert.equal(r.collision, true);
});

test('最小余量：距离为零 + 可变半径（穿过张开者中心），尖点处取最小', () => {
  // A 静止原点，半径 t∈[0,10] 由 20 升到 40；B 从 (0,0) 走到 (100,0)，r=20
  // t=0 距离 0、半径和 40 → 余量 -40（尖点）；之后距离 10t、半径和 40+2t，余量回升
  const a = {
    id: 'a', radius: 20,
    keys: [{ t: 0, x: 0, y: 0, r: 20 }, { t: 10, x: 0, y: 0, r: 40 }]
  };
  const b = { id: 'b', radius: 20, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }] };
  const r = core.analyzePair(a, b, 10);
  approx(r.minMargin, -40, 1e-7, '距离为零尖点：0-40');
  approx(r.minMarginT, 0, 1e-7);
});

test('最小余量：驻点位于段内部（接近速度等于半径张开速度）时精确取最小', () => {
  // A 静止原点，半径 t∈[0,10] 由 20 线性升到 40；B r=20 从 (0,30) 以 10/s
  // 走向 (100,30) → 距离 sqrt((10t)^2+30^2)，半径和 40+2t。
  // 驻点条件 100t/sqrt(100t²+900)=2 → t²=3/8 → t*=√(3/8)≈0.6124；
  // 该点 g≈-10.606，比左端点 g(0)=-10 更小——最小值在段内部而非端点。
  const a = {
    id: 'a', radius: 20,
    keys: [{ t: 0, x: 0, y: 0, r: 20 }, { t: 10, x: 0, y: 0, r: 40 }]
  };
  const b = { id: 'b', radius: 20, keys: [{ t: 0, x: 0, y: 30 }, { t: 10, x: 100, y: 30 }] };
  const r = core.analyzePair(a, b, 10);
  const tStar = Math.sqrt(3 / 8);
  const gMin = Math.sqrt(100 * tStar * tStar + 900) - 40 - 2 * tStar;
  approx(r.minMargin, gMin, 1e-6, '段内驻点余量（负，侵入）');
  approx(r.minMarginT, tStar, 1e-6, '驻点时刻 sqrt(3/8)');
  assert.equal(r.collision, true, '驻点余量为负，属碰撞');
  assert.ok(r.minMargin < -10 - 1e-6, '驻点余量应严格小于左端点 -10');
});

test('最小余量：段内驻点给出正的净空间隙最小值（无碰撞）', () => {
  // 同上但 B 半径为 0：半径和 20+2t，g(0)=30-20=10；驻点同为 √(3/8)，
  // g* = 25√6/2 - 20 - √6/2 = 12√6 - 20 ≈ 9.394 < 10，仍为正 → 全程安全。
  const a = {
    id: 'a', radius: 20,
    keys: [{ t: 0, x: 0, y: 0, r: 20 }, { t: 10, x: 0, y: 0, r: 40 }]
  };
  const b = { id: 'b', radius: 0, keys: [{ t: 0, x: 0, y: 30 }, { t: 10, x: 100, y: 30 }] };
  const r = core.analyzePair(a, b, 10);
  approx(r.minMargin, 12 * Math.sqrt(6) - 20, 1e-6, '正的驻点余量');
  approx(r.minMarginT, Math.sqrt(3 / 8), 1e-6);
  assert.equal(r.collision, false);
  assert.ok(r.minMargin > 0 && r.minMargin < 10 - 1e-6, '驻点正余量应小于左端点 10');
});

test('最小余量：跨关键帧分段时取全时域全局最小', () => {
  // A 三段折线，B 静止 (60,0) r=10；A r=10：
  // t∈[0,4] (0,0)→(100,0)，t∈[4,8] 折返回 (0,0)，t∈[8,12] 再去 (100,0)
  // 每次经过 x=60 都更近，但半径恒定，最小余量 = |60-60|-20 = -20，
  // 第一次经过（最早）t=2.4
  const a = {
    id: 'a', radius: 10,
    keys: [
      { t: 0, x: 0, y: 0 }, { t: 4, x: 100, y: 0 },
      { t: 8, x: 0, y: 0 }, { t: 12, x: 100, y: 0 }
    ]
  };
  const b = { id: 'b', radius: 10, keys: [{ t: 0, x: 60, y: 0 }] };
  const r = core.analyzePair(a, b, T);
  approx(r.minMargin, -20, 1e-7);
  approx(r.minMarginT, 2.4, 1e-7, '跨多段同值取最早发生时刻');
});

test('analyzeScene：pairs 按最小余量升序，顶层汇总全局最紧的一对', () => {
  const r = core.analyzeScene(core.builtinScene());
  assert.equal(r.pairCount, 6);
  for (let i = 1; i < r.pairs.length; i++) {
    assert.ok(r.pairs[i - 1].minMargin <= r.pairs[i].minMargin + 1e-12,
      'pairs 必须按 minMargin 升序');
  }
  // 顶层字段即排序首对
  const tight = r.pairs[0];
  approx(r.minMargin, tight.minMargin, 1e-12);
  approx(r.minMarginT, tight.minMarginT, 1e-12);
  assert.deepEqual(r.minMarginPair, [tight.aId, tight.bId]);
  // 甲乙 t=6 完全重合、半径和 40 → 最小余量 -40；丙丁持续重叠但侵入更浅
  const p12 = r.pairs.find((p) => p.aId === 'p1' && p.bId === 'p2');
  approx(p12.minMargin, -40, 1e-6, '甲乙重合：0-(20+20)');
  approx(p12.minMarginT, 6, 1e-6);
  // 安全对余量为正
  const p13 = r.pairs.find((p) => p.aId === 'p1' && p.bId === 'p3');
  assert.ok(p13.minMargin > 0, '错峰不碰的一对余量为正');
});

test('最小余量：随机批量场景下与 collision 布尔一致，且报告时刻处余量自洽', () => {
  let seed = 7;
  const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
  for (let iter = 0; iter < 300; iter++) {
    const TT = 4 + Math.floor(rnd() * 8);
    const mk = () => {
      const keys = [];
      for (let k = 0; k < 3; k++) {
        const key = { t: Math.round(rnd() * TT), x: Math.round(rnd() * 200), y: Math.round(rnd() * 200) };
        if (rnd() < 0.4) key.r = Math.round(rnd() * 60);
        keys.push(key);
      }
      keys.sort((x, y) => x.t - y.t);
      return { id: 'x', radius: Math.round(rnd() * 30), keys };
    };
    const pa = mk();
    const pb = mk();
    const r = core.analyzePair(pa, pb, TT);
    // |margin| 远离 0 时必须与 collision 布尔一致
    if (Math.abs(r.minMargin) > 1e-6) {
      assert.equal(r.minMargin < 0, r.collision, `iter ${iter} margin=${r.minMargin}`);
    }
    // 报告时刻处直接求值应与 minMargin 吻合（自洽性）
    const sa = core.stateAt(pa.keys, r.minMarginT, pa.radius);
    const sb = core.stateAt(pb.keys, r.minMarginT, pb.radius);
    const g = Math.hypot(sa.x - sb.x, sa.y - sb.y) - sa.r - sb.r;
    approx(g, r.minMargin, 1e-5, `iter ${iter} 报告时刻余量自洽`);
  }
});
