// 발표 초안 — 강의자료(부산대 TECH WEEK Physical AI) p4 의 인지→계획→행동 순서를 따른다.
// 구성 근거: docs/발표_구성안.md (Stanley JFR 2006 순서, Peyton Jones·Winston 발표법).
//
// 다시 만들기:
//   npm install pptxgenjs            (한 번)
//   node docs/발표/build_deck.js docs/발표/발표_초안.pptx
// 글꼴은 시연 Mac 기준 "Apple SD Gothic Neo". Windows 용: DECK_FONT="Malgun Gothic" node ...
let pptxgen;
try { pptxgen = require("pptxgenjs"); } catch (e) { pptxgen = require("/tmp/deck/node_modules/pptxgenjs"); }

const OUT = process.argv[2] || "docs/발표/발표_초안.pptx";
const pres = new pptxgen();
pres.layout = "LAYOUT_16x9"; // 10 x 5.625 in
pres.title = "sar_rescue — 절대 위치 없이 빨간 사과를 찾아 돌아오기";

// 시연 Mac 에 있는 한글 글꼴. Windows 에서 열면 PowerPoint 가 맑은 고딕 등으로 대신 보여 준다.
const FONT = process.env.DECK_FONT || "Apple SD Gothic Neo";
const C = {
  dark: "1B2631", ink: "1B2631", sub: "5D6D7E", light: "F2F4F4", white: "FFFFFF",
  red: "C0392B", teal: "117A65", amber: "B9770E", purple: "6C3483", line: "D5D8DC",
};
const STAGE = {
  "인지": { color: C.teal, pages: "강의 p8–92" },
  "계획": { color: C.amber, pages: "강의 p93–103" },
  "행동": { color: C.purple, pages: "강의 p104–147" },
};

function text(slide, str, opts) {
  slide.addText(str, Object.assign({ fontFace: FONT, color: C.ink, isTextBox: true, margin: 0 }, opts));
}

function header(slide, title, stage, page) {
  if (stage) {
    const s = STAGE[stage];
    slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x: 0.5, y: 0.32, w: 1.55, h: 0.32, rectRadius: 0.16, fill: { color: s.color }, line: { color: s.color },
    });
    text(slide, stage + (page ? "  ·  " + page : ""), {
      x: 0.5, y: 0.32, w: 1.55, h: 0.32, fontSize: 10, bold: true, color: C.white, align: "center", valign: "middle",
    });
  }
  text(slide, title, { x: 0.5, y: stage ? 0.72 : 0.4, w: 9.0, h: 0.6, fontSize: 26, bold: true, valign: "top" });
}

function gif(slide, x, y, w, h, label) {
  slide.addShape(pres.shapes.RECTANGLE, {
    x, y, w, h, fill: { color: C.light }, line: { color: "95A5A6", width: 1.25, dashType: "dash" },
  });
  text(slide, "▶  " + label, {
    x: x + 0.15, y, w: w - 0.3, h, fontSize: 12, color: C.sub, align: "center", valign: "middle",
  });
}

// 결정 5칸 표 (Stanley 5.2 의 틀: 문제를 숫자로 → 방법 → 측정으로 확인)
function decision(slide, rows, x, y, w, colW) {
  const labels = ["① 관찰", "② 가설", "③ 기각한 대안", "④ 기준값의 출처", "⑤ 확인 · 한계"];
  const data = rows.map((r, i) => [
    { text: labels[i], options: { bold: true, color: C.white, fill: { color: C.dark }, fontSize: 10.5, valign: "middle" } },
    { text: r, options: { color: C.ink, fontSize: 10.5, valign: "middle", fill: { color: i % 2 ? C.white : C.light } } },
  ]);
  slide.addTable(data, {
    x, y, w, colW: [colW, w - colW], fontFace: FONT, border: { type: "solid", pt: 0.5, color: C.line },
    margin: [0.04, 0.08, 0.04, 0.08],
  });
}

function bigStat(slide, x, y, w, num, label, color) {
  text(slide, num, { x, y, w, h: 0.75, fontSize: 40, bold: true, color: color || C.red });
  text(slide, label, { x, y: y + 0.78, w, h: 0.45, fontSize: 11, color: C.sub, valign: "top" });
}

// ───────────────────────── 1. 표지 ─────────────────────────
{
  const s = pres.addSlide();
  s.background = { color: C.dark };
  s.addShape(pres.shapes.OVAL, { x: 7.6, y: 1.35, w: 1.5, h: 1.5, fill: { color: C.red }, line: { color: C.red } });
  text(s, "절대 위치 없이\n빨간 사과를 찾아 돌아오기", { x: 0.6, y: 1.2, w: 6.8, h: 1.6, fontSize: 34, bold: true, color: C.white, valign: "top" });
  text(s, "Autonomous Mobile Robot의 Search & Rescue — Webots apartment, TurtleBot3 Burger", {
    x: 0.6, y: 3.0, w: 8.5, h: 0.4, fontSize: 13, color: "CAD3DA" });
  text(s, "팀 이름  ·  팀원 이름 (여기에 적기)", { x: 0.6, y: 4.5, w: 8.5, h: 0.4, fontSize: 13, color: "CAD3DA" });
  s.addNotes("시작 30초 — 농담이 아니라 약속으로 시작한다 (Winston, How to Speak).\n" +
    "대본: \"이 발표가 끝나면, GPS 없이 바퀴와 나침반과 LiDAR 만으로 로봇이 자기 위치가 얼마나 틀리는지 스스로 알아채는 방법, 그리고 우리가 기준 숫자 하나하나를 어떻게 정했는지를 보시게 됩니다.\"\n" +
    "목차 슬라이드는 두지 않는다 — Peyton Jones: \"Outline of my talk: conveys near zero information at the start\".\n" +
    "공저자(팀원)는 표지에서 밝힌다 (Peyton Jones).");
}

// ───────────────────────── 2. 과제와 문제 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "과제: 모르는 집에서 빨간 사과 2개를 찾아 가까이 가고, 시작점으로 돌아온다");
  const rules = [
    "지도 없음 — 집 구조를 모른다", "GNSS·절대 위치(Supervisor) 금지",
    "기본 로봇 속도 초과 금지", "보행자 1명이 로봇을 피하지 않고 걷는다 (0.2 m/s)",
  ];
  text(s, rules.map((r, i) => ({ text: r, options: { bullet: true, breakLine: i < rules.length - 1 } })), {
    x: 0.5, y: 1.55, w: 4.3, h: 1.9, fontSize: 14, paraSpaceAfter: 8, valign: "top" });
  bigStat(s, 0.5, 3.6, 4.3, "0 / 2", "첫 완주: 소화기를 사과로 알고 방문했다 — 문제의 크기");
  gif(s, 5.2, 1.55, 4.3, 3.3, "GIF 1 — 전체 주행 타임랩스 (시작 → 사과 2개 → 복귀)");
  s.addNotes("동기 2분 (Peyton Jones: 동기 20%, 첫 2분에 '무슨 문제이고 왜 어려운가').\n" +
    "숫자 하나로 어려움을 보인다: 첫 완주에서 빨간 사과 0/2 — 로봇은 소화기를 사과로 확정하고 방문했다.\n" +
    "GIF 1 로 '해냈다'를 먼저 보여 준다 (Winston: 첫 5분에 비전과 한 일).\n" +
    "근거: 대회 계획안·질의응답, 월드 파일 worlds/apartment.wbt (보행자 --speed=0.2).");
}

// ───────────────────────── 3. 핵심 문장 ─────────────────────────
{
  const s = pres.addSlide();
  s.background = { color: C.dark };
  text(s, "모든 문턱값은 측정에서 왔고,\n켜서 나빠진 기능은 뺐다.", {
    x: 0.6, y: 1.3, w: 8.8, h: 1.7, fontSize: 32, bold: true, color: C.white, valign: "top" });
  text(s, "근거: 녹화·재생으로 같은 주행에서 방법을 바꿔 끼워 비교 · 뺀 것의 기록 docs/무엇을-빼기로-했나.md", {
    x: 0.6, y: 3.4, w: 8.8, h: 0.5, fontSize: 13, color: "CAD3DA" });
  s.addNotes("핵심 아이디어 한 문장 30초. Stanley(JFR 2006) 3.1 이 설계 원칙을 한 문장으로 내세운 것처럼 — \"Treat autonomous navigation as a software problem.\"\n" +
    "이 문장을 각 절 끝에서 반복한다 (Winston: cycling).");
}

// ───────────────────────── 4. 로봇과 센서 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "로봇과 센서 — LiDAR 는 바닥에서 17 cm 높이의 평면만 본다", "인지", "p9–14");
  const rows = [
    ["센서", "무엇을 주나", "주기", "우리가 쓰는 곳"],
    ["바퀴 엔코더", "바퀴 누적 회전각", "64 ms", "이동 거리"],
    ["나침반", "북쪽 벡터 (방위)", "64 ms", "방향 θ — 누적 오차 없음"],
    ["LiDAR LDS-01", "360점, 0.12–3.5 m, 평면 1층", "64 ms", "지도 · 사람 · 충돌 회피"],
    ["카메라", "640×480, 시야 60°", "128 ms", "사과 (색 · 모양 · YOLO)"],
    ["GPS", "—", "—", "달지 않는다 (금지)"],
  ];
  s.addTable(rows.map((r, i) => r.map(c => ({ text: c, options: i === 0
    ? { bold: true, color: C.white, fill: { color: C.dark } } : { fill: { color: i % 2 ? C.white : C.light } } }))), {
    x: 0.5, y: 1.5, w: 5.6, colW: [1.25, 1.85, 0.7, 1.8], fontFace: FONT, fontSize: 10.5,
    border: { type: "solid", pt: 0.5, color: C.line }, margin: [0.05, 0.08, 0.05, 0.08] });
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 6.4, y: 1.5, w: 3.1, h: 1.75, rectRadius: 0.1,
    fill: { color: C.light }, line: { color: C.light } });
  text(s, "복선", { x: 6.6, y: 1.65, w: 2.8, h: 0.35, fontSize: 13, bold: true, color: C.red });
  text(s, [
    { text: "카펫 두께 2 cm", options: { bullet: true, breakLine: true } },
    { text: "사과 지름 10 cm", options: { bullet: true, breakLine: true } },
    { text: "→ 둘 다 LiDAR 에 안 보인다", options: { bold: true } },
  ], { x: 6.6, y: 2.05, w: 2.8, h: 1.1, fontSize: 13, paraSpaceAfter: 6, valign: "top" });
  s.addNotes("30초. '17 cm 평면' 을 여기서 말해 둔다 — 뒤의 카펫(2 cm, Carpet.proto:209)과 사과(지름 10 cm) 문제의 복선.\n" +
    "주기 64 ms 는 대회 월드 basicTimeStep. 16 ms 기준으로 정한 틱 상수는 시작 시 같은 '시간' 이 되게 다시 계산한다.");
}

// ───────────────────────── 5. 파이프라인 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "강의의 파이프라인 그대로: 인지 → 계획 → 행동 (강의 p4)");
  const cols = [
    ["인지", "데이터 수집·분석", ["위치 추정 (엔코더+나침반)", "점유 격자 지도 (log-odds)", "사과: 색 → 두 거리 → YOLO", "사람: 지도에 없고 움직이는 것 + 칼만"]],
    ["계획", "목표·경로 계획", ["Costmap: 벽 0.35 m 팽창", "사람 둘레 비용", "Global Planner: A*", "0.3초마다 재계획"]],
    ["행동", "동작 제어", ["탐험: 프론티어 (강의 p104–105)", "Local Planner: DWA → 바퀴", "Decision Making: FSM", "Recovery: 되짚기·탈출·후진"]],
  ];
  cols.forEach(([name, sub, items], i) => {
    const x = 0.5 + i * 3.1, col = STAGE[name].color;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 1.45, w: 2.8, h: 0.95, rectRadius: 0.1, fill: { color: col }, line: { color: col } });
    text(s, name, { x, y: 1.5, w: 2.8, h: 0.5, fontSize: 20, bold: true, color: C.white, align: "center" });
    text(s, sub + "  ·  " + STAGE[name].pages, { x, y: 1.98, w: 2.8, h: 0.35, fontSize: 10, color: C.white, align: "center" });
    if (i < 2) s.addShape(pres.shapes.RIGHT_ARROW, { x: x + 2.83, y: 1.78, w: 0.24, h: 0.3, fill: { color: C.sub }, line: { color: C.sub } });
    text(s, items.map((t, k) => ({ text: t, options: { bullet: true, breakLine: k < items.length - 1 } })), {
      x: x + 0.05, y: 2.6, w: 2.75, h: 2.3, fontSize: 11.5, paraSpaceAfter: 6, valign: "top" });
  });
  s.addNotes("강의자료 p4·p8·p93·p106·p148 의 인지(Perception) → 계획(Planning) → 행동(Action) 을 그대로 따른다.\n" +
    "주의: 강의는 프론티어 탐험(p104–105)과 FSM·Behavior Tree·Recovery(p127–147)를 '행동' 에 넣었다. 우리도 그 분류를 따른다.\n" +
    "SLAM(p92)은 인지의 끝 — 위치 추정과 지도를 한 틱에 함께 갱신한다.");
}

// ───────────────────────── 6. 개발 방식 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "어떻게 판단했나 — 녹화하고, 재생하고, 같은 주행에서 비교한다");
  const steps = [
    ["기록", "매 틱 LiDAR 스캔 · 추정 위치 · 정답 위치를 녹화 (tape.npz)"],
    ["재생", "Webots 없이 녹화를 다시 돌리며 방법만 바꿔 끼운다"],
    ["채점", "정답 위치는 채점용 월드 사본에서만 (Supervisor) — 주행 코드엔 없다, 자동 검사"],
    ["재현", "시뮬레이션은 결정적 — 같은 설정이면 로그가 소수점까지 같다"],
  ];
  steps.forEach(([k, v], i) => {
    const y = 1.5 + i * 0.85;
    s.addShape(pres.shapes.OVAL, { x: 0.5, y, w: 0.6, h: 0.6, fill: { color: C.dark }, line: { color: C.dark } });
    text(s, String(i + 1), { x: 0.5, y, w: 0.6, h: 0.6, fontSize: 18, bold: true, color: C.white, align: "center", valign: "middle" });
    text(s, k, { x: 1.3, y: y + 0.02, w: 1.1, h: 0.55, fontSize: 15, bold: true, valign: "middle" });
    text(s, v, { x: 2.4, y: y + 0.02, w: 7.1, h: 0.55, fontSize: 13, color: C.sub, valign: "middle" });
  });
  s.addNotes("1분. Stanley(JFR 2006) 3.1.4 와 같은 방식이다: \"all data are logged. By using a special replay module, the software can be run on recorded data.\"\n" +
    "예: 모듈 실행과 제출 파일 실행의 상태 로그 605줄이 한 글자도 다르지 않았다. GPS→Supervisor 로 채점 방식을 바꾼 뒤에도 결과가 소수점까지 같았다.");
}

// ───────────────────────── 7. 인지: 위치 추정 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "위치 추정 — 로봇이 자기 미끄러짐을 알아채게", "인지", "p57–62, 92");
  decision(s, [
    "복귀했다고 판단했는데 실제로는 시작점에서 1.99 m. 팀이 GUI 에서 '카펫에서 오차가 67.8 cm 로 뛴다' 를 발견",
    "방향은 나침반(절대 방위)이라 안 틀어진다 → 틀어지는 건 '거리'. (a) 바닥 탓 일정 비율 (b) 카펫 턱 미끄러짐",
    "스캔 매칭 그냥 켜기: 오차 21 → 164 cm, 사과 1 → 0. 벽 근처 비용 기울기: 오차 951 cm",
    "(a) 직진 38구간 '바퀴 거리 / 실제 거리' 중앙값 0.983 → 반지름 0.033/0.983\n(b) 나침반 회전 / 명령 회전: 평소 0.87–1.14, 카펫 턱 0–0.3 → 문턱 0.6 rad/s · 0.3초",
    "(a) 보정 후 비율 1.0015 (b) 녹화 재생에서 카펫 사건과 185 cm 튐을 모두 감지. 실제 주행 점수 개선은 측정 중",
  ], 0.5, 1.45, 5.9, 1.25);
  gif(s, 6.6, 1.45, 2.9, 3.6, "GIF 2 — 카펫 턱 미끄러짐 (명령은 제자리 회전, 몸은 옆으로)");
  s.addNotes("2분. 식: Δs = r(ΔφL+ΔφR)/2, θ = atan2(m_x, m_y) (나침반 벡터 = (sinθ, cosθ)), 중점법 적분.\n" +
    "선 긋기(near miss, Winston): '벽에 끼어 헛도는 것과 다르다. 헛돌면 몸은 제자리인데 추정이 앞으로 가고, 이건 추정은 제자리인데 몸이 옆으로 간다.' 그래서 기존 끼임 감지(추정 위치 기준)로는 못 잡는다.\n" +
    "빈 월드에서는 64 ms 로 돌려도 비율 1.002 → 물리 주기 탓이 아님을 배제했다.\n" +
    "백업: docs/발표_기술해설.md 3장.");
}

// ───────────────────────── 8. 인지: 지도 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "지도 — 확률을 덧셈으로 (베이즈 필터의 log-odds)", "인지", "p53–55");
  text(s, "l = log( p / (1 − p) )        l ← l + (맞음 +0.9 / 지나감 −0.4),   l ∈ [−3, +5]", {
    x: 0.5, y: 1.5, w: 9.0, h: 0.5, fontSize: 16, bold: true, color: C.teal });
  bigStat(s, 0.5, 2.3, 2.8, "1번", "벽은 한 번 맞으면 바로 '막힘'  (0.9 ≥ 0.8)", C.teal);
  bigStat(s, 3.5, 2.3, 2.8, "14번", "굳은 벽(+5)을 지우려면 광선이 지나가야 하는 횟수", C.red);
  bigStat(s, 6.5, 2.3, 3.0, "15 cm", "보행자가 문 한가운데 남긴 가짜 벽 → 방이 갇혀 보였다", C.red);
  text(s, "→ 그래서 복귀는 '지도에 길이 없으면 지나온 길을 되짚는다' (행동 · Recovery)", {
    x: 0.5, y: 4.2, w: 9.0, h: 0.45, fontSize: 13, color: C.sub });
  s.addNotes("log-odds 로 쓰면 베이즈 갱신 P(벽|z) ∝ P(z|벽)P(벽) 이 곱셈에서 덧셈이 된다. 5 cm 칸 320×320 = 16×16 m.\n" +
    "역센서 모델 값: +0.9 (p=0.71), −0.4 (p=0.40). 판정: 막힘 l ≥ 0.8, 빔 l ≤ −0.4.\n" +
    "14번 = ceil((5 + 0.4)/0.4). 이것이 가짜 벽이 오래 남는 이유다.");
}

// ───────────────────────── 9. 인지: 사과 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "사과 인식 — 가짜를 하나씩 이름 붙여 걸렀다", "인지", "p29–50");
  text(s, "카메라 한 장으로 거리를 두 번 잰다:   d크기 = R / sin(γ)      d바닥 = h / tan(δ)      → 둘이 맞아야 바닥의 공", {
    x: 0.5, y: 1.4, w: 9.0, h: 0.4, fontSize: 13, bold: true, color: C.teal });
  const rows = [
    ["가짜", "무엇을 봤나", "걸러낸 기준", "기준의 출처"],
    ["가까운 소화기", "첫 완주에서 확정·방문", "두 거리 차 ≤ 30%", "사과 1–7%, 소화기 81%"],
    ["먼 소화기", "195초 재방문 (23×72 px)", "세로/가로 ≤ 1.5", "거리 차 29% 로 통과한 사례"],
    ["누운 캔", "사람 검출 켠 실행마다 방문", "세로/가로 ≥ 0.67", "캔 26×11 = 0.42, 사과 1.0–1.2"],
    ["병·컵", "(예비)", "YOLO: bottle·cup 이면 버림", "사과 23장 중 23장 sports ball"],
  ];
  s.addTable(rows.map((r, i) => r.map(c => ({ text: c, options: i === 0
    ? { bold: true, color: C.white, fill: { color: C.dark } } : { fill: { color: i % 2 ? C.white : C.light } } }))), {
    x: 0.5, y: 1.95, w: 6.1, colW: [1.1, 1.7, 1.55, 1.75], fontFace: FONT, fontSize: 10,
    border: { type: "solid", pt: 0.5, color: C.line }, margin: [0.05, 0.07, 0.05, 0.07] });
  text(s, "YOLO 는 '문지기'가 아니라 '거부권' — 멀리 있는 사과를 자주 놓치기 때문", {
    x: 0.5, y: 4.55, w: 6.1, h: 0.5, fontSize: 11.5, color: C.red, bold: true, valign: "top" });
  gif(s, 6.8, 1.95, 2.7, 3.1, "GIF 3 — 소화기가 보이는데 확정되지 않고 지나침");
  s.addNotes("Stanley 5.2 처럼 오탐을 숫자로 보인다 (\"false positive rate drops from 12.6% to 0.002%\").\n" +
    "사과는 지름 10 cm 라 LiDAR(17 cm) 로 거리를 못 잰다 → 카메라만으로. R=0.05 m, h=0.073 m, 초점거리 554 px.\n" +
    "녹화 668장 재판정: 소화기 오탐 10 → 0, YOLO 는 24장에서만 돌았다 (CPU 약 90 ms/장).\n" +
    "YOLO 를 문지기로 안 쓴 이유: 초기 측정에서 빨간 픽셀과 겹친 YOLO 검출이 390장 중 3장뿐.\n" +
    "25번 보면 확정 (카메라 128 ms → 최소 3.2초 관찰).");
}

// ───────────────────────── 10. 인지: 사람 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "보행자 — 도망이 아니라 예측 (칼만 필터)", "인지", "p76–77");
  decision(s, [
    "사람 검출을 끈 실행에서 보행자와 3.8초 닿음",
    "월드 파일: 보행자 --speed=0.2 = 로봇 최고 속도와 같다. Webots 보행자는 로봇을 피하지 않는다 → 갈 곳을 예측해야 한다",
    "사람 없는 연습 월드에서 켜면 헛것 100% (maze0: 3,237회) → 연습 월드에선 끔. 두 스캔 차이로 낸 속도는 잡음(위치 오차 23 cm)이 더 컸다",
    "측정 잡음 σz = 0.23 m (실측 위치 오차) · 예측 1.2초 = DWA 가 굴려 보는 시간과 같게",
    "접촉 3.8초 → 0초 (사람 검출 350초, + 칼만 342초 완주)",
  ], 0.5, 1.45, 6.0, 1.25);
  gif(s, 6.7, 1.45, 2.8, 3.6, "GIF 4 — 보행자 앞에서 멈추거나 비켜 보내기");
  s.addNotes("후보: 지도의 '빈 칸' 위에 찍힌 LiDAR 조각 (모양·색 무관) → 9스캔 중 2번 이상, 0.15 m/s 이상 움직였어야 사람.\n" +
    "등속 칼만: x=(x,y,vx,vy), F=[[1,0,Δt,0],[0,1,0,Δt],[0,0,1,0],[0,0,0,1]], Q=σa²GGᵀ (σa=1.0), H=[I 0], R=σz² I.\n" +
    "DWA 는 사람이 0.4·0.8·1.2초 뒤 있을 자리를 장애물로 본다. 전역 계획은 사람 둘레 1.2 m 를 비싸게.");
}

// ───────────────────────── 11. 계획 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "Costmap 과 A* — 벽을 로봇 크기만큼 부풀린다", "계획", "p96–103");
  bigStat(s, 0.5, 1.45, 3.0, "0.35 m", "팽창 = 몸 반경 0.13 + 안전거리 0.22 (정지거리 0.23 과 맞물림)", C.amber);
  bigStat(s, 3.6, 1.45, 3.0, "0.28 m", "좁은 문에서 길이 없을 때 한 번 더 — '좁은 여유'", C.amber);
  bigStat(s, 6.7, 1.45, 2.8, "0.3 s", "재계획 주기 + 경로가 막히면 즉시", C.amber);
  const items = [
    "A*: f = g + h, 8방향, 대각선은 옆 두 칸이 모두 열려 있을 때만 (모서리 스치기 금지)",
    "h = 옥타일 거리 — 칸 비용 ≥ 1 이라 과대추정하지 않는다 → 최단 경로 보장",
    "사람 둘레 1.2 m 는 비용을 최대 4배로 (강의 Costmap 의 사회적 비용 층)",
    "칸 경로를 직선으로 보이는 가장 먼 점까지 건너뛰어 꺾이는 곳만 남긴다",
  ];
  text(s, items.map((t, k) => ({ text: t, options: { bullet: true, breakLine: k < items.length - 1 } })), {
    x: 0.5, y: 3.0, w: 9.0, h: 2.0, fontSize: 12.5, paraSpaceAfter: 6, valign: "top" });
  s.addNotes("강의 p93–103 (Global Planner / Global Path / Costmap / A*).\n" +
    "Dijkstra 대신 A* 인 이유: 같은 최단 경로를 휴리스틱으로 더 적게 탐색. D* Lite 도 구현해 비교했지만 320×320 에서는 A* 로 충분 (docs/계획기-비교-Astar-vs-DstarLite.md).\n" +
    "백업: docs/발표_기술해설.md 9장.");
}

// ───────────────────────── 12. 행동: 탐험 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "탐험 — 프론티어 하나, 규칙 하나: A* 경로가 가장 짧은 곳", "행동", "p104–105");
  const steps = ["빈 칸 옆의 모르는 칸 = 프론티어", "덩어리로 묶고 설 자리를 정한다", "A* 로 정말 갈 수 있는지 확인", "경로가 가장 짧은 곳으로"];
  steps.forEach((t, i) => {
    const x = 0.5 + i * 2.3;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 1.5, w: 2.1, h: 1.0, rectRadius: 0.1, fill: { color: C.light }, line: { color: C.light } });
    text(s, (i + 1) + ". " + t, { x: x + 0.1, y: 1.5, w: 1.9, h: 1.0, fontSize: 12, valign: "middle" });
  });
  decision(s, [
    "강의 p105 는 거리·예상 cost·새로 탐색할 영역 크기 등 기준을 제안한다",
    "여러 점수를 섞으면 월드마다 최선이 달라진다",
    "크기 가점·회전 비용·카메라 이득을 섞은 버전: 월드마다 들쭉날쭉 → 뺐다",
    "연습 16월드: 최소 규칙으로 46–48 / 48",
    "한계: 실패한 목표가 3번 실패 전까지 다시 뽑힌다 · 가짜 벽이면 일찍 포기 (61초 사례)",
  ], 0.5, 2.75, 9.0, 1.6);
  s.addNotes("강의는 탐험을 '행동' 에 넣었다 (p104 탐험, p105 Frontier Exploration 과정). 그 분류를 따른다.\n" +
    "점수 = 로봇→경로 첫 점 거리 + 경로 길이. 4칸 미만 덩어리는 잡음, 설 자리가 0.7 m 넘게 밀리면 버림.\n" +
    "팀 관찰: '갔던 곳을 여러 번 가고, 갈 필요 없는 곳도 간다' → 원인 두 가지를 한계로 먼저 말한다.");
}

// ───────────────────────── 13. 행동: Local Planner ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "Local Planner — DWA 가 매 틱 바퀴에 넣을 값을 고른다", "행동", "p109–125");
  const flow = [
    ["동적 창", "가속 한계 안의 (v, ω)\n7 × 17 = 119 후보"],
    ["예측", "1.2초 원호 궤적\n8점"],
    ["거르기", "장애물·사람 예측점에\n0.28 m 보다 가까우면 버림"],
    ["점수", "조준점에 가깝게 + 장애물에서 멀게\n후진·정지 감점"],
  ];
  flow.forEach(([k, v], i) => {
    const x = 0.5 + i * 2.3;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 1.5, w: 2.1, h: 1.45, rectRadius: 0.1, fill: { color: C.purple }, line: { color: C.purple } });
    text(s, k, { x: x + 0.1, y: 1.55, w: 1.9, h: 0.4, fontSize: 14, bold: true, color: C.white });
    text(s, v, { x: x + 0.1, y: 1.95, w: 1.9, h: 0.95, fontSize: 10.5, color: C.white, valign: "top" });
  });
  text(s, "ωL = (v − ωB/2) / r      ωR = (v + ωB/2) / r      한쪽이 6.67 rad/s 를 넘으면 두 바퀴를 같은 비율로 줄인다", {
    x: 0.5, y: 3.25, w: 9.0, h: 0.45, fontSize: 13, bold: true, color: C.purple });
  text(s, [
    { text: "같은 비율로 줄이는 이유: 한쪽만 자르면 회전 반경이 바뀐다 — 전진하며 최대로 돌 때 바깥 바퀴가 한계를 12–44% 넘는다", options: { bullet: true, breakLine: true } },
    { text: "속도 규칙: 전진 상한 0.20 m/s (Burger 약 0.22), 모터 한계는 시작할 때 Webots 에서 읽어 대조", options: { bullet: true } },
  ], { x: 0.5, y: 3.85, w: 9.0, h: 1.2, fontSize: 12, paraSpaceAfter: 6, valign: "top" });
  s.addNotes("강의 p108–111 (Local Planner/Controller, DWA·TEB·MPPI), p116–125 (Look-ahead, 역기구학).\n" +
    "조준점 = 경로를 따라 max(0.20, 0.2×1.2)=0.24 m 앞. 창 폭 ±0.08 m/s, ±0.53 rad/s 라 119 후보도 촘촘하다 (팀 질문: '후보가 너무 적지 않나?' 의 답).\n" +
    "점수 J = 1.0·N(−|p8−g|) + 0.8·N(min(c,0.8)) − 2.0[v<0] − 1.9[|v|≤0.01].");
}

// ───────────────────────── 14. 행동: Decision Making & Recovery ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "Decision Making — FSM 과 Recovery", "행동", "p127–147");
  const states = ["SCAN", "EXPLORE", "APPROACH", "RETURN", "DONE"];
  states.forEach((st, i) => {
    const x = 0.5 + i * 1.85;
    s.addShape(pres.shapes.OVAL, { x, y: 1.45, w: 1.5, h: 0.75, fill: { color: i === 3 ? C.red : C.dark }, line: { color: C.dark } });
    text(s, st, { x, y: 1.45, w: 1.5, h: 0.75, fontSize: 12, bold: true, color: C.white, align: "center", valign: "middle" });
  });
  const rec = [
    ["지나온 길 되짚기", "지도에 길이 3초 없으면 기록한 길을 거꾸로 (0.3 m 안 고리는 건너뜀)", "가짜 벽에 갇힘: 447초 시간 초과 → 297.6초 복귀"],
    ["눌림 탈출", "0.20 m 안 광선이 2개 이상이면 반대쪽으로 (공간 확인 후)", "광선 1개일 때 실제 거리 중앙값 339 cm (잡음), 3개↑ 33–36 cm"],
    ["목표 없어도 복귀", "사과를 못 찾아도 RETURN", "예전엔 61초 포기 후 시작점 4.55 m 앞에서 끝났다"],
  ];
  s.addTable([["Recovery", "무엇을", "기준의 출처"]].concat(rec).map((r, i) => r.map(c => ({ text: c, options: i === 0
    ? { bold: true, color: C.white, fill: { color: C.dark } } : { fill: { color: i % 2 ? C.white : C.light } } }))), {
    x: 0.5, y: 2.45, w: 6.1, colW: [1.35, 2.45, 2.3], fontFace: FONT, fontSize: 10,
    border: { type: "solid", pt: 0.5, color: C.line }, margin: [0.05, 0.07, 0.05, 0.07] });
  gif(s, 6.8, 2.45, 2.7, 2.6, "GIF 5 — 막힌 방에서 지나온 길을 되짚어 나오기");
  s.addNotes("강의 p128–130: FSM 은 '상태가 적고 서로 복잡하게 얽히지 않을 때', BT 는 '예외 처리가 복잡할 때'. 우리 상태는 6개(SCAN·EXPLORE·SWEEP·APPROACH·RETURN·DONE)라 FSM 을 골랐다.\n" +
    "Recovery 는 강의 p142 의 Recovery Node 개념(주 행동 실패 시 회복 행동)과 같은 역할을 FSM 안에서 한다.\n" +
    "시간 예산: 제한 900초에서 1.5×(집까지 A* 경로 / 평균 속도) (최소 30초)를 남기고 무조건 복귀.");
}

// ───────────────────────── 15. 시연 ─────────────────────────
{
  const s = pres.addSlide();
  s.background = { color: C.dark };
  text(s, "Webots 시연", { x: 0.6, y: 0.5, w: 8.8, h: 0.7, fontSize: 32, bold: true, color: C.white });
  text(s, "지금부터 볼 것", { x: 0.6, y: 1.4, w: 8.8, h: 0.4, fontSize: 15, color: "CAD3DA" });
  const watch = ["① 빨간 사과를 확정하는 순간 (카메라 오버레이)", "② 보행자 앞에서 멈추거나 비켜 보내는 순간", "③ 시작점으로 돌아와 멈추는 순간"];
  text(s, watch.map((t, k) => ({ text: t, options: { breakLine: k < watch.length - 1 } })), {
    x: 0.6, y: 1.9, w: 8.8, h: 1.8, fontSize: 20, color: C.white, paraSpaceAfter: 10, valign: "top" });
  text(s, "라이브가 안 되면 같은 설정의 녹화 GIF — 시뮬레이션은 결정적이라 같은 장면이 나온다", {
    x: 0.6, y: 4.3, w: 8.8, h: 0.5, fontSize: 12, color: "CAD3DA" });
  s.addNotes("Stanley §10.3 'The Race' 처럼 무엇이 일어났는지 말하며 보여 준다.\n" +
    "전체 주행은 약 7분(433초) — 발표 시작 때 실시간으로 켜 두고 이 순서에 화면을 맞추거나, 결정적 장면은 GIF 로.\n" +
    "Peyton Jones: \"Laptops break: leave a backup copy\" — GIF 를 백업으로.");
}

// ───────────────────────── 16. 결과 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "결과 — 무엇을 더했을 때 무엇이 좋아졌나");
  const rows = [
    ["단계", "빨간 사과", "보행자 접촉", "복귀"],
    ["첫 완주", "0 / 2 (소화기)", "—", "—"],
    ["+ 두 거리 일치 검사", "1 / 2", "3.8초", "성공"],
    ["+ 사람 검출 · 칼만", "1 / 2 (+ 캔 오탐)", "0초", "성공"],
    ["+ 모양 검사 · YOLO", "2 / 2", "0초", "로봇은 성공 판단 — 실제 1.99 m 모자람"],
  ];
  s.addTable(rows.map((r, i) => r.map(c => ({ text: c, options: i === 0
    ? { bold: true, color: C.white, fill: { color: C.dark } }
    : { fill: { color: i === 4 ? "FDEDEC" : (i % 2 ? C.white : C.light) }, bold: i === 4 } }))), {
    x: 0.5, y: 1.5, w: 9.0, colW: [2.3, 2.0, 1.4, 3.3], fontFace: FONT, fontSize: 13,
    border: { type: "solid", pt: 0.5, color: C.line }, margin: [0.08, 0.1, 0.08, 0.1] });
  text(s, "apartment, 보행자 있음, 설정마다 한 번 실행 (결정적). 측정: debug/mission_check.py", {
    x: 0.5, y: 4.55, w: 9.0, h: 0.4, fontSize: 11, color: C.sub });
  s.addNotes("Stanley §11 Discussion 처럼 결과를 표로. 한 줄씩 무엇을 더했는지와 그 효과만 말한다.\n" +
    "마지막 줄을 우리가 먼저 짚는다: 로봇은 복귀했다고 판단했지만 실제로는 1.99 m 모자랐다 — 다음 장의 한계로 이어진다.");
}

// ───────────────────────── 17. 한계 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "한계 — 우리가 먼저 말한다");
  const lim = [
    ["카펫 미끄러짐", "한 번 걸리면 위치가 20–50 cm 튄다. 몇 번 걸리느냐는 경로에 따라 운이다"],
    ["스캔 매칭", "녹화 재생에선 오차 186 → 21 cm. 실제 주행 점수 개선은 아직 확인 못 해 꺼 두었다"],
    ["낮은 물체", "다른 색 사과·캔은 LiDAR 에 안 보인다. 카메라로 계획 지도에 넣는 기능은 측정 중"],
    ["측정 표본", "월드 하나, 설정마다 한 번 실행"],
  ];
  lim.forEach(([k, v], i) => {
    const y = 1.45 + i * 0.85;
    s.addShape(pres.shapes.OVAL, { x: 0.5, y: y + 0.05, w: 0.5, h: 0.5, fill: { color: C.red }, line: { color: C.red } });
    text(s, "!", { x: 0.5, y: y + 0.05, w: 0.5, h: 0.5, fontSize: 18, bold: true, color: C.white, align: "center", valign: "middle" });
    text(s, k, { x: 1.2, y, w: 1.9, h: 0.6, fontSize: 15, bold: true, valign: "middle" });
    text(s, v, { x: 3.1, y, w: 6.4, h: 0.6, fontSize: 12.5, color: C.sub, valign: "middle" });
  });
  s.addNotes("Peyton Jones: 질문은 기회다. 과장하지 않는다.\n" +
    "스캔 매칭 두 방식(격자 correlative, 정밀 Gauss-Newton)의 재생 비교표는 백업 슬라이드 / docs/발표_기술해설.md 3.6.");
}

// ───────────────────────── 18. 기여 ─────────────────────────
{
  const s = pres.addSlide();
  s.background = { color: C.dark };
  text(s, "기여", { x: 0.6, y: 0.45, w: 8.8, h: 0.7, fontSize: 32, bold: true, color: C.white });
  const contrib = [
    "절대 위치 없이, 나침반 대 바퀴 비교로 로봇이 자기 미끄러짐을 알아챈다",
    "LiDAR 없이 카메라 한 장으로 거리를 두 번 재서 사과와 가짜(소화기·캔)를 가른다",
    "보행자와 속도가 같은 상황에서 칼만 예측으로 접촉 0초",
    "모든 문턱값에 측정 출처가 있고, 켜서 나빠진 기능은 기록과 함께 뺐다",
  ];
  contrib.forEach((t, i) => {
    const y = 1.35 + i * 0.85;
    s.addShape(pres.shapes.OVAL, { x: 0.6, y, w: 0.55, h: 0.55, fill: { color: C.red }, line: { color: C.red } });
    text(s, String(i + 1), { x: 0.6, y, w: 0.55, h: 0.55, fontSize: 16, bold: true, color: C.white, align: "center", valign: "middle" });
    text(s, t, { x: 1.4, y: y - 0.02, w: 8.0, h: 0.6, fontSize: 15, color: C.white, valign: "middle" });
  });
  s.addNotes("Winston: 마지막 장은 '기여' — 'Questions?' 나 '감사합니다' 로 끝내지 않는다. 이 장을 띄워 둔 채 질문을 받는다.\n" +
    "첫 장의 약속(자기 오차를 알아채는 방법, 기준을 정한 방법)을 지켰음을 한 문장으로 확인한다.");
}

// ───────────────────────── 19. 출처 ─────────────────────────
{
  const s = pres.addSlide();
  header(s, "출처");
  const refs = [
    "부산대 TECH WEEK Physical AI 강의자료 (김규래, 2026) — 인지·계획·행동 파이프라인 p4, 쪽 번호는 각 슬라이드",
    "S. Thrun et al., Stanley: The Robot that Won the DARPA Grand Challenge, J. Field Robotics 23(9):661–692, 2006",
    "S. Peyton Jones, J. Launchbury, J. Hughes, How to give a great research talk, Microsoft Research, 2016",
    "P. H. Winston, How to Speak, MIT OpenCourseWare RES.TLL-005, 2018",
    "코드·측정 기록: github.com/ahnjun0/robot_hack (브랜치 competition) — docs/발표_기술해설.md, docs/발표_구성안.md",
  ];
  text(s, refs.map((t, k) => ({ text: t, options: { bullet: true, breakLine: k < refs.length - 1 } })), {
    x: 0.5, y: 1.4, w: 9.0, h: 3.6, fontSize: 12, paraSpaceAfter: 10, valign: "top" });
}

pres.writeFile({ fileName: OUT }).then(f => console.log("만들었다:", f));
