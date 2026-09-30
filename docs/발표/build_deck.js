// 발표 초안 생성기 - 인지(Perception) -> 계획(Planning) -> 행동(Action) 순서.
// 구성 근거: docs/발표_구성안.md (Stanley JFR 2006 순서, Peyton Jones·Winston 발표법).
//
// 다시 만들기:
//   npm install pptxgenjs            (한 번)
//   node docs/발표/build_deck.js docs/발표/발표_초안.pptx
// 글꼴은 시연 Mac 기준 "Apple SD Gothic Neo". Windows 용: DECK_FONT="Malgun Gothic" node ...
//
// 문구 규칙: 화면에는 꼭 필요한 말만 (설명은 발표 노트로). 대시는 하이픈(-)만. 제목은 한 줄.
// 줄을 나눌 때는 어절 단위로, 한 단어만 넘기지 않는다. 코드·파일 이름은 쓰지 않는다.
// 저장할 때 모든 문단에 eaLnBrk="0" (한국어 단어를 중간에서 끊지 않는다)을 넣는다.
let pptxgen, JSZip;
try { pptxgen = require("pptxgenjs"); JSZip = require("jszip"); } catch (e) {
  pptxgen = require("/tmp/deck/node_modules/pptxgenjs"); JSZip = require("/tmp/deck/node_modules/jszip");
}
const fs = require("fs");

const OUT = process.argv[2] || "docs/발표/발표_초안.pptx";
const pres = new pptxgen();
pres.layout = "LAYOUT_16x9"; // 10 x 5.625 in
pres.title = "특명: 집 나간 빨간 사과 찾아오기";

const FONT = process.env.DECK_FONT || "Apple SD Gothic Neo";
const C = {
  dark: "1B2631", ink: "1B2631", sub: "5D6D7E", light: "F2F4F4", white: "FFFFFF",
  red: "C0392B", teal: "117A65", amber: "B9770E", purple: "6C3483", line: "D5D8DC", pale: "CAD3DA",
};
const STAGE = {
  "인지": { color: C.teal, en: "Perception" },
  "계획": { color: C.amber, en: "Planning" },
  "행동": { color: C.purple, en: "Action" },
};

function text(slide, str, opts) {
  slide.addText(str, Object.assign({ fontFace: FONT, color: C.ink, isTextBox: true, margin: 0 }, opts));
}

function bullets(slide, items, opts) {
  text(slide, items.map((t, k) => ({ text: t, options: { bullet: true, breakLine: k < items.length - 1 } })),
    Object.assign({ paraSpaceAfter: 8, valign: "top" }, opts));
}

function header(slide, title, stage, label) {
  if (stage) {
    const s = STAGE[stage];
    slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x: 0.5, y: 0.32, w: 1.2, h: 0.32, rectRadius: 0.16, fill: { color: s.color }, line: { color: s.color },
    });
    text(slide, label || stage, {
      x: 0.5, y: 0.32, w: 1.2, h: 0.32, fontSize: 11, bold: true, color: C.white, align: "center", valign: "middle",
    });
  }
  text(slide, title, { x: 0.5, y: stage ? 0.74 : 0.42, w: 9.0, h: 0.5, fontSize: 22, bold: true, valign: "top" });
}

// 수식은 LaTeX 로 렌더링한 그림 (make_eq.py), GIF 는 주행 기록으로 만든 그림 (make_gifs.py).
const ASSET = process.env.DECK_ASSETS || "docs/발표/assets";
const EQS = JSON.parse(fs.readFileSync(ASSET + "/eq_sizes.json", "utf-8"));
const GIFS = JSON.parse(fs.readFileSync(ASSET + "/gif_sizes.json", "utf-8"));

// scale 1 = 렌더링 때 20pt. maxW 를 넘으면 줄인다.
function eq(slide, name, x, y, scale, maxW) {
  const [pw, ph] = EQS[name];
  let w = pw / 300 * scale, h = ph / 300 * scale;
  if (maxW && w > maxW) { h *= maxW / w; w = maxW; }
  slide.addImage({ path: `${ASSET}/eq/${name}.png`, x, y, w, h });
  return y + h;
}

// 상자 안에 비율을 지켜 가운데 맞춘다. 그림이 없으면 점선 자리 표시.
function gif(slide, x, y, w, h, label, name) {
  if (name && GIFS[name]) {
    const [pw, ph] = GIFS[name];
    const k = Math.min(w / pw, h / ph);
    const gw = pw * k, gh = ph * k;
    slide.addImage({ path: `${ASSET}/gif/${name}.gif`, x: x + (w - gw) / 2, y: y + (h - gh) / 2, w: gw, h: gh });
    return;
  }
  slide.addShape(pres.shapes.RECTANGLE, {
    x, y, w, h, fill: { color: C.light }, line: { color: "95A5A6", width: 1.25, dashType: "dash" },
  });
  text(slide, label, { x: x + 0.2, y, w: w - 0.4, h, fontSize: 12, color: C.sub, align: "center", valign: "middle" });
}

function table(slide, rows, opts) {
  slide.addTable(rows.map((r, i) => r.map(c => ({ text: c, options: i === 0
    ? { bold: true, color: C.white, fill: { color: C.dark } }
    : { fill: { color: i % 2 ? C.white : C.light } } }))),
    Object.assign({ fontFace: FONT, border: { type: "solid", pt: 0.5, color: C.line }, margin: [0.07, 0.1, 0.07, 0.1] }, opts));
}

// 결정 표: 문제(숫자) -> 기준(출처) -> 결과 (Stanley 5.2 의 틀)
function decision(slide, rows, x, y, w) {
  const labels = ["문제", "버린 대안", "기준", "결과"];
  const data = rows.map((r, i) => [
    { text: labels[i], options: { bold: true, color: C.white, fill: { color: C.dark }, fontSize: 12, valign: "middle", align: "center" } },
    { text: r, options: { color: C.ink, fontSize: 12, valign: "middle", fill: { color: i % 2 ? C.white : C.light } } },
  ]);
  slide.addTable(data, {
    x, y, w, colW: [1.1, w - 1.1], fontFace: FONT, border: { type: "solid", pt: 0.5, color: C.line },
    margin: [0.09, 0.12, 0.09, 0.12],
  });
}

function bigStat(slide, x, y, w, num, label, color) {
  text(slide, num, { x, y, w, h: 0.75, fontSize: 40, bold: true, color: color || C.red });
  text(slide, label, { x, y: y + 0.78, w, h: 0.3, fontSize: 12, color: C.sub, valign: "top" });
}

// ───────── 1. 표지 ─────────
{
  const s = pres.addSlide();
  s.background = { color: C.dark };
  s.addShape(pres.shapes.OVAL, { x: 7.6, y: 1.35, w: 1.5, h: 1.5, fill: { color: C.red }, line: { color: C.red } });
  text(s, "특명:\n집 나간 빨간 사과 찾아오기", { x: 0.6, y: 1.2, w: 6.8, h: 1.6, fontSize: 34, bold: true, color: C.white, valign: "top" });
  text(s, "Webots apartment  ·  TurtleBot3 Burger", { x: 0.6, y: 3.0, w: 8.8, h: 0.35, fontSize: 14, color: C.pale });
  text(s, "로봇이아닙니다  ·  안준영  송지윤  배준호", { x: 0.6, y: 4.5, w: 8.5, h: 0.35, fontSize: 13, color: C.pale });
  s.addNotes("시작 15초. \"저희 로봇은 GPS도, 지도도 없이 처음 보는 집에서 빨간 사과 두 개를 찾아 방문하고 시작점으로 돌아옵니다.\"\n" +
    "목차는 두지 않는다 (Peyton Jones).");
}

// ───────── 2. 과제와 시연 ─────────
{
  const s = pres.addSlide();
  header(s, "과제: 모르는 집에서 빨간 사과 2개를 찾고 돌아오기");
  bullets(s, ["지도 없음", "GNSS·절대 위치 금지", "속도 한계 준수", "로봇을 피하지 않는 보행자"],
    { x: 0.5, y: 1.35, w: 4.5, h: 1.8, fontSize: 16 });
  bigStat(s, 0.5, 3.3, 4.5, "0 / 2", "첫 완주: 소화기를 사과로 착각");
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 5.4, y: 1.35, w: 4.1, h: 3.0, rectRadius: 0.12, fill: { color: C.dark }, line: { color: C.dark } });
  text(s, "시연 ▶", { x: 5.7, y: 1.55, w: 3.6, h: 0.5, fontSize: 22, bold: true, color: C.white });
  text(s, [
    { text: "① 첫 사과 - 약 3분", options: { breakLine: true } },
    { text: "② 보행자 - 약 6분", options: { breakLine: true } },
    { text: "③ 복귀 - 약 8분 반" },
  ], { x: 5.7, y: 2.25, w: 3.6, h: 1.8, fontSize: 18, color: C.white, paraSpaceAfter: 10, valign: "top" });
  text(s, "모든 문턱값은 측정에서 왔고, 켜서 나빠진 기능은 뺐다.", {
    x: 0.5, y: 4.75, w: 9.0, h: 0.4, fontSize: 15, bold: true, color: C.teal });
  s.addNotes("이 장에서 Webots 재생 (1.0x). 이후 로봇이 도는 동안 슬라이드를 넘기고, 장면 시각에 Webots 로 넘어간다 (대본의 시각표).\n" +
    "보행자는 0.2 m/s 로 로봇 최고 속도와 같고, 로봇을 피하지 않는다. 첫 완주는 소화기를 사과로 확정해 0/2.\n" +
    "핵심 문장: 매 틱 녹화해 두고, 같은 주행을 재생하며 방법만 바꿔 비교했다 (Stanley 3.1.4 의 replay).\n" +
    "라이브가 멈추면 같은 설정의 녹화로 바꾼다 (시뮬레이션이 결정적이라 같은 장면).");
}

// ───────── 3. 구조와 센서 ─────────
{
  const s = pres.addSlide();
  header(s, "구조: 인지 → 계획 → 행동");
  const cols = [
    ["인지", ["위치: 엔코더 + 나침반", "지도: LiDAR 360°", "사과: 카메라 60°", "사람: LiDAR + 칼만"]],
    ["계획", ["벽 부풀리기", "A* 경로", "0.3초마다 재계획"]],
    ["행동", ["프론티어 탐험", "DWA 주행", "상태 기계 · 복구"]],
  ];
  cols.forEach(([name, items], i) => {
    const x = 0.5 + i * 3.1, col = STAGE[name].color;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 1.3, w: 2.8, h: 0.7, rectRadius: 0.1, fill: { color: col }, line: { color: col } });
    text(s, name, { x, y: 1.3, w: 2.8, h: 0.7, fontSize: 20, bold: true, color: C.white, align: "center", valign: "middle" });
    if (i < 2) s.addShape(pres.shapes.RIGHT_ARROW, { x: x + 2.83, y: 1.5, w: 0.24, h: 0.3, fill: { color: C.sub }, line: { color: C.sub } });
    bullets(s, items, { x: x + 0.1, y: 2.2, w: 2.7, h: 1.7, fontSize: 14 });
  });
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.5, y: 4.05, w: 9.0, h: 0.75, rectRadius: 0.1, fill: { color: C.light }, line: { color: C.light } });
  text(s, [
    { text: "LiDAR 는 17 cm 높이의 평면만 본다  →  ", options: {} },
    { text: "카펫 2 cm, 사과 10 cm 는 안 보인다", options: { bold: true, color: C.red } },
  ], { x: 0.75, y: 4.05, w: 8.5, h: 0.75, fontSize: 15, valign: "middle" });
  s.addNotes("인지가 '나는 어디 있고 주변에 뭐가 있나', 계획이 '어디로 갈까', 행동이 '지금 바퀴를 어떻게 돌릴까'.\n" +
    "주기: 엔코더·나침반·LiDAR 64 ms, 카메라 128 ms. GPS 는 달지 않는다.\n" +
    "'17 cm 평면' 이 카펫(미끄러짐)과 사과(카메라로 거리 재기) 문제의 복선이다.");
}

// ───────── 4. 인지: 위치 추정 ─────────
{
  const s = pres.addSlide();
  header(s, "위치 추정 - 미끄러짐을 알아채고, 지도에 맞춰 되돌린다", "인지");
  text(s, "오도메트리 · 미끄러짐 감지", { x: 0.5, y: 1.3, w: 6.0, h: 0.3, fontSize: 13, bold: true, color: C.teal });
  eq(s, "odom", 0.5, 1.65, 0.6, 6.0);
  eq(s, "slip", 0.5, 2.2, 0.6, 6.0);
  text(s, "스캔 매칭", { x: 0.5, y: 2.85, w: 6.0, h: 0.3, fontSize: 13, bold: true, color: C.teal });
  eq(s, "scan", 0.5, 3.2, 0.6, 6.0);
  bigStat(s, 0.5, 3.75, 6.0, "304 → 9.3 cm", "최대 위치 오차 (스캔 매칭 끔 → 켬, 실제 주행)", C.teal);
  gif(s, 6.7, 1.3, 2.8, 3.6, "카펫 미끄러짐", "gif2_carpet");
  s.addNotes("방향은 나침반에서 읽어 누적 오차가 없다. 거리는 바퀴 → 카펫 턱에서 헛돌면 20-67 cm 씩 튄다.\n" +
    "미끄러짐: 나침반 회전/바퀴 회전 비가 평소 0.87-1.14, 카펫 0-0.3. 0.6 rad/s 넘게 0.3초 어긋나면 1초 후진, 그 자리를 피한다.\n" +
    "스캔 매칭: 지도의 거리장 D 에 스캔 점을 맞추는 가우스-뉴턴, x·y 만. 근거가 없는 방향(긴 복도)은 고치지 않는다.\n" +
    "처음의 격자 탐색 방식은 오히려 164 cm 까지 키워서 바꿨다. 녹화 재생에서는 186 → 21 cm.\n" +
    "바퀴 반지름도 이 월드에서 잰 거리에 맞춰 0.033 → 0.0336 m.");
}

// ───────── 5. 인지: 사과와 보행자 ─────────
{
  const s = pres.addSlide();
  header(s, "사과는 두 번 재서 거르고, 보행자는 예측해 비킨다", "인지");
  text(s, "사과 - 카메라 한 장으로 거리 두 번", { x: 0.5, y: 1.3, w: 4.3, h: 0.3, fontSize: 13, bold: true, color: C.teal });
  eq(s, "range", 0.5, 1.7, 0.6, 4.3);
  table(s, [
    ["가짜", "걸러낸 기준"],
    ["가까운 소화기", "두 거리 차 ≤ 30%"],
    ["먼 소화기", "세로/가로 ≤ 1.5"],
    ["누운 캔", "세로/가로 ≥ 0.67"],
    ["병 · 컵", "YOLO 거부"],
  ], { x: 0.5, y: 2.45, w: 4.3, colW: [1.7, 2.6], fontSize: 12 });
  text(s, "보행자 - 칼만 필터로 1.2초 앞 자리", { x: 5.2, y: 1.3, w: 4.3, h: 0.3, fontSize: 13, bold: true, color: C.teal });
  eq(s, "kf", 5.2, 1.7, 0.6, 4.3);
  eq(s, "kf2", 5.2, 2.05, 0.6, 4.3);
  bigStat(s, 5.2, 2.65, 4.3, "3.8 → 0초", "보행자 접촉 (두 스캔 차이 → 칼만)", C.teal);
  text(s, "측정 잡음 0.23 m (실측)  ·  보행자 속도 = 로봇 최고 속도", {
    x: 5.2, y: 3.85, w: 4.3, h: 0.6, fontSize: 12, color: C.sub, valign: "top" });
  s.addNotes("사과는 LiDAR(17 cm)보다 낮아 카메라로만 잰다. 크기로 d = R/sin γ, 바닥 접점으로 d = h/tan δ (R=0.05, h=0.073 m). 진짜 바닥의 공이면 두 값이 맞아야 한다.\n" +
    "기준의 출처: 사과 1-7% vs 가까운 소화기 81% 어긋남 / 먼 소화기는 29% 로 통과해 세로/가로 3.1 로 걸렀다 / 캔 0.42.\n" +
    "YOLO 는 거부권으로만 - 멀리 있는 사과는 자주 놓쳐서 문지기로 쓰면 사과를 잃는다. 25번 보면 확정 (최소 3.2초).\n" +
    "보행자: LiDAR 로 '지도에 없는데 움직이는 것'. 두 스캔 차이로 속도를 내면 잡음(23 cm)이 속도보다 컸다 → 등속 칼만.\n" +
    "사람이 0.4·0.8·1.2초 뒤 있을 자리를 DWA 가 장애물로 보고, 전역 경로는 사람 둘레 1.2 m 를 비싸게 매긴다.");
}

// ───────── 6. 계획·행동: 지도 → 경로 → 바퀴 ─────────
{
  const s = pres.addSlide();
  header(s, "지도 → 경로 → 바퀴 명령", "계획", "계획 · 행동");
  const rows = [
    ["지도", "logodds", "벽은 1번에 생기고, 지우려면 14번 통과"],
    ["벽 부풀리기", "inflate", "좁은 문에서만 0.28 m 로 한 번 더"],
    ["A* · 탐험", "astar", "경로가 가장 짧은 프론티어로 - 연습 월드 46-48 / 48"],
    ["DWA", "dwa", "119개 후보를 1.2초 굴려 보고, 0.28 m 안이면 버린다"],
  ];
  rows.forEach(([k, e, cap], i) => {
    const y = 1.3 + i * 0.9;
    const col = i < 3 ? STAGE["계획"].color : STAGE["행동"].color;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.5, y: y + 0.05, w: 1.5, h: 0.7, rectRadius: 0.08, fill: { color: col }, line: { color: col } });
    text(s, k, { x: 0.5, y: y + 0.05, w: 1.5, h: 0.7, fontSize: 13, bold: true, color: C.white, align: "center", valign: "middle" });
    eq(s, e, 2.2, y + 0.02, 0.55, 7.3);
    text(s, cap, { x: 2.2, y: y + 0.5, w: 7.3, h: 0.28, fontSize: 11, color: C.sub });
  });
  s.addNotes("지도: log-odds 로 곱셈이 덧셈. 벽 +0.9, 통과 -0.4, 범위 [-3, 5]. 벽을 쉽게 잃지 않는 비대칭 - 대신 보행자가 15 cm 가짜 벽을 남긴다.\n" +
    "부풀리기: 몸 0.13 + 안전거리 0.22 = 0.35 m. 안전거리는 정지거리 0.23 m 와 맞물린다.\n" +
    "A*: 옥타일 휴리스틱은 실제 비용을 넘지 않아 최단 경로 보장. 탐험은 프론티어마다 설 자리를 정하고 A* 로 갈 수 있는지 확인한 뒤 가장 짧은 곳. 점수를 섞으면 월드마다 들쭉날쭉해서 규칙 하나로.\n" +
    "DWA: 가속 한계 안의 v 7개 × ω 17개, 목표에 가깝고 장애물에서 먼 것. 바퀴가 모터 한계를 넘으면 두 바퀴를 같은 비율로 줄여 회전 반경을 지킨다.");
}

// ───────── 7. 행동: 상태 기계와 결과 ─────────
{
  const s = pres.addSlide();
  header(s, "상태 기계와 복구, 그리고 결과", "행동");
  ["스캔", "탐색", "접근", "복귀", "끝"].forEach((st, i) => {
    const x = 0.5 + i * 1.85;
    s.addShape(pres.shapes.OVAL, { x, y: 1.3, w: 1.5, h: 0.6, fill: { color: i === 3 ? C.red : C.dark }, line: { color: C.dark } });
    text(s, st, { x, y: 1.3, w: 1.5, h: 0.6, fontSize: 14, bold: true, color: C.white, align: "center", valign: "middle" });
  });
  bullets(s, ["길이 막히면 지나온 길 되짚기", "눌리면 트인 쪽으로 (광선 2개 이상)", "사과가 없어도 반드시 복귀"],
    { x: 0.5, y: 2.15, w: 4.6, h: 1.3, fontSize: 13, paraSpaceAfter: 4 });
  eq(s, "budget", 5.3, 2.25, 0.6, 4.2);
  s.addShape(pres.shapes.LINE, { x: 0.5, y: 3.55, w: 9.0, h: 0, line: { color: C.line, width: 1 } });
  bigStat(s, 0.5, 3.7, 3.0, "2 / 2", "빨간 사과", C.red);
  bigStat(s, 3.5, 3.7, 3.0, "0초", "보행자 접촉", C.red);
  bigStat(s, 6.5, 3.7, 3.0, "0.34 m", "시작점까지 실제 거리", C.red);
  s.addNotes("상태가 적고 얽히지 않아 상태 기계를 골랐다.\n" +
    "되짚기: 가짜 벽에 갇혀 200초를 헤맨 판이 있어 넣었다. 실제로 지나온 길이니 갈 수 있다는 근거가 있다.\n" +
    "눌림 탈출: 0.2 m 안 광선 1개일 때 실제 거리 중앙값 339 cm(잡음), 3개 이상일 때 33-36 cm → 2개 이상.\n" +
    "시간 예산: 제한 시간에서 1.5×(집까지 A* 길이 / 평균 속도)를 남기면 무엇을 하던 중이든 복귀.\n" +
    "결과: 482.8초, 최대 위치 오차 19.6 cm, 과일·캔 0/8 밀림. 진행은 0/2 → 1/2 → 2/2 (복귀 1.99 m 모자람) → 스캔 매칭·미끄러짐 감지로 0.34 m.");
}

// ───────── 8. 시연 장면 (GIF 모음) ─────────
{
  const s = pres.addSlide();
  header(s, "시연 장면 - 주행 기록으로 다시 그림");
  const H = 1.8;
  const rows = [
    [["gif1_full", "전체 주행"], ["gif3_apple_camera", "① 사과 확정 (카메라)"], ["gif4_pedestrian", "② 보행자와 엇갈림"]],
    [["gif2_carpet", "카펫 미끄러짐"], ["gif5_retrace", "③ 지나온 길 되짚기"]],
  ];
  rows.forEach((row, r) => {
    const ws = row.map(([n]) => GIFS[n][0] / GIFS[n][1] * H);
    const gap = 0.35, total = ws.reduce((a, b) => a + b, 0) + gap * (row.length - 1);
    let x = 5.0 - total / 2;
    const y = 1.05 + r * 2.12;
    row.forEach(([n, cap], i) => {
      gif(s, x, y, ws[i], H, cap, n);
      text(s, cap, { x: x - 0.2, y: y + H + 0.03, w: ws[i] + 0.4, h: 0.25, fontSize: 11, color: C.sub, align: "center" });
      x += ws[i] + gap;
    });
  });
  s.addNotes("라이브 시연의 예비 장. 라이브가 멈추거나 느리면 이 장으로 장면을 보여 준다.\n" +
    "모두 확정 설정의 사과 2/2 주행 기록(482.8초)에서 다시 그렸다 - 카펫 미끄러짐만 그 전 주행(302-308초).\n" +
    "위에서 본 지도: 회색은 로봇이 그린 지도, 초록은 실제 경로, 점선은 로봇이 추정한 경로.");
}

// ───────── 9. 한계와 기여 ─────────
{
  const s = pres.addSlide();
  header(s, "한계와 기여");
  text(s, "한계", { x: 0.5, y: 1.25, w: 4.3, h: 0.4, fontSize: 16, bold: true, color: C.red });
  [
    ["카펫", "걸린 뒤에야 알아채고 빠져나온다"],
    ["좌우 맴돌기", "안전한데도 방향을 번갈아 고른다"],
    ["설정 민감도", "옵션 하나에 사과 1/2 와 2/2"],
  ].forEach(([k, v], i) => {
    const y = 1.8 + i * 0.95;
    text(s, k, { x: 0.5, y, w: 4.3, h: 0.35, fontSize: 15, bold: true });
    text(s, v, { x: 0.5, y: y + 0.36, w: 4.3, h: 0.35, fontSize: 13, color: C.sub });
  });
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 5.1, y: 1.2, w: 4.4, h: 3.8, rectRadius: 0.12, fill: { color: C.dark }, line: { color: C.dark } });
  text(s, "기여", { x: 5.4, y: 1.35, w: 3.9, h: 0.4, fontSize: 16, bold: true, color: C.white });
  [
    "나침반·바퀴 비교로 미끄러짐 감지",
    "거리를 두 번 재서 가짜 사과 걸러내기",
    "속도가 같은 보행자와 접촉 0초",
    "모든 문턱값에 측정 근거",
  ].forEach((t, i) => {
    const y = 1.9 + i * 0.75;
    s.addShape(pres.shapes.OVAL, { x: 5.4, y: y + 0.05, w: 0.4, h: 0.4, fill: { color: C.red }, line: { color: C.red } });
    text(s, String(i + 1), { x: 5.4, y: y + 0.05, w: 0.4, h: 0.4, fontSize: 13, bold: true, color: C.white, align: "center", valign: "middle" });
    text(s, t, { x: 5.95, y, w: 3.4, h: 0.5, fontSize: 13, color: C.white, valign: "middle" });
  });
  s.addNotes("한계는 우리가 먼저 말한다 (Peyton Jones).\n" +
    "좌우 맴돌기: 버벅인 19구간 중 15구간은 전진 후보가 100% 안전 - 여유가 아니라 방향 선택 문제. 방향 유지 가산점을 시험 중.\n" +
    "설정 민감도: 미끄러진 자리 표시가 서쪽 통로를 막으면 1/2, 포기 전에 표시를 풀면 2/2. 월드 하나, 설정마다 한 번 측정.\n" +
    "이 장을 띄운 채 질문을 받는다 (Winston: 마지막 장은 기여).\n" +
    "참고: 부산대 TECH WEEK Physical AI 교육 자료(2026) / Thrun et al., Stanley, J. Field Robotics 2006 / Peyton Jones et al., How to give a great research talk, 2016 / Winston, How to Speak, MIT OCW 2018.");
}

// ───────── 저장: 한국어 단어를 중간에서 끊지 않게 ─────────
function noWordSplit(xml) {
  xml = xml.replace(/<a:pPr(?![^>]*eaLnBrk)/g, '<a:pPr eaLnBrk="0"');
  return xml.replace(/<a:p>(?!<a:pPr)/g, '<a:p><a:pPr eaLnBrk="0"/>');
}

pres.write({ outputType: "nodebuffer" })
  .then(buf => JSZip.loadAsync(buf))
  .then(async zip => {
    for (const name of Object.keys(zip.files)) {
      if (/^ppt\/slides\/slide\d+\.xml$/.test(name)) {
        zip.file(name, noWordSplit(await zip.file(name).async("string")));
      }
    }
    return zip.generateAsync({ type: "nodebuffer", compression: "DEFLATE" });
  })
  .then(buf => { fs.writeFileSync(OUT, buf); console.log("만들었다:", OUT); });
