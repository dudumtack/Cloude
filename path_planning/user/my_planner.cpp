// my_planner.cpp — 경로 알고리즘 구현부

#include "planner.hpp"

#include <cmath>
#include <map>
#include <utility>

namespace planning {

// 셀 크기 값의 단위 (미터 / 센티미터).
enum class Unit { Meter, Centimeter };

// 셀의 실제 크기. w=가로, h=세로, unit=w/h 값의 단위(m 또는 cm).
// 기본 1x1 정사각형·미터 단위. 직사각형/다른 단위로 교체 가능.
struct CellSize {
    double w = 1.0;
    double h = 1.0;
    Unit   unit = Unit::Meter;
};

// 단위 1칸이 몇 미터인지 (m=1.0, cm=0.01).
static double metersPerUnit(Unit u) {
    return (u == Unit::Centimeter) ? 0.01 : 1.0;
}

// 인접한 두 셀 사이 한 스텝의 실제 이동 비용 (직사각형 셀 대응).
static double stepCost(Cell from, Cell to, CellSize cell = CellSize{}) {
    double dx = std::abs(from.x - to.x) * cell.w;
    double dy = std::abs(from.y - to.y) * cell.h;
    return std::sqrt(dx * dx + dy * dy);
}

// g(x): 시작점부터 to 까지 실제 누적 비용 (부모 g + 한 스텝 비용).
static double gCost(double parent_g, Cell from, Cell to, CellSize cell = CellSize{}) {
    return parent_g + stepCost(from, to, cell);
}

// h(x): x 에서 goal 까지 추정 비용. 8방향 옥타일 거리를 직사각형으로 일반화.
static double hCost(Cell x, Cell goal, CellSize cell = CellSize{}) {
    int dx = std::abs(x.x - goal.x);
    int dy = std::abs(x.y - goal.y);
    int dmin = (dx < dy) ? dx : dy;
    double diag = std::sqrt(cell.w * cell.w + cell.h * cell.h);
    double straight = (dx > dy) ? (dx - dy) * cell.w : (dy - dx) * cell.h;
    return dmin * diag + straight;   // 대각선 dmin번 + 남는 축 직선
}

// 장애물 셀 주변에 "가까울수록 큰" 소프트 비용을 깔아준다 (A*가 벽에 바짝 안 붙게).
//   - 장애물 셀 바로 이웃(체비쇼프 거리 1) → +0.5
//   - 그 바깥 한 겹(체비쇼프 거리 2)      → +0.25
//   중복은 그대로 누적 — 여러 장애물이 겹치거나 코너 꼭짓점에서 값이 합쳐짐.
//   반환: (x,y) → 누적 소프트 비용. 장애물 셀 자신(거리 0)은 넣지 않음(이동 불가라 별도 처리).
static std::map<std::pair<int,int>, double>
inflateCost(const std::vector<Cell>& obstacle_cells) {
    std::map<std::pair<int,int>, double> field;
    for (const Cell& o : obstacle_cells) {
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                int adx  = (dx < 0) ? -dx : dx;
                int ady  = (dy < 0) ? -dy : dy;
                int cheb = (adx > ady) ? adx : ady;   // 체비쇼프 거리 (감싸는 겹 번호)
                double add = (cheb == 1) ? 0.5 : (cheb == 2) ? 0.25 : 0.0;
                if (add == 0.0) continue;             // 중심(장애물 셀)만 제외
                field[{o.x + dx, o.y + dy}] += add;   // 누적 (중복 허용)
            }
        }
    }
    return field;
}

// 동적 장애물 하나를 소프트 비용 층으로 변환.
//   - 장애물 중심을 두 겹(체비쇼프 거리 1,2)으로 감싸 각 셀에 +0.7 (정적보다 더 크게 회피).
//   - 진행 방향으로 "속도에 비례하는 거리"만큼 뻗으며, 각 셀에 "속도에 비례하는 비용"을
//     추가 → 물체가 갈 곳을 미리 피하는 예측 위험구역.
//   반환: (x,y) → 누적 소프트 비용. inflateCost 결과와 그대로 합칠 수 있음(중복 누적).
static std::map<std::pair<int,int>, double>
dynamicCost(double dist, double bearing, double heading, double speed,
            CellSize cell = CellSize{}) {
    std::map<std::pair<int,int>, double> field;

    double mpu  = metersPerUnit(cell.unit);
    double cx_m = dist * std::sin(bearing);   // 장애물 중심 (미터, 원점 기준)
    double cy_m = dist * std::cos(bearing);
    int ccx = static_cast<int>(std::lround(cx_m / (cell.w * mpu)));
    int ccy = static_cast<int>(std::lround(cy_m / (cell.h * mpu)));

    // (1) 두 겹 감싸기 → 0.7
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            int adx  = (dx < 0) ? -dx : dx;
            int ady  = (dy < 0) ? -dy : dy;
            int cheb = (adx > ady) ? adx : ady;
            if (cheb == 1 || cheb == 2) field[{ccx + dx, ccy + dy}] += 0.7;
        }
    }

    // (2) 진행 방향 예측 위험구역: 뻗는 거리 ∝ 속도, 셀 비용 ∝ 속도.
    const double TIME_HORIZON = 2.0;   // 초: 몇 초 앞을 내다볼지
    const double SPEED_GAIN   = 0.5;   // (m/s)당 더해줄 비용
    double hx = std::sin(heading);     // 진행 단위벡터
    double hy = std::cos(heading);
    double step_m = ((cell.w < cell.h) ? cell.w : cell.h) * mpu;   // 한 칸(작은 축) 미터
    double reach_m = speed * TIME_HORIZON;                          // 예측 이동 거리 (m)
    int steps = (step_m > 0.0) ? static_cast<int>(std::lround(reach_m / step_m)) : 0;
    for (int i = 1; i <= steps; ++i) {
        double fx_m = cx_m + hx * i * step_m;
        double fy_m = cy_m + hy * i * step_m;
        int fx = static_cast<int>(std::lround(fx_m / (cell.w * mpu)));
        int fy = static_cast<int>(std::lround(fy_m / (cell.h * mpu)));
        field[{fx, fy}] += speed * SPEED_GAIN;                      // 속도 비례 비용
    }
    return field;
}

// 가변 셀: 위치(고정) + 탐색 중 갱신되는 비용/부모.
struct Node {
    Cell   pos;
    double g = 0.0;        // 시작점부터 실제 비용
    double h = 0.0;        // 목표까지 추정 비용
    double f = 0.0;        // g + h
    int    parent = -1;    // 부모 노드 인덱스 (-1 = 시작)
};

// 가변 셀 생성 (f = g + h 자동 계산).
static Node makeNode(Cell pos, double g = 0.0, double h = 0.0, int parent = -1) {
    Node n;
    n.pos    = pos;
    n.g      = g;
    n.h      = h;
    n.f      = g + h;
    n.parent = parent;
    return n;
}

// 현재 위치 GPS (getGps가 채움 — 나중에 로봇개 토픽 값으로 교체)
static double current_x = 0.0;
static double current_y = 0.0;

// 목적지 GPS (getGoalGps가 채움 — 나중에 토픽/입력으로 교체)
static double goal_x = 0.0;
static double goal_y = 0.0;

void getGps() {
//gps 받아오는 로직 구현하지 마셈 나중에 로봇개랑 통합할때 할꺼임
current_x = 36.994720;
current_y = 127.088890;
}

void getGoalGps() {
//목적지 gps 받아오는 로직도 나중에 통합할때 할꺼임
goal_x = 36.994900;   // 현재 위치에서 북쪽으로 약 20m
goal_y = 127.089100;  // 현재 위치에서 동쪽으로 약 19m
}

// 장애물 정보 (getObstacle이 채움 — 나중에 센서/토픽 값으로 교체)
// 장애물이 여러 개일 수 있어 id(0부터 시작)로 하나씩 조회하는 방식.
// 장애물의 "보이는 면"을 양 끝점 A, B 선분으로 표현 → 사선 장애물도 그대로 담김.
//   obstacle_count  : 장애물 개수 (getObstacleCount가 채움)
//   obstacle_a_dist : 끝점 A까지 거리 (m)
//   obstacle_a_dir  : 끝점 A 방향 (라디안). 0 = 북쪽(+y), 시계 방향 양수(동쪽 = +π/2).
//                     ※ 규약은 가정일 뿐 — 통합 때 실제 센서 규약에 맞춰 조정.
//   obstacle_b_dist / obstacle_b_dir : 끝점 B (규약 동일)
static int    obstacle_count  = 0;
static double obstacle_a_dist = 0.0;
static double obstacle_a_dir  = 0.0;
static double obstacle_b_dist = 0.0;
static double obstacle_b_dir  = 0.0;

void getObstacleCount() {
//장애물 개수 받아오는 로직도 나중에 통합할때 할꺼임
obstacle_count = 2;
}

void getObstacle(int id) {
//장애물 정보 받아오는 로직도 나중에 통합할때 할꺼임 (센서/토픽에서 옴)
//id번(0부터) 장애물의 양 끝점을 전역에 채운다
if (id == 0) {
    // 정면 폭 ~2m 장애물 (시선에 수직에 가까움)
    obstacle_a_dist = 5.1;   obstacle_a_dir = 0.98;   // 끝점 A
    obstacle_b_dist = 5.1;   obstacle_b_dir = 0.59;   // 끝점 B
} else if (id == 1) {
    // 사선으로 놓인 ~2.5m 벽
    obstacle_a_dist = 14.0;  obstacle_a_dir = 1.10;   // 끝점 A
    obstacle_b_dist = 16.0;  obstacle_b_dir = 1.00;   // 끝점 B
}
}

// 동적(움직이는) 장애물 정보 (getDynObstacle이 채움 — 나중에 추적기/센서 값으로 교체)
// 위치 + 진행 방향 + 속도로 표현. id(0부터)로 하나씩 조회.
//   dyn_count   : 동적 장애물 개수 (getDynObstacleCount가 채움)
//   dyn_dist    : 장애물까지 거리 (m)
//   dyn_bearing : 장애물이 "있는" 방위 (라디안, 0=북/시계+) — 거리와 함께 위치를 정함
//   dyn_heading : 장애물이 "가는" 진행 방향 (라디안, 규약 동일) — dyn_bearing과 별개
//   dyn_speed   : 장애물 속도 (m/s)
static int    dyn_count   = 0;
static double dyn_dist    = 0.0;
static double dyn_bearing = 0.0;
static double dyn_heading = 0.0;
static double dyn_speed   = 0.0;

void getDynObstacleCount() {
//동적 장애물 개수 받아오는 로직도 나중에 통합할때 할꺼임
dyn_count = 1;
}

void getDynObstacle(int id) {
//동적 장애물 정보 받아오는 로직도 나중에 통합할때 할꺼임 (추적기/센서에서 옴)
if (id == 0) {
    dyn_dist    = 8.0;             // 8m 거리
    dyn_bearing = 0.52;           // ~북북동에 위치 (있는 곳)
    dyn_heading = 1.57079632679;  // π/2 = 동쪽으로 진행 (가는 곳)
    dyn_speed   = 1.5;            // 1.5 m/s
}
}

// GPS 좌표(위도 lat, 경도 lon)를 원점(origin) 기준 셀 인덱스로 변환.
//   위도 1도 ≈ 111320 m, 경도 1도 ≈ 111320*cos(위도) m.
//   원점 대비 미터 오프셋을 "셀 크기(미터 환산)"로 나눠 인덱스를 만든다.
static Cell gpsToCell(double lat, double lon,
                      double origin_lat, double origin_lon,
                      CellSize cell) {
    const double M_PER_DEG = 111320.0;
    const double DEG2RAD   = 0.017453292519943295;
    double dy_m = (lat - origin_lat) * M_PER_DEG;                              // 남북(미터)
    double dx_m = (lon - origin_lon) * M_PER_DEG * std::cos(origin_lat * DEG2RAD); // 동서(미터)
    double mpu   = metersPerUnit(cell.unit);   // 셀 단위 1칸 = 몇 미터
    int cx = static_cast<int>(std::lround(dx_m / (cell.w * mpu)));
    int cy = static_cast<int>(std::lround(dy_m / (cell.h * mpu)));
    return Cell{cx, cy};
}

// 현재 GPS로 현재 셀 생성 (현재 위치를 원점으로 → (0,0)).
static Cell makeCurrentCell(CellSize cell = CellSize{}) {
    return gpsToCell(current_x, current_y, current_x, current_y, cell);
}

// 목적지 GPS로 목적지 셀 생성 (현재 위치를 원점으로).
static Cell makeGoalCell(CellSize cell = CellSize{}) {
    return gpsToCell(goal_x, goal_y, current_x, current_y, cell);
}

// 현재/목적지 GPS를 토대로 시작·목표 셀을 만든다.
static void buildCells(Cell& start_cell, Cell& goal_cell, CellSize cell = CellSize{}) {
    start_cell = makeCurrentCell(cell);
    goal_cell  = makeGoalCell(cell);
}

// 장애물 하나(양 끝점 A, B — 각각 거리 m + 방향 rad)를 원점 기준 셀 목록으로 변환.
//   방향 규약: 0 = 북쪽(+y), 시계 방향 양수 → 동쪽 성분 = sin(dir), 북쪽 성분 = cos(dir).
//   A~B 선분을 셀 반 칸 간격으로 샘플링해 걸치는 셀을 전부 모은다(빈틈 방지).
static std::vector<Cell> obstacleToCells(double a_dist, double a_dir,
                                         double b_dist, double b_dir,
                                         CellSize cell = CellSize{}) {
    double ax = a_dist * std::sin(a_dir);   // 끝점 A (미터, 원점 기준)
    double ay = a_dist * std::cos(a_dir);
    double bx = b_dist * std::sin(b_dir);   // 끝점 B
    double by = b_dist * std::cos(b_dir);

    double len  = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));  // 선분 길이 (m)
    double mpu  = metersPerUnit(cell.unit);
    double step = 0.5 * ((cell.w < cell.h) ? cell.w : cell.h) * mpu;  // 셀 반 칸 (미터)

    int n = (len > 0.0) ? static_cast<int>(std::ceil(len / step)) : 0;  // 샘플 구간 수

    std::vector<Cell> out;
    for (int i = 0; i <= n; ++i) {
        double t = (n > 0) ? static_cast<double>(i) / n : 0.0;  // 0(A) ~ 1(B)
        double sx_m = ax + (bx - ax) * t;
        double sy_m = ay + (by - ay) * t;
        Cell c{ static_cast<int>(std::lround(sx_m / (cell.w * mpu))),
                static_cast<int>(std::lround(sy_m / (cell.h * mpu))) };
        if (out.empty() || out.back() != c) out.push_back(c);  // 연속 중복 제거
    }
    return out;
}

// ───────────────────── 주행(이동) 함수 ─────────────────────
// rviz2 클릭 목표로 "바라보고 → 전진 → 도착"하는 실제 로봇 제어(test_goto_point.cpp)의
// 이동 메커니즘을 ROS 없이 순수 계산 함수로 옮긴 것. A*가 만든 경로의 각 웨이포인트를
// 이 함수들로 하나씩 추종하면 로봇이 경로를 따라간다.
//   프레임: 로봇 odom 좌표(x,y=m), yaw=rad.
//   (planPath의 셀 좌표와는 "셀 → 월드(m)" 변환 + yaw/북 규약 맞추기로 연결 예정.)

// 로봇에 내릴 한 틱 분량의 이동 명령. sport_client_.Move(req_, vx, vy, vyaw)에 그대로 대응.
struct MoveCmd {
    double vx      = 0.0;    // 전진 속도 (m/s)
    double vy      = 0.0;    // 좌우 속도 (m/s) — 여기선 0
    double vyaw    = 0.0;    // 회전 속도 (rad/s), 양수 = 왼쪽
    bool   arrived = false;  // 목표 도달 시 true (이때 vx=vy=vyaw=0)
};

// 현재 위치 A(ax,ay)에서 방향 theta_current를 볼 때, 목표 B(bx,by)를 바라보려면
// 몇 rad 돌아야 하는지 (-pi, pi] 로 반환. 양수 = 왼쪽, 음수 = 오른쪽이 최단.
static double computeYawDelta(double ax, double ay, double theta_current,
                             double bx, double by) {
    double theta_target = std::atan2(by - ay, bx - ax);    // A→B 방향각 (world)
    double delta = theta_target - theta_current;            // 현재 방향과의 차
    return std::atan2(std::sin(delta), std::cos(delta));    // (-pi,pi] 정규화 → 최단 회전
}

// 로봇 현재 자세(rx,ry)에서 목표점(tx,ty)까지 남은 수평 거리 (m).
static double distanceTo(double rx, double ry, double tx, double ty) {
    return std::hypot(tx - rx, ty - ry);
}

// 목표점 하나를 향한 한 틱 이동 명령을 만든다.
// test_goto_point.cpp의 phase 1·2 로직을 상태 없이 매 틱 재판단하는 형태로 정리:
//   - 도착 반경 안        → 정지(arrived=true)
//   - 목표를 아직 안 봄   → 제자리 회전만 (전진 없음)
//   - 목표를 보고 있음    → 전진 + 방향 미세 보정
static MoveCmd stepToward(double rx, double ry, double yaw, double tx, double ty) {
    const double ARRIVE_R = 0.2;   // 도착 판정 반경 (m)
    const double FACE_TOL = 0.1;   // 목표를 "보고 있다" 판정 (rad, 약 6°)
    const double TURN_SPD = 0.7;   // 제자리 회전 속도 (rad/s)
    const double FWD_SPD  = 0.5;   // 전진 속도 (m/s)

    MoveCmd cmd;
    double dist      = distanceTo(rx, ry, tx, ty);
    double yaw_delta = computeYawDelta(rx, ry, yaw, tx, ty);

    if (dist < ARRIVE_R) {                          // 도착
        cmd.arrived = true;
    } else if (std::fabs(yaw_delta) > FACE_TOL) {   // 아직 목표를 안 봄 → 회전만
        cmd.vyaw = (yaw_delta > 0) ? TURN_SPD : -TURN_SPD;
    } else {                                        // 목표를 봄 → 전진 + 보정
        cmd.vx   = FWD_SPD;
        cmd.vyaw = 0.5 * yaw_delta;
    }
    return cmd;
}

Path planPath(const Grid& grid, Cell start, Cell goal) {
    getGps();       // current_x, current_y 채움
    getGoalGps();   // goal_x, goal_y 채움

    Cell current_cell, goal_cell;
    buildCells(current_cell, goal_cell);   // GPS → 셀

    (void)grid;
    (void)start;
    (void)goal;
    (void)current_cell; (void)goal_cell;
    return {};
}

} // namespace planning
