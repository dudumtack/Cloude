// my_planner.cpp — 경로 알고리즘 구현부

#include "planner.hpp"

#include <cmath>

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
//   obstacle_count : 장애물 개수 (getObstacleCount가 채움)
//   obstacle_dist  : id번 장애물과 떨어진 거리 (m)
//   obstacle_dir   : id번 장애물 방향 (라디안). 0 = 북쪽(+y), 시계 방향 양수(동쪽 = +π/2).
//                    ※ 규약은 가정일 뿐 — 통합 때 실제 센서 규약에 맞춰 조정.
//   obstacle_width : id번 장애물의 가로 길이 (m)
static int    obstacle_count = 0;
static double obstacle_dist  = 0.0;
static double obstacle_dir   = 0.0;
static double obstacle_width = 0.0;

void getObstacleCount() {
//장애물 개수 받아오는 로직도 나중에 통합할때 할꺼임
obstacle_count = 2;
}

void getObstacle(int id) {
//장애물 정보 받아오는 로직도 나중에 통합할때 할꺼임 (센서/토픽에서 옴)
//id번(0부터) 장애물의 정보를 전역에 채운다
if (id == 0) {
    obstacle_dist  = 5.0;              // 5m 거리
    obstacle_dir   = 0.78539816339;    // π/4 = 북동쪽
    obstacle_width = 2.0;              // 가로 2m
} else if (id == 1) {
    obstacle_dist  = 15.0;             // 15m 거리
    obstacle_dir   = 1.04719755120;    // π/3 = 동북동쪽
    obstacle_width = 1.5;              // 가로 1.5m
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
