// my_planner.cpp — 경로 알고리즘 구현부

#include "planner.hpp"

#include <cmath>

namespace planning {

// 셀의 실제 크기(월드 단위). 기본 1x1 정사각형, 직사각형으로 교체 가능.
struct CellSize {
    double w = 1.0;
    double h = 1.0;
};

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
goal_x = 0.0;
goal_y = 0.0;
}

Path planPath(const Grid& grid, Cell start, Cell goal) {
    getGps();       // current_x, current_y 채움
    getGoalGps();   // goal_x, goal_y 채움

    (void)grid;
    (void)start;
    (void)goal;
    (void)current_x; (void)current_y;
    (void)goal_x;    (void)goal_y;
    return {};
}

} // namespace planning
