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

Path planPath(const Grid& grid, Cell start, Cell goal) {
    (void)grid;
    (void)start;
    (void)goal;
    return {};
}

} // namespace planning
