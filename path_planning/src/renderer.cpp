// =====================================================================
//  renderer.cpp — 격자/경로 ASCII 시각화
// =====================================================================

#include "renderer.hpp"

#include <vector>

namespace planning {

void render(std::ostream& os,
            const Grid& grid,
            const Cell& start,
            const Cell& goal,
            const Path& path) {
    const int W = grid.width();
    const int H = grid.height();

    // 화면 버퍼를 만들고 먼저 장애물/빈칸을 채운다.
    std::vector<std::string> canvas(H, std::string(W, '.'));
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (!grid.isFree({x, y})) canvas[y][x] = '#';
        }
    }

    // 경로가 지나간 칸을 '*'로 표시(시작/목표는 아래에서 덮어씀).
    for (const Cell& c : path) {
        if (grid.inBounds(c)) canvas[c.y][c.x] = '*';
    }

    // 시작/목표를 마지막에 찍어 항상 보이게 한다.
    if (grid.inBounds(start)) canvas[start.y][start.x] = 'S';
    if (grid.inBounds(goal))  canvas[goal.y][goal.x]   = 'G';

    // 상단 좌표 눈금(선택): 열 번호 10단위.
    for (const std::string& row : canvas) {
        os << row << '\n';
    }
}

} // namespace planning
