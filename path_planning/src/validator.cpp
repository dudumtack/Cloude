// =====================================================================
//  validator.cpp — 경로 유효성 검사 구현
// =====================================================================

#include "validator.hpp"

#include <cmath>

namespace planning {

ValidationResult validatePath(const Grid& grid,
                              const Cell& start,
                              const Cell& goal,
                              const Path& path,
                              Movement move,
                              bool allow_corner_cutting) {
    ValidationResult r;

    // [규칙 0] 빈 경로 = 경로를 못 찾음.
    if (path.empty()) {
        r.errors.push_back("경로가 비어 있습니다 (planPath가 빈 결과 반환).");
        return r;
    }

    r.length = static_cast<int>(path.size());

    // [규칙 1] 시작/끝 칸이 요청한 start/goal과 일치해야 한다.
    if (path.front() != start) {
        r.errors.push_back("경로의 첫 칸이 start와 다릅니다.");
    }
    if (path.back() != goal) {
        r.errors.push_back("경로의 마지막 칸이 goal과 다릅니다.");
    }

    // [규칙 2] 모든 칸은 격자 안에 있고 장애물이 아니어야 한다.
    for (size_t i = 0; i < path.size(); ++i) {
        const Cell& c = path[i];
        if (!grid.inBounds(c)) {
            r.errors.push_back("칸 #" + std::to_string(i) + " 이(가) 격자 범위를 벗어났습니다.");
        } else if (!grid.isFree(c)) {
            r.errors.push_back("칸 #" + std::to_string(i) + " 이(가) 장애물 위에 있습니다.");
        }
    }

    // [규칙 3] 연속한 두 칸은 서로 인접해야 하고, 이동 비용을 누적한다.
    const int maxDiag = (move == Movement::EightWay) ? 1 : 0;
    for (size_t i = 1; i < path.size(); ++i) {
        const Cell& a = path[i - 1];
        const Cell& b = path[i];
        int dx = std::abs(a.x - b.x);
        int dy = std::abs(a.y - b.y);

        bool straight = (dx + dy == 1);                 // 상하좌우 한 칸
        bool diagonal = (dx == 1 && dy == 1);           // 대각선 한 칸

        if (straight) {
            r.cost += 1.0;
        } else if (diagonal && maxDiag == 1) {
            r.cost += 1.41421356237;
            // [규칙 3-1] 모서리 뚫기 방지: 대각선이면 양옆 직선 칸도 뚫려야 함.
            if (!allow_corner_cutting) {
                Cell sideA{a.x, b.y};
                Cell sideB{b.x, a.y};
                if (!grid.isFree(sideA) || !grid.isFree(sideB)) {
                    r.errors.push_back("칸 #" + std::to_string(i) +
                                       " 에서 벽 모서리를 파고드는 대각선 이동입니다.");
                }
            }
        } else {
            // 인접하지 않거나(칸 건너뜀), 4방향 모드인데 대각선을 씀.
            r.errors.push_back("칸 #" + std::to_string(i) +
                               " 이(가) 직전 칸과 올바르게 인접하지 않습니다 (dx=" +
                               std::to_string(dx) + ", dy=" + std::to_string(dy) + ").");
        }
    }

    r.valid = r.errors.empty();
    return r;
}

} // namespace planning
