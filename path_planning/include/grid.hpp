#pragma once

// =====================================================================
//  grid.hpp
//  경로 알고리즘을 개발/테스트하기 위한 2D 환경(격자) 정의.
//  ※ 이 파일에는 "경로 알고리즘"이 들어있지 않습니다.
//    알고리즘 로직은 user/my_planner.cpp 에서 직접 구현하세요.
// =====================================================================

#include <cstdint>
#include <vector>

namespace planning {

// 격자 위의 한 칸(셀) 좌표. x = 열(col), y = 행(row).
struct Cell {
    int x = 0;
    int y = 0;

    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Cell& o) const { return !(*this == o); }
};

// 사용자의 경로 알고리즘이 돌려줄 결과 타입: 시작 -> 목표 순서의 칸 목록.
using Path = std::vector<Cell>;

// 점유 격자(Occupancy Grid).
//   0 : 이동 가능한 빈 칸
//   1 : 장애물 (통과 불가)
// 저장은 행 우선(row-major): index = y * width + x
class Grid {
public:
    Grid() = default;
    Grid(int width, int height);

    int width()  const { return width_; }
    int height() const { return height_; }

    // 좌표가 격자 범위 안에 있는가.
    bool inBounds(const Cell& c) const;

    // 이동 가능한 칸인가 (범위 안 + 장애물 아님).
    bool isFree(const Cell& c) const;

    // 장애물 설정/해제.
    void setObstacle(const Cell& c, bool blocked = true);

    // 원시 값 (0=빈칸, 1=장애물).
    uint8_t at(const Cell& c) const;

private:
    int width_  = 0;
    int height_ = 0;
    std::vector<uint8_t> data_;
};

} // namespace planning
