// =====================================================================
//  grid.cpp  — 2D 격자 환경 구현
// =====================================================================

#include "grid.hpp"

namespace planning {

Grid::Grid(int width, int height)
    : width_(width), height_(height),
      data_(static_cast<size_t>(width) * height, 0) {}

bool Grid::inBounds(const Cell& c) const {
    return c.x >= 0 && c.x < width_ && c.y >= 0 && c.y < height_;
}

uint8_t Grid::at(const Cell& c) const {
    return data_[static_cast<size_t>(c.y) * width_ + c.x];
}

bool Grid::isFree(const Cell& c) const {
    return inBounds(c) && at(c) == 0;
}

void Grid::setObstacle(const Cell& c, bool blocked) {
    if (!inBounds(c)) return;
    data_[static_cast<size_t>(c.y) * width_ + c.x] = blocked ? 1 : 0;
}

} // namespace planning
