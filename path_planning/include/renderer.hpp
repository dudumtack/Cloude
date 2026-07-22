#pragma once

// =====================================================================
//  renderer.hpp
//  격자와 경로를 터미널에 ASCII로 그려서 눈으로 확인하게 해준다.
// =====================================================================

#include <ostream>

#include "grid.hpp"

namespace planning {

// 격자 + 경로를 os 에 그린다.
//   '#' 장애물,  '.' 빈 칸,  'S' 시작,  'G' 목표,  '*' 경로가 지나간 칸.
// path 가 비어 있으면 장애물/시작/목표만 그린다.
void render(std::ostream& os,
            const Grid& grid,
            const Cell& start,
            const Cell& goal,
            const Path& path);

} // namespace planning
