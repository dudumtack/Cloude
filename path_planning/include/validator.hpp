#pragma once

// =====================================================================
//  validator.hpp
//  사용자의 planPath()가 돌려준 경로가 "진짜 유효한지" 검사한다.
//  당신이 로직을 고칠 때마다, 어디가 틀렸는지 바로 알려주는 역할.
// =====================================================================

#include <string>
#include <vector>

#include "grid.hpp"

namespace planning {

// 이동 방식: 검증 시 인접 규칙을 결정한다.
enum class Movement {
    FourWay,   // 상하좌우만 인접으로 인정
    EightWay   // 대각선 포함 8방향 인접 인정
};

struct ValidationResult {
    bool   valid = false;             // 모든 규칙을 통과했는가
    double cost  = 0.0;               // 경로 총 이동 비용(직선 1, 대각선 √2)
    int    length = 0;                // 경로 칸 수
    std::vector<std::string> errors;  // 실패 사유(없으면 통과)
};

// 경로를 검증한다.
//   빈 경로({})는 "경로 없음"으로 간주 → valid=false, 사유 1건.
//   allow_corner_cutting=false 이면 대각선 이동 시 양옆 칸이
//   모두 뚫려 있어야 통과(로봇이 벽 모서리를 파고들지 못하게).
ValidationResult validatePath(const Grid& grid,
                              const Cell& start,
                              const Cell& goal,
                              const Path& path,
                              Movement move = Movement::EightWay,
                              bool allow_corner_cutting = false);

} // namespace planning
