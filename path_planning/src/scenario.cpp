// =====================================================================
//  scenario.cpp — 테스트 시나리오 생성/로딩
// =====================================================================

#include "scenario.hpp"

#include <fstream>

namespace planning {

Scenario fromAsciiMap(const std::string& name, const std::vector<std::string>& rows) {
    Scenario s;
    s.name = name;

    int height = static_cast<int>(rows.size());
    int width  = 0;
    for (const std::string& r : rows) {
        width = std::max<int>(width, static_cast<int>(r.size()));
    }
    if (width == 0 || height == 0) {
        return s;  // 빈 맵
    }

    s.grid = Grid(width, height);
    bool has_start = false, has_goal = false;

    for (int y = 0; y < height; ++y) {
        const std::string& row = rows[y];
        for (int x = 0; x < width; ++x) {
            char ch = (x < static_cast<int>(row.size())) ? row[x] : ' ';
            switch (ch) {
                case '#':
                    s.grid.setObstacle({x, y}, true);
                    break;
                case 'S':
                    s.start = {x, y};
                    has_start = true;
                    break;
                case 'G':
                    s.goal = {x, y};
                    has_goal = true;
                    break;
                default:
                    break;  // '.', ' ' → 빈 칸
            }
        }
    }

    // S/G 표시가 없으면 좌상단/우하단을 기본값으로.
    if (!has_start) s.start = {0, 0};
    if (!has_goal)  s.goal  = {width - 1, height - 1};

    return s;
}

Scenario loadMapFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) return Scenario{};  // 이름 빈 시나리오 = 로드 실패

    std::vector<std::string> rows;
    std::string line;
    while (std::getline(f, line)) {
        // 끝의 캐리지리턴 제거(윈도우 줄바꿈 대비).
        if (!line.empty() && line.back() == '\r') line.pop_back();
        rows.push_back(line);
    }
    return fromAsciiMap(path, rows);
}

std::vector<Scenario> builtinScenarios() {
    std::vector<Scenario> out;

    // 1) 뻥 뚫린 맵 — 기본 동작 확인.
    out.push_back(fromAsciiMap("open_field", {
        "S........",
        ".........",
        ".........",
        ".........",
        "........G",
    }));

    // 2) 벽 하나에 통로 — 우회 여부 확인.
    out.push_back(fromAsciiMap("single_wall", {
        "S....#....",
        ".....#....",
        ".....#....",
        ".....#....",
        ".........G",
    }));

    // 3) ㄷ자 함정 — 지역 최소(local minimum)에 빠지는지 확인.
    out.push_back(fromAsciiMap("u_trap", {
        "..........",
        "..######..",
        "..#....#..",
        "..#.SG.#..",
        "..#....#..",
        "..######..",
        "..........",
    }));

    // 4) 좁은 지그재그 미로 — 대각선/모서리 처리 확인.
    out.push_back(fromAsciiMap("zigzag", {
        "S.#......",
        "..#.####.",
        "..#.#..#.",
        "..#.#.##.",
        "....#...G",
    }));

    // 5) 도달 불가 — 빈 경로를 올바로 반환하는지 확인.
    out.push_back(fromAsciiMap("unreachable", {
        "S..#..",
        "...#..",
        "...#.G",
        "...#..",
    }));

    return out;
}

} // namespace planning
