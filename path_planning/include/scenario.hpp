#pragma once

// =====================================================================
//  scenario.hpp
//  테스트용 2D 장애물 맵(시나리오)을 만들고 불러오는 도구.
//  당신의 알고리즘을 다양한 상황에서 반복 검증할 수 있게 해준다.
// =====================================================================

#include <string>
#include <vector>

#include "grid.hpp"

namespace planning {

// 하나의 테스트 시나리오: 맵 + 시작/목표 + 사람이 읽을 이름.
struct Scenario {
    std::string name;
    Grid  grid;
    Cell  start;
    Cell  goal;
};

// 텍스트로부터 시나리오를 만든다. 각 문자는 한 칸을 의미:
//   '#'  장애물
//   '.'  또는 ' '  빈 칸
//   'S'  시작 (빈 칸으로 취급)
//   'G'  목표 (빈 칸으로 취급)
// 줄 길이가 달라도 가장 긴 줄에 맞춰 오른쪽을 빈 칸으로 채운다.
Scenario fromAsciiMap(const std::string& name, const std::vector<std::string>& rows);

// 텍스트 맵 파일(.map)을 읽어 시나리오로 만든다. 형식은 fromAsciiMap과 동일.
// 파일을 못 열면 이름이 비어 있는(빈) 시나리오를 반환한다.
Scenario loadMapFile(const std::string& path);

// 코드로 바로 쓸 수 있는 내장 시나리오 모음(파일 없이도 테스트 가능).
std::vector<Scenario> builtinScenarios();

} // namespace planning
