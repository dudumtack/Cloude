// =====================================================================
//  run_tests.cpp — 테스트 하네스 (실행 진입점)
//
//  하는 일:
//    1) 시나리오(내장 + maps/*.map 파일)를 모은다.
//    2) 각 시나리오에 대해 당신의 planPath()를 호출한다.
//    3) 실행 시간을 재고, 결과 경로를 검증한다.
//    4) 격자/경로를 그림으로 보여주고, 통과/실패를 요약한다.
//
//  당신은 이 파일을 건드릴 필요가 없습니다.
//  user/my_planner.cpp 의 planPath()만 채우면 됩니다.
// =====================================================================

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "grid.hpp"
#include "planner.hpp"
#include "renderer.hpp"
#include "scenario.hpp"
#include "validator.hpp"

using namespace planning;

namespace {

// 한 시나리오를 실행하고 통과 여부를 반환한다.
bool runOne(const Scenario& sc, Movement move) {
    std::cout << "────────────────────────────────────────────\n";
    std::cout << "[시나리오] " << sc.name
              << "  (" << sc.grid.width() << "x" << sc.grid.height() << ")\n";

    // 사용자 알고리즘 호출 + 실행 시간 측정.
    auto t0 = std::chrono::steady_clock::now();
    Path path = planPath(sc.grid, sc.start, sc.goal);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 결과 검증.
    ValidationResult v = validatePath(sc.grid, sc.start, sc.goal, path, move);

    // 시각화.
    render(std::cout, sc.grid, sc.start, sc.goal, path);

    // 지표 출력.
    std::cout << "  · 소요시간 : " << ms << " ms\n";
    std::cout << "  · 경로길이 : " << v.length << " 칸\n";
    std::cout << "  · 이동비용 : " << v.cost << "\n";

    if (v.valid) {
        std::cout << "  ✅ 통과 (유효한 경로)\n";
    } else {
        // 'unreachable' 처럼 일부러 경로가 없어야 하는 맵도 있으니
        // 빈 경로 자체는 상황에 따라 정상일 수 있음을 함께 안내.
        std::cout << "  ❌ 실패:\n";
        for (const std::string& e : v.errors) {
            std::cout << "       - " << e << "\n";
        }
    }
    return v.valid;
}

} // namespace

int main(int argc, char** argv) {
    // 4방향/8방향 선택: 인자로 "4" 를 주면 4방향, 기본은 8방향.
    Movement move = Movement::EightWay;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "4") move = Movement::FourWay;
    }

    std::cout << "이동 방식: "
              << (move == Movement::EightWay ? "8방향(대각선 허용)" : "4방향(상하좌우)")
              << "\n";

    // 시나리오 수집: 내장 + 커맨드라인으로 넘긴 .map 파일들.
    std::vector<Scenario> scenarios = builtinScenarios();
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "4" || arg == "8") continue;  // 이동 방식 플래그
        Scenario s = loadMapFile(arg);
        if (!s.name.empty()) {
            scenarios.push_back(s);
        } else {
            std::cout << "[경고] 맵 파일을 열 수 없음: " << arg << "\n";
        }
    }

    int passed = 0;
    for (const Scenario& sc : scenarios) {
        if (runOne(sc, move)) ++passed;
    }

    std::cout << "════════════════════════════════════════════\n";
    std::cout << "요약: " << passed << " / " << scenarios.size() << " 시나리오 통과\n";

    // 모두 통과하면 0, 하나라도 실패하면 1 (CI에서 활용 가능).
    return (passed == static_cast<int>(scenarios.size())) ? 0 : 1;
}
