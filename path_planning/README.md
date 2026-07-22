# Path Planning Test Harness

로봇개의 **2D 경로 알고리즘**을 개발·테스트하기 위한 C++ 환경입니다.

> 이 저장소는 **테스트 환경(하네스)** 만 제공합니다.
> 경로 알고리즘 로직은 `user/my_planner.cpp` 에서 **직접** 구현하세요.
> 나머지(맵 로딩, 충돌·연결성 검증, 시각화, 실행시간 측정)는 환경이 자동으로 처리합니다.

## 당신이 채우는 곳은 딱 한 곳

```cpp
// user/my_planner.cpp
planning::Path planPath(const Grid& grid, Cell start, Cell goal);
```

- `grid` : 장애물이 표시된 2D 격자 (`grid.isFree({x,y})` 로 이동 가능 여부 확인)
- 반환 : `start`→`goal` 로 이어지는 **인접한 칸들의 목록**. 경로가 없으면 빈 `{}`.

## 빌드 & 실행

CMake:
```bash
cmake -B build && cmake --build build
./build/run_tests            # 8방향(대각선 허용)
./build/run_tests 4          # 4방향(상하좌우)
```

또는 Makefile:
```bash
make run
```

맵 파일을 추가로 넘기면 그 시나리오도 함께 테스트합니다:
```bash
./run_tests maps/rooms.map
```

## 출력 예시

각 시나리오마다 격자/경로가 그려지고 지표가 나옵니다:

```
[시나리오] single_wall  (10x5)
S....#....
.***.#....
...*.#....
...*.#....
...******G
  · 소요시간 : 0.03 ms
  · 경로길이 : 12 칸
  · 이동비용 : 10.24
  ✅ 통과 (유효한 경로)
```

기호: `#` 장애물 · `.` 빈 칸 · `S` 시작 · `G` 목표 · `*` 경로

## 검증 규칙 (validator.hpp)

당신의 경로가 다음을 모두 만족해야 `✅ 통과`:
1. 비어 있지 않음 (경로를 찾음)
2. 첫 칸 = `start`, 마지막 칸 = `goal`
3. 모든 칸이 격자 안 + 장애물 아님
4. 연속한 칸끼리 인접 (4방향 또는 8방향)
5. 대각선 이동 시 벽 모서리를 파고들지 않음 (기본값)

> `unreachable` 시나리오는 **일부러 경로가 없는** 맵이라, 빈 경로를 반환하는 것이 정답입니다.

## 디렉터리 구조

```
path_planning/
├─ include/        # 환경 헤더 (건드릴 필요 없음)
│  ├─ grid.hpp        2D 격자
│  ├─ planner.hpp     ★ 당신이 구현할 함수 선언
│  ├─ validator.hpp   경로 검증 규칙
│  ├─ renderer.hpp    ASCII 시각화
│  └─ scenario.hpp    테스트 맵
├─ src/            # 환경 구현 (건드릴 필요 없음)
├─ user/
│  └─ my_planner.cpp  ★★★ 당신의 알고리즘 ★★★
├─ tests/
│  └─ run_tests.cpp   테스트 러너
├─ maps/           # 텍스트 맵 파일 (.map)
├─ CMakeLists.txt
└─ Makefile
```

## 시나리오 추가하기

- **텍스트 맵 파일**: `maps/` 에 `.map` 파일을 만들고 `#`(장애물), `.`(빈칸), `S`(시작), `G`(목표)로 그린 뒤 실행 인자로 넘기세요.
- **코드 내장**: `src/scenario.cpp` 의 `builtinScenarios()` 에 `fromAsciiMap(...)` 로 추가하세요.
