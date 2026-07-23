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

---

# 🤖 로봇 통합 가이드 (로컬 세션 작업용)

> 이 섹션은 **로컬 세션에서 실제 로봇개(Unitree Go2) 제어 코드**(`test_goto_point.cpp`
> 같은 rviz2 클릭→이동 예제)**와 이 저장소의 경로 알고리즘을 나란히 놓고 통합**할 때
> 필요한 것들을 모아둔 것입니다. 세부 진행 로그는 `PROGRESS.md` 참고.
> 브랜치: `claude/path-planning-dstar` (D* Lite). A* 버전은 `claude/path-planning-astar`.

## 1. 지금 무엇이 완성돼 있나 (`user/my_planner.cpp`)

- **D\* Lite 경로 탐색** — `struct DStarLite` : `init → computeShortestPath → extractPath`,
  + 증분 재계획 `moveTo(new_start)` / `replan(changed_cells)`. 하네스 4/5 통과(최적).
- **격자 조립** — `buildLocalMap(start_cell, goal_cell, margin)` : 원점기준 셀 → 로컬 `Grid`.
- **비용** — `inflateCost`(정적 근접), `dynamicCost`(동적 예측). D*의 `soft` 인자로 얹음.
- **주행 계산(ROS 무의존)** — `computeYawDelta`, `distanceTo`, `stepToward`, `MoveCmd`.
- **입력 스텁(사용자 소유, 로봇 데이터로 교체 대상)** — `getGps`/`getGoalGps`,
  `getObstacleCount`/`getObstacle`, `getDynObstacleCount`/`getDynObstacle`.

## 2. 두 개의 트랙 (헷갈리지 말 것)

| | 하네스 트랙 | 로봇 트랙 |
|---|---|---|
| 입력 | `planPath(grid,start,goal)` 가 격자를 받음 | 센서/스텁에서 좌표·장애물을 받아 **직접** 격자를 만듦 |
| 격자 | 이미 있음 | `buildLocalMap` 으로 조립 |
| 용도 | 알고리즘 검증(테스트) | 실제 주행 |

`planPath` 는 지금 하네스 트랙만 연결돼 있음. 로봇 트랙은 `buildLocalMap`+`DStarLite`+주행 함수를
묶는 **로봇 루프**(§6)를 새로 배선해야 함.

## 3. 좌표계·방향 규약 ⚠️ (통합의 핵심, 반드시 정합)

세 좌표계가 있음:

1. **셀 좌표** — 원점=현재 위치. `start=(0,0)`, 나머지는 상대. **음수 가능**. `+x=동, +y=북`.
2. **그리드 인덱스** — `Grid`가 요구하는 `0..W-1`. `그리드 = 셀좌표 - offset`
   (`buildLocalMap`이 `offset`을 정함. `toGrid`/`toCell`로 왕복).
3. **월드(odom) 좌표** — 로봇 실제 미터. `+x=동, +y=북`(맞췄을 때).

**[확정한 방향 규약] odom yaw=0 = 북(+y), 반시계(왼쪽)+.** `computeYawDelta`가 `atan2(-Δx,Δy)`로
이 규약을 씀.

> ❗ **함정**: 실제 Go2의 odom yaw=0 은 "부팅 시점의 로봇이 바라본 방향"이지 **북쪽이 아님**.
> 예제 `test_goto_point.cpp`의 `computeYawDelta`는 `atan2(Δy,Δx)`(=odom x축 기준)를 씀.
> 통합 시 **둘 중 하나**를 반드시 선택:
> - (A) 시작 시 IMU로 odom을 북에 정렬(yaw offset 보정) → 우리 규약 그대로 사용, 또는
> - (B) 우리 `computeYawDelta`를 로봇 odom 기준으로 되돌림(예제와 동일 `atan2(Δy,Δx)`).
> 안 맞추면 "목표를 바라보는 회전"이 90°씩 틀어짐.

## 4. 실제 로봇 코드 ↔ 우리 함수 대응표

`test_goto_point.cpp` (Go2 예제) 기준:

| 로봇 예제 | 우리 코드(교체/대응) | 메모 |
|---|---|---|
| `state_.position[0],[1]` (odom m) | `getGps`→`current_x/y` 대신 **직접 사용** | Go2는 GPS가 아니라 **odom 미터**. 위경도 변환(`gpsToCell`) 불필요 — 미터÷셀크기로 바로 셀화. GPS 트랙은 실외 전역목표일 때만. |
| `state_.imu_state.rpy[2]` (yaw) | `stepToward`의 `yaw` 인자 | §3 방향 규약 정합 필요 |
| `GetClickedPoint(x,y,z)` (rviz2) | `getGoalGps`→목표 | 목표를 클릭/토픽에서 받음 |
| `GetLidarPoints(pts)` (odom 점들) | `getObstacle*` → `obstacleToCells` 대체 | 라이다 점을 셀로 찍어 장애물화(§5) |
| `GetFrontBlockedDist(dist)` | (선택) 반응형 급정지용 | D*와 별개의 안전장치로 병용 가능 |
| `sport_client_.Move(req_, vx,vy,vyaw)` | `stepToward`가 만든 `MoveCmd` | `Move(req_, cmd.vx, cmd.vy, cmd.vyaw)` |
| `sport_client_.StopMove(req_)` | `MoveCmd.arrived==true` | 도착 시 정지 |
| `RobotControl()` 0.1s 루프 | **로봇 루프**(§6) | phase 상태기계 → D* 경로 추종으로 대체 |
| `computeYawDelta(...)` | **동일 함수 이미 이식됨** | 우리 것과 로직 같음(부호 규약만 §3) |

## 5. 라이다 → 장애물 셀 (교체 포인트)

예제의 `GetLidarPoints`는 **odom 절대좌표 점 배열**을 줌. 통합 시 `getObstacle*` 스텁 대신:

```
각 라이다 점 p (높이 band 필터: Z_MIN<z<Z_MAX):
  월드(m) → 셀 = round((p - 현재위치)/셀크기)   // 원점=현재위치
  그 셀을 장애물로 표시
→ buildLocalMap 이 이 셀들을 grid에 찍고 inflateCost로 근접비용 생성
```

즉 `obstacleToCells`(선분 모델) 대신 **라이다 점 직접 셀화**가 실제로는 더 자연스러움.
`buildLocalMap`의 "정적 장애물 셀 목록"만 이 방식으로 채우면 나머지(조립·D*)는 그대로.

## 6. 로봇 루프 의사코드 (남은 배선의 최종 형태)

```
DStarLite planner;  bool first = true;

매 0.1초:
  현재위치 = state_.position (m)
  현재셀   = round(현재위치 / 셀크기)          // 원점 보정 포함
  목표셀   = round(목표위치 / 셀크기)

  라이다 → 장애물 셀 목록 수집
  변화 감지 = (이번 장애물 셀) - (지난 장애물 셀)

  if first:
     LocalMap m = buildLocalMap(현재셀, 목표셀, margin)
     planner.init(m.grid, m.start, m.goal, cell, m.soft)
     planner.computeShortestPath()
     first = false
  else if 변화 있음:
     planner.moveTo(현재셀→그리드)             // km 보정
     grid/soft 갱신(바뀐 셀)
     planner.replan(바뀐 셀들)                 // 부분 재계산(D*의 이점)

  Path 경로 = planner.extractPath()            // 그리드 인덱스
  다음웨이포인트 = 경로[1]                       // 현재 다음 칸
  월드목표 = toCell(다음웨이포인트, offset) * 셀크기   // ← 셀↔월드 브리지(아직 미구현)
  MoveCmd c = stepToward(현재x, 현재y, yaw, 월드목표x, 월드목표y)
  if c.arrived: sport_client_.StopMove(req_)
  else:         sport_client_.Move(req_, c.vx, c.vy, c.vyaw)
```

## 7. 아직 안 된 것 (로컬에서 만들 것)

1. **셀↔월드 브리지** — 경로 셀(그리드) → `toCell(+offset)` → ×셀크기(m) → 월드. **스케일만**(§3 규약 덕에 회전 불필요). *← 여기부터 하면 됨.*
2. **로봇 루프 배선**(§6) — ROS 노드/제어 스레드 안에 위 흐름을 넣기. (이 저장소엔 ROS 없음 → 로컬의 로봇 워크스페이스에서)
3. **라이다→셀**(§5) 교체 — `getObstacle*` 스텁을 실제 점군 처리로.
4. **방향 규약 정합**(§3) — odom 북 정렬 or `computeYawDelta` 되돌리기.
5. (선택) 동적 장애물 트래킹, 4방향 모드.

## 8. 빌드/실험 팁

- 이 저장소는 순수 알고리즘이라 **ROS 없이** `make run` 으로 D*·비용·조립 로직을 바로 검증 가능.
- 실제 로봇 워크스페이스에선 `user/my_planner.cpp`의 함수들을 그대로 가져다 쓰되,
  스텁(`getGps`/`getObstacle*` 등)만 로봇 토픽 콜백으로 교체.
- 실행 전(예제 기준): `source ~/unitree_ros2/setup.sh`,
  `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ChannelFactory::Init(0,"eno2")` 먼저 → `rclcpp::init`.
- 빌드 경고 "defined but not used"(주행 함수·장애물 스텁 등)는 로봇 루프 배선 전까지 **정상**.
