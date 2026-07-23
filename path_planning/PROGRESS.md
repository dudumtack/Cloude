# 진행 상황 & 인수인계 노트 (로봇개 경로 알고리즘)

> 새 세션은 이 파일을 먼저 읽고 이어가면 됩니다.
> 브랜치: `claude/path-planning-dstar` (D* Lite 트랙) · 저장소: `dudumtack/cloude`
> 참고 브랜치: `claude/path-planning-astar` (A* 버전 보존).

## 무엇을 만들고 있나
로봇개의 **2D 경로 알고리즘(A\*)** 을 개발 중.
- **테스트 환경(하네스)** 은 완성되어 있음 → `include/`, `src/`, `tests/`. **건드릴 필요 없음.**
- **알고리즘 로직은 `user/my_planner.cpp` 의 `planPath()`** 한 곳에만 작성한다.
  (사용자가 로직을 직접 이해하며 짜는 방식 — 큰 리팩터링 말고 요청한 조각만 추가)

## 빌드 & 실행
```bash
cd path_planning
make run          # 또는: cmake -B build && cmake --build build && ./build/run_tests
./run_tests 4     # 4방향 모드 (기본 8방향)
```

## `my_planner.cpp` 현재까지 구현된 조각
- `Unit{Meter,Centimeter}` + `CellSize{w,h,unit}` — 셀 실제 크기. **직사각형·단위(m/cm) 대응.**
- `metersPerUnit()` — 단위→미터 환산.
- `stepCost()` — 인접 셀 간 실제 이동 비용(유클리드, 직사각형 대응).
- `gCost()` — g(x): 부모 g + stepCost.
- `hCost()` — h(x): 8방향 옥타일 거리를 직사각형으로 일반화(admissible).
- `Node{pos,g,h,f,parent}` + `makeNode()` — 가변 셀(parent는 인덱스, 향후 `vector<Node>` 풀용).
- `getGps()` / `getGoalGps()` + 전역 `current_x/current_y`, `goal_x/goal_y`
  — **사용자 소유 스텁. 실제 수신 로직 구현하지 말 것**(나중에 로봇개 ROS 토픽으로 교체).
  `current_x=위도, current_y=경도` 규약. goal 스텁은 현재 위치 북동쪽 ~20m 지점.
- `getObstacleCount()` / `getObstacle(id)` + 전역 `obstacle_count`, `obstacle_{a,b}_{dist,dir}`
  — **사용자 소유 스텁**(센서/토픽에서 옴). 장애물 여러 개 → id(0부터)로 조회.
  장애물의 보이는 면을 **양 끝점 A/B 선분**(각각 거리 m + 방향 rad)으로 표현
  — 사선 장애물 대응. 방향 규약: 0=북, 시계+. 아직 미사용.
- `gpsToCell()` — GPS(위경도) → 원점 기준 셀 인덱스.
- `makeCurrentCell()` / `makeGoalCell()` / `buildCells()` — 현재/목적지 GPS로 셀 생성(현재 위치 = 원점).
- `obstacleToCells()` — 장애물 1개(끝점 A/B)를 원점 기준 셀 목록으로 변환.
  A~B 선분을 반 칸 간격 샘플링. 아직 미사용.
- `inflateCost()` — 장애물 셀 주변 소프트 비용(inflation) 층.
  이웃(체비쇼프1)=+0.5, 바깥 한 겹(체비쇼프2)=+0.25, 중복 누적 허용.
  반환은 `(x,y)→비용` sparse map. 아직 미사용(A* 비용에 더할 예정).
- `getDynObstacleCount()` / `getDynObstacle(id)` + 전역 `dyn_count`, `dyn_{dist,bearing,heading,speed}`
  — **사용자 소유 스텁**(추적기/센서에서 옴). 동적 장애물 N개, id(0부터) 조회.
  위치=`dist`+`bearing`(있는 방위), `heading`=진행 방향(별개), `speed` m/s. 방향 규약 0=북/시계+.
- `dynamicCost()` — 동적 장애물 1개 → 소프트 비용 map.
  중심 두 겹 감싸 +0.7, 진행 방향으로 속도 비례 거리만큼 뻗으며 속도 비례 비용 추가(예측 위험구역).
  `inflateCost` 결과와 합칠 수 있음(중복 누적). 아직 미사용.
- **주행(이동) 함수** — 실제 로봇 제어(`test_goto_point.cpp`, Unitree Go2 ROS2)의
  "바라보고→전진→도착" 메커니즘을 ROS 없이 순수 함수로 옮김.
  프레임: odom(x=동, y=북, m). **[규약 확정] yaw=0=북(+y), 반시계(왼쪽)+.**
  - `MoveCmd{vx,vy,vyaw,arrived}` — 한 틱 이동 명령(로봇 `Move(vx,vy,vyaw)`에 대응).
  - `computeYawDelta()` — 목표를 바라보는 최단 회전량 (-π,π]. 북=0 기준 `atan2(-Δx,Δy)`.
  - `distanceTo()` — 목표까지 수평 거리(m).
  - `stepToward()` — 목표점 1개 향한 한 틱 명령(도착/회전/전진+보정). 상태 없이 매 틱 재판단.
  - **연결 예정**: A* 경로의 셀 웨이포인트를 "셀↔m 스케일"만으로 월드화(회전 불필요) 후 `stepToward`로 추종.
- **D\* Lite 본체** (`neighbors` / `softAt` / `CostField` + `struct DStarLite`) — **완성**.
  - `neighbors()` — 8방향 이웃 + 대각선 모서리 뚫기 방지(validator 규칙 3-1 동일). (공용)
  - `DStarLite` — 목표에서 역방향 탐색. 셀별 `g`/`rhs` 맵, 2요소 키 `[min(g,rhs)+h+km, min(g,rhs)]`,
    lazy-deletion 우선순위 큐. `init`→`computeShortestPath`→`extractPath`.
    `edgeCost` = `stepCost` + 소프트비용(양끝 절반, 대칭). `soft` 인자로 inflate/dynamic 얹음.
    `km`·상태 멤버 보유.
  - **증분 재계획 완성**: `moveTo(new_start)`(km 보정) + `replan(changed_cells)`(바뀐 셀+이웃만
    updateVertex 후 computeShortestPath). 검증: replan 결과 = 처음부터 계산과 **비용 동일(최적)**,
    로봇 주행 메인 루프(이동 중 장애물 발견) 정상 도달, 30×30 국소변화 반복 시 **재계산 대비 ~12배 적게 확장**.
- `planPath()` — **하네스 grid/start/goal 로 D\* Lite 1회 실행** → 경로 반환.
  GPS→셀(`buildCells`)은 여전히 호출하되 로봇 통합용 별도 트랙(아직 grid와 미연결).
  **테스트 4/5 통과, 경로 비용은 A\*와 동일(최적)**. 5번째 `unreachable`은 경로 없음이 정답(`{}`).
- ※ `Node`/`makeNode`/`gCost` 는 A* 잔재라 D* 브랜치에선 미사용(경고). 재사용 여지 있어 남겨둠.

## 설계 결정 / 제약 (이어갈 때 지킬 것)
- **GPS 함수는 사용자 것** — 로직 채우지 말고 가정만 유지.
- **원점 = 현재 위치** → 현재 셀은 항상 `(0,0)`, 목표 셀은 상대 좌표.
- **북쪽 = +y** 로 나옴. 격자 시각화는 y가 아래로 증가라 축 뒤집힘이 있으나,
  **[확정] odom yaw=0 을 북쪽(+y)에 맞춤, 반시계(왼쪽)+.** planner 축과 일치 →
  셀↔월드는 스케일만(회전 불필요). 주행 함수(`computeYawDelta` 등)가 이 규약을 씀.
- 정사각형↔직사각형, m↔cm 전환은 `CellSize` 한 곳에서만 바꾸면 되도록 유지.

## 다음 할 일 (순서 제안)
> D* Lite 알고리즘 자체는 **완성**(1회 계산 + 증분 재계획 + 로봇 메인 루프 검증). 이제 로봇 연결 단계.
1. **격자 조립** — start/goal 바운딩 박스 + 마진으로 `Grid` 만들고, 정적/동적 장애물
   셀(`obstacleToCells`)을 찍고 `inflateCost`/`dynamicCost`로 `CostField` 생성.
2. **소프트 비용 연결** — 위 `CostField`를 `planner.init(grid,start,goal,cell,soft)`로 전달.
3. **셀↔월드 브리지** — 경로 셀을 월드(m)로(스케일만) 바꿔 `stepToward`로 주행.
4. **로봇 루프 배선** — 매 틱: 센서 장애물 변화 감지 → `moveTo`+`replan` → `extractPath` → `stepToward`.
5. **통합 결정** — GPS→셀 트랙과 하네스 grid/start/goal 을 실제로 어떻게 연결할지.
6. (선택) **4방향 모드** — 지금 8방향 고정. `run_tests 4` 검증엔 대각선이 걸림.

## 참고
- `hCost`/`stepCost` 는 D* Lite가 사용. `gCost`/`Node`/`makeNode` 는 A* 잔재라 미사용(경고 정상).
- 아직 미사용(경고 정상): `obstacleToCells`, `inflateCost`, `dynamicCost`, 주행 함수들,
  동적/정적 장애물 스텁 — 격자 조립·주행 연결 단계에서 붙는다.
