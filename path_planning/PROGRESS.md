# 진행 상황 & 인수인계 노트 (로봇개 경로 알고리즘)

> 새 세션은 이 파일을 먼저 읽고 이어가면 됩니다.
> 브랜치: `claude/robot-dog-project-3s09hc` · 저장소: `dudumtack/cloude`

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
- `planPath()` — 지금은 GPS→셀까지만 호출하고 **빈 경로 `{}` 반환**(A\* 루프 미구현).

## 설계 결정 / 제약 (이어갈 때 지킬 것)
- **GPS 함수는 사용자 것** — 로직 채우지 말고 가정만 유지.
- **원점 = 현재 위치** → 현재 셀은 항상 `(0,0)`, 목표 셀은 상대 좌표.
- **북쪽 = +y** 로 나옴. 격자 시각화는 y가 아래로 증가라 축 뒤집힘이 있으나,
  **나중에 IMU yaw(=0일 때 북쪽) / 센서로 맞추기로 함. 지금은 안 건드림.**
- 정사각형↔직사각형, m↔cm 전환은 `CellSize` 한 곳에서만 바꾸면 되도록 유지.

## 다음 할 일 (순서 제안)
1. **이웃 탐색** — 현재 셀의 8방향 이웃 + 대각선 모서리 뚫기 방지.
2. **open list** — f 최소 노드부터 꺼내는 우선순위 큐.
3. **A\* 메인 루프** — `gCost`/`hCost`/`makeNode` 활용해 확장 + 경로 복원.
   (이때 현재 unused 경고: `gCost`/`hCost`/`makeNode` 가 사라짐)
4. **통합 결정** — 하네스가 주는 `start/goal` 과 GPS→셀 결과를 어떻게 연결할지.

## 참고
- 빌드 시 `gCost`/`hCost`/`makeNode` "defined but not used" 경고는 A\* 루프 전까지 **정상**.
