/**********************************************************************
 astar_goto.cpp — A* 경로계획을 실제 Go2(로봇개)에 배선하는 통합 노드

   Cloude 저장소 브랜치 `claude/astar-work` 의 순수 알고리즘
   (user/my_planner.cpp) 을 그대로 가져와, 로봇의 odom·라이다 토픽과
   고수준 SportClient 명령에 연결한다. (dstar_goto.cpp 의 A* 버전)

   ── 지금까지 만든 조각 ──
   · [이 파일] 장애물 인식 = odom 좌표계 점군(deskewed cloud) → 원점기준 셀.
     (dstar_goto.cpp 의 LidarObstacles 를 그대로 이식 — A*/D* 무관하게
      "점군 → 장애물 셀" 만 하므로 알고리즘과 독립.)

   ── 다음 조각(예정) ──
   1. 격자 조립  : start/goal 바운딩박스 + margin 으로 Grid 만들고, 위 라이다
                   장애물 셀을 찍어 CostField(inflateCost) 까지 생성.
                   (astar 의 my_planner.cpp 에 LocalMap/toGrid/toCell 헬퍼 추가 필요)
   2. A* 루프    : 매 틱(또는 장애물 변화 시) aStar(grid,start,goal,cell,soft) 를
                   처음부터 다시 돌린다. (D* Lite 의 증분 replan 과 달리 A* 는
                   상태가 없어 재계산이 단순 — moveTo/replan 불필요.)
   3. 주행       : A* 경로 셀을 odom 월드(m)로 바꿔 stepTowardOdom 으로 추종.

   ── 설계 결정 (dstar_goto.cpp 와 동일하게 맞춤) ──
   · 좌표계 = odom 그대로. 원점 = 주행 시작 시 로봇 odom 위치(에피소드 고정).
     그리드·라이다장애물·목표가 전부 odom 에서 나오므로 셀↔월드는 스케일만.
   · 방향 규약 = 옵션 B. Go2 odom yaw=0 은 부팅 방향이므로, planner 의 북기준
     computeYawDelta 대신 odom 기준 atan2(Δy,Δx) 를 쓴다(주행 조각에서 추가).
   · planner 재사용 = my_planner.cpp 를 그대로 #include. 안의 static 헬퍼까지
     이 TU 에서 재사용(원본 무수정). getObstacle 등 스텁은 여기서 안 씀.

   ⚠️ 이 파일은 하네스 CMake 가 빌드하지 않는다(ROS2/Unitree SDK 의존).
      로봇 자체 ROS2 워크스페이스에서 빌드한다. (grid.cpp 를 함께 컴파일·링크)
      초기화 순서 필수: ChannelFactory::Init() → rclcpp::init()
      RMW_IMPLEMENTATION=rmw_fastrtps_cpp 필요.

   ── 알려진 한계(브링업용) ──
   · 라이다에서 사라진 점은 "빈칸"으로 되돌리지 않음(보수적). 지나간 장애물은 유지.
***********************************************************************/

#include <cmath>
#include <cstring>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// RViz2 시각화용 메시지 (경로/목적지)
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

// A* 순수 알고리즘 본체(원본 무수정). 안의 planning::Cell 등을 이 TU 에서 재사용.
// (Grid 구현은 grid.cpp 를 함께 컴파일해서 링크 — CMake 참고)
#include "my_planner.cpp"

using planning::Cell;
using planning::Path;

#define TOPIC_CLOUD_DESKEWED "rt/utlidar/cloud_deskewed"

// ───────── 라이다 deskewed 점군 → 원점기준 셀(장애물) getter ─────────
//   dstar_goto.cpp 의 LidarObstacles 를 그대로 이식.
//   높이 band(z_min~z_max) 안, 거리 band(min~max_range) 안의 모든 점을
//   원점기준 셀로 모은다. odom 절대좌표 점군을 받으므로 회전 없이 스케일만.
//   ⚠️ "임시" 장애물 인식 — 나중에 다른 인식 로직으로 교체 예정.
class LidarObstacles {
 public:
  explicit LidarObstacles(float z_min = 0.10f, float z_max = 1.50f,
                          float min_range = 0.30f, float max_range = 5.0f,
                          float front_half = 25.0f * (float)M_PI / 180.0f)
      : z_min_(z_min), z_max_(z_max),
        min_range_(min_range), max_range_(max_range), front_half_(front_half) {
    sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(
        TOPIC_CLOUD_DESKEWED);
    sub_->InitChannel([this](const void *msg) { this->Handle(msg); });
  }

  // 주행 루프가 매 틱 로봇 상태(odom 위치·yaw·원점·셀크기)를 알려준다.
  void SetContext(double origin_x, double origin_y, double robot_x,
                  double robot_y, double robot_yaw, double cell_res) {
    std::lock_guard<std::mutex> lk(ctx_mtx_);
    origin_x_ = origin_x;  origin_y_ = origin_y;
    robot_x_  = robot_x;   robot_y_  = robot_y;  robot_yaw_ = robot_yaw;
    cell_res_ = cell_res;  ctx_ready_ = true;
  }

  // 가장 최근 프레임의 장애물 셀 목록(원점기준, 중복 제거)을 복사해 반환.
  std::vector<Cell> GetObstacleCells() {
    std::lock_guard<std::mutex> lk(out_mtx_);
    return cells_;
  }

  // 정면(yaw ±front_half) 최근접 장애물까지 거리(m). 없으면 max_range_.
  float FrontDistance() {
    std::lock_guard<std::mutex> lk(out_mtx_);
    return front_dist_;
  }

 private:
  void Handle(const void *msg) {
    const auto *cloud = (const sensor_msgs::msg::dds_::PointCloud2_ *)msg;

    int off_x = -1, off_y = -1, off_z = -1;
    for (const auto &f : cloud->fields()) {
      if (f.name() == "x") off_x = (int)f.offset();
      else if (f.name() == "y") off_y = (int)f.offset();
      else if (f.name() == "z") off_z = (int)f.offset();
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) return;

    double ox, oy, rx, ry, ryaw, res;
    {
      std::lock_guard<std::mutex> lk(ctx_mtx_);
      if (!ctx_ready_) return;  // 원점/자세 아직 모름 → 셀화 불가
      ox = origin_x_;  oy = origin_y_;
      rx = robot_x_;   ry = robot_y_;  ryaw = robot_yaw_;  res = cell_res_;
    }
    if (res <= 0.0) return;

    const uint8_t *data = cloud->data().data();
    const uint32_t step = cloud->point_step();
    const uint32_t n    = cloud->width() * cloud->height();

    std::set<std::pair<int, int>> cells;
    float front_best = max_range_;
    for (uint32_t i = 0; i < n; i++) {
      float x, y, z;  // odom(월드) 절대좌표
      const uint8_t *p = data + (size_t)i * step;
      std::memcpy(&x, p + off_x, sizeof(float));
      std::memcpy(&y, p + off_y, sizeof(float));
      std::memcpy(&z, p + off_z, sizeof(float));

      if (z < z_min_ || z > z_max_) continue;  // 바닥/천장 band 밖 제거

      double dx = x - rx, dy = y - ry;
      double d  = std::sqrt(dx * dx + dy * dy);  // 로봇으로부터 거리
      if (d < min_range_ || d > max_range_) continue;

      // odom 절대점 → 원점기준 셀(정수, 음수 가능)
      int cx = (int)std::lround((x - ox) / res);
      int cy = (int)std::lround((y - oy) / res);
      cells.insert({cx, cy});

      // 정면 최근접 거리(비상정지용)
      double bearing = std::atan2(dy, dx);
      double ang = std::remainder(bearing - ryaw, 2.0 * M_PI);
      if (std::fabs(ang) <= front_half_ && d < front_best) front_best = (float)d;
    }

    std::vector<Cell> out;
    out.reserve(cells.size());
    for (const auto &c : cells) out.push_back(Cell{c.first, c.second});

    std::lock_guard<std::mutex> lk(out_mtx_);
    cells_ = std::move(out);
    front_dist_ = front_best;
  }

  float z_min_, z_max_, min_range_, max_range_, front_half_;
  std::shared_ptr<
      unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>
      sub_;

  std::mutex ctx_mtx_;
  bool ctx_ready_{false};
  double origin_x_{0}, origin_y_{0}, robot_x_{0}, robot_y_{0}, robot_yaw_{0},
      cell_res_{0};

  std::mutex out_mtx_;
  std::vector<Cell> cells_;
  float front_dist_{1e9f};
};

// ───────── RViz2 시각화: A* 경로 + 목적지 (fixed frame = "odom") ─────────
//   dstar_goto.cpp 의 PublishPath / PublishGoal 를 이식.
//   노드/격자 조립이 아직 없으므로, 셀↔월드 브리지에 필요한 컨텍스트
//   (offset·원점·셀크기)를 SetFrame 으로 외부에서 주입해 결합을 끊었다.
//   → 격자 조립 조각이 붙으면 그때 정해지는 map.offset / origin / cell_res 를
//      매 계획마다 SetFrame 으로 넘기면 그대로 동작.
//   토픽:  astar/path (nav_msgs/Path) · astar/goal (visualization_msgs/Marker)
class AStarViz {
 public:
  explicit AStarViz(rclcpp::Node *node) : node_(node) {
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("astar/path", 1);
    goal_pub_ =
        node->create_publisher<visualization_msgs::msg::Marker>("astar/goal", 1);
  }

  // 셀↔월드 브리지 컨텍스트를 설정한다.
  //   grid_offset : 격자 (0,0) 이 가리키는 원점기준 셀 (셀좌표 = 그리드인덱스 + offset)
  //   origin_x/y  : 에피소드 고정 원점 (odom, m)
  //   cell_res    : 셀 한 변 길이 (m)
  void SetFrame(Cell grid_offset, double origin_x, double origin_y,
                double cell_res) {
    offset_   = grid_offset;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    res_      = cell_res;
  }

  // A* 경로(그리드 인덱스 목록)를 odom 프레임 Path 로 발행.
  void PublishPath(const Path &path) {
    nav_msgs::msg::Path msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "odom";
    for (const Cell &c : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      double wx, wy;
      gridToOdom(c, wx, wy);
      ps.pose.position.x    = wx;
      ps.pose.position.y    = wy;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  // 목표 셀(그리드 인덱스)을 빨간 구 Marker 로 발행.
  void PublishGoal(Cell goal_grid) {
    double gx, gy;
    gridToOdom(goal_grid, gx, gy);
    visualization_msgs::msg::Marker m;
    m.header.stamp    = node_->now();
    m.header.frame_id = "odom";
    m.ns     = "astar";
    m.id     = 0;
    m.type   = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x    = gx;
    m.pose.position.y    = gy;
    m.pose.position.z    = 0.1;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.r = 1.0;  m.color.g = 0.2;  m.color.b = 0.2;  m.color.a = 1.0;
    goal_pub_->publish(m);
  }

 private:
  // 그리드 인덱스 → odom 월드(m). 그리드→원점기준 셀(+offset)→미터(×res)+원점.
  //   (격자 조립 조각의 toCell(g,offset) 과 동일 계산을 인라인.)
  void gridToOdom(Cell g, double &wx, double &wy) const {
    wx = origin_x_ + static_cast<double>(g.x + offset_.x) * res_;
    wy = origin_y_ + static_cast<double>(g.y + offset_.y) * res_;
  }

  rclcpp::Node *node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
  Cell   offset_{0, 0};
  double origin_x_{0.0}, origin_y_{0.0}, res_{0.25};
};

// ───────── 다음 조각 여기부터 ─────────
//   · buildLocalMapFromLidar(start, goal, obstacles, margin)  — 격자 조립
//   · odom 주행 헬퍼(computeYawDeltaOdom / stepTowardOdom)
//   · AStarGotoNode  — odom 구독 + 라이다 + A* 재계획 루프 + SportClient
//                       (루프에서 AStarViz.SetFrame → PublishPath/PublishGoal 호출)
//   · main()         — ChannelFactory::Init → rclcpp::init → spin
//   · (선택) PublishMap(OccupancyGrid) / PublishTf(로봇 pose) — 격자·로봇 표시
