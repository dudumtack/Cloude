/**********************************************************************
 dstar_goto.cpp — D* Lite 경로계획을 실제 Go2(로봇개)에 배선한 통합 노드

   Cloude 저장소 브랜치 `claude/path-planning-dstar` 의 순수 알고리즘
   (user/my_planner.cpp) 을 그대로 가져와, 로봇의 odom·라이다 토픽과
   고수준 SportClient 명령에 연결한다. (README §6 "로봇 루프" + §7 "셀↔월드 브리지")

   사용법:
     ./dstar_goto <goal_dx> <goal_dy> [nic] [cell_res]
       goal_dx : 주행 시작 위치 기준 odom +x 방향 목표 거리(m)
       goal_dy : 주행 시작 위치 기준 odom +y 방향 목표 거리(m)
       nic     : 네트워크 인터페이스(기본 eno2)
       cell_res: 셀 한 변 길이(m, 기본 0.25)
     예) ./dstar_goto 3.0 0.0            # 시작 자세 odom x축으로 3m 앞
         ./dstar_goto 2.0 -1.5 eno2 0.2  # x+2m, y-1.5m 지점, 20cm 셀

   ── 설계 결정(README 가이드에서 갈리는 지점) ──
   1) 좌표계 = odom 그대로. 셀 축을 odom 축에 맞춤(+x=odom_x, +y=odom_y).
      원점 = 주행 시작 시 로봇 odom 위치(에피소드 내 고정) → 시작셀=(0,0).
      그리드·라이다장애물·목표가 전부 odom에서 나오므로 서로 정합.
      → 셀↔월드는 스케일(×cell_res)만, 회전 불필요.
   2) 방향 규약 = 옵션 B. Go2 odom yaw=0 은 "북"이 아니라 부팅 방향이므로
      planner의 북기준 computeYawDelta(atan2(-Δx,Δy)) 대신, 여기서 odom 기준
      atan2(Δy,Δx) 로 다시 구현(computeYawDeltaOdom). IMU 북정렬 불필요.
   3) planner 재사용 = my_planner.cpp 를 그대로 #include. DStarLite·inflateCost·
      toGrid/toCell·CellSize 등이 전부 그 파일 안 static 이라 헤더로 안 나옴 →
      텍스트 포함으로 재사용(원본 무수정). getObstacle 등 스텁은 여기서 안 씀.
   4) 장애물 = 실제 라이다. buildLocalMap 은 가짜 스텁에 묶여 있어 안 쓰고,
      deskewed 점군을 셀로 찍는 buildLocalMapFromLidar() 를 두되 조립 내부는
      planner의 inflateCost/toGrid 를 재사용.

   ⚠️ 초기화 순서 필수: ChannelFactory::Init() → rclcpp::init()  (반대면 라이다 수신 불가)
      RMW_IMPLEMENTATION=rmw_fastrtps_cpp 필요.

   ── 알려진 한계(브링업용) ──
   · 그리드는 첫 계획 때 목표+margin 으로 크기를 고정. 그 밖에서 나중에 보이는
     장애물은 무시(inBounds 가드). 대부분 목표 경로 회랑은 이 안에 들어옴.
   · 라이다에서 사라진 점은 "빈칸"으로 되돌리지 않음(보수적). 지나간 장애물은 유지.
***********************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

// RViz2 시각화용 메시지/TF
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

// D* Lite 순수 알고리즘 본체(원본 무수정). 안의 static 헬퍼까지 이 TU에서 재사용.
// (Grid 구현은 grid.cpp 를 함께 컴파일해서 링크 — CMake 참고)
#include "my_planner.cpp"

using planning::Cell;
using planning::CellSize;
using planning::CostField;
using planning::DStarLite;
using planning::Grid;
using planning::LocalMap;
using planning::Path;
using planning::Unit;

#define TOPIC_CLOUD_DESKEWED "rt/utlidar/cloud_deskewed"
#define TOPIC_HIGHSTATE      "lf/sportmodestate"

// ───────── odom 기준 주행 헬퍼 (옵션 B: yaw=0 = odom +x, 반시계+) ─────────

// 현재 자세(rx,ry,yaw)에서 목표(tx,ty)를 바라보는 최단 회전량 (-π,π]. 양수=왼쪽.
//   bearing = atan2(Δy,Δx)  ← odom x축 기준(로봇 예제와 동일 규약).
static double computeYawDeltaOdom(double rx, double ry, double yaw,
                                  double tx, double ty) {
  double bearing = std::atan2(ty - ry, tx - rx);
  double delta   = bearing - yaw;
  return std::atan2(std::sin(delta), std::cos(delta));  // (-π,π] 정규화
}

// 목표점 하나를 향한 한 틱 이동 명령(도착/회전/전진+보정). planner::stepToward 의
// odom 규약 버전. distanceTo·MoveCmd 는 planner 것을 그대로 재사용.
static planning::MoveCmd stepTowardOdom(double rx, double ry, double yaw,
                                        double tx, double ty) {
  const double ARRIVE_R = 0.20;  // 도착 판정 반경(m)
  const double FACE_TOL = 0.15;  // "보고 있다" 판정(rad, ~9°)
  const double TURN_SPD = 0.6;   // 제자리 회전 속도(rad/s)
  const double FWD_SPD  = 0.4;   // 전진 속도(m/s)

  planning::MoveCmd cmd;
  double dist = planning::distanceTo(rx, ry, tx, ty);
  double dyaw = computeYawDeltaOdom(rx, ry, yaw, tx, ty);

  if (dist < ARRIVE_R) {
    cmd.arrived = true;
  } else if (std::fabs(dyaw) > FACE_TOL) {
    cmd.vyaw = (dyaw > 0) ? TURN_SPD : -TURN_SPD;  // 회전만
  } else {
    cmd.vx   = FWD_SPD;
    cmd.vyaw = 0.6 * dyaw;  // 전진하며 미세 보정
  }
  return cmd;
}

// ───────── 라이다 deskewed 점군 → 원점기준 셀(장애물) getter ─────────
//   patrol_to_lidar.cpp 의 CloudHandler 를 "정면 최근접 1점"이 아니라
//   "높이 band 안 모든 점을 셀로" 모으도록 확장.
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

// ───────── 라이다 장애물 셀로 로컬 D* 격자 조립 (buildLocalMap 의 라이다 버전) ─────────
//   원본 buildLocalMap 은 getObstacle 스텁을 부르므로 그대로는 못 씀. 여기선 실제
//   장애물 셀을 인자로 받고, 조립 내부(바운딩박스·offset·inflate)는 planner 재사용.
// (셀좌표만 다루므로 CellSize 불필요 — 원본 buildLocalMap과 달리 obstacleToCells를
//  안 쓰고 이미 셀화된 라이다 점을 받는다.)
static LocalMap buildLocalMapFromLidar(Cell start_cell, Cell goal_cell,
                                       const std::vector<Cell> &obstacles,
                                       int margin) {
  int minx = std::min(start_cell.x, goal_cell.x);
  int maxx = std::max(start_cell.x, goal_cell.x);
  int miny = std::min(start_cell.y, goal_cell.y);
  int maxy = std::max(start_cell.y, goal_cell.y);
  auto include = [&](int x, int y) {
    minx = std::min(minx, x); maxx = std::max(maxx, x);
    miny = std::min(miny, y); maxy = std::max(maxy, y);
  };
  for (const Cell &c : obstacles) include(c.x, c.y);
  minx -= margin; miny -= margin; maxx += margin; maxy += margin;

  LocalMap m;
  m.offset = Cell{minx, miny};
  m.grid   = Grid(maxx - minx + 1, maxy - miny + 1);
  m.start  = planning::toGrid(start_cell, m.offset);
  m.goal   = planning::toGrid(goal_cell, m.offset);

  for (const Cell &c : obstacles) {
    Cell gc = planning::toGrid(c, m.offset);
    if (m.grid.inBounds(gc) && !(gc == m.start) && !(gc == m.goal))
      m.grid.setObstacle(gc, true);
  }
  // 정적 근접 소프트 비용(벽에 바짝 안 붙게) — planner의 inflateCost 재사용.
  CostField inf = planning::inflateCost(obstacles);
  for (const auto &kv : inf) {
    Cell gc = planning::toGrid(Cell{kv.first.first, kv.first.second}, m.offset);
    if (m.grid.inBounds(gc)) m.soft[{gc.x, gc.y}] += kv.second;
  }
  return m;
}

// ───────── 통합 노드: odom 구독 + 라이다 + D* Lite 루프 + SportClient ─────────
class DStarGotoNode : public rclcpp::Node {
 public:
  DStarGotoNode(double goal_dx, double goal_dy, double cell_res)
      : Node("dstar_goto_node"),
        sport_client_(this),
        goal_dx_(goal_dx),
        goal_dy_(goal_dy) {
    cell_ = CellSize{cell_res, cell_res, Unit::Meter};

    // RViz2 퍼블리셔 + TF 브로드캐스터 (fixed frame = "odom")
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("dstar/local_map", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("dstar/path", 1);
    goal_pub_ = create_publisher<visualization_msgs::msg::Marker>("dstar/goal", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          std::lock_guard<std::mutex> lk(pose_mtx_);
          state_    = *data;
          have_pose_ = true;
        });

    ctrl_thread_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // spin 대기
      while (rclcpp::ok()) {
        RobotControl();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 0.1s
      }
    });
  }

  ~DStarGotoNode() {
    if (ctrl_thread_.joinable()) ctrl_thread_.join();
  }

  void AttachLidar(LidarObstacles *lidar) { lidar_ = lidar; }

 private:
  double cellRes() const { return cell_.w; }  // Meter 단위 정사각 셀

  // odom(m) → 원점기준 셀
  Cell odomToCell(double x, double y) const {
    return Cell{(int)std::lround((x - origin_x_) / cellRes()),
                (int)std::lround((y - origin_y_) / cellRes())};
  }
  // 그리드 인덱스 → odom 월드(m). (§7 셀↔월드 브리지: toCell(+offset) ×res +origin)
  void gridToOdom(Cell g, double &wx, double &wy) const {
    Cell c = planning::toCell(g, map_.offset);  // 원점기준 셀
    wx = origin_x_ + c.x * cellRes();
    wy = origin_y_ + c.y * cellRes();
  }

  // ── RViz2 시각화 (fixed frame = "odom") ──

  // 로봇 pose 를 TF odom→base_link 로 방송 (격자 위에서 로봇이 움직이게).
  void PublishTf(double rx, double ry, double yaw) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = now();
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";
    t.transform.translation.x = rx;
    t.transform.translation.y = ry;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  // 로컬 점유격자를 OccupancyGrid 로 발행(장애물=100, 빈칸=0).
  void PublishMap() {
    nav_msgs::msg::OccupancyGrid og;
    og.header.stamp    = now();
    og.header.frame_id = "odom";
    og.info.resolution = cellRes();
    og.info.width      = map_.grid.width();
    og.info.height     = map_.grid.height();
    double ox, oy;
    gridToOdom(Cell{0, 0}, ox, oy);            // 셀(0,0) 중심의 odom 위치
    og.info.origin.position.x    = ox - cellRes() * 0.5;  // 셀 좌하단 모서리로 보정
    og.info.origin.position.y    = oy - cellRes() * 0.5;
    og.info.origin.orientation.w = 1.0;
    og.data.assign((size_t)og.info.width * og.info.height, 0);
    for (int y = 0; y < map_.grid.height(); ++y)
      for (int x = 0; x < map_.grid.width(); ++x)
        og.data[(size_t)y * og.info.width + x] =
            map_.grid.at(Cell{x, y}) ? 100 : 0;
    grid_pub_->publish(og);
  }

  // D* 경로를 Path 로 발행.
  void PublishPath(const Path &path) {
    nav_msgs::msg::Path msg;
    msg.header.stamp    = now();
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

  // 목표점을 빨간 구 Marker 로 발행.
  void PublishGoal() {
    double gx, gy;
    gridToOdom(planner_.goal, gx, gy);
    visualization_msgs::msg::Marker m;
    m.header.stamp    = now();
    m.header.frame_id = "odom";
    m.ns   = "dstar";
    m.id   = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x    = gx;
    m.pose.position.y    = gy;
    m.pose.position.z    = 0.1;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.r = 1.0;  m.color.g = 0.2;  m.color.b = 0.2;  m.color.a = 1.0;
    goal_pub_->publish(m);
  }

  // ── 0.1초마다 호출되는 로봇 루프(README §6) ──
  void RobotControl() {
    // 1) 현재 자세 스냅샷
    double rx, ry, yaw;
    {
      std::lock_guard<std::mutex> lk(pose_mtx_);
      if (!have_pose_) { std::cout << "odom 대기중..." << std::endl; return; }
      rx  = state_.position[0];
      ry  = state_.position[1];
      yaw = state_.imu_state.rpy[2];
    }

    // 2) 원점/목표 확정(최초 1회): 원점 = 시작 odom, 목표 = 상대 offset
    if (!origin_set_) {
      origin_x_ = rx;  origin_y_ = ry;
      goal_cell_ = odomToCell(rx + goal_dx_, ry + goal_dy_);
      origin_set_ = true;
      RCLCPP_INFO(get_logger(),
                  "원점 odom=(%.2f,%.2f), 목표 offset=(%.2f,%.2f) → 목표셀(%d,%d), 셀=%.2fm",
                  origin_x_, origin_y_, goal_dx_, goal_dy_,
                  goal_cell_.x, goal_cell_.y, cellRes());
    }

    PublishTf(rx, ry, yaw);  // RViz: 격자 위 로봇 위치(매 틱)

    Cell cur_cell = odomToCell(rx, ry);

    // 3) 라이다 컨텍스트 갱신 + 장애물 셀·정면거리 읽기
    std::vector<Cell> obs;
    float front = 1e9f;
    if (lidar_) {
      lidar_->SetContext(origin_x_, origin_y_, rx, ry, yaw, cellRes());
      obs   = lidar_->GetObstacleCells();
      front = lidar_->FrontDistance();
    }

    // 3-b) 반응형 비상정지(D*와 별개 안전장치)
    if (front < FRONT_STOP_M) {
      sport_client_.StopMove(req_);
      std::cout << "⚠️ 정면 " << front << "m — 비상정지" << std::endl;
      return;
    }

    // 4) 최초 계획: 라이다 장애물로 로컬맵 조립 + D* init
    if (!planned_) {
      map_ = buildLocalMapFromLidar(cur_cell, goal_cell_, obs, MARGIN);
      planner_.init(map_.grid, map_.start, map_.goal, cell_, map_.soft);
      planner_.computeShortestPath();
      known_obs_.clear();
      for (const Cell &c : obs) {
        Cell gc = planning::toGrid(c, map_.offset);
        if (map_.grid.inBounds(gc)) known_obs_.insert({gc.x, gc.y});
      }
      planned_ = true;
      RCLCPP_INFO(get_logger(), "D* 격자 %dx%d, offset(%d,%d), 장애물셀 %zu",
                  map_.grid.width(), map_.grid.height(), map_.offset.x,
                  map_.offset.y, known_obs_.size());
    } else {
      // 5) 증분 재계획: 로봇 이동 반영(moveTo) + 새 장애물만 changed 로 replan
      planner_.moveTo(planning::toGrid(cur_cell, map_.offset));
      std::vector<Cell> changed;
      for (const Cell &c : obs) {
        Cell gc = planning::toGrid(c, map_.offset);
        if (!map_.grid.inBounds(gc)) continue;        // 고정격자 밖은 무시
        if (gc == planner_.goal) continue;            // 목표칸은 막지 않음
        auto key = std::make_pair(gc.x, gc.y);
        if (known_obs_.count(key)) continue;          // 이미 아는 장애물
        map_.grid.setObstacle(gc, true);              // 새 장애물 반영
        known_obs_.insert(key);
        changed.push_back(gc);
      }
      if (!changed.empty()) {
        // inflate 소프트 비용도 갱신(새 장애물 주변). soft 는 멤버라 직접 대입.
        CostField inf = planning::inflateCost(changed);
        for (const auto &kv : inf) {
          Cell gc = planning::toGrid(Cell{kv.first.first, kv.first.second},
                                     map_.offset);
          if (map_.grid.inBounds(gc)) planner_.soft[{gc.x, gc.y}] += kv.second;
        }
        planner_.replan(changed);
        // (매 틱 스팸 방지: 진단은 아래 throttle 로그로 통합)
      }
    }

    // 6) 경로 추출 → 다음 웨이포인트 → 주행 명령
    Path path = planner_.extractPath();

    // RViz: 격자·경로·목표 발행(어떤 분기든 매 틱)
    PublishMap();
    PublishPath(path);
    PublishGoal();

    if (path.size() < 2) {
      // 도착했거나(=1) 경로 없음(=0)
      double gx, gy;
      gridToOdom(planner_.goal, gx, gy);
      if (planning::distanceTo(rx, ry, gx, gy) < 0.3) {
        sport_client_.StopMove(req_);
        std::cout << "🏁 목표 도착" << std::endl;
      } else {
        sport_client_.StopMove(req_);
        std::cout << "경로 없음(막힘) — 정지" << std::endl;
      }
      return;
    }

    // 최종 도착 판정은 "목표"까지 거리로만 (중간 웨이포인트로 판정하면 조기 정지).
    double gx, gy;
    gridToOdom(planner_.goal, gx, gy);
    double dgoal = planning::distanceTo(rx, ry, gx, gy);
    if (dgoal < ARRIVE_R) {
      sport_client_.StopMove(req_);
      std::cout << "🏁 목표 도착" << std::endl;
      return;
    }

    // pure-pursuit lookahead: 경로 위에서 로봇으로부터 LOOKAHEAD_M 이상 떨어진
    // 첫 지점을 겨냥(없으면 경로 끝=목표). 바로 앞 한 칸(0.25m)을 겨냥하면 장애물
    // 재계획으로 방향이 매 틱 튀어 제자리 회전만 하게 되므로, 더 먼 안정된 점을 본다.
    double tx = gx, ty = gy;  // 기본값: 목표
    for (size_t i = 1; i < path.size(); ++i) {
      double px, py;
      gridToOdom(path[i], px, py);
      tx = px; ty = py;
      if (planning::distanceTo(rx, ry, px, py) >= LOOKAHEAD_M) break;
    }

    planning::MoveCmd cmd = stepTowardOdom(rx, ry, yaw, tx, ty);
    sport_client_.Move(req_, cmd.vx, cmd.vy, cmd.vyaw);

    // 진단 로그(0.5초마다): 왜 도는지/가는지 눈으로 확인.
    if (++dbg_tick_ % 5 == 0) {
      double dyaw = computeYawDeltaOdom(rx, ry, yaw, tx, ty);
      RCLCPP_INFO(get_logger(),
                  "pose=(%.2f,%.2f) yaw=%.0f° | goal=(%.2f,%.2f) d=%.2fm | "
                  "path=%zu look=(%.2f,%.2f) dyaw=%.0f° → %s (vx=%.2f,vyaw=%.2f)",
                  rx, ry, yaw * 180.0 / M_PI, gx, gy, dgoal, path.size(), tx, ty,
                  dyaw * 180.0 / M_PI,
                  (cmd.vx > 0 ? "전진" : "회전"), cmd.vx, cmd.vyaw);
    }
  }

  // ── 파라미터 ──
  static constexpr int    MARGIN       = 12;    // 격자 여유(셀) — 우회 공간
  static constexpr double FRONT_STOP_M = 0.35;  // 정면 비상정지 거리(m)
  static constexpr double LOOKAHEAD_M  = 0.6;   // pure-pursuit 겨냥 거리(m)
  static constexpr double ARRIVE_R     = 0.25;  // 목표 도착 판정 반경(m)

  long dbg_tick_{0};  // 진단 로그 throttle 카운터

  SportClient sport_client_;
  unitree_api::msg::Request req_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;

  std::mutex pose_mtx_;
  unitree_go::msg::SportModeState state_;
  bool have_pose_{false};

  LidarObstacles *lidar_{nullptr};

  // RViz2 시각화
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 목표(시작 기준 odom 상대 offset, m)
  double goal_dx_, goal_dy_;
  CellSize cell_;

  // 에피소드 고정 원점(odom m) + 목표셀
  bool   origin_set_{false};
  double origin_x_{0}, origin_y_{0};
  Cell   goal_cell_{};

  // D* 상태
  bool     planned_{false};
  LocalMap map_;
  DStarLite planner_;
  std::set<std::pair<int, int>> known_obs_;  // 이미 격자에 반영된 장애물(그리드 인덱스)

  std::thread ctrl_thread_;
};

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <goal_dx_m> <goal_dy_m> [nic] [cell_res_m]\n"
              << "  예: " << argv[0] << " 3.0 0.0            # x축 3m 앞\n"
              << "      " << argv[0] << " 2.0 -1.5 eno2 0.2\n";
    return 1;
  }
  double goal_dx  = std::atof(argv[1]);
  double goal_dy  = std::atof(argv[2]);
  std::string nic = (argc >= 4) ? argv[3] : "eno2";
  double cell_res = (argc >= 5) ? std::atof(argv[4]) : 0.25;

  // rclcpp 는 반드시 FastRTPS 로 (SDK 네이티브 CycloneDDS 와 충돌 방지).
  // setup.sh 가 RMW_IMPLEMENTATION=cyclonedds 로 덮어써도, 여기서 강제해 footgun 제거.
  // (rclcpp::init 이 RMW 를 고르기 전에 설정해야 함. SDK 의 cyclonedds 는 별개라 무관.)
  setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);

  // ⚠️ 순서 필수: ChannelFactory 먼저, rclcpp 나중
  unitree::robot::ChannelFactory::Instance()->Init(0, nic);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DStarGotoNode>(goal_dx, goal_dy, cell_res);
  LidarObstacles lidar;
  node->AttachLidar(&lidar);

  // MultiThreadedExecutor: 제어 스레드가 Move 를 계속 쏘는 동안에도 odom 구독
  // 콜백이 별도 스레드에서 계속 처리되도록(단일 spin 이면 굶어서 odom 이 멈춤).
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
