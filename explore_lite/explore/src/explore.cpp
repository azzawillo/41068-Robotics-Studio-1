/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>
// --- helpers to keep goals valid / safe ---
bool isInsideCostmap(nav2_costmap_2d::Costmap2D* cm, double wx, double wy) {
  unsigned int mx, my;
  return cm->worldToMap(wx, wy, mx, my);
}

// Return true if world cell is free (not unknown, not obstacle, not inscribed)
bool isFree(nav2_costmap_2d::Costmap2D* cm, double wx, double wy) {
  unsigned int mx, my;
  if (!cm->worldToMap(wx, wy, mx, my)) return false;
  unsigned char c = cm->getCost(mx, my);
  using namespace nav2_costmap_2d;
  // be strict: only free; relax to (c < INSCRIBED_INFLATED_OBSTACLE) if needed
  return c == FREE_SPACE;
}

// Nearest *free* cell to (wx, wy). Returns true and fills out (out_wx,out_wy)
bool nearestFreeCell(nav2_costmap_2d::Costmap2D* cm,
                     double wx, double wy,
                     double search_radius,
                     double step,
                     double& out_wx, double& out_wy) {
  // Spiral search in world coords
  const int rings = std::max(1, int(search_radius / step));
  if (isFree(cm, wx, wy)) { out_wx = wx; out_wy = wy; return true; }

  for (int r = 1; r <= rings; ++r) {
    double rad = r * step;
    const int samples = std::max(8, int(2 * M_PI * rad / step));
    for (int i = 0; i < samples; ++i) {
      double a = (2 * M_PI * i) / samples;
      double tx = wx + rad * std::cos(a);
      double ty = wy + rad * std::sin(a);
      if (isFree(cm, tx, ty)) { out_wx = tx; out_wy = ty; return true; }
    }
  }
  return false;
}

// Distance to nearest map boundary (world)
double distanceToBoundary(nav2_costmap_2d::Costmap2D* cm, double wx, double wy) {
  unsigned int mx, my;
  if (!cm->worldToMap(wx, wy, mx, my)) return 0.0;
  double res = cm->getResolution();
  double minx_w = cm->getOriginX();
  double miny_w = cm->getOriginY();
  double maxx_w = minx_w + cm->getSizeInCellsX() * res;
  double maxy_w = miny_w + cm->getSizeInCellsY() * res;
  double dx = std::min(wx - minx_w, maxx_w - wx);
  double dy = std::min(wy - miny_w, maxy_w - wy);
  return std::min(dx, dy);
}

// Clamp point inside map with a margin
void pushInside(nav2_costmap_2d::Costmap2D* cm, double margin, double& wx, double& wy) {
  double res = cm->getResolution();
  double minx_w = cm->getOriginX() + margin;
  double miny_w = cm->getOriginY() + margin;
  double maxx_w = cm->getOriginX() + cm->getSizeInCellsX() * res - margin;
  double maxy_w = cm->getOriginY() + cm->getSizeInCellsY() * res - margin;
  wx = std::min(std::max(wx, minx_w), maxx_w);
  wy = std::min(std::max(wy, miny_w), maxy_w);
}


inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  // ------------- helpers local to this function -------------
  auto* cm = costmap_client_.getCostmap();

  auto isInsideCostmap = [cm](double wx, double wy) -> bool {
    unsigned int mx, my;
    return cm->worldToMap(wx, wy, mx, my);
  };

  auto isFree = [cm](double wx, double wy) -> bool {
    unsigned int mx, my;
    if (!cm->worldToMap(wx, wy, mx, my)) return false;
    unsigned char c = cm->getCost(mx, my);
    using namespace nav2_costmap_2d;
    return c == FREE_SPACE;  // be strict; relax if needed
  };

  auto nearestFreeCell = [cm, isFree](double wx, double wy,
                                      double search_radius,
                                      double step,
                                      double& out_wx, double& out_wy) -> bool {
    if (isFree(wx, wy)) { out_wx = wx; out_wy = wy; return true; }
    const int rings = std::max(1, int(search_radius / step));
    for (int r = 1; r <= rings; ++r) {
      double rad = r * step;
      const int samples = std::max(8, int(2 * M_PI * rad / step));
      for (int i = 0; i < samples; ++i) {
        double a = (2 * M_PI * i) / samples;
        double tx = wx + rad * std::cos(a);
        double ty = wy + rad * std::sin(a);
        if (isFree(tx, ty)) { out_wx = tx; out_wy = ty; return true; }
      }
    }
    return false;
  };

  auto distanceToBoundary = [cm](double wx, double wy) -> double {
    unsigned int mx, my;
    if (!cm->worldToMap(wx, wy, mx, my)) return 0.0;
    double res = cm->getResolution();
    double minx_w = cm->getOriginX();
    double miny_w = cm->getOriginY();
    double maxx_w = minx_w + cm->getSizeInCellsX() * res;
    double maxy_w = miny_w + cm->getSizeInCellsY() * res;
    double dx = std::min(wx - minx_w, maxx_w - wx);
    double dy = std::min(wy - miny_w, maxy_w - wy);
    return std::min(dx, dy);
  };

  auto pushInside = [cm](double margin, double& wx, double& wy) {
    double res = cm->getResolution();
    double minx_w = cm->getOriginX() + margin;
    double miny_w = cm->getOriginY() + margin;
    double maxx_w = cm->getOriginX() + cm->getSizeInCellsX() * res - margin;
    double maxy_w = cm->getOriginY() + cm->getSizeInCellsY() * res - margin;
    wx = std::min(std::max(wx, minx_w), maxx_w);
    wy = std::min(std::max(wy, miny_w), maxy_w);
  };
  // ----------------------------------------------------------

  // find frontiers
  auto pose = costmap_client_.getRobotPose();

  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    stop(true);
    return;
  }

  // optionally publish markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // pick first non-blacklisted frontier
  auto it = std::find_if(
      frontiers.begin(), frontiers.end(),
      [this](const frontier_exploration::Frontier& f) {
        return !goalOnBlacklist(f.centroid);
      });

  if (it == frontiers.end()) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    stop(true);
    return;
  }

  geometry_msgs::msg::Point target_position = it->centroid;

  // progress timeout bookkeeping
  bool same_goal = same_point(prev_goal_, target_position);
  prev_goal_ = target_position;

  if (!same_goal || prev_distance_ > it->min_distance) {
    last_progress_ = this->now();
    prev_distance_ = it->min_distance;
  }
  if ((this->now() - last_progress_ > tf2::durationFromSec(progress_timeout_))
      && !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list (no progress)");
    makePlan();
    return;
  }

  // If still pursuing same goal, do nothing
  if (same_goal) {
    return;
  }

  // ----------------- SAFE GOAL SANITIZATION -----------------
  double wx = target_position.x;
  double wy = target_position.y;

  // 1) clamp inside map bounds by a safety margin (>= inflation radius)
  const double safety_margin = 1.2;   // tune for your robot/map
  pushInside(safety_margin, wx, wy);

  // 2) snap to nearest FREE cell within a small radius
  double snapped_x, snapped_y;
  bool ok = nearestFreeCell(wx, wy, /*search_radius*/1.5, /*step*/0.05,
                            snapped_x, snapped_y);
  if (!ok) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_WARN(logger_, "Frontier near edge/obstacle; blacklisting and replanning");
    makePlan();
    return;
  }

  // 3) reject goals that are still too close to the boundary
  if (distanceToBoundary(snapped_x, snapped_y) < 1.5) {  // meters
    frontier_blacklist_.push_back(target_position);
    RCLCPP_WARN(logger_, "Frontier goal too close to map edge; blacklisting");
    makePlan();
    return;
  }

  target_position.x = snapped_x;
  target_position.y = snapped_y;

  // final safety check: must be inside costmap
  if (!isInsideCostmap(target_position.x, target_position.y)) {
    frontier_blacklist_.push_back(it->centroid);
    RCLCPP_WARN(logger_, "Goal outside costmap; blacklisting and replanning");
    makePlan();
    return;
  }
  // ----------------------------------------------------------

  RCLCPP_DEBUG(logger_, "Sending goal to Nav2");

  // send goal to Nav2
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.0;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback =
      [this, frontier_goal = it->centroid]
      (const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, frontier_goal);
      };

  move_base_client_->async_send_goal(goal, send_goal_options);

  // clear the “resuming” flag only after first call post-resume
  if (resuming_) {
    resuming_ = false;
  }
}


void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  move_base_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal was aborted");
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
