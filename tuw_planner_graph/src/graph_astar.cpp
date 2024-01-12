#include <cmath>
#include <string>
#include <memory>
#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_graph/search_astar.hpp>
#include "nav2_util/node_utils.hpp"

#include "tuw_planner_graph/graph_astar.hpp"

using std::placeholders::_1;
namespace tuw_planner_graph
{

void GraphAStar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void GraphAStar::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnAStarGraphPlanner",
    name_.c_str());
}

void GraphAStar::activate()
{

  sub_graph_ = node_->create_subscription<tuw_graph_msgs::msg::Graph>(
    "/graph", 10, std::bind(&GraphAStar::callback_graph, this, _1));

  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnAStarGraphPlanner",
    name_.c_str());
}

void GraphAStar::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnAStarGraphPlanner",
    name_.c_str());
}

nav_msgs::msg::Path &GraphAStar::plan_straight_line(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path& global_path){

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);
  return global_path;
}

nav_msgs::msg::Path &GraphAStar::plan_graph_astar(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path& global_path){


  tuw_graph::Node* node_start = graph_->closest_node(Eigen::Vector3d(start.pose.position.x, start.pose.position.y, start.pose.position.z));  
  RCLCPP_INFO(
    node_->get_logger(), "node_start %3zu, <%4.3f, %4.3f, %4.3f> ", node_start->id, node_start->pose.translation().x(), node_start->pose.translation().y(), node_start->pose.translation().z());
  
  tuw_graph::Node* node_goal  = graph_->closest_node(Eigen::Vector3d( goal.pose.position.x,  goal.pose.position.y,  goal.pose.position.z));
  RCLCPP_INFO(
    node_->get_logger(), "node_goal  %3zu, <%4.3f, %4.3f, %4.3f> ", node_goal->id, node_goal->pose.translation().x(), node_goal->pose.translation().y(), node_goal->pose.translation().z());
 
  tuw_graph::SearchAStart astar(*graph_);
  astar.reset();
  std::vector<tuw_graph::Node*> path;
  path = astar.start_processing(node_start, node_goal, false);


  for (tuw_graph::Node* node: path) {
    RCLCPP_INFO(
      node_->get_logger(), "path  %3zu, <%4.3f, %4.3f, %4.3f> ", node->id, node->pose.translation().x(), node->pose.translation().y(), node->pose.translation().z());
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = node->pose.translation().x();
    pose.pose.position.y = node->pose.translation().y();
    pose.pose.position.z = node->pose.translation().z();
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(std::move(pose));
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);
  return global_path;
}
nav_msgs::msg::Path GraphAStar::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  RCLCPP_INFO(
    node_->get_logger(), "start <%4.3f, %4.3f, %4.3f> ", start.pose.position.x, start.pose.position.y, start.pose.position.z);
  RCLCPP_INFO(
    node_->get_logger(), "goal  <%4.3f, %4.3f, %4.3f> ", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

  if(graph_){

    plan_graph_astar(start, goal, global_path);
  } else {
    plan_straight_line(start, goal, global_path);
  }

  return global_path;
}

void GraphAStar::callback_graph(const tuw_graph_msgs::msg::Graph::SharedPtr msg){
  graph_ = std::make_shared<tuw_graph::Graph>();
  tuw_graph::from_msg(*msg, *graph_);
  RCLCPP_INFO(
    node_->get_logger(), "Graph received %zu edges %zu nodes", graph_->edges().size(), graph_->nodes().size());
}

}  // namespace tuw_planner_graph

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tuw_planner_graph::GraphAStar, nav2_core::GlobalPlanner)
