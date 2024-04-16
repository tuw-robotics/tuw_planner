#include <cmath>
#include <string>
#include <memory>
#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_graph/search_astar.hpp>
#include <tuw_graph/ros_bridge.hpp>
#include "nav2_util/node_utils.hpp"

#include "tuw_planner_graph/graph_astar.hpp"

using std::placeholders::_1;
namespace tuw_planner_graph
{

  void GraphAStar::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
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

    pub_path_debug_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("path_debug", 10);

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

  nav_msgs::msg::Path &GraphAStar::plan_graph_astar(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      nav_msgs::msg::Path &global_path)
  {

    Eigen::Vector3d start_map(start.pose.position.x, start.pose.position.y, 0);
    Eigen::Vector3d goal_map(goal.pose.position.x, goal.pose.position.y, 0);
    Eigen::Transform<double, 3, Eigen::Affine> tf_map_2_graph = graph_->origin().inverse();
    Eigen::Transform<double, 3, Eigen::Affine> tf_graph_2_map = graph_->origin();

    RCLCPP_INFO(
        node_->get_logger(), "graph_offset <%4.3f, %4.3f, %4.3f>",
        graph_->origin().translation().x(), graph_->origin().translation().y(), graph_->origin().translation().z());

    Eigen::Vector3d start_graph = tf_map_2_graph * start_map;
    Eigen::Vector3d goal_graph = tf_map_2_graph * goal_map;

    tuw_graph::Node *node_start = graph_->closest_node(start_graph);
    tuw_graph::Node *node_goal = graph_->closest_node(goal_graph);

    tuw_graph::SearchAStart astar(*graph_);
    astar.reset();
    std::vector<tuw_graph::Node *> path;
    path = astar.start_processing(node_start, node_goal, false);
    std::reverse(path.begin(), path.end());

    if (true)
    {
      /// Debug message with path
      std::stringstream ss;
      geometry_msgs::msg::PoseArray path_debug;
      path_debug.header.stamp = node_->now();
      path_debug.header.frame_id = global_frame_;
      for (size_t i = 0; i < path.size(); i++)
      {
        path_debug.poses.push_back(tuw_graph::to_msg(tf_graph_2_map * path[i]->pose));
        ss << path[i]->id << (i < path.size() - 1 ? " > " : "");
      }
      pub_path_debug_->publish(path_debug);
      RCLCPP_INFO(node_->get_logger(), "nodes: %s", ss.str().c_str());
    }

    if (!path.empty())
    {
      global_path.poses.clear();
      global_path.header.stamp = node_->now();
      global_path.header.frame_id = global_frame_;

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;

      // Insert a pose shortly before the start pose
      const double next_waypoint_distance = 0.0;
      Eigen::Vector3d next_pose;

      if (path.size() >= 2) {
        tuw_graph::Node *next = path[1];
        next_pose = tf_graph_2_map * next->pose.translation();
      } else {
        tuw_graph::Node *next = path[0];
        next_pose = tf_graph_2_map * next->pose.translation();
      }

      double distance = (next_pose - start_map).norm();

      if (distance > next_waypoint_distance)
      {
        Eigen::Vector3d next_waypoint = start_map + next_waypoint_distance * (next_pose - start_map).normalized();
        pose.pose.position.x = next_waypoint.x();
        pose.pose.position.y = next_waypoint.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        global_path.poses.push_back(pose);
      }   

      for (size_t i = 1; i < path.size(); i++)
      {
        tuw_graph::Node *node = path[i];
        Eigen::Vector3d node_map = tf_graph_2_map * node->pose.translation();
        pose.pose.position.x = node_map.x();
        pose.pose.position.y = node_map.y();
        pose.pose.position.z = 0.0;
        global_path.poses.push_back(pose);
      }
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Path empty!!!.");
    }
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(std::move(goal_pose));
    for (size_t i = 0; i < global_path.poses.size() - 1; i++)
    {
      geometry_msgs::msg::Point p0 = global_path.poses[i].pose.position;
      geometry_msgs::msg::Point p1 = global_path.poses[i + 1].pose.position;
      double yaw_angle = atan2(p1.y - p0.y, p1.x - p0.x);
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, yaw_angle); // Roll and pitch are set to zero
      tf2::convert(quaternion, global_path.poses[i].pose.orientation);
    }

    return global_path;
  }
  nav_msgs::msg::Path GraphAStar::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path global_path;

    RCLCPP_INFO(
        node_->get_logger(), "start <%4.3f, %4.3f, %4.3f> --> goal  <%4.3f, %4.3f, %4.3f> ",
        start.pose.position.x, start.pose.position.y, start.pose.position.z,
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

    if (msg_graph_)
    {

      plan_graph_astar(start, goal, global_path);
    }
    else
    {
      RCLCPP_WARN(
          node_->get_logger(), "Panner can't start! No graph for planning received.");
    }

    return global_path;
  }

  void GraphAStar::callback_graph(const tuw_graph_msgs::msg::Graph::SharedPtr msg)
  {
    msg_graph_ = std::make_shared<tuw_graph_msgs::msg::Graph>();
    *msg_graph_ = *msg;

    std::shared_ptr<tuw_graph::Graph> graph = std::make_shared<tuw_graph::Graph>();
    tuw_graph::from_msg(*msg_graph_, *graph);

    graph_ = graph;

    RCLCPP_INFO(
        node_->get_logger(), "Graph received %zu edges %zu nodes", msg_graph_->edges.size(), msg_graph_->nodes.size());
  }

} // namespace tuw_planner_graph

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tuw_planner_graph::GraphAStar, nav2_core::GlobalPlanner)
