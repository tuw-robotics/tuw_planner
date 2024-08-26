#include <cmath>
#include <string>
#include <memory>
#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_graph/search_astar.hpp>
#include <tuw_graph/search_dijkstar.hpp>
#include "tuw_eigen/eigen.hpp"
#include <tuw_graph/ros_bridge.hpp>
#include "nav2_util/node_utils.hpp"

#include "tuw_planner_graph/graph_planner.hpp"

using std::placeholders::_1;
namespace tuw_planner_graph
{

  void GraphPlanner::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    std::string name_algorithm;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".algorithm", rclcpp::ParameterValue("astar"));
    node_->get_parameter(name_ + ".algorithm", name_algorithm);
    alogrithm_ = tuw_graph::Search::name_to_algorithm(name_algorithm);

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".drive_on_step_size_", rclcpp::ParameterValue(0.2));
    node_->get_parameter(name_ + ".drive_on_step_size_", drive_on_step_size_);
  }

  void GraphPlanner::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnAStarGraphPlanner",
        name_.c_str());
  }

  void GraphPlanner::activate()
  {

    sub_graph_ = node_->create_subscription<tuw_graph_msgs::msg::Graph>(
        "/graph", 10, std::bind(&GraphPlanner::callback_graph, this, _1));

    pub_path_debug_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("nodes_on_path", 10);

    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnAStarGraphPlanner",
        name_.c_str());
  }

  void GraphPlanner::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnAStarGraphPlanner",
        name_.c_str());
  }

  void GraphPlanner::convert_path(
      const std::vector<tuw_graph::Node *> &graph_path,
      nav_msgs::msg::Path &global_path)
  {

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;

    for (size_t i = 0; i < graph_path.size(); i++)
    {
      tuw_graph::Node *node = graph_path[i];
      Eigen::Vector3d node_map = tf_graph_2_map_ * node->pose.translation();
      pose.pose.position.x = node_map.x();
      pose.pose.position.y = node_map.y();
      pose.pose.position.z = 0.0;
      global_path.poses.push_back(pose);
    }
  }

  void GraphPlanner::debug_publish_node_path(const std::vector<tuw_graph::Node *> node_path)
  {
    std::stringstream ss;
    geometry_msgs::msg::PoseArray path_debug;
    path_debug.header.stamp = node_->now();
    path_debug.header.frame_id = global_frame_;
    for (size_t i = 0; i < node_path.size(); i++)
    {
      path_debug.poses.push_back(tuw_graph::to_msg(tf_graph_2_map_ * node_path[i]->pose));
      ss << node_path[i]->id << (i < node_path.size() - 1 ? " > " : "");
    }
    pub_path_debug_->publish(path_debug);
    RCLCPP_INFO(node_->get_logger(), "nodes: %s", ss.str().c_str());
  }

  nav_msgs::msg::Path &GraphPlanner::compute_orientation(
      nav_msgs::msg::Path &path)
  {
    if (path.poses.size() < 2) return path;

    tuw_eigen::Pose3D pose;
    tuw_eigen::Point3D position;
    tuw_eigen::Point3D point_ahead;
    for(size_t i = 0; i < path.poses.size()-1; i++){
      position.copy_from(path.poses[i].pose.position);
      point_ahead.copy_from(path.poses[i+1].pose.position);
      pose.set(position, point_ahead);
      pose.copy_to(path.poses[i].pose);
    }
    return path;
  }

  nav_msgs::msg::Path &GraphPlanner::drive_on(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      nav_msgs::msg::Path &path)
  {
    // only modify path if it has at leas 3 nodes
    size_t n = path.poses.size();
    if (n < 2)
      return path;
    tuw_eigen::Point2D p0(path.poses[0].pose.position);
    tuw_eigen::Point2D p1(path.poses[1].pose.position);
    tuw_eigen::Point2D pr(start.pose.position);
    tuw_eigen::Line2D l(p0, p1);
    tuw_eigen::Point2D pi = l.pointOnLine(pr);
    pi.copy_to_clear(path.poses[0].pose.position); /// replace frist waypoint
    return compute_orientation(path);
  }

  nav_msgs::msg::Path &GraphPlanner::drive_off(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      nav_msgs::msg::Path &path)
  {
    // only modify path if it has at leas 3 nodes
    size_t n = path.poses.size();
    if (n < 2)
      return path;
    tuw_eigen::Point2D p0(path.poses[n-1].pose.position);
    tuw_eigen::Point2D p1(path.poses[n-2].pose.position);
    tuw_eigen::Point2D pr(goal.pose.position);
    tuw_eigen::Line2D l(p0, p1);
    tuw_eigen::Point2D pi = l.pointOnLine(pr);
    pi.copy_to_clear(path.poses[n-1].pose.position); /// replace last waypoint
    path.poses.push_back(goal);                      /// add goal
    path.poses[n].header = path.header;              /// add header info
    return compute_orientation(path);
  }


  nav_msgs::msg::Path &GraphPlanner::add_waypoints(
      nav_msgs::msg::Path &path)
  {
    std::vector<geometry_msgs::msg::PoseStamped> new_path;
    for (size_t i = 0; i < path.poses.size()-1; ++i)
    {
      tuw_eigen::Point2D p0(path.poses[i].pose.position);
      tuw_eigen::Point2D p1(path.poses[i+1].pose.position);
      tuw_eigen::LineSegment2D ls(p0, p1);
      Eigen::Vector2d v = ls.direction();



      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = 0.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;


      int total_number_of_loop = ls.length() / drive_on_step_size_;
      for (int i = 0; i < total_number_of_loop; ++i)
      {
        tuw_eigen::Point2D p = p0 + v * drive_on_step_size_ * i;
        p.copy_to_clear(pose.pose.position);
        new_path.push_back(pose);
      }
    }
    auto goal = path.poses.back();
    path.poses = new_path;
    path.poses.push_back(goal);
    return path;
  }

  nav_msgs::msg::Path &GraphPlanner::start_graph_serach(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      nav_msgs::msg::Path &global_path)
  {

    // tanslate start and goal into graph frame
    Eigen::Vector3d graph_start = tf_map_2_graph_ * Eigen::Vector3d(start.pose.position.x, start.pose.position.y, 0);
    Eigen::Vector3d graph_goal = tf_map_2_graph_ * Eigen::Vector3d(goal.pose.position.x, goal.pose.position.y, 0);

    // find the closest graph nodes
    tuw_graph::Node *node_start = graph_->closest_node(graph_start);
    tuw_graph::Node *node_goal = graph_->closest_node(graph_goal);

    // init search algorithm
    if (search_algorithm_ == NULL || (search_algorithm_->algorithm() != alogrithm_))
    {
      switch (alogrithm_)
      {
      case tuw_graph::SearchAlgorithm::AStar:
        search_algorithm_ = std::make_shared<tuw_graph::SearchAStart>(graph_);
        break;
      case tuw_graph::SearchAlgorithm::DijkStar:
        search_algorithm_ = std::make_shared<tuw_graph::SearchDijkStar>(graph_);
        break;
      default:
        RCLCPP_WARN(node_->get_logger(), "Unkown search algorithm, using DijkStar");
        search_algorithm_ = std::make_shared<tuw_graph::SearchDijkStar>(graph_);
      }
    }
    else
    {
      search_algorithm_->reset();
    }
    RCLCPP_INFO(node_->get_logger(), "Using: %s", search_algorithm_->info().c_str());

    std::vector<tuw_graph::Node *> node_path;
    node_path = search_algorithm_->start_processing(node_start, node_goal, false);
    std::reverse(node_path.begin(), node_path.end());

    debug_publish_node_path(node_path);

    if (!node_path.empty())
    {
      convert_path(node_path, global_path);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Path empty!!!.");
    }
    for (size_t i = 0; i < global_path.poses.size() - 1; i++)
    {
      geometry_msgs::msg::Point p0 = global_path.poses[i].pose.position;
      geometry_msgs::msg::Point p1 = global_path.poses[i + 1].pose.position;
      double yaw_angle = atan2(p1.y - p0.y, p1.x - p0.x);
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, yaw_angle); // Roll and pitch are set to zero
      tf2::convert(quaternion, global_path.poses[i].pose.orientation);
    }

    drive_on(start, goal, global_path);
    drive_off(start, goal, global_path);
    //global_path.poses.push_back(goal);
    add_waypoints(global_path);
    compute_orientation(global_path);
    //global_path.poses.push_back(goal);
    return global_path;
  }

  /**
   * Planning request by the NAV2
   * Starts search only if a graph was allready received
   */
  nav_msgs::msg::Path GraphPlanner::createPlan(
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
      // compute translation from map to graph and back
      tf_map_2_graph_ = graph_->origin().inverse();
      tf_graph_2_map_ = graph_->origin();
      start_graph_serach(start, goal, global_path);
    }
    else
    {
      RCLCPP_WARN(
          node_->get_logger(), "Panner can't start! No graph for planning received.");
    }

    return global_path;
  }

  /**
   * Callback on graph messages
   * only updates the graph on changes
   */
  void GraphPlanner::callback_graph(const tuw_graph_msgs::msg::Graph::SharedPtr msg)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "Graph received %zu edges %zu nodes",
        msg->edges.size(), msg->nodes.size());

    /// The graph should be only updated on changes
    if (!msg_graph_ || (*msg_graph_ != *msg))
    {
      msg_graph_ = std::make_shared<tuw_graph_msgs::msg::Graph>();
      *msg_graph_ = *msg;

      std::shared_ptr<tuw_graph::Graph> graph = std::make_shared<tuw_graph::Graph>();
      tuw_graph::from_msg(*msg_graph_, *graph);
      graph_ = graph;
      RCLCPP_INFO(node_->get_logger(), "Graph updated!");
    }
  }

} // namespace tuw_planner_graph

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tuw_planner_graph::GraphPlanner, nav2_core::GlobalPlanner)
