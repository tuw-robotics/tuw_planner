# tuw_planner_straight
## straight_line_planner
This planner is a copy from the [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html) and the [example](https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_straightline_planner).
### usage

```bash
ros2 launch tuw_nav2 nav2_minimal_launch.py planner_server_yaml:=empty # run nav2 without planner_server
ros2 run nav2_planner planner_server --ros-args --params-file $WS/src/tuw_planner/tuw_planner_straight/config/nav2/planner_tuw_graph.yaml 
```
## graph_planner
This planner plans a path based on a predifined graph
### usage

```bash
ros2 launch tuw_nav2 nav2_minimal_launch.py planner_server_yaml:=empty # run nav2 without planner_server
ros2 run nav2_planner planner_server --ros-args --params-file $WS/src/tuw_planner/tuw_planner_straight/config/nav2/planner_tuw_graph.yaml 

```