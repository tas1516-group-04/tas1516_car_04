 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_local_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>

 using std::string;

 #ifndef TAS_LOCAL_PLANNER_CPP
 #define TAS_LOCAL_PLANNER_CPP

 namespace tas_local_planner {

 class LocalPlanner : public nav_core::BaseLocalPlanner
 {
 public:

  LocalPlanner();
  LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseLocalPlanner **/
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  bool isGoalReached();

private:
  bool goalIsReached_;
  bool initialized_;
  tf::TransformListener* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  };
};
#endif