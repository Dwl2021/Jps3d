#include <jps_planner/jps_planner.h>
#include <plan_manage/visualization.h>
#include <ros/ros.h>

#include <Eigen/Core>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jps3d");
  ros::NodeHandle nh, nh_priv("~");

  vec_Vec3f path;
  Visualizer visualizer(nh);
  bool plan_once = false;
  bool success = false;

  Vec3f start, goal;
  nh_priv.param("start_x", start(0), 0.0);
  nh_priv.param("start_y", start(1), 0.0);
  nh_priv.param("start_z", start(2), 1.0);
  nh_priv.param("goal_x", goal(0), 10.0);
  nh_priv.param("goal_y", goal(1), 0.0);
  nh_priv.param("goal_z", goal(2), 1.0);

  /* map util */
  std::shared_ptr<MapUtil<3>> map_util;
  map_util = std::make_shared<MapUtil<3>>();
  map_util->setParam(nh);

  JPSPlanner3D jps3d(false);
  jps3d.setParam(nh);
  jps3d.setMapUtil(map_util);

  ros::Duration(1.0).sleep();
  ros::Rate rate(10);
  while (ros::ok())
  {
    /* visual start and goal points */
    visualizer.visualizeEndpoints(start, goal);

    /* Plan once */
    if (map_util->has_map_() && !plan_once)
    {
      success = jps3d.plan(start, goal);
      ROS_INFO(success ? "\033[1;32mPATH FOUND!\033[0m"
                       : "\033[1;31mPATH NOT FOUND\033[0m");
      /* visual path */
      if (success)
      {
        plan_once = true;
        path = jps3d.getSamplePath();
        visualizer.visualizePath(path);
      }
    }
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}