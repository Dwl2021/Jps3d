#include <jps_planner/jps_planner/jps_planner.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>

void visualizePath(const vec_Vec3f& path, ros::Publisher& marker_pub);
void visualizeEndpoints(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                        ros::Publisher& start_pub, ros::Publisher& goal_pub);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinodynamic_astar_node");
  ros::NodeHandle nh, nh_priv("~");
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("path", 1);
  ros::Publisher start_pub = nh.advertise<visualization_msgs::Marker>("start_point", 1);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("end_point", 1);
  vec_Vec3f path;
  bool plan_once = false;
  bool success = true;

  /*
    path searching example
  */
  double start_x, start_y, start_z;
  double start_vel_x, start_vel_y, start_vel_z;
  double start_acc_x, start_acc_y, start_acc_z;
  double goal_x, goal_y, goal_z;
  double goal_vel_x, goal_vel_y, goal_vel_z;
  nh_priv.param("start_x", start_x, 0.0);
  nh_priv.param("start_y", start_y, 0.0);
  nh_priv.param("start_z", start_z, 1.0);
  nh_priv.param("goal_x", goal_x, 10.0);
  nh_priv.param("goal_y", goal_y, 0.0);
  nh_priv.param("goal_z", goal_z, 1.0);

  Vec3f start, goal;
  start << start_x, start_y, start_z;
  goal << goal_x, goal_y, goal_z;

  /* init map util */
  std::shared_ptr<MapUtil<3>> map_util;
  map_util = std::make_shared<MapUtil<3>>();
  map_util->setParam(nh);

  /* init dynamic astar */
  JPSPlanner3D jps3d(false);
  jps3d.setParam(nh);
  jps3d.setMapUtil(map_util);

  ros::Duration(1.0).sleep();
  ros::Rate rate(10);
  while (ros::ok())
  {
    visualizeEndpoints(start, goal, start_pub, goal_pub);
    if (map_util->has_map_()) /* if MAP is not ready */
    {
      if (!plan_once) /* plan for only once */
      {
        ROS_INFO("READY TO PLAN");
        plan_once = true;

        /* main plan function */
        success = jps3d.plan(Vec3f(start_x, start_y, start_z),
                             Vec3f(goal_x, goal_y, goal_z), 1, false);
        if (success)
        {
          path = jps3d.getSamplePath();
          ROS_INFO("\033[1;32mPATH FOUND!\033[0m");
        }
        else
        {
          ROS_INFO("\033[1;31mPATH NOT FOUND\033[0m");
        }
      }
      else /* plan for onces */
      {
        if (success)
        {
          ROS_INFO("PATH PUBLISH!");
          visualizePath(path, path_pub);
        }
      }
    }
    else
    {
      ROS_INFO("MAP NOT READY!");
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

void visualizePath(const vec_Vec3f& path, ros::Publisher& path_pub)
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "path_visualization";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.05;  // 线条的宽度
  line_strip.color.r = 1.0;
  line_strip.color.g = 0.0;
  line_strip.color.b = 0.0;
  line_strip.color.a = 1.0;

  for (const auto& point : path)
  {
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    line_strip.points.push_back(p);
  }

  path_pub.publish(line_strip);
}

void visualizeEndpoints(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                        ros::Publisher& start_pub, ros::Publisher& goal_pub)
{
  visualization_msgs::Marker start_marker, goal_marker;

  start_marker.header.frame_id = goal_marker.header.frame_id = "world";
  start_marker.header.stamp = goal_marker.header.stamp = ros::Time::now();
  start_marker.ns = goal_marker.ns = "endpoints_visualization";
  start_marker.action = goal_marker.action = visualization_msgs::Marker::ADD;
  start_marker.pose.orientation.w = goal_marker.pose.orientation.w = 1.0;

  start_marker.id = 1;
  start_marker.type = visualization_msgs::Marker::SPHERE;
  start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.4;
  start_marker.color.r = 0.0;
  start_marker.color.g = 1.0;
  start_marker.color.b = 0.0;
  start_marker.color.a = 1.0;
  start_marker.pose.position.x = start.x();
  start_marker.pose.position.y = start.y();
  start_marker.pose.position.z = start.z();

  goal_marker.id = 2;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.4;
  goal_marker.color.r = 0.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 1.0;
  goal_marker.color.a = 1.0;
  goal_marker.pose.position.x = goal.x();
  goal_marker.pose.position.y = goal.y();
  goal_marker.pose.position.z = goal.z();

  start_pub.publish(start_marker);
  goal_pub.publish(goal_marker);
}
