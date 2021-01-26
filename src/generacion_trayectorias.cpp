#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>
#include <ros/package.h>
#include <fstream>
#include <algorithm>
#include <list>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <visualization_msgs/MarkerArray.h>
#include <tutorial_utilities/visualization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}


descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose,M_PI/2.0 , AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}

bool waitForSubscribers(ros::Publisher &pub, ros::Duration timeout);
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
namespace rvt = rviz_visual_tools;



int main(int argc, char** argv)
{

  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner (1);
  spinner.start();
  ros::Rate loop_rate(10);
  rvt::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rvt::RvizVisualTools("world","/rviz_visual_tools"));
  visual_tools_->loadMarkerPub();  // create publisher before waiting
  ros::Duration(5.0).sleep();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();


  visualization_msgs::MarkerArray ma;
  descartes_core::RobotModelPtr model (new descartes_moveit::IkFastMoveitStateAdapter());
  const std::string robot_description = "robot_description";
  const std::string group_name = "manipulator";
  const std::string world_frame = "world";
  const std::string tcp_frame = "tool1";


 

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }
  model->setCheckCollisions(true); // Let's turn on collision checking.

  //Partes dañadas
  // suma_partemala_central.csv
  // suma_partemala_infder.csv
  // suma_partemala_sup_izq.csv
 
  
  const static double time_between_points = 0.6;
  std::vector<Eigen::Affine3d> poses;
  EigenSTL::vector_Affine3d PATH;
  EigenSTL::vector_Isometry3d path; // results
  std::ifstream indata; // input file
  std::string filename = ros::package::getPath("tutorial2_support") + "/config/volumen_CADvsZED_malo3.pcd";
  indata.open(filename);
  std::string line;
  int lnum = 0;

 while (std::getline(indata, line)) 
{
  ++lnum;
  if (lnum < 11)
  continue;
  std::list<std::string> list;
  std::string buffer;
  while (std::getline(indata, buffer)) 
      {
        list.push_front(buffer);
      }
      std::stringstream lineStream ;
      std::copy(list.begin(), list.end(),std::ostream_iterator<std::string>(lineStream,"\n"));
      std::string line2;

      while(std::getline(lineStream,line2))
    	    {
		std::stringstream lineStream1(line2);
      		std::string  cell;
      		Eigen::Matrix<double, 4, 1> xyzijk;
      		int i = -1;
	        while (std::getline(lineStream1, cell, ' '))
     		  {
        		++i;
        		if (i == -1)
          		continue;
			xyzijk(i) = std::stod(cell);
			
                  }
       Eigen::Vector3d pos = xyzijk.head<3>();
       pos = pos ;  
       Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
       pose.matrix().col(3).head<3>() = pos;
       pose *= Eigen::AngleAxisd(M_PI/-2, Eigen::Vector3d::UnitX()); 
       path.push_back(pose);
       poses.push_back(pose);
       PATH.push_back(pose);	
  }
}
  indata.close();  

  

  Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
  pattern_origin.translation() = Eigen::Vector3d(-0.001, 0.019, -0.009);
  
  std::vector<descartes_core::TrajectoryPtPtr> points;
  for (const auto& pose : path)
  {
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_between_points);

    points.push_back(pt);
  }

  visual_tools_->publishPath(PATH, rviz_visual_tools::YELLOW,0.0001,"Path");
  visual_tools_->trigger();

  ma = tutorial_utilities::createMarkerArray(poses);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  //Wait for subscriber and publish the markerArray once the subscriber is found.
  ROS_INFO("Waiting for marker subscribers.");
  if (waitForSubscribers(vis_pub, ros::Duration(2.0)))
  {
    ROS_INFO("Subscriber found, publishing markers.");
    vis_pub.publish(ma);
    ros::spinOnce();
    loop_rate.sleep();
  }
  else
  {
    ROS_ERROR("No subscribers connected, markers not published");
  }

  descartes_planner::DensePlanner planner;
  if (!planner.initialize(model))
  {
    ROS_ERROR("Failed to initialize planner");
    return -2;
  }


  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -3;
  }

  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -4;
  }

  
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);


  trajectory_msgs::JointTrajectory joint_solution;
  joint_solution.joint_names = names;


  const static double default_joint_vel = 0.2; // rad/s
  if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
  {
    ROS_ERROR("Unable to convert Descartes trajectory to joint points");
    return -5;
  }

 
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -6;
  }

  ROS_INFO("Done!");
 	 
  return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
 
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  
  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool waitForSubscribers(ros::Publisher &pub, ros::Duration timeout)
{
  if (pub.getNumSubscribers() > 0)
    return true;
  ros::Time start = ros::Time::now();
  ros::Rate waitTime(0.5);
  while (ros::Time::now() - start < timeout)
  {
    waitTime.sleep();
    if (pub.getNumSubscribers() > 0)
      break;
  }
  return pub.getNumSubscribers() > 0;
}
