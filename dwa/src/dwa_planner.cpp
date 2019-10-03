#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

const double INF                        = std::numeric_limits<double>::infinity();
const double EPSILON                    = std::numeric_limits<double>::epsilon();
const double PI                         = 3.141592653589793238463;

const double STEP_LINEAR_VELOCITY       = 0.01;
const double STEP_ANGULAR_VELOCITY      = 5/180.0 * PI;
const double MAX_LINEAR_VELOCITY        = 0.5;
const double MAX_ANGULAR_VELOCITY       = PI*50.0/180.0;
const double MAX_LINEAR_ACCELERATION    = 0.5;
const double MAX_ANGULAR_ACCELERATION   = PI*50.0/180.0;

const double DT                         = 0.1;

const double WEIGHT_DISTPLAN            = 0.1;
const double WEIGHT_HEADING             = 0.4;
const double WEIGHT_VELOCITY            = 0.0;

const int N_SAMPLES_TRAJ                = 30;
const int N_STEPS_AHEAD                 = 30;

const double FINE_POS_TOLERANCE         = 0.2;
const double XY_GOAL_TOLERANCE          = 0.01;
const double YAW_GOAL_TOLERANCE         = 10*PI/180;

const double ROBOT_LENGTH               = 0.665;
const double ROBOT_WIDTH                = 0.445;
bool ROT_STARTED                        = false;

ros::Time t,oldT;

using namespace std;

class DWAPlanner: public nav_core::BaseLocalPlanner
{

  bool initialized;
  tf2_ros::Buffer* tf;
  costmap_2d::Costmap2DROS* costmap_ros;
  costmap_2d::Costmap2D* costmap;
  geometry_msgs::PoseStamped currentPose;
  geometry_msgs::PoseStamped nearestPlanPose;
  geometry_msgs::PoseStamped errGoalPose;
  vector<geometry_msgs::PoseStamped> globalPlan;
  ros::Subscriber odometry_sub;
  ros::Publisher guiPathPub;
  ros::Publisher waypointPub;
  nav_msgs::Path gui_path;
  ros::NodeHandle nh;
  ros::NodeHandle oh;

  double currentVx;
  double currentVy;
  double currentWz;

  double sumDistPlan = 0;
  double sumHeading = 0;
  double sumVelocity = 0;

  struct ScoringHelper
  {
    double vx;
    double vy;
    double wz;
    double distPlan;
    double heading;
    double velocity;
    double score;
  };
public:
  DWAPlanner(): initialized(false), nh("~/DWAPlanner") {}

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if(initialized)
    {
      ROS_WARN("DW local planner: Already initilized");
      return;
    }
    DWAPlanner::tf = tf;
    DWAPlanner::costmap_ros = costmap_ros;
    costmap = costmap_ros->getCostmap();
    costmap_ros->getRobotPose(currentPose);
    odometry_sub = oh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&DWAPlanner::odomCallback, this, _1));
    guiPathPub = nh.advertise<nav_msgs::Path>("plan", 1);

    initialized = true;
    ROS_INFO("Width is: %d, Height is: %d", costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  }

  // get odometry
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
    currentVx = odom->twist.twist.linear.x;
    currentVy = odom->twist.twist.linear.y;
    currentWz = odom->twist.twist.angular.z;
  }

  // get velocities
  const double getLinearVelocityX() const
  {
    return currentVx;
  }

  const double getLinearVelocityY() const
  {
    return currentVy;
  }

  const double getAngularVelocityZ() const
  {
    return currentWz;
  }

  double clamp(double value, double minValue, double maxValue)
  {
    if(value < minValue)
    return minValue;
    if(value > maxValue)
    return maxValue;
    return value;
  }

  // Check if there is an impact around the current location (each cell has the size specified in "resolution" in costmap_local_params.yaml)
  bool isImpactAroundLocation(geometry_msgs::PoseStamped currentPoseTemp)
  {
    unsigned int x, y, indX,indY;
    double yaw = tf::getYaw(currentPoseTemp.pose.orientation);
    costmap->worldToMap(currentPoseTemp.pose.position.x, currentPoseTemp.pose.position.y, x, y);

    double resolution = costmap->getResolution();

    for(int i = -(int)(ROBOT_LENGTH / resolution) ; i <= (int)(ROBOT_LENGTH / resolution) ; i++)
    {
      for(int j = -(int)(ROBOT_WIDTH / resolution) ; j <= (int)(ROBOT_WIDTH / resolution); j++)
      {
        indX = (unsigned int)clamp((x + (i*cos(yaw) - j*sin(yaw))),0,costmap->getSizeInCellsX());
        indY = (unsigned int)clamp((y + (i*sin(yaw) + j*cos(yaw))),0,costmap->getSizeInCellsY());
        if(costmap->getCost(indX, indY) != 0)
        {
          return true;
        }
      }
    }
    return false;
  }

  // get simulated robot pos
  geometry_msgs::PoseStamped getNewRobotPose(geometry_msgs::PoseStamped robotPose, double velocityVx, double velocityVy, double velocityWz)
  {
    geometry_msgs::PoseStamped newRobotPose;
    double robotYaw = tf::getYaw(robotPose.pose.orientation);
    newRobotPose.pose.position.x = robotPose.pose.position.x + (velocityVx*cos(robotYaw) - velocityVy*sin(robotYaw))*DT;
    newRobotPose.pose.position.y = robotPose.pose.position.y + (velocityVx*sin(robotYaw) + velocityVy*cos(robotYaw))*DT;
    newRobotPose.pose.position.z = 0;
    double newRobotYaw = robotYaw + velocityWz * DT;
    newRobotPose.pose.orientation = tf::createQuaternionMsgFromYaw(newRobotYaw);
    return newRobotPose;
  }

  // get robot goal obtained by calculating the nearest plan poitn and projecting it
  // N_STEPS_AHEAD points forward
  geometry_msgs::PoseStamped getNewRobotGoal(geometry_msgs::PoseStamped robotPose)
  {
    int closestPointInPath = 0;
    double shortestDistance = INF;

    for(int i = 0; i < globalPlan.size(); i++)
    {
      double dx = globalPlan[i].pose.position.x - robotPose.pose.position.x;
      double dy = globalPlan[i].pose.position.y - robotPose.pose.position.y;
      double newDistance = sqrt(dx*dx + dy*dy);
      if(newDistance < shortestDistance)
      {
        shortestDistance = newDistance;
        closestPointInPath = i;
      }
    }
    if(closestPointInPath + N_STEPS_AHEAD > globalPlan.size()-1)
    {
      return globalPlan.back();
    }
    return globalPlan[closestPointInPath + N_STEPS_AHEAD];
  }

  // set plan
  bool setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    globalPlan = orig_global_plan;
    gui_path.poses.clear();
    return true;
  }

  // get heaading to goal - the value is inverted as tha cost function tries to maximize the returned value
  double getHeading(geometry_msgs::PoseStamped robotPose)
  {
    double angleToGoal = atan2(nearestPlanPose.pose.position.y - robotPose.pose.position.y, nearestPlanPose.pose.position.x - robotPose.pose.position.x );
    angleToGoal = angles::normalize_angle((angles::normalize_angle_positive(angleToGoal) - angles::normalize_angle_positive(tf::getYaw(robotPose.pose.orientation))));
    return 180 -  abs(angleToGoal)/PI * 180;
  }

  // get difference between two poses
  geometry_msgs::PoseStamped getPoseDifference(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
  {
    geometry_msgs::PoseStamped diffPose;
    diffPose.pose.position.x = pose1.pose.position.x - pose2.pose.position.x;
    diffPose.pose.position.y = pose1.pose.position.y - pose2.pose.position.y;
    double tempYaw = tf::getYaw(pose1.pose.orientation) - tf::getYaw(pose2.pose.orientation);
    diffPose.pose.orientation = tf::createQuaternionMsgFromYaw(tempYaw);
    return diffPose;
  }

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    t = ros::Time::now() ;
    ros::Duration delta_time = (t-oldT);
    oldT = t;

    if(!initialized)
    {
      ROS_WARN("DW local planner: Must be initilized first");
      return false;
    }
    if(costmap_ros->getRobotPose(currentPose) == false)
    {
      ROS_WARN("DW local planner: Failed to get current local pose");
      return false;
    }
    vector<ScoringHelper> scoringhelper;
    nearestPlanPose = getNewRobotGoal(currentPose);

    costmap = costmap_ros->getCostmap();
    costmap_ros->getRobotPose(currentPose);

    double robotVx = getLinearVelocityX();
    double robotVy = getLinearVelocityY();
    double robotWz = getAngularVelocityZ();
    // get velocity ranges
    double minVx = clamp(robotVx - DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double maxVx = clamp(robotVx + DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double minVy = clamp(robotVy - DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double maxVy = clamp(robotVy + DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double minWz = clamp(robotWz - DT*MAX_ANGULAR_ACCELERATION, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    double maxWz = clamp(robotWz + DT*MAX_ANGULAR_ACCELERATION, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    if (hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) > FINE_POS_TOLERANCE)
    {
      // get velocity commands using the dwa algorithm
      sumDistPlan = 0;
      sumHeading = 0;
      sumVelocity = 0;
      getCommandsDWA(scoringhelper, minVx, maxVx, minVy, maxVy, 0.0, 0.0);
      getCommandsDWA(scoringhelper, minVx, maxVx, 0.0, 0.0, minWz, maxWz);
      //getCommandsDWA(scoringhelper, 0.0, 0.0, minVy, maxVy, minWz, maxWz);

      // find velocity commands that maximizes the score
      int bestScoreIndex = 0;
      for(int i = 0; i < scoringhelper.size(); i++)
      {
        double normalized_distPlan  = scoringhelper[i].distPlan / sumDistPlan;
        double normalized_heading   = scoringhelper[i].heading / sumHeading;
        double normalized_velocity  = scoringhelper[i].velocity / sumVelocity;

        scoringhelper[i].score =  WEIGHT_DISTPLAN / normalized_distPlan  + WEIGHT_HEADING * normalized_heading + WEIGHT_VELOCITY * normalized_velocity;
        if(scoringhelper[i].score > scoringhelper[bestScoreIndex].score)
        {
          bestScoreIndex = i;
        }
      }
      if(scoringhelper.size() == 0)
      {
        cout << "Failed finding velocity comands" << endl;
        return false;
      }
      else
      {
        cmd_vel.linear.x = scoringhelper[bestScoreIndex].vx;
        cmd_vel.linear.y = scoringhelper[bestScoreIndex].vy;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = scoringhelper[bestScoreIndex].wz;
      }
    }
    else
    {
      // get PID velocity commands for final positioning
      if ((hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) > XY_GOAL_TOLERANCE) && !ROT_STARTED)
      {
        getCommandsPID(cmd_vel, errGoalPose.pose.position.x, errGoalPose.pose.position.y, 0);
      }
      else
      {
        // get PID velocity commands for final rotation
        getCommandsPID(cmd_vel, 0,0, tf::getYaw(errGoalPose.pose.orientation));
        // if the final rotation is initiatd then complete it - to avoid toggling between different states
        if(fabs(tf::getYaw(errGoalPose.pose.orientation)) <= YAW_GOAL_TOLERANCE)
        {
          ROT_STARTED = false;
        }
        else
        {
          ROT_STARTED = true;
        }
      }
    }
    cout <<  delta_time  << " "<< cmd_vel.linear.x  << " " << cmd_vel.linear.y  << " " << cmd_vel.angular.z  << endl;
    return true;
  }

  // dwa algorithm - simulate trajectories usign N_SAMPLES_TRAJ samples
  bool getCommandsDWA(vector<ScoringHelper> &scoringhelper,double minVx,double maxVx,double minVy,double maxVy,double minWz,double maxWz)
  {
    for(double vx = minVx; vx <= maxVx; vx += STEP_LINEAR_VELOCITY)
    {
      for(double vy = minVy; vy <= maxVy; vy += STEP_LINEAR_VELOCITY)
      {
        for(double wz = minWz; wz <= maxWz; wz += STEP_ANGULAR_VELOCITY)
        {

          bool obstacleFound = false;
          geometry_msgs::PoseStamped currentPoseTemp = currentPose;

          for(int i = 0; i < N_SAMPLES_TRAJ; i++)
          {
            double oldX = currentPoseTemp.pose.position.x;
            double oldY = currentPoseTemp.pose.position.y;

            currentPoseTemp = getNewRobotPose(currentPoseTemp, vx, vy, wz);

            // if obstacl is found stop and do not use the velocity commands that would lead to that position
            if(isImpactAroundLocation(currentPoseTemp))
            {
              obstacleFound = true;
              break;
            }
          }
          if (!obstacleFound)
          {
            geometry_msgs::PoseStamped diffPlan = getPoseDifference(nearestPlanPose,currentPoseTemp);
            double distPlan = hypot(diffPlan.pose.position.x,diffPlan.pose.position.y);
            double heading = getHeading(currentPoseTemp);
            double velocity = hypot(vx,vy);

            sumDistPlan += distPlan;
            sumHeading += heading;
            sumVelocity += velocity;

            ScoringHelper sh;
            sh.vx = vx;
            sh.vy = vy;
            sh.wz = wz;
            sh.distPlan = distPlan;
            sh.heading = heading;
            sh.velocity = velocity;
            scoringhelper.push_back(sh);
          }
        }
      }
    }
    return true;
  }

  //  pid algorithm - used for final rotation
  bool getCommandsPID(geometry_msgs::Twist &cmd_vel, double err_x, double err_y, double err_th)
  {
    double th = tf::getYaw(currentPose.pose.orientation);
    double u_x_temp   = err_x;
    double u_y_temp   = err_y;
    double u_x   = cos(th) * u_x_temp + sin(th) * u_y_temp;
    double u_y   = -sin(th) * u_x_temp + cos(th) * u_y_temp;
    double u_th  = err_th ;

    cmd_vel.linear.x = clamp(u_x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    cmd_vel.linear.y = clamp(u_y, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = clamp(u_th, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    return true;
  }

  // determine if goal is reached
  bool isGoalReached()
  {
    if(!initialized)
    {
      ROS_WARN("DW local planner: Must be initilized first");
      return false;
    }
    if(costmap_ros->getRobotPose(currentPose) == false)
    {
      ROS_WARN("DW local planner: Failed to get current local pose");
      return false;
    }
    errGoalPose = getPoseDifference(globalPlan.back(),currentPose);

    //check to see if we've reached the goal position
    if(hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) <= XY_GOAL_TOLERANCE)
    {
      //check to see if the goal orientation has been reached
      if(fabs(tf::getYaw(errGoalPose.pose.orientation)) <= YAW_GOAL_TOLERANCE)
      {
        return true;
      }
    }
    return false;
  }

};
PLUGINLIB_EXPORT_CLASS(DWAPlanner, nav_core::BaseLocalPlanner);
