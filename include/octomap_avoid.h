#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>

#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"

#include "ros/ros.h"
class octomap_avoid
{
private:
  ros::Subscriber octSub_;
  ros::Subscriber poseSub_;
  ros::Subscriber goalSub_;

  double repGain_;
  double attGain_;
  double repMaxDist_;
  double attParaBnd_;

  Eigen::Vector3d goalPt_;

  DynamicEDTOctomap* octDist_;

public:
  octomap_avoid(ros::NodeHandle* nh);
  ~octomap_avoid();

  void pose_cb(const geometry_msgs::Pose& poseMsg);
  void goal_cb(const geometry_msgs::PointStamped& goalMsg);
  void octomap_cb(const octomap_msgs::Octomap& octmpMsg);
  

  void wait_for_params(ros::NodeHandle* nh);
  void update_dist_map(octomap::OcTree* octTree);
  double neg_grad_att(Eigen::Vector3d robPos, Eigen::Vector3d goalPos, double gain, double paraBnd);
  double neg_grad_rep(Eigen::Vector3d obsPos, double gain, double maxDist);
  
};

#endif
