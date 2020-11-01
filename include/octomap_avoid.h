#ifndef OCTOMAPAVOID_H
#define OCTOMAPAVOID_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>

#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

class octomap_avoid
{
private:
  ros::Subscriber octSub_;
  ros::Subscriber poseSub_;
  ros::Subscriber goalSub_;

  ros::Publisher twistPub_;

  double repGain_;
  double attGain_;
  double repMaxDist_;
  double attParaBnd_;

  double robSpeed_;

  Eigen::Vector3d goalPos_;

  DynamicEDTOctomap* octDist_;

  uint8_t isInitialized_;

public:
  octomap_avoid(ros::NodeHandle* nh);
  ~octomap_avoid();

  void pose_cb(const geometry_msgs::PoseStamped& poseMsg);
  void goal_cb(const geometry_msgs::PointStamped& goalMsg);
  void octomap_cb(const octomap_msgs::Octomap& octmpMsg);
  

  void wait_for_params(ros::NodeHandle* nh);
  void update_dist_map(octomap::OcTree* octTree);
  Eigen::Vector3d neg_grad_att(Eigen::Vector3d robPos, Eigen::Vector3d goalPos, double gain, double paraBnd);
  Eigen::Vector3d neg_grad_rep(Eigen::Vector3d robPos, Eigen::Vector3d obsPos, double gain, double maxDist);
  bool nearest_obs(Eigen::Vector3d pos, Eigen::Vector3d& obsPos);
  
};

#endif
