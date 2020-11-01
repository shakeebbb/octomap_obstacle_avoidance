#include "octomap_avoid.h"


octomap_avoid::octomap_avoid(ros::NodeHandle* nh)
{
  isInitialized_ = 0x00;
  wait_for_params(nh);

  octSub_ = nh->subscribe("octomap_in", 1, &octomap_avoid::octomap_cb, this);
  goalSub_ = nh->subscribe("goal_in", 1, &octomap_avoid::goal_cb, this);

  ROS_INFO("%s: Waiting for the first map and goal messages ...", nh->getNamespace().c_str());
  while( (isInitialized_ & 0x03) != 0x03 )
	  ros::spinOnce();

  poseSub_ = nh->subscribe("pose_in", 1, &octomap_avoid::pose_cb, this);
  twistPub_ = nh->advertise<geometry_msgs::TwistStamped>("twist_out", 10);

  ROS_INFO("%s: Initialized", nh->getNamespace().c_str());
}

// ***************************************************************************
void octomap_avoid::wait_for_params(ros::NodeHandle* nh)
{
  while(!nh->getParam("rep_pot_gain", repGain_));
  while(!nh->getParam("att_pot_gain", attGain_));
  while(!nh->getParam("rep_max_dist", repMaxDist_));
  while(!nh->getParam("att_para_bound", attParaBnd_));
  while(!nh->getParam("success_radius", succRad_));
  while(!nh->getParam("robot_speed", robSpeed_));
  
  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());
}

// ***************************************************************************
Eigen::Vector3d octomap_avoid::neg_grad_att(Eigen::Vector3d robPos, Eigen::Vector3d goalPos, double gain, double paraBnd)
{
  Eigen::Vector3d attVec = goalPos - robPos;

  double attVecNorm = attVec.norm();

  if(attVecNorm < succRad_)
    return Eigen::Vector3d(0,0,0);

  if(attVecNorm > paraBnd) // conical attractive field
  {
    attVec = attVec/attVecNorm;
    attVec *= (paraBnd*gain);
  }
  else // parabolic attractive field
  {
     attVec *= gain;
  }

  return attVec;
}

// ***************************************************************************
Eigen::Vector3d octomap_avoid::neg_grad_rep(Eigen::Vector3d robPos, Eigen::Vector3d obsPos, double gain, double maxDist)
{
  Eigen::Vector3d repVec = obsPos - robPos;
  double obsDist = repVec.norm();

  if(obsDist == 0)
   return Eigen::Vector3d(0,0,0);

  double repMag = gain * (1/obsDist - 1/maxDist) * (1/obsDist) * (1/obsDist);

  repVec = -1 * repMag * repVec/obsDist;
  
  return repVec;
}

// ***************************************************************************
void octomap_avoid::pose_cb(const geometry_msgs::PoseStamped& poseMsg)
{
  Eigen::Vector3d robPos;
  robPos(0) = poseMsg.pose.position.x;
  robPos(1) = poseMsg.pose.position.y;
  robPos(2) = poseMsg.pose.position.z;

  Eigen::Vector3d obsPos;
  Eigen::Vector3d negGrad;
  if (nearest_obs(robPos, obsPos))
    negGrad = neg_grad_att(robPos, goalPos_, attGain_, attParaBnd_) +
              neg_grad_rep(robPos, obsPos, repGain_, repMaxDist_);
  
  else
    negGrad = neg_grad_att(robPos, goalPos_, attGain_, attParaBnd_);

  std::cout << "Negative Gradient: " << negGrad.transpose() << std::endl;
  
  Eigen::Vector3d negGradDir = negGrad.normalized();
  Eigen::Vector3d cmdVel;

  if(negGradDir == negGrad)
    cmdVel = Eigen::Vector3d(0,0,0);
  else
    cmdVel = negGradDir * robSpeed_;
  
  geometry_msgs::TwistStamped cmdTwist;
  cmdTwist.header.stamp = ros::Time::now();
  cmdTwist.header.frame_id = poseMsg.header.frame_id;
  cmdTwist.twist.linear.x = cmdVel(0);
  cmdTwist.twist.linear.y = cmdVel(1);
  cmdTwist.twist.linear.z = cmdVel(2);
  cmdTwist.twist.angular.x = 0;
  cmdTwist.twist.angular.y = 0;
  cmdTwist.twist.angular.z = 0;

  twistPub_.publish(cmdTwist);
}

// ***************************************************************************
void octomap_avoid::goal_cb(const geometry_msgs::PointStamped& goalMsg)
{
  goalPos_(0) = goalMsg.point.x;
  goalPos_(1) = goalMsg.point.y;
  goalPos_(2) = goalMsg.point.z;

  if( (isInitialized_ & 0x02) != 0x02 )
    isInitialized_ = isInitialized_ | 0x02;
}

// ***************************************************************************
void octomap_avoid::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{
  ros::Time timeS = ros::Time::now();

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octomap::OcTree* octTree = dynamic_cast<octomap::OcTree*>(tree);

  //std::cout<<"read in tree, "<<octTree->getNumLeafNodes()<<" leaves "<<std::endl;

  if( (isInitialized_ & 0x01) != 0x01 )
    isInitialized_ = isInitialized_ | 0x01;
  else
    delete octDist_;
  
  update_dist_map(octTree);
  delete octTree;

  ros::Duration timeE = ros::Time::now() - timeS;
  //std::cout << "Time Elapsed = " << timeE.toSec() << " sec" << std::endl;
}

// ***************************************************************************
void octomap_avoid::update_dist_map(octomap::OcTree*& octTree)
{ 
  double x,y,z;

  octTree->getMetricMin(x,y,z);
  octomap::point3d min(x, y, z);

  octTree->getMetricMax(x,y,z);
  octomap::point3d max(x, y, z);

  bool unknownAsOccupied = false;

  double maxDist = repMaxDist_;

  //std::cout<<"read in tree, "<<octTree->getNumLeafNodes()<<" leaves "<<std::endl;
  
  octDist_ = new DynamicEDTOctomap(maxDist, octTree, min, max, unknownAsOccupied);

  //std::cout << "Updating dist map" << std::endl;
  octDist_->update(true);  //This computes the distance map
  //std::cout << "Updated dist map" << std::endl;
}

// ***************************************************************************
bool octomap_avoid::nearest_obs(Eigen::Vector3d pos, Eigen::Vector3d& obsPos)
{
  float distObs;
  octomap::point3d closestObs;
  octDist_->getDistanceAndClosestObstacle ( octomap::point3d(pos(0), pos(1), pos(2)), distObs, closestObs);

  if (distObs == DynamicEDTOctomap::distanceValue_Error)
    return false;
  else
  {
    obsPos = Eigen::Vector3d(closestObs(0), closestObs(1), closestObs(2));
    return true;
  }
}

// ***************************************************************************
octomap_avoid::~octomap_avoid()
{
  delete octDist_;
}
