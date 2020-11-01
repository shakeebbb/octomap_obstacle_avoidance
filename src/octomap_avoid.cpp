#include "octomap_avoid.h"


octomap_avoid::octomap_avoid(ros::NodeHandle* nh)
{
  isInitialized_ = 0x00;
  wait_for_params(nh);

  octSub_ = nh->subscribe("octomap_in", 1, &octomap_avoid::octomap_cb, this);
  poseSub_ = nh->subscribe("pose_in", 1, &octomap_avoid::pose_cb, this);
  goalSub_ = nh->subscribe("goal_in", 1, &octomap_avoid::goal_cb, this);

  ROS_INFO("%s: Waiting for the first map and goal messages ...", nh->getNamespace().c_str());
  while( (isInitialized_ & 0x03) != 0x03 )
	  ros::spinOnce();

}

// ***************************************************************************
void octomap_avoid::wait_for_params(ros::NodeHandle* nh)
{
  while(!nh->getParam("rep_pot_gain", repGain_));
  while(!nh->getParam("att_pot_gain", attGain_));
  while(!nh->getParam("rep_max_dist", repMaxDist_));
  while(!nh->getParam("att_para_bound", attParaBnd_));
  
  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());
}

// ***************************************************************************
double neg_grad_att(Eigen::Vector3d robPos, Eigen::Vector3d goalPos, double gain, double paraBnd)
{
}

// ***************************************************************************
double neg_grad_rep(Eigen::Vector3d obsPos, double gain, double maxDist)
{
}

// ***************************************************************************
void octomap_avoid::pose_cb(const geometry_msgs::Pose& poseMsg)
{
  
}

// ***************************************************************************
void octomap_avoid::goal_cb(const geometry_msgs::PointStamped& goalMsg)
{
  goalPt_(0) = goalMsg.point.x;
  goalPt_(1) = goalMsg.point.y;
  goalPt_(2) = goalMsg.point.z;

  if( (isInitialized_ & 0x02) != 0x02 )
    isInitialized_ = isInitialized_ | 0x02;
}

// ***************************************************************************
void octomap_avoid::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{
  ros::Time timeS = ros::Time::now();

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octomap::OcTree* octTree = dynamic_cast<octomap::OcTree*>(tree);

  std::cout<<"read in tree, "<<octTree->getNumLeafNodes()<<" leaves "<<std::endl;

  if( (isInitialized_ & 0x01) != 0x01 )
    isInitialized_ = isInitialized_ | 0x01;
  
  update_dist_map(octTree);
  
  delete octTree;

  ros::Duration timeE = ros::Time::now() - timeS;
  std::cout << "Time Elapsed = " << timeE.toSec() << " sec" << std::endl;
}

// ***************************************************************************
void octomap_avoid::update_dist_map(octomap::OcTree* octTree)
{ 
  double x,y,z;

  octTree->getMetricMin(x,y,z);
  octomap::point3d min(x, y, z);

  octTree->getMetricMax(x,y,z);
  octomap::point3d max(x, y, z);

  bool unknownAsOccupied = false;

  double maxDist = repMaxDist_;

  delete octDist_;
  octDist_ = new DynamicEDTOctomap(maxDist, octTree, min, max, unknownAsOccupied);

  octDist_->update(true);  //This computes the distance map
  std::cout << "Updated dist map" << std::endl;
}

// ***************************************************************************
octomap_avoid::~octomap_avoid()
{
  delete octDist_;
}
