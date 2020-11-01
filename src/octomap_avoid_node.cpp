#include "octomap_avoid.h"

using namespace std;
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "octomap_avoid_node");
	ros::NodeHandle nh(ros::this_node::getName());
	
	octomap_avoid octomapAvoid(&nh);
	
	ros::spin();

	return 0;
}
