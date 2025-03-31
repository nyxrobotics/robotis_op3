#include <ros/ros.h>
#include "op3_optimization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "op3_optimization");
  ros::NodeHandle nh("~");

  robotis_framework::Op3Optimization optimization;

  ros::ServiceServer server =
      nh.advertiseService("/robotis/online_walking/get_preview_matrix",
                          &robotis_framework::Op3Optimization::getPreviewMatrixCallback, &optimization);

  ros::spin();
  return 0;
}
