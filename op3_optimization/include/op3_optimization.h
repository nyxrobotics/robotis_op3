#ifndef OP3_OPTIMIZATION_H
#define OP3_OPTIMIZATION_H

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include "robotis_math/robotis_math.h"
#include "scilab_optimization/scilab_optimization.h"

#include "op3_online_walking_module_msgs/GetPreviewMatrix.h"

namespace robotis_framework
{
class Op3Optimization
{
public:
  Op3Optimization();
  ~Op3Optimization();

  bool getPreviewMatrixCallback(op3_online_walking_module_msgs::GetPreviewMatrix::Request& req,
                                op3_online_walking_module_msgs::GetPreviewMatrix::Response& res);

private:
  bool calcPreviewParam(double control_cycle, double lipm_height);

  ScilabOptimization scilab_optimization_;

  double P_row_, P_col_;
  std::vector<double_t> P_;
  double K_row_, K_col_;
  std::vector<double_t> K_;
};

}  // namespace robotis_framework

#endif  // OP3_OPTIMIZATION_H
