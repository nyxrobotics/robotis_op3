#include "op3_optimization.h"

namespace robotis_framework
{
Op3Optimization::Op3Optimization()
{
}

Op3Optimization::~Op3Optimization()
{
}

bool Op3Optimization::calcPreviewParam(double control_cycle, double lipm_height)
{
  double preview_time_ = 1.6;
  int preview_size_;

  Eigen::MatrixXd A_, b_, c_;

  double t = control_cycle;
  preview_size_ = round(preview_time_ / t);

  A_.resize(3, 3);
  b_.resize(3, 1);
  c_.resize(1, 3);
  A_ << 1, t, t * t / 2.0, 0, 1, t, 0, 0, 1;
  b_ << t * t * t / 6.0, t * t / 2.0, t;
  c_ << 1, 0, -lipm_height / 9.81;

  Eigen::MatrixXd tempA = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd tempb = Eigen::MatrixXd::Zero(4, 1);
  Eigen::MatrixXd tempc = Eigen::MatrixXd::Zero(1, 4);

  tempA(0, 0) = 1;
  tempA.block<1, 3>(0, 1) = c_ * A_;
  tempA.block<3, 3>(1, 1) = A_;
  tempb(0, 0) = (c_ * b_)(0, 0);
  tempb.block<3, 1>(1, 0) = b_;
  tempc(0, 0) = 1;

  double Q_e = 1, R = 1e-6, Q_x = 0;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
  Q.diagonal() << Q_e, Q_e, Q_e, Q_x;

  double matrix_A[16], matrix_Q[16];
  int row_A = 4, col_A = 4;
  int row_Q = 4, col_Q = 4;

  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
    {
      matrix_A[c * 4 + r] = tempA(r, c);
      matrix_Q[c * 4 + r] = Q(r, c);
    }

  double matrix_B[] = { tempb(0, 0), tempb(1, 0), tempb(2, 0), tempb(3, 0) };
  int row_B = 4, col_B = 1;

  double matrix_R[] = { R };
  int row_R = 1, col_R = 1;

  double* matrix_K = (double*)malloc(100 * sizeof(double));
  double* matrix_P = (double*)malloc(100 * sizeof(double));
  double* matrix_E_real = (double*)malloc(100 * sizeof(double));
  double* matrix_E_imag = (double*)malloc(100 * sizeof(double));
  int row_K, col_K, row_P, col_P, row_E, col_E;

  scilab_optimization_.solveRiccatiEquation(matrix_K, &row_K, &col_K, matrix_P, &row_P, &col_P, matrix_E_real,
                                            matrix_E_imag, &row_E, &col_E, matrix_A, row_A, col_A, matrix_B, row_B,
                                            col_B, matrix_Q, row_Q, col_Q, matrix_R, row_R, col_R);

  K_.assign(matrix_K, matrix_K + row_K * col_K);
  K_row_ = row_K;
  K_col_ = col_K;

  P_.assign(matrix_P, matrix_P + row_P * col_P);
  P_row_ = row_P;
  P_col_ = col_P;

  free(matrix_K);
  free(matrix_P);
  free(matrix_E_real);
  free(matrix_E_imag);

  return true;
}

bool Op3Optimization::getPreviewMatrixCallback(op3_online_walking_module_msgs::GetPreviewMatrix::Request& req,
                                               op3_online_walking_module_msgs::GetPreviewMatrix::Response& res)
{
  if (!calcPreviewParam(req.req.control_cycle, req.req.lipm_height))
    return false;

  res.res.K_row = K_row_;
  res.res.K_col = K_col_;
  res.res.K = K_;
  res.res.P_row = P_row_;
  res.res.P_col = P_col_;
  res.res.P = P_;

  return true;
}

}  // namespace robotis_framework
