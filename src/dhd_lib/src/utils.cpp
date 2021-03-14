#include <dhd_lib/utils.h>

#include <Eigen/Dense>

namespace dhd_
{

// ====================   PRINT  ============================

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}

// ====================   MATH  ============================

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm)
{
  Eigen::Quaternion<double> temp_quat(rotm);
  Eigen::Vector4d quat;
  quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

  quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

  return quat;
}

arma::vec rotm2quat(const arma::mat &rotm)
{
  arma::vec quat(4);

  Eigen::Map<const Eigen::Matrix3d> rotm_wrapper(rotm.memptr());
  Eigen::Map<Eigen::Vector4d> quat_wrapper(quat.memptr());
  quat_wrapper = rotm2quat(rotm_wrapper);

  return quat;
}


} // namespace dhd_
