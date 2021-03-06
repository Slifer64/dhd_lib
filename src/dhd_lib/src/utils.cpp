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

arma::mat axang2rotm(const arma::vec &axis, double angle)
{
  double x=axis(0), y=axis(1), z=axis(2), c=std::cos(angle), s=std::sin(angle), t=1-c;

  return { {   t*x*x + c,	  t*x*y - z*s,    t*x*z + y*s },
    	     { t*x*y + z*s,     t*y*y + c,	  t*y*z - x*s },
           { t*x*z - y*s,   t*y*z + x*s,      t*z*z + c } };
}

arma::mat rotx(double a)
{
  double sin_a = std::sin(a);
  double cos_a = std::cos(a);

  return { { 1,     0,      0},
           { 0, cos_a, -sin_a},
           { 0, sin_a,  cos_a} };
}

arma::mat roty(double a)
{
  double sin_a = std::sin(a);
  double cos_a = std::cos(a);

  return { { cos_a,   0,  sin_a},
           {     0,   1,      0},
           {-sin_a,  0,   cos_a} };
}

arma::mat rotz(double a)
{
  double sin_a = std::sin(a);
  double cos_a = std::cos(a);

  return { {cos_a, -sin_a,  0},
           {sin_a,  cos_a,  0},
           {    0,      0,  1} };
}



} // namespace dhd_
