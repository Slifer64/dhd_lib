#ifndef DHD_UTILS_H
#define DHD_UTILS_H

#include <iostream>
#include <fstream>
#include <memory>
#include <iomanip>

#include <armadillo>

namespace dhd_
{

// ====================   PRINT  ============================

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);

// ====================   MATH  ============================

arma::vec rotm2quat(const arma::mat &rotm);

arma::mat axang2rotm(const arma::vec &axis, double angle);

arma::mat wristAng2rotm(double oa, double ob, double og);

arma::mat rotx(double a);
arma::mat roty(double a);
arma::mat rotz(double a);

} // namespace dhd_

#endif // DHD_UTILS_H
