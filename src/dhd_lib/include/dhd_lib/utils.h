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

} // namespace dhd_

#endif // DHD_UTILS_H
