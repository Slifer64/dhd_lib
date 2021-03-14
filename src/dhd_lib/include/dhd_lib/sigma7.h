#ifndef DHD_SIGMA_7_H
#define DHD_SIGMA_7_H

#include <iostream>
#include <iomanip>

#include <armadillo>

#include <dhd_sdk/dhdc.h>

namespace dhd_
{

class Sigma7
{
public:

  Sigma7(int i = 0);

  ~Sigma7();

  void setForce(double fx, double fy, double fz);

  arma::mat getWristRotm() const;
  arma::vec getEePos() const;

  arma::vec getEePose() const;


  arma::vec getForce() const;

  arma::vec getWrench() const;

  arma::vec getLinVel() const;

  double getGripperForce() const;
  double getGripperAngle() const;
  double getGripperOpenDist() const;
  double getGripperAngVel() const;
  double getGripperLinVel() const;

  void gravityComp();

  bool isButtonPressed() const;

  double getEndEffectorMass() const;

private:

  int id_;

  void throwError(const std::string fun_name, int err_code) const;

}; // Sigma7

} // dhd_


# endif // DHD_SIGMA_7_H
