#ifndef DHD_SIGMA_7_H
#define DHD_SIGMA_7_H

#include <iostream>
#include <iomanip>

#include <armadillo>

#include <dhd_sdk/dhdc.h>
#include <dhd_sdk/drdc.h>

namespace dhd_
{

enum Mode
{
  RESET,
  IDLE,
  FORCE,
  BRAKE
};

class Sigma7
{
public:

  Sigma7(int i = -1);

  ~Sigma7();

  bool isOk() const;

  bool isLeftHanded() const;

  void calibrate();

  void waitNextCycle() const;

  double getCtrlCycle() const;

  bool isButtonPressed() const;

  void setPosCtrl(bool set);
  void setOrientCtrl(bool set);
  void setGripCtrl(bool set);

  void moveToPose(const arma::vec &pose, bool is_blocking=true)
  {
   	double p[DHD_MAX_DOF];
    for (int i=0; i<pose.size();i++) 
    if (drdMoveTo(id_) < 0)
      throwError(__func__, "Failed to move to requested pose: ");
  }
  //void moveToPos(const arma::vec &pos, bool is_blocking=true);
  //void moveToRot(const arma::vec &quat, bool is_blocking=true);
  //void moveToGrip(const arma::vec &pos, bool is_blocking=true);

  arma::vec getPosition() const;
  arma::vec getRotm() const;
  arma::vec getQuat() const;
  arma::vec getPose() const;

  arma::vec getTwist() const;
  double getGripperVel() const;
  arma::vec getWrench() const;
  double getGripperForce() const;


  void setWrenchAndGripForce(const arma::vec &wrench, double grip_force);

  void setForce(const arma::vec &force);
  void setWrench(const arma::vec &wrench);

  arma::vec getLinVel() const;


  void gravityComp(bool set);



  double getEndEffectorMass() const;



private:

  int id_;

  void throwError(const std::string &fun_name, const std::string &msg=0) const;

}; // Sigma7

} // dhd_


# endif // DHD_SIGMA_7_H
