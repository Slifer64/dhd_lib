#include <dhd_lib/sigma7.h>
#include <dhd_lib/utils.h>

namespace dhd_
{

Sigma7::Sigma7(int i)
{
  // open the device
  if (i < 0) id_ = drdOpen();
  else id_ = drdOpenID(i);

  if (id_ < 0) throwError(__func__, "Cannot open the device: ");

  // print out device identifier
  if (!drdIsSupported(id_)) throwError(__func__, "Unsupported device: ");

  // perform auto-initialization
  if (!drdIsInitialized(id_) && drdAutoInit(id_) < 0) throwError(__func__, "Auto-initialization failed: ");


  if (drdStart(id_) < 0) throwError(__func__, "Regulation thread failed to start: ");


  drdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0,  // force
                                       0.0, 0.0, 0.0,  // torque
                                       0.0);           // gripper force

  std::cout << "=====================================\n";
  std::cout << "Force Dimension - Sigma7 , SDK-" << dhdGetSDKVersionStr() << "\n";
  std::cout << dhdGetSystemName() << " haptic device detected.\n";
  std::cout << "=====================================\n";
}

Sigma7::~Sigma7()
{
  if (drdClose(id_) < 0) throwError(__func__,"Error on closing the DRD: ");
}


bool Sigma7::isOk() const
{
  int status[DHD_MAX_STATUS];
  if (dhdGetStatus(status, id_) < 0) throwError(__func__, "Failed to get device status: ");

  return !status[DHD_STATUS_ERROR];
}

bool Sigma7::isLeftHanded() const
{
  return dhdIsLeftHanded(id_);
}

double Sigma7::getCtrlCycle() const
{
  double ctrl_freq = drdGetCtrlFreq(id_);
  if (ctrl_freq < 0) throwError(__func__,"Failed to get the ctrl frequency: ");

  return 1.0/ctrl_freq;
}

void Sigma7::calibrate()
{
  if (!drdIsInitialized(id_) && drdAutoInit(id_) < 0) throwError(__func__, "Auto-calibration failed: ");
}

void Sigma7::setPosCtrl(bool set)
{
  if (drdRegulatePos(set, id_) < 0) throwError(__func__, "Failed to set position control: ");
}

void Sigma7::setOrientCtrl(bool set)
{
  if (drdRegulateRot(set, id_) < 0) throwError(__func__, "Failed to set orientation control: ");
}

void Sigma7::setGripCtrl(bool set)
{
  if (drdRegulateGrip(set, id_) < 0) throwError(__func__, "Failed to set gripper control: ");
}

arma::vec Sigma7::getTwist() const
{
  double vx, vy, vz, wx, wy, wz, vg;
  if (drdGetVelocity(&vx, &vy, &vz, &wx, &wy, &wz, &vg, id_) < 0)
    throwError(__func__, "Failed to get end-effector twist: ");

  return {vx, vy, vz, wx, wy, wz};
}

double Sigma7::getGripperVel() const
{
  double vx, vy, vz, wx, wy, wz, vg;
  if (drdGetVelocity(&vx, &vy, &vz, &wx, &wy, &wz, &vg, id_) < 0)
    throwError(__func__, "Failed to get gripper velocity: ");

  return vg;
}

arma::vec Sigma7::getWrench() const
{
  double fx, fy, fz, tx, ty, tz;
  if (dhdGetForceAndTorque(&fx, &fy, &fz, &tx, &ty, &tz, id_) < 0)
    throwError(__func__, "Failed to get end-effector wrench: ");

  return arma::vec({fx, fy, fz, tz, ty, tz});
}

double Sigma7::getGripperForce() const
{
  double fx, fy, fz, tx, ty, tz, f_grip;
  if (dhdGetForceAndTorqueAndGripperForce(&fx, &fy, &fz, &tx, &ty, &tz, &f_grip, id_) < 0)
    throwError(__func__, "Failed to get gripper force: ");

  return f_grip;
}

arma::vec Sigma7::getPosition() const
{
  double px, py, pz;
  double oa, ob, og;
  double pg;
  double matrix[3][3];

  if (drdGetPositionAndOrientation(&px, &py, &pz, &oa, &ob, &og, &pg, matrix, id_) < 0)
    throwError(__func__, "Failed to get position and orientation: ");

  return {px, py, pz};
}

arma::vec Sigma7::getRotm() const
{
  double px, py, pz;
  double oa, ob, og;
  double pg;
  double matrix[3][3];

  if (drdGetPositionAndOrientation(&px, &py, &pz, &oa, &ob, &og, &pg, matrix, id_) < 0)
    throwError(__func__, "Failed to get position and orientation: ");

    arma::mat R(3,3);
    for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++) R(i,j) = matrix[i][j];
    }

    return R;
}

arma::vec Sigma7::getQuat() const
{
  double px, py, pz;
  double oa, ob, og;
  double pg;
  double matrix[3][3];

  if (drdGetPositionAndOrientation(&px, &py, &pz, &oa, &ob, &og, &pg, matrix, id_) < 0)
    throwError(__func__, "Failed to get position and orientation: ");

    arma::mat R(3,3);
    for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++) R(i,j) = matrix[i][j];
    }

    return dhd_::rotm2quat(R);
}

arma::vec Sigma7::getPose() const
{
  double px, py, pz;
  double oa, ob, og;
  double pg;
  double matrix[3][3];

  if (drdGetPositionAndOrientation(&px, &py, &pz, &oa, &ob, &og, &pg, matrix, id_) < 0)
    throwError(__func__, "Failed to get position and orientation: ");

  arma::mat R(3,3);
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++) R(i,j) = matrix[i][j];
  }

  arma::mat Q = dhd_::rotm2quat(R);

  return {px, py, pz, Q(0), Q(1), Q(2), Q(3)};
}


void Sigma7::setWrenchAndGripForce(const arma::vec &wrench, double grip_force)
{
  if (drdSetForceAndTorqueAndGripperForce(wrench(0), wrench(1), wrench(2),  wrench(3), wrench(4), wrench(5), grip_force, id_) < 0)
    throwError(__func__, "Failed to set wrench and gripper force: ");
}


void Sigma7::setForce(const arma::vec &force)
{
  int ret = dhdSetForce(force(0), force(1), force(2), id_);
  if (ret < 0) throwError(__func__);
  else if (ret == DHD_MOTOR_SATURATED) PRINT_WARNING_MSG("[Sigma7::setForce]: DHD_MOTOR_SATURATED\n");

}

void Sigma7::setWrench(const arma::vec &wrench)
{
  int ret = dhdSetForceAndTorque(wrench(0), wrench(1), wrench(2), wrench(3), wrench(4), wrench(5), id_);
  if (ret < 0) throwError(__func__);
  else if (ret == DHD_MOTOR_SATURATED) PRINT_WARNING_MSG("[Sigma7::setWrench]: DHD_MOTOR_SATURATED\n");

}




arma::vec Sigma7::getLinVel() const
{
  double vx, vy, vz;
  if (dhdGetForce(&vx, &vy, &vz, id_) < 0) throwError(__func__);

  return arma::vec({vx, vy, vz});
}



void Sigma7::gravityComp(bool set)
{
  int val = set?DHD_ON:DHD_OFF;
  if (dhdSetGravityCompensation(val,id_) < 0) throwError(__func__);
}

bool Sigma7::isButtonPressed() const
{
  return dhdGetButton(0, id_);
}

void Sigma7::waitNextCycle() const
{
  drdWaitForTick(id_);
}

void Sigma7::throwError(const std::string &fun_name, const std::string &msg) const
{
  drdSleep(2.0);
  drdClose(id_);
  throw std::runtime_error("[Sigma7::" + fun_name + " - device id: " + std::to_string(id_) + "]: " + msg + dhdErrorGetLastStr());
}


} // dhd_
