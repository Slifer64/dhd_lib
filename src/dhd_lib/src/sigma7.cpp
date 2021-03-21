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

  PRINT_INFO_MSG("=====================================\n");
  PRINT_INFO_MSG(std::string("Force Dimension - Sigma7 , SDK-") + dhdGetSDKVersionStr() + "\n");
  PRINT_INFO_MSG(std::string(dhdGetSystemName()) + " haptic device detected.\n");
  PRINT_INFO_MSG("=====================================\n");

  // perform auto-initialization
  if (!drdIsInitialized(id_))
  {
    PRINT_WARNING_MSG("Device is not initilized. Auto-initialization...\n");
    if (drdAutoInit(id_) < 0) throwError(__func__, "Auto-initialization failed: ");
    PRINT_CONFIRM_MSG("Device initialization completed!\n");
  }

  if (drdStart(id_) < 0) throwError(__func__, "Regulation thread failed to start: ");
  pos_ctrl_ = orient_ctrl_ = grip_ctrl_ = true;
  setGripCtrl(false);

  drdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0,  // force
                                       0.0, 0.0, 0.0,  // torque
                                       0.0);           // gripper force

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
  pos_ctrl_ = set;
}

void Sigma7::setOrientCtrl(bool set)
{
  if (drdRegulateRot(set, id_) < 0) throwError(__func__, "Failed to set orientation control: ");
  orient_ctrl_ = set;
}

void Sigma7::setGripCtrl(bool set)
{
  if (drdRegulateGrip(set, id_) < 0) throwError(__func__, "Failed to set gripper control: ");
  grip_ctrl_ = set;
}

void Sigma7::moveToPosWristJoints(const arma::vec &pos, const arma::vec &wrist_joints, bool is_blocking)
{
  double p[7] = {pos(0), pos(1), pos(2), wrist_joints(0), wrist_joints(1), wrist_joints(2), getGripperAngle()};

  std::string warn_msg = "";
  if (!pos_ctrl_) warn_msg += "Position control is disabled!\n";
  if (!orient_ctrl_) warn_msg += "Orientation control is disabled!\n";
  if (!warn_msg.empty()) PRINT_WARNING_MSG("[Sigma7::moveToPosWristJoints]:\n" + warn_msg);

  if (drdMoveTo(p, is_blocking, id_) < 0)
    throwError(__func__, "Failed to move to requested pose: ");
}

void Sigma7::moveToNullPose()
{
  moveToPosWristJoints({0,0,0}, {0,0,0}, true);
  // double p[DHD_MAX_DOF];
  // memset(p, 0, DHD_MAX_DOF);
  //
  // bool pos_ctrl_0 = pos_ctrl_;
  // bool orient_ctrl_0 = orient_ctrl_;
  // bool grip_ctrl_0 = grip_ctrl_;
  //
  // setPosCtrl(true);
  // setOrientCtrl(true);
  // setGripCtrl(true);
  //
  // if ( drdMoveTo(p, true, id_) )
  //   throwError(__func__, "Failed to move to null pose: ");
  //
  // setPosCtrl(pos_ctrl_0);
  // setOrientCtrl(orient_ctrl_0);
  // setGripCtrl(grip_ctrl_0);
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

arma::mat Sigma7::getRotm() const
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

double Sigma7::getGripperAngle() const
{
  double grip_ang;
  if (dhdGetGripperAngleRad(&grip_ang, id_) < 0)
    throwError(__func__, "Failed to get gripper angle: ");

  return grip_ang;
}

arma::mat Sigma7::wristAng2rotm(double j0, double j1, double j2) const
{
  // arma::vec ax0 = joint 0 rot axis w.r.t. base frame = {1, 0, 0}
  // arma::vec ax1 = joint 1 rot axis w.r.t. base frame = {0, 1, 0}
  // arma::vec ax2 = joint 2 rot axis w.r.t. base frame = {0, 0, 1}

  // first rotation is around ax0 by angle 'j0'
  arma::mat R_0 = rotx(j0); // axang2rotm(ax0, j0);

  // second rotation is around the new (rotated) axes ax1' = R_0*ax1 by angle 'j1'
  arma::mat R_1 = axang2rotm(R_0.col(1), j1); // axang2rotm(R_0*ax1, j1)

  // the total rotation so far is R_10 = R_1*R_0
  arma::mat R_10 = R_1*R_0;

  // third rotation is around the new (rotated) axes ax2' = R_10*ax2 by angle 'j2'
  arma::mat R_2 = axang2rotm(R_10.col(2), j2);  // axang2rotm(R_10*ax2, j2)

  // final total rotation R_210 = R_2 * R_1 * R_0
  return R_2 * R_10;
}

arma::mat Sigma7::getWristRotJacob(double j0, double j1, double j2) const
{
  // arma::vec ax0 = joint 0 rot axis w.r.t. base frame = {1, 0, 0}
  // arma::vec ax1 = joint 1 rot axis w.r.t. base frame = {0, 1, 0}
  // arma::vec ax2 = joint 2 rot axis w.r.t. base frame = {0, 0, 1}

  // first rotation is around ax0 by angle 'j0'
  arma::mat R_0 = rotx(j0); // axang2rotm(ax0, j0);

  // second rotation is around the new (rotated) axes ax1' = R_0*ax1 by angle 'j1'
  arma::mat R_1 = axang2rotm(R_0.col(1), j1); // axang2rotm(R_0*ax1, j1)

  // the total rotation so far is R_10 = R_1*R_0
  arma::mat R_10 = R_1*R_0;

  arma::mat J(3,3);
  J.col(0) = arma::vec({1, 0, 0}); // ax0;
  J.col(1) = R_0.col(1); // R_0*ax1;
  J.col(2) = R_10.col(2); // R_10*ax2;

  return J;
}

arma::mat Sigma7::getWristRotJacob() const
{
  arma::vec joints = getWristJoints();
  return getWristRotJacob(joints(0), joints(1), joints(2));
}

arma::vec Sigma7::getWristJoints() const
{
  dhdEnableExpertMode();

  double j0, j1, j2;
  if (dhdGetWristJointAngles(&j0, &j1, &j2, id_) < 0)
    throwError(__func__, "Failed to get wrist joints: ");

  dhdDisableExpertMode();

  return {j0, j1, j2};
}

arma::vec Sigma7::getWristJointsUpperLim() const
{
  return {1.72848, 1.2163, 0.3514};
}

arma::vec Sigma7::getWristJointsLowerLim() const
{
  return {-2.3534, -1.22546, -3.13459};
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
