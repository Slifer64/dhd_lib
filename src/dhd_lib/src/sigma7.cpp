#include <dhd_lib/sigma7.h>
#include <dhd_lib/utils.h>

namespace dhd_
{

Sigma7::Sigma7(int i)
{
  // open the first available device
  this->id_ = dhdOpenID(i);
  if (id_ < 0) throwError("Sigma7");
}

Sigma7::~Sigma7()
{
  if (dhdClose(id_) < 0) throwError("~Sigma7");
}


bool Sigma7::isOk() const
{
  int status[DHD_MAX_STATUS];
  if (dhdGetStatus(status, id_) < 0) throwError("isOk");

  return !status[DHD_STATUS_ERROR];
}

bool Sigma7::isLeftHanded() const
{
  return dhdIsLeftHanded(id_);
}


void Sigma7::calibrate()
{
  if (dhdReset(id_) < 0) throwError("calibrate");
}


void Sigma7::setForce(const arma::vec &force)
{
  int ret = dhdSetForce(force(0), force(1), force(2), id_);
  if (ret < 0) throwError("setForce");
  else if (ret == DHD_MOTOR_SATURATED) PRINT_WARNING_MSG("[Sigma7::setForce]: DHD_MOTOR_SATURATED\n");

}

void Sigma7::setWrench(const arma::vec &wrench)
{
  int ret = dhdSetForceAndTorque(wrench(0), wrench(1), wrench(2), wrench(3), wrench(4), wrench(5), id_);
  if (ret < 0) throwError("setWrench");
  else if (ret == DHD_MOTOR_SATURATED) PRINT_WARNING_MSG("[Sigma7::setWrench]: DHD_MOTOR_SATURATED\n");

}

arma::vec Sigma7::getEePos() const
{
  double px, py, pz;
  int ret = dhdGetPosition(&px, &py, &pz, id_);
  if (ret < 0) throwError("getEePos");
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getEePos]: DHD_TIMEGUARD\n");

  return arma::vec({px, py, pz});
}

arma::mat Sigma7::getWristRotm() const
{
  arma::mat R(3,3);

  double tmp[3][3];
  int ret = dhdGetOrientationFrame(tmp, id_);
  if (ret < 0) throwError("getWristRotm");
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getWristRotm]: DHD_TIMEGUARD\n");

  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++) R(i,j) = tmp[i][j];
  }

  return R;
}

arma::vec Sigma7::getEePose() const
{
  double px, py, pz;
  double tmp[3][3];

  int ret = dhdGetPositionAndOrientationFrame(&px, &py, &pz, tmp, id_);
  if (ret < 0) throwError("getEePose");
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getEePose]: DHD_TIMEGUARD\n");

  arma::mat R(3,3);
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++) R(i,j) = tmp[i][j];
  }

  arma::vec pos = {px, py, pz};
  arma::vec quat = dhd_::rotm2quat(R);

  return arma::join_vert(pos, quat);
}


arma::vec Sigma7::getForce() const
{
  double fx, fy, fz;
  if (dhdGetForce(&fx, &fy, &fz, id_) < 0) throwError("getForce");

  return arma::vec({fx, fy, fz});
}

arma::vec Sigma7::getWrench() const
{
  double fx, fy, fz, tx, ty, tz;
  if (dhdGetForceAndTorque(&fx, &fy, &fz, &tx, &ty, &tz, id_) < 0) throwError("getWrench");

  return arma::vec({fx, fy, fz, tz, ty, tz});
}

arma::vec Sigma7::getLinVel() const
{
  double vx, vy, vz;
  if (dhdGetForce(&vx, &vy, &vz, id_) < 0) throwError("getLinVel");

  return arma::vec({vx, vy, vz});
}

double Sigma7::getGripperForce() const
{
  double fx, fy, fz, tx, ty, tz, f_grip;
  if (dhdGetForceAndTorqueAndGripperForce(&fx, &fy, &fz, &tx, &ty, &tz, &f_grip, id_) < 0) throwError("getGripperForce");

  return f_grip;
}

double Sigma7::getGripperAngle() const
{
  double grip_ang;
  int ret = dhdGetGripperAngleRad(&grip_ang, id_);
  if (ret < 0) throwError("getGripperAngle");
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperAngle]: DHD_TIMEGUARD\n");

  return grip_ang;
}

double Sigma7::getGripperOpenDist() const
{
  double fing_pos;
  int ret = dhdGetGripperGap(&fing_pos, id_);
  if (ret < 0) throwError("getGripperFingerPos");
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperFingerPos]: DHD_TIMEGUARD\n");

  return fing_pos;
}

double Sigma7::getGripperAngVel() const
{
  double ang_vel;
  int ret = dhdGetGripperAngularVelocityRad(&ang_vel, id_);
  if (ret < 0) throwError("getGripperAngVel");
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperAngVel]: DHD_TIMEGUARD\n");

  return ang_vel;
}

double Sigma7::getGripperLinVel() const
{
  double lin_vel;
  int ret = dhdGetGripperLinearVelocity(&lin_vel, id_);
  if (ret < 0) throwError("getGripperLinVel");
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperLinVel]: DHD_TIMEGUARD\n");

  return lin_vel;
}


void Sigma7::gravityComp(bool set)
{
  int val = set?DHD_ON:DHD_OFF;
  if (dhdSetGravityCompensation(val,id_) < 0) throwError("gravityComp");
}

bool Sigma7::isButtonPressed() const
{
  return dhdGetButton(0, id_);
}

double Sigma7::getEndEffectorMass() const
{
  double mass;
  if (dhdGetEffectorMass(&mass, id_) < 0) throwError("getEndEffectorMass");

  return mass;
}

void Sigma7::throwError(const std::string fun_name) const
{
  throw std::runtime_error("[Sigma7::" + fun_name + "Sigma7]: " + dhdErrorGetLastStr());
}


} // dhd_
