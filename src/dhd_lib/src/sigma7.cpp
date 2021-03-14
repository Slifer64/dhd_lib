#include <dhd_lib/sigma7.h>
#include <dhd_lib/utils.h>

namespace dhd_
{

Sigma7::Sigma7(int i)
{
  // open the first available device
  this->id_ = dhdOpenID(i);
  if (id_ < 0) throwError("Sigma7", id_);
}

Sigma7::~Sigma7()
{
  int ret = dhdClose(id_);
  if (ret < 0) throwError("~Sigma7", ret);
}


void Sigma7::setForce(double fx, double fy, double fz)
{
  int ret = dhdSetForce(fx, fy, fz, id_);
  if (ret < 0) throwError("setForce", ret);
}

arma::vec Sigma7::getEePos() const
{
  double px, py, pz;
  int ret = dhdGetPosition(&px, &py, &pz, id_);
  if (ret < 0) throwError("getEePos", ret);
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getEePos]: DHD_TIMEGUARD\n");

  return arma::vec({px, py, pz});
}

arma::mat Sigma7::getWristRotm() const
{
  arma::mat R(3,3);

  double tmp[3][3];
  int ret = dhdGetOrientationFrame(tmp, id_);
  if (ret < 0) throwError("getWristRotm", ret);
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
  int ret = dhdGetForce(&fx, &fy, &fz, id_);
  if (ret < 0) throwError("getForce", ret);
  return arma::vec({fx, fy, fz});
}

arma::vec Sigma7::getWrench() const
{
  double fx, fy, fz, tx, ty, tz;
  int ret = dhdGetForceAndTorque(&fx, &fy, &fz, &tx, &ty, &tz, id_);
  if (ret < 0) throwError("getWrench", ret);

  return arma::vec({fx, fy, fz, tz, ty, tz});
}

arma::vec Sigma7::getLinVel() const
{
  double vx, vy, vz;
  int ret = dhdGetForce(&vx, &vy, &vz, id_);
  if (ret < 0) throwError("getLinVel", ret);
  return arma::vec({vx, vy, vz});
}

double Sigma7::getGripperForce() const
{
  double fx, fy, fz, tx, ty, tz, f_grip;
  int ret = dhdGetForceAndTorqueAndGripperForce(&fx, &fy, &fz, &tx, &ty, &tz, &f_grip, id_);
  if (ret < 0) throwError("getGripperForce", ret);

  return f_grip;
}

double Sigma7::getGripperAngle() const
{
  double grip_ang;
  int ret = dhdGetGripperAngleRad(&grip_ang, id_);
  if (ret < 0) throwError("getGripperAngle", ret);
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperAngle]: DHD_TIMEGUARD\n");

  return grip_ang;
}

double Sigma7::getGripperOpenDist() const
{
  double fing_pos;
  int ret = dhdGetGripperGap(&fing_pos, id_);
  if (ret < 0) throwError("getGripperFingerPos", ret);
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperFingerPos]: DHD_TIMEGUARD\n");

  return fing_pos;
}

double Sigma7::getGripperAngVel() const
{
  double ang_vel;
  int ret = dhdGetGripperAngularVelocityRad(&ang_vel, id_);
  if (ret < 0) throwError("getGripperAngVel", ret);
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperAngVel]: DHD_TIMEGUARD\n");

  return ang_vel;
}

double Sigma7::getGripperLinVel() const
{
  double lin_vel;
  int ret = dhdGetGripperLinearVelocity(&lin_vel, id_);
  if (ret < 0) throwError("getGripperLinVel", ret);
  else if (ret == 0) PRINT_WARNING_MSG("[Sigma7::getGripperLinVel]: DHD_TIMEGUARD\n");

  return lin_vel;
}


void Sigma7::gravityComp()
{
  setForce(0,0,0);
}

bool Sigma7::isButtonPressed() const
{
  return dhdGetButton(0, id_);
}

double Sigma7::getEndEffectorMass() const
{
  double mass;
  int ret = dhdGetEffectorMass(&mass, id_);
  if (ret < 0) throwError("getEndEffectorMass", ret);

  return mass;
}

void Sigma7::throwError(const std::string fun_name, int err_code) const
{
  throw std::runtime_error("[Sigma7::" + fun_name + "Sigma7]: " + dhdErrorGetStr(err_code));
}


} // dhd_
