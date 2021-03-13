#include <dhd_lib/sigma7.h>

namespace dhd_
{

Sigma7::Sigma7(int i)
{
  // open the first available device
  int id_ = dhdOpenID(i);
  if (id_ < 0) throw std::runtime_error(std::string("[Sigma7::Sigma7]: ") + dhdErrorGetLastStr());
}

Sigma7::~Sigma7()
{
  if (dhdClose(id_) < 0) throw std::runtime_error(std::string("[Sigma7::~Sigma7]: ") + dhdErrorGetLastStr());
}


void Sigma7::setForce(double fx, double fy, double fz)
{
  if (dhdSetForce(fx, fy, fz, id_) < 0) throw std::runtime_error(std::string("[Sigma7::setForce]: ") + dhdErrorGetLastStr());
}

void Sigma7::gravityComp()
{
  setForce(0,0,0);
}

bool Sigma7::isButtonPressed()
{
  return dhdGetButton(0, id_);
}


} // dhd_
