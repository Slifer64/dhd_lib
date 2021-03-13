#ifndef DHD_SIGMA_7_H
#define DHD_SIGMA_7_H

#include <iostream>
#include <iomanip>

#include <dhd_sdk/dhdc.h>

namespace dhd_
{

class Sigma7
{
public:

  Sigma7(int i = 0);

  ~Sigma7();

  void setForce(double fx, double fy, double fz);

  void gravityComp();

  bool isButtonPressed();

private:

  int id_;

}; // Sigma7

} // dhd_


# endif // DHD_SIGMA_7_H
