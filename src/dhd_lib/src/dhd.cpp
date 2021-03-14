#include <dhd_lib/dhd.h>

#include <iostream>
#include <cstring>

namespace dhd_
{

int getDeviceCount()
{
  int device_count = dhdGetDeviceCount();
  if (device_count < 0) std::cerr << "[dhd_::getDeviceCount]: ERROR: " << dhdErrorGetStr(device_count) << std::endl;
  return device_count;
}

} // dhd_
