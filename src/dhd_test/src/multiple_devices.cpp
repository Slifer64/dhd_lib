////////////////////////////////////////////////////////////////////////////////
//
//  This example implements a simple gravity compensation loop for two devices.
//  Note that a more efficient approach would require the creation of two haptic
//  threads, one for each device. This implementation is OS specific however
//  and is not implemented here.
//
////////////////////////////////////////////////////////////////////////////////
// C++ library headers
#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
// project headers
#include <dhd_lib/sigma7.h>
#include <dhd_lib/dhd.h>
////////////////////////////////////////////////////////////////////////////////

#include <dhd_sdk/dhdc.h>

int getDevCount()
{
  int device_count = dhdGetDeviceCount();
  if (device_count < 0) std::cerr << "[dhd_::getDeviceCount]: ERROR: " << dhdErrorGetLastStr() << std::endl;
  return device_count;
}

int main(int argc, char* argv[])
{
    // get device count
    int dev_count = dhd_::getDeviceCount();

    if (dev_count < 0)
    {
        std::cout << "error: " << dhdErrorGetLastStr() << std::endl;
        return -1;
    }
    else if (dev_count < 1)
    {
        std::cout << "error: no device detected" << std::endl;
    }
    else if (dev_count < 2)
    {
        std::cout << "error: singled device detected" << std::endl;
    }

    // create and open the devices
    std::vector<std::shared_ptr<dhd_::Sigma7>> sig_dev;
    for (int i=0; i<dev_count; i++) sig_dev[i].reset(new dhd_::Sigma7(i));

    // haptic loop
    while (true)
    {
      // apply a null force to put the device in gravity compensation
      for (int i=0; i<dev_count; i++) sig_dev[i]->setForce(0,0,0);

      for (int i=0; i<dev_count; i++)
      {
        if (sig_dev[i]->isButtonPressed()) break;
      }
    }

    // the connection to the devices closes automatically by the class destructor

    return 0;
}
