////////////////////////////////////////////////////////////////////////////////
// C++ library headers
#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <thread>

#include <ros/ros.h>

////////////////////////////////////////////////////////////////////////////////
#include <dhd_lib/sigma7.h>
#include <main_ctrl.h>


// void printStateThread(unsigned sleep_t_ms)
// {
//   std::thread([sleep_t_ms]()
//   {
//     arma::vec wrench;
//     arma::vec pos;
//     arma::vec quat;
//     double ctrl_cycle;
//
//     while (run_)
//     {
//       wrench = sigma7->getWrench();
//
//       std::cout << "==================================\n";
//       printf("force: %.2f  ,  %.2f  ,  %.2f  (N)\n", wrench(0), wrench(1), wrench(2));
//       printf("wrench: %.2f  ,  %.2f  ,  %.2f  (Nm)\n", wrench(3), wrench(4), wrench(5));
//       std::cout << "==================================\n";
//
//       std::this_thread::sleep_for(std::chrono::milliseconds(sleep_t_ms));
//     }
//
//
//   }).detach();
// }

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_sigma7");

  MainCtrl main_ctrl;

  // while (ros::ok());

  while (true)
  {
    if (dhdKbHit() && dhdKbGet()=='q') break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cerr << "==========> Terminating!!!!\n";

  // sigma7.reset(new dhd_::Sigma7());
  // printStateThread(200);
  // while (run_)
  // {
  //   if (dhdKbHit()) {
  //     switch (dhdKbGet())
  //     {
  //       case 'q': run_ = false; break;
  //       case 'p':
  //         ctrl_pos = !ctrl_pos;
  //         sigma7->setPosCtrl(ctrl_pos);
  //         break;
  //       case 'o':
  //         ctrl_orient = !ctrl_orient;
  //         sigma7->setOrientCtrl(ctrl_orient);
  //         break;
  //       case 'g':
  //         ctrl_grip = !ctrl_grip;
  //         sigma7->setGripCtrl(ctrl_grip);
  //         break;
  //     }
  //   }
  //
  //   if (dhdKbHit() && dhdKbGet() == 'q') run_ = false;
  // }

  return 0;
}
