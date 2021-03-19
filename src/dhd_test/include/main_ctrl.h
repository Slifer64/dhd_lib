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
#include <gui.h>

using namespace as64_;


class MainCtrl
{
public:
  MainCtrl();

  ~MainCtrl();

  arma::vec getWrench() const;
  arma::vec getPose() const;

  void setPosCtrl(bool set);
  void setOrientCtrl(bool set);
  void setGripCtrl(bool set);

  void gotoNullPose();

private:

  void launchGui();

  MainWindow *gui;

  bool run_;
  bool gui_finished;
  std::shared_ptr<dhd_::Sigma7> sigma7;

  bool ctrl_pos;
  bool ctrl_orient;
  bool ctrl_grip;
};
