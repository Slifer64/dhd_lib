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

  void gotoNullPose() { sigma7->moveToNullPose(); }

  arma::vec getWristJointsLowerLim() const { return sigma7->getWristJointsLowerLim(); }
  arma::vec getWristJointsUpperLim() const { return sigma7->getWristJointsUpperLim(); }
  arma::vec getWristJoints() const { return sigma7->getWristJoints(); }

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
