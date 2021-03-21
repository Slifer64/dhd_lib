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

#include <thread_lib/thread_lib.h>

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

  bool startRecording();
  void stopRecording();
  bool isRecData() const { return Pos_data.size()!=0; }

  void replayRecTraj();

private:

  thr_::Semaphore rec_finish_sem;
  bool rec_data_on;
  arma::rowvec Time_data;
  arma::mat Pos_data;
  arma::mat Quat_data;
  arma::mat Wris_joint_data;

  void launchGui();

  MainWindow *gui;

  bool run_;
  bool gui_finished;
  std::shared_ptr<dhd_::Sigma7> sigma7;

  bool ctrl_pos;
  bool ctrl_orient;
  bool ctrl_grip;
};
