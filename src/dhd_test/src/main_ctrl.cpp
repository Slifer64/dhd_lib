#include <main_ctrl.h>

#include <timer.h>

MainCtrl::MainCtrl()
{
  run_ = true;
  gui_finished = false;
  sigma7.reset(new dhd_::Sigma7);

  ctrl_pos = true;
  ctrl_orient = true;
  ctrl_grip = true;

  launchGui();

  sigma7->getRotm();
}

MainCtrl::~MainCtrl()
{
  gui_finished = true;
  run_ = false;
}

void MainCtrl::setPosCtrl(bool set)
{
  sigma7->setPosCtrl(set);
}

void MainCtrl::setOrientCtrl(bool set)
{
  sigma7->setOrientCtrl(set);
}

void MainCtrl::setGripCtrl(bool set)
{
  sigma7->setGripCtrl(set);
}

arma::vec MainCtrl::getWrench() const
{
  return sigma7->getWrench();
}

arma::vec MainCtrl::getPose() const
{
  return sigma7->getPose();
}

void MainCtrl::launchGui()
{
    gui_finished = false;
    std::thread gui_thr = std::thread([this]()
    {
      gui_::launchGui([this](){ gui=new MainWindow(this); return gui; }, &gui_finished, QThread::LowestPriority);
    });
    gui_thr.detach();
}

bool MainCtrl::startRecording()
{
  Time_data.clear();
  Pos_data.clear();
  Quat_data.clear();
  Wris_joint_data.clear();

  rec_data_on = true;

  std::thread([this]()
  {
    Timer timer;
    double t = 0;
    while (rec_data_on)
    {
      timer.start();
      arma::vec pose = sigma7->getPose();
      arma::vec wrist_joints = sigma7->getWristJoints();
      Time_data = arma::join_horiz(Time_data, arma::vec({t}));
      Pos_data = arma::join_horiz(Pos_data, pose.subvec(0,2));
      Quat_data = arma::join_horiz(Quat_data, pose.subvec(3,6));
      Wris_joint_data = arma::join_horiz(Wris_joint_data, wrist_joints);

      sigma7->waitNextCycle();

      t += timer.elapsedSec();
    }
    this->rec_finish_sem.notify();
  }).detach();

  return true;
}

void MainCtrl::stopRecording()
{
  rec_data_on = false;
  if (!rec_finish_sem.wait_until(500))
   gui->showWarnMsgSignal("Time limit exceeded on waiting for recording thread to stop...");

  std::cout << "Mean rec cycle: " << arma::mean(arma::diff(Time_data))*1000 << " ms\n";
}

void MainCtrl::replayRecTraj()
{
  sigma7->moveToPosWristJoints(Pos_data.col(0), Wris_joint_data.col(0), true);
  try
  {
    int n_data = Time_data.size();
    for (int i=0; i<n_data; i++)
    {
      sigma7->setPos(Pos_data.col(i));
      sigma7->setWristJoints(Wris_joint_data.col(i));
      sigma7->waitNextCycle();
    }
  }
  catch(std::exception &e)
  {
    gui->showErrMsgSignal(e.what());
  }
}
