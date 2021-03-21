#include <main_ctrl.h>

#include <timer.h>
#include <file_io.h>

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
  Wrist_joint_data.clear();

  rec_data_on = true;

  std::thread([this]()
  {
    Timer timer;
    double t = 0;
    unsigned long Ts_ns = 0.002 * 1e9;
    while (rec_data_on)
    {
      timer.start();
      arma::vec pose = sigma7->getPose();
      arma::vec wrist_joints = sigma7->getWristJoints();
      Time_data = arma::join_horiz(Time_data, arma::vec({t}));
      Pos_data = arma::join_horiz(Pos_data, pose.subvec(0,2));
      Quat_data = arma::join_horiz(Quat_data, pose.subvec(3,6));
      Wrist_joint_data = arma::join_horiz(Wrist_joint_data, wrist_joints);

      unsigned long diff_T = Ts_ns - timer.elapsedNanoSec();
      if (diff_T > 0) std::this_thread::sleep_for(std::chrono::nanoseconds(diff_T));
      t += timer.elapsedSec();

      // sigma7->waitNextCycle();
    }
    trimRecData();
    this->rec_stop_sem.notify();
  }).detach();

  return true;
}

void MainCtrl::stopRecording()
{
  rec_data_on = false;
  if (!rec_stop_sem.wait_until(1000))
    gui->showWarnMsgSignal("Timeout on waiting for recording thread to stop...");

  std::cout << "Mean rec cycle: " << arma::mean(arma::diff(Time_data))*1000 << " ms\n";
}

void MainCtrl::trimRecData()
{
  int n_data = Time_data.size();

  int i1 = 0;
  int i2 = n_data - 1;

  double pos_thres = 0.005;
  double ang_thres = 0.01;

  arma::vec p0 = Pos_data.col(0);
  arma::vec pf = Pos_data.col(n_data-1);

  arma::vec q0 = Wrist_joint_data.col(0);
  arma::vec qf = Wrist_joint_data.col(n_data-1);

  for (int i=i1; i<i2; i++)
  {
    if (arma::norm(Pos_data.col(i)-p0) > pos_thres || arma::norm(Wrist_joint_data.col(i)-q0) > ang_thres)
    { i1 = i; break; }
  }

  for (int i=i2; i>i1; i--)
  {
    if (arma::norm(Pos_data.col(i)-pf) > pos_thres || arma::norm(Wrist_joint_data.col(i)-qf) > ang_thres)
    { i2 = i; break; }
  }

  Time_data = Time_data.subvec(i1,i2);
  Time_data = Time_data - Time_data(0);
  Pos_data = Pos_data.cols(i1,i2);
  Quat_data = Quat_data.cols(i1,i2);
  Wrist_joint_data = Wrist_joint_data.cols(i1,i2);
}

void MainCtrl::startTrajReplay()
{
  traj_replay_ = true;

  std::thread([this]()
  {
    try
    {
      sigma7->moveToPosWristJoints(Pos_data.col(0), Wrist_joint_data.col(0), true);
      int n_data = Time_data.size();
      for (int i=0; i<n_data; i++)
      {
        // std::cout << "i = " << i << "\n";
        if (!traj_replay_) break;
        sigma7->setPos(Pos_data.col(i));
        sigma7->setWristJoints(Wrist_joint_data.col(i));
        //sigma7->waitNextCycle();
        std::this_thread::sleep_for(std::chrono::nanoseconds(2000000));
      }
      replay_stop_sem.notify();
      stopTrajReplay();
    }
    catch(std::exception &e)
    {
      replay_stop_sem.notify();
      emit gui->replayTrajStoppedSignal();
      gui->showErrMsgSignal(e.what());
    }
  }).detach();
}

void MainCtrl::stopTrajReplay()
{
  traj_replay_ = false;

  if (!replay_stop_sem.wait_until(1000))
    gui->showWarnMsgSignal("Timeout on waiting for traj-replay thread to stop...");
}

void MainCtrl::saveRecData(const std::string &filename) const
{
  if (!isRecData())
  {
    gui->showWarnMsgSignal("No recorded data...");
    return;
  }

  try
  {
    io_::FileIO fid(filename, io_::FileIO::out|io_::FileIO::trunc);
    fid.write("Time", Time_data);
    fid.write("Pos", Pos_data);
    fid.write("Quat", Quat_data);
    fid.write("Wrist_joints", Wrist_joint_data);
    gui->showInfoMsgSignal("Saved recorded data!");
  }
  catch(std::exception &e)
  { gui->showErrMsgSignal(e.what()); }
}
