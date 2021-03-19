#include <main_ctrl.h>


MainCtrl::MainCtrl()
{
  run_ = true;
  gui_finished = false;
  sigma7.reset(new dhd_::Sigma7);

  ctrl_pos = true;
  ctrl_orient = true;
  ctrl_grip = true;

  launchGui();
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
