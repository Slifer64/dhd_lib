#include <gui.h>

#include <main_ctrl.h>

MainWindow::MainWindow(MainCtrl *main_ctrl, QWidget *parent) : QMainWindow(parent)
{
  this->main_ctrl = main_ctrl;

  std::vector<std::string> priority_name = {"IdlePriority", "LowestPriority", "LowPriority", "NormalPriority", "HighPriority", "HighestPriority", "TimeCriticalPriority", "InheritPriority"};
  QThread::Priority priority = QThread::currentThread()->priority();

  QObject::connect( this, SIGNAL(showInfoMsgSignal(const char *)), this, SLOT(showInfoMsg(const char *)) );
  QObject::connect( this, SIGNAL(showWarnMsgSignal(const char *)), this, SLOT(showWarnMsg(const char *)) );
  QObject::connect( this, SIGNAL(showErrMsgSignal(const char *)), this, SLOT(showErrMsg(const char *)) );

  //this->resize(400,350);
  this->setWindowTitle("Main window");

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  QFont font1 = QFont("Ubuntu", 15, QFont::DemiBold);
  QFont font2 = QFont("Ubuntu", 13, QFont::DemiBold);

  // QLabel *lh_label = new QLabel("Left handle ctrl");
  // lh_label->setFont(font1);
  // lh_label->setAlignment(Qt::AlignCenter);

  // =============================================
  // ===========  Menu Actions ===================
  // =============================================

  arma::vec jlow_lim = main_ctrl->getWristJointsLowerLim() * 180/3.14159265359;
  arma::vec jup_lim = main_ctrl->getWristJointsUpperLim() * 180/3.14159265359;
  gui_::ViewJPosDialog *view_wrist_joints_dialog = new gui_::ViewJPosDialog(jlow_lim, jup_lim, [this](){ return this->main_ctrl->getWristJoints(); }, this);
  view_wrist_joints_dialog->setTitle("Wrist joints");
  view_wrist_joints_act = new QAction(tr("View wrist joints"), this);
  view_wrist_joints_act->setStatusTip(tr("Opens a window displaying the wrist's joints."));
  QObject::connect( view_wrist_joints_act, &QAction::triggered, this, [view_wrist_joints_dialog](){ view_wrist_joints_dialog->launch();} );

  gui_::ViewWrenchDialog *view_wrench_dialog = new gui_::ViewWrenchDialog([this](){ return this->main_ctrl->getWrench(); }, [this](){ return arma::mat().eye(3,3); }, this);
  view_wrench_dialog->setTitle("Wrench");
  view_wrench_act = new QAction(tr("View wrench"), this);
  view_wrench_act->setStatusTip(tr("Opens a window displaying the wrench at the endeffector."));
  QObject::connect( view_wrench_act, &QAction::triggered, this, [view_wrench_dialog](){ view_wrench_dialog->launch(); } );


  gui_::ViewPoseDialog *view_pose_dialog = new gui_::ViewPoseDialog([this](){ return this->main_ctrl->getPose(); }, this);
  view_pose_dialog->setTitle("End-effector pose");
  view_pose_act = new QAction(tr("View pose"), this);
  view_pose_act->setStatusTip(tr("Opens a window displaying the end-effector pose."));
  QObject::connect( view_pose_act, &QAction::triggered, this, [view_pose_dialog](){ view_pose_dialog->launch();} );


  save_rec_data_act = new QAction(tr("Save rec data"), this);
  save_rec_data_act->setStatusTip(tr("Opens a to select the path for storing the recorded data."));
  QObject::connect( save_rec_data_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getSaveFileName(this, tr("Save recorded data"), this->main_ctrl->getDefaultDataPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    this->main_ctrl->saveRecData(path);
  });

  // =========================================================
  // ===========  DOFs control layout  ===================
  // =========================================================
  pos_ctrl_chbx = new QCheckBox("Position control");
  pos_ctrl_chbx->setFont(font1);
  pos_ctrl_chbx->setChecked(true);
  QObject::connect( pos_ctrl_chbx, &QCheckBox::stateChanged, this, [this](){ this->main_ctrl->setPosCtrl(pos_ctrl_chbx->isChecked());} );
  QObject::connect( this, &MainWindow::posCtrlChangedSignal, this, [this](bool set){ this->pos_ctrl_chbx->setChecked(set);} );

  orient_ctrl_chbx = new QCheckBox("Orientation control");
  orient_ctrl_chbx->setFont(font1);
  orient_ctrl_chbx->setChecked(true);
  QObject::connect( orient_ctrl_chbx, &QCheckBox::stateChanged, this, [this](){ this->main_ctrl->setOrientCtrl(orient_ctrl_chbx->isChecked());} );
  QObject::connect( this, &MainWindow::orientCtrlChangedSignal, this, [this](bool set){ this->orient_ctrl_chbx->setChecked(set);} );

  grip_ctrl_chbx = new QCheckBox("Gripper control");
  grip_ctrl_chbx->setFont(font1);
  grip_ctrl_chbx->setChecked(false);
  QObject::connect( grip_ctrl_chbx, &QCheckBox::stateChanged, this, [this](){ this->main_ctrl->setGripCtrl(grip_ctrl_chbx->isChecked());} );
  QObject::connect( this, &MainWindow::gripCtrlChangedSignal, this, [this](bool set){ this->grip_ctrl_chbx->setChecked(set);} );

  QVBoxLayout *dofs_ctrl_layout = new QVBoxLayout();
  dofs_ctrl_layout->addWidget(pos_ctrl_chbx);
  dofs_ctrl_layout->addWidget(orient_ctrl_chbx);
  dofs_ctrl_layout->addWidget(grip_ctrl_chbx);
  dofs_ctrl_layout->addStretch(0);

  // ==============================================
  // ===========  Utils layout  ===================
  // ==============================================
  goto_null_pose_btn = new QPushButton("Goto null pose");
  goto_null_pose_btn->setFont(font1);
  QObject::connect( goto_null_pose_btn, &QPushButton::pressed, this, [this](){ this->main_ctrl->gotoNullPose();} );

  QVBoxLayout *utils_layout = new QVBoxLayout();
  utils_layout->addWidget(goto_null_pose_btn);
  utils_layout->addStretch(0);

  // ===============================================
  // ===========  Record frame  ===================
  // ===============================================
  QLabel *record_lb = new QLabel("Record Trajectory");
  record_lb->setFont(font1);
  record_lb->setAlignment(Qt::AlignCenter);

  start_rec_btn = new QPushButton("start");
  start_rec_btn->setFont(font1);
  QObject::connect( start_rec_btn, &QPushButton::pressed, this, [this]()
  {
    bool success = this->main_ctrl->startRecording();
    if (success)
    {
      stop_rec_btn->setEnabled(true);
      start_rec_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
    }
  });

  stop_rec_btn = new QPushButton("stop");
  stop_rec_btn->setFont(font1);
  stop_rec_btn->setEnabled(false);
  QObject::connect( stop_rec_btn, &QPushButton::pressed, this, [this]()
  {
    this->main_ctrl->stopRecording();
    stop_rec_btn->setEnabled(false);
    start_replay_traj_btn->setEnabled(this->main_ctrl->isRecData());
    start_rec_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  });

  QHBoxLayout *start_stop_rec_layout = new QHBoxLayout;
  start_stop_rec_layout->addWidget(start_rec_btn);
  start_stop_rec_layout->addWidget(stop_rec_btn);

  QVBoxLayout *rec_layout = new QVBoxLayout;
  rec_layout->addWidget(record_lb);
  rec_layout->addLayout(start_stop_rec_layout);

  QFrame *rec_frame = new QFrame;
  rec_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  rec_frame->setLineWidth(1);
  rec_frame->setLayout(rec_layout);

  // ===================================================
  // ===========  Traj Replay frame  ===================
  // ===================================================

  QLabel *replay_traj_lb = new QLabel("replay trajectory");
  replay_traj_lb->setFont(font1);
  replay_traj_lb->setAlignment(Qt::AlignCenter);
  start_replay_traj_btn = new QPushButton("start");
  start_replay_traj_btn->setFont(font1);
  start_replay_traj_btn->setEnabled(false);
  QObject::connect( start_replay_traj_btn, &QPushButton::pressed, this, [this]()
  {
    this->main_ctrl->startTrajReplay();
    stop_replay_traj_btn->setEnabled(true);
    start_replay_traj_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  });
  stop_replay_traj_btn = new QPushButton("stop");
  stop_replay_traj_btn->setFont(font1);
  stop_replay_traj_btn->setEnabled(false);
  QObject::connect( stop_replay_traj_btn, &QPushButton::pressed, this, [this]()
  {
    this->main_ctrl->stopTrajReplay();
    stop_replay_traj_btn->setEnabled(false);
    start_replay_traj_btn->setEnabled(this->main_ctrl->isRecData());
    start_replay_traj_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  });
  QObject::connect( this, &MainWindow::replayTrajStoppedSignal, this, [this](){ emit this->stop_replay_traj_btn->pressed(); } );

  QHBoxLayout *start_stop_replay_layout = new QHBoxLayout;
  start_stop_replay_layout->addWidget(start_replay_traj_btn);
  start_stop_replay_layout->addWidget(stop_replay_traj_btn);

  QVBoxLayout *replay_layout = new QVBoxLayout;
  replay_layout->addWidget(replay_traj_lb);
  replay_layout->addLayout(start_stop_replay_layout);

  QFrame *replay_frame = new QFrame;
  replay_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  replay_frame->setLineWidth(1);
  replay_frame->setLayout(replay_layout);

  // --------------------------------------------

  QVBoxLayout *rec_replay_layout = new QVBoxLayout;
  rec_replay_layout->addWidget(rec_frame);
  rec_replay_layout->addWidget(replay_frame);


  // =============================================
  // ===========  Main Layout  ===================
  // =============================================

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addLayout(dofs_ctrl_layout);
  main_layout->addLayout(utils_layout);
  main_layout->addLayout(rec_replay_layout);

  createMenu();
}

MainWindow::~MainWindow()
{}

void MainWindow::createMenu()
{
  // =======   Create menus   ==========

  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(save_rec_data_act);
  // file_menu->addAction(load_model_act);
  // file_menu->addSeparator();

  QMenu *edit_menu = menu_bar->addMenu(tr("&Edit"));
  // edit_menu->addAction(set_start_pose_act);
  // edit_menu->addSeparator();

  QMenu *view_menu = menu_bar->addMenu(tr("&View"));
  view_menu->addAction(view_wrist_joints_act);
  view_menu->addAction(view_pose_act);
  view_menu->addAction(view_wrench_act);
  // view_menu->addSeparator();
  // view_menu->addAction(train_win->plot_train_data_act);
  // view_menu->addAction(train_win->plot_demo_sim_data_act);

}

void MainWindow::showInfoMsg(const char *msg)
{
  QMessageBox msg_box;

  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Information);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.setModal(true);

  msg_box.exec();
}

void MainWindow::showWarnMsg(const char *msg)
{
  QMessageBox msg_box;

  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Warning);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.setModal(true);

  msg_box.exec();
}

void MainWindow::showErrMsg(const char *msg)
{
  QMessageBox msg_box;

  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Critical);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.setModal(true);

  msg_box.exec();
}
