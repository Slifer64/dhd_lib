#include <gui.h>

#include <main_ctrl.h>

MainWindow::MainWindow(MainCtrl *main_ctrl, QWidget *parent) : QMainWindow(parent)
{
  this->main_ctrl = main_ctrl;

  std::vector<std::string> priority_name = {"IdlePriority", "LowestPriority", "LowPriority", "NormalPriority", "HighPriority", "HighestPriority", "TimeCriticalPriority", "InheritPriority"};
  QThread::Priority priority = QThread::currentThread()->priority();

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

  // ==============================================
  // ===========  View Dialogs  ===================
  // ==============================================

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


  // =========================================================
  // ===========  DOFs control layout  ===================
  // =========================================================
  pos_ctrl_chbx = new QCheckBox("Position control");
  pos_ctrl_chbx->setFont(font1);
  pos_ctrl_chbx->setChecked(true);
  QObject::connect( pos_ctrl_chbx, &QCheckBox::stateChanged, this, [this](){ this->main_ctrl->setPosCtrl(pos_ctrl_chbx->isChecked());} );

  orient_ctrl_chbx = new QCheckBox("Orientation control");
  orient_ctrl_chbx->setFont(font1);
  orient_ctrl_chbx->setChecked(true);
  QObject::connect( orient_ctrl_chbx, &QCheckBox::stateChanged, this, [this](){ this->main_ctrl->setOrientCtrl(orient_ctrl_chbx->isChecked());} );

  grip_ctrl_chbx = new QCheckBox("Gripper control");
  grip_ctrl_chbx->setFont(font1);
  grip_ctrl_chbx->setChecked(true);
  QObject::connect( grip_ctrl_chbx, &QCheckBox::stateChanged, this, [this](){ this->main_ctrl->setGripCtrl(grip_ctrl_chbx->isChecked());} );

  QVBoxLayout *dofs_ctrl_layout = new QVBoxLayout();
  dofs_ctrl_layout->addWidget(pos_ctrl_chbx);
  dofs_ctrl_layout->addWidget(orient_ctrl_chbx);
  dofs_ctrl_layout->addWidget(grip_ctrl_chbx);

  // ==============================================
  // ===========  Utils layout  ===================
  // ==============================================
  goto_null_pose_btn = new QPushButton("Goto null pose");
  goto_null_pose_btn->setFont(font1);
  QObject::connect( goto_null_pose_btn, &QPushButton::pressed, this, [this](){ this->main_ctrl->gotoNullPose();} );

  QVBoxLayout *utils_layout = new QVBoxLayout();
  utils_layout->addWidget(goto_null_pose_btn);

  // =============================================
  // ===========  Main Layout  ===================
  // =============================================

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addLayout(dofs_ctrl_layout);
  main_layout->addLayout(utils_layout);

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
  // file_menu->addAction(train_win->load_train_data_act);
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
