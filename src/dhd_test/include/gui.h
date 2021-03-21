#ifndef GUI_MAINWINDOW_H
#define GUI_MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMenu>
#include <QPushButton>
#include <QLabel>
#include <QAction>
#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPalette>
#include <QColor>
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QCheckBox>
#include <QThread>

#include <map>
#include <functional>

#include <gui_lib/utils.h>
#include <gui_lib/view_pose_dialog.h>
#include <gui_lib/view_wrench_dialog.h>
#include <gui_lib/view_jpos_dialog.h>

using namespace as64_;

class MainCtrl; // forward declaration

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:

  explicit MainWindow(MainCtrl *main_ctrl, QWidget *parent = 0);
  ~MainWindow();

signals:
  void closeSignal();
  void showInfoMsgSignal(const char *msg);
  void showWarnMsgSignal(const char *msg);
  void showErrMsgSignal(const char *msg);
  void replayTrajStoppedSignal();

private slots:
  void showInfoMsg(const char *msg);
  void showWarnMsg(const char *msg);
  void showErrMsg(const char *msg);

public:

  MainCtrl *main_ctrl;

  QCheckBox *pos_ctrl_chbx;
  QCheckBox *orient_ctrl_chbx;
  QCheckBox *grip_ctrl_chbx;

  gui_::ViewWrenchDialog *view_wrench_dialog;
  QAction *view_wrench_act;

  gui_::ViewPoseDialog *view_pose_dialog;
  QAction *view_pose_act;

  gui_::ViewJPosDialog *view_wrist_joints_dialog;
  QAction *view_wrist_joints_act;

  QPushButton *goto_null_pose_btn;

  QPushButton *start_rec_btn;
  QPushButton *stop_rec_btn;
  QPushButton *start_replay_traj_btn;
  QPushButton *stop_replay_traj_btn;

  void createMenu();

  QWidget *central_widget;
  QStatusBar *status_bar;

};

#endif // GUI_MAINWINDOW_H
