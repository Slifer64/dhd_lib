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

public:

  MainCtrl *main_ctrl;

  QCheckBox *pos_ctrl_chbx;
  QCheckBox *orient_ctrl_chbx;
  QCheckBox *grip_ctrl_chbx;

  gui_::ViewWrenchDialog *view_wrench_dialog;
  QAction *view_wrench_act;

  gui_::ViewPoseDialog *view_pose_dialog;
  QAction *view_pose_act;

  void createMenu();

  QWidget *central_widget;
  QStatusBar *status_bar;

};

#endif // GUI_MAINWINDOW_H
