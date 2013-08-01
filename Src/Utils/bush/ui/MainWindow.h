#pragma once

#include <QMainWindow>
#include <QSplitter>

class TeamSelector;
class ShortcutBar;
class Console;
class RobotPool;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow();
  ~MainWindow();
private:
  TeamSelector *teamSelector;
  ShortcutBar *shortcutBar;
  Console *console;
  RobotPool *robotPool;
  QSplitter *splitter;
  QSplitter *hSplitter;

  QAction *aLoadTeams;
  QAction *aSaveTeams;
  QAction *aAddTeam;
  QAction *aAddTeamsFromFile;
  QAction *aRemoveTeam;

private slots:
  void teamActionTriggered(QAction *action);
};
