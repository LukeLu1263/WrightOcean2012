#include "Platform/File.h"
#include <QBoxLayout>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>
#include <QPixmap>
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/ui/MainWindow.h"
#include "Utils/bush/ui/RobotPool.h"
#include "Utils/bush/ui/ShortcutBar.h"
#include "Utils/bush/ui/TeamSelector.h"

MainWindow::MainWindow()
  : teamSelector(new TeamSelector()),
    shortcutBar(0),
    console(new Console(teamSelector)),
    robotPool(0),
    splitter(new QSplitter(Qt::Vertical)),
    hSplitter(new QSplitter(Qt::Horizontal))
{
  splitter->addWidget(teamSelector);
  splitter->addWidget(console);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  shortcutBar = new ShortcutBar(console);
  addToolBar(Qt::BottomToolBarArea, shortcutBar);
  shortcutBar->addShortcut("help", "help", "help");
  shortcutBar->addShortcut("deploy", "deploy", "browser-download");
  shortcutBar->addShortcut("restart", "restart", "reload");
  shortcutBar->addShortcut("ping", "ping", "network");

  hSplitter->addWidget(splitter);
  robotPool = new RobotPool(teamSelector);
  connect(teamSelector, SIGNAL(currentChanged(int)), robotPool, SLOT(update()));
  QFrame *rightSide = new QFrame(this);
  QGridLayout *rsLayout = new QGridLayout(rightSide);
  rsLayout->addWidget(new QLabel("<b>Robot Pool:</b>"), 0, 0);
  rsLayout->addWidget(robotPool, 1, 0);
  rightSide->setLayout(rsLayout);
  hSplitter->addWidget(rightSide);
  hSplitter->setSizes(QList<int>() << -1 << 0);
  setCentralWidget(hSplitter);

  this->setWindowIcon(QPixmap(":icons/bush.png"));
  console->setFocus(Qt::OtherFocusReason);

  QMenuBar *_menuBar = menuBar();
  QMenu *teamMenu = _menuBar->addMenu("&Team");
  aLoadTeams = teamMenu->addAction("&Load Teams");
  aLoadTeams->setShortcuts(QKeySequence::Open);
  aSaveTeams = teamMenu->addAction("&Save Teams");
  aSaveTeams->setShortcuts(QKeySequence::Save);
  teamMenu->addSeparator();
  aAddTeam = teamMenu->addAction("&Add Team");
  aAddTeamsFromFile = teamMenu->addAction("Add Teams from &File");
  aRemoveTeam = teamMenu->addAction("&Remove Team");

  connect(teamMenu, SIGNAL(triggered(QAction*)), this, SLOT(teamActionTriggered(QAction*)));

  teamSelector->loadTeams();
}

MainWindow::~MainWindow()
{
  delete console;
  delete splitter;
}

void MainWindow::teamActionTriggered(QAction *action)
{
  if (!action) return;
  QString filter = "ConfigMaps (*.cfg);;all (*.*)";
  if (action == aLoadTeams || action == aAddTeamsFromFile)
  {
    QString fileName = QFileDialog::getOpenFileName(this, "Load Teams", File::getBHDir(), filter);
    if (!fileName.isNull())
      teamSelector->loadTeams(fileName, action != aAddTeamsFromFile);
  }
  else if (action == aSaveTeams)
  {
    QString fileName = QFileDialog::getSaveFileName(this, "Save Teams", File::getBHDir(), filter);
    if (!fileName.isNull())
      teamSelector->saveTeams(fileName);
  }
  else if (action == aRemoveTeam)
  {
    teamSelector->removeTeam(teamSelector->getSelectedTeam());
  }
  //TODO: implement aAddTeam
}
