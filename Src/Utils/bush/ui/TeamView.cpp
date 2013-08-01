#include "Utils/bush/ui/TeamView.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/ui/RobotView.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Filesystem.h"

#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>

void TeamView::init()
{
  if(team)
  {
    QFormLayout* layout = new QFormLayout();
    QGridLayout* settingsGrid = new QGridLayout();

    cbColor = new QComboBox(this);
    cbColor->addItem("red");
    cbColor->addItem("blue");
    cbColor->setCurrentIndex(cbColor->findText(fromString(team->color)));
    settingsGrid->addWidget(new QLabel("color:", cbColor), 0, 0);
    settingsGrid->addWidget(cbColor, 0, 1);
    connect(cbColor, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(colorChanged(const QString&)));

    sbNumber = new QSpinBox(this);
    sbNumber->setRange(0, 50);
    sbNumber->setValue(team->number);
    settingsGrid->addWidget(new QLabel("number:", sbNumber), 0, 2);
    settingsGrid->addWidget(sbNumber, 0, 3);
    connect(sbNumber, SIGNAL(valueChanged(int)), this, SLOT(numberChanged(int)));

    lePort = new QLineEdit(QString::number(team->port), this);
    lePort->setMinimumWidth(60);
    settingsGrid->addWidget(new QLabel("port:", lePort), 0, 4);
    settingsGrid->addWidget(lePort, 0, 5);
    connect(lePort, SIGNAL(textChanged(const QString&)), this, SLOT(portChanged(const QString&)));

    cbLocation = new QComboBox(this);
    std::vector<std::string> locations = Filesystem::getLocations();
    for(size_t i = 0; i < locations.size(); ++i)
      cbLocation->addItem(fromString(locations[i]));
    cbLocation->setCurrentIndex(cbLocation->findText(fromString(team->location)));
    settingsGrid->addWidget(new QLabel("location:", lePort), 0, 6);
    settingsGrid->addWidget(cbLocation, 0, 7);
    connect(cbLocation, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(locationChanged(const QString&)));

    cbWlanConfig = new QComboBox(this);
    std::vector<std::string> configs = Filesystem::getWlanConfigs();
    for(size_t i = 0; i < configs.size(); ++i)
      cbWlanConfig->addItem(fromString(configs[i]));
    cbWlanConfig->setCurrentIndex(cbWlanConfig->findText(fromString(team->wlanConfig)));
    settingsGrid->addWidget(new QLabel("wlanConfig:", cbWlanConfig), 0, 8);
    settingsGrid->addWidget(cbWlanConfig, 0, 9);
    connect(cbWlanConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(wlanConfigChanged(const QString&)));

    cbBuildConfig = new QComboBox(this);
    cbBuildConfig->addItem("Develop");
    cbBuildConfig->addItem("Release");
    cbBuildConfig->addItem("Debug");
    cbBuildConfig->addItem("Operate");
    cbBuildConfig->setCurrentIndex(cbBuildConfig->findText(fromString(team->buildConfig)));
    settingsGrid->addWidget(new QLabel("buildConfig:", cbBuildConfig), 0, 10);
    settingsGrid->addWidget(cbBuildConfig, 0, 11);
    connect(cbBuildConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(buildConfigChanged(const QString&)));

    layout->addRow(settingsGrid);
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    layout->addRow(hr);

    QGridLayout* teamGrid = new QGridLayout();
    generateRobotViews(teamGrid);
    layout->addRow(teamGrid);

    setLayout(layout);
  }
}

TeamView::TeamView(TeamSelector* parent, Team* team)
  : QFrame(parent),
    teamSelector(parent),
    team(team),
    robotViews(),
    cbColor(0),
    sbNumber(0),
    lePort(0),
    cbLocation(0),
    cbWlanConfig(0),
    cbBuildConfig(0)
{
  init();
}

void TeamView::generateRobotViews(QGridLayout* teamGrid)
{
  std::vector<std::vector<Robot*> > robots = team->getPlayersPerNumber();
  size_t max = robots.size();
  bool backup = true;
  for(size_t j = 0; j < 2; ++j)
    for(size_t i = 0; i < max; ++i)
    {
      RobotView* rv = new RobotView(teamSelector, robots[i][j], i + 1, j);
      robotViews.push_back(rv);
      teamGrid->addWidget(rv, j > 0 ? 2 : 0, i);
    }
  if(backup)
  {
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    teamGrid->addWidget(hr, 1, 0, 1, max);
  }
}

void TeamView::update(size_t index)
{
  robotViews[index]->update();
}

void TeamView::colorChanged(const QString& color)
{
  if(team)
    team->color = toString(color);
}

void TeamView::numberChanged(int number)
{
  if(team)
    team->number = number;
}

void TeamView::portChanged(const QString& port)
{
  if(team)
    team->port = port.toUShort();
}

void TeamView::locationChanged(const QString& location)
{
  if(team)
    team->location = toString(location);
}

void TeamView::wlanConfigChanged(const QString& config)
{
  if(team)
    team->wlanConfig = toString(config);
}

void TeamView::buildConfigChanged(const QString& build)
{
  if(team)
    team->buildConfig = toString(build);
}
