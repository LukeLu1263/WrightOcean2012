#include <QTreeWidget>
#include <QHeaderView>
#include <QSettings>
#include <QAction>
#include <QContextMenuEvent>
#include <QMenu>
#include <QApplication>

#include "SceneGraphDockWidget.h"
#include "MainWindow.h"
#ifdef MACOSX
#include "MacStyle.h"
#endif

SceneGraphDockWidget::SceneGraphDockWidget(QMenu* contextMenu, QWidget* parent) : QDockWidget(parent), contextMenu(contextMenu)
{
  setAllowedAreas(Qt::TopDockWidgetArea);
  setFocusPolicy(Qt::ClickFocus);
  setObjectName(".SceneGraph");
  setWindowTitle(tr("Scene Graph"));
  treeWidget = new QTreeWidget(this);
  italicFont = treeWidget->font();
  italicFont.setItalic(true);
  boldFont = treeWidget->font();
  boldFont.setBold(true);
  treeWidget->setFrameStyle(QFrame::NoFrame);
  setWidget(treeWidget);
  setFocusProxy(treeWidget);
#if (QT_VERSION >= QT_VERSION_CHECK(4, 4, 0))
  treeWidget->setExpandsOnDoubleClick(false);
#endif
  treeWidget->header()->hide();

  connect(treeWidget, SIGNAL(activated(const QModelIndex&)), this, SLOT(itemActivated(const QModelIndex&)));
  connect(treeWidget, SIGNAL(collapsed(const QModelIndex&)), this, SLOT(itemCollapsed(const QModelIndex&)));
  connect(treeWidget, SIGNAL(expanded(const QModelIndex&)), this, SLOT(itemExpanded(const QModelIndex&)));

  // load layout settings
  QSettings& settings = MainWindow::application->getLayoutSettings();
  settings.beginGroup(".SceneGraph");
  expandedItems = QSet<QString>::fromList(settings.value("ExpandedItems").toStringList());
  settings.endGroup();
  
  // buttons are on the left in Mac OS X
#ifdef MACOSX
  setStyleSheet(BUTTONS_LEFT);
#endif
}

SceneGraphDockWidget::~SceneGraphDockWidget()
{
  // save layout settings
  QSettings& settings = MainWindow::application->getLayoutSettings();
  settings.beginGroup(".SceneGraph");
  settings.setValue("ExpandedItems", QStringList(expandedItems.values()));
  settings.endGroup();
}

void SceneGraphDockWidget::registerObject(const SimRobot::Module* module, SimRobot::Object* object, const SimRobot::Object* parent, int flags)
{
  QTreeWidgetItem* parentItem = parent ? registeredObjectsByObject.value(parent) : treeWidget->invisibleRootItem();
  RegisteredObject* newItem = new RegisteredObject(module, object, parentItem, flags);
  int parentFullNameLength = parent ? ((RegisteredObject*)parentItem)->fullName.length() : 0;
  newItem->setText(0, parent ? newItem->fullName.mid(parentFullNameLength + 1) : newItem->fullName);
  const QIcon* icon = object->getIcon();
  if(icon)
    newItem->setIcon(0, *icon);
  if(flags & SimRobot::Flag::hidden)
    newItem->setHidden(true);
  if(flags & SimRobot::Flag::windowless)
    newItem->setFont(0, italicFont);
  else
    newItem->setDisabled(true);
  parentItem->addChild(newItem);
  if(!parent)
    parentItem->sortChildren(0, Qt::AscendingOrder);
  if(expandedItems.contains(newItem->fullName))
    treeWidget->expandItem(newItem);

  registeredObjectsByObject.insert(object, newItem);

  int kind = object->getKind();
  QHash<QString, RegisteredObject*>* registeredObjectsByName = registeredObjectsByKindAndName.value(kind);
  if(!registeredObjectsByName)
  {
    registeredObjectsByName = new QHash<QString, RegisteredObject*>();
    registeredObjectsByKindAndName.insert(kind, registeredObjectsByName);
  }

  registeredObjectsByName->insert(newItem->fullName, newItem);
}

void SceneGraphDockWidget::unregisterAllObjects()
{
  QMutableHashIterator<const void*, RegisteredObject*> roboIt(registeredObjectsByObject);
  roboIt.toFront();
  while(roboIt.hasNext())
  {
    roboIt.next();
    if (!roboIt.value()->module->isStatic())
      roboIt.remove();
  }
  
  QMutableHashIterator<int, QHash<QString, RegisteredObject*>*> robkanIt(registeredObjectsByKindAndName);
  robkanIt.toFront();
  while(robkanIt.hasNext())
  {
    robkanIt.next();
    QHash<QString, RegisteredObject*>* robn = robkanIt.value();
    QMutableHashIterator<QString, RegisteredObject*> robnIt(*robn);
    robnIt.toFront();
    while(robnIt.hasNext())
    {
      robnIt.next();
      if(!robnIt.value()->module->isStatic())
        robnIt.remove();
    }
  }
  
  int index = treeWidget->topLevelItemCount() - 1;
  while(index >= 0)
  {
    RegisteredObject* ro = (RegisteredObject*) treeWidget->topLevelItem(index);
    if(!ro->module->isStatic())
    {
      QTreeWidgetItem* topLevelItem = treeWidget->takeTopLevelItem(index);
      qDeleteAll(topLevelItem->takeChildren());
      index = treeWidget->topLevelItemCount() - 1;
      delete topLevelItem;
    } else --index;
  }  
//  registeredObjectsByObject.clear();
//  qDeleteAll(registeredObjectsByKindAndName);
//  registeredObjectsByKindAndName.clear();
//  treeWidget->clear();
}

void SceneGraphDockWidget::unregisterObjectsFromModule(const SimRobot::Module* module)
{
  for(int i = treeWidget->topLevelItemCount() - 1; i >= 0; --i)
    deleteRegisteredObjectsFromModule((RegisteredObject*)treeWidget->topLevelItem(i), module);
}

bool SceneGraphDockWidget::unregisterObject(const SimRobot::Object* object)
{
  RegisteredObject* regObject = registeredObjectsByObject.value(object);
  if(!regObject)
    return false;
  deleteRegisteredObject(regObject);
  return true;
}

SimRobot::Object* SceneGraphDockWidget::resolveObject(const QString& fullName, int kind)
{
  QHash<QString, RegisteredObject*>* registeredObjectsByName = registeredObjectsByKindAndName.value(kind);
  if(!registeredObjectsByName)
    return 0;
  RegisteredObject* object = registeredObjectsByName->value(fullName);
  if(!object)
    return 0;
  return object->object;
}

SimRobot::Object* SceneGraphDockWidget::resolveObject(const SimRobot::Object* parent, const QVector<QString>& parts, int kind)
{
  const int partsCount = parts.count();
  if(partsCount <= 0)
    return 0;
  QHash<QString, RegisteredObject*>* registeredObjectsByName = registeredObjectsByKindAndName.value(kind);
  if(!registeredObjectsByName)
    return 0;
  const QString& lastPart = parts.at(partsCount - 1);
  foreach(RegisteredObject* object, *registeredObjectsByName)
  {
    if(object->fullName.endsWith(lastPart))
    {
      RegisteredObject* currentObject = object;
      for(int i = partsCount - 2; i >= 0; --i)
      {
        currentObject = (RegisteredObject*)currentObject->parent();
        const QString& currentPart = parts.at(i);
        for(;;)
        {
          if(!currentObject)
            goto continueSearch;
          if(currentObject->fullName.endsWith(currentPart))
            break;
          currentObject = (RegisteredObject*)currentObject->parent();
        }
      }
      if(parent)
      {
        currentObject = (RegisteredObject*)currentObject->parent();
        for(;;)
        {
          if(!currentObject)
            goto continueSearch;
          if(currentObject->object == parent)
            break;
          currentObject = (RegisteredObject*)currentObject->parent();
        }
      }
      return object->object;
    }
continueSearch:
    ;
  }
  return 0;
}

int SceneGraphDockWidget::getObjectChildCount(const SimRobot::Object* object)
{
  RegisteredObject* item = registeredObjectsByObject.value(object);
  return item ? item->childCount() : 0;
}

SimRobot::Object* SceneGraphDockWidget::getObjectChild(const SimRobot::Object* object, int index)
{
  RegisteredObject* item = registeredObjectsByObject.value(object);
  return item && index >= 0 && index < item->childCount() ? ((RegisteredObject*)item->child(index))->object : 0;
}

bool SceneGraphDockWidget::activateFirstObject()
{
  RegisteredObject* item = (RegisteredObject*)treeWidget->invisibleRootItem()->child(0);
  if(!item)
    return false;
  emit activatedObject(item->object->getFullName(), item->module, item->object, item->flags);
  return true;
}

bool SceneGraphDockWidget::activateObject(const SimRobot::Object* object)
{
  RegisteredObject* item = registeredObjectsByObject.value(object);
  if(!item)
    return false;
  emit activatedObject(item->object->getFullName(), item->module, item->object, item->flags);
  return true;
}

bool SceneGraphDockWidget::setOpened(const SimRobot::Object* object, bool opened)
{
  RegisteredObject* item = registeredObjectsByObject.value(object);
  if(!item)
    return false;
  item->opened = opened;
  //item->setFont(0, opened ? boldFont : QFont());
  item->setDisabled(!opened);
  if(!opened)
    item->setFont(0, QFont());
  return true;
}

bool SceneGraphDockWidget::setActive(const SimRobot::Object* object, bool active)
{
  RegisteredObject* item = registeredObjectsByObject.value(object);
  if(!item)
    return false;
  item->setFont(0, active ? boldFont : QFont());
  if(active)
    treeWidget->setCurrentItem(item);
  return true;
}

QAction *SceneGraphDockWidget::toggleViewAction() const
{
  QAction* action = QDockWidget::toggleViewAction();
  action->setIcon(QIcon(":/Icons/application_side_tree.png"));
  action->setShortcut(QKeySequence(Qt::Key_F2));
  return action;
}

void SceneGraphDockWidget::deleteRegisteredObjectsFromModule(RegisteredObject* registeredObject, const SimRobot::Module* module)
{
  if(registeredObject->module == module)
    deleteRegisteredObject(registeredObject);
  else
    for(int i = registeredObject->childCount() - 1; i >= 0; --i)
      deleteRegisteredObjectsFromModule((RegisteredObject*)registeredObject->child(i), module);
}

void SceneGraphDockWidget::deleteRegisteredObject(RegisteredObject* registeredObject)
{
  for(int i = registeredObject->childCount() - 1; i >= 0; --i)
    deleteRegisteredObject((RegisteredObject*)registeredObject->child(i));
  registeredObjectsByObject.remove(registeredObject->object);
  int kind = registeredObject->object->getKind();
  QHash<QString, RegisteredObject*>* registeredObjectsByName = registeredObjectsByKindAndName.value(kind);
  registeredObjectsByName->remove(registeredObject->object->getFullName());
  if(registeredObjectsByName->count() == 0)
  {
    registeredObjectsByKindAndName.remove(kind);
    delete registeredObjectsByName;
  }
  delete registeredObject;
}

void SceneGraphDockWidget::contextMenuEvent(QContextMenuEvent* event)
{
  QRect content(treeWidget->geometry());
  if(!content.contains(event->x(), event->y()))
  { // click on window frame
    QDockWidget::contextMenuEvent(event);
    return;
  };

  clickedItem = (RegisteredObject*)treeWidget->itemAt(treeWidget->mapFromParent(event->pos()));

  //
  QMenu menu; 
  if(clickedItem)
  {
    if(!(clickedItem->flags & SimRobot::Flag::windowless))
    {
      QAction* action = menu.addAction(tr(clickedItem->opened ? "&Close" : "&Open"));
      connect(action, SIGNAL(triggered()), this, SLOT(openOrCloseObject()));
      menu.addSeparator();
    }
    if(clickedItem->childCount() > 0)
    {
      QAction* action = menu.addAction(tr(clickedItem->isExpanded() ? "Collabs&e" : "&Expand"));
      connect(action, SIGNAL(triggered()), this, SLOT(expandOrCollabseObject()));
      menu.addSeparator();
    }
  }
  menu.addAction(contextMenu->menuAction());
  event->accept();
  menu.exec(mapToGlobal(QPoint(event->x(), event->y())));
  clickedItem = 0;
}

void SceneGraphDockWidget::itemActivated(const QModelIndex& index)
{
  RegisteredObject* item = (RegisteredObject*)index.internalPointer();
  if(item->flags & SimRobot::Flag::windowless)
  {
    if(item->isExpanded())
      treeWidget->collapseItem(item);
    else
      treeWidget->expandItem(item);
  }
  else
    emit activatedObject(item->object->getFullName(), item->module, item->object, item->flags);
}

void SceneGraphDockWidget::itemCollapsed(const QModelIndex& index)
{
  RegisteredObject* item = (RegisteredObject*)index.internalPointer();
  expandedItems.remove(item->object->getFullName());
}

void SceneGraphDockWidget::itemExpanded(const QModelIndex& index)
{
  RegisteredObject* item = (RegisteredObject*)index.internalPointer();
  expandedItems.insert(item->object->getFullName());
}

void SceneGraphDockWidget::openOrCloseObject()
{
  if(clickedItem->opened)
    emit deactivatedObject(clickedItem->object->getFullName());
  else
    emit activatedObject(clickedItem->object->getFullName(), clickedItem->module, clickedItem->object, clickedItem->flags);
}

void SceneGraphDockWidget::expandOrCollabseObject()
{
  if(clickedItem->isExpanded())
    treeWidget->collapseItem(clickedItem);
  else
    treeWidget->expandItem(clickedItem);
}