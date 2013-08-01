/** 
* @file SimRobotEditor/EditorWidget.cpp
* Implementation of class EditorWidget
* @author Colin Graf
*/

#include <QMenu>
#include <QSet>
#include <QFileInfo>
#include <QTextStream>
#include <QMessageBox>
#include <QSettings>
#include <QScrollBar>
#include <QResizeEvent>

#include "EditorModule.h"
#include "EditorWidget.h"

EditorObject::EditorObject(const QString& name, EditorObject* parent) : 
  parent(parent), name(name), fullName(parent ? parent->fullName + "." + name : name) {}

const QIcon* EditorObject::getIcon() const
{
  return &EditorModule::module->folderIcon;
}

SimRobotEditor::Editor* EditorObject::addFile(const QString& filePath, const QString& subFileRegExpPattern)
{
  return addEditor(filePath, subFileRegExpPattern, true);
}

SimRobotEditor::Editor* EditorObject::addEditor(const QString& filePath, const QString& subFileRegExpPattern, bool persistent)
{
  FileEditorObject* editor = EditorModule::module->findEditor(filePath);
  if(editor)
  {
    if(persistent)
      editor->persistent = true;
    return editor;
  }
  editor = new FileEditorObject(filePath, subFileRegExpPattern, persistent, this);
  editors.append(editor);
  EditorModule::module->registerEditor(editor);
  EditorModule::application->registerObject(*EditorModule::module, *editor, this);
  return editor;
}

SimRobotEditor::Editor* EditorObject::addFolder(const QString& name)
{
  EditorObject* folder = foldersByName.value(name);
  if(folder)
    return folder;
  folder = new EditorObject(name, this);
  foldersByName.insert(name, folder);
  editors.append(folder);
  EditorModule::application->registerObject(*EditorModule::module, *folder, this, SimRobot::Flag::windowless);
  return folder;
}

void EditorObject::removeEditor(FileEditorObject* editor)
{
  if(editor->persistent)
    return;

  if(!editors.removeOne(editor))
    return;

  EditorModule::application->unregisterObject(*editor);
  EditorModule::module->unregisterEditor(editor);
  delete editor;
}

void EditorObject::loadFromSettings()
{
  QSettings& settings = EditorModule::application->getLayoutSettings();
  int count = 1;
  for(int i = 0; i < count; ++i)
  {
    count = settings.beginReadArray(fullName);
    if(i >= count)
    {
      settings.endArray();
      break;
    }
    settings.setArrayIndex(i);
    QString filePath = settings.value("filePath").toString();
    if(filePath.isEmpty())
    {
      QString name = settings.value("name").toString();
      settings.endArray();
      ((EditorObject*)addFolder(name))->loadFromSettings();
    }
    else
    {
      QString subFileRegExpPattern = settings.value("subFileRegExpPattern").toString();
      settings.endArray();
      ((EditorObject*)addEditor(filePath, subFileRegExpPattern, false))->loadFromSettings();
    }
  }
}

EditorObject::~EditorObject()
{
  QSettings& settings = EditorModule::application->getLayoutSettings();
  settings.beginWriteArray(fullName, editors.size());
  int i = 0;
  foreach(EditorObject* editor, editors)
  {
    settings.setArrayIndex(i++);
    FileEditorObject* fileEditorObject = dynamic_cast<FileEditorObject*>(editor);
    if(!fileEditorObject)
    {
      settings.setValue("filePath", QString());
      settings.setValue("name", editor->name);
    }
    else
    {
      settings.setValue("filePath", fileEditorObject->filePath);
      settings.setValue("subFileRegExpPattern", fileEditorObject->subFileRegExpPattern);
    }
  }
  settings.endArray();
  qDeleteAll(editors);
}

FileEditorObject::FileEditorObject(const QString& filePath, const QString& subFileRegExpPattern, bool persistent, EditorObject* parent) :
  EditorObject(QFileInfo(filePath).fileName(), parent), filePath(filePath), subFileRegExpPattern(subFileRegExpPattern), persistent(persistent) {}

const QIcon* FileEditorObject::getIcon() const
{
  return &EditorModule::module->fileIcon;
}

SimRobot::Widget* FileEditorObject::createWidget()
{
  QFile file(filePath);
  if(!file.open(QFile::ReadOnly | QFile::Text))
  {
    EditorModule::application->showWarning(QObject::tr("SimRobotEditor"), QObject::tr("Cannot read file %1:\n%2.").arg(filePath).arg(file.errorString()));
    return 0;
  }
  QTextStream in(&file);
  return new EditorWidget(this, in.readAll());
}

EditorWidget::EditorWidget(FileEditorObject* editorObject, const QString& fileContent) : 
  editorObject(editorObject),
  saveAct(0), undoAct(0), redoAct(0), cutAct(0), copyAct(0), pasteAct(0), deleteAct(0),
  canCopy(false), canUndo(false), canRedo(false),
  highlighter(0)
{
  if(editorObject->filePath.endsWith(".ros") || editorObject->filePath.endsWith(".rsi") ||
    editorObject->filePath.endsWith(".ros2") || editorObject->filePath.endsWith(".rsi2"))
    highlighter = new SyntaxHighlighter(document());
  setFrameStyle(QFrame::NoFrame);
  
#ifdef WIN32
  QFont font("Courier New", 10);
#elif defined(MACOSX)
  QFont font("Monaco", 11);
#else
  QFont font("Bitstream Vera Sans Mono", 9);
#endif
  setFont(font);
  setLineWrapMode(QTextEdit::NoWrap);
  setAcceptRichText(false);
  setPlainText(fileContent);
  document()->setModified(false);

  QSettings& settings = EditorModule::application->getLayoutSettings();
  settings.beginGroup(editorObject->fullName);  
  int selectionStart = settings.value("selectionStart").toInt();
  int selectionEnd = settings.value("selectionEnd").toInt();
  if(selectionStart || selectionEnd)
  {
    QTextCursor cursor = textCursor();
    cursor.setPosition(selectionStart);
    cursor.setPosition(selectionEnd, QTextCursor::KeepAnchor);
    setTextCursor(cursor);
  }
  verticalScrollBar()->setValue(settings.value("verticalScrollPosition").toInt());
  horizontalScrollBar()->setValue(settings.value("horizontalScrollPosition").toInt());
  settings.endGroup();

  connect(document(), SIGNAL(modificationChanged(bool)), this, SLOT(saveAvailable(bool)));
  connect(this, SIGNAL(copyAvailable(bool)), this, SLOT(copyAvailable(bool)));
  connect(this, SIGNAL(undoAvailable(bool)), this, SLOT(undoAvailable(bool)));
  connect(this, SIGNAL(redoAvailable(bool)), this, SLOT(redoAvailable(bool)));
  connect(&openFileMapper, SIGNAL(mapped(const QString&)), this, SLOT(openFile(const QString&)));
}

EditorWidget::~EditorWidget()
{
  QSettings& settings = EditorModule::application->getLayoutSettings();
  settings.beginGroup(editorObject->fullName);
  QTextCursor cursor = textCursor();
  settings.setValue("selectionStart", cursor.anchor());
  settings.setValue("selectionEnd", cursor.position());  
  settings.setValue("windowSize", size());
  settings.setValue("verticalScrollPosition", verticalScrollBar()->value());
  settings.setValue("horizontalScrollPosition", horizontalScrollBar()->value());
  settings.endGroup();

  if(!EditorModule::module->application->getFilePath().isEmpty()) // !closingDocument
    editorObject->parent->removeEditor(editorObject);
  
  if(highlighter)
    delete highlighter;
}

bool EditorWidget::canClose()
{
  if(!document()->isModified())
    return true;
  switch(QMessageBox::warning(this, tr("SimRobotEditor"), tr("Do you want to save changes to %1?").arg(editorObject->name), QMessageBox::Save  | QMessageBox::Discard | QMessageBox::Cancel))
  {
  case QMessageBox::Save:
    save();
    break;
  case QMessageBox::Discard:
    break;
  default:
    return false;
  }
  return true;
}

QMenu* EditorWidget::createFileMenu()
{
  QMenu* menu = new QMenu(tr("&File"));
  
  if(!saveAct)
  {
    saveAct = new QAction(QIcon(":/Icons/disk.png"), tr("&Save"), this); 
    saveAct->setShortcut(QKeySequence(QKeySequence::Save));
    saveAct->setStatusTip(tr("Save the document to disk"));
    saveAct->setEnabled(document()->isModified());
    connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));
  }
  menu->addAction(saveAct);

  return menu;
}

QMenu* EditorWidget::createEditMenu()
{
  QMenu* menu = new QMenu(tr("&Edit"));
  connect(menu, SIGNAL(aboutToShow()), this, SLOT(updateEditMenu()));
  updateEditMenu(menu, false);
  return menu;
}

void EditorWidget::updateEditMenu(QMenu* menu, bool aboutToShow)
{
  menu->clear();

  if(aboutToShow && !editorObject->subFileRegExpPattern.isEmpty())
  {
    QRegExp rx(editorObject->subFileRegExpPattern, Qt::CaseInsensitive);
    QString fileContent = toPlainText();
    QStringList includeFiles;
    QSet<QString> inculdeFilesSet;
    QString suffix = QFileInfo(editorObject->name).suffix();
    int pos = 0;
    while((pos = rx.indexIn(fileContent, pos)) != -1)
    {
      QString file = rx.cap(1).remove('\"');
      if(QFileInfo(file).suffix().isEmpty())
        (file += '.') += suffix;
      if(!inculdeFilesSet.contains(file))
      {
        includeFiles.append(file);
        inculdeFilesSet.insert(file);
      }
      pos += rx.matchedLength();
    }

    if(includeFiles.count() > 0)
    {
      foreach(QString str, includeFiles)
      {
        QAction* action = menu->addAction(tr("Open \"%1\"").arg(str));
        openFileMapper.setMapping(action, str);
        connect(action, SIGNAL(triggered()), &openFileMapper, SLOT(map()));
      }
      menu->addSeparator();
    }
  }

  if(!undoAct)
  {
    undoAct = new QAction(QIcon(":/Icons/arrow_undo.png"), tr("&Undo"), this);
    undoAct->setShortcut(QKeySequence(QKeySequence::Undo));
    undoAct->setStatusTip(tr("Undo the last action"));
    undoAct->setEnabled(canUndo);
    connect(undoAct, SIGNAL(triggered()), this, SLOT(undo()));
  }
  menu->addAction(undoAct);

  if(!redoAct)
  {
    redoAct = new QAction(QIcon(":/Icons/arrow_redo.png"), tr("&Redo"), this);
    redoAct->setShortcut(QKeySequence(QKeySequence::Redo));
    redoAct->setStatusTip(tr("Redo the previously undone action"));
    redoAct->setEnabled(canRedo);
    connect(redoAct, SIGNAL(triggered()), this, SLOT(redo()));
  }
  menu->addAction(redoAct);
  menu->addSeparator();

  if(!cutAct)
  {
    cutAct = new QAction(QIcon(":/Icons/cut.png"), tr("Cu&t"), this);
    cutAct->setShortcut(QKeySequence(QKeySequence::Cut));
    cutAct->setStatusTip(tr("Cut the current selection's contents to the clipboard"));
    cutAct->setEnabled(canCopy);
    connect(cutAct, SIGNAL(triggered()), this, SLOT(cut()));
  }
  menu->addAction(cutAct);

  if(!copyAct)
  {
    copyAct = new QAction(QIcon(":/Icons/page_copy.png"), tr("&Copy"), this);
    copyAct->setShortcut(QKeySequence(QKeySequence::Copy));
    copyAct->setStatusTip(tr("Copy the current selection's contents to the clipboard"));
    copyAct->setEnabled(canCopy);
    connect(copyAct, SIGNAL(triggered()), this, SLOT(copy()));
  }
  menu->addAction(copyAct);

  if(!pasteAct)
  {
    pasteAct = new QAction(QIcon(":/Icons/page_paste.png"), tr("&Paste"), this);
    pasteAct->setShortcut(QKeySequence(QKeySequence::Paste));
    pasteAct->setStatusTip(tr("Paste the clipboard's contents into the current selection"));
    connect(pasteAct, SIGNAL(triggered()), this, SLOT(paste()));
  }
  pasteAct->setEnabled(canPaste());
  menu->addAction(pasteAct);

  if(!deleteAct)
  {
    deleteAct = new QAction(tr("&Delete"), this);
    deleteAct->setShortcut(QKeySequence(QKeySequence::Delete));
    deleteAct->setStatusTip(tr("Delete the currently selected content"));
    deleteAct->setEnabled(canCopy);
    connect(deleteAct, SIGNAL(triggered()), this, SLOT(deleteText()));
  }
  menu->addAction(deleteAct);
  menu->addSeparator();

  QAction* action = menu->addAction(tr("Select &All"));
  action->setShortcut(QKeySequence(QKeySequence::SelectAll));
  action->setStatusTip(tr("Select the whole document"));
  connect(action, SIGNAL(triggered()), this, SLOT(selectAll()));
}

void EditorWidget::focusInEvent(QFocusEvent * event)
{
  QTextEdit::focusInEvent(event);

  if(pasteAct)
    pasteAct->setEnabled(canPaste());
}

void EditorWidget::contextMenuEvent(QContextMenuEvent* event)
{
  QWidget::contextMenuEvent(event);
}

void EditorWidget::updateEditMenu()
{
  QMenu* menu = qobject_cast<QMenu*>(sender());
  updateEditMenu(menu, true);
}

void EditorWidget::saveAvailable(bool available)
{
  if(saveAct)
    saveAct->setEnabled(available);
}

void EditorWidget::copyAvailable(bool available)
{
  canCopy = available;
  if(copyAct)
    copyAct->setEnabled(available);
  if(cutAct)
    cutAct->setEnabled(available);
  if(deleteAct)
    deleteAct->setEnabled(available);
}

void EditorWidget::redoAvailable(bool available)
{
  canRedo = available;
  if(redoAct)
    redoAct->setEnabled(available);
}

void EditorWidget::undoAvailable(bool available)
{
  canUndo = available;
  if(undoAct)
    undoAct->setEnabled(available);
}

void EditorWidget::save()
{
  QFile file(editorObject->filePath);
  if(!file.open(QFile::WriteOnly | QFile::Text))
  {
    EditorModule::application->showWarning(QObject::tr("SimRobotEditor"), QObject::tr("Cannot write file %1:\n%2.").arg(editorObject->filePath).arg(file.errorString()));
    return;
  }
  QTextStream out(&file);
  out << toPlainText();
  document()->setModified(false);
}

void EditorWidget::cut()
{
  QTextEdit::cut();
  if(pasteAct)
    pasteAct->setEnabled(canPaste());
}

void EditorWidget::copy()
{
  QTextEdit::copy();
  if(pasteAct)
    pasteAct->setEnabled(canPaste());
}

void EditorWidget::deleteText()
{
  insertPlainText(QString());
}

void EditorWidget::openFile(const QString& fileName)
{
  QString filePath = QFileInfo(editorObject->filePath).path() + "/" + fileName;
  editorObject->addEditor(filePath, editorObject->subFileRegExpPattern, false);
  EditorModule::module->openEditor(filePath);
}
