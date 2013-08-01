/**
* @file Controller/Views/ConsoleView.cpp
* Implementation of class ConsoleView
* @author Colin Graf
*/

#include <QScrollBar>
#include <QKeyEvent>
#include <QMenu>
#include <QSettings>

#include "ConsoleView.h"
#include "Platform/BHAssert.h"
#include "Controller/ConsoleRoboCupCtrl.h"

SimRobot::Widget* ConsoleView::createWidget()
{
  ASSERT(!consoleWidget);
  consoleWidget = new ConsoleWidget(*this, console, output, consoleWidget);
  output.clear();
  return consoleWidget;
}

void ConsoleView::clear()
{
  output.clear();
  if(consoleWidget)
    consoleWidget->setPlainText(QString());
}

void ConsoleView::printLn(const QString& text)
{
  print(text + "\n");
}

void ConsoleView::print(const QString& text)
{
  if(consoleWidget)
    consoleWidget->print(text);
  else
    output += text;
}

ConsoleWidget::ConsoleWidget(ConsoleView& consoleView, ConsoleRoboCupCtrl& console, QString& output, ConsoleWidget*& consoleWidget) :
  consoleView(consoleView), console(console), output(output), consoleWidget(consoleWidget),
  saveAct(0), undoAct(0), redoAct(0), cutAct(0), copyAct(0), pasteAct(0), deleteAct(0),
  canCopy(false), canUndo(false), canRedo(false)
{
  setFrameStyle(QFrame::NoFrame);
  setAcceptRichText(false);

  if(consoleView.loadAndSaveOutput)
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(consoleView.fullName);
    output = settings.value("Output").toString();
    settings.endGroup();
  }

  setPlainText(output);
  output.clear();
  QTextCursor cursor = textCursor();
  cursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
  setTextCursor(cursor);

  connect(this, SIGNAL(copyAvailable(bool)), this, SLOT(copyAvailable(bool)));
  connect(this, SIGNAL(undoAvailable(bool)), this, SLOT(undoAvailable(bool)));
  connect(this, SIGNAL(redoAvailable(bool)), this, SLOT(redoAvailable(bool)));

}

ConsoleWidget::~ConsoleWidget()
{
  output = toPlainText();
  consoleWidget = 0;

  if(consoleView.loadAndSaveOutput)
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(consoleView.fullName);
    settings.setValue("Output", output);
    settings.endGroup();
    output.clear();
  }
}

void ConsoleWidget::print(const QString& text)
{
  QScrollBar* scrollBar = verticalScrollBar();
  bool scroll = scrollBar->value() == scrollBar->maximum();
  QTextCursor cursor = textCursor();
  cursor.movePosition(QTextCursor::StartOfBlock);
  /*
  cursor.movePosition(QTextCursor::NextBlock);
  if(cursor.atEnd() && !cursor.atStart())
  {
    cursor.insertText("\n");
    cursor.movePosition(QTextCursor::NextBlock);
  }*/
  cursor.insertText(text);
  if(scroll)
    scrollBar->setValue(scrollBar->maximum());
  cursor.movePosition(QTextCursor::End);
}


QMenu* ConsoleWidget::createEditMenu()
{
  QMenu* menu = new QMenu(tr("&Edit"));

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
  action->setStatusTip(tr("Select the whole text"));
  connect(action, SIGNAL(triggered()), this, SLOT(selectAll()));

  return menu;
}


void ConsoleWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
  case Qt::Key_Tab:
  case Qt::Key_Backtab:
    event->accept();
    {
      QTextCursor cursor = textCursor();
      int begin = cursor.position();
      int end = cursor.anchor();
      if(end < begin)
      {
        int tmp = end;
        end = begin;
        begin = tmp;
      }
      cursor.setPosition(end);
      cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
      QString line = cursor.selectedText();
      int lineLength = begin - cursor.position();
      std::string command(line.toAscii().constData());
      console.completeConsoleCommand(command, event->key() == Qt::Key_Tab, begin == end);
      line = command.c_str();
      cursor.insertText(line);
      cursor.setPosition(begin);
      cursor.setPosition(begin + (line.length() - lineLength), QTextCursor::KeepAnchor);
      setTextCursor(cursor);
    }
    break;

  case Qt::Key_Return:
  case Qt::Key_Enter:
    if(event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier))
    {
      QTextCursor cursor = textCursor();
      cursor.insertBlock();
      setTextCursor(cursor);
    }
    else
    {
      event->accept();
      {
        QTextCursor cursor = textCursor();
        cursor.movePosition(QTextCursor::StartOfBlock);
        cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
        QString line = cursor.selectedText();
        cursor.movePosition(QTextCursor::EndOfLine);
        if(cursor.atEnd())
          cursor.insertText("\n");
        cursor.movePosition(QTextCursor::NextBlock);
        setTextCursor(cursor);

        console.executeConsoleCommand(line.toAscii().constData());

        // stores unix like history entry
        history.removeAll(line);
        history.append(line);
        history_iter = history.end();
      }
    }
    break;

    // History browsing keys
  case Qt::Key_Up:
    if((event->modifiers() & Qt::ControlModifier)
       && !history.isEmpty() && history_iter != history.begin())
    {
      event->accept();
      history_iter--;
      QTextCursor cursor = textCursor();
      cursor.movePosition(QTextCursor::End);
      cursor.movePosition(QTextCursor::StartOfBlock, QTextCursor::KeepAnchor);
      cursor.removeSelectedText();
      cursor.insertText(*history_iter);
      setTextCursor(cursor);
    }
    else
    {
      QTextEdit::keyPressEvent(event);
    }
    break;
  case Qt::Key_Down:
    if(event->modifiers() & Qt::ControlModifier)
    {
      event->accept();
      QTextCursor cursor = textCursor();
      cursor.movePosition(QTextCursor::End);
      cursor.movePosition(QTextCursor::StartOfBlock, QTextCursor::KeepAnchor);
      cursor.removeSelectedText();
      if(!history.isEmpty() && history_iter != history.end())
      {
        history_iter++;
        if(history_iter != history.end())
        {
          cursor.insertText(*history_iter);
          setTextCursor(cursor);
        }
      }
    }
    else
    {
      QTextEdit::keyPressEvent(event);
    }
    break;

  default:
    QTextEdit::keyPressEvent(event);
    break;
  }
}

void ConsoleWidget::contextMenuEvent(QContextMenuEvent* event)
{
  QWidget::contextMenuEvent(event);
}

void ConsoleWidget::copyAvailable(bool available)
{
  canCopy = available;
  if(copyAct)
    copyAct->setEnabled(available);
  if(cutAct)
    cutAct->setEnabled(available);
  if(deleteAct)
    deleteAct->setEnabled(available);
}

void ConsoleWidget::redoAvailable(bool available)
{
  canRedo = available;
  if(redoAct)
    redoAct->setEnabled(available);
}

void ConsoleWidget::undoAvailable(bool available)
{
  canUndo = available;
  if(undoAct)
    undoAct->setEnabled(available);
}

void ConsoleWidget::cut()
{
  QTextEdit::cut();
  if(pasteAct)
    pasteAct->setEnabled(canPaste());
}

void ConsoleWidget::copy()
{
  QTextEdit::copy();
  if(pasteAct)
    pasteAct->setEnabled(canPaste());
}

void ConsoleWidget::deleteText()
{
  insertPlainText(QString());
}
