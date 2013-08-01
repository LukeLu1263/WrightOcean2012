#pragma once

#include <QFrame>
#include <QListWidget>
#include <QVariant>
#include "Utils/bush/cmdlib/AbstractConsole.h"

class TeamSelector;
class QLabel;
class Context;
class QFormLayout;
class QScrollArea;
class QPushButton;

class CommandLineEdit;
class Console;
class VisualContext;

class VisualContext : public QFrame
{
  Q_OBJECT

public:
  struct Entry
  {
    enum Type
    {
      TEXT_OUTPUT,
      TEXT_ERROR,
      CONTEXT,
      CONCURRENT
    } type;
    union
    {
      QString *text;
      VisualContext *context;
    };
    Entry(Type type, const QString &text)
      : type(type),
        text(new QString(text))
    { }
    Entry(VisualContext *context)
      : type(CONTEXT),
        context(context)
    { }
    ~Entry();
  };


private:
  /** The entries of the visual context. Actually they can be text or sub
   * contexts. */
  QList<Entry*> entries;

  /** The widges which visualize the entries. */
  QList<QWidget*> widgets;

  QFormLayout *formLayout;

  /** Indicates if the last printed line had a newline at their end. */
  bool nl;

  inline QFormLayout* getLayout() { return formLayout; }
  void updateWidget(size_t index, Entry *entry);
  void addWidget(Entry *entry, const QString &commandLine = "");

public:
  VisualContext(QWidget *parent);
  virtual ~VisualContext() { }

  virtual void executeInContext(Console *console, TeamSelector *teamSelector, const QString &cmdLine);

protected slots:
  void updateMinHeight();

public slots:
  virtual void doPrint(ConsolePrintTarget target, const QString &msg);
  void commandExecuted(Context *context, const QString &cmdLine);
  void commandFinished(bool status);
  void commandCanceled();
  void cancel();

signals:
  void minHeightChanged();
  void statusChanged(bool status);
  void sCanceled();
  void sCancel();

};

class Icons
{
  static Icons theIcons;
public:
  QIcon ICON_GRAY;
  QIcon ICON_GREEN;
  QIcon ICON_ORANGE;
  QIcon ICON_RED;
  static inline Icons& getInstance() { return theIcons; }
  void init();
};

/** Draws a cool Frame around a VisualContext. */
class VisualContextDecoration : public QFrame
{
  Q_OBJECT

  QPushButton *button; //TODO: button still inactive

  /** Shows which commandLine is executed. */
  QLabel *header;

  /** The visual context representation which should be decorated. */
  VisualContext *visualContext;

  /** The parent in the tree. */
  VisualContext *parentContext;

public:
  VisualContextDecoration(const QString &commandLine, VisualContext *parent, VisualContext *context);

public slots:
  void updateMinHeight();
  void updateStatus(bool status);
  void canceled();

signals:
  void minHeightChanged();
};

class Console : public QFrame
{
  Q_OBJECT

  /** The root visualContext.
   * It does not really display output but contains all other visual
   * representations of further contexts.
   */
  VisualContext *visualContext;

  /** The teamSelector which knows which robots and which team are selected. */
  TeamSelector *teamSelector;

  QScrollArea *scrollArea;

  /** bush> */
  QLabel *prompt;

  /** The thing where commands can be typed in. */
  CommandLineEdit *cmdLine;

  bool scrollEnabled;

public:
  ~Console() { }
  Console(TeamSelector *teamSelector);
  QSize minimumSizeHint() const { return QSize(100, 250); }

  /** Is executed when the console is shown and sets the focus to the
   * commandLineEdit so that the user can just start to type.
   */
  void showEvent(QShowEvent *event);

  void setCommand(const std::string &command);
  void cancel();

public slots:
  /** Scrolls the viewport to the bottom so that the last line of output becomes
   * visible.
   */
  void scroll();
  void updateScrollEndabled();

protected slots:
  /** Starts a new thread which executes the entered command. */
  virtual void returnPressed();
};
