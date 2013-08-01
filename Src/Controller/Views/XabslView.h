/**
* @file Controller/Views/XabslView.h
*
* Declaration of class XabslView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "SimRobot.h"

class RobotConsole;
class XabslInfo;

/**
* @class XabslView
*
* A class to represent a view with information about a Xabsl behavior.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/
class XabslView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph
  * @param console The console object.
  * @param info The Xabsl info object to be visualized.
  */
  XabslView(const QString& fullName, RobotConsole& console, const XabslInfo& info);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const XabslInfo& info; /**< The Xabsl info structure. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class XabslWidget;
};
