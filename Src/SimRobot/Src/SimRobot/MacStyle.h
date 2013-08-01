
#pragma once

#include <QMacStyle>

static const QString BUTTONS_LEFT = "QDockWidget::close-button, QDockWidget::float-button { subcontrol-position: left; } QDockWidget::close-button { left: 4px; } QDockWidget::float-button { left: 24px; }";

class MacStyle : public QMacStyle
{
public:
  int pixelMetric(PixelMetric metric, const QStyleOption* option = 0, const QWidget* widget = 0 ) const
  {
    int s = QMacStyle::pixelMetric(metric, option, widget);
    if (metric == QStyle::PM_SmallIconSize)
    {
      s = 20;
    }
    return s;
  }
};
