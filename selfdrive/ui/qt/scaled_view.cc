#include "selfdrive/ui/qt/scaled_view.h"

#include <QResizeEvent>
#include <QVBoxLayout>

ScaledViewContainer::ScaledViewContainer(QWidget* widget, QWidget* parent)
    : QWidget(parent), scaled_widget(widget) {
  // Reparent the widget to this container
  scaled_widget->setParent(this);

  // Set up the container
  setAttribute(Qt::WA_OpaquePaintEvent);
  setStyleSheet("background-color: black;");
}

ScaledViewContainer::~ScaledViewContainer() {
}

void ScaledViewContainer::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);

  if (scaled_widget) {
    // Calculate scaled dimensions
    int scaled_width = width() * ONROAD_SCALE_FACTOR;
    int scaled_height = height() * ONROAD_SCALE_FACTOR;

    // Position at bottom-left (flush against edges)
    int x = 0;
    int y = height() - scaled_height;

    scaled_widget->setGeometry(x, y, scaled_width, scaled_height);
  }
}
