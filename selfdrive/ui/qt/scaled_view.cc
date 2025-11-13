#include "selfdrive/ui/qt/scaled_view.h"

#include <QResizeEvent>
#include <QPainter>

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
    scaled_widget->resize(width(), height());
  }
}

void ScaledViewContainer::paintEvent(QPaintEvent* event) {
  QPainter painter(this);

  // Fill background
  painter.fillRect(rect(), Qt::black);

  if (scaled_widget) {
    // Save painter state
    painter.save();

    // Position at bottom-left
    int scaled_height = height() * SCALE_FACTOR;
    int x = 0;
    int y = height() - scaled_height;

    // Translate to bottom-left position, then scale
    painter.translate(x, y);
    painter.scale(SCALE_FACTOR, SCALE_FACTOR);

    // Render the widget
    scaled_widget->render(&painter);

    // Restore painter state
    painter.restore();
  }
}
