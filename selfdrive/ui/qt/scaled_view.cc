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
    // Keep widget at full container size for rendering
    scaled_widget->resize(width(), height());
  }
}

void ScaledViewContainer::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  // Fill background
  painter.fillRect(rect(), Qt::black);

  if (scaled_widget) {
    // Grab the widget's rendered content
    QPixmap pixmap = scaled_widget->grab();

    // Calculate scaled dimensions
    int scaled_width = width() * SCALE_FACTOR;
    int scaled_height = height() * SCALE_FACTOR;
    int x = 0;
    int y = height() - scaled_height;

    // Draw the scaled pixmap at bottom-left
    painter.drawPixmap(x, y, scaled_width, scaled_height, pixmap);
  }
}
