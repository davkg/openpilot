#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QStackedLayout>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  split_wrapper = new QWidget;
  QVBoxLayout *split_wrapper_layout = new QVBoxLayout(split_wrapper);
  split_wrapper_layout->setContentsMargins(30, 0, 0, 0);  // Left border
  split_wrapper_layout->setSpacing(0);
  split_wrapper_layout->addStretch(1);

  split_surface = new QWidget(split_wrapper);
  split_surface->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  split = new QHBoxLayout(split_surface);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  split_wrapper_layout->addWidget(split_surface, 0, Qt::AlignLeft | Qt::AlignBottom);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, this);
    split->insertWidget(0, arCam);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts_container = new QWidget;
  QVBoxLayout *alerts_layout = new QVBoxLayout(alerts_container);
  alerts_layout->setContentsMargins(0, 0, 0, 0);
  alerts_layout->setSpacing(0);
  alerts_layout->addStretch(1);

  alerts = new OnroadAlerts(alerts_container);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  alerts_layout->addWidget(alerts, 0, Qt::AlignLeft | Qt::AlignBottom);

  stacked_layout->addWidget(alerts_container);

  // setup stacking order
  alerts->raise();

  updateScaledLayout();

  setAttribute(Qt::WA_OpaquePaintEvent);

  // We handle the connection of the signals on the derived class
#ifndef SUNNYPILOT
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
#endif
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  alerts->updateState(s);
  nvg->updateState(s);

  QColor bgColor = bg_colors[s.status];
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
  alerts->clear();
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

void OnroadWindow::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  updateScaledLayout();
}

void OnroadWindow::updateScaledLayout() {
  if (!split_surface || !alerts) return;

  const int scaled_w = int(width() * 0.75);
  const int scaled_h = int(height() * 0.75);
  const QSize scaled_size(scaled_w, scaled_h);

  split_surface->setFixedSize(scaled_size);
  alerts->setFixedSize(scaled_size);
}
