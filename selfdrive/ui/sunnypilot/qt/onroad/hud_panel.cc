#include "selfdrive/ui/sunnypilot/qt/onroad/hud_panel.h"
#include <QPainter>
#include "selfdrive/ui/qt/util.h"

HudPanel::HudPanel(QWidget* parent) : QWidget(parent) {
  // Transparent for mouse events by default so it doesn't grab input
  setAttribute(Qt::WA_TransparentForMouseEvents, true);
  // Allow transparent background such that underlying camera remains visible
  setAttribute(Qt::WA_NoSystemBackground, true);
}

void HudPanel::updateState(const UIState &s) {
  hud.updateState(s);
  update();
}

void HudPanel::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  hud.draw(p, rect());
}
