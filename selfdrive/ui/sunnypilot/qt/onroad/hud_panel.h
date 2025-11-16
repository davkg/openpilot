#pragma once

#include <QWidget>
#include <QPainter>

#include "selfdrive/ui/sunnypilot/qt/onroad/hud.h"

class HudPanel : public QWidget {
  Q_OBJECT

public:
  explicit HudPanel(QWidget *parent = nullptr);
  virtual ~HudPanel() = default;
  void updateState(const UIState &s);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  HudRendererSP hud;
};
