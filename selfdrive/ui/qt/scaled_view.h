#pragma once

#include <QWidget>

// Scale factor for onroad view (75% of original size)
constexpr float ONROAD_SCALE_FACTOR = 0.75f;

class ScaledViewContainer : public QWidget {
  Q_OBJECT

public:
  explicit ScaledViewContainer(QWidget* widget, QWidget* parent = nullptr);
  ~ScaledViewContainer();

protected:
  void resizeEvent(QResizeEvent* event) override;

private:
  QWidget* scaled_widget;
};
