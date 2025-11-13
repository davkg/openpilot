#pragma once

#include <QWidget>

// Scale factor for UI view (75% of original size)
constexpr float SCALE_FACTOR = 0.75f;

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
