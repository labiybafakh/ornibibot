#ifndef ORNIBIBOTGUI_H
#define ORNIBIBOTGUI_H

#include <QMainWindow>

namespace Ui {
class OrnibiBotGUI;
}

class OrnibiBotGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit OrnibiBotGUI(QWidget *parent = nullptr);
  ~OrnibiBotGUI();

private:
  Ui::OrnibiBotGUI *ui;
};

#endif // ORNIBIBOTGUI_H
