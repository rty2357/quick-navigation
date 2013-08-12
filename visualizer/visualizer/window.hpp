#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QMenu>
#include "widget-gl.hpp"
#include "widget-msg.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:

         MainWindow();
        ~MainWindow();

         bool init();

         void test_add_menu_SSM(QObject* obj, const char* str);

    private slots:

        void fps01() { fps=1;  viewer->setfps(fps); }
        void fps05() { fps=5;  viewer->setfps(fps); }
        void fps10() { fps=10; viewer->setfps(fps); }
        void fps20() { fps=20; viewer->setfps(fps); }
        void fps30() { fps=30; viewer->setfps(fps); }

    public:

        WidgetGL  *viewer;
        WidgetMSG *status;
        WidgetMSG *message;

    private:

        int fps;
        QMenu *ssm_menu;
};

#endif
