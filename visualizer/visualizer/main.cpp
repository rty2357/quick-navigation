#include <QApplication>
#include <QTextCodec>
#include "main.hpp"
#include "window.hpp"

MainWindow *window;

int main(int argc, char *argv[])
{
    int retval;
    QApplication app(argc, argv);
    QTextCodec::setCodecForCStrings(QTextCodec::codecForLocale());

    window = new MainWindow();
    window->show();
    window->init();

    retval = app.exec();

    delete window;
    return retval;
}
