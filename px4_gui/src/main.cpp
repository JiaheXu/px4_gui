#include <QtGui>
#include <QApplication>
#include "../include/px4_gui/main_window.hpp"

int main(int argc, char **argv) {

 
    QApplication app(argc, argv);
qDebug() << "main";
    px4_gui::MainWindow w(argc,argv);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    return result;
}
