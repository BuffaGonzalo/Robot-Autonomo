#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyle("fusion"); //linea para colocar el color de la ventana segun el tema de windows
    MainWindow w;
    w.show();
    return a.exec();
}
