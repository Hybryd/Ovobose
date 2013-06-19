#include <QApplication>
#include <QtGui>
#include "MainWin.h"
 
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setStyle("plastique");
 
    MainWin fenetre;
    fenetre.show();
 
    return app.exec();
}
