#include "PtzFollowTest.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PtzFollowTest w;
    w.show();
    return a.exec();
}
