#include "QtGui.h"
#include <QtWidgets/QApplication>
#include "Utils/src/Utils.h"

int main(int argc, char* argv[])
{
	init();
	QApplication a(argc, argv);
	Qt_gui window;

	window.show();

	return a.exec();
}
