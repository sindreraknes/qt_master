#include <QtGui>
#include <QApplication>
#include "../include/qt_master/main_window.hpp"

/*!
 * \brief Main method for the program. Starts the application.
 * \param argc Arguments.
 * \param argv Arguments.
 * \return Returns 1 if shut down correct, 0 otherwise.
 */
int main(int argc, char **argv) {
    QApplication app(argc, argv);
    qt_master::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
