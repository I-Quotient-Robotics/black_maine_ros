#include <QCoreApplication>
#include <QTimer>
#include <QObject>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QUrl>
#include <QString>
#include <QQuickWindow>
#include <QtQml>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>

#include <functional>

#include "qml_mediator.h"

int main(int argc, char** argv) {
    //Init ros stuff
    ros::init(argc, argv, "black_maine_panel_node");
    ros::NodeHandle node;

    //Init Qt
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);
    QMLMediator mediate(&app);

    //Start ros in separate thread, and trigger Qt shutdown when it exits
    //If Qt exits before ros, be sure to shutdown ros
    QFutureWatcher<void> rosThread;
    rosThread.setFuture(QtConcurrent::run(&ros::spin));
    QObject::connect(&rosThread, &QFutureWatcher<void>::finished, &app, &QCoreApplication::quit);
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [](){ros::shutdown();});

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("mediator", &mediate);
    engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
