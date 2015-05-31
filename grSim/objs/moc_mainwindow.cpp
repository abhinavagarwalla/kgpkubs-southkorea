/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Wed Jun 26 04:33:07 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      26,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      21,   11,   11,   11, 0x0a,
      42,   40,   11,   11, 0x0a,
      63,   40,   11,   11, 0x0a,
      87,   11,   11,   11, 0x0a,
     108,   11,   11,   11, 0x0a,
     128,   11,   11,   11, 0x0a,
     145,   11,   11,   11, 0x0a,
     171,   11,   11,   11, 0x0a,
     191,   11,   11,   11, 0x0a,
     207,   11,   11,   11, 0x0a,
     221,   11,   11,   11, 0x0a,
     244,  240,   11,   11, 0x0a,
     272,   11,   11,   11, 0x0a,
     295,   11,   11,   11, 0x0a,
     321,   11,   11,   11, 0x0a,
     336,   11,   11,   11, 0x0a,
     366,  362,   11,   11, 0x0a,
     381,   11,   11,   11, 0x0a,
     393,   11,   11,   11, 0x0a,
     418,   11,   11,   11, 0x0a,
     448,   11,   11,   11, 0x0a,
     476,   11,   11,   11, 0x0a,
     500,   11,   11,   11, 0x0a,
     514,   11,   11,   11, 0x0a,
     526,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0update()\0updateRobotLabel()\0"
    "v\0showHideConfig(bool)\0showHideSimulator(bool)\0"
    "changeCurrentRobot()\0changeCurrentTeam()\0"
    "changeBallMass()\0changeBallGroundSurface()\0"
    "changeBallDamping()\0changeGravity()\0"
    "changeTimer()\0restartSimulator()\0act\0"
    "ballMenuTriggered(QAction*)\0"
    "toggleFullScreen(bool)\0setCurrentRobotPosition()\0"
    "takeSnapshot()\0takeSnapshotToClipboard()\0"
    "fps\0customFPS(int)\0showAbout()\0"
    "reconnectCommandSocket()\0"
    "reconnectYellowStatusSocket()\0"
    "reconnectBlueStatusSocket()\0"
    "reconnectVisionSocket()\0recvActions()\0"
    "startGame()\0stopGame()\0"
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: update(); break;
        case 1: updateRobotLabel(); break;
        case 2: showHideConfig((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: showHideSimulator((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: changeCurrentRobot(); break;
        case 5: changeCurrentTeam(); break;
        case 6: changeBallMass(); break;
        case 7: changeBallGroundSurface(); break;
        case 8: changeBallDamping(); break;
        case 9: changeGravity(); break;
        case 10: changeTimer(); break;
        case 11: restartSimulator(); break;
        case 12: ballMenuTriggered((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 13: toggleFullScreen((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: setCurrentRobotPosition(); break;
        case 15: takeSnapshot(); break;
        case 16: takeSnapshotToClipboard(); break;
        case 17: customFPS((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 18: showAbout(); break;
        case 19: reconnectCommandSocket(); break;
        case 20: reconnectYellowStatusSocket(); break;
        case 21: reconnectBlueStatusSocket(); break;
        case 22: reconnectVisionSocket(); break;
        case 23: recvActions(); break;
        case 24: startGame(); break;
        case 25: stopGame(); break;
        default: ;
        }
        _id -= 26;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
