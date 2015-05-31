/****************************************************************************
** Meta object code from reading C++ file 'statuswidget.h'
**
** Created: Wed Jun 26 04:33:19 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../statuswidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'statuswidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CStatusWidget[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      25,   15,   14,   14, 0x0a,
      51,   47,   14,   14, 0x2a,
      66,   14,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_CStatusWidget[] = {
    "CStatusWidget\0\0str,color\0write(QString,QColor)\0"
    "str\0write(QString)\0update()\0"
};

const QMetaObject CStatusWidget::staticMetaObject = {
    { &QDockWidget::staticMetaObject, qt_meta_stringdata_CStatusWidget,
      qt_meta_data_CStatusWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CStatusWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CStatusWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CStatusWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CStatusWidget))
        return static_cast<void*>(const_cast< CStatusWidget*>(this));
    return QDockWidget::qt_metacast(_clname);
}

int CStatusWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDockWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: write((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 1: write((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: update(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
