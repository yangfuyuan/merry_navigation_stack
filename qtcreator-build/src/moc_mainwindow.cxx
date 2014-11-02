/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Mon Nov 3 01:22:31 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,
      32,   11,   11,   11, 0x05,
      54,   11,   11,   11, 0x05,
      69,   11,   11,   11, 0x05,
      84,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     103,   11,   11,   11, 0x0a,
     126,   11,   11,   11, 0x0a,
     149,   11,   11,   11, 0x0a,
     173,   11,   11,   11, 0x0a,
     200,  197,   11,   11, 0x0a,
     241,   11,   11,   11, 0x08,
     269,   11,   11,   11, 0x08,
     291,   11,   11,   11, 0x08,
     312,   11,   11,   11, 0x08,
     333,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0toggleGenerateMap()\0"
    "resetRgbdslamSystem()\0reqFrontier0()\0"
    "reqFrontier1()\0toggleAutonomous()\0"
    "writeToLogLp0(QString)\0writeToLogLp1(QString)\0"
    "writeToLogMgp0(QString)\0writeToLogMgp1(QString)\0"
    ",,\0writeToDisplayImage(QImage,QString,uint)\0"
    "on_generateMapBtn_clicked()\0"
    "on_resetBtn_clicked()\0on_req0Btn_clicked()\0"
    "on_req1Btn_clicked()\0on_autonomousBtn_clicked()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->toggleGenerateMap(); break;
        case 1: _t->resetRgbdslamSystem(); break;
        case 2: _t->reqFrontier0(); break;
        case 3: _t->reqFrontier1(); break;
        case 4: _t->toggleAutonomous(); break;
        case 5: _t->writeToLogLp0((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->writeToLogLp1((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->writeToLogMgp0((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->writeToLogMgp1((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->writeToDisplayImage((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3]))); break;
        case 10: _t->on_generateMapBtn_clicked(); break;
        case 11: _t->on_resetBtn_clicked(); break;
        case 12: _t->on_req0Btn_clicked(); break;
        case 13: _t->on_req1Btn_clicked(); break;
        case 14: _t->on_autonomousBtn_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
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
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::toggleGenerateMap()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void MainWindow::resetRgbdslamSystem()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void MainWindow::reqFrontier0()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void MainWindow::reqFrontier1()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void MainWindow::toggleAutonomous()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}
QT_END_MOC_NAMESPACE
