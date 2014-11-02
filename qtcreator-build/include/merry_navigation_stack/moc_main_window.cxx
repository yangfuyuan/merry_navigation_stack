/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Sat Nov 1 15:37:37 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/merry_navigation_stack/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_merry_navigation_stack__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      36,   35,   35,   35, 0x0a,
      69,   63,   35,   35, 0x0a,
     107,  101,   35,   35, 0x0a,
     153,   35,   35,   35, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_merry_navigation_stack__MainWindow[] = {
    "merry_navigation_stack::MainWindow\0\0"
    "on_actionAbout_triggered()\0check\0"
    "on_button_connect_clicked(bool)\0state\0"
    "on_checkbox_use_environment_stateChanged(int)\0"
    "updateLoggingView()\0"
};

void merry_navigation_stack::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_button_connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_checkbox_use_environment_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->updateLoggingView(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData merry_navigation_stack::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject merry_navigation_stack::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_merry_navigation_stack__MainWindow,
      qt_meta_data_merry_navigation_stack__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &merry_navigation_stack::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *merry_navigation_stack::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *merry_navigation_stack::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_merry_navigation_stack__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int merry_navigation_stack::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
