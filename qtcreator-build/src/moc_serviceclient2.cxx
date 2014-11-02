/****************************************************************************
** Meta object code from reading C++ file 'serviceclient2.h'
**
** Created: Sun Nov 2 17:40:35 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/serviceclient2.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'serviceclient2.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ServiceClient2[] = {

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
      16,   15,   15,   15, 0x0a,
      24,   15,   15,   15, 0x0a,
      38,   15,   15,   15, 0x0a,
      60,   15,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ServiceClient2[] = {
    "ServiceClient2\0\0start()\0togglePause()\0"
    "resetRgbdslamSystem()\0run_send_all()\0"
};

void ServiceClient2::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ServiceClient2 *_t = static_cast<ServiceClient2 *>(_o);
        switch (_id) {
        case 0: _t->start(); break;
        case 1: _t->togglePause(); break;
        case 2: _t->resetRgbdslamSystem(); break;
        case 3: _t->run_send_all(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData ServiceClient2::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ServiceClient2::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_ServiceClient2,
      qt_meta_data_ServiceClient2, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ServiceClient2::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ServiceClient2::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ServiceClient2::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ServiceClient2))
        return static_cast<void*>(const_cast< ServiceClient2*>(this));
    return QObject::qt_metacast(_clname);
}

int ServiceClient2::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
