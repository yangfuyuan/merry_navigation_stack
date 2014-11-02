/****************************************************************************
** Meta object code from reading C++ file 'maplistener.h'
**
** Created: Mon Nov 3 00:23:45 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/maplistener.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'maplistener.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MapListener[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x0a,
      21,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MapListener[] = {
    "MapListener\0\0start()\0run()\0"
};

void MapListener::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MapListener *_t = static_cast<MapListener *>(_o);
        switch (_id) {
        case 0: _t->start(); break;
        case 1: _t->run(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData MapListener::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MapListener::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MapListener,
      qt_meta_data_MapListener, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MapListener::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MapListener::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MapListener::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MapListener))
        return static_cast<void*>(const_cast< MapListener*>(this));
    return QObject::qt_metacast(_clname);
}

int MapListener::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
