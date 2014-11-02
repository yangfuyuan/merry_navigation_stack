/****************************************************************************
** Meta object code from reading C++ file 'mission_planner.h'
**
** Created: Mon Nov 3 00:23:45 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/mission_planner.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mission_planner.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MissionPlanner[] = {

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
      16,   15,   15,   15, 0x08,
      22,   15,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MissionPlanner[] = {
    "MissionPlanner\0\0run()\0start()\0"
};

void MissionPlanner::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MissionPlanner *_t = static_cast<MissionPlanner *>(_o);
        switch (_id) {
        case 0: _t->run(); break;
        case 1: _t->start(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData MissionPlanner::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MissionPlanner::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MissionPlanner,
      qt_meta_data_MissionPlanner, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MissionPlanner::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MissionPlanner::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MissionPlanner::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MissionPlanner))
        return static_cast<void*>(const_cast< MissionPlanner*>(this));
    return QObject::qt_metacast(_clname);
}

int MissionPlanner::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
