/****************************************************************************
** Meta object code from reading C++ file 'global_planner.h'
**
** Created: Mon Nov 3 00:23:44 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/global_planner.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'global_planner.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GlobalPlanner[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x08,
      21,   14,   14,   14, 0x08,
      37,   14,   14,   14, 0x0a,
      45,   14,   14,   14, 0x0a,
      65,   14,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GlobalPlanner[] = {
    "GlobalPlanner\0\0run()\0runAutonomous()\0"
    "start()\0on_reqBtn_clicked()\0"
    "toggleAutonomous()\0"
};

void GlobalPlanner::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GlobalPlanner *_t = static_cast<GlobalPlanner *>(_o);
        switch (_id) {
        case 0: _t->run(); break;
        case 1: _t->runAutonomous(); break;
        case 2: _t->start(); break;
        case 3: _t->on_reqBtn_clicked(); break;
        case 4: _t->toggleAutonomous(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData GlobalPlanner::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GlobalPlanner::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_GlobalPlanner,
      qt_meta_data_GlobalPlanner, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GlobalPlanner::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GlobalPlanner::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GlobalPlanner::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GlobalPlanner))
        return static_cast<void*>(const_cast< GlobalPlanner*>(this));
    return QObject::qt_metacast(_clname);
}

int GlobalPlanner::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
