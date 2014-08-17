/****************************************************************************
** Meta object code from reading C++ file 'Tutorial1.h'
**
** Created: Sun 10. Aug 22:46:45 2014
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../Git_prova/Opticks_GSoC2014/LiDAR/Tutorial1.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Tutorial1.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Tutorial1[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_Tutorial1[] = {
    "Tutorial1\0"
};

const QMetaObject Tutorial1::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Tutorial1,
      qt_meta_data_Tutorial1, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Tutorial1::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Tutorial1::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Tutorial1::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Tutorial1))
        return static_cast<void*>(const_cast< Tutorial1*>(this));
    if (!strcmp(_clname, "ViewerShell"))
        return static_cast< ViewerShell*>(const_cast< Tutorial1*>(this));
    return QObject::qt_metacast(_clname);
}

int Tutorial1::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
