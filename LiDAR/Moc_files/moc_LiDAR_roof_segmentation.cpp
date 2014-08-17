/****************************************************************************
** Meta object code from reading C++ file 'LiDAR_roof_segmentation.h'
**
** Created: Wed 13. Aug 22:28:52 2014
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../Git_prova/Opticks_GSoC2014/LiDAR/LiDAR_roof_segmentation.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LiDAR_roof_segmentation.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LiDAR_roof_segmentation[] = {

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

static const char qt_meta_stringdata_LiDAR_roof_segmentation[] = {
    "LiDAR_roof_segmentation\0"
};

const QMetaObject LiDAR_roof_segmentation::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_LiDAR_roof_segmentation,
      qt_meta_data_LiDAR_roof_segmentation, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LiDAR_roof_segmentation::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LiDAR_roof_segmentation::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LiDAR_roof_segmentation::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LiDAR_roof_segmentation))
        return static_cast<void*>(const_cast< LiDAR_roof_segmentation*>(this));
    if (!strcmp(_clname, "ViewerShell"))
        return static_cast< ViewerShell*>(const_cast< LiDAR_roof_segmentation*>(this));
    return QObject::qt_metacast(_clname);
}

int LiDAR_roof_segmentation::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
