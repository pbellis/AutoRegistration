/****************************************************************************
** Meta object code from reading C++ file 'pclviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../AlignmentVisualizationLib/pclviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pclviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PCLViewer_t {
    QByteArrayData data[14];
    char stringdata0[205];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PCLViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PCLViewer_t qt_meta_stringdata_PCLViewer = {
    {
QT_MOC_LITERAL(0, 0, 9), // "PCLViewer"
QT_MOC_LITERAL(1, 10, 15), // "onAddPointCloud"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 35), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(4, 63, 5), // "cloud"
QT_MOC_LITERAL(5, 69, 11), // "std::string"
QT_MOC_LITERAL(6, 81, 4), // "name"
QT_MOC_LITERAL(7, 86, 26), // "onAddPointCloudWithNormals"
QT_MOC_LITERAL(8, 113, 33), // "pcl::PointCloud<pcl::Normal>:..."
QT_MOC_LITERAL(9, 147, 7), // "normals"
QT_MOC_LITERAL(10, 155, 8), // "onUpdate"
QT_MOC_LITERAL(11, 164, 10), // "onClearAll"
QT_MOC_LITERAL(12, 175, 22), // "onSetSingleFrameViewer"
QT_MOC_LITERAL(13, 198, 6) // "single"

    },
    "PCLViewer\0onAddPointCloud\0\0"
    "pcl::PointCloud<pcl::PointXYZ>::Ptr\0"
    "cloud\0std::string\0name\0"
    "onAddPointCloudWithNormals\0"
    "pcl::PointCloud<pcl::Normal>::Ptr\0"
    "normals\0onUpdate\0onClearAll\0"
    "onSetSingleFrameViewer\0single"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PCLViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   39,    2, 0x0a /* Public */,
       7,    3,   44,    2, 0x0a /* Public */,
      10,    0,   51,    2, 0x0a /* Public */,
      11,    0,   52,    2, 0x0a /* Public */,
      12,    1,   53,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 8, 0x80000000 | 5,    4,    9,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   13,

       0        // eod
};

void PCLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PCLViewer *_t = static_cast<PCLViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onAddPointCloud((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< const std::string(*)>(_a[2]))); break;
        case 1: _t->onAddPointCloudWithNormals((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< const pcl::PointCloud<pcl::Normal>::Ptr(*)>(_a[2])),(*reinterpret_cast< const std::string(*)>(_a[3]))); break;
        case 2: _t->onUpdate(); break;
        case 3: _t->onClearAll(); break;
        case 4: _t->onSetSingleFrameViewer((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject PCLViewer::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PCLViewer.data,
      qt_meta_data_PCLViewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PCLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PCLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PCLViewer.stringdata0))
        return static_cast<void*>(const_cast< PCLViewer*>(this));
    return QWidget::qt_metacast(_clname);
}

int PCLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
