/****************************************************************************
** Meta object code from reading C++ file 'autoaligner.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../AlignmentPredictionApp/autoaligner.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'autoaligner.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_AutoAligner_t {
    QByteArrayData data[15];
    char stringdata0[221];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AutoAligner_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AutoAligner_t qt_meta_stringdata_AutoAligner = {
    {
QT_MOC_LITERAL(0, 0, 11), // "AutoAligner"
QT_MOC_LITERAL(1, 12, 24), // "messageAlignmentStrategy"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 15), // "Eigen::MatrixXd"
QT_MOC_LITERAL(4, 54, 15), // "distance_matrix"
QT_MOC_LITERAL(5, 70, 15), // "Eigen::VectorXi"
QT_MOC_LITERAL(6, 86, 4), // "path"
QT_MOC_LITERAL(7, 91, 17), // "messagePointCloud"
QT_MOC_LITERAL(8, 109, 35), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(9, 145, 5), // "cloud"
QT_MOC_LITERAL(10, 151, 11), // "std::string"
QT_MOC_LITERAL(11, 163, 4), // "name"
QT_MOC_LITERAL(12, 168, 16), // "messageFinalized"
QT_MOC_LITERAL(13, 185, 15), // "onAddPointCloud"
QT_MOC_LITERAL(14, 201, 19) // "onFinalizeAlignment"

    },
    "AutoAligner\0messageAlignmentStrategy\0"
    "\0Eigen::MatrixXd\0distance_matrix\0"
    "Eigen::VectorXi\0path\0messagePointCloud\0"
    "pcl::PointCloud<pcl::PointXYZ>::Ptr\0"
    "cloud\0std::string\0name\0messageFinalized\0"
    "onAddPointCloud\0onFinalizeAlignment"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AutoAligner[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   39,    2, 0x06 /* Public */,
       7,    2,   44,    2, 0x06 /* Public */,
      12,    0,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    2,   50,    2, 0x0a /* Public */,
      14,    0,   55,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 8, 0x80000000 | 10,    9,   11,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 8, 0x80000000 | 10,    9,   11,
    QMetaType::Void,

       0        // eod
};

void AutoAligner::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        AutoAligner *_t = static_cast<AutoAligner *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->messageAlignmentStrategy((*reinterpret_cast< const Eigen::MatrixXd(*)>(_a[1])),(*reinterpret_cast< const Eigen::VectorXi(*)>(_a[2]))); break;
        case 1: _t->messagePointCloud((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< const std::string(*)>(_a[2]))); break;
        case 2: _t->messageFinalized(); break;
        case 3: _t->onAddPointCloud((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< const std::string(*)>(_a[2]))); break;
        case 4: _t->onFinalizeAlignment(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (AutoAligner::*_t)(const Eigen::MatrixXd & , const Eigen::VectorXi & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&AutoAligner::messageAlignmentStrategy)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (AutoAligner::*_t)(const pcl::PointCloud<pcl::PointXYZ>::Ptr & , const std::string & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&AutoAligner::messagePointCloud)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (AutoAligner::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&AutoAligner::messageFinalized)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject AutoAligner::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_AutoAligner.data,
      qt_meta_data_AutoAligner,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *AutoAligner::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AutoAligner::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_AutoAligner.stringdata0))
        return static_cast<void*>(const_cast< AutoAligner*>(this));
    return QObject::qt_metacast(_clname);
}

int AutoAligner::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void AutoAligner::messageAlignmentStrategy(const Eigen::MatrixXd & _t1, const Eigen::VectorXi & _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void AutoAligner::messagePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & _t1, const std::string & _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void AutoAligner::messageFinalized()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
