/****************************************************************************
** Meta object code from reading C++ file 'photoview.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../annotation/photoview.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'photoview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PhotoView[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   11,   10,   10, 0x05,
      41,   10,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
      57,   11,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PhotoView[] = {
    "PhotoView\0\0pId\0selectedPointUpdated(int)\0"
    "pointsUpdated()\0setMirrorPoint(int)\0"
};

void PhotoView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PhotoView *_t = static_cast<PhotoView *>(_o);
        switch (_id) {
        case 0: _t->selectedPointUpdated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->pointsUpdated(); break;
        case 2: _t->setMirrorPoint((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PhotoView::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PhotoView::staticMetaObject = {
    { &QGraphicsView::staticMetaObject, qt_meta_stringdata_PhotoView,
      qt_meta_data_PhotoView, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PhotoView::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PhotoView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PhotoView::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PhotoView))
        return static_cast<void*>(const_cast< PhotoView*>(this));
    return QGraphicsView::qt_metacast(_clname);
}

int PhotoView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void PhotoView::selectedPointUpdated(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PhotoView::pointsUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
