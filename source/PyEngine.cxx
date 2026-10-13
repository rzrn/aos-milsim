/*
    Copyright © 2024–2026 rzrn

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Milsim/PyEngine.hxx>
#include <Milsim/Engine.hxx>

template<typename T> inline T PyDictLargestKey(PyObject * dict, T minimum = -1) {
    T retval = minimum;

    Py_ssize_t i = 0; PyObject * k, * v;

    while (PyDict_Next(dict, &i, &k, &v))
        retval = std::max<T>(retval, PyDecode<T>(k));

    return retval;
}

template<> inline Vector3i PyDecode<Vector3i>(PyObject * o) {
    Vector3i v;

    v.x = PyGetAttr<int>(o, "x"); RETDEFIFERR();
    v.y = PyGetAttr<int>(o, "y"); RETDEFIFERR();
    v.z = PyGetAttr<int>(o, "z"); RETDEFIFERR();

    return v;
}

template<> inline Vector3d PyDecode<Vector3d>(PyObject * o) {
    Vector3d v;

    v.x = PyGetAttr<double>(o, "x"); RETDEFIFERR();
    v.y = PyGetAttr<double>(o, "y"); RETDEFIFERR();
    v.z = PyGetAttr<double>(o, "z"); RETDEFIFERR();

    return v;
}

struct PyEngine {
    PyObject_HEAD
    Engine * ref;
};

static_assert(std::is_standard_layout_v<PyEngine> == true);

static PyEngine * PyEngineNew(PyTypeObject * type, PyObject * w, PyObject * kw) {
    auto self = (PyEngine *) type->tp_alloc(type, 0); RETZIFZ(self);

    self->ref = nullptr;
    return self;
}

static int PyEngineInit(PyEngine * self, PyObject * w, PyObject * kw) {
    PyObject * o; if (!PyArg_ParseTuple(w, "O", &o)) return -1;

    RETERRIFZ(self->ref = new Engine(o));

    RETERRIFZ(self->ref->onPlayerHit = PyOwnedRef(o, "onPlayerHit"));
    RETERRIFZ(self->ref->onBlockHit  = PyOwnedRef(o, "onBlockHit"));
    RETERRIFZ(self->ref->onDestroy   = PyOwnedRef(o, "onDestroy"));

    return 0;
}

static int PyEngineClear(PyEngine * self) {
    if (self->ref == nullptr) return 0;

    self->ref->clear();

    self->ref->map = nullptr;

    self->ref->protocol.retain(nullptr);
    self->ref->onTrace.retain(nullptr);
    self->ref->onBlockHit.retain(nullptr);
    self->ref->onPlayerHit.retain(nullptr);
    self->ref->onDestroy.retain(nullptr);

    return 0;
}

static void PyEngineDealloc(PyEngine * self) {
    PyObject_GC_UnTrack(self);
    PyEngineClear(self);
    Py_TYPE(self)->tp_free(self);
}

static int PyEngineTraverse(PyEngine * self, visitproc visit, void * arg) {
    Py_VISIT(self->ref->protocol);

    for (auto & o : self->ref->objects)
        Py_VISIT(o.object());

    if (self->ref->onTrace != nullptr)
        Py_VISIT(self->ref->onTrace);

    if (self->ref->onBlockHit != nullptr)
        Py_VISIT(self->ref->onBlockHit);

    if (self->ref->onPlayerHit != nullptr)
        Py_VISIT(self->ref->onPlayerHit);

    if (self->ref->onDestroy != nullptr)
        Py_VISIT(self->ref->onDestroy);

    return 0;
}

static PyObject * PyEngineLag(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->lag()); }

static PyObject * PyEnginePeak(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->peak()); }

static PyObject * PyEngineAlive(PyEngine * self, void *)
{ return PyEncode<size_t>(self->ref->alive()); }

static PyObject * PyEngineTotal(PyEngine * self, void *)
{ return PyEncode<size_t>(self->ref->total()); }

static PyObject * PyEngineUsage(PyEngine * self, void *)
{ return PyEncode<size_t>(self->ref->usage()); }

static PyObject * PyEngineTemperature(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->temperature); }

static PyObject * PyEnginePressure(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->pressure); }

static PyObject * PyEngineHumidity(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->humidity); }

static PyObject * PyEngineWind(PyEngine * self, void *) {
    auto retval = PyTuple_New(3);

    auto & w = self->ref->wind;

    PyTuple_SET_ITEM(retval, 0, PyEncode<double>(w.x));
    PyTuple_SET_ITEM(retval, 1, PyEncode<double>(w.y));
    PyTuple_SET_ITEM(retval, 2, PyEncode<double>(w.z));

    return retval;
}

static PyObject * PyEngineDensity(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->density()); }

static PyObject * PyEngineMach(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->mach()); }

static PyObject * PyEnginePPO2(PyEngine * self, void *)
{ return PyEncode<double>(self->ref->ppo2()); }

static PyObject * PyEngineUpdate(PyEngine * self, PyObject * E) {
    self->ref->temperature = PyGetAttr<double>(E, "temperature"); RETZIFERR();
    self->ref->pressure    = PyGetAttr<double>(E, "pressure");    RETZIFERR();
    self->ref->humidity    = PyGetAttr<double>(E, "humidity");    RETZIFERR();
    self->ref->wind        = PyGetAttr<Vector3d>(E, "wind");      RETZIFERR();

    self->ref->update();

    Py_RETURN_NONE;
}

static PyObject * PyEngineDragRaycast(PyEngine * self, PyObject * w) {
    double CD, m, A, v₀; PyObject * ro1, * ro2;

    if (!PyArg_ParseTuple(w, "ddddOO", &CD, &m, &A, &v₀, &ro1, &ro2))
        return nullptr;

    auto r1 = PyDecode<Vector3d>(ro1); RETZIFERR();
    auto r2 = PyDecode<Vector3d>(ro2); RETZIFERR();

    auto v = self->ref->dragRaycast(CD, m, A, v₀, r1, r2);
    return PyEncode<double>(v);
}
static PyObject * PyEngineAttenuationCofficient(PyEngine * self, PyObject * fo) {
    auto f = PyDecode<double>(fo); RETZIFERR();

    auto α = self->ref->attenuationCoefficient(f);
    return PyEncode<double>(α);
}

static PyObject * PyEngineHCCofficient(PyEngine * self, PyObject * Wo) {
    auto W = PyDecode<double>(Wo); RETZIFERR();

    auto HCC = self->ref->HopkinsonCranzCoefficient(W);
    return PyEncode<double>(HCC);
}

static PyObject * PyEngineOverpressureValue(PyEngine * self, PyObject * Zo) {
    auto p = self->ref->pressure;

    auto Z = PyDecode<double>(Zo); RETZIFERR();
    return PyEncode<double>(p * overpressurePeakToAmbientRatio<double>(Z));
}

static PyObject * PyEngineOverpressureDuration(PyObject *, PyObject * Zo) {
    auto Z = PyDecode<double>(Zo); RETZIFERR();
    return PyEncode<double>(overpressureDuration<double>(Z));
}

static PyObject * PyEngineOverpressureImpulse(PyObject *, PyObject * Zo) {
    auto Z = PyDecode<double>(Zo); RETZIFERR();
    return PyEncode<double>(overpressureImpulse<double>(Z));
}

static PyObject * PyEngineExposed(PyEngine * self, PyObject * w) {
    int i; PyObject * co;

    if (!PyArg_ParseTuple(w, "iO", &i, &co))
        return nullptr;

    auto c = PyDecode<Vector3d>(co); RETZIFERR();
    auto [hd, to, ll, lr, ar, al] = self->ref->players[i].exposed(c);

    auto retval = PyTuple_New(6);

    PyTuple_SET_ITEM(retval, 0, PyEncode<double>(hd));
    PyTuple_SET_ITEM(retval, 1, PyEncode<double>(to));
    PyTuple_SET_ITEM(retval, 2, PyEncode<double>(ll));
    PyTuple_SET_ITEM(retval, 3, PyEncode<double>(lr));
    PyTuple_SET_ITEM(retval, 4, PyEncode<double>(ar));
    PyTuple_SET_ITEM(retval, 5, PyEncode<double>(al));

    return retval;
}

static PyObject * PyEngineAdd(PyEngine * self, PyObject * w) {
    int player_id; PyObject * ro, * vo; double timestamp; PyObject * po;

    if (!PyArg_ParseTuple(w, "iOOdO", &player_id, &ro, &vo, &timestamp, &po))
        return nullptr;

    auto r = PyDecode<Vector3d>(ro); RETZIFERR();
    auto v = PyDecode<Vector3d>(vo); RETZIFERR();

    auto i = PyGetAttr<uint32_t>(po, "model");   RETZIFERR();
    auto m = PyGetAttr<double>(po, "effmass");   RETZIFERR();
    auto b = PyGetAttr<double>(po, "ballistic"); RETZIFERR();
    auto A = PyGetAttr<double>(po, "area");      RETZIFERR();

    auto & o = self->ref->objects.emplace_back(po, i, player_id, r, v, timestamp);
    o.mass = m; o.ballistic = b; o.area = A;

    self->ref->trace(o.index(), o.position, 1.0, true);

    return PyFloat_FromDouble(o.energy());
}

static PyObject * PyEngineStep(PyEngine * self, PyObject * w) {
    double t1, t2;

    if (!PyArg_ParseTuple(w, "dd", &t1, &t2))
        return nullptr;

    self->ref->step(t1, t2);

    Py_RETURN_NONE;
}

static PyObject * PyEngineGetitem(PyEngine * self, PyObject * k) {
    int x, y, z;

    if (!PyArg_ParseTuple(k, "iii", &x, &y, &z))
        return nullptr;

    auto & voxel = self->ref->vxlData.get(x, y, z);

    auto retval = PyTuple_New(2);
    PyTuple_SET_ITEM(retval, 0, voxel.object.incref());
    PyTuple_SET_ITEM(retval, 1, PyFloat_FromDouble(voxel.durability));

    return retval;
}

static int PyEngineSetitem(PyEngine * self, PyObject * k, PyObject * o) {
    int x, y, z;

    if (!PyArg_ParseTuple(k, "iii", &x, &y, &z))
        return -1;

    if (o == nullptr)
        self->ref->vxlData.erase(x, y, z);
    else {
        if (!PyObject_TypeCheck(o, &MaterialType)) {
            PyErr_SetString(PyExc_TypeError, "must be Material");
            return -1;
        }

        self->ref->vxlData.set(x, y, z, o);
    }

    return 0;
}

static PyObject * PyEngineClearMeth(PyEngine * self, PyObject *) {
    self->ref->clear();

    PyOwnedRef M(self->ref->protocol, "map"); RETZIFZ(M);
    RETZIFZ(self->ref->map = mapBufRef(M));

    Py_RETURN_NONE;
}

static PyObject * PyEngineFlush(PyEngine * self, PyObject *) {
    self->ref->objects.clear();

    Py_RETURN_NONE;
}

static PyObject * PyEngineDig(PyEngine * self, PyObject * w) {
    int player_id, x, y, z; double value;

    if (!PyArg_ParseTuple(w, "iiiid", &player_id, &x, &y, &z, &value))
        return nullptr;

    if (self->ref->indestructible(x, y, z))
        Py_RETURN_NONE;

    auto & voxel = self->ref->vxlData.get(x, y, z);
    auto M = voxel.material();

    if (voxel.isub(value / M->durability))
        self->ref->onDestroy(player_id, x, y, z);

    Py_RETURN_NONE;
}

static PyObject * PyEngineSmash(PyEngine * self, PyObject * w) {
    int player_id, x, y, z; double ΔE;

    if (!PyArg_ParseTuple(w, "iiiid", &player_id, &x, &y, &z, &ΔE))
        return nullptr;

    if (self->ref->indestructible(x, y, z))
        Py_RETURN_NONE;

    auto & voxel = self->ref->vxlData.get(x, y, z);
    auto M = voxel.material();

    if (M->crumbly && randbool<double>(0.5) && self->ref->unstable(x, y, z)) {
        self->ref->onDestroy(player_id, x, y, z);
        Py_RETURN_NONE;
    }

    if (voxel.isub(ΔE * (M->durability / M->absorption)))
        self->ref->onDestroy(player_id, x, y, z);

    Py_RETURN_NONE;
}

static PyObject * PyEngineApply(PyEngine * self, PyObject * dict) {
    if (!PyDict_Check(dict)) {
        PyErr_SetString(PyExc_TypeError, "must be dict");

        return nullptr;
    }

    // TODO: we have to do something about this
    for (long xy = 0; xy < MAP_XY; xy++) for (int z = 0; z < MAP_Z; z++) {
        if (!map_is_solid_xy(self->ref->map, xy, z))
            continue;

        unsigned char * color = map_get_color_xy(self->ref->map, xy, z);

        unsigned int v = (static_cast<unsigned int>(color[0]) << 0)
                       | (static_cast<unsigned int>(color[1]) << 8)
                       | (static_cast<unsigned int>(color[1]) << 16);

        PyOwnedRef i(PyEncode<unsigned int>(v & 0xFFFFFF));
        self->ref->vxlData.set(xy * MAP_Z + z, PyDict_GetItem(dict, i));
    }

    Py_RETURN_NONE;
}

static PyObject * PyEngineOnSpawn(PyEngine * self, PyObject * w) {
    int i;

    if (!PyArg_ParseTuple(w, "i", &i))
        return nullptr;

    PyOwnedRef ds(self->ref->protocol, "players"); RETZIFZ(ds);

    self->ref->players.resize(PyDictLargestKey<int>(ds) + 1);

    auto o = PyDict_GetItem(ds, PyOwnedRef(PyEncode<size_t>(i))); RETZIFZ(o);

    PyOwnedRef wo(o, "world_object"); RETZIFZ(wo);
    PyOwnedRef po(wo, "position");    RETZIFZ(po);
    PyOwnedRef fo(wo, "orientation"); RETZIFZ(fo);
    PyOwnedRef co(wo, "crouch");      RETZIFZ(co);

    auto p = vectorRef(po); RETZIFZ(p);
    auto f = vectorRef(fo); RETZIFZ(f);

    auto & player = self->ref->players[i];
    player.set_position(p);
    player.set_orientation(f);
    player.set_crouch(co == Py_True);

    Py_RETURN_NONE;
}

static PyObject * PyEngineOnDespawn(PyEngine * self, PyObject * w) {
    int i;

    if (!PyArg_ParseTuple(w, "i", &i))
        return nullptr;

    if (i < self->ref->players.size()) { // `PyEngineOnSpawn` is not called for spectators
        auto & player = self->ref->players[i];
        player.set_crouch(false);
        player.set_position(nullptr);
        player.set_orientation(nullptr);

        PyOwnedRef ds(self->ref->protocol, "players"); RETZIFZ(ds);
        self->ref->players.resize(PyDictLargestKey<int>(ds) + 1);
    }

    Py_RETURN_NONE;
}

static PyObject * PyEngineSetAnimation(PyEngine * self, PyObject * w) {
    int i, crouch;

    if (!PyArg_ParseTuple(w, "ip", &i, &crouch))
        return nullptr;

    self->ref->players[i].set_crouch(crouch);

    Py_RETURN_NONE;
}

static PyObject * PyEngineGetOnTrace(PyEngine * self, void *) {
    auto newref = self->ref->onTrace.incref();
    return newref == nullptr ? Py_NewRef(Py_None) : newref;
}

static int PyEngineSetOnTrace(PyEngine * self, PyObject * o, void *) {
    self->ref->onTrace.retain(PyCallable_Check(o) ? o : nullptr);

    return 0;
}

static PyObject * PyEngineGetDefault(PyEngine * self, void *) {
    return self->ref->vxlData.defaultMaterial.incref();
}

static int PyEngineSetDefault(PyEngine * self, PyObject * o, void *) {
    if (!PyObject_TypeCheck(o, &MaterialType)) {
        PyErr_SetString(PyExc_TypeError, "must be Material");
        return -1;
    }

    self->ref->vxlData.defaultMaterial.retain(o);
    return 0;
}

static PyObject * PyEngineGetWater(PyEngine * self, void *) {
    return self->ref->vxlData.waterMaterial().incref();
}

static int PyEngineSetWater(PyEngine * self, PyObject * o, void *) {
    if (!PyObject_TypeCheck(o, &MaterialType)) {
        PyErr_SetString(PyExc_TypeError, "must be Material");
        return -1;
    }

    self->ref->vxlData.waterMaterial().retain(o);
    return 0;
}

static PyMappingMethods PyEngineMapping = {
    .mp_length        = NULL,
    .mp_subscript     = binaryfunc(PyEngineGetitem),
    .mp_ass_subscript = objobjargproc(PyEngineSetitem)
};

static PyMethodDef PyEngineMethods[] = {
    {"cast",          PyCFunction(PyEngineDragRaycast),           METH_VARARGS,         NULL},
    {"attcoeff",      PyCFunction(PyEngineAttenuationCofficient), METH_O,               NULL},
    {"hccoeff",       PyCFunction(PyEngineHCCofficient),          METH_O,               NULL},
    {"opvalue",       PyCFunction(PyEngineOverpressureValue),     METH_O,               NULL},
    {"opduration",    PyCFunction(PyEngineOverpressureDuration),  METH_O | METH_STATIC, NULL},
    {"opimpulse",     PyCFunction(PyEngineOverpressureImpulse),   METH_O | METH_STATIC, NULL},
    {"exposed",       PyCFunction(PyEngineExposed),               METH_VARARGS,         NULL},
    {"step",          PyCFunction(PyEngineStep),                  METH_VARARGS,         NULL},
    {"add",           PyCFunction(PyEngineAdd),                   METH_VARARGS,         NULL},
    {"update",        PyCFunction(PyEngineUpdate),                METH_O,               NULL},
    {"dig",           PyCFunction(PyEngineDig),                   METH_VARARGS,         NULL},
    {"smash",         PyCFunction(PyEngineSmash),                 METH_VARARGS,         NULL},
    {"apply",         PyCFunction(PyEngineApply),                 METH_O,               NULL},
    {"clear",         PyCFunction(PyEngineClearMeth),             METH_NOARGS,          NULL},
    {"flush",         PyCFunction(PyEngineFlush),                 METH_NOARGS,          NULL},
    {"on_spawn",      PyCFunction(PyEngineOnSpawn),               METH_VARARGS,         NULL},
    {"on_despawn",    PyCFunction(PyEngineOnDespawn),             METH_VARARGS,         NULL},
    {"set_animation", PyCFunction(PyEngineSetAnimation),          METH_VARARGS,         NULL},
    {NULL                                                                                   }
};

static PyGetSetDef PyEngineGetset[] = {
    {"lag",         getter(PyEngineLag),         nullptr,                    "Average time elapsed in `Engine.step` (μs)", NULL},
    {"peak",        getter(PyEnginePeak),        nullptr,                    "Peak time elapsed in `Engine.lag` (μs)",     NULL},
    {"alive",       getter(PyEngineAlive),       nullptr,                    "Number of alive objects",                    NULL},
    {"total",       getter(PyEngineTotal),       nullptr,                    "Total number of registered objects",         NULL},
    {"usage",       getter(PyEngineUsage),       nullptr,                    "Approximate memory usage (byte)",            NULL},
    {"temperature", getter(PyEngineTemperature), nullptr,                    "Ambient temperature (°C)",                   NULL},
    {"pressure",    getter(PyEnginePressure),    nullptr,                    "Ambient pressure (Pa)",                      NULL},
    {"humidity",    getter(PyEngineHumidity),    nullptr,                    "Ambient relative humidity",                  NULL},
    {"wind",        getter(PyEngineWind),        nullptr,                    "Wind velocity (m/s)",                        NULL},
    {"density",     getter(PyEngineDensity),     nullptr,                    "Air density (kg/m³)",                        NULL},
    {"mach",        getter(PyEngineMach),        nullptr,                    "Speed of sound (m/s)",                       NULL},
    {"ppo2",        getter(PyEnginePPO2),        nullptr,                    "Partial pressure of oxygen (Pa)",            NULL},
    {"on_trace",    getter(PyEngineGetOnTrace),  setter(PyEngineSetOnTrace), "Object position update callback",            NULL},
    {"default",     getter(PyEngineGetDefault),  setter(PyEngineSetDefault), "Default material",                           NULL},
    {"water",       getter(PyEngineGetWater),    setter(PyEngineSetWater),   "Water material",                             NULL},
    {NULL                                                                                                                      }
};

PyTypeObject PyEngineType = {
    .ob_base       = PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name       = "Engine",
    .tp_basicsize  = sizeof(PyEngine),
    .tp_itemsize   = 0,
    .tp_dealloc    = destructor(PyEngineDealloc),
    .tp_as_mapping = &PyEngineMapping,
    .tp_flags      = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_HAVE_GC,
    .tp_traverse   = traverseproc(PyEngineTraverse),
    .tp_clear      = inquiry(PyEngineClear),
    .tp_methods    = PyEngineMethods,
    .tp_getset     = PyEngineGetset,
    .tp_init       = initproc(PyEngineInit),
    .tp_new        = newfunc(PyEngineNew),
};

void PyEngineReady() {
    PyType_Ready(&PyEngineType);
}
