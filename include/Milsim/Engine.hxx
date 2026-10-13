#pragma once

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

#include <Milsim/Vector.hxx>
#include <Milsim/AABB.hxx>

#include <Milsim/Fundamentals.hxx>

#include <unordered_map>
#include <utility>
#include <cstdint>
#include <vector>
#include <chrono>
#include <list>
#include <map>

#include <Python.hxx>

#include <common_c.h>
#include <pvx2/map.h>

#include <engine.h>

struct Object {
private:
    static uint64_t gidx;

    PyObject * _object; uint64_t _index; uint32_t _model;
    int _thrower; double _timestamp, _v0;

public:
    double mass, ballistic, area;
    Vector3d position, velocity;

    inline static void     flush() { gidx = 0;    }
    inline static uint64_t total() { return gidx; }

    inline Object(PyObject * o, const uint32_t model, const int i, const Vector3d & r, const Vector3d & v, const double t) :
    _object(o), _model(model), _thrower(i), _timestamp(t), _v0(v.abs()), position(r), velocity(v)
    { Py_INCREF(o); _index = gidx++; }

    inline ~Object() { Py_XDECREF(_object); }

    inline double energy() const { return 0.5 * mass * velocity.norm(); }

    inline PyObject * object()    const { return _object;    }
    inline uint64_t   index()     const { return _index;     }
    inline uint32_t   model()     const { return _model;     }
    inline int        thrower()   const { return _thrower;   }
    inline double     timestamp() const { return _timestamp; }
    inline double     v0()        const { return _v0;        }

    inline bool valid() const { return _object != nullptr; }
    inline void invalidate() { Py_DECREF(_object); _object = nullptr; }
};

struct Player {
    bool c; Vector * p; Vector * f;

    inline Player() : p(nullptr), f(nullptr) {}

    inline bool valid() const { return p != nullptr; }

    inline void set_crouch(bool b)          { c = b; }
    inline void set_position(Vector * v)    { p = v; }
    inline void set_orientation(Vector * v) { f = v; }

    inline bool     crouch()      const { return c;           }
    inline Vector3d position()    const { return Vector3d(p); }
    inline Vector3d orientation() const { return Vector3d(f); }

    inline Vector3d origin() const
    { return position().translate(0, 0, crouch() ? -1.05 : -1.1); }

    inline auto intersect(const Ray<double> & r) const {
        using namespace std;

        auto ray = r.translate(-origin()).pointAt(
            orientation().xOy().normal(), Vector3d(0, 1, 0)
        );

        auto & head  = Box::head<double>;
        auto & torso = crouch() ? Box::torsoc<double>     : Box::torso<double>;
        auto & legl  = crouch() ? Box::legc_left<double>  : Box::leg_left<double>;
        auto & legr  = crouch() ? Box::legc_right<double> : Box::leg_right<double>;
        auto & armr  = crouch() ? Box::armc_right<double> : Box::arm_right<double>;
        auto & arml  = crouch() ? Box::armc_left<double>  : Box::arm_left<double>;

        return min(
            [](auto & w1, auto & w2) { return w1 < w2; },
            head.intersect(ray),
            torso.intersect(ray),
            legl.intersect(ray),
            legr.intersect(ray),
            armr.intersect(ray),
            arml.intersect(ray.rot(Vector3d(0, 0, 1), -std::numbers::pi_v<double> / 4))
        );
    }

    inline auto exposed(const Vector3d & center) const {
        auto r0 = (center - origin()).pointAt(
            orientation().xOy().normal(), Vector3d(0, 1, 0)
        );

        auto & head  = Box::head<double>;
        auto & torso = crouch() ? Box::torsoc<double>     : Box::torso<double>;
        auto & legl  = crouch() ? Box::legc_left<double>  : Box::leg_left<double>;
        auto & legr  = crouch() ? Box::legc_right<double> : Box::leg_right<double>;
        auto & armr  = crouch() ? Box::armc_right<double> : Box::arm_right<double>;
        auto & arml  = crouch() ? Box::armc_left<double>  : Box::arm_left<double>;

        return std::tuple(
            head.aabb.exposed(r0),
            torso.aabb.exposed(r0),
            legl.aabb.exposed(r0),
            legr.aabb.exposed(r0),
            armr.aabb.exposed(r0.rot(Vector3d(0, 0, 1), -std::numbers::pi_v<double> / 4)),
            arml.aabb.exposed(r0)
        );
    }

};

enum class Terminal { flying, ricochet, penetration };

using ObjectQueue    = std::list<Object>;
using ObjectIterator = ObjectQueue::iterator;

struct Voxel {
    PyOwnedRef object; double durability;

    inline Voxel() : object(), durability(0) {}
    inline Voxel(PyObject * o, const double f) : object(o), durability(f) { Py_INCREF(o); }

    inline Material * material() const { return reinterpret_cast<Material *>(static_cast<PyObject *>(object)); }

    inline bool isub(double delta) { durability -= delta; return durability <= 0; }
};

class VoxelData {
private:
    Voxel water; std::unordered_map<int, Voxel> data;
public:
    PyOwnedRef defaultMaterial;

    inline VoxelData() { water.durability = std::numeric_limits<double>::infinity(); }

    inline auto & waterMaterial() { return water.object; }

    Voxel & set(int i, PyObject * o);
    Voxel & get(int x, int y, int z);

    inline Voxel & get(const Vector3i & R)
    { return get(R.x, R.y, R.z); }

    inline Voxel & set(int x, int y, int z, PyObject * o)
    { return set(map_to_xy(x, y) * MAP_Z + z, o); }

    inline Voxel & set(const Vector3i & R, PyObject * o)
    { return set(map_to_xy(R.x, R.y) * MAP_Z + R.z, o); }

    inline void erase(int x, int y, int z)
    { data.erase(map_to_xy(x, y) * MAP_Z + z); }

    inline void clear() { data.clear(); defaultMaterial.retain(nullptr); waterMaterial().retain(nullptr); }

    // This is only the lower bound.
    inline size_t usage() const {
        constexpr size_t entrySize = sizeof(int) + sizeof(Voxel);
        return sizeof(decltype(data)) + entrySize * data.size();
    }
};

struct Engine {
public:
    PyOwnedRef protocol;
    MapBuf * map;

    VoxelData vxlData;
    ObjectQueue objects;
    std::vector<Player> players;

    PyOwnedRef onTrace, onBlockHit, onPlayerHit, onDestroy;

    // Independent variables.
    double   temperature; // °C
    double   pressure;    // Pa
    double   humidity;    // 1
    Vector3d wind;        // m/s

private:
    // Derived variables.
    double _density;                     // kg/m³
    double _mach;                        // m/s
    double _ppo2;                        // Pa
    double _oxygenRelaxationFrequency;   // Hz
    double _nitrogenRelaxationFrequency; // Hz

    double _lag, _peak;

    inline int intersectPlayer(const Ray<double> &, Arc<double> &);

    inline bool terminal(Object &, const Vector3i &, const Vector3d &);
    inline void external(Object &, const double, const Vector3d &);
    inline bool impactPlayer(Object &, const int, const Vector3i &, const Ray<double> &, const Arc<double> &);
    inline bool impactSurface(Object &, const Vector3i &, const Vector3d & n, const Vector3d & r);

    void next(double t1, const double t2, ObjectIterator &);
public:
    inline Engine(PyObject * o) : protocol(o), _lag(0.0), _peak(0.0)
    { srand(time(NULL)); players.reserve(32); }

    inline bool indestructible(int x, int y, int z)
    { return 62 <= z || !map_is_solid(map, x, y, z); }

    inline bool unstable(int x₀, int y₀, int z₀) {
        for (int z = z₀ + 1; z < 62; z++) {
            if (!map_is_solid(map, x₀, y₀, z))
                return true;

            if (!vxlData.get(x₀, y₀, z).material()->crumbly)
                return false;
        }

        return false;
    }

    inline bool solid(const Vector3i & R) {
        return 0 <= R.x && R.x < MAP_X
            && 0 <= R.y && R.y < MAP_Y
            && 0 <= R.z && R.z < MAP_Z
            && map_is_solid(map, R.x, R.y, R.z);
    }

    inline double density()                     const { return _density;                     }
    inline double mach()                        const { return _mach;                        }
    inline double ppo2()                        const { return _ppo2;                        }
    inline double oxygenRelaxationFrequency()   const { return _oxygenRelaxationFrequency;   }
    inline double nitrogenRelaxationFrequency() const { return _nitrogenRelaxationFrequency; }

    inline double lag()  const { return _lag;  }
    inline double peak() const { return _peak; }

    inline size_t alive() const { return objects.size(); }
    inline size_t total() const { return Object::total(); }

    inline size_t usage() const { return vxlData.usage(); }

    void update();
    void clear();

    inline void trace(const uint64_t index, const Vector3d & r, const double value, bool origin)
    { onTrace(index, r.x, r.y, r.z, value, origin); }

    void step(const double t1, const double t2);

    double dragRaycast(double CD, double m, double A, double v₀, Vector3d, const Vector3d &);
    double HopkinsonCranzCoefficient(double);
    double attenuationCoefficient(double);
};
