# Copyright © 2024, 2026 rzrn

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from cpython.ref cimport PyTypeObject
from libcpp cimport bool as bool_t
from libcpp.pair cimport pair
from libc.math cimport floor

from pyspades.common cimport Vector, Vertex3
from pyspades.vxl cimport VXLData, MapBuf
from pyspades.common import Vertex3

from pyspades.world cimport Character

cdef public class Material[object Material, type MaterialType]:
    cdef public str name
    "Material name"

    cdef public double durability
    "Average number of seconds to break material with a shovel"

    cdef public double absorption
    "Amount of energy that material can absorb before breaking (J)"

    cdef public double density
    "Density of material (kg/m³)"

    cdef public double strength
    "Material cavity strength (Pa)"

    cdef public double ricochet
    "Conditional probability of ricochet"

    cdef public double deflecting
    "Minimum angle required for a ricochet to occur (radians)"

    cdef public bool_t crumbly
    "Whether material can crumble"

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

def digamma(y):
    return c_digamma[double](y)

def idigamma(y):
    return c_idigamma[double](y)

def igamma(y):
    return c_igamma[double](y)

def shapeScaleWeibull(p, x0, μ):
    return c_shapeScaleWeibull[double](p, x0, μ)

cdef public MapBuf * mapBufRef(object o):
    assert isinstance(o, VXLData)

    return (<VXLData> o).mapbuf

cdef public Vector * vectorRef(object o):
    assert isinstance(o, Vertex3)

    cdef Vertex3 v = o

    assert v.is_ref
    return v.value

cdef class WorldObject(Character):
    cdef:
        int x, y, z
        bool_t location_changed
    cdef public:
        object on_block_stepped

    cdef int update(self, double dt) except -1:
        cdef int retval = Character.update(self, dt)

        cdef int x = <int> floor(self.player.p.x)
        cdef int y = <int> floor(self.player.p.y)
        cdef int z = <int> floor(self.player.p.z)

        z += 2 if self.player.crouch else 3

        if self.x != x or self.y != y or self.z != z:
            self.x = x
            self.y = y
            self.z = z

            self.location_changed = True

        if self.location_changed and not self.player.airborne:
            self.on_block_stepped(x, y, z)

            self.location_changed = False

        return retval

cdef extern from "Milsim/PyEngine.hxx":
    cdef double c_stefanBoltzmann     "Fundamentals::stefanBoltzmann<double>"
    cdef double c_molarMassDryAir     "Fundamentals::molarMassDryAir<double>"
    cdef double c_molarMassWaterVapor "Fundamentals::molarMassWaterVapor<double>"
    cdef double c_gasConstant         "Fundamentals::gasConstant<double>"
    cdef double c_absoluteZero        "Fundamentals::absoluteZero<double>"

    cdef T c_ofMeters "ofMeters"[T](const T)
    cdef T c_toMeters "toMeters"[T](const T)

    cdef cppclass Vector3[T]:
        T x, y, z

        Vector3()
        Vector3(T, T, T)

    cdef Vector3[T] c_cone "cone"[T](const Vector3[T] &, const T)

    void PyEngineReady()
    PyTypeObject PyEngineType

cdef extern from "Math.hxx":
    cdef Real c_digamma "digamma"[Real](Real)
    cdef Real c_idigamma "idigamma"[Real](const Real)
    cdef Real c_igamma "igamma"[Real](const Real)
    cdef pair[Real, Real] c_shapeScaleWeibull "shapeScaleWeibull"[Real](const Real p, const Real x, const Real μ)

stefanBoltzmann     = c_stefanBoltzmann
molarMassDryAir     = c_molarMassDryAir
molarMassWaterVapor = c_molarMassWaterVapor
gasConstant         = c_gasConstant
absoluteZero        = c_absoluteZero

PyEngineReady()
Engine = <type> &PyEngineType

def ofMeters(float x): return c_ofMeters[double](x)
def toMeters(float y): return c_toMeters[double](y)

def cone(v, float deviation):
    cdef Vector3[double] u = c_cone[double](Vector3[double](v.x, v.y, v.z), deviation)

    return Vertex3(u.x, u.y, u.z)
