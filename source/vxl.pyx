# Copyright © 2024–2026 rzrn

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

from pyspades.vxl cimport MAP_X, MAP_Y, MAP_Z, VXLData, MapBuf, map_to_xy, map_is_solid

cdef extern from "VXL.hxx":
    int traverseNode(int, int, int, MapBuf *)
    int c_deleteQueuePop "deleteQueuePop"()
    void c_deleteQueueClear "deleteQueueClear"()

cdef class VxlData(VXLData):
    cpdef int cull(self, int x, int y, int z):
        return traverseNode(x, y, z, self.mapbuf)

    cpdef int get_z(self, int x, int y, int zmin = 0, int zmax = 64, int zerr = 0):
        for z in range(zmin, zmax):
            if map_is_solid(self.mapbuf, x, y, z):
                return z

        return zerr

cdef extern from "world_c.cpp":
    MapBuf * global_map

    int c_can_see "can_see"(
        MapBuf *,
        float x0, float y0, float z0,
        float x1, float y1, float z1
    )

    int c_cast_ray "cast_ray"(
        MapBuf *, float x0, float y0, float z0,
        float x1, float y1, float z1, float length,
        long * x, long * y, long * z
    )

def can_see(VXLData data, float x0, float y0, float z0, float x1, float y1, float z1):
    global global_map
    global_map = data.mapbuf

    cdef bint retval = c_can_see(NULL, x0, y0, z0, x1, y1, z1)
    return retval

def cast_ray(VXLData data, float x0, float y0, float z0, float x1, float y1, float z1, float length):
    global global_map
    global_map = data.mapbuf

    cdef long x = -1, y = -1, z = -1

    if c_cast_ray(NULL, x0, y0, z0, x1, y1, z1, length, &x, &y, &z):
        return x, y, z

def deleteQueueClear():
    c_deleteQueueClear()

def onDeleteQueue():
    cdef int x, y, z, i

    while True:
        i = c_deleteQueuePop()

        if i < 0:
            return

        # TODO: this is very dirty
        z = i % MAP_Z
        x = (i // MAP_Z) % MAP_X
        y = (i // MAP_Z) // MAP_X

        yield x, y, z
