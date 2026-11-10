"""Generator for scout/test/fixtures/two_storey.ifc (needs ifcopenshell; run from anywhere).

    python scout/test/fixtures/make_ifc_fixture.py [out.ifc] [--unit m|mm|ft]

Layout (metres; every length is scaled into the project unit, so --unit mm or ft yields the
same building in millimetres or feet). Walls are 0.2 m thick, storeys are 3 m apart.

  Level 1, z = 0.0:  outer room, centrelines (0,0)-(6,4), 3 m walls
                     partition at x = 3 with a 0.9 x 2.1 m doorway (opening + IfcDoor)
                     free-standing "tall wall" at x = 1.5, y 1..2, 6 m high, contained in
                     Level 1 but crossing Level 2 as well
  Level 2, z = 3.0:  room, centrelines (0,0)-(4,3), 3 m walls
                     north wall has a 1.2 m wide window, sill 0.9 m, head 2.1 m
  Level 3, z = 6.0:  empty roof storey (nothing is solid at its slice height)

Placement matrices and wall/opening sizes are SI metres (the API converts them to the project
unit); door/window overall sizes and the storey Elevation attribute are in the project unit.
Guids are deterministic so regenerating gives a stable diff.
"""

import argparse
import math
import pathlib

import ifcopenshell
import ifcopenshell.api.aggregate
import ifcopenshell.api.context
import ifcopenshell.api.feature
import ifcopenshell.api.geometry
import ifcopenshell.api.project
import ifcopenshell.api.root
import ifcopenshell.api.spatial
import ifcopenshell.api.unit
import ifcopenshell.guid
import numpy as np

THICK = 0.2
WALL_H = 3.0
TALL_H = 6.0
STOREY_GAP = 3.0
DOOR_W, DOOR_H = 0.9, 2.1
WINDOW_W, WINDOW_SILL, WINDOW_H = 1.2, 0.9, 1.2
OPENING_DEPTH = 0.4
STAMP = '2026-11-10T00:00:00'

UNITS = {
    'm': ({'is_metric': True, 'raw': 'METERS'}, 1.0),
    'mm': ({'is_metric': True, 'raw': 'MILLIMETERS'}, 1000.0),
    'ft': ({'is_metric': False, 'raw': 'FEET'}, 1 / 0.3048),
}


class _Guids:
    def __init__(self):
        self.n = 0

    def __call__(self):
        self.n += 1
        return ifcopenshell.guid.compress(f'{self.n:032x}')


class Builder:
    def __init__(self, unit):
        length_unit, self.k = UNITS[unit]
        ifcopenshell.guid.new = _Guids()
        self.f = ifcopenshell.api.project.create_file(version='IFC4')
        project = ifcopenshell.api.root.create_entity(self.f, ifc_class='IfcProject', name='U8a')
        ifcopenshell.api.unit.assign_unit(self.f, length=length_unit)
        model = ifcopenshell.api.context.add_context(self.f, context_type='Model')
        self.body = ifcopenshell.api.context.add_context(
            self.f,
            context_type='Model',
            context_identifier='Body',
            target_view='MODEL_VIEW',
            parent=model,
        )
        site = self._spatial('IfcSite', 'Site')
        building = self._spatial('IfcBuilding', 'Building')
        ifcopenshell.api.aggregate.assign_object(self.f, [site], project)
        ifcopenshell.api.aggregate.assign_object(self.f, [building], site)
        self.building = building

    def _spatial(self, ifc_class, name):
        e = ifcopenshell.api.root.create_entity(self.f, ifc_class=ifc_class, name=name)
        ifcopenshell.api.geometry.edit_object_placement(self.f, product=e)
        return e

    def storey(self, name, elevation_m):
        s = self._spatial('IfcBuildingStorey', name)
        m = np.eye(4)
        m[2, 3] = elevation_m
        ifcopenshell.api.geometry.edit_object_placement(self.f, product=s, matrix=m)
        s.Elevation = elevation_m * self.k
        ifcopenshell.api.aggregate.assign_object(self.f, [s], self.building)
        return s

    def _frame(self, a, b, z):
        (x0, y0), (x1, y1) = a, b
        length = math.hypot(x1 - x0, y1 - y0)
        c, s = (x1 - x0) / length, (y1 - y0) / length
        m = np.array([[c, -s, 0, x0], [s, c, 0, y0], [0, 0, 1, z], [0, 0, 0, 1]])
        return length, m

    def _local(self, frame, dx, dy, dz):
        m = frame.copy()
        m[:3, 3] += frame[:3, :3] @ np.array([dx, dy, dz])
        return m

    def wall(self, storey, name, a, b, z, height=WALL_H):
        length, frame = self._frame(a, b, z)
        wall = ifcopenshell.api.root.create_entity(self.f, ifc_class='IfcWall', name=name)
        rep = ifcopenshell.api.geometry.add_wall_representation(
            self.f,
            context=self.body,
            length=length,
            height=height,
            thickness=THICK,
            offset=-THICK / 2,
        )
        ifcopenshell.api.geometry.assign_representation(self.f, product=wall, representation=rep)
        ifcopenshell.api.geometry.edit_object_placement(self.f, product=wall, matrix=frame)
        ifcopenshell.api.spatial.assign_container(
            self.f, products=[wall], relating_structure=storey
        )
        return wall, frame

    def opening(self, wall, frame, at, width, sill, height):
        void = ifcopenshell.api.root.create_entity(self.f, ifc_class='IfcOpeningElement')
        rep = ifcopenshell.api.geometry.add_wall_representation(
            self.f,
            context=self.body,
            length=width,
            height=height,
            thickness=OPENING_DEPTH,
            offset=-OPENING_DEPTH / 2,
        )
        ifcopenshell.api.geometry.assign_representation(self.f, product=void, representation=rep)
        m = self._local(frame, at, 0.0, sill)
        ifcopenshell.api.geometry.edit_object_placement(self.f, product=void, matrix=m)
        ifcopenshell.api.feature.add_feature(self.f, feature=void, element=wall)
        return void, m

    def door(self, storey, void, void_matrix, width, height):
        door = ifcopenshell.api.root.create_entity(self.f, ifc_class='IfcDoor', name='Door')
        rep = ifcopenshell.api.geometry.add_door_representation(
            self.f,
            context=self.body,
            overall_width=width * self.k,
            overall_height=height * self.k,
        )
        ifcopenshell.api.geometry.assign_representation(self.f, product=door, representation=rep)
        ifcopenshell.api.geometry.edit_object_placement(
            self.f, product=door, matrix=void_matrix
        )
        ifcopenshell.api.spatial.assign_container(
            self.f, products=[door], relating_structure=storey
        )
        ifcopenshell.api.feature.add_filling(self.f, opening=void, element=door)

    def window(self, storey, void, void_matrix, width, height):
        window = ifcopenshell.api.root.create_entity(self.f, ifc_class='IfcWindow', name='Window')
        rep = ifcopenshell.api.geometry.add_window_representation(
            self.f,
            context=self.body,
            overall_width=width * self.k,
            overall_height=height * self.k,
        )
        ifcopenshell.api.geometry.assign_representation(self.f, product=window, representation=rep)
        ifcopenshell.api.geometry.edit_object_placement(
            self.f, product=window, matrix=void_matrix
        )
        ifcopenshell.api.spatial.assign_container(
            self.f, products=[window], relating_structure=storey
        )
        ifcopenshell.api.feature.add_filling(self.f, opening=void, element=window)


def _ring(b, storey, prefix, w, d, z):
    half = THICK / 2
    corners = np.array([(0, 0), (w, 0), (w, d), (0, d)], dtype=float)
    walls = {}
    for i, name in enumerate(('South', 'East', 'North', 'West')):
        p, q = corners[i], corners[(i + 1) % 4]
        u = (q - p) / np.hypot(*(q - p))
        a, e = p - u * half, q + u * half
        walls[name] = b.wall(storey, f'{prefix} {name}', tuple(a), tuple(e), z)
    return walls


def build(unit='m'):
    b = Builder(unit)
    levels = (('Level 1', 0.0), ('Level 2', 3.0), ('Level 3', 6.0))
    l1, l2, _roof = (b.storey(n, e) for n, e in levels)
    _ring(b, l1, 'L1', 6.0, 4.0, 0.0)
    part, frame = b.wall(l1, 'L1 Partition', (3.0, 0.1), (3.0, 3.9), 0.0)
    void, m = b.opening(part, frame, at=1.5, width=DOOR_W, sill=0.0, height=DOOR_H)
    b.door(l1, void, m, DOOR_W, DOOR_H)
    b.wall(l1, 'Tall wall', (1.5, 1.0), (1.5, 2.0), 0.0, height=TALL_H)
    ring2 = _ring(b, l2, 'L2', 4.0, 3.0, STOREY_GAP)
    north, nframe = ring2['North']
    void, m = b.opening(
        north, nframe, at=(4.2 - WINDOW_W) / 2, width=WINDOW_W, sill=WINDOW_SILL, height=WINDOW_H
    )
    b.window(l2, void, m, WINDOW_W, WINDOW_H)
    b.f.header.file_name.time_stamp = STAMP
    return b.f


def main():
    p = argparse.ArgumentParser()
    default = pathlib.Path(__file__).with_name('two_storey.ifc')
    p.add_argument('out', nargs='?', default=str(default))
    p.add_argument('--unit', choices=sorted(UNITS), default='m')
    args = p.parse_args()
    build(args.unit).write(args.out)


if __name__ == '__main__':
    main()
