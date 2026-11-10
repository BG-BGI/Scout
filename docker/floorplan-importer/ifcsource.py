"""IFC storeys -> wall geometry for planraster (ifcopenshell + shapely + numpy).

A pure library: no HTTP, no globals, no files written. Memory is about 6x the IFC size per
open model, so open once per job and slice one storey at a time.

Public API
----------
    open_model(source, *, max_bytes=MAX_IFC_BYTES) -> IfcModel
    list_storeys(model) -> list[Storey]
    slice_storey(model, storey, *, slice_height=DEFAULT_SLICE_HEIGHT, include=DEFAULT_INCLUDE,
                 exclude=DEFAULT_EXCLUDE, face_budget=DEFAULT_FACE_BUDGET, threads=None)
                 -> StoreySlice
    iterator_threads(requested=None) -> int

`source` is IFC STEP text as bytes, or a str / os.PathLike path (a str is always a path).
`.ifczip` and `.ifcxml` are rejected. `storey` is a `Storey` or its integer `Storey.id`.

Types (frozen dataclasses)
    IfcModel     file, unit_scale (metres per project length unit)
    Storey       id (STEP id, unique and stable for one file), global_id, name (raw, may be ""),
                 label (name, else long name, else "Storey N"), elevation_m, floor (1..N)
    StoreySlice  storey, z (absolute slice height, m), geometry (planraster.Geometry),
                 bounds (minx, miny, maxx, maxy in m), faces (triangles examined)

Units and frames
    Everything returned is in metres. The geometry iterator already emits metres whatever the
    file's length unit (IfcOpenShell 0.9.0.post1, length-unit 1.0), so mesh coordinates are used
    as they come; storey placements and the Elevation attribute are in project units and are
    multiplied by `unit_scale` once. A millimetre file therefore reads like the same model in
    metres, and scaling both would put walls 1000x off the slice plane.
    Coordinates are the IFC world frame (placement chain applied), x east / y north / z up, so
    `geometry` is y-up like planraster's source frame. No origin shift is applied, and real
    exports can sit hundreds of metres from (0, 0); `bounds` lets the caller pick an origin.
    Rasterize with an identity-scale transform:

        s = ifcsource.slice_storey(model, storey)
        T = calib.map_frame(1.0, rotation, origin)
        grid, meta = planraster.rasterize(s.geometry, T, resolution)

Slicing
    The plane is z = storey elevation + slice_height (default 0.241 m, the lidar height).
    It cuts every included element in the model whatever storey contains it, so a wall that
    crosses two floors shows on both. The section is filled by non-zero winding, so
    overlapping shells inside one element (door lining + leaf) do not cancel. Openings are
    subtracted by the geometry iterator (disable-opening-subtractions is off). Solid counts as
    present on [bottom, top): a plane exactly at a bottom face cuts the element, one exactly at
    a top face does not.
    `geometry.fills` holds one Fill per polygon: exterior ring first, then hole rings.
    `include` is a tuple of IFC class names (subtypes follow, and so do decomposed parts such
    as curtain wall panels and stair flights). The default omits IfcDoor on purpose: a door
    leaf is solid at 0.241 m and would seal every doorway, and planraster already bridges door
    gaps on its own copy for inside/outside detection. A window or curtain wall is only in the
    slice when the plane crosses it, so a sill above the slice height leaves the wall solid
    below it, which is what the lidar sees; a sill at or under it adds the thin frame/glass.
    `exclude` removes classes (and subtypes) after `include`: spaces, openings, furniture, MEP.

Budgets
    One storey per call. The iterator uses min(MAX_THREADS, cpu) threads. `face_budget` caps
    the triangles examined (all included elements, not only those the plane cuts); iteration
    stops as soon as it is passed, so `FaceBudgetError.count` is the running total at that
    moment, not the model total.

Errors (all subclass IfcSourceError, a ValueError); suggested HTTP status in brackets
    IfcParseError [422]       unreadable, truncated, wrong type, unsupported schema, bad unit
    NoStoreysError [422]      no IfcBuildingStorey in the file
    UnknownStoreyError [404]  storey id not in the file; `.storey_id`
    EmptySliceError [422]     nothing solid at the slice height; message names the storey;
                              `.storey` (Storey), `.z`
    IfcTooLargeError [413]    base of the next two; `.count`, `.limit`
      FileTooLargeError         input bigger than max_bytes (bytes)
      FaceBudgetError           more than face_budget triangles (faces)
    IfcSourceError [422]      bad arguments (slice_height, class names, budgets, source type)
"""

import dataclasses
import math
import os

import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util.placement
import ifcopenshell.util.unit
import numpy as np
import planraster
import shapely

MAX_IFC_BYTES = 256 * 1024 * 1024
DEFAULT_SLICE_HEIGHT = 0.241
DEFAULT_FACE_BUDGET = 3_000_000
MAX_THREADS = 4
DEFAULT_INCLUDE = (
    "IfcWall",
    "IfcColumn",
    "IfcSlab",
    "IfcStair",
    "IfcStairFlight",
    "IfcWindow",
    "IfcCurtainWall",
)
DEFAULT_EXCLUDE = (
    "IfcSpace",
    "IfcOpeningElement",
    "IfcFurnishingElement",
    "IfcDistributionElement",
)
SNAP_M = 1e-6
MIN_AREA_M2 = 1e-9
MIN_NORMAL = 1e-12
WINDING_CHUNK = 256
STEP_MAGIC = b"ISO-10303-21"
STEP_TRAILER = b"END-ISO-10303-21"
BOM = b"\xef\xbb\xbf"
HEAD_BYTES = 1024
TAIL_BYTES = 1024
FALLBACK_LABEL = "Storey"


class IfcSourceError(ValueError):
    pass


class IfcParseError(IfcSourceError):
    pass


class NoStoreysError(IfcSourceError):
    pass


class UnknownStoreyError(IfcSourceError):
    def __init__(self, storey_id):
        super().__init__(f"no storey with id {storey_id!r} in this model")
        self.storey_id = storey_id


class EmptySliceError(IfcSourceError):
    def __init__(self, storey, z, slice_height):
        super().__init__(
            f"storey {storey.label!r} has nothing solid at z = {z:.3f} m "
            f"(elevation {storey.elevation_m:.3f} m + slice height {slice_height:.3f} m)"
        )
        self.storey = storey
        self.z = z


class IfcTooLargeError(IfcSourceError):
    def __init__(self, message, count, limit):
        super().__init__(message)
        self.count = count
        self.limit = limit


class FileTooLargeError(IfcTooLargeError):
    pass


class FaceBudgetError(IfcTooLargeError):
    pass


@dataclasses.dataclass(frozen=True, eq=False)
class IfcModel:
    file: object
    unit_scale: float


@dataclasses.dataclass(frozen=True)
class Storey:
    id: int
    global_id: str
    name: str
    label: str
    elevation_m: float
    floor: int


@dataclasses.dataclass(frozen=True, eq=False)
class StoreySlice:
    storey: Storey
    z: float
    geometry: planraster.Geometry
    bounds: tuple
    faces: int


def open_model(source, *, max_bytes=MAX_IFC_BYTES):
    limit = _positive_int(max_bytes, "max_bytes")
    model_file = _read(source, limit)
    scale = _unit_scale(model_file)
    if not model_file.by_type("IfcBuildingStorey"):
        raise NoStoreysError("the model has no IfcBuildingStorey")
    return IfcModel(model_file, scale)


def _unit_scale(model_file):
    try:
        scale = float(ifcopenshell.util.unit.calculate_unit_scale(model_file))
    except (RuntimeError, ValueError, TypeError, AttributeError, ArithmeticError) as e:
        raise IfcParseError(f"the model's length unit is unusable: {e}") from e
    if not math.isfinite(scale) or scale <= 0:
        raise IfcParseError("the model's length unit is unusable")
    return scale


def list_storeys(model):
    rows = []
    for s in model.file.by_type("IfcBuildingStorey"):
        elevation = float(ifcopenshell.util.placement.get_storey_elevation(s)) * model.unit_scale
        rows.append((elevation, s.Name or "", s.id(), s))
    rows.sort(key=lambda r: r[:3])
    return [_storey(floor, *row) for floor, row in enumerate(rows, start=1)]


def slice_storey(
    model,
    storey,
    *,
    slice_height=DEFAULT_SLICE_HEIGHT,
    include=DEFAULT_INCLUDE,
    exclude=DEFAULT_EXCLUDE,
    face_budget=DEFAULT_FACE_BUDGET,
    threads=None,
):
    height = _positive(slice_height, "slice_height")
    budget = _positive_int(face_budget, "face_budget")
    target = _find_storey(model, storey)
    z = target.elevation_m + height
    elements = _elements(model.file, include, exclude)
    polygons, faces = _cut_polygons(model, elements, z, budget, iterator_threads(threads))
    merged = _union(polygons)
    if not merged:
        raise EmptySliceError(target, z, height)
    return StoreySlice(target, z, _geometry(merged), _bounds(merged), faces)


def iterator_threads(requested=None):
    cap = min(MAX_THREADS, os.cpu_count() or 1)
    if requested is None:
        return cap
    return max(1, min(_positive_int(requested, "threads"), MAX_THREADS))


def _storey(floor, elevation, name, step_id, entity):
    label = name.strip() or (entity.LongName or "").strip() or f"{FALLBACK_LABEL} {floor}"
    return Storey(step_id, entity.GlobalId or "", name, label, elevation, floor)


def _find_storey(model, storey):
    wanted = getattr(storey, "id", storey)
    for s in list_storeys(model):
        if s.id == wanted:
            return s
    raise UnknownStoreyError(wanted)


def _positive(value, what):
    try:
        v = float(value)
    except (TypeError, ValueError) as e:
        raise IfcSourceError(f"{what} must be a number") from e
    if not math.isfinite(v) or v <= 0:
        raise IfcSourceError(f"{what} must be finite and > 0")
    return v


def _positive_int(value, what):
    v = _positive(value, what)
    if v != int(v):
        raise IfcSourceError(f"{what} must be a whole number")
    return int(v)


def _read(source, limit):
    if isinstance(source, (bytes, bytearray, memoryview)):
        data = bytes(source)
        _check_step(len(data), data[:HEAD_BYTES], data[-TAIL_BYTES:], limit)
        return _parse(lambda: ifcopenshell.file.from_string(data.decode("utf-8", "replace")))
    try:
        path = os.fspath(source)
    except TypeError as e:
        raise IfcSourceError("source must be IFC bytes or a path") from e
    size, head, tail = _probe(path)
    _check_step(size, head, tail, limit)
    return _parse(lambda: ifcopenshell.open(path))


def _probe(path):
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as fh:
            head = fh.read(HEAD_BYTES)
            fh.seek(max(size - TAIL_BYTES, 0))
            return size, head, fh.read()
    except OSError as e:
        raise IfcParseError(f"cannot read the IFC file: {e}") from e


def _check_step(size, head, tail, limit):
    if size > limit:
        raise FileTooLargeError(f"IFC is {size} bytes; the cap is {limit}", size, limit)
    if not head.lstrip(BOM + b" \t\r\n").startswith(STEP_MAGIC):
        raise IfcParseError(
            "not an IFC STEP file (no ISO-10303-21 header; .ifczip and .ifcxml are unsupported)"
        )
    if STEP_TRAILER not in tail:
        raise IfcParseError("the IFC file is truncated (no END-ISO-10303-21 trailer)")


def _parse(opener):
    try:
        parsed = opener()
        schema = parsed.schema_identifier
    except (ifcopenshell.Error, RuntimeError, OSError) as e:
        raise IfcParseError(f"not a readable IFC file: {e}") from e
    if not schema:
        raise IfcParseError("not a readable IFC file: no schema")
    return parsed


def _by_type(model_file, name):
    try:
        return model_file.by_type(name)
    except RuntimeError as e:
        raise IfcSourceError(f"unknown IFC class {name!r} for schema {model_file.schema}") from e


def _elements(model_file, include, exclude):
    banned = tuple(exclude)
    for name in banned:
        _by_type(model_file, name)
    wanted = {}
    for name in include:
        for e in _by_type(model_file, name):
            wanted[e.id()] = e
            for part in _parts(e):
                wanted[part.id()] = part
    return [e for e in wanted.values() if not any(e.is_a(b) for b in banned)]


def _parts(element):
    """Aggregated and nested parts only; util.element.get_decomposition also returns openings
    and their fillings, which would put every IfcDoor back in through its wall."""
    found, queue = {}, [element]
    while queue:
        current = queue.pop()
        for rel in (*getattr(current, "IsDecomposedBy", ()), *getattr(current, "IsNestedBy", ())):
            for part in rel.RelatedObjects:
                if part.id() not in found:
                    found[part.id()] = part
                    queue.append(part)
    return found.values()


def _cut_polygons(model, elements, z, budget, threads):
    if not elements:
        return [], 0
    settings = ifcopenshell.geom.settings()
    settings.set("use-world-coords", True)
    settings.set("disable-opening-subtractions", False)
    it = ifcopenshell.geom.iterator(settings, model.file, threads, include=elements)
    polygons, faces = [], 0
    if not it.initialize():
        return polygons, faces
    while True:
        geo = it.get().geometry
        tris = np.asarray(geo.faces, dtype=np.int64).reshape(-1, 3)
        faces += len(tris)
        if faces > budget:
            raise FaceBudgetError(f"more than {budget} faces (reached {faces})", faces, budget)
        verts = np.asarray(geo.verts, dtype=float).reshape(-1, 3)
        polygons.extend(_element_polygons(verts, tris, z))
        if not it.next():
            return polygons, faces


def _element_polygons(verts, tris, z):
    if len(tris) == 0 or not verts[:, 2].min() <= z <= verts[:, 2].max():
        return []
    segs = _section(verts, tris, z)
    if len(segs) == 0:
        return []
    noded = shapely.node(shapely.multilinestrings(shapely.linestrings(segs)))
    cells = shapely.get_parts(shapely.polygonize(shapely.get_parts(noded)))
    if len(cells) == 0:
        return []
    inside = _winding(shapely.get_coordinates(shapely.point_on_surface(cells)), segs) != 0
    return list(cells[inside])


def _section(verts, tris, z):
    """Oriented plane/triangle segments, interior on the left; shared edges match exactly."""
    tv = verts[tris]
    nxt = np.roll(tv, -1, axis=1)
    above = tv[:, :, 2] > z
    crossing = above != np.roll(above, -1, axis=1)
    keep = crossing.sum(axis=1) == 2
    if not keep.any():
        return np.empty((0, 2, 2))
    tv, nxt, crossing = tv[keep], nxt[keep], crossing[keep]
    swap = _lex_greater(tv, nxt)[..., None]
    lo, hi = np.where(swap, nxt, tv), np.where(swap, tv, nxt)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (z - lo[..., 2]) / (hi[..., 2] - lo[..., 2])
        points = lo[..., :2] + t[..., None] * (hi[..., :2] - lo[..., :2])
    ends = points[crossing].reshape(-1, 2, 2)
    normal = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
    want = np.stack([-normal[:, 1], normal[:, 0]], axis=1)
    forward = np.einsum("ij,ij->i", ends[:, 1] - ends[:, 0], want)
    ends = np.where((forward < 0)[:, None, None], ends[:, ::-1], ends)
    usable = (np.hypot(*want.T) > MIN_NORMAL) & (np.abs(ends[:, 1] - ends[:, 0]).max(axis=1) > 0)
    return ends[usable]


def _lex_greater(a, b):
    gt = a > b
    eq = a == b
    return gt[..., 0] | (eq[..., 0] & (gt[..., 1] | (eq[..., 1] & gt[..., 2])))


def _winding(points, segs):
    """Winding number of the oriented segment soup around each point (+x ray crossings)."""
    a, b = segs[:, 0], segs[:, 1]
    rising = np.where(b[:, 1] > a[:, 1], 1, -1)
    out = np.zeros(len(points), dtype=np.int64)
    for start in range(0, len(points), WINDING_CHUNK):
        p = points[start : start + WINDING_CHUNK]
        px, py = p[:, 0, None], p[:, 1, None]
        straddle = (a[:, 1] > py) != (b[:, 1] > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            cross_x = a[:, 0] + (py - a[:, 1]) * (b[:, 0] - a[:, 0]) / (b[:, 1] - a[:, 1])
        out[start : start + len(p)] = ((straddle & (cross_x > px)) * rising).sum(axis=1)
    return out


def _union(polygons):
    if not polygons:
        return []
    merged = shapely.normalize(shapely.unary_union(polygons, grid_size=SNAP_M))
    parts = shapely.get_parts(merged)
    keep = [p for p in parts if p.geom_type == "Polygon" and p.area >= MIN_AREA_M2]
    return keep


def _geometry(polygons):
    fills = []
    for poly in polygons:
        rings = [poly.exterior, *poly.interiors]
        fills.append(planraster.Fill(rings=tuple(np.asarray(r.coords)[:-1] for r in rings)))
    return planraster.Geometry(fills=tuple(fills))


def _bounds(polygons):
    boxes = np.array([p.bounds for p in polygons])
    return (
        float(boxes[:, 0].min()),
        float(boxes[:, 1].min()),
        float(boxes[:, 2].max()),
        float(boxes[:, 3].max()),
    )
