"""IFC job state and operations behind the importer's storey routes.

Glue between ifcsource (parse + slice) and planraster (grid); no HTTP and no import of
server.py. server.py imports this module only when ifcopenshell and shapely are installed, so
a PDF-only environment still runs the service. The API contract lives in server.py's docstring.

A job holds one open model and one StoreyState per storey. The raw upload is never kept: only
the parsed model, the sha256 and the size. Cached grids are bounded by trim_grids().
"""

import dataclasses
import math

import calib
import ifcsource
import planraster

DEFAULT_SLICE_HEIGHT = ifcsource.DEFAULT_SLICE_HEIGHT


@dataclasses.dataclass(eq=False)
class StoreyState:
    storey: object
    slice: dict | None = None
    grid: object = None
    meta: dict | None = None
    preview: dict | None = None
    seq: int = 0
    last_install: dict | None = None


def open_model(data, max_bytes):
    """(model, {storey id: StoreyState} in floor order) from IFC STEP bytes."""
    model = ifcsource.open_model(data, max_bytes=max_bytes)
    return model, {s.id: StoreyState(s) for s in ifcsource.list_storeys(model)}


def run_slice(model, storey, req, *, threads, face_budget=None):
    """Slice one storey and rasterize it with an identity-scale map frame.

    Returns (StoreySlice, origin, grid, meta). The origin defaults to the lower-left corner of
    the slice bounds (IFC world metres can sit hundreds of metres from (0, 0))."""
    extra = {} if face_budget is None else {"face_budget": face_budget}
    result = ifcsource.slice_storey(model, storey, slice_height=req["slice_height"],
                                    threads=threads, **extra)
    origin = req["origin"] or (result.bounds[0], result.bounds[1])
    frame = calib.map_frame(1.0, math.radians(req["rotation_deg"]), origin)
    grid, meta = planraster.rasterize(result.geometry, frame, req["resolution"],
                                      close_m=req["close_m"])
    return result, tuple(origin), grid, meta


def trim_grids(states, budget, keep):
    """Drop the oldest cached grids (never `keep`) until the rest fit `budget` bytes.

    A dropped storey reads as not sliced again (slice null, previews 409) but keeps its
    last_install. Returns the labels of the storeys dropped."""
    held = [s for s in states.values() if s.grid is not None]
    total = sum(s.grid.nbytes for s in held)
    dropped = []
    for state in sorted(held, key=lambda s: s.seq):
        if total <= budget:
            break
        if state is keep:
            continue
        total -= state.grid.nbytes
        state.grid = state.meta = state.preview = state.slice = None
        dropped.append(state.storey.label)
    return dropped


def storey_brief(storey):
    return {"id": storey.id, "label": storey.label, "elevation_m": storey.elevation_m,
            "floor": storey.floor}


def storey_json(state):
    s = state.storey
    return {"id": s.id, "global_id": s.global_id, "name": s.name, "label": s.label,
            "elevation_m": s.elevation_m, "floor": s.floor, "slice": state.slice,
            "last_install": state.last_install}


def slice_json(result, req, origin, preview, warnings):
    return {"storey": storey_brief(result.storey), "z": result.z,
            "slice_height": req["slice_height"], "rotation_deg": req["rotation_deg"],
            "origin": list(origin), "resolution": req["resolution"], "close_m": req["close_m"],
            "bounds": list(result.bounds), "faces": result.faces, "preview": preview,
            "warnings": warnings}


def sidecar(version, job, state, created_utc):
    s, sl = state.storey, state.slice
    return {
        "importer_version": version, "kind": "ifc",
        "source_file": {"name": job.filename, "sha256": job.sha256, "bytes": job.n_bytes},
        "storey": {"id": s.id, "global_id": s.global_id, "name": s.name, "label": s.label,
                   "elevation_m": s.elevation_m},
        "slice": {"slice_height": sl["slice_height"], "z": sl["z"],
                  "rotation_deg": sl["rotation_deg"], "origin": sl["origin"],
                  "resolution": sl["resolution"], "close_m": sl["close_m"],
                  "faces": sl["faces"]},
        "created_utc": created_utc}


def error_response(exc):
    """(status, JSON body) for an ifcsource.IfcSourceError."""
    if isinstance(exc, ifcsource.UnknownStoreyError):
        return 404, {"error": str(exc)}
    if isinstance(exc, ifcsource.IfcTooLargeError):
        return 413, {"error": str(exc), "count": exc.count, "limit": exc.limit}
    return 422, {"error": str(exc)}
