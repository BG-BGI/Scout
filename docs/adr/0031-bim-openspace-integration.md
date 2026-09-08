# ADR-0031: BIM + OpenSpace Integration

**Date:** 2026-09-08  
**Status:** Accepted

## Context

Scout's SLAM/AMCL maps are in the robot `map` frame (metres, arbitrary origin).
Autodesk Construction Cloud (ACC) provides Revit room geometry; OpenSpace hosts
360° construction captures positioned on floor-plan sheets (sheet pixel coords).
The operator needs to reconcile SLAM scans with the as-built Revit model and
correlate robot position with OpenSpace capture history.

SLAM and AMCL remain unchanged — this integration is purely additive.

## Decision

### SE2+scale alignment transform

A single per-map `bim.alignment` block in `site.json` stores `{tx, ty, theta, scale}`:

```
sheet = scale * R(theta) * map + t
```

where `map` is robot map-frame metres and `sheet` is OpenSpace sheet pixels.
Two landmark correspondences (operator-measured) solve the four unknowns.

### Standalone proxy repo: `BG-BGI/openspace-acc-sdk`

Same monorepo pattern as `BG-BGI/schindler-rbl`:

- **`apps/web/`** — Next.js proxy (port 3100). Handles all auth:
  OpenSpace `api-key` header, ACC OAuth2 2-legged `client_credentials`.
  Exposes clean JSON/PNG endpoints at `/api/...` with no credentials required
  from callers.
- **`py/`** — `openspace-acc` Python package (httpx, no credentials).
  Installed into `scout-skills` via BuildKit secret + SHA pin, identical
  pattern to `schindler-rbl`.

### Scout repo additions

| Component | Change |
|---|---|
| `docker/scout-skills/bim.py` | SE2+scale math, pgm loading, deviation detection, overlay rendering |
| `docker/scout-skills/server.py` | 10 new MCP tools (bim_link, bim_set_alignment, bim_overlay, bim_deviations, bim_rooms, go_to_room, openspace_scans, openspace_nearest, openspace_timeline, openspace_field_notes) |
| `docker/fleet-status/server.py` | GET/PATCH `/api/sites/active/maps/<map>/bim`; `_norm_map_entry` preserves bim key |
| `docker-compose.yaml` | `openspace_acc` service (profile: full, port 3100); `OPENSPACE_ACC_URL` in scout_skills env |
| `docker/scout-skills/Dockerfile` | `openspace-acc` package install (BuildKit secret, SHA-pinned) |
| `webui/app.js` | BIM scan markers + room labels in drawMap; toggle buttons in Site panel |
| `webui/index.html` | `#bim-controls` toggle row in Site panel |

### site.json per-map bim schema (optional, absent = unlinked)

```json
"bim": {
  "acc_model_urn": "",
  "acc_level_name": "Level 1",
  "openspace_site_id": "",
  "openspace_sheet_id": "",
  "alignment": { "tx": 0.0, "ty": 0.0, "theta": 0.0, "scale": 1.0 }
}
```

`fleet_status._norm_map_entry` now preserves the `bim` key across all
`_write_site_json` calls (previously silently dropped).

### Alignment workflow

1. Robot at landmark A → record map pose from `/odom`.
2. Operator reads sheet (x,y) of same landmark from OpenSpace sheet viewer.
3. Repeat for landmark B (≥ 1 m from A for numerical stability).
4. `bim_set_alignment` with two `{map_x,map_y,sheet_x,sheet_y}` pairs → stored.

Optional: register AprilTags with known sheet coords → `bim_set_alignment --from-tags`
for least-squares over all tagged pairs.

### Four outputs

| Output | Tool | Dependency |
|---|---|---|
| Visual overlay | `bim_overlay` | alignment + openspace_sheet_id |
| Deviation detection | `bim_deviations` | alignment + acc_model_urn |
| Room navigation | `go_to_room` | alignment + acc_model_urn |
| OpenSpace imagery | `openspace_nearest`, `openspace_timeline` | alignment + openspace_site/sheet_id |

## Consequences

- SLAM/AMCL are untouched; this feature is a no-op unless `bim_link` is called.
- `openspace_acc` compose service only starts under `--profile full`; robot
  service unaffected.
- Building `scout-skills` image requires `gh_token` secret (same requirement
  as schindler-rbl since ADR-0030).
- `OPENSPACE_ACC_REF` in Dockerfile must be pinned to a SHA once the repo exists
  (currently `main`).
- `compute_deviations` is O(N·M) in numpy chunks; practical limit ~10 000 BIM
  wall cells. Sufficient for one floor of a typical building.
