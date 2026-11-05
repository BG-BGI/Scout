# ADR-0038: POI registry + tag-family naming

Status: accepted · Date: 2026-11-05

## Context

"tags" was ambiguous: the MCP `*_tag` tools are AprilTags while RFID/NFC/UHF
reads have their own `list_*_tags` tools. Operators also need to mark places
that have no physical marker.

## Decision

Three separate families, each its own registry and tool namespace:

| Family | Marker | Store | MCP |
|---|---|---|---|
| AprilTag | printed fiducial, detected | `maps/tags.db` | `detect_/register_/resurvey_/list_/delete_apriltag(s)`, `apriltag_watch` |
| RFID / NFC / UHF | physical tag, read | companion `rfid.db`/`nfc.db`/`uhf.db` | `list_rfid_tags`, `list_nfc_tags`, `list_uhf_tags` (unchanged) |
| POI | none — operator-marked map point | `maps/pois.db` | `register_poi`, `list_pois`, `update_poi`, `delete_poi`, `go_to_poi` |

- AprilTag MCP tools renamed (`detect_tags`→`detect_apriltags`, `register_tag`→
  `register_apriltag`, `resurvey_tag`, `list_tags`, `delete_tag`,
  `tag_watch`→`apriltag_watch`). No aliases. Storage (`tags.db`) and REST
  (`/api/apriltags`) unchanged; no migration.
- POI row: name (PK), category, notes, map_x/y/yaw, map_name (stamped on write;
  moving re-stamps), created/updated. `register_poi` without x/y marks the
  robot's current pose. `go_to_poi` refuses a POI from another map.
- Webui "POIs" panel (create by map tap, edit/move, delete, drive) via
  fleet_status `GET/POST /api/pois`, `POST/DELETE /api/pois/<name>`; markers
  share the corner Tags toggle.
- `poi.py` is duplicated byte-identically into scout-skills and fleet-status
  (separate images, same pattern as tags/tagpose); `test_poi.py` enforces it.

## Not done

POIs do not write waypoints, are not routable in patrols, and carry no
detection. Not yet verified on-robot or in a built image.
