## The Requirement

The scoring server has been updated, so the old scorecard submissions no longer fit the requirements.
Submissions are now divided into **4 gates** plus **HMT**, each with its own API endpoint and ROS node.
The new gate-based pipeline **replaces** the old `submission_node.py`.

Each gate is launched conditionally from `gate_conditional.launch.py` via boolean args (`gate_1:=true`, etc.).
`system_location_sub.py` runs unconditionally alongside whichever gates are active.

---

## API Layer (`submission.py`) — Complete

All API functions are implemented for all gates:

| Function | Endpoint | Gate |
|---|---|---|
| `start_run()` | `/api/run/new` + `/api/run/start` | — |
| `post_system_location()` | `/api/system_location` | always-on |
| `location_report(lat, lon, level, id, system)` | `/api/location_report` | Gate 1 |
| `triage_report(category, id, system)` | `/api/triage` | Gate 2 |
| `trauma_report(type, value, time_ago, id, system)` | `/api/trauma` | Gate 3 |
| `vitals_report(type, value, time_ago, id, system)` | `/api/vitals` | Gate 4 |
| `hmt_location_report(lat, lon, category, time_ago, id, system)` | `/api/hmt/casualty` | HMT |
| `hmt_assessment_report(type, value, time_ago, id, system)` | `/api/hmt/assessment` | HMT |

---

## Casualty ID Resolution — All Gates

Two mechanisms, depending on gate:

### Gates 1, 4 & HMT — UAV-driven matching table
The UAV (dione) publishes `CasualtyFixArray` with `casualty_id` + GPS per detection. Each node maintains an
in-memory matching table built from the UAV's detections. When a UGV report arrives, it is GPS-matched
against this table to resolve the scoring-server casualty ID. If no match is found within threshold, the
report is still submitted with `id=None` (unmatched). The UAV is the source of truth for IDs in these gates.

### Gates 2 & 3 — Pre-loaded casualty ID JSON
Before the run starts, a JSON file is populated at the basestation with the known casualties and their
scoring-server IDs. When a robot report arrives (with GPS location), it is matched against this file using
GPS distance. Schema:

```json
[
  {
    "casualty_id": 1,
    "lat": 39.9421,
    "lon": -75.1990
  }
]
```

File path: configurable via ROS param `casualty_id_path` (default: `config/casualty_ids.json`).

---

## Gate Architecture

### Gate 1 — Casualty Location (`casualty_location_sub.py`) — Complete

**ROS inputs:**
- UAV (dione): `CasualtyFixArray` on `/dione/casualty_info`
- UGV (jackals): `CasualtyFix` on `/<robot>/casualty_report`

**`CasualtyFix` fields:** `casualty_id` (uint32), `location` (NavSatFix), `image` (Image), `time_ago` (Time)

**Behaviour:** GPS-matches UGV report against UAV table → `location_report(lat, lon, level=1, id, system)`.
If no match, submits with `id=None`. `level` hardcoded to `1`.

**Status: Complete.**

---

### Gate 2 — Triage Report (`triage_report_sub.py`) — Complete

**ROS inputs:**
- `Triage` (dtc_msgs) on `/<robot>/triage_report`

**`Triage` fields:** `casualty_id` (uint32), `location` (NavSatFixTimeAgo), `category` (Uint8TimeAgo)

**Behaviour:** GPS-matches `msg.location` against `casualty_ids.json` → `triage_report(category, id, system)`.
Drops report if no match within threshold.

**Status: Complete.**

---

### Gate 3 — Assessment Report (`assessement_report_sub.py`) — Complete

**ROS inputs:**
- `Assessment` (dtc_msgs) on `/<robot>/assessment_report`

**`Assessment` fields:** `casualty_id` (uint32), `location` (NavSatFixTimeAgo), `type` (String), `value` (Uint8TimeAgo)
- `type.data` = assessment field name (e.g. `"trauma_head"`, `"alertness_ocular"`)
- `value.value` = numeric assessment value; `value.time_ago` = measurement timestamp

**Behaviour:** GPS-matches `msg.location` against `casualty_ids.json` → `trauma_report(type, value, time_ago, id, system)`.
Drops report if no match within threshold.

**Status: Complete.**

---

### Gate 4 — Vitals Report (`vital_report_sub.py`) — Complete

**ROS inputs:**
- UAV (dione): `CasualtyFixArray` on `/dione/casualty_info`
- UGV (jackals): `HeartRate` on `/<robot>/heart_rate`, `RespirationRate` on `/<robot>/respiration_rate`

**`HeartRate` / `RespirationRate` fields:** `casualty_id` (uint32), `location` (NavSatFixTimeAgo), `rate` (Float64), `header`

**Behaviour:** GPS-matches `msg.location` against UAV detection table →
`vitals_report(type='hr'/'rr', value=msg.rate.data, time_ago, id, system)`.
`time_ago` = now − `msg.location.time_ago`. Drops report if no match within threshold.

**Status: Complete.**

---

### HMT Node (`hmt_node.py`) — Complete

**ROS inputs:**
- UAV (dione): `CasualtyFixArray` on `/dione/casualty_info`
- UGV (jackals): `Triage` on `/<robot>/triage_report`, `Assessment` on `/<robot>/assessment_report`

**Behaviour:**
- `Triage` → GPS-match against UAV table → `hmt_location_report(lat, lon, category, time_ago, id, system)`
- `Assessment` → GPS-match against UAV table → `hmt_assessment_report(type, value, time_ago, id, system)`
- If no UAV match within threshold: **still submits** with `id=None` and logs a warning (unmatched report).
  `time_ago` for triage = now − header.stamp; for assessment = now − `msg.value.time_ago`.

**Status: Complete.**

---

### Always-On — System Location (`system_location_sub.py`) — Complete

Subscribes to NavSatFix for each robot, posts to `/api/system_location` at configurable rate with retry logic.

---

## CMakeLists.txt — Complete

All nodes registered in `install(PROGRAMS ...)`:

- [x] `casualty_location_sub.py`
- [x] `triage_report_sub.py`
- [x] `assessement_report_sub.py`
- [x] `vital_report_sub.py`
- [x] `system_location_sub.py`
- [x] `hmt_node.py`

---

## Open Questions

1. **Topic naming** — All Gates 2–4 topic names (`/<robot>/triage_report`, etc.) are provisional and likely to change.
2. **Spoofer** — Will need to be updated to publish `Triage`, `Assessment`, `HeartRate`, `RespirationRate`, and `CasualtyFix` messages for testing.
