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

### Gates 1 & 4 — UAV-driven matching table
The UAV (dione) publishes `CasualtyFixArray` with `casualty_id` + GPS per detection. The basestation maintains a **matching table** (runtime JSON, same pattern as old `helpers.py`) built from the UAV's detections. When a UGV report arrives, it is GPS-matched against this table to resolve the scoring-server casualty ID. The UAV is the source of truth for IDs in these gates.

### Gates 2 & 3 — Pre-loaded casualty ID JSON
Before the run starts, a JSON file is populated at the basestation with the known casualties and their scoring-server IDs. When a robot report arrives (with GPS location), it is matched against this file using GPS distance. Proposed schema:

```json
[
  {
    "casualty_id": 1,
    "lat": 39.9421,
    "lon": -75.1990
  }
]
```

File path: configurable via ROS param `casualty_id_path`, defaulting to e.g. `config/casualty_ids.json`.

---

## Gate Architecture

### Gate 1 — Casualty Location (`casualty_location_sub.py`)

**ROS inputs:**
- UAV (dione): `CasualtyFixArray` (dtc_msgs) on `/dione/casualty_info`
- UGV (jackals): `CasualtyFix` (dtc_msgs) on `/<robot>/casualty_report`

**`CasualtyFix` fields:** `casualty_id` (uint32), `location` (NavSatFix), `image` (Image), `time_ago` (Time)

**What needs to be done:**
- [ ] Implement callback: GPS-match incoming location against UAV matching table → call `location_report(lat, lon, level=1, id=resolved_id, system=robot)`
- `level` is hardcoded to `1` (robots cannot determine building floor level)

**Status:** `__init__` with subscriptions written; **callback body empty**; matching not implemented.

---

### Gate 2 — Triage Report (`triage_report_sub.py`)

**ROS inputs:**
- `Triage` (dtc_msgs) on `/<robot>/triage_report` (topic name TBD)

**`Triage` fields:** `casualty_id` (uint32), `location` (NavSatFixTimeAgo), `category` (Uint8TimeAgo)

**What needs to be done:**
- [ ] Implement `__init__` with subscriptions for each robot in `robots` param
- [ ] Implement callback: GPS-match `msg.location` against pre-loaded `casualty_ids.json` → call `triage_report(category=msg.category.value, id=resolved_id, system=robot)`
- Topic names: `/<robot>/triage_report` (subject to change)

**Status:** **Stub only** — class declared, no body.

---

### Gate 3 — Assessment Report (`assessement_report_sub.py`)

**ROS inputs:**
- `Assessment` (dtc_msgs) on `/<robot>/assessment_report` (topic name TBD)

**`Assessment` fields:** `casualty_id` (uint32), `location` (NavSatFixTimeAgo), `type` (String), `value` (Uint8TimeAgo)
- `type.data` = assessment field name (e.g. `"trauma_head"`, `"alertness_ocular"`)
- `value.value` = numeric assessment value; `value.time_ago` = measurement timestamp

**What needs to be done:**
- [ ] Implement `__init__` with subscriptions for each robot
- [ ] Implement callback: GPS-match `msg.location` against pre-loaded `casualty_ids.json` → call `trauma_report(type=msg.type.value, value=msg.type.value, time_ago=msg.type.time_ago, id=resolved_id, system=robot)`
- Topic names: `/<robot>/assessment_report` (subject to change)

**Status:** **Stub only** — class declared, no body.

---

### Gate 4 — Vitals Report (`vital_report_sub.py`)

**ROS inputs:**
- `HeartRate` (dtc_msgs) on `/<robot>/heart_rate` (topic name TBD)
- `RespirationRate` (dtc_msgs) on `/<robot>/respiration_rate` (topic name TBD)

**`HeartRate` / `RespirationRate` fields:** `casualty_id` (UInt64), `rate` (Float64), header timestamp
**GPS will be added to these messages** — pending dtc-msgs update (same `NavSatFixTimeAgo` pattern as `Triage`/`Assessment`).

**What needs to be done:**
- [ ] Implement `__init__` with subscriptions for HR and RR for each robot
- [ ] Once GPS is added to messages: GPS-match against UAV matching table → call `vitals_report(type='hr'/'rr', value=msg.rate.data, time_ago=..., id=resolved_id, system=robot)`
- [ ] `time_ago` = difference between `msg.header.stamp` and current ROS time
- Topic names: `/<robot>/heart_rate` and `/<robot>/respiration_rate` (subject to change)

**Status:** **Stub only** — class declared, no body.

---

### HMT Node (`hmt_node.py`)

**Status:** **Deferred.** HMT is a human-machine teaming variant of all gates (same structure, different API endpoints: `/api/hmt/casualty` and `/api/hmt/assessment`). Will be implemented after Gates 1–4 are complete. API functions already exist in `submission.py`.

---

### Always-On — System Location (`system_location_sub.py`)

**Status: Complete.** Subscribes to NavSatFix for each robot, posts to `/api/system_location` at configurable rate with retry logic.

---

## CMakeLists.txt — Needs Update

Currently only `submission_node.py` and `spoofer.py` are registered. All new nodes must be added to `install(PROGRAMS ...)`:

- [ ] `casualty_location_sub.py`
- [ ] `triage_report_sub.py`
- [ ] `assessement_report_sub.py`
- [ ] `vital_report_sub.py`
- [ ] `system_location_sub.py`
- [ ] `hmt_node.py` (once created)

---

## Open Questions

1. **Gate 4 GPS** — `HeartRate`/`RespirationRate` msgs need GPS added in dtc-msgs before Gate 4 matching can be implemented.
2. **Casualty ID JSON path** — `casualty_ids.json` schema is defined; exact config param name and default path to be confirmed.
3. **Topic naming** — All Gates 2–4 topic names (`/<robot>/triage_report`, etc.) are provisional and likely to change.
4. **Spoofer** — Will need to be updated to publish `Triage`, `Assessment`, `HeartRate`, `RespirationRate`, and `CasualtyFix` messages for testing.
