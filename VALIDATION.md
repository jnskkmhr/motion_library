# Extraction validation — 2026-10-23

- `uv sync --locked` succeeded in a fresh local environment; `uv lock --check` passed.
- The catalog contains 54 clips and 80 paired video/NPZ variants.
- `tools/validate_library.py` passed: file hashes, named array shapes, finite values,
  unit quaternions, terrain metadata, video frame counts, and video FPS.
- The extracted CLAW importer reproduced the saved hand-crawl qpos and labels exactly.
- Terrain normalization preserved joint and body-state arrays.
- Kimodo pipeline dry-run resolved the relocated generator, exporter, and player paths.
  Model generation was not rerun; copied outputs retain their original provenance.
- The extracted USD exporter produced a six-frame ball-hand G1 dataset with
  29 joints and 35 body frames in the existing Isaac Lab environment.
- The extracted Isaac Lab player recorded the missing platform before-refinement
  preview: 199 frames at 50 FPS. No motion data was regenerated for this preview.
- Browser checks passed for video playback, source/search filters, before/after
  switching, GitHub links, repeat control, empty results, mobile layout, and direct
  opening of docs/index.html. Desktop and mobile layouts were visually inspected.
- Scoped Ruff checks and Python compilation passed.

These checks establish packaging and playback consistency. They do not establish
contact feasibility, policy tracking success, or readiness of every clip for training.

# ACP source additions — 2026-10-23

- Added 22 OmniRetarget references: all 29 released climbing clips are now present.
  Each new clip was exported on the ball-hand G1 USD asset at 50 Hz and recorded
  in Isaac Lab with its original supplied terrain.
- The pinned OmniRetarget downloader/importer reproduced the saved climb_00 qpos,
  joint labels, and terrain exactly using a fresh download directory.
- Added five local LAFAN previews from the pinned public Unitree G1 retargeting:
  two walking windows, two falling/getting-up windows, and one jumping window.
  The second get-up window was trimmed to end after recovery, before the next fall.
  Source frame ranges and the root XY translation are recorded in provenance.
- All 27 new references passed the sampled joint-limit check. Foot-geometry reports
  are included; this is not a full contact/dynamic validation.
- At that stage, the public catalog contained 76 clips / 102 NPZ-video pairs,
  and the local catalog 81 clips / 107 pairs. See the navigation update below
  for current counts.
- Public and local galleries passed browser checks, including 29 OmniRetarget and
  five LAFAN selections, video loading, source/download links, and mobile layout.
- Mixamo FBX import uses the uv-locked Blender 4.5 Python 3.11 wheel. A synthetic
  animated skeleton passed the FBX transform/timing round trip and joint-limited
  G1 IK. The initial unrelated dance preview has been removed from the galleries.
- Python compilation, scoped Ruff checks, JavaScript syntax, and uv lock checks passed.

LAFAN derivatives, local catalogs, and Mixamo asset directories are ignored by Git.
No claim is made to reproduce ACP's exact demonstrations, retargeting, contact
labels, policy rollouts, or training results.

# Navigation collection — 2026-10-25

- Selected and downloaded 82 Mixamo motions from a pinned UniMate dataset revision:
  stairs, ladders, climbing/slopes, vaults, crawling/hand support, crouching/ducking,
  jumps/landings, sitting/standing, and recovery/rolls. No conversion jobs failed.
- All were retargeted to the 29-joint ball-hand G1 and exported with its exact
  Isaac Lab USD asset. Recordings have 50 Hz timing and match their saved NPZs.
- Each joint/root trajectory is distinct. All 82 sampled joint-position-limit
  checks pass. These checks do not validate speed limits, self-collisions, terrain
  contacts, balance, or dynamic tracking. Ground-contact errors remain.
- New clips total 221.22 seconds. Individual durations are 0.42–11.42 seconds,
  below the 60-second cap. No short clips were slowed down or repeated.
- A real FBX cap check requested one second and produced a 0.98-second 50 Hz
  export, with truncation recorded in provenance.
- Public catalog: 158 clips, 102 NPZ/video pairs, 82 video-only previews.
  Local catalog: 163 clips, 189 NPZ/video pairs. Full array, quaternion, geometry,
  checksum, video frame-count, and FPS validation passed for both catalogs.
- Sampled frames were inspected across the collection, with source-human/G1 crawl
  comparison. The initial release lacked obstacle assets. The paired-terrain update
  below replaces those previews and adds terrain to every final Mixamo NPZ.
- Browser checks passed for all nine category counts and sample videos, direct
  clip links, search, mobile layout, retained OmniRetarget links, and local Mixamo
  NPZ downloads. Public previews do not advertise unavailable NPZ downloads.
- Scoped Ruff formatting/lint checks, JavaScript syntax and diff checks passed.

# Mixamo paired terrain — 2026-10-25

- Rebuilt all 82 Mixamo references with 372 static terrain boxes. Each final NPZ
  follows the existing `terrain_position`, `terrain_dimension` (full sizes),
  `terrain_quat` (xyzw), `terrain_id`, and `ground_height` convention. The existing
  Isaac Lab player renders those saved boxes without a Mixamo-specific loader.
- Original motion arrays and timing are bit-for-bit unchanged from `motion_raw.npz`.
  All 82 videos were regenerated from the paired NPZs; public and local recordings
  match, and frame counts/FPS agree with the saved references.
- All 11,061 saved poses were checked against the ball-hand G1 MJCF collision
  shapes, each terrain box, and the ground plane. Maximum measured robot–terrain
  penetration was 0 mm. Common named body positions agree with the exported USD
  reference within the 2 mm check tolerance; omitted fixed MJCF bodies are listed
  explicitly in each report.
- Each clip includes fitting provenance, the final NPZ hash, and an all-frame
  terrain collision report. Geometry is fitted to the G1 reference, not supplied
  by Mixamo. Conservative clearance can leave support gaps. The fitted ladder
  rungs and side rails may be nonuniform. No supporting contact, self-collision,
  contact force, balance, or dynamic tracking result is claimed.
- Inspected representative frames of stairs, ladders, vaults, crawling, jumping,
  sitting, and recovery. The gallery exposes terrain reports and labels the
  rebuilt recordings “Motion + terrain.”
- Full public/local catalog validation passed (158 public clips, including 82
  video-only previews; 163 local clips / 189 NPZ-video pairs). Browser checks
  passed for file opening, a local HTTP server, terrain/report labels, playback,
  category/search filters, mobile layout, and local Mixamo NPZ download links.
- Ruff formatting/lint, JavaScript syntax, uv lock, and diff checks passed.

# Terrain and motion curation — 2026-10-26

The preceding 82-clip Mixamo terrain check was insufficient: nonempty boxes and
zero penetration did not establish meaningful obstacles or supporting contact.
“Jump Up On Box” had terrain tops at ground height. This update replaces that
collection with a smaller reviewed subset.

- Deleted 460 verified terrain-free generated motion archives and their matching
  obsolete videos. The exact list is in `data/terrain_free_removal.json`.
- Removed 75 poor Mixamo pairings from active data and the public gallery. Seven
  candidates remain: one partial box climb, three crouch/duck clips, and three
  running gap jumps. Exclusion reasons and measurements are preserved in
  `data_generator/mixamo/rejected_clips.json`.
- Removed eight terrain-free early Kimodo and five terrain-free LAFAN clips, plus
  five Kimodo final references that failed their existing geometric checks.
  Terrain-bearing rejects, remaining intermediates and before-refinement variants
  were moved outside active data to `outputs/quarantine/2026-10-26/` for recovery.
- Every active NPZ has nonempty canonical box or triangle-mesh terrain. No raw
  qpos NPZ, raw export, or before-refinement variant remains under `data/`.
  Public catalog: 70 clips, 63 NPZ/video pairs and seven Mixamo video previews.
  Local catalog: 70 clips and 70 NPZ/video pairs. Both full validators passed.
- New catalog gates reject absent/empty terrain, restored rejected clips, and
  intermediate NPZs. Checked these failure cases with temporary fixtures, including
  compatibility with the existing optional identity-quaternion convention.
- Mixamo generation uses temporary raw/work folders and screens obstacle presence,
  foot-to-terrain gaps, and climb support gaps before promoting a paired result.
  Rebuilt `climbing_5` end to end: retarget, exact USD export, fitting, collision
  checks, quality screening, and video. Its temporary intermediates were cleaned
  up, and the active folder contains only the final NPZ plus video and reports.
- All seven retained Mixamo references pass the new coarse screening. These
  thresholds are exclusion criteria, not proof of physical support or tracking.
- Browser checks passed for seven Mixamo selections, absence of “Jump Up On Box,”
  terrain labels, video playback, filters, mobile layout, existing OmniRetarget
  downloads, and local Mixamo NPZ links. Python lint/format, JavaScript syntax,
  and diff checks passed.

# Publish retained Mixamo data — 2026-10-26

- Published all seven retained Mixamo final G1 NPZs with their paired terrain,
  videos, and provenance/review reports. Raw FBXs and quarantined clips remain
  excluded. The public catalog now contains 70 NPZ/video pairs, with no video-only
  entries.
- Full library validation passed. Browser checks confirmed GitHub and direct NPZ
  download links for each Mixamo entry. Every catalog NPZ, video and report is
  included in the tracked publication files; local NPZ hashes match the catalog.
- Updated the generator and ignore rules to preserve final-pair publication on
  subsequent builds. Recorded the user's instruction to work directly on main
  in this standalone repository.

# Publish requested ladder clips — 2026-10-26

Restored all five Mixamo ladder NPZ/video pairs with their fitted terrain, source
provenance, and existing quality reports at the user's explicit request. Their
hashes and restoration are recorded in `data/curation.json`; `quality_review.json`
labels them `published_for_inspection`, preserving the original issues. This is
publication of existing references, not a claim that ladder fitting is repaired.
The gallery now contains 75 paired references and five selectable ladder clips.
Browser checks verified all five videos, terrain labels, and NPZ links.
