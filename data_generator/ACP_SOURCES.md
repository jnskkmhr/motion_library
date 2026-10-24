# Motion sources cited by ACP

[ACP supplementary Table S1](https://arxiv.org/html/2610.31608v1) lists OmniRetarget
for box climbing; Mixamo for stairs, ladders and sitting; LAFAN for walking/get-up;
Kimodo for vaulting; and AME policy rollouts for sparse terrain.

| Source | Added here | Relationship to ACP |
| --- | --- | --- |
| OmniRetarget | 22 more G1 clips; all 29 published climbing clips now present | Same source dataset; exact ACP selection unknown |
| LAFAN | Five locally built G1 previews; pinned downloader/converter | Public Unitree retargeting, not ACP's own retargeting |
| Mixamo | Local FBX-to-G1 importer; no motion files available yet | Supports the cited source; needs user exports and matching terrain |

The default gallery indexes publishable bundled data. `--include-local` creates
`docs/local.html`, `docs/local-catalog.js`, and `data/catalog.local.json`, including
local LAFAN/Mixamo derivatives. These files and derived data directories are
ignored by Git. The public gallery links to setup instructions for those sources.

No ACP-specific reference trajectories, hand-authored contact labels, or AME policy
rollouts were retrieved. None of these additions reproduces the ACP training pipeline.
