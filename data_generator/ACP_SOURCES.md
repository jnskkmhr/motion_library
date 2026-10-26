# Motion sources cited by ACP

[ACP supplementary Table S1](https://arxiv.org/html/2610.31608v1) lists OmniRetarget
for box climbing; Mixamo for stairs, ladders and sitting; LAFAN for walking/get-up;
Kimodo for vaulting; and AME policy rollouts for sparse terrain.

| Source | Added here | Relationship to ACP |
| --- | --- | --- |
| OmniRetarget | 22 more G1 clips; all 29 published climbing clips now present | Same source dataset; exact ACP selection unknown |
| LAFAN | Importer retained; five terrain-free previews removed | Public Unitree retargeting, not ACP's own retargeting |
| Mixamo | Seven retained terrain-paired G1 candidates | Box climb, crouch/duck and gap jumps; 75 poor pairings excluded with recorded reasons |

The default gallery includes the seven retained Mixamo G1 NPZs, their fitted
terrain, videos and reports. `--include-local` also supports any separately prepared
local-only LAFAN candidates. Source FBXs and rejected clips are not published.

No ACP-specific reference trajectories, hand-authored contact labels, or AME policy
rollouts were retrieved. None of these additions reproduces the ACP training pipeline.
