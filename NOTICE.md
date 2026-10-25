# Attribution and data terms

The copied Isaac Lab integration/generation code retains its original SPDX
headers. `LICENSE` contains the Isaac Lab BSD-3-Clause code license. This license
applies to code; it does not grant a new license for datasets, model weights,
robot assets, or their derived motion files.

Upstream sources are recorded in each clip's provenance:

- Kimodo: https://github.com/nv-tlabs/kimodo (pinned revision in pyproject.toml).
  Model/text-encoder access and usage follow their respective upstream terms.
- CLAW: https://huggingface.co/datasets/JianuoCao/CLAW . The original downloaded
  card did not specify a dataset license; no new data license is asserted here.
- OmniRetarget: https://huggingface.co/datasets/omniretarget/OmniRetarget_Dataset .
  The source dataset card recorded MIT.
- PARC: https://huggingface.co/datasets/mxucg/PARC . Retain upstream data terms
  and attribution for the original motion/terrain pairs.
- Deep Whole-body Parkour: https://project-instinct.github.io/deep-whole-body-parkour/ .
  The released references and terrain retain their upstream terms.
- LAFAN: https://github.com/ubisoft/ubisoft-laforge-animation-dataset and
  https://huggingface.co/datasets/lvhaidong/LAFAN1_Retargeting_Dataset . Source
  terms are CC BY-NC-ND 4.0; converted outputs are kept local and ignored by Git.
- Mixamo: https://www.mixamo.com/ . The navigation previews use selected animations
  from https://huggingface.co/datasets/Linzhan/Mixamo-Animations-Characters at the
  revision recorded in data_generator/mixamo/navigation_manifest.json. Credit:
  Adobe Mixamo and the Linzhan Mou / UniMate dataset release. Adobe terms apply.
  Public files contain rendered G1 videos and provenance, not source FBX or derived
  animation arrays. This does not grant permission to redistribute or train on the data.
- G1 robot geometry: Unitree assets and the local newton-assets hand-pad variant.
  Robot meshes, model weights, and credentials are not bundled here.

Provenance, geometric reports, and conversion metadata are retained alongside
motion/video exports. No dynamic tracking success or training readiness is implied.
