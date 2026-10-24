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
- G1 robot geometry: Unitree assets and the local newton-assets hand-pad variant.
  Robot meshes, model weights, and credentials are not bundled here.

Provenance, geometric reports, and conversion metadata are retained alongside
motion/video exports. No dynamic tracking success or training readiness is implied.
