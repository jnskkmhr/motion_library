# CLAW G1 import

Imports released robot trajectories from [JianuoCao/CLAW](https://huggingface.co/datasets/JianuoCao/CLAW).
No human-to-robot retargeting is performed. The selected source trajectories do not
include terrain; `shared/fit_overhead.py` fits a roof and posts to a fixed motion.

Run from the repository root after `uv sync --locked`:

```bash
uv run hf download JianuoCao/CLAW --repo-type dataset \
  --revision cbd516f611e46d52182d261ca1651563d8044850 --local-dir vendor/claw
uv run python data_generator/claw/import_claw.py --source vendor/claw \
  --clips 00452_crouch 00543_hand_crawl 02067_elbow_crawl --output outputs/claw
```

The importer expects `g1_xml/g1_29dof_rev_1_0.xml` and
`motion/<clip>/{trajectory_dynamic.csv,annotation.json}` inside `--source`.
It writes `g1_qpos.npz` and provenance. Use the [shared export commands](../shared/README.md)
to compute body states on the selected G1 USD asset and fit a passage.

Existing results: [`data/claw`](../../data/claw). Elbow crawl exceeds target wrist
limits; the included clip is retained for inspection. The original downloaded
dataset card did not state a data license; preserve upstream attribution and terms.
