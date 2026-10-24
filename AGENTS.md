# Motion library development

This standalone repository owns offline motion generation/import, derived data,
and the static gallery. Tracker training continues in the separate Isaac Lab mimic
branch. Keep the two environments explicit; do not add a hard-coded local Isaac Lab
path to Python import search paths.

- Manage Python dependencies with uv and commit both pyproject.toml and uv.lock.
- Preserve source provenance and distinguish geometric validation from dynamic tracking.
- Keep Kimodo candidates labeled experimental; do not add them to mimic training selections by default.
- Use named joints/bodies and the documented coordinate conventions. Terrain box dimensions are full sizes.
- Keep generated reference NPZs paired with their actual videos. Do not pair an older conversion's video with new motion arrays.
- After data changes: `uv run python tools/build_catalog.py` and `uv run python tools/validate_library.py`.
- After code changes: `uv run ruff check tools data_generator integrations/isaaclab/play_motion_file.py`.
- The gallery must work via a static server, direct file opening, and GitHub Pages. Keep its labels concise and explain technical terms.
