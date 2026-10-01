# What can be deleted

No files have been deleted by this implementation. Updated working files remain where they were. `sources/` contains synced read-only references: do not edit, move, or delete those files from this mirror.

## Safe to remove now

These are generated caches or document previews; deleting them does not affect robot operation:

- `__pycache__/` and every `tests/**/__pycache__/`.
- `tmp/docx_render/`, `tmp/docx_render_check2/`, `tmp/docx_render_final/`, `tmp/docx_render_final2/`.
- `tmp/pdfs/rendered/` and loose preview/crop images under `tmp/pdfs/`.
- `tmp/pdfs/fontcache/` and `tmp/pdfs/fonts.conf`.
- `tmp/architecture_requirements.txt` and `tmp/source_reference_hashes.json` after reviewing the implementation report.

Document builder scripts in `tmp/pdfs/` can be removed if you do not need to regenerate the earlier PDFs/Word document. They are not robot software dependencies.

## Optional archives

- `output/docx/Robot_Architecture_Plan.docx` is the earlier Word copy; keep it only if you want an editable planning document.
- `output/pdf/XR4_to_Pi4_Engineering_Review.pdf` is a historical review; keep it if useful for wiring provenance.
- `tmp/pdfs/xr4_manual.pdf` is a downloaded reference copy; it is not needed to run the robot.
- Retain `tmp/pdfs/hat_schematic.pdf` until the independent cutoff topology is verified and a schematic is archived with the physical hardware record.
- Keep `output/pdf/Robot_Architecture_Plan.pdf`, the baseline archive, release manifest, source archive, and Git history bundle as design/release records. These can live off the Pi; the Pi does not need them at runtime.

## Do not delete yet

- `motor_setup.py` and `ddsm115.py`: required to discover/assign/check IDs while the HAT has factory firmware. They remain useful after retiring the legacy drive.
- `config.legacy.example.json`: keep until fast-path commissioning is accepted, or retain in the baseline archive if choosing to retire the factory-firmware drive later.
- Existing/new runtime `.py` modules, `requirements.txt`, the complete `hat_firmware/robot_hat/` folder, protocol schema/generation files, and tests: these are the maintained implementation and release checks.
- `deploy/`, `tools/`, `commissioning/`, setup/control/protocol/stop documents: these reproduce installation, recovery, releases, and acceptance.
- Actual `config.json`, session logs, metadata/events, and commissioning records needed to reproduce measured operating limits.

Retiring legacy means removing its driving entry point only after hardware acceptance. Removing `ddsm115.py` wholesale would break the retained motor-ID setup utility.
