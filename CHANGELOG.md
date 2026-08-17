# Changelog

All notable changes to this project are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-08-16

### Added

- GitHub Actions CI across Python 3.10–3.13.
- Coverage reporting with a 90% minimum CI threshold for the numerical/control core.
- Ruff and Black development configuration.
- `CITATION.cff` for research-software citation metadata.
- `ROADMAP.md` with the proposed v2 joint braking + steering HOCBF-QP formulation.
- A root-level `demo.py` compatibility entry point matching the README commands.
- Expanded deterministic tests for QP projection, conflicting bounds, tied active constraints, HOCBF identities, all checked-in scenarios, and timestep sensitivity.
- Explicit `limiting_obstacles` and `active_obstacles` diagnostics in `QPResult`.
- Backward-compatible `binding_obstacles` property for existing callers.

### Changed

- Clarified README wording around QP infeasibility and the diagnostic fallback command.
- Improved repository metadata, development dependencies, and project URLs in `pyproject.toml`.
- Updated the README repository tree and quality/reproducibility instructions.

### Fixed

- Removed ambiguity in the old `binding_obstacles` terminology: only constraints whose residual is approximately zero at the returned optimum are now reported as active.
- Ensured the documented `python demo.py --scenario ...` command has a corresponding checked-in entry point.
