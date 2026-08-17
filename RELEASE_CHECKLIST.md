# v1.0.0 release checklist

- [ ] Ensure the intended v1.0.0 files are committed and the working tree is clean.
- [ ] Run `python -m pip install -e ".[dev]"`.
- [ ] Run `pytest --cov=cbf_safe_steering --cov-report=term-missing --cov-fail-under=90`.
- [ ] Run `ruff check --fix src tests scripts demo.py`, review the changes, and commit them.
- [ ] Run `black src tests scripts demo.py`, review the changes, and commit them.
- [ ] Push to `main` and confirm the CI workflow is green on every supported Python version (3.10–3.14).
- [ ] Create Git tag `v1.0.0` from the validated commit.
- [ ] Create a GitHub Release from `v1.0.0`, using `CHANGELOG.md` as the basis for release notes.
- [ ] Connect the GitHub repository to Zenodo and archive the v1.0.0 release.
- [ ] After Zenodo mints a DOI, add the DOI to `CITATION.cff` and the README citation section.
