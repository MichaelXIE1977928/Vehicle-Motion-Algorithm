# Contributing

Thank you for your interest in VMA.

## Getting started

1. Fork or clone the repository.
2. Create a virtual environment (recommended), for example:
   - `python -m venv .venv`
   - Activate it (platform-specific).
3. Install in editable mode with test dependencies:

   ```bash
   pip install -e ".[dev]"
   ```

4. Run the test suite:

   ```bash
   pytest
   ```

5. For UI work, run Streamlit from the repo root:

   ```bash
   streamlit run src/vma/gui/app.py
   ```

## Pull requests

- Keep changes focused on a single concern when possible.
- Add or update tests when you change behavior that is covered or should be covered by `tests/`.
- Ensure `pytest` passes locally before opening a PR.

## Code of conduct

Be respectful and constructive in issues and pull requests. For a formal policy, the maintainer may add a `CODE_OF_CONDUCT.md` later; until then, follow common open-source norms.

## License

By contributing, you agree that your contributions will be licensed under the same terms as the project ([MIT](LICENSE)), unless you clearly state otherwise in writing.
