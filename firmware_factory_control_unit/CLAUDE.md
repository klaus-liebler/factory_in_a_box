# Project rules – Factory Control Unit firmware

The firmware runs on rtoos (submodule `rtoos/`); its rules (`rtoos/CLAUDE.md`) apply here too.

## Regression tests (binding)
- A fix that is not obvious, because the bug came from a combination of individual causes, always gets an automated regression test. The test must fail without the fix (check this once) and pass with it.
- Firmware and driver bugs: a test in rtoos (host test or emulator self-test). Web UI bugs: a test in `web/` (`npm test`).
