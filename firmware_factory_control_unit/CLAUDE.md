# Project rules – Factory Control Unit firmware

The firmware runs on rtoos (submodule `rtoos/`); its rules (`rtoos/CLAUDE.md`) apply here too.

## Regression tests (binding)
- A fix that is not obvious, because the bug came from a combination of individual causes, always gets an automated regression test. The test must fail without the fix (check this once) and pass with it.
- Firmware and driver bugs: a test in rtoos (host test or emulator self-test). Bugs in the application code (`src/`): a host test in `tests/host` (simulated hardware such as the SD card, see its README). Web UI bugs: a test in the npm workspace that has the bug - `web/`, `web_sim/` or `web_packages/plc` (`npm test` in `firmware_factory_control_unit/` runs them all); for the shell, a test in `npm-packages/@klaus-liebler/ui-shell` (shared with sensact and labathome, `npm test` there).
