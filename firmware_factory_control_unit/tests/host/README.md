# Host tests of the application code

GoogleTest tests of the Factory Control Unit's own code (`src/`), on the PC with GCC 16. rtoos is
built as a subproject for its host libraries; hardware is replaced by simulations in `sim/`:

| Simulation | Replaces | What it is |
|---|---|---|
| `sim/sd_card_sim.cpp` | `src/sd_card.cpp` (SDMMC + `rtos::storage::SdCard`) | the SD card as a FAT-formatted FileX RAM disk; the test inserts, removes and swaps cards (`fcu::sd::sim`) |

`stubs/` replaces headers the code under test includes but does not use on the host
(`board/fcu.hpp`, `system.hpp`).

Run (from this directory, PowerShell or cmd):

```
cmake --preset host
cmake --build --preset host
ctest --preset host
```

Needs the generated `../../generated/ws_protocol.hh` (`dotnet run --project builder --
GenerateBestBinaryBufferFiles` in `firmware_factory_control_unit/`).

New tests: one `*_test.cpp` per unit, listed in `CMakeLists.txt` together with the `src/` files
under test.
