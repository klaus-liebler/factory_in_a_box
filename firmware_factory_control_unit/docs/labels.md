# Device labels: Brother QL-700, Zebra, A4 sheet

Every provisioned board gets a label with hostname, board ID (chip UID), MAC, hardware revision,
provisioning date, the one-time web UI password and a QR code with `https://<hostname>.local/` (the URL
only, never the password); see [security.md](security.md) for the password itself. The builder writes all
label files to the board archive, `<BoardsDir>/<board id>/labels/` (`BoardStorage:BoardsDir`, next to the
board's `board_info.json`; never into the repository: they carry the password) and prints them as `LabelPrinter` in
`builder/appsettings.json` says. The password is read from the board archive (`board_info.json`,
`webAdminPassword`) and never created by the label steps.
Older builder versions wrote to `build/labels/` in the repository; move such files into the board's
`labels/` folder in the archive (`build/labels/` is no longer used).

| Step | What |
|---|---|
| `GenerateWebCredentials` (every pipeline) | prints the label when it creates a new password |
| `PrintLabel` | the label again: current board, `--board <id or prefix>` (repeatable) or `--all` |
| `PrintLabelSheet` | `<BoardsDir>/labels_A4.pdf`: labels on A4 at true scale with cut marks (same board selection, `--copies <n>`, `--out <file>`) |

Options for one run, without touching `appsettings.json`: `--dry-run` (write the print job, send
nothing), `--label-type Zebra|BrotherQL|None`, `--ql-label 62|29x90|17x54`,
`--ql-backend WindowsRaw|WindowsDriver|Tcp|BrotherQlCli|File`, `--ql-target <queue|host:port|identifier>`.

```
dotnet run --project builder -- PrintLabel -- --dry-run                         # current board, files only
dotnet run --project builder -- PrintLabel -- --board 303537                    # one board from the archive
dotnet run --project builder -- PrintLabel -- --all                             # every board with a password
dotnet run --project builder -- PrintLabelSheet -- --all                        # A4 sheet of all boards
dotnet run --project builder -- PrintLabelSheet -- --board 303537 --copies 3    # one board, three times
dotnet run --project builder -- PrintLabelSheet                                 # just the current board
```

## Settings (`LabelPrinter`)

```json
"LabelPrinter": {
  "Type": "BrotherQL",            // Zebra (default) | BrotherQL | None
  "DryRun": false,                // true: never send, only write <BoardsDir>/<board id>/labels/
  "A4Sheet": false,               // true: every printed label also as <hostname>.A4.pdf
  "SheetCellWidthMm": 62,         // A4 sheet cells (3 x 6 per page at 62 x 40 mm)
  "SheetCellHeightMm": 40,
  "BrotherQL": {
    "Model": "QL-700",            // QL-700, QL-710W, QL-720NW, QL-800, QL-810W, QL-820NWB
    "Label": "62",                // 62 (DK-22205 endless), 29x90 (DK-11201), 17x54 (DK-11204)
    "Backend": "WindowsRaw",      // WindowsRaw | WindowsDriver | Tcp | BrotherQlCli | File
    "Target": "Brother QL-700",   // queue name, host[:port], brother_ql identifier or file
    "BrotherQlCommand": "brother_ql",
    "BrotherQlBackend": null      // pyusb | network | linux_kernel (BrotherQlCli only)
  },
  "Host": null, "WindowsPrinter": null, "Port": 9100,      // Zebra
  "WidthMm": 51, "HeightMm": 25, "Dpi": 300               // Zebra
}
```

## Brother QL-700

The QL-700 speaks no ZPL; it takes Brother's raster protocol (300 dpi, a 720-dot head). The builder
renders the label at exactly the printable dots of the tape and encodes the raster job itself
(`builder/Labels/BrotherQl.cs`), byte for byte as the python package `brother_ql` does
(`tests/builder` checks that). Per label it writes

- `<BoardsDir>/<board id>/labels/<hostname>.QL-700-62.png`: the label, black/white, 696 dots wide (62 mm tape: 696
  printable dots, 12 dots right margin), length as the content needs (about 35 mm), cut after the label;
- `<BoardsDir>/<board id>/labels/<hostname>.QL-700-62.bin`: the raster job that goes to the printer.

`29x90` (DK-11201, 991 x 306 dots) and `17x54` (DK-11204, 566 x 165 dots) are die-cut and use a compact
layout: QR code left, hostname/ID and password right (17x54: ID and password only).

### Backends

- **WindowsRaw** (recommended): the raster job through the Windows queue of Brother's driver with
  datatype RAW. The spooler passes the bytes unchanged to the printer's USB port; the driver renders
  nothing. No libusb, no Zadig, and the printer keeps working from every other Windows program.
- **WindowsDriver**: the PNG through the Brother driver (System.Drawing). The driver does the raster;
  needs the matching paper size in the driver (62 mm endless with the label's length). Fallback if a
  driver refuses RAW.
- **Tcp**: the raster job raw to `host:port` (9100): network QLs (QL-710W/720NW/820NWB) or a print
  server in front of a USB QL-700 (lab server, see below).
- **BrotherQlCli**: the PNG through `brother_ql` (`pip install brother_ql-inventree`):
  `brother_ql -b pyusb -m QL-700 -p usb://0x04f9:0x2042 print -l 62 <png>`.
- **File**: the raster job written to `Target` (and always to `<BoardsDir>/<board id>/labels/`).

### Setting up the QL-700 on Windows

1. Install Brother's QL-700 printer driver (support.brother.com, QL-700, Windows, "Printer Driver"). The
   queue is called `Brother QL-700`; check with
   `Get-Printer | Where-Object Name -like "*QL*"` and put the exact name into `BrotherQL:Target`.
2. **Editor Lite off.** The QL-700 has an *Editor Lite* button with a green LED (as the QL-800 series).
   With the LED on, the printer shows up as a USB drive (P-touch Editor Lite) instead of a printer and
   takes no print jobs. Hold the Editor Lite button until the green LED goes out; the setting is kept.
3. Load DK-22205 (62 mm endless). The printer measures the roll itself; the job says 62 mm endless.
4. `builder/appsettings.json`: `"Type": "BrotherQL"` and `BrotherQL` as above (`Backend` `WindowsRaw`).
5. First a dry run, look at the PNG, then print:
   `dotnet run --project builder -- PrintLabel -- --dry-run` and
   `dotnet run --project builder -- PrintLabel`.

If a RAW job is accepted but nothing prints: printer properties -> Advanced -> Print Processor: `winprint`,
default data type `RAW`; or use `WindowsDriver`.

### brother_ql / pyusb on Windows (only for BrotherQlCli)

pyusb needs a libusb driver on the printer: replace the driver of `QL-700` (USB ID 04F9:2042) with
WinUSB or libusb-win32 using [Zadig](https://zadig.akeo.ie/) and put `libusb-1.0.dll` on the PATH. From
then on the Brother Windows driver no longer sees the printer (undo: Device Manager -> uninstall the
device including its driver, replug). That is why WindowsRaw is the default on Windows.

### Lab server (Linux)

A QL-700 on a Linux server: `brother_ql -b linux_kernel -m QL-700 -p file:///dev/usb/lp0 print -l 62 <png>`
(the PNG from `<BoardsDir>/<board id>/labels/`), or a raw print server on port 9100 (e.g. `p910nd` on `/dev/usb/lp0`) and
`"Backend": "Tcp", "Target": "<server>:9100"` in the builder. Network QLs take the job on 9100 directly.

## A4 sheet for any printer

`PrintLabelSheet` lays the labels out on A4 at true scale: cells of 62 x 40 mm (3 x 6 per page,
`SheetCellWidthMm/HeightMm`), dashed cell edges and cut marks outside the grid, a 50 mm check bar at the
bottom. Print with 100 % / "Actual size" (not "Fit to page"), check the bar with a ruler, cut along the
marks and tape the label on. Boards without a password in the archive are skipped with `--all`, an error
with `--board`. The PDF contains the passwords: shred misprints.

## Zebra (ZPL)

`Type` `Zebra` (or empty): ZPL II, 51 x 25 mm at 300 dpi by default (`WidthMm/HeightMm/Dpi`), raw to
`Host:Port` (9100) or, with `WindowsPrinter`, the preview PNG through that Windows driver. Files:
`<hostname>.zpl/.png/.pdf`.

## Tests

`dotnet test tests/builder`: label sizes, raster job structure and mirroring, dry run and `File` backend,
A4 sheet (pages, true scale), reading the board archive (never inventing a password). With
`BROTHER_QL_PYTHON` pointing to a python that has `brother_ql-inventree`, the raster job is also compared
byte for byte with brother_ql's for all three labels.
