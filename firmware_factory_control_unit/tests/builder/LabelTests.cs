using System.Diagnostics;
using System.Drawing;
using System.Text;
using System.Text.Json;
using Builder;
using Builder.Labels;

namespace Builder.Tests;

// Label output without a printer: sizes, the Brother QL raster job (structure, and byte for byte
// against python brother_ql when BROTHER_QL_PYTHON names a python that has it), the dry run of the
// print step, the A4 sheet and reading the board archive. Test data only, no real password.
public sealed class LabelTests : IDisposable
{
    private readonly string _dir = Directory.CreateTempSubdirectory("fcu-label-test-").FullName;

    private static readonly WebCredentials.Device Board = new(
        "303537_003400343035510834303537", "003400343035510834303537", "factory-box-303537", "02:04:31:64:0B:01",
        "rev1 (1.0.0)", "Test-Pass-Word-Xyz2", DateTimeOffset.FromUnixTimeSeconds(1791125942));

    public void Dispose() => Directory.Delete(_dir, recursive: true);

    [Theory]
    [InlineData("62", 696, null)]
    [InlineData("29x90", 991, 306)]
    [InlineData("17x54", 566, 165)]
    public void Image_has_the_printable_size_of_the_label(string label, int width, int? height)
    {
        var media = BrotherQl.FindMedia(label);
        var (w, h) = BrotherQl.ImageSize(media);
        Assert.Equal(width, w);
        Assert.Equal(height, h);
        using var bmp = LabelArt.Render(Board, w, h, monochrome: true);
        Assert.Equal(width, bmp.Width);
        if (height is { } hh)
        {
            Assert.Equal(hh, bmp.Height);
        }
        else
        {
            // 62 mm endless: auto length, about 35 mm, within the QL-700's limits
            Assert.InRange(bmp.Height, 300, 600);
        }
        // black and white only
        Assert.All(LabelArt.Gray(bmp), v => Assert.True(v is 0 or 255));
    }

    [Theory]
    [InlineData("62")]
    [InlineData("29x90")]
    [InlineData("17x54")]
    public void Raster_job_has_one_line_per_dot_row_and_the_right_header(string label)
    {
        var model = BrotherQl.FindModel("QL-700");
        var media = BrotherQl.FindMedia(label);
        var (w, h) = BrotherQl.ImageSize(media);
        using var bmp = LabelArt.Render(Board, w, h, monochrome: true);
        var job = BrotherQl.Encode(LabelArt.Gray(bmp), bmp.Width, bmp.Height, model, media);
        var lines = media.DieCut ? media.PrintableLength : bmp.Height;

        // invalidate, ESC @, ESC i S, ESC i z ...
        Assert.All(job[..200], b => Assert.Equal(0, b));
        Assert.Equal(new byte[] { 0x1B, 0x40, 0x1B, 0x69, 0x53, 0x1B, 0x69, 0x7A, 0xCE }, job[200..209]);
        Assert.Equal(media.DieCut ? 0x0B : 0x0A, job[209]);
        Assert.Equal(media.WidthMm, job[210]);
        Assert.Equal(media.LengthMm, job[211]);
        Assert.Equal(lines, BitConverter.ToInt32(job, 212));
        var header = 200 + 2 + 3 + 3 + 4 + 4 + 2 + 4 + 4 + 4 + 5;  // ... ESC i z n1-n10, ESC i M, ESC i A, ESC i K, ESC i d
        Assert.Equal(header + lines * (3 + 90) + 1, job.Length);
        Assert.Equal(new byte[] { 0x67, 0x00, 90 }, job[header..(header + 3)]);
        Assert.Equal(0x1A, job[^1]);
        // something is printed
        Assert.Contains(job[header..^1].Where((_, i) => i % 93 >= 3), b => b != 0);
    }

    [Fact]
    public void Raster_is_mirrored_and_right_aligned_with_the_label_margin()
    {
        // a single black dot in the top left corner of a 62 mm label image
        var media = BrotherQl.FindMedia("62");
        var gray = Enumerable.Repeat((byte)255, 696 * 150).ToArray();
        gray[0] = 0;
        var job = BrotherQl.Encode(gray, 696, 150, BrotherQl.FindModel("QL-700"), media);
        var header = job.Length - 1 - 150 * 93;
        var row = job[(header + 3)..(header + 93)];
        // padded column 720 - 696 - 12 = 12, mirrored: dot 707 = byte 88, bit 0x10
        Assert.Equal(0x10, row[88]);
        Assert.Equal(1, row.Count(b => b != 0));
    }

    [Fact]
    public void Dry_run_writes_png_and_raster_job_to_the_board_archive()
    {
        var boards = Path.Combine(_dir, "boards");
        var o = new LabelPrinterOptions { Type = "BrotherQL", BrotherQL = { Backend = "WindowsRaw", Target = "no such printer" } };
        var basePath = DeviceLabel.Write(boards, Board, o, dryRun: true);  // would throw if it tried to print
        var png = basePath + ".QL-700-62.png";
        var bin = basePath + ".QL-700-62.bin";
        Assert.Equal(Path.Combine(boards, Board.BoardId, "labels", Board.Hostname), basePath);
        Assert.Equal(Path.Combine(boards, Board.BoardId, "labels"), BoardArchiveLabels.LabelsDir(boards, Board.BoardId));
        Assert.Equal(Path.Combine(boards, "labels_A4.pdf"), BoardArchiveLabels.SheetPath(boards));
        using (var bmp = new Bitmap(png))
        {
            Assert.Equal(696, bmp.Width);
        }
        Assert.True(new FileInfo(bin).Length > 200);
    }

    [Fact]
    public void File_backend_writes_the_job_to_the_target()
    {
        var target = Path.Combine(_dir, "job.bin");
        var o = new LabelPrinterOptions { Type = "BrotherQL", BrotherQL = { Backend = "File", Target = target, Label = "29x90" } };
        var basePath = DeviceLabel.Write(Path.Combine(_dir, "boards"), Board, o, dryRun: false);
        Assert.Equal(File.ReadAllBytes(basePath + ".QL-700-29x90.bin"), File.ReadAllBytes(target));
    }

    [Fact]
    public void A4_sheet_has_18_labels_per_page_and_true_scale()
    {
        Assert.Equal((3, 6), LabelSheet.Grid(62, 40));
        var path = Path.Combine(_dir, "labels_A4.pdf");
        var pages = LabelSheet.Write(path, [Board, Board with { Hostname = "factory-box-363836", BoardId = "363836_x" }], copies: 10);
        Assert.Equal(2, pages);  // 20 labels
        var pdf = Encoding.Latin1.GetString(File.ReadAllBytes(path));
        Assert.StartsWith("%PDF-1.4", pdf);
        Assert.Contains("/Count 2", pdf);
        Assert.Contains("/MediaBox [0 0 595.276 841.89]", pdf);
        // 62 x 40 mm cells: 175.748 x 113.386 pt, one image per board (not per copy)
        Assert.Contains("175.748 0 0 113.386", pdf);
        Assert.Equal(2, CountOf(pdf, "/Subtype /Image"));
        Assert.Equal(20, CountOf(pdf, " Do Q"));
        Assert.EndsWith("%%EOF\n", pdf);
    }

    [Fact]
    public void Board_archive_gives_the_label_data_and_never_invents_a_password()
    {
        var boards = Path.Combine(_dir, "boards");
        var a = Path.Combine(boards, "303537_003400343035510834303537");
        Directory.CreateDirectory(Path.Combine(a, "generated"));
        File.WriteAllText(Path.Combine(a, "board_info.json"), JsonSerializer.Serialize(new
        {
            firstConnectedAtEpoch = 1791125942, webAdminPassword = "Test-Pass-Word-Xyz2", chipUid = "003400343035510834303537",
            boardTypeName = "FactoryControlUnit", boardTypeVersion = (string?)null,
        }));
        File.WriteAllText(Path.Combine(a, "generated", "hardware-identity.json"), JsonSerializer.Serialize(new
        {
            hostname = "factory-box-303537", chipUid = "003400343035510834303537", ethMacBytes = "AgQxZAsB",
        }));
        var b = Path.Combine(boards, "363836_004500173434510434363836");
        Directory.CreateDirectory(b);
        var noPassword = """{"firstConnectedAtEpoch":1785513294,"chipUid":"004500173434510434363836"}""";
        File.WriteAllText(Path.Combine(b, "board_info.json"), noPassword);
        // label files live in the archive too and must not look like boards
        Directory.CreateDirectory(Path.Combine(a, "labels"));
        File.WriteAllText(BoardArchiveLabels.SheetPath(boards), "%PDF-1.4");

        Assert.Equal(2, BoardArchiveLabels.AllBoardIds(boards).Count);
        Assert.Equal("303537_003400343035510834303537", BoardArchiveLabels.Resolve(boards, "303537"));
        Assert.Equal("303537_003400343035510834303537", BoardArchiveLabels.Resolve(boards, "factory-box-303537"));
        Assert.Throws<ArgumentException>(() => BoardArchiveLabels.Resolve(boards, "999"));

        var d = BoardArchiveLabels.TryLoad(boards, "303537_003400343035510834303537", "FactoryControlUnit", "1.0.0", out _);
        Assert.NotNull(d);
        Assert.Equal(Board, d);

        Assert.Null(BoardArchiveLabels.TryLoad(boards, "363836_004500173434510434363836", "FactoryControlUnit", "1.0.0", out var why));
        Assert.Contains("kein Web-Passwort", why);
        Assert.Equal(noPassword, File.ReadAllText(Path.Combine(b, "board_info.json")));  // untouched
    }

    // Byte for byte as python brother_ql (inventree fork) converts the same PNG. Runs when
    // BROTHER_QL_PYTHON is a python with brother_ql installed (docs/labels.md), otherwise passes
    // without checking.
    [Theory]
    [InlineData("62")]
    [InlineData("29x90")]
    [InlineData("17x54")]
    public void Raster_job_equals_python_brother_ql(string label)
    {
        var python = Environment.GetEnvironmentVariable("BROTHER_QL_PYTHON");
        if (string.IsNullOrEmpty(python))
        {
            return;
        }
        var o = new LabelPrinterOptions { Type = "BrotherQL", BrotherQL = { Label = label } };
        var basePath = DeviceLabel.Write(_dir, Board, o, dryRun: true);
        var png = $"{basePath}.QL-700-{label}.png";
        var ours = File.ReadAllBytes($"{basePath}.QL-700-{label}.bin");
        var theirs = Path.Combine(_dir, "brother_ql.bin");
        var psi = new ProcessStartInfo(python) { UseShellExecute = false, RedirectStandardError = true };
        foreach (var a in new[] { "-c",
            "import sys; from brother_ql.raster import BrotherQLRaster; from brother_ql.conversion import convert; " +
            "open(sys.argv[3], 'wb').write(convert(BrotherQLRaster('QL-700'), [sys.argv[1]], sys.argv[2], cut=True))",
            png, label, theirs })
        {
            psi.ArgumentList.Add(a);
        }
        using var p = Process.Start(psi)!;
        var err = p.StandardError.ReadToEnd();
        p.WaitForExit();
        Assert.True(p.ExitCode == 0, err);
        Assert.Equal(File.ReadAllBytes(theirs), ours);
    }

    private static int CountOf(string s, string what)
    {
        var n = 0;
        for (var i = s.IndexOf(what, StringComparison.Ordinal); i >= 0; i = s.IndexOf(what, i + 1, StringComparison.Ordinal)) ++n;
        return n;
    }
}
