using System.Drawing;
using System.Globalization;
using System.IO.Compression;
using System.Runtime.Versioning;
using System.Text;

namespace Builder.Labels;

// A minimal PDF writer: pages with grey images (8 bit, Flate) and vector content, Helvetica
// (standard font, WinAnsi, not embedded). Enough for labels at true scale - no dependency.
public sealed class SimplePdf
{
    private readonly List<(byte[] Data, int W, int H)> _images = [];
    private readonly List<(string Content, double W, double H)> _pages = [];

    public const double PtPerMm = 72 / 25.4;

    public int AddImage(byte[] gray, int width, int height)
    {
        using var ms = new MemoryStream();
        using (var z = new ZLibStream(ms, CompressionLevel.Optimal, leaveOpen: true)) z.Write(gray);
        _images.Add((ms.ToArray(), width, height));
        return _images.Count - 1;
    }

    public void AddPage(string content, double widthPt, double heightPt) => _pages.Add((content, widthPt, heightPt));

    public int PageCount => _pages.Count;

    public static string N(double v) => v.ToString("0.###", CultureInfo.InvariantCulture);

    /// Image i placed at x, y (lower left, pt) with size w x h (pt).
    public static string Image(int i, double x, double y, double w, double h) => $"q {N(w)} 0 0 {N(h)} {N(x)} {N(y)} cm /Im{i} Do Q\n";

    /// Text at x, y (baseline, pt) in Helvetica of size pt.
    public static string Text(string s, double x, double y, double size, bool bold = false)
    {
        var sb = new StringBuilder();
        foreach (var c in s)
        {
            if (c is '(' or ')' or '\\') sb.Append('\\').Append(c);
            else if (c < 128) sb.Append(c);
            else if (c < 256) sb.Append('\\').Append(Convert.ToString(c, 8));  // Latin-1 = WinAnsi here
            else sb.Append('?');
        }
        return $"BT /{(bold ? "F2" : "F1")} {N(size)} Tf {N(x)} {N(y)} Td ({sb}) Tj ET\n";
    }

    public void Save(string path)
    {
        var imageBase = 5;
        var pageBase = imageBase + _images.Count;
        using var f = new FileStream(path, FileMode.Create);
        var offsets = new List<long>();
        void W(string s) => f.Write(Encoding.ASCII.GetBytes(s));
        void Obj(string head, byte[]? stream = null)
        {
            offsets.Add(f.Position);
            W($"{offsets.Count} 0 obj\n{head}\n");
            if (stream is not null)
            {
                W("stream\n");
                f.Write(stream);
                W("\nendstream\n");
            }
            W("endobj\n");
        }
        W("%PDF-1.4\n");
        f.Write([0x25, 0xE2, 0xE3, 0xCF, 0xD3, 0x0A]);  // binary marker
        var kids = string.Join(" ", Enumerable.Range(0, _pages.Count).Select(i => $"{pageBase + 2 * i} 0 R"));
        Obj("<< /Type /Catalog /Pages 2 0 R >>");
        Obj($"<< /Type /Pages /Kids [{kids}] /Count {_pages.Count} >>");
        Obj("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>");
        Obj("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>");
        foreach (var (data, w, h) in _images)
        {
            Obj($"<< /Type /XObject /Subtype /Image /Width {w} /Height {h} /ColorSpace /DeviceGray /BitsPerComponent 8 " +
                $"/Filter /FlateDecode /Length {data.Length} >>", data);
        }
        var xobjects = string.Join(" ", Enumerable.Range(0, _images.Count).Select(i => $"/Im{i} {imageBase + i} 0 R"));
        for (var i = 0; i < _pages.Count; ++i)
        {
            var (content, w, h) = _pages[i];
            var bytes = Encoding.ASCII.GetBytes(content);
            Obj($"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {N(w)} {N(h)}] " +
                $"/Resources << /Font << /F1 3 0 R /F2 4 0 R >> /XObject << {xobjects} >> >> /Contents {pageBase + 2 * i + 1} 0 R >>");
            Obj($"<< /Length {bytes.Length} >>", bytes);
        }
        var xref = f.Position;
        W($"xref\n0 {offsets.Count + 1}\n0000000000 65535 f \n");
        foreach (var off in offsets) W($"{off:D10} 00000 n \n");
        W($"trailer\n<< /Size {offsets.Count + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n");
    }
}

// Labels for several boards on A4 at true scale (cells of LabelPrinter:SheetCellWidthMm x
// SheetCellHeightMm, default 62 x 40 mm: 3 x 6 per page), with cut marks outside the grid and thin
// dashed cell edges, for any laser printer: print at 100 % ("Actual size"), check the 50 mm bar, cut.
[SupportedOSPlatform("windows")]
public static class LabelSheet
{
    public const double A4W = 210, A4H = 297;
    private const double Margin = 10, Header = 20, Footer = 10;
    private const int Dpi = 300;

    public static (int Columns, int Rows) Grid(double cellW, double cellH) =>
        ((int)Math.Floor((A4W - 24) / cellW), (int)Math.Floor((A4H - 2 * Margin - Header - Footer) / cellH));

    /// Returns the number of pages.
    public static int Write(string path, IReadOnlyList<WebCredentials.Device> devices, double cellW = 62, double cellH = 40, int copies = 1)
    {
        var (cols, rows) = Grid(cellW, cellH);
        if (cols < 1 || rows < 1)
        {
            throw new ArgumentException($"Zelle {cellW} x {cellH} mm passt nicht auf A4.");
        }
        var cells = devices.SelectMany(d => Enumerable.Repeat(d, Math.Max(1, copies))).ToList();
        var perPage = cols * rows;
        var pages = Math.Max(1, (cells.Count + perPage - 1) / perPage);
        var k = SimplePdf.PtPerMm;
        var gridW = cols * cellW;
        var x0 = (A4W - gridW) / 2;
        var pxW = (int)Math.Round(cellW / 25.4 * Dpi);
        var pxH = (int)Math.Round(cellH / 25.4 * Dpi);

        var pdf = new SimplePdf();
        var imageOf = new Dictionary<WebCredentials.Device, int>();
        foreach (var d in devices)
        {
            using var bmp = LabelArt.Render(d, pxW, pxH);
            imageOf[d] = pdf.AddImage(LabelArt.Gray(bmp), pxW, pxH);
        }

        for (var p = 0; p < pages; ++p)
        {
            var onPage = cells.Skip(p * perPage).Take(perPage).ToList();
            var usedRows = Math.Max(1, (onPage.Count + cols - 1) / cols);
            var usedCols = Math.Max(1, Math.Min(cols, onPage.Count));
            var top = Margin + Header;  // mm from the top of the sheet
            double Y(double mmFromTop) => (A4H - mmFromTop) * k;
            var c = new StringBuilder();

            c.Append(SimplePdf.Text("Factory Control Unit - Etiketten mit Web-UI-Einmalpasswort (vertraulich: Reste vernichten)",
                                    Margin * k, Y(Margin + 4), 10, bold: true));
            c.Append(SimplePdf.Text($"Mit 100 % / \"Tatsächliche Größe\" drucken, Balken unten muss 50 mm messen. " +
                                    $"Zelle {SimplePdf.N(cellW)} x {SimplePdf.N(cellH)} mm. Seite {p + 1}/{pages}.",
                                    Margin * k, Y(Margin + 9), 7.5));
            // 50 mm check bar at the bottom
            c.Append(CultureInfo.InvariantCulture, $"0 g {SimplePdf.N(Margin * k)} {SimplePdf.N(Y(A4H - Margin))} {SimplePdf.N(50 * k)} {SimplePdf.N(1.2 * k)} re f\n");
            c.Append(SimplePdf.Text("Kontrollmaß: dieser Balken = 50 mm", (Margin + 53) * k, Y(A4H - Margin - 0.2), 7));

            for (var i = 0; i < onPage.Count; ++i)
            {
                var col = i % cols;
                var row = i / cols;
                var x = x0 + col * cellW;
                var yTop = top + row * cellH;
                c.Append(SimplePdf.Image(imageOf[onPage[i]], x * k, Y(yTop + cellH), cellW * k, cellH * k));
            }

            // dashed cell edges (light grey), only around used cells
            c.Append("q 0.6 G 0.25 w [2 2] 0 d\n");
            for (var i = 0; i < onPage.Count; ++i)
            {
                var x = x0 + (i % cols) * cellW;
                var yTop = top + (i / cols) * cellH;
                c.Append(CultureInfo.InvariantCulture, $"{SimplePdf.N(x * k)} {SimplePdf.N(Y(yTop + cellH))} {SimplePdf.N(cellW * k)} {SimplePdf.N(cellH * k)} re S\n");
            }
            c.Append("Q\n");

            // cut marks: every grid line extended outside the used grid (2 mm gap, 6 mm long)
            var gridH = usedRows * cellH;
            c.Append("q 0 G 0.4 w\n");
            void L(double ax, double ay, double bx, double by) =>
                c.Append(CultureInfo.InvariantCulture, $"{SimplePdf.N(ax * k)} {SimplePdf.N(Y(ay))} m {SimplePdf.N(bx * k)} {SimplePdf.N(Y(by))} l S\n");
            for (var ci = 0; ci <= usedCols; ++ci)
            {
                var x = x0 + ci * cellW;
                L(x, top - 2, x, top - 6);
                L(x, top + gridH + 2, x, top + gridH + 6);
            }
            for (var ri = 0; ri <= usedRows; ++ri)
            {
                var y = top + ri * cellH;
                L(x0 - 2, y, x0 - 6, y);
                L(x0 + usedCols * cellW + 2, y, x0 + usedCols * cellW + 6, y);
            }
            c.Append("Q\n");
            pdf.AddPage(c.ToString(), A4W * k, A4H * k);
        }
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        pdf.Save(path);
        return pages;
    }

    /// One label as a one-page PDF of its own size (the preview).
    public static void WriteSingle(string path, Bitmap bmp, double widthMm, double heightMm)
    {
        var pdf = new SimplePdf();
        var k = SimplePdf.PtPerMm;
        var i = pdf.AddImage(LabelArt.Gray(bmp), bmp.Width, bmp.Height);
        pdf.AddPage(SimplePdf.Image(i, 0, 0, widthMm * k, heightMm * k), widthMm * k, heightMm * k);
        pdf.Save(path);
    }
}
