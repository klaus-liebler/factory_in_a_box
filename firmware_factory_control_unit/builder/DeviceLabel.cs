using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Drawing.Text;
using System.Globalization;
using System.Runtime.Versioning;
using System.Text;
using Builder.Labels;
using QRCoder;

namespace Builder;

// The device label (docs/security.md "Provisioning"): hostname, board ID (chip UID), MAC, hardware
// revision, the one-time web UI password, the provisioning date and a QR code with
// https://<hostname>.local/ (the URL only, never the password).
//
//   <hostname>.zpl   ZPL II for a Zebra-type label printer (LabelPrinter:WidthMm/HeightMm/Dpi,
//                    default 51 x 25 mm at 300 dpi); sent raw to LabelPrinter:Host:Port (9100) if a
//                    host is configured, otherwise only written
//   <hostname>.png   preview of the same layout (QR code from QRCoder, the printer draws its own)
//   <hostname>.pdf   the preview as a one-page PDF of the label's size
//
// LabelPrinter:Type BrotherQL instead (docs/labels.md): <hostname>.<model>-<label>.png/.bin, the
// layout of Labels/LabelArt.cs at the label's exact printable dots and its Brother QL raster job.
// LabelPrinter:A4Sheet: also <hostname>.A4.pdf; PrintLabelSheet: build/labels/labels_A4.pdf.
//
// The files go to build/labels/ (gitignored: they carry the password).
[SupportedOSPlatform("windows")]
public static class DeviceLabel
{
    private sealed record Line(string Text, int Height, bool Bold, int Y);

    /// The label of one board with the settings of appsettings.json (LabelPrinter); dryRun (or
    /// LabelPrinter:DryRun) writes the print job to build/labels/ and sends nothing.
    public static string Write(string rootDir, WebCredentials.Device d, bool dryRun = false) =>
        Write(rootDir, d, BuilderSettings.Current.LabelPrinter, dryRun);

    public static string Write(string rootDir, WebCredentials.Device d, LabelPrinterOptions o, bool dryRun)
    {
        dryRun |= o.DryRun;
        var dir = Path.Combine(rootDir, "build", "labels");
        Directory.CreateDirectory(dir);
        var basePath = Path.Combine(dir, d.Hostname);
        var type = string.IsNullOrWhiteSpace(o.Type) ? "Zebra" : o.Type.Trim();
        switch (type.ToLowerInvariant())
        {
            case "zebra":
            case "none":
                WriteZebra(basePath, d, o, send: !dryRun && type.ToLowerInvariant() == "zebra");
                break;
            case "brotherql":
                WriteBrotherQl(basePath, d, o.BrotherQL, send: !dryRun);
                break;
            default:
                throw new ArgumentException($"LabelPrinter:Type \"{o.Type}\" unbekannt (Zebra, BrotherQL, None).");
        }
        if (o.A4Sheet)
        {
            var pdf = basePath + ".A4.pdf";
            LabelSheet.Write(pdf, [d], o.SheetCellWidthMm, o.SheetCellHeightMm);
            Console.WriteLine($"A4-Bogen: {pdf}");
        }
        if (dryRun)
        {
            Console.WriteLine("Trockenlauf (--dry-run / LabelPrinter:DryRun): nichts an einen Drucker gesendet.");
        }
        return basePath;
    }

    /// Brother QL: <hostname>.<model>-<label>.png (exactly the printable dots, black/white) and .bin
    /// (the raster job), then sent by the configured backend.
    public static (string Png, string Bin) WriteBrotherQl(string basePath, WebCredentials.Device d, BrotherQlOptions q, bool send)
    {
        var model = BrotherQl.FindModel(q.Model);
        var media = BrotherQl.FindMedia(q.Label);
        var (w, h) = BrotherQl.ImageSize(media);
        using var bmp = LabelArt.Render(d, w, h, monochrome: true);
        var stem = $"{basePath}.{model.Name}-{media.Name}";
        bmp.Save(stem + ".png", ImageFormat.Png);
        var job = BrotherQl.Encode(LabelArt.Gray(bmp), bmp.Width, bmp.Height, model, media);
        File.WriteAllBytes(stem + ".bin", job);
        var lengthMm = media.DieCut ? media.LengthMm : bmp.Height * 25.4 / 300;
        Console.WriteLine($"Etikett {model.Name}, {media.Dk}: {stem}.png ({bmp.Width} x {bmp.Height} Punkte, " +
                          $"{media.WidthMm} x {lengthMm:0} mm) / .bin ({job.Length} Bytes Rasterauftrag)");
        if (!send)
        {
            return (stem + ".png", stem + ".bin");
        }
        var target = string.IsNullOrWhiteSpace(q.Target) ? null : q.Target.Trim();
        switch (q.Backend.Trim().ToLowerInvariant())
        {
            case "windowsraw":
                LabelTransport.WindowsRaw(job, target ?? $"Brother {model.Name}", $"FCU label {d.Hostname}");
                break;
            case "windowsdriver":
                LabelTransport.WindowsDriver(bmp, target ?? $"Brother {model.Name}",
                                             media.DieCut ? media.LengthMm : media.WidthMm,
                                             media.DieCut ? media.WidthMm : lengthMm);
                break;
            case "tcp":
                var (host, port) = LabelTransport.HostPort(target ?? throw new ArgumentException("LabelPrinter:BrotherQL:Target (host[:port]) fehlt."));
                LabelTransport.Tcp(job, host, port);
                break;
            case "brotherqlcli":
                LabelTransport.BrotherQlCli(q.BrotherQlCommand, q.BrotherQlBackend, model.Name,
                                            target ?? "usb://0x04f9:0x2042", media.Name, stem + ".png");
                break;
            case "file":
                if (target is not null)
                {
                    File.WriteAllBytes(target, job);
                    Console.WriteLine($"Rasterauftrag nach {target} geschrieben.");
                }
                break;
            default:
                throw new ArgumentException($"LabelPrinter:BrotherQL:Backend \"{q.Backend}\" unbekannt " +
                                            "(WindowsRaw, WindowsDriver, Tcp, BrotherQlCli, File).");
        }
        return (stem + ".png", stem + ".bin");
    }

    private static void WriteZebra(string basePath, WebCredentials.Device d, LabelPrinterOptions o, bool send)
    {
        var width = Dots(o.WidthMm, o.Dpi);
        var height = Dots(o.HeightMm, o.Dpi);
        var s = o.Dpi / 300.0;  // the layout is drawn for 300 dpi
        var url = $"https://{d.Hostname}.local/";
        var lines = Layout(d, s);
        // QR code: version 3 (29 modules) at ECC M fits the URL; module size so it fills ~2/3 of the height
        var qr = new QRCodeGenerator().CreateQrCode(url, QRCodeGenerator.ECCLevel.M);
        var modules = qr.ModuleMatrix.Count - 8;  // without the quiet zone
        var magnification = Math.Max(1, (int)Math.Floor(height * 0.72 / modules));
        var qrSize = modules * magnification;
        var qrX = width - qrSize - (int)(14 * s);
        var qrY = (int)(16 * s);

        var zpl = Zpl(lines, width, height, s, url, qrX, qrY, magnification);
        File.WriteAllText(basePath + ".zpl", zpl);
        using (var bitmap = Preview(lines, width, height, qr, qrX, qrY, magnification))
        {
            bitmap.Save(basePath + ".png", ImageFormat.Png);
            LabelSheet.WriteSingle(basePath + ".pdf", bitmap, o.WidthMm, o.HeightMm);
            Console.WriteLine($"Etikett: {basePath}.zpl / .png / .pdf ({o.WidthMm} x {o.HeightMm} mm, {o.Dpi} dpi)");
            if (send && !string.IsNullOrWhiteSpace(o.WindowsPrinter))
            {
                LabelTransport.WindowsDriver(bitmap, o.WindowsPrinter, o.WidthMm, o.HeightMm);  // printers without ZPL (driver)
                return;
            }
        }
        if (send && !string.IsNullOrWhiteSpace(o.Host))
        {
            LabelTransport.Tcp(Encoding.UTF8.GetBytes(zpl), o.Host, o.Port);
        }
        else if (send)
        {
            Console.WriteLine("Kein Etikettendrucker konfiguriert (LabelPrinter:Host/WindowsPrinter/Type): nur Dateien geschrieben.");
        }
    }

    private static int Dots(double mm, int dpi) => (int)Math.Round(mm / 25.4 * dpi);

    private static List<Line> Layout(WebCredentials.Device d, double s)
    {
        int Px(double v) => (int)Math.Round(v * s);
        var date = d.ProvisionedAt.ToString("yyyy-MM-dd", CultureInfo.InvariantCulture);
        return
        [
            new(d.Hostname, Px(34), true, Px(14)),
            new($"ID {d.ChipUid}", Px(21), false, Px(58)),
            new($"MAC {d.EthMac}", Px(21), false, Px(84)),
            new($"HW {d.Revision}", Px(21), false, Px(110)),
            new($"Provisioniert {date}", Px(21), false, Px(136)),
            new("Web-Passwort (einmalig):", Px(21), false, Px(176)),
            new(d.Password, Px(34), true, Px(202)),
        ];
    }

    private static string Zpl(List<Line> lines, int width, int height, double s, string url, int qrX, int qrY, int magnification)
    {
        var x = (int)(16 * s);
        var z = new StringBuilder();
        z.Append("^XA\n^CI28\n");  // UTF-8
        z.Append(CultureInfo.InvariantCulture, $"^PW{width}\n^LL{height}\n^LH0,0\n");
        foreach (var l in lines)
        {
            // font 0 (scalable); bold lines a little wider
            var w = l.Bold ? (int)(l.Height * 0.95) : (int)(l.Height * 0.8);
            z.Append(CultureInfo.InvariantCulture, $"^FO{x},{l.Y}^A0N,{l.Height},{w}^FD{Escape(l.Text)}^FS\n");
        }
        // QR code, model 2, error correction M, manual input (M = ECC, A = automatic mode)
        z.Append(CultureInfo.InvariantCulture, $"^FO{qrX},{qrY}^BQN,2,{magnification}^FDMA,{Escape(url)}^FS\n");
        z.Append("^XZ\n");
        return z.ToString();
    }

    // ^ and ~ are ZPL command prefixes: never in field data (hex escape with ^FH would be needed)
    private static string Escape(string s) => s.Replace("^", " ").Replace("~", " ");

    private static Bitmap Preview(List<Line> lines, int width, int height, QRCodeData qr, int qrX, int qrY, int magnification)
    {
        var bmp = new Bitmap(width, height, PixelFormat.Format24bppRgb);
        using var g = Graphics.FromImage(bmp);
        g.Clear(Color.White);
        g.TextRenderingHint = TextRenderingHint.AntiAliasGridFit;
        g.SmoothingMode = SmoothingMode.None;
        var x = (int)(16 * width / 602.0);
        foreach (var l in lines)
        {
            // ZPL's font height is the cell height; the cap height of Arial is about 0.72 of it
            using var font = new Font("Arial", l.Height * 0.86f, l.Bold ? FontStyle.Bold : FontStyle.Regular, GraphicsUnit.Pixel);
            g.DrawString(l.Text, font, Brushes.Black, x, l.Y);
        }
        var m = qr.ModuleMatrix;
        for (var row = 4; row < m.Count - 4; ++row)
        {
            for (var col = 4; col < m.Count - 4; ++col)
            {
                if (m[row][col])
                {
                    g.FillRectangle(Brushes.Black, qrX + (col - 4) * magnification, qrY + (row - 4) * magnification, magnification, magnification);
                }
            }
        }
        using var pen = new Pen(Color.FromArgb(200, 200, 200));
        g.DrawRectangle(pen, 0, 0, width - 1, height - 1);  // the label's edge (preview only)
        return bmp;
    }

}
