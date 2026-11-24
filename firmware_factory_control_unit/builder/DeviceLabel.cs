using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Drawing.Text;
using System.Globalization;
using System.IO.Compression;
using System.Net.Sockets;
using System.Runtime.Versioning;
using System.Text;
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
// The files go to build/labels/ (gitignored: they carry the password).
[SupportedOSPlatform("windows")]
public static class DeviceLabel
{
    private sealed record Line(string Text, int Height, bool Bold, int Y);

    public static string Write(string rootDir, WebCredentials.Device d)
    {
        var o = BuilderSettings.Current.LabelPrinter;
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

        var dir = Path.Combine(rootDir, "build", "labels");
        Directory.CreateDirectory(dir);
        var basePath = Path.Combine(dir, d.Hostname);
        var zpl = Zpl(lines, width, height, s, url, qrX, qrY, magnification);
        File.WriteAllText(basePath + ".zpl", zpl);
        using (var bitmap = Preview(lines, width, height, qr, qrX, qrY, magnification))
        {
            bitmap.Save(basePath + ".png", ImageFormat.Png);
            WritePdf(basePath + ".pdf", bitmap, o.WidthMm, o.HeightMm);
            Console.WriteLine($"Etikett: {basePath}.zpl / .png / .pdf ({o.WidthMm} x {o.HeightMm} mm, {o.Dpi} dpi)");
            if (!string.IsNullOrWhiteSpace(o.WindowsPrinter))
            {
                PrintImage(bitmap, o.WindowsPrinter, o.WidthMm, o.HeightMm);  // printers without ZPL (driver)
                return basePath;
            }
        }
        if (!string.IsNullOrWhiteSpace(o.Host))
        {
            Print(zpl, o.Host, o.Port);
        }
        else
        {
            Console.WriteLine("Kein Etikettendrucker konfiguriert (LabelPrinter:Host): nur Dateien geschrieben.");
        }
        return basePath;
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

    /// One page of the label's size with the preview as an RGB image (Flate).
    private static void WritePdf(string path, Bitmap bmp, double widthMm, double heightMm)
    {
        var raw = new byte[bmp.Width * bmp.Height * 3];
        var data = bmp.LockBits(new Rectangle(0, 0, bmp.Width, bmp.Height), ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            var row = new byte[data.Stride];
            for (var y = 0; y < bmp.Height; ++y)
            {
                System.Runtime.InteropServices.Marshal.Copy(data.Scan0 + y * data.Stride, row, 0, data.Stride);
                for (var xx = 0; xx < bmp.Width; ++xx)
                {
                    raw[(y * bmp.Width + xx) * 3 + 0] = row[xx * 3 + 2];  // BGR -> RGB
                    raw[(y * bmp.Width + xx) * 3 + 1] = row[xx * 3 + 1];
                    raw[(y * bmp.Width + xx) * 3 + 2] = row[xx * 3 + 0];
                }
            }
        }
        finally
        {
            bmp.UnlockBits(data);
        }
        byte[] image;
        using (var ms = new MemoryStream())
        {
            using (var z = new ZLibStream(ms, CompressionLevel.Optimal, leaveOpen: true)) z.Write(raw);
            image = ms.ToArray();
        }
        var wPt = (widthMm / 25.4 * 72).ToString("0.##", CultureInfo.InvariantCulture);
        var hPt = (heightMm / 25.4 * 72).ToString("0.##", CultureInfo.InvariantCulture);
        var content = Encoding.ASCII.GetBytes($"q {wPt} 0 0 {hPt} 0 0 cm /Im0 Do Q\n");
        using var f = new FileStream(path, FileMode.Create);
        var offsets = new List<long>();
        void W(string s) { var b = Encoding.ASCII.GetBytes(s); f.Write(b); }
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
        Obj("<< /Type /Catalog /Pages 2 0 R >>");
        Obj("<< /Type /Pages /Kids [3 0 R] /Count 1 >>");
        Obj($"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {wPt} {hPt}] /Resources << /XObject << /Im0 4 0 R >> >> /Contents 5 0 R >>");
        Obj($"<< /Type /XObject /Subtype /Image /Width {bmp.Width} /Height {bmp.Height} /ColorSpace /DeviceRGB /BitsPerComponent 8 " +
            $"/Filter /FlateDecode /Length {image.Length} >>", image);
        Obj($"<< /Length {content.Length} >>", content);
        var xref = f.Position;
        W($"xref\n0 {offsets.Count + 1}\n0000000000 65535 f \n");
        foreach (var off in offsets) W($"{off:D10} 00000 n \n");
        W($"trailer\n<< /Size {offsets.Count + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n");
    }

    /// The preview as a raster image through a Windows printer driver, at the label's size: for label
    /// printers that speak no ZPL (e.g. NIIMBOT, Brother QL) but have a Windows driver.
    private static void PrintImage(Bitmap bmp, string printer, double widthMm, double heightMm)
    {
        using var doc = new System.Drawing.Printing.PrintDocument();
        doc.PrinterSettings.PrinterName = printer;
        if (!doc.PrinterSettings.IsValid)
        {
            throw new InvalidOperationException($"Windows-Drucker \"{printer}\" nicht gefunden (LabelPrinter:WindowsPrinter).");
        }
        // hundredths of an inch
        var w = (int)Math.Round(widthMm / 25.4 * 100);
        var h = (int)Math.Round(heightMm / 25.4 * 100);
        doc.DefaultPageSettings.PaperSize = new System.Drawing.Printing.PaperSize("FCU label", w, h);
        doc.DefaultPageSettings.Margins = new System.Drawing.Printing.Margins(0, 0, 0, 0);
        doc.OriginAtMargins = true;
        doc.PrintPage += (_, e) =>
        {
            e.Graphics!.InterpolationMode = InterpolationMode.NearestNeighbor;
            e.Graphics.DrawImage(bmp, new Rectangle(0, 0, w, h));
            e.HasMorePages = false;
        };
        doc.Print();
        Console.WriteLine($"Etikett an den Windows-Drucker \"{printer}\" gesendet.");
    }

    private static void Print(string zpl, string host, int port)
    {
        using var client = new TcpClient();
        if (!client.ConnectAsync(host, port).Wait(TimeSpan.FromSeconds(5)))
        {
            throw new InvalidOperationException($"Etikettendrucker {host}:{port} antwortet nicht.");
        }
        using var stream = client.GetStream();
        stream.Write(Encoding.UTF8.GetBytes(zpl));
        Console.WriteLine($"Etikett an {host}:{port} gesendet.");
    }
}
