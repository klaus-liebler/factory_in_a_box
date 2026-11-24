using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Drawing.Text;
using System.Globalization;
using System.Runtime.Versioning;
using QRCoder;

namespace Builder.Labels;

// The device label as a bitmap of any size at 300 dpi (Brother QL raster, A4 sheet). Two layouts:
//   tall    (62 mm tape, A4 cell): text left (hostname, ID, MAC, HW, date), QR code right, the
//           password over the full width below, the URL under it
//   compact (die-cut 29x90, 17x54, width > 2 x height): QR code left, ID and password right
// Text is fitted to its box (shrunk until it fits). The QR code carries https://<hostname>.local/
// only, never the password, with integer module size (sharp edges on a thermal head).
[SupportedOSPlatform("windows")]
public static class LabelArt
{
    public static string Url(WebCredentials.Device d) => $"https://{d.Hostname}.local/";

    /// height null: as long as the content needs (endless tape). Otherwise the content is centred
    /// vertically in width x height.
    public static Bitmap Render(WebCredentials.Device d, int width, int? height = null, bool monochrome = false)
    {
        var compact = height is { } h0 && width > 2 * h0;
        var canvasHeight = height ?? width * 2;
        var bmp = new Bitmap(width, canvasHeight, PixelFormat.Format24bppRgb);
        int used;
        using (var g = Graphics.FromImage(bmp))
        {
            g.Clear(Color.White);
            g.TextRenderingHint = TextRenderingHint.AntiAliasGridFit;
            g.SmoothingMode = SmoothingMode.None;
            g.PixelOffsetMode = PixelOffsetMode.Half;
            used = compact ? Compact(g, d, width, canvasHeight) : Tall(g, d, width, height);
        }
        Bitmap result;
        if (height is null)
        {
            result = bmp.Clone(new Rectangle(0, 0, width, used), PixelFormat.Format24bppRgb);
            bmp.Dispose();
        }
        else
        {
            result = bmp;
        }
        if (monochrome)
        {
            Threshold(result);
        }
        return result;
    }

    private static int Tall(Graphics g, WebCredentials.Device d, int w, int? fixedHeight)
    {
        int P(double f) => (int)Math.Round(f * w);
        var pad = P(0.03);
        var qr = Qr(d);
        var n = qr.ModuleMatrix.Count - 8;
        var mod = Math.Max(1, P(0.34) / n);
        var qs = n * mod;
        var qx = w - pad - qs;
        var tw = qx - 2 * pad;
        var date = d.ProvisionedAt.ToString("yyyy-MM-dd", CultureInfo.InvariantCulture);

        var top = new List<Item>
        {
            new(d.Hostname, Style.Bold, tw, P(0.062)),
            new($"ID {d.ChipUid}", Style.Regular, tw, P(0.038)),
            new($"MAC {d.EthMac}", Style.Regular, tw, P(0.038)),
            new($"HW {d.Revision}", Style.Regular, tw, P(0.038)),
            new($"Provisioniert {date}", Style.Regular, tw, P(0.038)),
        };
        var full = w - 2 * pad;
        var bottom = new List<Item>
        {
            new("Web-Passwort (einmalig, Benutzer admin):", Style.Regular, full, P(0.036)),
            new(d.Password, Style.Mono, full, P(0.09)),
            new(Url(d), Style.Regular, full, P(0.034)),
        };
        var gap = P(0.02);
        var topH = Measure(g, top);
        var blockTop = Math.Max(topH, qs);
        var total = pad + blockTop + gap + 2 + gap + Measure(g, bottom) + pad;
        var y0 = fixedHeight is { } fh ? Math.Max(0, (fh - total) / 2) : 0;

        var y = y0 + pad;
        DrawQr(g, qr, qx, y, mod);
        Draw(g, top, pad, y);
        y += blockTop + gap;
        g.FillRectangle(Brushes.Black, pad, y, full, 2);
        y += 2 + gap;
        y += Draw(g, bottom, pad, y);
        return y + pad;
    }

    private static int Compact(Graphics g, WebCredentials.Device d, int w, int h)
    {
        var pad = Math.Max(4, (int)Math.Round(h * 0.05));
        var qr = Qr(d);
        var n = qr.ModuleMatrix.Count - 8;
        var mod = Math.Max(1, (h - 2 * pad) / n);
        var qs = n * mod;
        DrawQr(g, qr, pad, (h - qs) / 2, mod);
        var tx = pad + qs + 2 * pad;
        var tw = w - tx - pad;
        int H(double f) => (int)Math.Round(f * h);
        List<Item> lines = h >= 250
            ?
            [
                new(d.Hostname, Style.Bold, tw, H(0.15)),
                new($"ID {d.ChipUid}", Style.Regular, tw, H(0.105)),
                new("Web-Passwort (admin):", Style.Regular, tw, H(0.095)),
                new(d.Password, Style.Mono, tw, H(0.2)),
            ]
            :
            [
                new($"ID {d.ChipUid}", Style.Regular, tw, H(0.17)),
                new("Web-Passwort (admin):", Style.Regular, tw, H(0.15)),
                new(d.Password, Style.Mono, tw, H(0.3)),
            ];
        var th = Measure(g, lines);
        Draw(g, lines, tx, Math.Max(pad, (h - th) / 2));
        return h;
    }

    private enum Style { Regular, Bold, Mono }

    private sealed record Item(string Text, Style Style, int MaxWidth, int MaxHeight);

    private static QRCodeData Qr(WebCredentials.Device d) =>
        new QRCodeGenerator().CreateQrCode(Url(d), QRCodeGenerator.ECCLevel.M);

    private static void DrawQr(Graphics g, QRCodeData qr, int x, int y, int mod)
    {
        var m = qr.ModuleMatrix;
        for (var row = 4; row < m.Count - 4; ++row)
        {
            for (var col = 4; col < m.Count - 4; ++col)
            {
                if (m[row][col])
                {
                    g.FillRectangle(Brushes.Black, x + (col - 4) * mod, y + (row - 4) * mod, mod, mod);
                }
            }
        }
    }

    private static Font Fit(Graphics g, Item it)
    {
        var family = it.Style == Style.Mono ? "Consolas" : "Arial";
        var style = it.Style == Style.Regular ? FontStyle.Regular : FontStyle.Bold;
        // MaxHeight is the line height; the em size is about 1/1.15 of it
        var px = Math.Max(6, (int)(it.MaxHeight / 1.15));
        while (true)
        {
            var f = new Font(family, px, style, GraphicsUnit.Pixel);
            var width = g.MeasureString(it.Text, f, PointF.Empty, StringFormat.GenericTypographic).Width;
            if (width <= it.MaxWidth || px <= 6)
            {
                return f;
            }
            f.Dispose();
            px = Math.Max(6, Math.Min(px - 1, (int)(px * it.MaxWidth / width)));
        }
    }

    private static int LineHeight(Graphics g, Font f) => (int)Math.Ceiling(f.Size * 1.15);

    private static int Measure(Graphics g, List<Item> items)
    {
        var h = 0;
        foreach (var it in items)
        {
            using var f = Fit(g, it);
            h += LineHeight(g, f);
        }
        return h;
    }

    private static int Draw(Graphics g, List<Item> items, int x, int y)
    {
        var y0 = y;
        foreach (var it in items)
        {
            using var f = Fit(g, it);
            g.DrawString(it.Text, f, Brushes.Black, x, y, StringFormat.GenericTypographic);
            y += LineHeight(g, f);
        }
        return y - y0;
    }

    /// Black or white only (what a thermal head prints): grey below 180 becomes black, as brother_ql's
    /// default threshold (70 %) does.
    public static void Threshold(Bitmap bmp)
    {
        var gray = Gray(bmp);
        var data = bmp.LockBits(new Rectangle(0, 0, bmp.Width, bmp.Height), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
        try
        {
            var row = new byte[data.Stride];
            for (var y = 0; y < bmp.Height; ++y)
            {
                for (var x = 0; x < bmp.Width; ++x)
                {
                    var v = gray[y * bmp.Width + x] < 180 ? (byte)0 : (byte)255;
                    row[x * 3] = row[x * 3 + 1] = row[x * 3 + 2] = v;
                }
                System.Runtime.InteropServices.Marshal.Copy(row, 0, data.Scan0 + y * data.Stride, data.Stride);
            }
        }
        finally
        {
            bmp.UnlockBits(data);
        }
    }

    /// Luminance per pixel (ITU-R 601, as PIL's convert("L")), row by row.
    public static byte[] Gray(Bitmap bmp)
    {
        var w = bmp.Width;
        var h = bmp.Height;
        var result = new byte[w * h];
        using var src = bmp.PixelFormat == PixelFormat.Format24bppRgb ? null : bmp.Clone(new Rectangle(0, 0, w, h), PixelFormat.Format24bppRgb);
        var b = src ?? bmp;
        var data = b.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            var row = new byte[data.Stride];
            for (var y = 0; y < h; ++y)
            {
                System.Runtime.InteropServices.Marshal.Copy(data.Scan0 + y * data.Stride, row, 0, data.Stride);
                for (var x = 0; x < w; ++x)
                {
                    int bl = row[x * 3], gr = row[x * 3 + 1], r = row[x * 3 + 2];
                    result[y * w + x] = (byte)((r * 299 + gr * 587 + bl * 114) / 1000);
                }
            }
        }
        finally
        {
            b.UnlockBits(data);
        }
        return result;
    }
}
