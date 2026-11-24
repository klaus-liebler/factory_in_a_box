namespace Builder.Labels;

// Brother QL raster command stream ("Raster Command Reference" QL-700/800 series), byte for byte as
// the python package brother_ql (inventree fork 1.3, convert() with cut, threshold 70, no dither,
// no compression) produces it - tests/builder checks that against brother_ql when it is installed.
//
// The head has number_bytes_per_row x 8 dots (QL-700: 90 bytes = 720 dots). An image of the
// label's printable width is placed right-aligned with the label's right margin and mirrored; each
// image row is one raster line ("g" 0x00 <n> <n bytes>, MSB = leftmost dot after mirroring), in
// feed direction. Die-cut labels in landscape (length x width) are turned by 90 degrees first.
public static class BrotherQl
{
    public sealed record Model(string Name, int BytesPerRow, int MinLength, int MaxLength, bool ModeSetting,
                               bool Compression, int InvalidateBytes, bool Cutting = true, int AdditionalOffsetR = 0);

    /// Printable is (width, length) in dots at 300 dpi; length 0 = endless.
    public sealed record Media(string Name, int WidthMm, int LengthMm, bool DieCut, int PrintableWidth, int PrintableLength,
                               int RightMargin, int FeedMargin, string Dk);

    public static readonly IReadOnlyList<Model> Models =
    [
        new("QL-700", 90, 150, 11811, ModeSetting: false, Compression: false, InvalidateBytes: 200),
        new("QL-710W", 90, 150, 11811, ModeSetting: true, Compression: true, InvalidateBytes: 200),
        new("QL-720NW", 90, 150, 11811, ModeSetting: true, Compression: true, InvalidateBytes: 200),
        new("QL-800", 90, 150, 11811, ModeSetting: true, Compression: false, InvalidateBytes: 400),
        new("QL-810W", 90, 150, 11811, ModeSetting: true, Compression: true, InvalidateBytes: 400),
        new("QL-820NWB", 90, 150, 11811, ModeSetting: true, Compression: true, InvalidateBytes: 400),
    ];

    public static readonly IReadOnlyList<Media> Labels =
    [
        new("62", 62, 0, DieCut: false, 696, 0, RightMargin: 12, FeedMargin: 35, "DK-22205 (62 mm endlos)"),
        new("29x90", 29, 90, DieCut: true, 306, 991, RightMargin: 6, FeedMargin: 0, "DK-11201 (29 x 90 mm)"),
        new("17x54", 17, 54, DieCut: true, 165, 566, RightMargin: 0, FeedMargin: 0, "DK-11204 (17 x 54 mm)"),
    ];

    public static Model FindModel(string name) => Models.FirstOrDefault(m => string.Equals(m.Name, name, StringComparison.OrdinalIgnoreCase))
        ?? throw new ArgumentException($"Unbekanntes Brother-QL-Modell \"{name}\" (LabelPrinter:BrotherQL:Model). Bekannt: {string.Join(", ", Models.Select(m => m.Name))}.");

    public static Media FindMedia(string name) => Labels.FirstOrDefault(l => string.Equals(l.Name, name, StringComparison.OrdinalIgnoreCase))
        ?? throw new ArgumentException($"Unbekanntes Etikett \"{name}\" (LabelPrinter:BrotherQL:Label). Bekannt: {string.Join(", ", Labels.Select(l => l.Name))}.");

    /// The size the label image must have: endless (printable width, any length), die-cut in
    /// landscape (length x width) - the orientation in which the label is read.
    public static (int Width, int? Height) ImageSize(Media media) =>
        media.DieCut ? (media.PrintableLength, media.PrintableWidth) : (media.PrintableWidth, null);

    /// gray: one byte per pixel, row by row (0 = black). Below 180 prints black (brother_ql threshold 70 %).
    public static byte[] Encode(byte[] gray, int width, int height, Model model, Media media, bool cut = true)
    {
        if (gray.Length != width * height)
        {
            throw new ArgumentException("gray hat nicht width x height Bytes.");
        }
        if (media.DieCut && width == media.PrintableLength && height == media.PrintableWidth)
        {
            (gray, width, height) = RotateCcw(gray, width, height);
        }
        if (width != media.PrintableWidth || (media.DieCut && height != media.PrintableLength))
        {
            var expected = media.DieCut ? $"{media.PrintableWidth} x {media.PrintableLength}" : $"{media.PrintableWidth} x beliebig";
            throw new ArgumentException($"Bild {width} x {height} passt nicht zum Etikett {media.Name} (erwartet {expected} Punkte).");
        }
        if (height < model.MinLength || height > model.MaxLength)
        {
            throw new ArgumentException($"Etikettenlaenge {height} Punkte ausserhalb {model.MinLength}..{model.MaxLength} ({model.Name}).");
        }

        var o = new MemoryStream();
        void B(params byte[] b) => o.Write(b);
        if (model.ModeSetting) B(0x1B, 0x69, 0x61, 0x01);              // ESC i a: raster mode
        o.Write(new byte[model.InvalidateBytes]);                       // invalidate
        B(0x1B, 0x40);                                                  // ESC @: initialise
        if (model.ModeSetting) B(0x1B, 0x69, 0x61, 0x01);
        B(0x1B, 0x69, 0x53);                                            // ESC i S: status request
        // ESC i z: print information. Valid flags: recover 0x80 | kind 0x02 | width 0x04 | length 0x08 | quality 0x40
        B(0x1B, 0x69, 0x7A, 0xCE, (byte)(media.DieCut ? 0x0B : 0x0A), (byte)media.WidthMm, (byte)media.LengthMm);
        B(BitConverter.GetBytes((uint)height));                        // raster lines, little endian
        B(0x00, 0x00);                                                  // starting page, n10
        if (model.Cutting && cut)
        {
            B(0x1B, 0x69, 0x4D, 0x40);                                  // ESC i M: auto cut
            B(0x1B, 0x69, 0x41, 0x01);                                  // ESC i A: cut every label
        }
        B(0x1B, 0x69, 0x4B, (byte)(cut ? 0x08 : 0x00));                 // ESC i K: cut at end
        B(0x1B, 0x69, 0x64, (byte)(media.FeedMargin & 0xFF), (byte)(media.FeedMargin >> 8));  // ESC i d: margin
        if (model.Compression) B(0x4D, 0x00);                           // M: no compression

        var dots = model.BytesPerRow * 8;
        var offset = dots - width - media.RightMargin - model.AdditionalOffsetR;
        var line = new byte[model.BytesPerRow];
        for (var y = 0; y < height; ++y)
        {
            Array.Clear(line);
            for (var x = 0; x < width; ++x)
            {
                if (gray[y * width + x] < 180)
                {
                    var dot = dots - 1 - (offset + x);                 // mirrored
                    line[dot >> 3] |= (byte)(0x80 >> (dot & 7));
                }
            }
            B(0x67, 0x00, (byte)model.BytesPerRow);
            o.Write(line);
        }
        B(0x1A);                                                        // print, last page
        return o.ToArray();
    }

    // PIL's rotate(90, expand=True): counter-clockwise
    private static (byte[], int, int) RotateCcw(byte[] g, int w, int h)
    {
        var r = new byte[g.Length];
        for (var ny = 0; ny < w; ++ny)
        {
            for (var nx = 0; nx < h; ++nx)
            {
                r[ny * h + nx] = g[nx * w + (w - 1 - ny)];
            }
        }
        return (r, h, w);
    }
}
