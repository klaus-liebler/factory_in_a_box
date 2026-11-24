using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Runtime.Versioning;

namespace Builder.Labels;

// The ways a finished print job reaches a printer.
public static class LabelTransport
{
    /// Raw bytes to host:port (ZPL printers, network Brother QL, print servers; port 9100).
    public static void Tcp(byte[] data, string host, int port)
    {
        using var client = new TcpClient();
        if (!client.ConnectAsync(host, port).Wait(TimeSpan.FromSeconds(5)))
        {
            throw new InvalidOperationException($"Drucker {host}:{port} antwortet nicht.");
        }
        using var stream = client.GetStream();
        stream.Write(data);
        Console.WriteLine($"Druckauftrag ({data.Length} Bytes) an {host}:{port} gesendet.");
    }

    public static (string Host, int Port) HostPort(string target, int defaultPort = 9100)
    {
        var t = target.StartsWith("tcp://", StringComparison.OrdinalIgnoreCase) ? target[6..] : target;
        var i = t.LastIndexOf(':');
        return i > 0 && int.TryParse(t[(i + 1)..], out var p) ? (t[..i], p) : (t, defaultPort);
    }

    /// Raw bytes through a Windows printer queue, datatype RAW: the spooler passes them unchanged to
    /// the port (USB001 of the Brother driver), the driver does not render. No libusb/Zadig needed.
    [SupportedOSPlatform("windows")]
    public static void WindowsRaw(byte[] data, string printer, string jobName)
    {
        if (!OpenPrinter(printer, out var h, IntPtr.Zero))
        {
            throw new InvalidOperationException($"Windows-Drucker \"{printer}\" nicht gefunden ({new Win32Exception().Message}).");
        }
        try
        {
            var doc = new DOC_INFO_1 { pDocName = jobName, pOutputFile = null, pDataType = "RAW" };
            if (StartDocPrinter(h, 1, ref doc) == 0)
            {
                throw new InvalidOperationException($"StartDocPrinter(\"{printer}\", RAW): {new Win32Exception().Message}");
            }
            try
            {
                if (!StartPagePrinter(h)) throw new Win32Exception();
                var ptr = Marshal.AllocHGlobal(data.Length);
                try
                {
                    Marshal.Copy(data, 0, ptr, data.Length);
                    if (!WritePrinter(h, ptr, data.Length, out var written) || written != data.Length)
                    {
                        throw new InvalidOperationException($"WritePrinter(\"{printer}\"): {new Win32Exception().Message}");
                    }
                }
                finally
                {
                    Marshal.FreeHGlobal(ptr);
                }
                EndPagePrinter(h);
            }
            finally
            {
                EndDocPrinter(h);
            }
        }
        finally
        {
            ClosePrinter(h);
        }
        Console.WriteLine($"Druckauftrag ({data.Length} Bytes, RAW) an den Windows-Drucker \"{printer}\" gesendet.");
    }

    /// A bitmap through a Windows printer driver at the given size (no scaling beyond the driver's
    /// own dpi): for printers that have a driver but no raw command language we speak.
    [SupportedOSPlatform("windows")]
    public static void WindowsDriver(Bitmap bmp, string printer, double widthMm, double heightMm)
    {
        using var doc = new System.Drawing.Printing.PrintDocument();
        doc.PrinterSettings.PrinterName = printer;
        if (!doc.PrinterSettings.IsValid)
        {
            throw new InvalidOperationException($"Windows-Drucker \"{printer}\" nicht gefunden.");
        }
        doc.DocumentName = "FCU label";
        // hundredths of an inch
        var w = (int)Math.Round(widthMm / 25.4 * 100);
        var h = (int)Math.Round(heightMm / 25.4 * 100);
        doc.DefaultPageSettings.PaperSize = new System.Drawing.Printing.PaperSize("FCU label", w, h);
        doc.DefaultPageSettings.Margins = new System.Drawing.Printing.Margins(0, 0, 0, 0);
        doc.OriginAtMargins = false;
        doc.PrintPage += (_, e) =>
        {
            var g = e.Graphics!;
            g.InterpolationMode = InterpolationMode.NearestNeighbor;
            g.PixelOffsetMode = PixelOffsetMode.Half;
            // the hard margin of the driver: printable area starts there, not at the paper edge
            g.TranslateTransform(-e.PageSettings.HardMarginX, -e.PageSettings.HardMarginY);
            g.DrawImage(bmp, new RectangleF(0, 0, (float)(widthMm / 25.4 * 100), (float)(heightMm / 25.4 * 100)));
            e.HasMorePages = false;
        };
        doc.Print();
        Console.WriteLine($"Etikett ({widthMm:0.#} x {heightMm:0.#} mm) an den Windows-Drucker \"{printer}\" (Treiber) gesendet.");
    }

    /// The PNG through the python brother_ql CLI (pip install brother_ql-inventree):
    ///   brother_ql -b <backend> -m <model> -p <identifier> print -l <label> <png>
    public static void BrotherQlCli(string command, string? backend, string model, string identifier, string label, string png)
    {
        backend ??= identifier.StartsWith("usb://", StringComparison.OrdinalIgnoreCase) ? "pyusb"
                  : identifier.StartsWith("tcp://", StringComparison.OrdinalIgnoreCase) ? "network"
                  : identifier.StartsWith("file://", StringComparison.OrdinalIgnoreCase) ? "linux_kernel" : null;
        var psi = new ProcessStartInfo(command) { UseShellExecute = false };
        if (backend is not null)
        {
            psi.ArgumentList.Add("-b");
            psi.ArgumentList.Add(backend);
        }
        foreach (var a in new[] { "-m", model, "-p", identifier, "print", "-l", label, png })
        {
            psi.ArgumentList.Add(a);
        }
        using var p = Process.Start(psi) ?? throw new InvalidOperationException($"{command} liess sich nicht starten.");
        p.WaitForExit();
        if (p.ExitCode != 0)
        {
            throw new InvalidOperationException($"{command} endete mit {p.ExitCode}.");
        }
        Console.WriteLine($"Etikett ueber {command} an {identifier} gesendet.");
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    private struct DOC_INFO_1
    {
        public string pDocName;
        public string? pOutputFile;
        public string pDataType;
    }

    [DllImport("winspool.drv", CharSet = CharSet.Unicode, SetLastError = true, EntryPoint = "OpenPrinterW")]
    private static extern bool OpenPrinter(string pPrinterName, out IntPtr phPrinter, IntPtr pDefault);

    [DllImport("winspool.drv", SetLastError = true)]
    private static extern bool ClosePrinter(IntPtr hPrinter);

    [DllImport("winspool.drv", CharSet = CharSet.Unicode, SetLastError = true, EntryPoint = "StartDocPrinterW")]
    private static extern int StartDocPrinter(IntPtr hPrinter, int level, ref DOC_INFO_1 pDocInfo);

    [DllImport("winspool.drv", SetLastError = true)]
    private static extern bool EndDocPrinter(IntPtr hPrinter);

    [DllImport("winspool.drv", SetLastError = true)]
    private static extern bool StartPagePrinter(IntPtr hPrinter);

    [DllImport("winspool.drv", SetLastError = true)]
    private static extern bool EndPagePrinter(IntPtr hPrinter);

    [DllImport("winspool.drv", SetLastError = true)]
    private static extern bool WritePrinter(IntPtr hPrinter, IntPtr pBytes, int dwCount, out int dwWritten);
}
