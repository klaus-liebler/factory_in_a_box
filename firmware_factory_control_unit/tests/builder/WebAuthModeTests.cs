using Builder;

namespace Builder.Tests;

// WebAuth:Modes (docs/security.md "Login mode per interface"): login is the default for every
// interface, Open only where it is written, and a typo never opens or silently closes an interface.
public sealed class WebAuthModeTests
{
    [Fact]
    public void Every_interface_needs_a_login_by_default()
    {
        var o = new WebAuthOptions();
        Assert.False(o.IsOpen("Ethernet"));
        Assert.False(o.IsOpen("UsbNcm"));
    }

    [Fact]
    public void Open_is_per_interface_and_case_insensitive()
    {
        var o = new WebAuthOptions { Modes = { ["usbncm"] = "open", ["Ethernet"] = "OneTimePassword" } };
        Assert.True(o.IsOpen("UsbNcm"));
        Assert.False(o.IsOpen("Ethernet"));
    }

    [Theory]
    [InlineData("Ethernet", "Offen")]
    [InlineData("Wlan", "Open")]
    public void Unknown_modes_and_interfaces_are_errors(string iface, string mode)
    {
        var o = new WebAuthOptions { Modes = { [iface] = mode } };
        Assert.Throws<ArgumentException>(() => o.IsOpen("Ethernet"));
    }
}
