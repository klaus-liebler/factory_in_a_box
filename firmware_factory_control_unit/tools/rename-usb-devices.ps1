# FactoryControl Unit -- Geraetenamen im Geraete-Manager setzen (Registry, keine Treiberinstallation)
#
# Setzt den im Geraete-Manager angezeigten Namen fuer die drei USB-Funktionen des
# FactoryControl Unit (VID_1209&PID_0001) direkt ueber die "FriendlyName"-Registrierung des
# jeweiligen Geraete-Instanzknotens (HKLM\SYSTEM\CurrentControlSet\Enum\<InstanceId>). Das
# umgeht das Windows-Treibersignatur-Erfordernis, das factory_control_usb_ports.inf ausloest
# (Windows verlangt seit Windows 10 eine signierte Katalogdatei fuer JEDES per Plug&Play
# installierte Treiberpaket, auch fuer reine "Include=mdmcpq.inf"-Weiterleitungen ohne eigene
# .sys-Datei) -- hier wird ueberhaupt kein Treiber installiert, nur ein bereits vorhandener
# Anzeige-String auf einem laengst installierten Geraet ueberschrieben.
#
# Hardware-IDs (s. Kommentar in Core/Src/usbd_device.c zum Interface-Layout):
#   MI_00 = CDC-NCM Control-Interface (Netzwerkadapter)
#   MI_02 = CDC-ACM Debug (erster COM-Port)
#   MI_04 = CDC-ACM Modbus (zweiter COM-Port)
#
# Ausfuehrung: PowerShell ALS ADMINISTRATOR starten, dann:
#   powershell -ExecutionPolicy Bypass -File rename-usb-devices.ps1
# Danach im Geraete-Manager F5 (Aktualisieren) oder das Board aus-/wieder einstecken.
#
# WICHTIG: Dieser Eintrag wird von Windows automatisch neu (mit dem generischen Standardnamen)
# angelegt, sobald das Geraet ueber "Deinstallieren" + "Treibersoftware loeschen" entfernt und
# neu erkannt wird -- das Skript muss dann erneut ausgefuehrt werden. Ein einfaches Aus-/
# Einstecken oder ein Neustart loescht den Eintrag dagegen NICHT.

$ErrorActionPreference = "Stop"

$targets = @(
    @{ HwidPattern = "USB\VID_1209&PID_0001&MI_00*"; Name = "FactoryControl Network" }
    @{ HwidPattern = "USB\VID_1209&PID_0001&MI_02*"; Name = "FactoryControl Debug" }
    @{ HwidPattern = "USB\VID_1209&PID_0001&MI_04*"; Name = "FactoryControl Modbus RTU" }
)

foreach ($target in $targets) {
    $devices = Get-PnpDevice | Where-Object { $_.InstanceId -like $target.HwidPattern }
    if (-not $devices) {
        Write-Warning "Kein Geraet gefunden fuer $($target.HwidPattern) -- ist das Board angeschlossen?"
        continue
    }
    foreach ($device in $devices) {
        $regPath = "HKLM:\SYSTEM\CurrentControlSet\Enum\$($device.InstanceId)"
        Set-ItemProperty -Path $regPath -Name "FriendlyName" -Value $target.Name
        Write-Host "OK: $($device.InstanceId) -> `"$($target.Name)`""
    }
}

Write-Host "`nFertig. Im Geraete-Manager F5 druecken oder das Board aus-/wieder einstecken."
