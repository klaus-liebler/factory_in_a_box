<#
.SYNOPSIS
    Die SPS-Web-App ohne Hardware (web_sim) bauen und nach https://liebler.iui.hs-osnabrueck.de/factorycontrol/ ausrollen.
.DESCRIPTION
    nginx liefert /var/www/html statisch und ohne Passwort aus; die App liegt in /var/www/html/factorycontrol (gehoert srvadmin,
    wie rgb/). Nur beim allerersten Mal legt das Skript dieses Verzeichnis mit sudo an (fragt nach dem Passwort von srvadmin),
    danach braucht es kein sudo. Auf dem Server spielt ausrollen.sh die neue Version ein (erst assets/, dann index.html, dann
    Altes weg) und behaelt die vorige als ~/factorycontrol.vorher (Rueckweg: -Zurueck).
.EXAMPLE
    .\web_sim\deploy\deploy.ps1               # bauen + hochladen + einspielen + pruefen
.EXAMPLE
    .\web_sim\deploy\deploy.ps1 -NurBauen     # nur bauen (web_sim\dist)
.EXAMPLE
    .\web_sim\deploy\deploy.ps1 -OhneBauen    # das vorhandene web_sim\dist ausrollen
.EXAMPLE
    .\web_sim\deploy\deploy.ps1 -Zurueck      # die vorige Version wieder einspielen
#>
param(
    [string] $Ziel = "srvadmin@liebler.iui.hs-osnabrueck.de",
    [string] $Schluessel = "$env:USERPROFILE\.ssh\id_ed25519_liebler_iui_hs_osnabrueck",
    [string] $Url = "https://liebler.iui.hs-osnabrueck.de/factorycontrol/",
    [switch] $NurBauen,
    [switch] $OhneBauen,
    [switch] $Zurueck
)

$ErrorActionPreference = "Stop"
$projekt = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path   # firmware_factory_control_unit
$dist = Join-Path $projekt "web_sim\dist"
$serverVerzeichnis = "/var/www/html/factorycontrol"
$sshArgs = @()
if (Test-Path $Schluessel) { $sshArgs = @("-i", $Schluessel) }

function Pruefen {
    Write-Host "» pruefen: $Url" -ForegroundColor Cyan
    $seite = Invoke-WebRequest -Uri $Url -UseBasicParsing
    if ($seite.StatusCode -ne 200 -or $seite.Content -notmatch "SPS-Simulator") { throw "$Url liefert nicht die App (Status $($seite.StatusCode))." }
    $skript = [regex]::Match($seite.Content, 'src="\./(assets/[^"]+\.js)"').Groups[1].Value
    if (-not $skript) { throw "index.html verweist auf kein Skript." }
    $js = Invoke-WebRequest -Uri ($Url + $skript) -UseBasicParsing
    if ($js.StatusCode -ne 200) { throw "$skript fehlt auf dem Server." }
    Write-Host ("✓ erreichbar: {0} ({1}, {2:N0} kB)" -f $Url, $skript, ($js.RawContentLength / 1KB)) -ForegroundColor Green
}

if ($Zurueck) {
    scp @sshArgs (Join-Path $PSScriptRoot "ausrollen.sh") "${Ziel}:/tmp/factorycontrol-ausrollen.sh"
    ssh @sshArgs $Ziel "bash /tmp/factorycontrol-ausrollen.sh --zurueck"
    if ($LASTEXITCODE -ne 0) { throw "Zurueckspielen fehlgeschlagen - Meldung oben." }
    Pruefen
    return
}

if (-not $OhneBauen) {
    # Der Builder erzeugt zuerst die Registerkarte (web/generated/register-map.ts, ohne Board) und baut dann web_sim.
    Write-Host "» bauen: BuildSimulationWebApp" -ForegroundColor Cyan
    if (-not (Test-Path (Join-Path $projekt "node_modules\vite"))) {
        Push-Location $projekt
        try { npm install; if ($LASTEXITCODE -ne 0) { throw "npm install fehlgeschlagen." } } finally { Pop-Location }
    }
    dotnet run --project (Join-Path $projekt "builder") -- BuildSimulationWebApp
    if ($LASTEXITCODE -ne 0) { throw "Build fehlgeschlagen." }
}
if (-not (Test-Path (Join-Path $dist "index.html"))) { throw "$dist\index.html fehlt - erst bauen." }
if ($NurBauen) { Write-Host "✓ gebaut: $dist" -ForegroundColor Green; return }

$archiv = Join-Path $env:TEMP "factorycontrol.tar.gz"
tar -czf $archiv -C $dist .
if ($LASTEXITCODE -ne 0) { throw "Archiv konnte nicht erstellt werden." }
Write-Host ("» hochladen ({0:N0} kB)" -f ((Get-Item $archiv).Length / 1KB)) -ForegroundColor Cyan
scp @sshArgs $archiv "${Ziel}:/tmp/factorycontrol.tar.gz"
scp @sshArgs (Join-Path $PSScriptRoot "ausrollen.sh") "${Ziel}:/tmp/factorycontrol-ausrollen.sh"
if ($LASTEXITCODE -ne 0) { throw "Hochladen fehlgeschlagen." }
Remove-Item $archiv

# Einmalig: das Verzeichnis anlegen und srvadmin geben (danach kein sudo mehr)
ssh @sshArgs $Ziel "test -w $serverVerzeichnis"
if ($LASTEXITCODE -ne 0) {
    Write-Host "» einmalig: $serverVerzeichnis anlegen (sudo fragt nach dem Passwort von srvadmin)" -ForegroundColor Cyan
    ssh @sshArgs -t $Ziel "sudo install -d -o srvadmin -g srvadmin -m 755 $serverVerzeichnis"
    if ($LASTEXITCODE -ne 0) { throw "$serverVerzeichnis konnte nicht angelegt werden." }
}

Write-Host "» einspielen" -ForegroundColor Cyan
ssh @sshArgs $Ziel "bash /tmp/factorycontrol-ausrollen.sh /tmp/factorycontrol.tar.gz"
if ($LASTEXITCODE -ne 0) { throw "Einspielen fehlgeschlagen - Meldung oben (die vorige Version liegt in ~/factorycontrol.vorher, -Zurueck)." }
Pruefen
