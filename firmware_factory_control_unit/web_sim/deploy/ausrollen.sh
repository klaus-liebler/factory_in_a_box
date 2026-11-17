#!/usr/bin/env bash
# Neue Version der SPS-Web-App (web_sim) in /var/www/html/factorycontrol einspielen - von deploy.ps1 aufgerufen, als srvadmin
# (kein sudo; das Verzeichnis gehoert srvadmin). Reihenfolge ohne Ausfall: erst die neuen assets/ (Namen mit Hash, stoeren
# die laufende Version nicht), dann index.html, zuletzt die alten Dateien entfernen. Die vorige Version bleibt als
# ~/factorycontrol.vorher.
# Aufruf: bash ausrollen.sh /tmp/factorycontrol.tar.gz   ·   Rueckweg: bash ausrollen.sh --zurueck
set -euo pipefail

ZIEL=/var/www/html/factorycontrol
VORHER="$HOME/factorycontrol.vorher"
HOST=liebler.iui.hs-osnabrueck.de

ok()    { printf '\033[1;32m✓\033[0m %s\n' "$*"; }
fehler(){ printf '\033[1;31m✗ %s\033[0m\n' "$*" >&2; exit 1; }

[[ -d "$ZIEL" && -w "$ZIEL" ]] || fehler "$ZIEL fehlt oder ist nicht beschreibbar (einmalig: sudo install -d -o srvadmin -g srvadmin -m 755 $ZIEL)"

# einspielen <verzeichnis mit index.html>: Inhalt von $ZIEL durch den des Verzeichnisses ersetzen
einspielen() {
    local neu="$1"
    [[ -f "$neu/index.html" ]] || fehler "keine index.html in $neu"
    (cd "$neu" && find . -mindepth 1 -type d) | while read -r d; do mkdir -p "$ZIEL/$d"; done
    (cd "$neu" && find . -type f ! -path ./index.html) | while read -r f; do cp -f "$neu/$f" "$ZIEL/$f"; done
    cp -f "$neu/index.html" "$ZIEL/.index.html.neu" && mv -f "$ZIEL/.index.html.neu" "$ZIEL/index.html"
    (cd "$ZIEL" && find . -type f) | while read -r f; do [[ -e "$neu/$f" ]] || rm -f "$ZIEL/$f"; done
    find "$ZIEL" -mindepth 1 -type d -empty -delete
    find "$ZIEL" -type d -exec chmod 755 {} +
    find "$ZIEL" -type f -exec chmod 644 {} +
}

pruefen() {
    curl -fsS --resolve "$HOST:443:127.0.0.1" "https://$HOST/factorycontrol/" | grep -q "SPS-Simulator" \
        || fehler "https://$HOST/factorycontrol/ liefert die App nicht"
    ok "nginx liefert https://$HOST/factorycontrol/ aus"
}

if [[ "${1:-}" == "--zurueck" ]]; then
    [[ -f "$VORHER/index.html" ]] || fehler "keine vorige Version in $VORHER"
    tmp=$(mktemp -d)
    cp -a "$VORHER/." "$tmp/"
    einspielen "$tmp"
    rm -rf "$tmp"
    ok "vorige Version wieder eingespielt"
    pruefen
    exit 0
fi

archiv="${1:?Pfad zum Archiv fehlt}"
[[ -f "$archiv" ]] || fehler "Archiv $archiv fehlt"
neu=$(mktemp -d)
trap 'rm -rf "$neu"' EXIT
tar -xzf "$archiv" -C "$neu"
[[ -f "$neu/index.html" ]] || fehler "Archiv enthaelt keine index.html - nichts geaendert"

if [[ -f "$ZIEL/index.html" ]]; then
    rm -rf "$VORHER" && cp -a "$ZIEL" "$VORHER"
fi
einspielen "$neu"
rm -f "$archiv"
ok "eingespielt: $(find "$ZIEL" -type f | wc -l) Dateien in $ZIEL"
pruefen
