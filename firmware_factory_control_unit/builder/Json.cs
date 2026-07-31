// Gemeinsame JsonSerializerOptions fuer alle Phasen. System.Text.Json escaped mit dem
// Default-Encoder vorsichtshalber auch Zeichen wie '/>/< als '/>/< (HTML-sicher) --
// das waere zwar gueltiges JSON/TS, weicht aber unnoetig von Node's JSON.stringify() ab (das
// builder/ verwendet) und macht generierte Dateien schwerer lesbar. UnsafeRelaxedJsonEscaping
// deaktiviert das (unproblematisch hier: alle Ausgaben sind .ts/.json-Quelldateien, kein HTML).
using System.Text.Encodings.Web;
using System.Text.Json;

namespace Builder;

public static class Json
{
	public static readonly JsonSerializerOptions Options = new()
	{
		WriteIndented = true,
		Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
	};

	public static readonly JsonSerializerOptions Compact = new()
	{
		Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
	};
}
