// Phase "ReadWebSocketProtocolAndGenerateFiles" (s. docs/websocket-protocol.md) -- erzeugt aus
// allen *.json-Dateien in ws-protocol/ (programmiersprachen-neutrales, von Hand gepflegtes Schema)
// zwei Dateien: Core/generated/ws_protocol.hh (C++) und web/generated/ws-protocol.ts (TypeScript).
//
// Kodierung ist durchgehend Cursor-basiert (eine laufende Byte-Position "pos"/"offset", die nach
// jedem Feld weiterrueckt) statt statisch vorausberechneter Offsets -- noetig, weil variabel lange
// Felder (string/array/class) an BELIEBIGER Position und MEHRFACH in einer Nachricht vorkommen
// duerfen (jedes traegt sein eigenes Laengenpraefix, s. docs/websocket-protocol.md).
//
// Namespaces sind KEINE Dateien mehr, sondern ergeben sich rein aus dem Namen jeder einzelnen
// Deklaration (Message/Enum/Struct/Class): enthaelt der Name einen Punkt, ist der Teil davor der
// Namespace-Name; ein Name ohne Punkt gehoert zum NULL-Namespace (Name "", NAMESPACE_ID immer 0,
// fest reserviert -- nicht Teil der normalen ID-Vergabe). Dadurch koennen beliebig viele Dateien
// zum selben Namespace beitragen, und eine einzelne Datei kann Deklarationen mehrerer Namespaces
// enthalten (s. docs/websocket-protocol.md Abschnitt 8).
using System.Text;
using System.Text.Json;

namespace Builder.Phases;

public static class ReadWebSocketProtocol
{
	// IsSigned steuert auf TS-Seite die Wahl zwischen getUint*/getInt* (DataView) -- C++ braucht das
	// nicht separat, dort reicht ein C-Style-Cast des vollstaendig rekonstruierten vorzeichenlosen
	// Werts auf den Zieltyp (reinterpretiert das Bitmuster automatisch korrekt als Zweierkomplement,
	// s. CppDecodeField). Auf der DataView-API muss dagegen VOR dem Lesen die passende
	// getInt*-Funktion gewaehlt werden, sonst liefert z.B. ein negatives int32-Feld nach dem Decode
	// einen positiven Wert zurueck (Bug, per Roundtrip-Test gefunden: -123456 wurde zu 4294843840).
	// IsFloat: eigener Kodierungspfad noetig (Bit-Muster per memcpy/DataView.getFloat32 uebertragen,
	// NICHT per Integer-Bitshift wie bei allen anderen Fixed-Typen -- ein numerischer Cast auf
	// uint32_t wuerde bei einem float eine WERT-Konvertierung statt einer Bit-Reinterpretation
	// durchfuehren und den Wert dabei zerstoeren, z.B. (uint32_t)3.14f == 3, nicht das IEEE754-Bitmuster).
	private sealed record FixedTypeInfo(string CppType, string TsType, int Size, bool IsBool, bool IsSigned, bool IsFloat = false);

	private static readonly Dictionary<string, FixedTypeInfo> FixedTypes = new()
	{
		["uint8"] = new("uint8_t", "number", 1, false, false),
		["int8"] = new("int8_t", "number", 1, false, true),
		["uint16"] = new("uint16_t", "number", 2, false, false),
		["int16"] = new("int16_t", "number", 2, false, true),
		["uint32"] = new("uint32_t", "number", 4, false, false),
		["int32"] = new("int32_t", "number", 4, false, true),
		["bool"] = new("bool", "boolean", 1, true, false),
		["float32"] = new("float", "number", 4, false, false, IsFloat: true),
		// TS-Seite bildet 64-Bit-Ganzzahlen bewusst auf "number" (nicht bigint) ab -- korrekt fuer
		// alle Werte im JS-sicheren Ganzzahlbereich (bis 2^53, z.B. Unix-Sekunden fuer die naechsten
		// ~285 Millionen Jahre), UND fuer int64 nur fuer NICHT-NEGATIVE Werte (s. Kommentar bei
		// TsDecode64BitInto/TsEncode64BitChunk). Fuer sensacts aktuell bekannten Bedarf
        // (ResponseSystemData.seconds_epoch/seconds_uptime, immer >=0) ausreichend; echte
		// beliebige 64-Bit-Praezision (negative int64, > 2^53) wuerde bigint + getBig(U)Int64
		// erfordern -- bislang nicht implementiert.
		["uint64"] = new("uint64_t", "number", 8, false, false),
		["int64"] = new("int64_t", "number", 8, false, true),
	};

	// Feld-Hierarchie statt eines einzelnen Records mit vielen optionalen Eigenschaften -- macht
	// die Codegenerierung unten (switch-Pattern-Matching) fuer jede Feldart explizit.
	private abstract record FieldBase(string Name, string? Description);
	private sealed record FixedField(string Name, string? Description, string CppType, string TsType, int Size, bool IsBool, bool IsSigned, bool IsFloat = false) : FieldBase(Name, Description);
	private sealed record EnumValue(string Name, long Value);
	// Inline/lokal deklariertes Enum (enumName+enumValues direkt am Feld) -- wird als eigener Typ
	// INNERHALB des umschliessenden Message/Struct/Class-Codes generiert, nicht wiederverwendbar.
	private sealed record EnumField(string Name, string? Description, string EnumName, int Size, List<EnumValue> Values) : FieldBase(Name, Description);
	// Referenz auf ein an anderer Stelle (namespace-level "enums") deklariertes, wiederverwendbares
	// Enum -- s. EnumDef unten. Wire-Format identisch zu EnumField (nur die Typ-Herkunft ist anders).
	private sealed record EnumRefField(string Name, string? Description, EnumDef Enum) : FieldBase(Name, Description);
	private sealed record StringField(string Name, string? Description) : FieldBase(Name, Description);
	private sealed record StructField(string Name, string? Description, StructDef Struct) : FieldBase(Name, Description);

	// "count": N auf einem Fixed-/Enum-/EnumRef-/Struct-Feld -- N gleichartige feste Werte
	// hintereinander (C-Array), z.B. eine 6-Byte-MAC-Adresse als "uint8 mit count 6" statt 6
	// einzeln benannter Felder. Bleibt selbst ein fester Feldtyp (Groesse = Elementgroesse * Count).
	private sealed record RepeatedField(string Name, string? Description, FieldBase Element, int Count) : FieldBase(Name, Description);

	private sealed record FixedArrayField(string Name, string? Description, string ElementName, List<FieldBase> ElementFields) : FieldBase(Name, Description);

	// Heterogenes/polymorphes Array: jedes Element ist eines von mehreren deklarierten "classes".
	// Muss laut ValidateNoArraysExceptTrailingClassField Teil eines zusammenhaengenden Blocks aus
	// ClassArrayField/ClassField am ENDE seines umschliessenden Objekts sein (mehrere solcher Felder
	// hintereinander sind erlaubt, s. z.B. fingerprint.ResponseFingers mit scheduleNames+fingers) --
	// Klassen-Elemente sind unterschiedlich lang (duerfen Strings enthalten), die Gesamtbytelaenge
	// ist deshalb ohne vollstaendiges Durchlaufen nicht bekannt, ein fester Feldtyp DAHINTER waere
	// also nicht ohne Weiteres adressierbar.
	private sealed record ClassArrayField(string Name, string? Description, List<ClassDef> Classes) : FieldBase(Name, Description);

	// Wie ClassArrayField, aber EIN einzelnes Element statt einer Liste (kein Laengenpraefix/Count)
	// -- fuer Felder, die zur Laufzeit eine von mehreren Klassen sein koennen, ohne dass es sich um
	// eine Liste handelt (z.B. "eine von drei Schedule-Varianten"). Unterliegt derselben
	// "muss Teil des trailing Blocks sein"-Einschraenkung wie ClassArrayField, aus demselben Grund.
	private sealed record ClassField(string Name, string? Description, List<ClassDef> Classes) : FieldBase(Name, Description);

	// Wiederverwendbares, benanntes Enum (namespace-level "enums") -- referenziert per EnumRefField
	// aus beliebig vielen Messages/Structs/Classes/Arrays, auch namespaceuebergreifend. Kein
	// Laufzeit-Overhead ggue. einem inline deklarierten Enum, nur einmal statt mehrfach im Schema
	// ausgeschrieben.
	private sealed record EnumDef(string Namespace, string Name, string? Description, int Size, List<EnumValue> Values);

	// Reiner Daten-Verbund aus ausschliesslich festen Feldtypen (Fixed/Enum/EnumRef/Struct/Repeated,
	// rekursiv). Kein Laufzeit-Header/keine ID (nie eigenstaendig verschickt, immer eingebettet).
	private sealed record StructDef(string Namespace, string Name, string? Description, List<FieldBase> Fields);

	// Wie eine Message (Name+Felder), aber nur als Array-/ClassField-Element-Typ verwendbar, kein
	// eigener Nachrichten-Header. Darf im Gegensatz zu Struct auch StringField und (als letztes
	// Feld) ein weiteres ClassField enthalten, aber KEIN Array (weder FixedArrayField noch
	// ClassArrayField) -- das ist die Umsetzung von "ein Array darf nur in einer Message stehen,
	// nicht rekursiv in den Objekten eines Arrays". CLASS_ID ist global (nicht pro Namespace)
	// eindeutig, weil eine einzelne Klassenliste Klassen aus verschiedenen Namespaces mischen kann
	// und der Laufzeit-Discriminator (2 Byte vor jedem Element) allein eindeutig sein muss.
	private sealed record ClassDef(string Namespace, string Name, string? Description, List<FieldBase> Fields, int Id);

	private sealed record Message(string Namespace, string Name, int Id, string Kind, string? Description, List<FieldBase> Fields);

	// Vollstaendig aufgeloester Namespace (nach Pass 2). Name=="" ist der reservierte NULL-Namespace
	// (Id immer 0) -- seine Inhalte werden ohne umschliessenden "namespace {}"-Block direkt auf
	// WsProtocol-Ebene (C++) bzw. Modul-Ebene (TS) generiert, s. GenerateCpp/GenerateTs.
	private sealed record NamespaceDef(string Name, int Id, List<EnumDef> Enums, List<StructDef> Structs, List<ClassDef> Classes, List<Message> Messages);

	private static string? GetStringOpt(JsonElement el, string prop) =>
		el.TryGetProperty(prop, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() : null;

	// Punkt-Konvention (s. Kommentar am Dateianfang): "ns.Local" -> (ns, "Local"); "Local" (kein
	// Punkt) -> ("", "Local") -- NULL-Namespace. Gilt einheitlich fuer Deklarationen UND Referenzen
	// (structRef/classes/enumRef) -- deshalb genuegt der rohe, wie im JSON angegebene Name auch
	// direkt als Registry-Key (zwei gleiche Strings meinen immer dasselbe (ns, Local)-Paar).
	private static (string Namespace, string Local) SplitName(string name)
	{
		var dot = name.IndexOf('.');
		return dot < 0 ? ("", name) : (name[..dot], name[(dot + 1)..]);
	}

	// --- ID-Zuordnung (s. docs/websocket-protocol.md Abschnitt 8) ---------------------------------

	// Persistente, von Hand lesbare Zuordnungstabelle Name->ID (ws-protocol/ids.txt). Namespaces,
	// Messages und Classes deklarieren in ihrer *.json KEINE numerische ID -- sie werden
	// ausschliesslich ueber ihren (vollen) Namen identifiziert. Beim Generieren vergibt der
	// Generator fuer jeden neu angetroffenen Namen die naechste freie ID im jeweiligen Zaehlerkreis
	// und haelt die Zuordnung dauerhaft in dieser Datei fest -- IDs bleiben so ueber wiederholte
	// Codegenerierungen (auch bei Schema-Erweiterungen) stabil. Einmal vergebene IDs werden NIE
	// wiederverwendet, auch wenn der zugehoerige Name spaeter entfernt wird (Zeile bleibt stehen) --
	// Wire-Stabilitaet hat Vorrang vor einer luecken losen ID-Folge. Enums/Structs brauchen keine
	// ID (nie eigenstaendig auf dem Wire identifiziert, immer eingebettet/nur zur Compile-Zeit
	// referenziert). Der NULL-Namespace ("") bekommt IMMER Id 0, fest reserviert, nie ueber diese
	// Tabelle vergeben.
	private sealed class IdMap
	{
		private readonly Dictionary<string, int> ids = new(StringComparer.Ordinal);
		private readonly List<string> insertionOrder = new();
		private bool dirty;

		public static IdMap Load(string path)
		{
			var map = new IdMap();
			if (!File.Exists(path)) return map;
			foreach (var rawLine in File.ReadAllLines(path))
			{
				var line = rawLine.Trim();
				if (line.Length == 0 || line.StartsWith('#')) continue;
				var lastSpace = line.LastIndexOf(' ');
				if (lastSpace < 0 || !int.TryParse(line[(lastSpace + 1)..], out var id))
				{
					throw new InvalidOperationException($"ws-protocol: ungueltige Zeile in {path}: \"{rawLine}\" (erwartet \"<kind> <name> <id>\").");
				}
				var key = line[..lastSpace];
				if (map.ids.ContainsKey(key))
				{
					throw new InvalidOperationException($"ws-protocol: doppelter Eintrag \"{key}\" in {path}.");
				}
				map.ids[key] = id;
				map.insertionOrder.Add(key);
			}
			return map;
		}

		public void SaveIfDirty(string path)
		{
			if (!dirty) return;
			Directory.CreateDirectory(Path.GetDirectoryName(path)!);
			var lines = new List<string>
			{
				"# ws-protocol ID-Lookup-Tabelle -- automatisch von ReadWebSocketProtocol.cs gepflegt.",
				"# NICHT von Hand umsortieren/umnummerieren. Neue Zeilen werden beim naechsten Generieren",
				"# automatisch angehaengt; bestehende Zeilen NIE geaendert oder entfernt (s. Kommentar bei",
				"# 'class IdMap' im Generator) -- das haelt die Wire-IDs ueber die Zeit stabil.",
				"# Format je Zeile: <kind> <name> <id>",
			};
			lines.AddRange(insertionOrder.Select(key => $"{key} {ids[key]}"));
			File.WriteAllLines(path, lines);
		}

		private int NextIdFor(string kindPrefix) =>
			ids.Where(kv => kv.Key.StartsWith(kindPrefix, StringComparison.Ordinal)).Select(kv => kv.Value).DefaultIfEmpty(0).Max() + 1;

		// Nur Message-Keys OHNE Punkt im Namensteil (also "message X", nicht "message ns.X") --
		// s. Kommentar bei GetOrAssignMessage.
		private int NextIdForNullNamespaceMessages() =>
			ids.Where(kv => kv.Key.StartsWith("message ", StringComparison.Ordinal) && !kv.Key["message ".Length..].Contains('.'))
				.Select(kv => kv.Value).DefaultIfEmpty(0).Max() + 1;

		public int GetOrAssignNamespace(string namespaceName)
		{
			if (namespaceName.Length == 0) return 0; // NULL-Namespace: fest reserviert, nie ueber die Tabelle vergeben.
			var key = $"namespace {namespaceName}";
			if (ids.TryGetValue(key, out var existing)) return existing;
			var id = NextIdFor("namespace ");
			ids[key] = id;
			insertionOrder.Add(key);
			dirty = true;
			return id;
		}

		// Message-IDs sind pro Namespace vergeben (nur innerhalb ihres Namespace eindeutig noetig --
		// s. Wire-Format: namespaceId+messageTypeId zusammen identifizieren eine Nachricht). Fuer den
		// NULL-Namespace (ns=="") reicht der Praefix "message " NICHT aus, um nur dessen Nachrichten
		// zu treffen -- das ist ein Praefix von JEDEM Message-Key (auch "message sensact.Foo"), daher
		// eigener Filter, der nur Keys ohne Punkt im Namensteil zaehlt.
		public int GetOrAssignMessage(string fullMessageName)
		{
			var key = $"message {fullMessageName}";
			if (ids.TryGetValue(key, out var existing)) return existing;
			var (ns, _) = SplitName(fullMessageName);
			var id = ns.Length == 0 ? NextIdForNullNamespaceMessages() : NextIdFor($"message {ns}.");
			ids[key] = id;
			insertionOrder.Add(key);
			dirty = true;
			return id;
		}

		// Class-IDs sind GLOBAL vergeben (ueber alle Namespaces hinweg ein einziger Zaehlerkreis) --
		// eine Klassenliste (ClassArrayField/ClassField) kann Klassen aus verschiedenen Namespaces
		// mischen, der 2-Byte-Discriminator vor jedem Element muss ohne Namespace-Praefix eindeutig
		// sein.
		public int GetOrAssignClass(string fullClassName)
		{
			var key = $"class {fullClassName}";
			if (ids.TryGetValue(key, out var existing)) return existing;
			var id = NextIdFor("class ");
			ids[key] = id;
			insertionOrder.Add(key);
			dirty = true;
			return id;
		}
	}

	// --- Pass 1: alle Dateien roh einlesen (nur rohes JSON, noch keine Feld-Aufloesung) ------------

	private sealed record RawFile(JsonElement Root, string FilePath);

	private static RawFile ParseFileRaw(string path)
	{
		using var doc = JsonDocument.Parse(File.ReadAllText(path));
		// .Clone(): das JsonDocument wird am Methodenende disposed, das zurueckgegebene RootElement
		// muss aber ueber die gesamte Pass-1/Pass-2-Verarbeitung hinweg gueltig bleiben.
		return new RawFile(doc.RootElement.Clone(), path);
	}

	// --- Pass 2: Felder aufloesen (Enum-/Struct-/Klassen-Referenzen ueber ParseContext gegen die in
	// Pass 1 gefuellten, globalen Registries -- unabhaengig davon, aus welcher Datei/welchem
	// Namespace sie stammen; kein "include" noetig) --------------------------------------------------

	private sealed class ParseContext
	{
		// Key: der rohe, vollstaendige Name wie im JSON angegeben (z.B. "wifimanager.Mac6" oder
		// "Foo" fuer den NULL-Namespace) -- s. SplitName-Kommentar oben.
		public readonly Dictionary<string, JsonElement> EnumJson = new(StringComparer.Ordinal);
		public readonly Dictionary<string, JsonElement> StructJson = new(StringComparer.Ordinal);
		public readonly Dictionary<string, JsonElement> ClassJson = new(StringComparer.Ordinal);
		private readonly Dictionary<string, EnumDef> enumCache = new(StringComparer.Ordinal);
		private readonly Dictionary<string, StructDef> structCache = new(StringComparer.Ordinal);
		private readonly Dictionary<string, ClassDef> classCache = new(StringComparer.Ordinal);
		private readonly HashSet<string> structsBeingResolved = new(StringComparer.Ordinal);
		public readonly IdMap Ids;

		public ParseContext(IdMap ids) => Ids = ids;

		public EnumDef ResolveEnum(string name, string usedFrom)
		{
			if (enumCache.TryGetValue(name, out var cached)) return cached;
			if (!EnumJson.TryGetValue(name, out var json))
			{
				throw new InvalidOperationException(
					$"ws-protocol: {usedFrom} referenziert unbekanntes Enum \"{name}\" -- ist es in einer der " +
					"eingelesenen ws-protocol/*.json-Dateien unter \"enums\" deklariert?");
			}
			var (ns, local) = SplitName(name);
			var sizeType = json.GetProperty("size").GetString()!;
			var size = sizeType switch
			{
				"u8" => 1,
				"u16" => 2,
				"u32" => 4,
				_ => throw new InvalidOperationException($"ws-protocol: Enum \"{name}\": unbekannte \"size\" \"{sizeType}\" (gueltig: u8/u16/u32)."),
			};
			var values = json.GetProperty("values").EnumerateArray()
				.Select(v => new EnumValue(v.GetProperty("name").GetString()!, v.GetProperty("value").GetInt64()))
				.ToList();
			var def = new EnumDef(ns, local, GetStringOpt(json, "description"), size, values);
			enumCache[name] = def;
			return def;
		}

		public StructDef ResolveStruct(string name, string usedFrom)
		{
			if (structCache.TryGetValue(name, out var cached)) return cached;
			if (!StructJson.TryGetValue(name, out var json))
			{
				throw new InvalidOperationException(
					$"ws-protocol: {usedFrom} referenziert unbekannten Struct \"{name}\" -- ist er in einer der " +
					"eingelesenen ws-protocol/*.json-Dateien unter \"structs\" deklariert?");
			}
			if (!structsBeingResolved.Add(name))
			{
				throw new InvalidOperationException($"ws-protocol: zyklische Struct-Referenz ueber \"{name}\" (von {usedFrom} aus).");
			}
			var (ns, local) = SplitName(name);
			var fields = new List<FieldBase>();
			if (json.TryGetProperty("fields", out var fieldsEl))
			{
				foreach (var f in fieldsEl.EnumerateArray())
				{
					var parsed = ParseField(f, $"Struct \"{name}\"", this);
					if (parsed is StringField or FixedArrayField or ClassArrayField or ClassField)
					{
						throw new InvalidOperationException(
							$"ws-protocol: Struct \"{name}\", Feld \"{parsed.Name}\": Structs duerfen nur feste Feldtypen " +
							"(Zahlen/bool/Enum/Struct, ggf. mit \"count\") enthalten, keine Strings/Arrays/Classes.");
					}
					fields.Add(parsed);
				}
			}
			structsBeingResolved.Remove(name);
			var def = new StructDef(ns, local, GetStringOpt(json, "description"), fields);
			structCache[name] = def;
			return def;
		}

		public ClassDef ResolveClass(string name, string usedFrom)
		{
			if (classCache.TryGetValue(name, out var cached)) return cached;
			if (!ClassJson.TryGetValue(name, out var json))
			{
				throw new InvalidOperationException(
					$"ws-protocol: {usedFrom} referenziert unbekannte Klasse \"{name}\" -- ist sie in einer der " +
					"eingelesenen ws-protocol/*.json-Dateien unter \"classes\" deklariert?");
			}
			var (ns, local) = SplitName(name);
			var fields = new List<FieldBase>();
			if (json.TryGetProperty("fields", out var fieldsEl))
			{
				foreach (var f in fieldsEl.EnumerateArray())
				{
					fields.Add(ParseField(f, $"Klasse \"{name}\"", this));
				}
			}
			ValidateNoArraysExceptTrailingClassField(fields, $"Klasse \"{name}\"", allowArrays: false);
			var id = Ids.GetOrAssignClass(name);
			var def = new ClassDef(ns, local, GetStringOpt(json, "description"), fields, id);
			classCache[name] = def;
			return def;
		}
	}

	// Gemeinsame Validierung fuer Message- UND Class-Felder: FixedArrayField/ClassArrayField duerfen
	// generell nur in einer Message vorkommen (in einer Class ueberhaupt nicht, s. allowArrays) --
	// "ein Array darf nur in einem Top-Level-Objekt stehen, nicht rekursiv in den Objekten eines
	// Arrays". ClassField/ClassArrayField (variable Laenge, erst beim Durchlaufen bekannt) muessen
	// zusaetzlich einen zusammenhaengenden Block am ENDE der Felderliste bilden -- sobald eines
	// auftritt, darf kein Feld eines ANDEREN Typs mehr folgen. Mehrere ClassArrayField/ClassField
	// hintereinander sind dagegen erlaubt (z.B. fingerprint.ResponseFingers: scheduleNames + fingers,
	// beides eigene classes-Arrays) -- Encode/Decode laufen ohnehin sequenziell durch alle Felder,
	// es gibt also keinen technischen Grund, mehr als eines zu verbieten.
	private static void ValidateNoArraysExceptTrailingClassField(List<FieldBase> fields, string ownerDescription, bool allowArrays)
	{
		var inTrailingClassBlock = false;
		foreach (var f in fields)
		{
			switch (f)
			{
				case FixedArrayField when !allowArrays:
				case ClassArrayField when !allowArrays:
					throw new InvalidOperationException(
						$"ws-protocol: {ownerDescription}, Feld \"{f.Name}\": Classes duerfen keine Array-Felder " +
						"enthalten -- ein Array darf nur in einer Message stehen, nicht rekursiv in den Objekten eines Arrays.");
				case ClassArrayField:
				case ClassField:
					inTrailingClassBlock = true;
					break;
				default:
					if (inTrailingClassBlock)
					{
						throw new InvalidOperationException(
							$"ws-protocol: {ownerDescription}, Feld \"{f.Name}\": nach einem Array mit \"classes\" oder " +
							"einem einzelnen polymorphen Feld (\"type\":\"class\") duerfen nur noch weitere Felder " +
							"desselben Typs folgen -- sie muessen einen zusammenhaengenden Block am Ende der Felderliste bilden.");
					}
					break;
			}
		}
	}

	// --- Parsing -----------------------------------------------------------------------------------

	private static FieldBase WrapWithCountIfPresent(JsonElement f, string name, string? description, FieldBase element)
	{
		if (!f.TryGetProperty("count", out var countEl)) return element;
		var count = countEl.GetInt32();
		if (element is not (FixedField or EnumField or EnumRefField or StructField))
		{
			throw new InvalidOperationException($"ws-protocol: Feld \"{name}\": \"count\" ist nur fuer feste Skalar-/Enum-/Struct-Feldtypen erlaubt.");
		}
		return new RepeatedField(name, description, element, count);
	}

	private static FieldBase ParseField(JsonElement f, string ownerDescription, ParseContext ctx)
	{
		var name = f.GetProperty("name").GetString()!;
		var description = GetStringOpt(f, "description");
		var type = f.GetProperty("type").GetString()!;

		if (type == "string") return new StringField(name, description);

		if (type == "struct")
		{
			var structRef = f.GetProperty("structRef").GetString()!;
			var field = new StructField(name, description, ctx.ResolveStruct(structRef, $"{ownerDescription}, Feld \"{name}\""));
			return WrapWithCountIfPresent(f, name, description, field);
		}

		if (type == "enumRef")
		{
			var enumRef = f.GetProperty("enumRef").GetString()!;
			var field = new EnumRefField(name, description, ctx.ResolveEnum(enumRef, $"{ownerDescription}, Feld \"{name}\""));
			return WrapWithCountIfPresent(f, name, description, field);
		}

		// Einzelnes polymorphes Feld ("type":"class") -- wie ClassArrayField, aber ohne Liste/Count.
		if (type == "class")
		{
			var classRefs = f.GetProperty("classes").EnumerateArray().Select(c => c.GetString()!).ToList();
			if (classRefs.Count == 0)
			{
				throw new InvalidOperationException($"ws-protocol: {ownerDescription}, Feld \"{name}\": \"classes\" darf nicht leer sein.");
			}
			var classes = classRefs.Select(cr => ctx.ResolveClass(cr, $"{ownerDescription}, Feld \"{name}\"")).ToList();
			return new ClassField(name, description, classes);
		}

		if (type == "array")
		{
			// Heterogenes Array: "classes" statt "elementName"/"fields".
			if (f.TryGetProperty("classes", out var classesEl))
			{
				var classRefs = classesEl.EnumerateArray().Select(c => c.GetString()!).ToList();
				if (classRefs.Count == 0)
				{
					throw new InvalidOperationException($"ws-protocol: {ownerDescription}, Array-Feld \"{name}\": \"classes\" darf nicht leer sein.");
				}
				var classes = classRefs.Select(cr => ctx.ResolveClass(cr, $"{ownerDescription}, Array-Feld \"{name}\"")).ToList();
				return new ClassArrayField(name, description, classes);
			}

			// Homogenes, fest-groesses Array.
			var elementName = f.GetProperty("elementName").GetString()!;
			var elementFields = f.GetProperty("fields").EnumerateArray()
				.Select(ef => ParseField(ef, $"{ownerDescription}, Array-Feld \"{name}\" (Element \"{elementName}\")", ctx))
				.ToList();
			foreach (var ef in elementFields)
			{
				if (ef is StringField or FixedArrayField or ClassArrayField or ClassField)
				{
					throw new InvalidOperationException(
						$"ws-protocol: {ownerDescription}, Array-Feld \"{name}\" (Element \"{elementName}\"): " +
						$"Element-Feld \"{ef.Name}\" ist variabel lang oder selbst ein Array/eine Klasse -- Array-Elemente " +
						"duerfen aktuell nur feste Feldtypen (Zahlen/bool/Enum/Struct) enthalten.");
				}
			}
			return new FixedArrayField(name, description, elementName, elementFields);
		}

		if (type.StartsWith("Enum", StringComparison.Ordinal))
		{
			var size = type switch
			{
				"EnumU8" => 1,
				"EnumU16" => 2,
				"EnumU32" => 4,
				_ => throw new InvalidOperationException(
					$"ws-protocol: {ownerDescription}, Feld \"{name}\": unbekannter Enum-Typ \"{type}\" " +
					"(gueltig: EnumU8/EnumU16/EnumU32)."),
			};
			var enumName = f.GetProperty("enumName").GetString()!;
			var values = f.GetProperty("enumValues").EnumerateArray()
				.Select(v => new EnumValue(v.GetProperty("name").GetString()!, v.GetProperty("value").GetInt64()))
				.ToList();
			var field = new EnumField(name, description, enumName, size, values);
			return WrapWithCountIfPresent(f, name, description, field);
		}

		if (!FixedTypes.TryGetValue(type, out var info))
		{
			throw new InvalidOperationException($"ws-protocol: {ownerDescription}, Feld \"{name}\": unbekannter Feldtyp \"{type}\".");
		}
		var fixedField = new FixedField(name, description, info.CppType, info.TsType, info.Size, info.IsBool, info.IsSigned, info.IsFloat);
		return WrapWithCountIfPresent(f, name, description, fixedField);
	}

	private static Message ParseMessage(JsonElement msg, ParseContext ctx)
	{
		var fullName = msg.GetProperty("name").GetString()!;
		var (ns, local) = SplitName(fullName);
		var kind = msg.GetProperty("kind").GetString()!;
		var fields = new List<FieldBase>();
		if (msg.TryGetProperty("fields", out var fieldsEl))
		{
			foreach (var f in fieldsEl.EnumerateArray())
			{
				fields.Add(ParseField(f, $"Nachricht \"{fullName}\"", ctx));
			}
		}
		ValidateNoArraysExceptTrailingClassField(fields, $"Nachricht \"{fullName}\"", allowArrays: true);
		var id = ctx.Ids.GetOrAssignMessage(fullName);
		return new Message(ns, local, id, kind, GetStringOpt(msg, "description"), fields);
	}

	// requestId (uint16) ist bei request/response-Nachrichten ein IMPLIZITES erstes Feld direkt
	// nach dem 4-Byte-Kopf (s. docs/websocket-protocol.md) -- wird hier synthetisch vor die in der
	// jeweiligen ws-protocol/*.json deklarierten Felder gestellt, statt dass jede request/response-
	// Nachricht es selbst auflisten muesste.
	private static List<FieldBase> EffectiveFields(Message msg)
	{
		var fields = new List<FieldBase>();
		if (msg.Kind is "request" or "response")
		{
			fields.Add(new FixedField("requestId", "Vom Client vergebene Korrelations-ID, in der Response unveraendert zurueckgegeben", "uint16_t", "number", 2, false, false));
		}
		fields.AddRange(msg.Fields);
		return fields;
	}

	private static string KindEnumCpp(string kind) => kind switch
	{
		"event" => "MessageKind::Event",
		"request" => "MessageKind::Request",
		"response" => "MessageKind::Response",
		_ => throw new InvalidOperationException($"ws-protocol: unbekanntes kind \"{kind}\"."),
	};

	private static string KindEnumTs(string kind) => kind switch
	{
		"event" => "MessageKind.Event",
		"request" => "MessageKind.Request",
		"response" => "MessageKind.Response",
		_ => throw new InvalidOperationException($"ws-protocol: unbekanntes kind \"{kind}\"."),
	};

	private static string FieldName(FieldBase f) => f.Name;

	// Rekursiv -- ein StructField traegt die Groesse seines (selbst wieder nur aus festen Feldern
	// bestehenden) Structs, ein RepeatedField die Groesse seines Elements mal Count.
	private static int FixedSizeOf(FieldBase f) => f switch
	{
		FixedField ff => ff.Size,
		EnumField ef => ef.Size,
		EnumRefField erf => erf.Enum.Size,
		StructField sf => sf.Struct.Fields.Sum(FixedSizeOf),
		RepeatedField rf => FixedSizeOf(rf.Element) * rf.Count,
		_ => throw new InvalidOperationException("Nur fuer feste Feldtypen definiert."),
	};

	private const string HeaderComment = """
		// GENERIERT von builder/Phases/ReadWebSocketProtocol.cs aus allen *.json-Dateien in
		// ws-protocol/ -- nicht von Hand editieren. Aenderungen gehoeren in die jeweilige
		// ws-protocol/*.json, anschliessend
		// "dotnet run --project builder -- ReadWebSocketProtocolAndGenerateFiles" erneut ausfuehren.
		// Wire-Format: docs/websocket-protocol.md.
		""";

	// --- C++ ---------------------------------------------------------------------------------

	private static string CppEnumUnderlyingType(int size) => size switch
	{
		1 => "uint8_t",
		2 => "uint16_t",
		4 => "uint32_t",
		_ => throw new InvalidOperationException(),
	};

	// Voll qualifizierter C++-Name relativ zum umschliessenden "namespace WsProtocol { ... }" --
	// NULL-Namespace-Typen (Namespace=="") liegen DIREKT in WsProtocol, ein unqualifizierter Name
	// findet sie ueberall via normaler C++-Scope-Aufloesung (auesserer Scope), deshalb ohne "::"-
	// Praefix.
	private static string CppQualify(string ns, string name) => ns.Length == 0 ? name : $"{ns}::{name}";
	private static string CppStructTypeName(StructDef s) => CppQualify(s.Namespace, s.Name);
	private static string CppEnumRefTypeName(EnumDef e) => CppQualify(e.Namespace, e.Name);
	private static string CppClassQualifiedName(ClassDef c) => CppQualify(c.Namespace, c.Name);

	private static string CppElementTypeName(FieldBase f) => f switch
	{
		FixedField ff => ff.CppType,
		EnumField ef => ef.EnumName,
		EnumRefField erf => CppEnumRefTypeName(erf.Enum),
		StructField sf => CppStructTypeName(sf.Struct),
		_ => throw new InvalidOperationException("RepeatedField: unerwarteter Elementtyp."),
	};

	private static string CppFieldDecl(FieldBase f) => f switch
	{
		FixedField ff => $"    {ff.CppType} {ff.Name};" + (ff.Description is not null ? $" // {ff.Description}" : "") + "\n",
		EnumField ef => $"    {ef.EnumName} {ef.Name};" + (ef.Description is not null ? $" // {ef.Description}" : "") + "\n",
		EnumRefField erf => $"    {CppEnumRefTypeName(erf.Enum)} {erf.Name};" + (erf.Description is not null ? $" // {erf.Description}" : "") + "\n",
		StructField sf => $"    {CppStructTypeName(sf.Struct)} {sf.Name};" + (sf.Description is not null ? $" // {sf.Description}" : "") + "\n",
		RepeatedField rf => $"    {CppElementTypeName(rf.Element)} {rf.Name}[{rf.Count}];" + (rf.Description is not null ? $" // {rf.Description}" : "") + "\n",
		StringField sf => $"    const char* {sf.Name}; // UTF-8, NICHT '\\0'-terminiert" +
			(sf.Description is not null ? $" -- {sf.Description}" : "") + $"\n    size_t {sf.Name}Length;\n",
		FixedArrayField af => $"    const uint8_t* {af.Name}Data; // rohe {af.ElementName}-Elemente, s. Decode{af.ElementName}At()" +
			(af.Description is not null ? $" -- {af.Description}" : "") + $"\n    size_t {af.Name}Count;\n",
		ClassArrayField caf => $"    const uint8_t* {caf.Name}Data; // vorserialisierte Elemente (je [classId:u16][Klassenfelder]), s. Append*Element-Funktionen" +
			(caf.Description is not null ? $" -- {caf.Description}" : "") + $"\n    size_t {caf.Name}Count;\n    size_t {caf.Name}DataSize; // Gesamtbytes von {caf.Name}Data\n",
		ClassField clf => $"    const uint8_t* {clf.Name}Data; // vorserialisiertes Element (classId:u16 + Klassenfelder), s. Append*Element-Funktionen" +
			(clf.Description is not null ? $" -- {clf.Description}" : "") + $"\n    size_t {clf.Name}DataSize;\n",
		_ => throw new InvalidOperationException(),
	};

	// Akkumulator-Typ fuer die Bitshift-basierte Kodierung -- MUSS fuer 8-Byte-Felder (int64/uint64)
	// uint64_t sein, sonst wuerde entweder vor dem ersten Shift schon auf 32 Bit abgeschnitten
	// (Encode) oder ein Shift um >=32 Bit auf einem 32-Bit-Typ ausgefuehrt (Decode, undefiniertes
	// Verhalten in C++). Fuer alle anderen Groessen bleibt uint32_t ausreichend und wird beibehalten,
	// um an den generierten Code fuer bestehende Faelle nichts zu aendern.
	private static string CppAccumulatorType(int size) => size > 4 ? "uint64_t" : "uint32_t";

	// Schreibt EIN festes (Fixed/Enum/EnumRef) Skalarelement an Cursor "pos". "valueExpr" ist der
	// ROHE Ausdruck (KEIN vorab durchgefuehrter (uint32_t)-Cast durch den Aufrufer!) -- der Cast auf
	// den passenden (groessenabhaengigen) Akkumulator-Typ passiert hier zentral.
	private static string CppEncodeScalar(string valueExpr, int size)
	{
		var castExpr = $"({CppAccumulatorType(size)})({valueExpr})";
		var sb = new StringBuilder();
		sb.Append($"    if (pos + {size} > dest_size) return 0;\n");
		if (size == 1) sb.Append($"    dest[pos++] = (uint8_t){castExpr};\n");
		else for (var i = 0; i < size; i++) sb.Append($"    dest[pos++] = (uint8_t)(({castExpr}) >> {i * 8});\n");
		return sb.ToString();
	}

	private static string CppDecodeScalarInto(string targetExpr, string castType, int size)
	{
		var accType = CppAccumulatorType(size);
		var sb = new StringBuilder();
		sb.Append($"    if (len < pos + {size}) return false;\n");
		if (size == 1)
		{
			sb.Append($"    {targetExpr} = ({castType})data[pos];\n");
		}
		else
		{
			var terms = string.Join(" | ", Enumerable.Range(0, size).Select(i => i == 0 ? $"({accType})data[pos]" : $"(({accType})data[pos + {i}] << {i * 8})"));
			sb.Append($"    {targetExpr} = ({castType})({terms});\n");
		}
		sb.Append($"    pos += {size};\n");
		return sb.ToString();
	}

	// Float-Felder: Bit-Muster per memcpy uebertragen (NICHT wie CppEncodeScalar per
	// Integer-Bitshift-Cast, s. Kommentar bei FixedTypeInfo.IsFloat). Setzt little-endian voraus --
	// gilt fuer alle in diesem Oekosystem eingesetzten Zielplattformen (ESP32, STM32, x86/ARM64 als
	// Entwicklungsrechner), exakt dieselbe Annahme, die der Rest des Protokolls (Multi-Byte-Felder
	// per Bitshift in LE-Reihenfolge) ohnehin schon macht.
	private static string CppEncodeFloatBytes(string valueExpr, string cppType, int size)
	{
		return
			$"    if (pos + {size} > dest_size) return 0;\n" +
			$"    {{ {cppType} tmp_ = {valueExpr}; memcpy(dest + pos, &tmp_, {size}); pos += {size}; }}\n";
	}

	private static string CppDecodeFloatInto(string targetExpr, int size)
	{
		return
			$"    if (len < pos + {size}) return false;\n" +
			$"    memcpy(&{targetExpr}, data + pos, {size});\n" +
			$"    pos += {size};\n";
	}

	private static string Indent(string block, string prefix) =>
		string.Concat(block.Split('\n').Select(line => line.Length == 0 ? line : prefix + line + "\n")).TrimEnd('\n') + "\n";

	// Schreibt EIN Feld an Cursor "pos" in "dest" (fuer Message-/Class-/Struct-Payloads gleichermassen
	// verwendbar -- der generierte Code drumherum deklariert stets lokale Variablen "pos"/"dest"/
	// "dest_size" mit passender Bedeutung).
	private static string CppEncodeField(FieldBase f)
	{
		switch (f)
		{
			case FixedField ff when ff.IsFloat:
				return CppEncodeFloatBytes($"payload.{ff.Name}", ff.CppType, ff.Size);
			case FixedField ff:
				// Kein vorab durchgefuehrter (uint32_t)-Cast hier -- CppEncodeScalar waehlt den
				// Akkumulator-Typ selbst passend zur Feldgroesse (wichtig fuer 8-Byte-Felder
				// wie uint64/int64, s. CppAccumulatorType).
				return CppEncodeScalar(ff.IsBool ? $"(payload.{ff.Name} ? 1u : 0u)" : $"payload.{ff.Name}", ff.Size);
			case EnumField ef:
				return CppEncodeScalar($"(uint32_t)payload.{ef.Name}", ef.Size);
			case EnumRefField erf:
				return CppEncodeScalar($"(uint32_t)payload.{erf.Name}", erf.Enum.Size);
			case StructField sf:
			{
				var t = CppStructTypeName(sf.Struct);
				return
					$"    {{\n" +
					$"        size_t newPos = {t}Encode(payload.{sf.Name}, dest, pos, dest_size);\n" +
					$"        if (newPos == 0) return 0;\n" +
					$"        pos = newPos;\n" +
					$"    }}\n";
			}
			case RepeatedField rf:
			{
				var elemSize = FixedSizeOf(rf.Element);
				var sb = new StringBuilder();
				sb.Append($"    if (pos + {elemSize} * {rf.Count} > dest_size) return 0;\n");
				sb.Append($"    for (size_t i = 0; i < {rf.Count}; i++) {{\n");
				switch (rf.Element)
				{
					case FixedField ff when ff.IsFloat:
						sb.Append(Indent(CppEncodeFloatBytes($"payload.{rf.Name}[i]", ff.CppType, ff.Size), "        "));
						break;
					case FixedField ff:
						sb.Append(Indent(CppEncodeScalar(ff.IsBool ? $"(payload.{rf.Name}[i] ? 1u : 0u)" : $"payload.{rf.Name}[i]", ff.Size), "        "));
						break;
					case EnumField ef:
						sb.Append(Indent(CppEncodeScalar($"(uint32_t)payload.{rf.Name}[i]", ef.Size), "        "));
						break;
					case EnumRefField erf:
						sb.Append(Indent(CppEncodeScalar($"(uint32_t)payload.{rf.Name}[i]", erf.Enum.Size), "        "));
						break;
					case StructField sf:
						sb.Append($"        size_t newPos = {CppStructTypeName(sf.Struct)}Encode(payload.{rf.Name}[i], dest, pos, dest_size);\n");
						sb.Append("        if (newPos == 0) return 0;\n");
						sb.Append("        pos = newPos;\n");
						break;
				}
				sb.Append("    }\n");
				return sb.ToString();
			}
			case StringField sf:
				return
					$"    if (pos + 4 + payload.{sf.Name}Length > dest_size) return 0;\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{sf.Name}Length >> 0);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{sf.Name}Length >> 8);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{sf.Name}Length >> 16);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{sf.Name}Length >> 24);\n" +
					$"    if (payload.{sf.Name}Length > 0) {{ memcpy(dest + pos, payload.{sf.Name}, payload.{sf.Name}Length); pos += payload.{sf.Name}Length; }}\n";
			case FixedArrayField af:
				return
					$"    if (pos + 4 + payload.{af.Name}Count * {af.ElementName}_SIZE > dest_size) return 0;\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{af.Name}Count >> 0);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{af.Name}Count >> 8);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{af.Name}Count >> 16);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{af.Name}Count >> 24);\n" +
					$"    if (payload.{af.Name}Count > 0) {{ memcpy(dest + pos, payload.{af.Name}Data, payload.{af.Name}Count * {af.ElementName}_SIZE); pos += payload.{af.Name}Count * {af.ElementName}_SIZE; }}\n";
			case ClassArrayField caf:
				// Elemente sind bereits vorserialisiert (je classId+Klassenfelder) -- s.
				// Append<Owner><Feld><Klasse>Element()-Hilfsfunktionen. Muss das letzte Feld sein.
				return
					$"    if (pos + 4 + payload.{caf.Name}DataSize > dest_size) return 0;\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{caf.Name}Count >> 0);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{caf.Name}Count >> 8);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{caf.Name}Count >> 16);\n" +
					$"    dest[pos++] = (uint8_t)((uint32_t)payload.{caf.Name}Count >> 24);\n" +
					$"    if (payload.{caf.Name}DataSize > 0) {{ memcpy(dest + pos, payload.{caf.Name}Data, payload.{caf.Name}DataSize); pos += payload.{caf.Name}DataSize; }}\n";
			case ClassField clf:
				// Wie ClassArrayField, aber ohne Count-Praefix -- genau ein vorserialisiertes Element.
				return
					$"    if (pos + payload.{clf.Name}DataSize > dest_size) return 0;\n" +
					$"    if (payload.{clf.Name}DataSize > 0) {{ memcpy(dest + pos, payload.{clf.Name}Data, payload.{clf.Name}DataSize); pos += payload.{clf.Name}DataSize; }}\n";
			default:
				throw new InvalidOperationException();
		}
	}

	private static string CppDecodeField(FieldBase f)
	{
		switch (f)
		{
			case FixedField ff when ff.IsBool:
				return $"    if (len < pos + 1) return false;\n    out.{ff.Name} = data[pos] != 0;\n    pos += 1;\n";
			case FixedField ff when ff.IsFloat:
				return CppDecodeFloatInto($"out.{ff.Name}", ff.Size);
			case FixedField ff:
				return CppDecodeScalarInto($"out.{ff.Name}", ff.CppType, ff.Size);
			case EnumField ef:
				return CppDecodeScalarInto($"out.{ef.Name}", ef.EnumName, ef.Size);
			case EnumRefField erf:
				return CppDecodeScalarInto($"out.{erf.Name}", CppEnumRefTypeName(erf.Enum), erf.Enum.Size);
			case StructField sf:
			{
				var t = CppStructTypeName(sf.Struct);
				return $"    if (!{t}Decode(data, len, pos, out.{sf.Name})) return false;\n";
			}
			case RepeatedField rf:
			{
				var elemSize = FixedSizeOf(rf.Element);
				var sb = new StringBuilder();
				sb.Append($"    if (len < pos + {elemSize} * {rf.Count}) return false;\n");
				sb.Append($"    for (size_t i = 0; i < {rf.Count}; i++) {{\n");
				switch (rf.Element)
				{
					case FixedField ff when ff.IsBool:
						sb.Append($"        out.{rf.Name}[i] = data[pos] != 0;\n        pos += 1;\n");
						break;
					case FixedField ff when ff.IsFloat:
						sb.Append(Indent(CppDecodeFloatInto($"out.{rf.Name}[i]", ff.Size), "        "));
						break;
					case FixedField ff:
						sb.Append(Indent(CppDecodeScalarInto($"out.{rf.Name}[i]", ff.CppType, ff.Size), "        "));
						break;
					case EnumField ef:
						sb.Append(Indent(CppDecodeScalarInto($"out.{rf.Name}[i]", ef.EnumName, ef.Size), "        "));
						break;
					case EnumRefField erf:
						sb.Append(Indent(CppDecodeScalarInto($"out.{rf.Name}[i]", CppEnumRefTypeName(erf.Enum), erf.Enum.Size), "        "));
						break;
					case StructField sf:
						sb.Append($"        if (!{CppStructTypeName(sf.Struct)}Decode(data, len, pos, out.{rf.Name}[i])) return false;\n");
						break;
				}
				sb.Append("    }\n");
				return sb.ToString();
			}
			case StringField sf:
				return
					$"    if (len < pos + 4) return false;\n" +
					"    {\n" +
					$"        uint32_t {sf.Name}Len = (uint32_t)data[pos] | ((uint32_t)data[pos + 1] << 8) | ((uint32_t)data[pos + 2] << 16) | ((uint32_t)data[pos + 3] << 24);\n" +
					"        pos += 4;\n" +
					$"        if (len < pos + {sf.Name}Len) return false;\n" +
					$"        out.{sf.Name} = (const char*)(data + pos);\n" +
					$"        out.{sf.Name}Length = {sf.Name}Len;\n" +
					$"        pos += {sf.Name}Len;\n" +
					"    }\n";
			case FixedArrayField af:
				// Ueberlaufschutz: Count kommt roh vom Client, Count*ELEMENT_SIZE koennte sonst als
				// size_t umlaufen und die Laengenpruefung darunter umgehen.
				return
					$"    if (len < pos + 4) return false;\n" +
					"    {\n" +
					$"        uint32_t {af.Name}Count_ = (uint32_t)data[pos] | ((uint32_t)data[pos + 1] << 8) | ((uint32_t)data[pos + 2] << 16) | ((uint32_t)data[pos + 3] << 24);\n" +
					"        pos += 4;\n" +
					$"        if ((uint64_t){af.Name}Count_ * {af.ElementName}_SIZE > (uint64_t)(len - pos)) return false;\n" +
					$"        out.{af.Name}Data = data + pos;\n" +
					$"        out.{af.Name}Count = {af.Name}Count_;\n" +
					$"        pos += (size_t){af.Name}Count_ * {af.ElementName}_SIZE;\n" +
					"    }\n";
			case ClassArrayField caf:
				// Muss das letzte Feld sein: die exakte Gesamtbytelaenge der Elemente ist ohne sie
				// einzeln zu dekodieren nicht bekannt -- "DataSize" ist deshalb nur eine konservative
				// Obergrenze ("Rest des Frames"), keine exakte Groesse. Tatsaechliches Dekodieren
				// erst bei Bedarf im Aufrufer via Decode<Owner><Feld>Elements().
				return
					$"    if (len < pos + 4) return false;\n" +
					"    {\n" +
					$"        uint32_t {caf.Name}Count_ = (uint32_t)data[pos] | ((uint32_t)data[pos + 1] << 8) | ((uint32_t)data[pos + 2] << 16) | ((uint32_t)data[pos + 3] << 24);\n" +
					"        pos += 4;\n" +
					$"        out.{caf.Name}Count = {caf.Name}Count_;\n" +
					$"        out.{caf.Name}Data = data + pos;\n" +
					$"        out.{caf.Name}DataSize = len - pos;\n" +
					"        pos = len;\n" +
					"    }\n";
			case ClassField clf:
				// Wie ClassArrayField, aber ohne Count-Praefix -- "Rest des Frames" ist auch hier nur
				// eine konservative Obergrenze, tatsaechliches Dekodieren via Decode<Owner><Feld>Elements()
				// mit count=1 im Aufrufer.
				return
					$"    out.{clf.Name}Data = data + pos;\n" +
					$"    out.{clf.Name}DataSize = len - pos;\n" +
					"    pos = len;\n";
			default:
				throw new InvalidOperationException();
		}
	}

	private static string CppSizeTerm(FieldBase f) => f switch
	{
		FixedField ff => $"{ff.Size}",
		EnumField ef => $"{ef.Size}",
		EnumRefField erf => $"{erf.Enum.Size}",
		StructField sf => $"{CppStructTypeName(sf.Struct)}_SIZE",
		RepeatedField rf => $"({FixedSizeOf(rf.Element)} * {rf.Count})",
		StringField sf => $"4 + payload.{sf.Name}Length",
		FixedArrayField af => $"4 + payload.{af.Name}Count * {af.ElementName}_SIZE",
		ClassArrayField caf => $"4 + payload.{caf.Name}DataSize",
		ClassField clf => $"payload.{clf.Name}DataSize",
		_ => throw new InvalidOperationException(),
	};

	private static string GenerateCppEnum(EnumField ef) =>
		$"enum class {ef.EnumName} : {CppEnumUnderlyingType(ef.Size)} {{ " +
		string.Join(", ", ef.Values.Select(v => $"{v.Name} = {v.Value}")) +
		" };\n\n";

	// Wiederverwendbares, benanntes Enum (namespace-level "enums") -- im Gegensatz zu
	// GenerateCppEnum(EnumField) ohne umschliessende Einrueckung/Kontext, weil es auf
	// Namespace-Ebene (nicht innerhalb einer Message/eines Structs) generiert wird.
	private static string GenerateCppNamedEnum(EnumDef e)
	{
		var sb = new StringBuilder();
		if (e.Description is not null) sb.Append($"// {e.Description}\n");
		sb.Append($"enum class {e.Name} : {CppEnumUnderlyingType(e.Size)} {{ ");
		sb.Append(string.Join(", ", e.Values.Select(v => $"{v.Name} = {v.Value}")));
		sb.Append(" };\n\n");
		return sb.ToString();
	}

	// Struct: reiner Verbund fester Felder, ohne Kopf/ID. StructName_SIZE ist zur Compile-Zeit
	// bekannt (rekursiv aus den Groessen der Mitgliedsfelder). Encode/Decode nehmen "pos" als
	// Ein-/Ausgabeparameter (nicht wie bei Message stets bei 0/4 startend), weil ein Struct an
	// beliebiger Cursor-Position innerhalb einer Message/Class/eines anderen Structs eingebettet
	// sein kann.
	private static string GenerateCppStruct(StructDef s)
	{
		var sb = new StringBuilder();
		if (s.Description is not null) sb.Append($"// {s.Description}\n");
		sb.Append($"struct {s.Name} {{\n");
		foreach (var f in s.Fields) sb.Append(CppFieldDecl(f));
		sb.Append("};\n\n");

		var totalSize = s.Fields.Sum(FixedSizeOf);
		sb.Append($"constexpr size_t {s.Name}_SIZE = {totalSize};\n\n");

		foreach (var ef in s.Fields.OfType<EnumField>()) sb.Append(GenerateCppEnum(ef));

		sb.Append($"inline size_t {s.Name}Encode(const {s.Name}& payload, uint8_t* dest, size_t pos, size_t dest_size) {{\n");
		foreach (var f in s.Fields) sb.Append(CppEncodeField(f));
		sb.Append("    return pos;\n}\n\n");

		sb.Append($"inline bool {s.Name}Decode(const uint8_t* data, size_t len, size_t& pos, {s.Name}& out) {{\n");
		foreach (var f in s.Fields) sb.Append(CppDecodeField(f));
		sb.Append("    return true;\n}\n\n");

		return sb.ToString();
	}

	// Class: wie eine Message (Payload-Struct + Encode/Decode), aber ohne NAMESPACE_ID/TYPE_ID-Kopf
	// -- wird nie eigenstaendig ueber den WebSocket verschickt, sondern nur als Element eines
	// ClassArrayField/ClassField. CLASS_ID ist der global eindeutige 2-Byte-Discriminator.
	private static string GenerateCppClass(ClassDef c)
	{
		var sb = new StringBuilder();
		if (c.Description is not null) sb.Append($"// {c.Description}\n");
		sb.Append($"namespace {c.Name} {{\n");
		sb.Append($"constexpr uint16_t CLASS_ID = {c.Id};\n\n");

		foreach (var ef in c.Fields.OfType<EnumField>()) sb.Append(GenerateCppEnum(ef));
		// Eine Class darf (als letztes Feld) selbst ein ClassField tragen (z.B. "Schedule" mit einem
		// abschliessenden polymorphen "schedule"-Feld) -- braucht dieselben Append*/Decode*Elements-
		// Hilfsfunktionen wie ein ClassArrayField/ClassField einer Message (s. GenerateCppMessage).
		// FixedArrayField/ClassArrayField sind hier per Validierung ausgeschlossen (s.
		// ValidateNoArraysExceptTrailingClassField), daher nur ClassField zu behandeln.
		foreach (var clf in c.Fields.OfType<ClassField>()) sb.Append(GenerateCppClassTaggedHelpers(c.Name, clf.Name, clf.Classes));

		sb.Append("struct Payload {\n");
		foreach (var f in c.Fields) sb.Append(CppFieldDecl(f));
		sb.Append("};\n\n");

		sb.Append("inline size_t Encode(const Payload& payload, uint8_t* dest, size_t pos, size_t dest_size) {\n");
		foreach (var f in c.Fields) sb.Append(CppEncodeField(f));
		sb.Append("    return pos;\n}\n\n");

		sb.Append("inline bool Decode(const uint8_t* data, size_t len, size_t& pos, Payload& out) {\n");
		foreach (var f in c.Fields) sb.Append(CppDecodeField(f));
		sb.Append("    return true;\n}\n\n");

		sb.Append($"}} // namespace {c.Name}\n\n");
		return sb.ToString();
	}

	// Array-Elemente sind auf feste Feldtypen beschraenkt (s. ParseField) -- deshalb hat jedes
	// Element eine zur Compile-Zeit bekannte Groesse (ELEMENT_SIZE) und kann per einfacher
	// Index-Multiplikation adressiert werden, ohne einen fortlaufenden Cursor durch alle
	// vorherigen Elemente zu fuehren.
	private static string GenerateCppElementStruct(FixedArrayField af)
	{
		var sb = new StringBuilder();
		sb.Append($"struct {af.ElementName} {{\n");
		foreach (var ef in af.ElementFields) sb.Append(CppFieldDecl(ef));
		sb.Append("};\n\n");

		var totalSize = af.ElementFields.Sum(FixedSizeOf);
		sb.Append($"constexpr size_t {af.ElementName}_SIZE = {totalSize};\n\n");

		foreach (var ef in af.ElementFields.OfType<EnumField>()) sb.Append(GenerateCppEnum(ef));

		sb.Append($"inline size_t Encode{af.ElementName}(const {af.ElementName}& payload, uint8_t* dest, size_t dest_size) {{\n");
		sb.Append("    size_t pos = 0;\n");
		foreach (var ef in af.ElementFields) sb.Append(CppEncodeField(ef));
		sb.Append("    return pos;\n}\n\n");

		sb.Append($"inline bool Decode{af.ElementName}At(const uint8_t* base_, size_t index, {af.ElementName}& out) {{\n");
		sb.Append($"    const uint8_t* data = base_ + index * {af.ElementName}_SIZE;\n");
		sb.Append($"    size_t len = {af.ElementName}_SIZE;\n");
		sb.Append("    size_t pos = 0;\n");
		foreach (var ef in af.ElementFields) sb.Append(CppDecodeField(ef));
		sb.Append("    return true;\n}\n\n");

		return sb.ToString();
	}

	private static string Capitalize(string s) => s.Length == 0 ? s : char.ToUpperInvariant(s[0]) + s[1..];

	// Fuer ein ClassArrayField/ClassField werden generiert: je zulaessiger Klasse eine
	// "Append<Owner><Feld><Klasse>Element"-Hilfsfunktion (Sender-Seite: haengt [classId][Klassenfelder]
	// an einen Scratch-Puffer an), sowie EINE "Decode<Owner><Feld>Elements"-Visitor-Funktion
	// (Empfaenger-Seite: laeuft die Elemente sequentiell ab -- fuer ClassField immer mit count=1
	// aufgerufen). Ein Visitor-Callback (Template statt materialisierter Ergebnisliste) vermeidet
	// Heap-Allokation und einen zusaetzlichen Variant-/Union-Typ.
	private static string GenerateCppClassTaggedHelpers(string ownerName, string fieldName, List<ClassDef> classes)
	{
		var baseName = $"{ownerName}{Capitalize(fieldName)}";
		var sb = new StringBuilder();

		foreach (var c in classes)
		{
			var qc = CppClassQualifiedName(c);
			sb.Append($"inline size_t Append{baseName}{c.Name}Element(const {qc}::Payload& item, uint8_t* dest, size_t pos, size_t dest_size) {{\n");
			sb.Append("    if (pos + 2 > dest_size) return 0;\n");
			sb.Append($"    dest[pos++] = (uint8_t)({qc}::CLASS_ID & 0xFF);\n");
			sb.Append($"    dest[pos++] = (uint8_t)({qc}::CLASS_ID >> 8);\n");
			sb.Append($"    size_t newPos = {qc}::Encode(item, dest, pos, dest_size);\n");
			sb.Append("    return newPos;\n");
			sb.Append("}\n\n");
		}

		sb.Append($"// Laeuft {fieldName}-Element(e) von 'data' (Laenge 'dataSize') sequentiell ab und ruft 'visitor'\n");
		sb.Append("// je nach vorangestellter classId mit dem passenden dekodierten Klassen-Payload auf. Gibt false\n");
		sb.Append("// bei unbekannter classId oder einem zu kurzen/inkonsistenten Element zurueck.\n");
		sb.Append($"template <typename Visitor>\n");
		sb.Append($"inline bool Decode{baseName}Elements(const uint8_t* data, size_t dataSize, size_t count, Visitor&& visitor) {{\n");
		sb.Append("    size_t pos = 0;\n");
		sb.Append("    for (size_t i = 0; i < count; i++) {\n");
		sb.Append("        if (pos + 2 > dataSize) return false;\n");
		sb.Append("        uint16_t classId = (uint16_t)((uint32_t)data[pos] | ((uint32_t)data[pos + 1] << 8));\n");
		sb.Append("        pos += 2;\n");
		sb.Append("        switch (classId) {\n");
		foreach (var c in classes)
		{
			var qc = CppClassQualifiedName(c);
			sb.Append($"        case {qc}::CLASS_ID: {{\n");
			sb.Append($"            {qc}::Payload item{{}};\n");
			sb.Append($"            if (!{qc}::Decode(data, dataSize, pos, item)) return false;\n");
			sb.Append("            visitor(item);\n");
			sb.Append("            break;\n");
			sb.Append("        }\n");
		}
		sb.Append("        default: return false;\n");
		sb.Append("        }\n");
		sb.Append("    }\n");
		sb.Append("    return true;\n");
		sb.Append("}\n\n");

		return sb.ToString();
	}

	private static string GenerateCppMessage(Message msg)
	{
		var fields = EffectiveFields(msg);
		var sb = new StringBuilder();
		if (msg.Description is not null) sb.Append($"// {msg.Description}\n");
		sb.Append($"namespace {msg.Name} {{\n");
		sb.Append($"constexpr uint16_t TYPE_ID = {msg.Id};\n");
		sb.Append($"constexpr MessageKind KIND = {KindEnumCpp(msg.Kind)};\n\n");

		foreach (var ef in fields.OfType<EnumField>()) sb.Append(GenerateCppEnum(ef));
		foreach (var af in fields.OfType<FixedArrayField>()) sb.Append(GenerateCppElementStruct(af));
		foreach (var caf in fields.OfType<ClassArrayField>()) sb.Append(GenerateCppClassTaggedHelpers(msg.Name, caf.Name, caf.Classes));
		foreach (var clf in fields.OfType<ClassField>()) sb.Append(GenerateCppClassTaggedHelpers(msg.Name, clf.Name, clf.Classes));

		sb.Append("struct Payload {\n");
		foreach (var f in fields) sb.Append(CppFieldDecl(f));
		sb.Append("};\n\n");

		sb.Append("// Minimal benoetigte Puffergroesse fuer Encode() bei den AKTUELL in payload gesetzten\n");
		sb.Append("// String-/Array-Laengen (haengt vom jeweiligen Aufruf ab, keine Compile-Zeit-Konstante).\n");
		sb.Append("inline size_t EncodedSize(const Payload& payload) {\n");
		sb.Append("    return 4" + string.Concat(fields.Select(f => $" + ({CppSizeTerm(f)})")) + ";\n");
		sb.Append("}\n\n");

		sb.Append("// Schreibt Kopf+Payload nach 'dest' (s. EncodedSize() fuer die benoetigte Mindestgroesse).\n");
		sb.Append("// Gibt die geschriebene Gesamtlaenge zurueck, oder 0 bei zu kleinem Zielpuffer.\n");
		sb.Append("inline size_t Encode(const Payload& payload, uint8_t* dest, size_t dest_size) {\n");
		sb.Append("    size_t pos = 0;\n");
		sb.Append("    if (dest_size < 4) return 0;\n");
		sb.Append("    dest[pos++] = (uint8_t)(NAMESPACE_ID & 0xFF); dest[pos++] = (uint8_t)(NAMESPACE_ID >> 8);\n");
		sb.Append("    dest[pos++] = (uint8_t)(TYPE_ID & 0xFF); dest[pos++] = (uint8_t)(TYPE_ID >> 8);\n");
		foreach (var f in fields) sb.Append(CppEncodeField(f));
		sb.Append("    return pos;\n}\n\n");

		sb.Append("// 'data' zeigt auf den KOMPLETTEN Frame inkl. 4-Byte-Kopf (namespaceId/messageTypeId werden\n");
		sb.Append("// hier nicht erneut geprueft -- Aufgabe des aufrufenden Dispatchers). false bei zu kurzem\n");
		sb.Append("// oder inkonsistentem Frame (z.B. Laengenpraefix zeigt ueber das Frame-Ende hinaus).\n");
		sb.Append("inline bool Decode(const uint8_t* data, size_t len, Payload& out) {\n");
		sb.Append("    size_t pos = 4;\n");
		foreach (var f in fields) sb.Append(CppDecodeField(f));
		sb.Append("    return true;\n}\n\n");

		sb.Append($"}} // namespace {msg.Name}\n\n");
		return sb.ToString();
	}

	private static string GenerateCpp(List<NamespaceDef> namespaces)
	{
		var sb = new StringBuilder();
		sb.Append(HeaderComment);
		sb.Append("\n#pragma once\n#include <cstdint>\n#include <cstddef>\n#include <cstring>\n\nnamespace WsProtocol {\n\n");
		sb.Append("enum class MessageKind : uint8_t { Event = 0, Request = 1, Response = 2 };\n\n");

		// Vier Phasen statt "alles in einem Rutsch pro Namespace": Enums/Structs muessen VOR
		// Classes und Messages vollstaendig deklariert sein (werden dort als Feldtyp referenziert,
		// auch namespaceuebergreifend), Classes VOR Messages. Jede Phase oeffnet "namespace X { ... }"
		// erneut -- in C++/TS zulaessig ("reopening"). NAMESPACE_ID wird deshalb bewusst nur in
		// Phase 1 einmalig deklariert. NULL-Namespace (Name=="") bekommt KEINEN "namespace {}"-
		// Wrapper -- seine Inhalte landen direkt auf WsProtocol-Ebene.
		foreach (var ns in namespaces)
		{
			var open = ns.Name.Length > 0;
			if (open) sb.Append($"namespace {ns.Name} {{\n");
			sb.Append($"constexpr uint16_t NAMESPACE_ID = {ns.Id};\n\n");
			foreach (var e in ns.Enums) sb.Append(GenerateCppNamedEnum(e));
			foreach (var s in ns.Structs) sb.Append(GenerateCppStruct(s));
			if (open) sb.Append($"}} // namespace {ns.Name}\n\n");
		}
		foreach (var ns in namespaces)
		{
			if (ns.Classes.Count == 0) continue;
			var open = ns.Name.Length > 0;
			if (open) sb.Append($"namespace {ns.Name} {{\n");
			foreach (var c in ns.Classes) sb.Append(GenerateCppClass(c));
			if (open) sb.Append($"}} // namespace {ns.Name}\n\n");
		}
		foreach (var ns in namespaces)
		{
			var open = ns.Name.Length > 0;
			if (open) sb.Append($"namespace {ns.Name} {{\n");
			foreach (var msg in ns.Messages) sb.Append(GenerateCppMessage(msg));
			if (open) sb.Append($"}} // namespace {ns.Name}\n\n");
		}

		sb.Append("} // namespace WsProtocol\n");
		return sb.ToString();
	}

	// --- TypeScript ----------------------------------------------------------------------------

	private static string TsSetterName(int size, bool isSigned = false, bool isFloat = false)
	{
		if (isFloat) return "setFloat32";
		var prefix = isSigned ? "setInt" : "setUint";
		return size switch { 1 => prefix + "8", 2 => prefix + "16", 4 => prefix + "32", _ => throw new InvalidOperationException() };
	}
	private static string TsGetterName(int size, bool isSigned = false, bool isFloat = false)
	{
		if (isFloat) return "getFloat32";
		var prefix = isSigned ? "getInt" : "getUint";
		return size switch { 1 => prefix + "8", 2 => prefix + "16", 4 => prefix + "32", _ => throw new InvalidOperationException() };
	}

	// Voll qualifizierter TS-Name -- anders als in C++ gibt es in TS kein implizites "von hier aus
	// sichtbar" ueber Namespace-Grenzen hinweg per Praefix-losem Zugriff auf FREMDE Namespaces,
	// daher IMMER "<namespace>.<Name>" bei benannten Namespaces. NULL-Namespace-Typen (Namespace=="")
	// liegen auf Modul-Ebene (kein Namespace-Praefix) -- dank lexikalischem Scoping in TS trotzdem
	// unqualifiziert aus jedem verschachtelten "export namespace" heraus sichtbar.
	private static string TsQualify(string ns, string name) => ns.Length == 0 ? name : $"{ns}.{name}";
	private static string TsStructTypeName(StructDef s) => TsQualify(s.Namespace, s.Name);
	private static string TsEnumRefTypeName(EnumDef e) => TsQualify(e.Namespace, e.Name);
	private static string TsClassQualifiedName(ClassDef c) => TsQualify(c.Namespace, c.Name);

	private static string TsElementTypeName(FieldBase f) => f switch
	{
		FixedField ff => ff.TsType,
		EnumField ef => ef.EnumName,
		EnumRefField erf => TsEnumRefTypeName(erf.Enum),
		StructField sf => TsStructTypeName(sf.Struct),
		_ => throw new InvalidOperationException("RepeatedField: unerwarteter Elementtyp."),
	};

	private static string TsFieldDecl(FieldBase f) => f switch
	{
		FixedField ff => $"\t\t\t{ff.Name}: {ff.TsType};" + (ff.Description is not null ? $" // {ff.Description}" : "") + "\n",
		EnumField ef => $"\t\t\t{ef.Name}: {ef.EnumName};" + (ef.Description is not null ? $" // {ef.Description}" : "") + "\n",
		EnumRefField erf => $"\t\t\t{erf.Name}: {TsEnumRefTypeName(erf.Enum)};" + (erf.Description is not null ? $" // {erf.Description}" : "") + "\n",
		StructField sf => $"\t\t\t{sf.Name}: {TsStructTypeName(sf.Struct)};\n",
		RepeatedField rf => $"\t\t\t{rf.Name}: {TsElementTypeName(rf.Element)}[];\n",
		StringField sf => $"\t\t\t{sf.Name}: string;\n",
		FixedArrayField af => $"\t\t\t{af.Name}: {af.ElementName}[];\n",
		ClassArrayField caf => $"\t\t\t{caf.Name}: {ClassTaggedUnionTypeName(caf.Classes)}[];\n",
		ClassField clf => $"\t\t\t{clf.Name}: {ClassTaggedUnionTypeName(clf.Classes)};\n",
		_ => throw new InvalidOperationException(),
	};

	// Discriminated Union der erlaubten Klassen eines heterogenen Arrays/Einzelfelds, je Element um
	// ein "classId"-Feld ergaenzt -- TypeScript materialisiert die dekodierten Elemente direkt und
	// vollstaendig typisiert.
	private static string ClassTaggedUnionTypeName(List<ClassDef> classes) =>
		"(" + string.Join(" | ", classes.Select(c => $"({{ classId: typeof {TsClassQualifiedName(c)}.CLASS_ID }} & {TsClassQualifiedName(c)}.Payload)")) + ")";

	// Direktes Schreiben in eine bereits passend grosse DataView an Cursor "pos" -- fuer
	// Array-Elemente und Structs (feste Groesse), NICHT fuer Nachrichten/Klassen auf oberster Ebene
	// (dort chunk-basiert, s. TsEncodeFieldChunk). "varName" ist der Name der lokalen Variable/des
	// Parameters, aus dem gelesen wird ("item" bei Array-Elementen, "value" bei Structs, s. Aufrufer).
	private static string TsEncodeFieldDirect(FieldBase f, string varName = "item")
	{
		switch (f)
		{
			// 64-Bit-Ganzzahlen: DataView kennt keinen number-basierten 64-Bit-Zugriff (nur
			// getBigInt64/getBigUint64), daher manuelle Zerlegung in zwei 32-Bit-Haelften (LE).
			// Nur fuer nicht-negative Werte <= 2^53 exakt -- s. Kommentar bei FixedTypes["uint64"].
			case FixedField ff when ff.Size == 8:
			{
				var valueExpr = $"{varName}.{ff.Name}";
				return $"\t\t\tview.setUint32(pos, ({valueExpr}) >>> 0, true); view.setUint32(pos + 4, Math.floor(({valueExpr}) / 4294967296) >>> 0, true); pos += 8;\n";
			}
			case FixedField ff:
			{
				var valueExpr = ff.IsBool ? $"({varName}.{ff.Name} ? 1 : 0)" : $"{varName}.{ff.Name}";
				var setter = TsSetterName(ff.Size, ff.IsSigned, ff.IsFloat);
				var call = ff.Size == 1 ? $"view.{setter}(pos, {valueExpr});" : $"view.{setter}(pos, {valueExpr}, true);";
				return $"\t\t\t{call} pos += {ff.Size};\n";
			}
			case EnumField ef:
			{
				var setter = TsSetterName(ef.Size);
				var call = ef.Size == 1 ? $"view.{setter}(pos, {varName}.{ef.Name});" : $"view.{setter}(pos, {varName}.{ef.Name}, true);";
				return $"\t\t\t{call} pos += {ef.Size};\n";
			}
			case EnumRefField erf:
			{
				var setter = TsSetterName(erf.Enum.Size);
				var call = erf.Enum.Size == 1 ? $"view.{setter}(pos, {varName}.{erf.Name});" : $"view.{setter}(pos, {varName}.{erf.Name}, true);";
				return $"\t\t\t{call} pos += {erf.Enum.Size};\n";
			}
			case StructField sf:
				return $"\t\t\tpos = {TsQualify(sf.Struct.Namespace, $"encode{sf.Struct.Name}")}Into({varName}.{sf.Name}, view, pos);\n";
			case RepeatedField rf:
			{
				var sb = new StringBuilder();
				sb.Append($"\t\t\tfor (let i = 0; i < {rf.Count}; i++) {{\n");
				switch (rf.Element)
				{
					case FixedField ff when ff.Size == 8:
					{
						var valueExpr = $"{varName}.{rf.Name}[i]";
						sb.Append($"\t\t\t\tview.setUint32(pos, ({valueExpr}) >>> 0, true); view.setUint32(pos + 4, Math.floor(({valueExpr}) / 4294967296) >>> 0, true); pos += 8;\n");
						break;
					}
					case FixedField ff:
					{
						var valueExpr = ff.IsBool ? $"({varName}.{rf.Name}[i] ? 1 : 0)" : $"{varName}.{rf.Name}[i]";
						var setter = TsSetterName(ff.Size, ff.IsSigned, ff.IsFloat);
						sb.Append(ff.Size == 1 ? $"\t\t\t\tview.{setter}(pos, {valueExpr}); pos += 1;\n" : $"\t\t\t\tview.{setter}(pos, {valueExpr}, true); pos += {ff.Size};\n");
						break;
					}
					case EnumField ef:
					{
						var setter = TsSetterName(ef.Size);
						sb.Append(ef.Size == 1 ? $"\t\t\t\tview.{setter}(pos, {varName}.{rf.Name}[i]); pos += 1;\n" : $"\t\t\t\tview.{setter}(pos, {varName}.{rf.Name}[i], true); pos += {ef.Size};\n");
						break;
					}
					case EnumRefField erf:
					{
						var setter = TsSetterName(erf.Enum.Size);
						sb.Append(erf.Enum.Size == 1 ? $"\t\t\t\tview.{setter}(pos, {varName}.{rf.Name}[i]); pos += 1;\n" : $"\t\t\t\tview.{setter}(pos, {varName}.{rf.Name}[i], true); pos += {erf.Enum.Size};\n");
						break;
					}
					case StructField sf:
						sb.Append($"\t\t\t\tpos = {TsQualify(sf.Struct.Namespace, $"encode{sf.Struct.Name}")}Into({varName}.{rf.Name}[i], view, pos);\n");
						break;
				}
				sb.Append("\t\t\t}\n");
				return sb.ToString();
			}
			default:
				throw new InvalidOperationException("Array-Elemente/Structs duerfen nur feste Feldtypen enthalten.");
		}
	}

	private static string TsDecodeField(FieldBase f, string msgOrElementName)
	{
		switch (f)
		{
			case FixedField ff when ff.Size == 8:
				return $"\t\t\tconst {ff.Name} = view.getUint32(pos + 4, true) * 4294967296 + view.getUint32(pos, true);\n\t\t\tpos += 8;\n";
			case FixedField ff:
			{
				var getter = TsGetterName(ff.Size, ff.IsSigned, ff.IsFloat);
				var call = ff.Size == 1 ? $"view.{getter}(pos)" : $"view.{getter}(pos, true)";
				var expr = ff.IsBool ? $"{call} !== 0" : call;
				return $"\t\t\tconst {ff.Name} = {expr};\n\t\t\tpos += {ff.Size};\n";
			}
			case EnumField ef:
			{
				var call = ef.Size == 1 ? $"view.{TsGetterName(ef.Size)}(pos)" : $"view.{TsGetterName(ef.Size)}(pos, true)";
				return $"\t\t\tconst {ef.Name} = {call} as {ef.EnumName};\n\t\t\tpos += {ef.Size};\n";
			}
			case EnumRefField erf:
			{
				var call = erf.Enum.Size == 1 ? $"view.{TsGetterName(erf.Enum.Size)}(pos)" : $"view.{TsGetterName(erf.Enum.Size)}(pos, true)";
				return $"\t\t\tconst {erf.Name} = {call} as {TsEnumRefTypeName(erf.Enum)};\n\t\t\tpos += {erf.Enum.Size};\n";
			}
			case StructField sf:
				return $"\t\t\tconst {{ value: {sf.Name}, nextPos: {sf.Name}NextPos }} = {TsQualify(sf.Struct.Namespace, $"decode{sf.Struct.Name}")}(view, pos);\n\t\t\tpos = {sf.Name}NextPos;\n";
			case RepeatedField rf:
			{
				var sb = new StringBuilder();
				sb.Append($"\t\t\tconst {rf.Name}: {TsElementTypeName(rf.Element)}[] = [];\n");
				sb.Append($"\t\t\tfor (let i = 0; i < {rf.Count}; i++) {{\n");
				switch (rf.Element)
				{
					case FixedField ff when ff.Size == 8:
					{
						sb.Append($"\t\t\t\t{rf.Name}.push(view.getUint32(pos + 4, true) * 4294967296 + view.getUint32(pos, true)); pos += 8;\n");
						break;
					}
					case FixedField ff:
					{
						var getter = TsGetterName(ff.Size, ff.IsSigned, ff.IsFloat);
						var call = ff.Size == 1 ? $"view.{getter}(pos)" : $"view.{getter}(pos, true)";
						var expr = ff.IsBool ? $"{call} !== 0" : call;
						sb.Append($"\t\t\t\t{rf.Name}.push({expr}); pos += {ff.Size};\n");
						break;
					}
					case EnumField ef:
					{
						var call = ef.Size == 1 ? $"view.{TsGetterName(ef.Size)}(pos)" : $"view.{TsGetterName(ef.Size)}(pos, true)";
						sb.Append($"\t\t\t\t{rf.Name}.push({call} as {ef.EnumName}); pos += {ef.Size};\n");
						break;
					}
					case EnumRefField erf:
					{
						var call = erf.Enum.Size == 1 ? $"view.{TsGetterName(erf.Enum.Size)}(pos)" : $"view.{TsGetterName(erf.Enum.Size)}(pos, true)";
						sb.Append($"\t\t\t\t{rf.Name}.push({call} as {TsEnumRefTypeName(erf.Enum)}); pos += {erf.Enum.Size};\n");
						break;
					}
					case StructField sf:
						sb.Append($"\t\t\t\tconst decoded = {TsQualify(sf.Struct.Namespace, $"decode{sf.Struct.Name}")}(view, pos);\n");
						sb.Append($"\t\t\t\t{rf.Name}.push(decoded.value); pos = decoded.nextPos;\n");
						break;
				}
				sb.Append("\t\t\t}\n");
				return sb.ToString();
			}
			case StringField sf:
				return
					$"\t\t\tif (view.byteLength - pos < 4) throw new Error(\"{msgOrElementName}: frame too short\");\n" +
					$"\t\t\tconst {sf.Name}Len = view.getUint32(pos, true); pos += 4;\n" +
					$"\t\t\tif (view.byteLength - pos < {sf.Name}Len) throw new Error(\"{msgOrElementName}: frame too short\");\n" +
					$"\t\t\tconst {sf.Name} = new TextDecoder().decode(new Uint8Array(view.buffer, view.byteOffset + pos, {sf.Name}Len));\n" +
					$"\t\t\tpos += {sf.Name}Len;\n";
			case FixedArrayField af:
				return
					$"\t\t\tif (view.byteLength - pos < 4) throw new Error(\"{msgOrElementName}: frame too short\");\n" +
					$"\t\t\tconst {af.Name}Count = view.getUint32(pos, true); pos += 4;\n" +
					$"\t\t\tconst {af.Name}: {af.ElementName}[] = [];\n" +
					$"\t\t\tfor (let i = 0; i < {af.Name}Count; i++) {{ {af.Name}.push(decode{af.ElementName}(view, pos)); pos += {af.ElementName}_SIZE; }}\n";
			case ClassArrayField caf:
				return TsDecodeClassTaggedField(caf.Name, caf.Classes, msgOrElementName, isArray: true);
			case ClassField clf:
				return TsDecodeClassTaggedField(clf.Name, clf.Classes, msgOrElementName, isArray: false);
			default:
				throw new InvalidOperationException();
		}
	}

	// Gemeinsame Decodier-Logik fuer ClassArrayField (Schleife mit Count-Praefix) und ClassField
	// (genau ein Element, kein Praefix) -- beides muss laut Validierung das letzte Feld sein,
	// deshalb hier ohne Sorge um "wie viele Bytes fuer nachfolgende Felder uebrig lassen" bis zum
	// Ende des Frames dekodierbar.
	private static string TsDecodeClassTaggedField(string fieldName, List<ClassDef> classes, string msgName, bool isArray)
	{
		var unionType = ClassTaggedUnionTypeName(classes);
		var sb = new StringBuilder();
		var pad = isArray ? "\t" : "";

		string DecodeOneInto(string target)
		{
			var inner = new StringBuilder();
			inner.Append($"\t\t\t{pad}if (view.byteLength - pos < 2) throw new Error(\"{msgName}: frame too short\");\n");
			inner.Append($"\t\t\t{pad}const classId = view.getUint16(pos, true); pos += 2;\n");
			inner.Append($"\t\t\t{pad}switch (classId) {{\n");
			foreach (var c in classes)
			{
				var qc = TsClassQualifiedName(c);
				inner.Append($"\t\t\t{pad}case {qc}.CLASS_ID: {{\n");
				inner.Append($"\t\t\t{pad}\tconst {{ value, nextPos }} = {qc}.decodeAt(view, pos);\n");
				inner.Append($"\t\t\t{pad}\t{target} = {{ classId: {qc}.CLASS_ID, ...value }};\n");
				inner.Append($"\t\t\t{pad}\tpos = nextPos;\n");
				inner.Append($"\t\t\t{pad}\tbreak;\n");
				inner.Append($"\t\t\t{pad}}}\n");
			}
			inner.Append($"\t\t\t{pad}default: throw new Error(\"{msgName}: unbekannte classId \" + classId + \" in {fieldName}\");\n");
			inner.Append($"\t\t\t{pad}}}\n");
			return inner.ToString();
		}

		if (isArray)
		{
			sb.Append($"\t\t\tif (view.byteLength - pos < 4) throw new Error(\"{msgName}: frame too short\");\n");
			sb.Append($"\t\t\tconst {fieldName}Count = view.getUint32(pos, true); pos += 4;\n");
			sb.Append($"\t\t\tconst {fieldName}: {unionType}[] = [];\n");
			sb.Append($"\t\t\tfor (let i = 0; i < {fieldName}Count; i++) {{\n");
			sb.Append($"\t\t\t\tlet element!: {unionType};\n");
			sb.Append(DecodeOneInto("element"));
			sb.Append($"\t\t\t\t{fieldName}.push(element);\n");
			sb.Append("\t\t\t}\n");
		}
		else
		{
			sb.Append($"\t\t\tlet {fieldName}!: {unionType};\n");
			sb.Append(DecodeOneInto(fieldName));
		}
		return sb.ToString();
	}

	private static string TsEncodeFieldChunk(FieldBase f)
	{
		switch (f)
		{
			case FixedField ff:
				return TsEncodeChunkFixedLike(ff.Size, ff.IsBool ? $"(payload.{ff.Name} ? 1 : 0)" : $"payload.{ff.Name}", ff.IsSigned, ff.IsFloat);
			case EnumField ef:
				return TsEncodeChunkFixedLike(ef.Size, $"payload.{ef.Name}");
			case EnumRefField erf:
				return TsEncodeChunkFixedLike(erf.Enum.Size, $"payload.{erf.Name}");
			case StructField sf:
				return $"\t\t\tchunks.push({TsQualify(sf.Struct.Namespace, $"encode{sf.Struct.Name}")}(payload.{sf.Name}));\n";
			case RepeatedField rf:
			{
				var sb = new StringBuilder();
				sb.Append($"\t\t\tfor (let i = 0; i < {rf.Count}; i++) {{\n");
				switch (rf.Element)
				{
					case FixedField ff:
						sb.Append("\t\t\t\t" + TsEncodeChunkFixedLike(ff.Size, ff.IsBool ? $"(payload.{rf.Name}[i] ? 1 : 0)" : $"payload.{rf.Name}[i]", ff.IsSigned, ff.IsFloat).Trim() + "\n");
						break;
					case EnumField ef:
						sb.Append("\t\t\t\t" + TsEncodeChunkFixedLike(ef.Size, $"payload.{rf.Name}[i]").Trim() + "\n");
						break;
					case EnumRefField erf:
						sb.Append("\t\t\t\t" + TsEncodeChunkFixedLike(erf.Enum.Size, $"payload.{rf.Name}[i]").Trim() + "\n");
						break;
					case StructField sf:
						sb.Append($"\t\t\t\tchunks.push({TsQualify(sf.Struct.Namespace, $"encode{sf.Struct.Name}")}(payload.{rf.Name}[i]));\n");
						break;
				}
				sb.Append("\t\t\t}\n");
				return sb.ToString();
			}
			case StringField sf:
				return $"\t\t\t{{ const bytes = new TextEncoder().encode(payload.{sf.Name}); const len = new Uint8Array(4); " +
					"new DataView(len.buffer).setUint32(0, bytes.length, true); chunks.push(len); chunks.push(bytes); }\n";
			case FixedArrayField af:
				return $"\t\t\t{{ const count = new Uint8Array(4); new DataView(count.buffer).setUint32(0, payload.{af.Name}.length, true); " +
					$"chunks.push(count); for (const item of payload.{af.Name}) chunks.push(encode{af.ElementName}(item)); }}\n";
			case ClassArrayField caf:
				return TsEncodeClassTaggedFieldChunk(caf.Name, caf.Classes, isArray: true);
			case ClassField clf:
				return TsEncodeClassTaggedFieldChunk(clf.Name, clf.Classes, isArray: false);
			default:
				throw new InvalidOperationException();
		}
	}

	private static string TsEncodeOneTaggedElement(string itemExpr, List<ClassDef> classes, string indent)
	{
		var sb = new StringBuilder();
		sb.Append($"{indent}const tag = new Uint8Array(2); new DataView(tag.buffer).setUint16(0, {itemExpr}.classId, true); chunks.push(tag);\n");
		sb.Append($"{indent}switch ({itemExpr}.classId) {{\n");
		foreach (var c in classes)
		{
			var qc = TsClassQualifiedName(c);
			sb.Append($"{indent}case {qc}.CLASS_ID: chunks.push({qc}.encode({itemExpr})); break;\n");
		}
		sb.Append($"{indent}default: throw new Error(\"unbekannte classId \" + ({itemExpr} as any).classId);\n");
		sb.Append($"{indent}}}\n");
		return sb.ToString();
	}

	private static string TsEncodeClassTaggedFieldChunk(string fieldName, List<ClassDef> classes, bool isArray)
	{
		var sb = new StringBuilder();
		if (isArray)
		{
			sb.Append("\t\t\t{\n");
			sb.Append($"\t\t\t\tconst count = new Uint8Array(4); new DataView(count.buffer).setUint32(0, payload.{fieldName}.length, true); chunks.push(count);\n");
			sb.Append($"\t\t\t\tfor (const item of payload.{fieldName}) {{\n");
			sb.Append(TsEncodeOneTaggedElement("item", classes, "\t\t\t\t\t"));
			sb.Append("\t\t\t\t}\n");
			sb.Append("\t\t\t}\n");
		}
		else
		{
			sb.Append(TsEncodeOneTaggedElement($"payload.{fieldName}", classes, "\t\t\t"));
		}
		return sb.ToString();
	}

	private static string TsEncodeChunkFixedLike(int size, string valueExpr, bool isSigned = false, bool isFloat = false)
	{
		if (size == 8)
		{
			return $"\t\t\t{{ const b = new Uint8Array(8); const v = new DataView(b.buffer); " +
				$"v.setUint32(0, ({valueExpr}) >>> 0, true); v.setUint32(4, Math.floor(({valueExpr}) / 4294967296) >>> 0, true); chunks.push(b); }}\n";
		}
		var setter = TsSetterName(size, isSigned, isFloat);
		var call = size == 1 ? $"v.{setter}(0, {valueExpr});" : $"v.{setter}(0, {valueExpr}, true);";
		return $"\t\t\t{{ const b = new Uint8Array({size}); const v = new DataView(b.buffer); {call} chunks.push(b); }}\n";
	}

	private static string GenerateTsEnum(EnumField ef) =>
		$"\t\texport enum {ef.EnumName} {{ " + string.Join(", ", ef.Values.Select(v => $"{v.Name} = {v.Value}")) + " }\n\n";

	private static string GenerateTsNamedEnum(EnumDef e, string indent)
	{
		var sb = new StringBuilder();
		if (e.Description is not null) sb.Append($"{indent}// {e.Description}\n");
		sb.Append($"{indent}export enum {e.Name} {{ ");
		sb.Append(string.Join(", ", e.Values.Select(v => $"{v.Name} = {v.Value}")));
		sb.Append(" }\n\n");
		return sb.ToString();
	}

	// Struct: "encode<Name>Into"/"decode<Name>" arbeiten mit einem expliziten Cursor (statt einer
	// frischen, exakt passenden DataView wie bei einem Array-Element), weil ein Struct an beliebiger
	// Position innerhalb einer Message/Class/eines anderen Structs eingebettet sein kann.
	private static string GenerateTsStruct(StructDef s, string indent)
	{
		var sb = new StringBuilder();
		if (s.Description is not null) sb.Append($"{indent}// {s.Description}\n");
		sb.Append($"{indent}export interface {s.Name} {{\n");
		foreach (var f in s.Fields) sb.Append(TsFieldDecl(f));
		sb.Append($"{indent}}}\n");

		var totalSize = s.Fields.Sum(FixedSizeOf);
		sb.Append($"{indent}export const {s.Name}_SIZE = {totalSize};\n\n");

		foreach (var ef in s.Fields.OfType<EnumField>()) sb.Append(GenerateTsEnum(ef));

		sb.Append($"{indent}export function encode{s.Name}Into(value: {s.Name}, view: DataView, pos: number): number {{\n");
		foreach (var f in s.Fields) sb.Append(TsEncodeFieldDirect(f, "value"));
		sb.Append($"{indent}\treturn pos;\n{indent}}}\n\n");

		sb.Append($"{indent}export function encode{s.Name}(value: {s.Name}): Uint8Array {{\n");
		sb.Append($"{indent}\tconst buffer = new ArrayBuffer({s.Name}_SIZE);\n{indent}\tencode{s.Name}Into(value, new DataView(buffer), 0);\n{indent}\treturn new Uint8Array(buffer);\n{indent}}}\n\n");

		sb.Append($"{indent}export function decode{s.Name}(view: DataView, offset: number): {{ value: {s.Name}; nextPos: number }} {{\n");
		sb.Append("\t\tlet pos = offset;\n");
		foreach (var f in s.Fields) sb.Append(TsDecodeField(f, s.Name));
		sb.Append("\t\treturn { value: { " + string.Join(", ", s.Fields.Select(FieldName)) + " }, nextPos: pos };\n\t}\n\n");

		return sb.ToString();
	}

	// Class: wie eine Message ohne Kopf -- "decodeAt"/"encode" statt "decode"/"encode" der Message,
	// weil "decode" hier zusaetzlich den aktuellen Cursor zurueckgeben muss (wird sequentiell
	// innerhalb eines heterogenen Arrays/Einzelfelds aufgerufen).
	private static string GenerateTsClass(ClassDef c, string indent)
	{
		var sb = new StringBuilder();
		if (c.Description is not null) sb.Append($"{indent}// {c.Description}\n");
		sb.Append($"{indent}export namespace {c.Name} {{\n");
		sb.Append($"{indent}\texport const CLASS_ID = {c.Id};\n\n");

		foreach (var ef in c.Fields.OfType<EnumField>()) sb.Append(GenerateTsEnum(ef));

		sb.Append("\t\texport interface Payload {\n");
		foreach (var f in c.Fields) sb.Append(TsFieldDecl(f));
		sb.Append("\t\t}\n\n");

		sb.Append("\t\texport function encode(payload: Payload): Uint8Array {\n");
		sb.Append("\t\t\tconst chunks: Uint8Array[] = [];\n");
		foreach (var f in c.Fields) sb.Append(TsEncodeFieldChunk(f));
		sb.Append("\t\t\tlet total = 0;\n\t\t\tfor (const ch of chunks) total += ch.length;\n");
		sb.Append("\t\t\tconst out = new Uint8Array(total);\n\t\t\tlet o = 0;\n\t\t\tfor (const ch of chunks) { out.set(ch, o); o += ch.length; }\n");
		sb.Append("\t\t\treturn out;\n\t\t}\n\n");

		sb.Append($"\t\texport function decodeAt(view: DataView, offset: number): {{ value: Payload; nextPos: number }} {{\n");
		sb.Append("\t\t\tlet pos = offset;\n");
		foreach (var f in c.Fields) sb.Append(TsDecodeField(f, c.Name));
		sb.Append("\t\t\treturn { value: { " + string.Join(", ", c.Fields.Select(FieldName)) + " }, nextPos: pos };\n\t\t}\n");
		sb.Append($"{indent}}}\n\n");
		return sb.ToString();
	}

	private static string GenerateTsElementStruct(FixedArrayField af)
	{
		var totalSize = af.ElementFields.Sum(FixedSizeOf);
		var sb = new StringBuilder();
		sb.Append($"\t\texport interface {af.ElementName} {{\n");
		foreach (var ef in af.ElementFields) sb.Append(TsFieldDecl(ef));
		sb.Append("\t\t}\n");
		sb.Append($"\t\texport const {af.ElementName}_SIZE = {totalSize};\n\n");

		foreach (var ef in af.ElementFields.OfType<EnumField>()) sb.Append(GenerateTsEnum(ef));

		sb.Append($"\t\texport function encode{af.ElementName}(item: {af.ElementName}): Uint8Array {{\n");
		sb.Append($"\t\t\tconst buffer = new ArrayBuffer({af.ElementName}_SIZE);\n\t\t\tconst view = new DataView(buffer);\n\t\t\tlet pos = 0;\n");
		foreach (var ef in af.ElementFields) sb.Append(TsEncodeFieldDirect(ef));
		sb.Append("\t\t\treturn new Uint8Array(buffer);\n\t\t}\n\n");

		sb.Append($"\t\texport function decode{af.ElementName}(view: DataView, offset: number): {af.ElementName} {{\n");
		sb.Append("\t\t\tlet pos = offset;\n");
		foreach (var ef in af.ElementFields) sb.Append(TsDecodeField(ef, af.ElementName));
		sb.Append("\t\t\treturn { " + string.Join(", ", af.ElementFields.Select(FieldName)) + " };\n\t\t}\n\n");

		return sb.ToString();
	}

	private static string GenerateTsMessage(Message msg, string indent)
	{
		var fields = EffectiveFields(msg);
		var sb = new StringBuilder();
		if (msg.Description is not null) sb.Append($"{indent}// {msg.Description}\n");
		sb.Append($"{indent}export namespace {msg.Name} {{\n");
		sb.Append($"{indent}\texport const TYPE_ID = {msg.Id};\n");
		sb.Append($"{indent}\texport const KIND = {KindEnumTs(msg.Kind)};\n\n");

		foreach (var ef in fields.OfType<EnumField>()) sb.Append(GenerateTsEnum(ef));
		foreach (var af in fields.OfType<FixedArrayField>()) sb.Append(GenerateTsElementStruct(af));

		sb.Append("\t\texport interface Payload {\n");
		foreach (var f in fields) sb.Append(TsFieldDecl(f));
		sb.Append("\t\t}\n\n");

		sb.Append("\t\texport function encode(payload: Payload): Uint8Array {\n");
		sb.Append("\t\t\tconst chunks: Uint8Array[] = [];\n");
		sb.Append("\t\t\t{ const h = new Uint8Array(4); const hv = new DataView(h.buffer); hv.setUint16(0, NAMESPACE_ID, true); hv.setUint16(2, TYPE_ID, true); chunks.push(h); }\n");
		foreach (var f in fields) sb.Append(TsEncodeFieldChunk(f));
		sb.Append("\t\t\tlet total = 0;\n\t\t\tfor (const c of chunks) total += c.length;\n");
		sb.Append("\t\t\tconst out = new Uint8Array(total);\n\t\t\tlet o = 0;\n\t\t\tfor (const c of chunks) { out.set(c, o); o += c.length; }\n");
		sb.Append("\t\t\treturn out;\n\t\t}\n\n");

		sb.Append("\t\t// 'view' ist der KOMPLETTE Frame inkl. 4-Byte-Kopf, 'offset' zeigt auf dessen Anfang\n");
		sb.Append("\t\t// (namespaceId/messageTypeId werden hier nicht erneut geprueft -- Aufgabe des Dispatchers\n");
		sb.Append("\t\t// in ws-client.ts). Wirft, wenn der Frame kuerzer als angegeben/erwartet ist.\n");
		sb.Append("\t\texport function decode(view: DataView, offset: number): Payload {\n");
		sb.Append("\t\t\tlet pos = offset + 4;\n");
		foreach (var f in fields) sb.Append(TsDecodeField(f, msg.Name));
		sb.Append("\t\t\treturn { " + string.Join(", ", fields.Select(FieldName)) + " };\n\t\t}\n");
		sb.Append($"{indent}}}\n\n");
		return sb.ToString();
	}

	private static string GenerateTs(List<NamespaceDef> namespaces)
	{
		var sb = new StringBuilder();
		sb.Append(HeaderComment);
		sb.Append("\n\nexport enum MessageKind { Event = 0, Request = 1, Response = 2 }\n\n");

		// Gleiche Vier-Phasen-Begruendung wie in GenerateCpp. NULL-Namespace (Name=="") bekommt
		// keinen "export namespace {}"-Wrapper -- Inhalte landen direkt auf Modul-Ebene (indent "").
		foreach (var ns in namespaces)
		{
			var open = ns.Name.Length > 0;
			var indent = open ? "\t" : "";
			if (open) sb.Append($"export namespace {ns.Name} {{\n");
			sb.Append($"{indent}export const NAMESPACE_ID = {ns.Id};\n\n");
			foreach (var e in ns.Enums) sb.Append(GenerateTsNamedEnum(e, indent));
			foreach (var s in ns.Structs) sb.Append(GenerateTsStruct(s, indent));
			if (open) sb.Append("}\n\n");
		}
		foreach (var ns in namespaces)
		{
			if (ns.Classes.Count == 0) continue;
			var open = ns.Name.Length > 0;
			var indent = open ? "\t" : "";
			if (open) sb.Append($"export namespace {ns.Name} {{\n");
			foreach (var c in ns.Classes) sb.Append(GenerateTsClass(c, indent));
			if (open) sb.Append("}\n\n");
		}
		foreach (var ns in namespaces)
		{
			var open = ns.Name.Length > 0;
			var indent = open ? "\t" : "";
			if (open) sb.Append($"export namespace {ns.Name} {{\n");
			foreach (var msg in ns.Messages) sb.Append(GenerateTsMessage(msg, indent));
			if (open) sb.Append("}\n\n");
		}

		return sb.ToString();
	}

	// --- Entry point ---------------------------------------------------------------------------

	// Loest eine einzelne --ws-protocol-path-Angabe auf: ein VERZEICHNIS liefert alle direkt darin
	// liegenden *.json-Dateien (nicht rekursiv), eine einzelne .json-DATEI wird direkt uebernommen.
	// Relative Pfade werden gegen Paths.RootDir aufgeloest, absolute Pfade unveraendert uebernommen.
	private static List<string> ResolveSourceFiles(string source)
	{
		var resolved = Path.IsPathRooted(source) ? source : Path.Combine(Paths.RootDir, source);
		if (Directory.Exists(resolved))
		{
			return Directory.GetFiles(resolved, "*.json").ToList();
		}
		if (File.Exists(resolved))
		{
			return [resolved];
		}
		throw new InvalidOperationException(
			$"ws-protocol-Quelle nicht gefunden (weder Verzeichnis noch Datei): {resolved}" +
			(source != resolved ? $" (angegeben als \"{source}\")" : ""));
	}

	private static void CollectDeclarations(JsonElement root, string propertyName, Dictionary<string, JsonElement> registry, string kindLabel)
	{
		if (!root.TryGetProperty(propertyName, out var arrEl)) return;
		foreach (var entry in arrEl.EnumerateArray())
		{
			var name = entry.GetProperty("name").GetString()!;
			if (!registry.TryAdd(name, entry))
			{
				throw new InvalidOperationException($"ws-protocol: {kindLabel} \"{name}\" ist mehrfach deklariert.");
			}
		}
	}

	// 'sources' -- Verzeichnisse und/oder einzelne *.json-Dateien, deren Inhalte zu EINEM
	// gemeinsamen ws_protocol.hh/ws-protocol.ts zusammengefasst werden (s. docs/websocket-protocol.md).
	// Ohne Angabe (null/leer) faellt das auf das bisherige Alleinverhalten zurueck: nur
	// Paths.WsProtocolDir ("ws-protocol/" im Repo-Root).
	public static void Run(string? explicitBoardId, IReadOnlyList<string>? sources = null)
	{
		var effectiveSources = (sources is { Count: > 0 }) ? sources : [Paths.WsProtocolDir];

		var files = effectiveSources.SelectMany(ResolveSourceFiles)
			.Select(Path.GetFullPath)
			.Distinct()
			.OrderBy(f => f, StringComparer.Ordinal)
			.ToList();
		if (files.Count == 0)
		{
			throw new InvalidOperationException(
				$"Keine ws-protocol-*.json-Dateien gefunden (durchsucht: {string.Join(", ", effectiveSources)}) -- " +
				"s. docs/websocket-protocol.md.");
		}

		// Pass 1: alle Dateien roh einlesen, dann ALLE enums/structs/classes-Deklarationen ueber
		// ALLE Dateien hinweg in globale Registries eintragen -- das ist der Mechanismus, der
		// Referenzen (structRef/enumRef/classes) unabhaengig von Datei- ODER Namespace-Grenzen ohne
		// "include" ermoeglicht.
		var rawFiles = files.Select(ParseFileRaw).ToList();

		var idMapPath = Path.Combine(Paths.WsProtocolDir, "ids.txt");
		var idMap = IdMap.Load(idMapPath);
		var ctx = new ParseContext(idMap);

		foreach (var raw in rawFiles)
		{
			CollectDeclarations(raw.Root, "enums", ctx.EnumJson, "Enum");
			CollectDeclarations(raw.Root, "structs", ctx.StructJson, "Struct");
			CollectDeclarations(raw.Root, "classes", ctx.ClassJson, "Klasse");
		}

		// Pass 2: Enums/Structs/Classes eager aufloesen (nicht erst bei erster Referenz) -- damit
		// werden auch ungenutzte Definitionen generiert und Fehler (unbekannte Referenz, zyklischer
		// Struct) fallen sofort auf. Danach Messages parsen (koennen quer durch alle Namespaces/
		// Dateien auf Enums/Structs/Classes verweisen).
		var enumsByName = ctx.EnumJson.Keys.ToDictionary(k => k, k => ctx.ResolveEnum(k, $"Namespace-Registrierung \"{k}\""));
		var structsByName = ctx.StructJson.Keys.ToDictionary(k => k, k => ctx.ResolveStruct(k, $"Namespace-Registrierung \"{k}\""));
		var classesByName = ctx.ClassJson.Keys.ToDictionary(k => k, k => ctx.ResolveClass(k, $"Namespace-Registrierung \"{k}\""));

		var messages = new List<Message>();
		var seenMessageNames = new HashSet<string>(StringComparer.Ordinal);
		foreach (var raw in rawFiles)
		{
			if (!raw.Root.TryGetProperty("messages", out var messagesEl)) continue;
			foreach (var m in messagesEl.EnumerateArray())
			{
				var fullName = m.GetProperty("name").GetString()!;
				if (!seenMessageNames.Add(fullName))
				{
					throw new InvalidOperationException($"ws-protocol: Nachricht \"{fullName}\" ist mehrfach deklariert.");
				}
				messages.Add(ParseMessage(m, ctx));
			}
		}

		// Gruppieren nach Namespace (aus den Namen abgeleitet, s. SplitName) -- jede Datei kann zu
		// mehreren Namespaces beitragen, jeder Namespace kann aus mehreren Dateien zusammengesetzt
		// sein.
		var allNamespaceNames = enumsByName.Values.Select(e => e.Namespace)
			.Concat(structsByName.Values.Select(s => s.Namespace))
			.Concat(classesByName.Values.Select(c => c.Namespace))
			.Concat(messages.Select(m => m.Namespace))
			.Distinct()
			.OrderBy(n => n, StringComparer.Ordinal)
			.ToList();

		var namespaces = allNamespaceNames.Select(nsName => new NamespaceDef(
			nsName,
			idMap.GetOrAssignNamespace(nsName),
			enumsByName.Values.Where(e => e.Namespace == nsName).ToList(),
			structsByName.Values.Where(s => s.Namespace == nsName).ToList(),
			classesByName.Values.Where(c => c.Namespace == nsName).ToList(),
			messages.Where(m => m.Namespace == nsName).ToList()
		)).ToList();

		idMap.SaveIfDirty(idMapPath);

		var cpp = GenerateCpp(namespaces);
		var ts = GenerateTs(namespaces);

		var boardId = BoardContext.ResolveBoardId(explicitBoardId);
		var coreOut = boardId is not null ? BoardArchive.BoardGeneratedDir(boardId) : Paths.CoreGeneratedDir;
		var webOut = boardId is not null ? BoardArchive.BoardGeneratedDir(boardId) : Paths.WebGeneratedDir;

		Directory.CreateDirectory(coreOut);
		Directory.CreateDirectory(webOut);
		File.WriteAllText(Path.Combine(coreOut, "ws_protocol.hh"), cpp);
		File.WriteAllText(Path.Combine(webOut, "ws-protocol.ts"), ts);

		var sourcesLabel = string.Join(", ", effectiveSources);
		Console.WriteLine(boardId is not null
			? $"{files.Count} Datei(en) aus {effectiveSources.Count} Quelle(n) ({sourcesLabel}) -> Board-Archiv ({boardId})"
			: $"Kein Board-Kontext bekannt -- schreibe ws-protocol ({files.Count} Datei(en) aus {effectiveSources.Count} Quelle(n): {sourcesLabel}) " +
			  "direkt nach Core/generated bzw. web/generated (kein Archiv-Eintrag).");
	}
}
