// Phase "ReadModbusRegisterMapAndGenerateFiles" (s. docs/build-process.md Abschnitt 2) -- erzeugt
// aus der programmiersprachen-neutralen register-map.json drei reine Konstanten-Fragmente
// (register_input.inc, register_holding.inc, register_maxindex.inc) und register-map.ts.
// register-map.json ist von Hand gepflegtes Schema,
// kein generierter Code -- hier bewusst locker (JsonElement) statt eines vollen Schema-Records
// gelesen, das bei jeder register-map.json-Erweiterung mitgepflegt werden muesste.
using System.Text;
using System.Text.Json;

namespace Builder.Phases;

public static class ReadModbusRegisterMap
{
	private static string? GetString(JsonElement el, string prop) =>
		el.TryGetProperty(prop, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() : null;

	private static bool GetBool(JsonElement el, string prop) =>
		el.TryGetProperty(prop, out var v) && v.ValueKind == JsonValueKind.True;

	private static int? GetInt(JsonElement el, string prop) =>
		el.TryGetProperty(prop, out var v) && v.ValueKind == JsonValueKind.Number ? v.GetInt32() : null;

	private static List<string> RegCommentParts(JsonElement reg)
	{
		var parts = new List<string>();
		var description = GetString(reg, "description");
		if (description is not null) parts.Add(description);
		var unit = GetString(reg, "unit");
		if (unit is not null) parts.Add($"[{unit}]");
		if (GetBool(reg, "signed")) parts.Add("signed");
		var gpio = GetString(reg, "gpio");
		if (gpio is not null) parts.Add(gpio);
		if (reg.TryGetProperty("i2c", out var i2c) && i2c.ValueKind == JsonValueKind.Object)
		{
			var bus = GetString(i2c, "bus");
			var irqPin = GetString(i2c, "irqPin");
			parts.Add(irqPin is not null ? $"{bus}, IRQ {irqPin}" : bus!);
		}
		return parts;
	}

	// Ein Eintrag in region.registers ist entweder ein normales Register (hat "name") oder eine
	// combine-Gruppe (hat "combine", darin ein "registers"-Array mit den zugrundeliegenden
	// Einzelregistern -- s. register-map.json, z.B. CHIP_ID/CAN_TX_COUNT). Fuer die C++-Ausgabe
	// spielt die Gruppierung keine Rolle, nur die zugrundeliegenden Einzelregister zaehlen.
	private static IEnumerable<JsonElement> FlattenRegisters(JsonElement registers)
	{
		foreach (var entry in registers.EnumerateArray())
		{
			if (entry.TryGetProperty("combine", out var combine))
			{
				foreach (var r in combine.GetProperty("registers").EnumerateArray()) yield return r;
			}
			else
			{
				yield return entry;
			}
		}
	}

	private static string GenFlatBank(JsonElement bank)
	{
		var sb = new StringBuilder();
		foreach (var region in bank.GetProperty("regions").EnumerateArray())
		{
			var title = GetString(region, "title");
			var startAddress = GetInt(region, "startAddress")!.Value;
			var endAddress = GetInt(region, "endAddress")!.Value;
			var id = GetString(region, "id");
			sb.Append($"// {title} (Region-Start {startAddress}, Reserve bis {endAddress})\n");
			foreach (var reg in FlattenRegisters(region.GetProperty("registers")))
			{
				var name = GetString(reg, "name");
				var address = GetInt(reg, "address")!.Value;
				var comment = string.Join(" -- ", RegCommentParts(reg));
				sb.Append($"constexpr uint16_t {name} = {address};{(comment.Length > 0 ? " // " + comment : "")}\n");
			}
			sb.Append($"constexpr uint16_t {id}_REGION_START = {startAddress};\n");
			sb.Append($"constexpr uint16_t {id}_REGION_END   = {endAddress};\n\n");
		}
		return sb.ToString();
	}

	private const string IncFileHeader = """
		// GENERIERT von builder/Phases/ReadModbusRegisterMap.cs aus register-map.json -- nicht von
		// Hand editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend die
		// Phase "ReadModbusRegisterMapAndGenerateFiles" erneut ausfuehren.
		//
		// Nur per #include aus Core/Src/modbus_register_model.hh eingebunden (innerhalb des dort
		// bereits geoeffneten namespace-Blocks), nicht eigenstaendig uebersetzbar -- enthaelt bewusst
		// nur constexpr-Zeilen, keine eigene namespace-Deklaration/Klammern.

		""";

	private static string GenerateInputInc(JsonElement schema)
	{
		var healthBits = new StringBuilder("// HealthState-Bitfeld (Input::HEALTH_STATE), s. register-map.json healthBits\n");
		foreach (var bit in schema.GetProperty("healthBits").EnumerateArray())
		{
			healthBits.Append($"constexpr uint16_t HEALTH_BIT_{GetString(bit, "name")} = {GetInt(bit, "bit")};\n");
		}

		return $"""
			{IncFileHeader}//
			// Adressen sind Offsets in ModbusRegisterModel::input_registers_.
			// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.
			// Block-Markierungen heissen *_REGION_START/*_REGION_END statt *_BASE/*_END, da z.B.
			// ETH_BASE/PWR_BASE bereits als CMSIS-Peripherie-Basisadressen vergeben sind.

			{GenFlatBank(schema.GetProperty("banks").GetProperty("input"))}{healthBits}
			""";
	}

	private static string GenerateHoldingInc(JsonElement schema)
	{
		return $"""
			{IncFileHeader}//
			// Adressen sind Offsets in ModbusRegisterModel::holding_registers_.
			// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.

			{GenFlatBank(schema.GetProperty("banks").GetProperty("holding"))}
			""";
	}

	private static string GenerateMaxIndexInc(JsonElement schema)
	{
		var inputRegions = schema.GetProperty("banks").GetProperty("input").GetProperty("regions").EnumerateArray().ToList();
		var holdingRegions = schema.GetProperty("banks").GetProperty("holding").GetProperty("regions").EnumerateArray().ToList();
		var lastInputRegion = inputRegions[^1];
		var lastHoldingRegion = holdingRegions[^1];

		return $"""
			{IncFileHeader}//
			// Hoechster belegter Index je Register-Bank -- bestimmt die Groesse der
			// ModbusRegisterModel-Arrays (s. modbus_register_model.hh). Bei Erweiterung der Register-Map
			// (neue Region ans Ende angehaengt) hier mitziehen. Referenziert Input::/Holding::, die im
			// umschliessenden modbus_register_model.hh bereits vor dieser #include definiert sind.
			constexpr uint16_t INPUT_REGISTER_MAX_INDEX   = Input::{GetString(lastInputRegion, "id")}_REGION_END;
			constexpr uint16_t HOLDING_REGISTER_MAX_INDEX = Holding::{GetString(lastHoldingRegion, "id")}_REGION_END;

			""";
	}

	private static string TsStringLiteral(string s) => JsonSerializer.Serialize(s, Json.Compact);

	// entry ist entweder ein normales Register (hat "name") oder eine combine-Gruppe (hat
	// "combine", darin "registers": [...] mit den zugrundeliegenden Einzelregistern -- s.
	// register-map.json). Erzeugt in beiden Faellen GENAU einen RegisterDef-Eintrag; die
	// combine-Gruppe nutzt ihre eigenen label/description/display-Angaben statt derer des ersten
	// zugrundeliegenden Registers.
	private static string GenTsRegister(JsonElement entry, string bank)
	{
		if (entry.TryGetProperty("combine", out var group))
		{
			var registers = group.GetProperty("registers").EnumerateArray().ToList();
			var first = registers[0];
			var fields = new List<string>
			{
				$"name: {TsStringLiteral(GetString(group, "label")!)}",
				$"address: {GetInt(first, "address")}",
				$"bank: {TsStringLiteral(bank)}",
			};
			var description = GetString(group, "description") ?? GetString(first, "description");
			if (description is not null) fields.Add($"description: {TsStringLiteral(description)}");
			var unit = GetString(first, "unit");
			if (unit is not null) fields.Add($"unit: {TsStringLiteral(unit)}");
			if (GetBool(group, "signed")) fields.Add("signed: true");
			fields.Add($"display: {TsStringLiteral(GetString(group, "display") ?? "decimal")}");
			fields.Add($"combine: {{ count: {registers.Count} }}");
			return $"{{ {string.Join(", ", fields)} }}";
		}

		{
			var fields = new List<string>
			{
				$"name: {TsStringLiteral(GetString(entry, "name")!)}",
				$"address: {GetInt(entry, "address")}",
				$"bank: {TsStringLiteral(bank)}",
			};
			var description = GetString(entry, "description");
			if (description is not null) fields.Add($"description: {TsStringLiteral(description)}");
			var unit = GetString(entry, "unit");
			if (unit is not null) fields.Add($"unit: {TsStringLiteral(unit)}");
			var gpio = GetString(entry, "gpio");
			if (gpio is not null) fields.Add($"gpio: {TsStringLiteral(gpio)}");
			if (GetBool(entry, "signed")) fields.Add("signed: true");
			if (entry.TryGetProperty("i2c", out var i2c) && i2c.ValueKind == JsonValueKind.Object)
			{
				var i2cFields = new List<string> { $"bus: {TsStringLiteral(GetString(i2c, "bus")!)}" };
				var irqPin = GetString(i2c, "irqPin");
				if (irqPin is not null) i2cFields.Add($"irqPin: {TsStringLiteral(irqPin)}");
				fields.Add($"i2c: {{ {string.Join(", ", i2cFields)} }}");
			}
			var min = GetInt(entry, "min");
			if (min is not null) fields.Add($"min: {min}");
			var max = GetInt(entry, "max");
			if (max is not null) fields.Add($"max: {max}");
			fields.Add($"display: {TsStringLiteral(GetString(entry, "display") ?? "decimal")}");
			return $"{{ {string.Join(", ", fields)} }}";
		}
	}

	private static string GenerateTs(JsonElement schema)
	{
		var orderedByOriginalLayout = new List<(string Title, string Bank, JsonElement Registers)>();
		foreach (var region in schema.GetProperty("banks").GetProperty("input").GetProperty("regions").EnumerateArray())
		{
			var registers = region.GetProperty("registers");
			if (registers.GetArrayLength() > 0) orderedByOriginalLayout.Add((GetString(region, "title")!, "input", registers));
		}
		foreach (var region in schema.GetProperty("banks").GetProperty("holding").GetProperty("regions").EnumerateArray())
		{
			var registers = region.GetProperty("registers");
			if (registers.GetArrayLength() > 0) orderedByOriginalLayout.Add((GetString(region, "title")!, "holding", registers));
		}

		var regionsSource = string.Join(",\n", orderedByOriginalLayout.Select(region =>
		{
			var regsSource = string.Join(",\n", region.Registers.EnumerateArray().Select(entry => "\t\t\t" + GenTsRegister(entry, region.Bank)));
			return $"\t{{\n\t\ttitle: {TsStringLiteral(region.Title)},\n\t\tregisters: [\n{regsSource}\n\t\t]\n\t}}";
		}));

		var inputRegions = schema.GetProperty("banks").GetProperty("input").GetProperty("regions").EnumerateArray().ToList();
		var holdingRegions = schema.GetProperty("banks").GetProperty("holding").GetProperty("regions").EnumerateArray().ToList();
		var lastInputRegion = inputRegions[^1];
		var lastHoldingRegion = holdingRegions[^1];
		var inputCount = GetInt(lastInputRegion, "endAddress")!.Value + 1;
		var holdingCount = GetInt(lastHoldingRegion, "endAddress")!.Value + 1;

		return $$"""
			// GENERIERT von builder/Phases/ReadModbusRegisterMap.cs aus register-map.json -- nicht von Hand
			// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend die
			// Phase "ReadModbusRegisterMapAndGenerateFiles" erneut ausfuehren.

			export type RegisterBank = "input" | "holding";
			// Eine Darstellungsform fuers Lesen UND Schreiben (s. register-panel.ts):
			//   - "decimal" (Default): Zahl; bei Holding-Registern generisches Zahlenfeld + Schreiben-Button.
			//   - "hex" / "binary": Zahl als 0x.../0b...-String; Schreiben wie "decimal".
			//   - "bool": bei Input-Registern grau/gruen eingefaerbtes Badge (0=aus/1=an), bei
			//     Holding-Registern zusaetzlich ein Toggle-Switch zum Schreiben.
			//   - "bool-error": wie "bool", aber grau/rot statt grau/gruen -- fuer Register, bei denen 1
			//     einen Fehler-/Alarmzustand meldet statt eines neutralen "an" (z.B. PWR_ALERT).
			//   - "status": Fehlercode-Konvention (0 = ok, ungleich 0 = Fehler) -- 0 grau/neutral, jeder
			//     andere Wert rot MIT der tatsaechlichen Zahl (nicht nur "1"), fuer *_STATUS-Register, die
			//     kuenftig auch verschiedene Fehlercodes tragen koennten (aktuell ueberall nur 0/1).
			//   - "range": Slider (braucht min/max); nur fuer Holding-Register sinnvoll.
			export type RegisterDisplay = "decimal" | "hex" | "binary" | "bool" | "bool-error" | "status" | "range";

			export interface RegisterDef {
				name: string;
				address: number;
				bank: RegisterBank;
				display: RegisterDisplay;
				description?: string;
				unit?: string;
				signed?: boolean;
				gpio?: string;
				i2c?: { bus: string; irqPin?: string };
				min?: number;
				max?: number;
				// Fasst "count" aufeinanderfolgende Register (ab "address", MSB zuerst) zu einem
				// gemeinsamen Anzeigewert zusammen (z.B. 6 Register -> eine 96-Bit Chip-ID als Hex-String,
				// oder 2 Register -> ein 32-Bit-Zaehlerstand). Nur lesend, nur sinnvoll bei Input-Registern.
				combine?: { count: number };
			}

			export interface RegisterRegion {
				title: string;
				registers: RegisterDef[];
			}

			export const REGIONS: RegisterRegion[] = [
			{{regionsSource}}
			];

			// Muss mit ModbusRegisters::INPUT_REGISTER_MAX_INDEX / HOLDING_REGISTER_MAX_INDEX
			// (generated/register_maxindex.inc) uebereinstimmen -- bestimmt, wie viele Werte
			// /api/registers liefert.
			export const INPUT_REGISTER_COUNT = {{inputCount}}; // Index 0..{{GetInt(lastInputRegion, "endAddress")}}
			export const HOLDING_REGISTER_COUNT = {{holdingCount}}; // Index 0..{{GetInt(lastHoldingRegion, "endAddress")}}

			""";
	}

	public static void Run(string? explicitBoardId)
	{
		var schemaPath = Path.Combine(Paths.RootDir, "register-map.json");
		using var doc = JsonDocument.Parse(File.ReadAllText(schemaPath));
		var schema = doc.RootElement;

		var inputInc = GenerateInputInc(schema);
		var holdingInc = GenerateHoldingInc(schema);
		var maxIndexInc = GenerateMaxIndexInc(schema);
		var ts = GenerateTs(schema);

		var boardId = BoardContext.ResolveBoardId(explicitBoardId);
		var coreOut = boardId is not null ? BoardArchive.BoardGeneratedDir(boardId) : Paths.CoreGeneratedDir;
		var webOut = boardId is not null ? BoardArchive.BoardGeneratedDir(boardId) : Paths.WebGeneratedDir;

		Directory.CreateDirectory(coreOut);
		Directory.CreateDirectory(webOut);
		File.WriteAllText(Path.Combine(coreOut, "register_input.inc"), inputInc);
		File.WriteAllText(Path.Combine(coreOut, "register_holding.inc"), holdingInc);
		File.WriteAllText(Path.Combine(coreOut, "register_maxindex.inc"), maxIndexInc);
		File.WriteAllText(Path.Combine(webOut, "register-map.ts"), ts);

		if (boardId is not null)
		{
			Console.WriteLine($"{schemaPath} -> Board-Archiv ({boardId})");
		}
		else
		{
			Console.WriteLine(
				"Kein Board-Kontext bekannt -- schreibe Register-Map direkt nach Core/generated bzw. web/generated (kein Archiv-Eintrag).");
		}
	}
}
