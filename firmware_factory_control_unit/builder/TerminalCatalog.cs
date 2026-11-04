using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Text.RegularExpressions;

namespace Builder;

// The command catalog of the web UI's Terminal page (schema/terminal/commands.json) for both
// sides: web/generated/terminal-commands.ts (completion, help, parameter tooltip; compiled into
// the web app) and generated/terminal_commands.hh (src/terminal.cpp: a handler per command,
// an answer per lookup, both checked at compile time).
public static partial class TerminalCatalog
{
	public sealed class Value
	{
		public string value { get; set; } = "";
		public string doc { get; set; } = "";
	}

	public sealed class Param
	{
		public string name { get; set; } = "";
		public string doc { get; set; } = "";
		public bool optional { get; set; }
		[JsonConverter(typeof(ValueListConverter))] public List<Value>? values { get; set; }
		public string? lookup { get; set; }
		public string? source { get; set; }
	}

	public sealed class Command
	{
		public string name { get; set; } = "";
		public string doc { get; set; } = "";
		public bool client { get; set; }
		public List<Param> @params { get; set; } = [];
	}

	public sealed class Lookup
	{
		public string name { get; set; } = "";
		public string doc { get; set; } = "";
	}

	public sealed class Catalog
	{
		public List<Lookup> lookups { get; set; } = [];
		public List<Command> commands { get; set; } = [];
	}

	// "values": plain strings or {value, doc}
	private sealed class ValueListConverter : JsonConverter<List<Value>>
	{
		public override List<Value> Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
		{
			var list = new List<Value>();
			using var doc = JsonDocument.ParseValue(ref reader);
			foreach (var e in doc.RootElement.EnumerateArray())
			{
				list.Add(e.ValueKind == JsonValueKind.String
					? new Value { value = e.GetString()! }
					: new Value { value = e.GetProperty("value").GetString()!, doc = e.TryGetProperty("doc", out var d) ? d.GetString()! : "" });
			}
			return list;
		}

		public override void Write(Utf8JsonWriter writer, List<Value> value, JsonSerializerOptions options) =>
			JsonSerializer.Serialize(writer, value.ToArray(), options);
	}

	private static readonly string[] Sources = ["commands", "registers"];
	private static readonly HashSet<string> CppKeywords = ["and", "auto", "bool", "break", "case", "catch", "char", "class",
		"const", "continue", "default", "delete", "do", "double", "else", "enum", "explicit", "export", "extern", "false",
		"float", "for", "friend", "goto", "if", "inline", "int", "long", "mutable", "namespace", "new", "not", "operator",
		"or", "private", "protected", "public", "register", "return", "short", "signed", "sizeof", "static", "struct",
		"switch", "template", "this", "throw", "true", "try", "typedef", "typename", "union", "unsigned", "using",
		"virtual", "void", "volatile", "while", "xor"];

	[GeneratedRegex("^[a-z][a-z0-9_]*$")]
	private static partial Regex Identifier();

	public static void Generate(string schemaFile, string cppOutputDir, string tsOutputDir)
	{
		var catalog = JsonSerializer.Deserialize<Catalog>(File.ReadAllText(schemaFile))
			?? throw new InvalidOperationException($"{schemaFile}: empty");
		Validate(catalog, schemaFile);
		Directory.CreateDirectory(cppOutputDir);
		Directory.CreateDirectory(tsOutputDir);
		WriteIfChanged(Path.Combine(cppOutputDir, "terminal_commands.hh"), Cpp(catalog));
		WriteIfChanged(Path.Combine(tsOutputDir, "terminal-commands.ts"), TypeScript(catalog));
	}

	public static string Usage(Command c) =>
		string.Join(" ", [c.name, .. c.@params.Select(p => p.optional ? $"[<{p.name}>]" : $"<{p.name}>")]);

	private static void Validate(Catalog catalog, string file)
	{
		void Fail(string what) => throw new InvalidOperationException($"{file}: {what}");
		void Name(string kind, string name)
		{
			if (!Identifier().IsMatch(name) || CppKeywords.Contains(name))
				Fail($"{kind} '{name}': lower-case identifier expected (and no C++ keyword)");
		}

		var lookups = new HashSet<string>();
		foreach (var l in catalog.lookups)
		{
			Name("lookup", l.name);
			if (!lookups.Add(l.name)) Fail($"lookup '{l.name}' twice");
		}
		if (lookups.Count > 255) Fail("more than 255 lookups");

		var commands = new HashSet<string>();
		foreach (var c in catalog.commands)
		{
			Name("command", c.name);
			if (!commands.Add(c.name)) Fail($"command '{c.name}' twice");
			if (c.doc.Length == 0) Fail($"command '{c.name}': doc missing");
			bool optionalSeen = false;
			foreach (var p in c.@params)
			{
				var where = $"command '{c.name}', parameter '{p.name}'";
				if (p.name.Length == 0 || p.doc.Length == 0) Fail($"{where}: name and doc required");
				if (optionalSeen && !p.optional) Fail($"{where}: a required parameter after an optional one");
				optionalSeen |= p.optional;
				int kinds = (p.values is null ? 0 : 1) + (p.lookup is null ? 0 : 1) + (p.source is null ? 0 : 1);
				if (kinds > 1) Fail($"{where}: at most one of values, lookup, source");
				if (p.lookup is not null && !lookups.Contains(p.lookup)) Fail($"{where}: unknown lookup '{p.lookup}'");
				if (p.source is not null && !Sources.Contains(p.source))
					Fail($"{where}: unknown source '{p.source}' (known: {string.Join(", ", Sources)})");
				if (p.values is not null)
				{
					if (p.values.Count == 0) Fail($"{where}: empty values");
					if (p.values.Any(v => v.value.Length == 0 || v.value.Contains(' '))) Fail($"{where}: a value is empty or contains a space");
					if (p.values.Select(v => v.value).Distinct().Count() != p.values.Count) Fail($"{where}: a value twice");
				}
			}
		}
	}

	private static string CppString(string s) =>
		"\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";

	private static string Cpp(Catalog catalog)
	{
		var device = catalog.commands.Where(c => !c.client).ToList();
		var sb = new StringBuilder();
		sb.Append("""
			// GENERATED by the builder (builder/TerminalCatalog.cs) from schema/terminal/commands.json -- do not edit.
			// The commands the device runs (the client ones, e.g. help and clear, stay in the browser) and the
			// value lookups the browser asks for (terminal.LookupRequest.lookup = Lookup).
			#pragma once

			#include <array>
			#include <cstddef>
			#include <cstdint>
			#include <string_view>

			namespace terminal_catalog {

			""");
		sb.Append("\nenum class Command : std::uint8_t {\n");
		foreach (var c in device) sb.Append($"    {c.name},\n");
		sb.Append("};\n\n");
		sb.Append("struct CommandInfo {\n    Command id;\n    std::string_view name;\n    std::string_view usage;\n};\n\n");
		sb.Append($"inline constexpr std::array<CommandInfo, {device.Count}> commands{{{{\n");
		foreach (var c in device) sb.Append($"    {{Command::{c.name}, {CppString(c.name)}, {CppString(Usage(c))}}},\n");
		sb.Append("}};\n\n");
		sb.Append("enum class Lookup : std::uint8_t {\n");
		foreach (var l in catalog.lookups) sb.Append($"    {l.name},  // {l.doc}\n");
		sb.Append("};\n\n");
		sb.Append($"inline constexpr std::size_t lookup_count = {catalog.lookups.Count};\n\n");
		sb.Append("}  // namespace terminal_catalog\n");
		return sb.ToString();
	}

	private static string TypeScript(Catalog catalog)
	{
		var lookupIds = catalog.lookups.Select((l, i) => (l.name, i)).ToDictionary(x => x.name, x => x.i);
		var options = new JsonSerializerOptions
		{
			WriteIndented = true,
			DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
			Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
		};
		var commands = catalog.commands.Select(c => new
		{
			c.name,
			c.doc,
			c.client,
			usage = Usage(c),
			@params = c.@params.Select(p => new
			{
				p.name,
				p.doc,
				p.optional,
				p.values,
				lookup = p.lookup is null ? (int?)null : lookupIds[p.lookup],
				p.source,
			}),
		});
		var sb = new StringBuilder();
		sb.Append("""
			// GENERATED by the builder (builder/TerminalCatalog.cs) from schema/terminal/commands.json -- do not edit.

			export interface TerminalValue {
				value: string;
				doc: string;
			}

			/** Values computed in the browser: command names, register addresses of the bank given before. */
			export type TerminalSource = "commands" | "registers";

			export interface TerminalParam {
				name: string;
				doc: string;
				optional: boolean;
				values?: TerminalValue[];
				/** fetched live from the device: terminal.LookupRequest.lookup */
				lookup?: number;
				source?: TerminalSource;
			}

			export interface TerminalCommand {
				name: string;
				doc: string;
				/** runs in the browser (help, clear) */
				client: boolean;
				usage: string;
				params: TerminalParam[];
			}

			export interface TerminalLookup {
				name: string;
				doc: string;
			}


			""");
		sb.Append("export const TERMINAL_LOOKUPS: readonly TerminalLookup[] = ");
		sb.Append(JsonSerializer.Serialize(catalog.lookups, options)).Append(";\n\n");
		sb.Append("export const TERMINAL_COMMANDS: readonly TerminalCommand[] = ");
		sb.Append(JsonSerializer.Serialize(commands, options)).Append(";\n");
		return sb.ToString().Replace("\r\n", "\n");
	}

	private static void WriteIfChanged(string path, string content)
	{
		content = content.Replace("\r\n", "\n");
		if (File.Exists(path) && File.ReadAllText(path) == content) return;
		File.WriteAllText(path, content, new UTF8Encoding(false));
		Console.WriteLine($"generated {path}");
	}
}
