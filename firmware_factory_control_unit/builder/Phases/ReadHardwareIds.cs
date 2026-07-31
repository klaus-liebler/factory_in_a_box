// Phasen "ReadHardwareIDsAndGenerateFilesAndCertsLazy"/"...Forced" (s. docs/build-process.md
// Abschnitt 2) -- schreibt ausschliesslich ins Board-Archiv (stm32_boards/<boardId>/generated/).
using System.Text.Json.Serialization;

namespace Builder.Phases;

public sealed record CertificateInfo(
	[property: JsonPropertyName("issuer")] string Issuer,
	// Unix-Epoch-Sekunden statt vorformatierter Strings -- Formatierung/Zeitzone passiert
	// ausschliesslich im Browser.
	[property: JsonPropertyName("issuedAtEpoch")] long IssuedAtEpoch,
	[property: JsonPropertyName("validUntilEpoch")] long ValidUntilEpoch);

// Inhalt von device-ids.json -- wird von ReadGitStatus fuer build-info.ts/firmware_constants.hh
// wiederverwendet.
public sealed record DeviceIdentity(
	[property: JsonPropertyName("hostname")] string Hostname,
	[property: JsonPropertyName("chipUid")] string ChipUid,
	[property: JsonPropertyName("usbNcmMac")] string UsbNcmMac,
	[property: JsonPropertyName("ethMac")] string EthMac,
	// Aufgeloest per Board-Archiv-Nachschlag (board.json, s. BoardStore.cs), Fallback
	// BuilderOptions.DefaultBoardTypeName fuer ein noch nie erfolgreich geflashtes Board (s. Run()
	// unten) -- gleichzeitig Anzeigename in Firmware/Web-UI, kein separater BOARD_NAME-String mehr.
	[property: JsonPropertyName("boardName")] string BoardName,
	[property: JsonPropertyName("certificate")] CertificateInfo Certificate);

public sealed record HardwareIdsResult(string BoardId, string Hostname, string ChipUid);

public static class ReadHardwareIds
{
	// UID_BASE, siehe stm32h563xx.h -- chip-fest, keine Konfiguration/Umgebungsvariable noetig.
	private const string UidAddress = "0x08FFF800";

	private readonly record struct UidResult(string Uid, uint[] Words);

	private static UidResult ReadUniqueId()
	{
		var cliPath = BuilderOptions.Current.ResolveStm32ProgrammerCli();
		string output;
		try
		{
			output = ProcessRunner.Run(cliPath, ["-c", "port=SWD", "-r32", UidAddress, "12"]);
		}
		catch (ProcessException ex)
		{
			// Gezielt die zwei haeufigsten Ursachen erkennen und eine kurze, umsetzbare Meldung
			// ausgeben, statt .NET's rohen Prozess-Fehler durchzureichen.
			if (ex.ExitCode == -1)
			{
				throw new InvalidOperationException(
					$"STM32_Programmer_CLI.exe nicht gefunden unter:\n  {cliPath}\n" +
					"Pruefe die STM32CubeProgrammer-Installation bzw. die Umgebungsvariable STM32_PRG_PATH " +
					"(darf auf die .exe selbst oder deren bin/-Ordner zeigen).");
			}
			var details = (ex.StdErr.Length > 0 ? ex.StdErr : ex.StdOut).Trim();
			throw new InvalidOperationException(
				$"STM32_Programmer_CLI konnte das Board nicht auslesen (Exit-Code {ex.ExitCode}).\n" +
				"Moegliche Ursachen: ST-LINK nicht angeschlossen/nicht mit Strom versorgt, oder bereits von " +
				"einer anderen Anwendung belegt (z.B. eine laufende Debug-Sitzung in STM32CubeIDE -- die " +
				"beenden und erneut versuchen).\n" +
				(details.Length > 0 ? $"Ausgabe des Tools:\n{details}" : ""));
		}

		var match = System.Text.RegularExpressions.Regex.Match(
			output, $@"{System.Text.RegularExpressions.Regex.Escape(UidAddress)}\s*:\s*([0-9A-Fa-f]{{8}})\s+([0-9A-Fa-f]{{8}})\s+([0-9A-Fa-f]{{8}})");
		if (!match.Success)
		{
			throw new InvalidOperationException($"Konnte Unique-ID nicht aus STM32_Programmer_CLI-Ausgabe lesen:\n{output}");
		}
		// words[0..2] entsprechen exakt HAL_GetUIDw0()/_w1()/_w2() -- wichtig fuer
		// ComputeUsbNcmMac()/ComputeEthMac() unten, deren Bit-Layout exakt zu App::SetupBeforeThreadX()'
		// Chip-UID-Mismatch-Check (app.cc) passen muss.
		var words = new[]
		{
			Convert.ToUInt32(match.Groups[1].Value, 16),
			Convert.ToUInt32(match.Groups[2].Value, 16),
			Convert.ToUInt32(match.Groups[3].Value, 16),
		};
		var uid = string.Concat(match.Groups[1].Value, match.Groups[2].Value, match.Groups[3].Value).ToUpperInvariant();
		return new UidResult(uid, words);
	}

	// Locally-administered MAC (Byte 0 = 0x02, s. IEEE 802-2014 Tabelle 8-1) fuer die virtuelle
	// USB-CDC-NCM-NIC.
	private static byte[] ComputeUsbNcmMac(uint[] words)
	{
		var (w0, w1, w2) = (words[0], words[1], words[2]);
		return [0x02, (byte)(w0 >> 24), (byte)w0, (byte)(w1 >> 24), (byte)w1, (byte)w2];
	}

	// Bitgenauer Nachbau des USER-CODE-Blocks "MACADDRESS" in main.c's MX_ETH_Init() -- MUSS mit
	// dieser Stelle synchron bleiben, falls sich dort je etwas aendert. Andere Adresse als
	// ComputeUsbNcmMac() oben, damit die physische Ethernet-NIC (LAN8742) und die virtuelle
	// USB-CDC-NCM-NIC nicht dieselbe MAC tragen.
	private static byte[] ComputeEthMac(uint[] words)
	{
		var (w0, w1, w2) = (words[0], words[1], words[2]);
		var folded = w0 ^ w1 ^ w2;
		return
		[
			0x02,
			(byte)(folded >> 24),
			(byte)(folded >> 16),
			(byte)(folded >> 8),
			(byte)folded,
			(byte)(((w0 >> 8) ^ (w1 >> 16) ^ (w2 >> 24)) & 0xff),
		];
	}

	private static string ToHex2(byte b) => b.ToString("X2");

	private static (string KeyPath, string CrtPath) OpensslCreateCertificate(string hostname, string boardDir)
	{
		var keyPath = Path.Combine(boardDir, $"{hostname}.pem.key");
		var csrPath = Path.Combine(boardDir, $"{hostname}.csr");
		var crtPath = Path.Combine(boardDir, $"{hostname}.pem.crt");
		var extPath = Path.Combine(Path.GetTempPath(), $"{hostname}.ext.cnf");

		// ECDSA (P-256/prime256v1) statt RSA-2048: deutlich billigere Signaturberechnung
		// (Stack + CPU-Zeit) beim TLS-Handshake auf dem STM32H573.
		ProcessRunner.Run("openssl", ["ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", keyPath]);
		ProcessRunner.Run("openssl", ["req", "-new", "-key", keyPath, "-out", csrPath, "-subj", $"{BuilderOptions.Current.SubjectPrefix}/CN={hostname}"]);

		// Eigene, pro-Board generierte ext-Datei statt der geteilten certificates/openssl.cnf zu
		// mutieren -- die traegt das subjectAltName sonst fest fuer nur EIN Board. Zwei
		// DNS-SAN-Eintraege ("<hostname>" und "<hostname>.local") + ein IP-SAN fuer die
		// USB-CDC-NCM-NIC (192.168.173.1, keine eigene mDNS-Instanz, s. net_setup.cpp).
		File.WriteAllText(extPath,
			"authorityKeyIdentifier=keyid,issuer\n" +
			"basicConstraints=CA:FALSE\n" +
			"keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment\n" +
			"extendedKeyUsage = serverAuth\n" +
			"subjectAltName = @alt_names\n\n" +
			"[alt_names]\n" +
			$"DNS.1 = {hostname}\n" +
			$"DNS.2 = {hostname}.local\n" +
			"IP.1 = 192.168.173.1\n");

		ProcessRunner.Run("openssl",
		[
			"x509", "-req", "-in", csrPath,
			"-CA", BuilderOptions.Current.CaCert,
			"-CAkey", BuilderOptions.Current.CaKey,
			"-CAcreateserial",
			"-out", crtPath,
			"-days", BuilderOptions.Current.CertDays,
			"-sha256",
			"-extfile", extPath,
		]);

		return (keyPath, crtPath);
	}

	private static byte[] PemToDer(string pemPath, string kind)
	{
		// "openssl ec -outform DER" schreibt das flache SEC1/RFC-5915-ECPrivateKey-SEQUENCE --
		// exakt das Format, das nx_secure_x509_certificate_initialize() mit
		// NX_SECURE_X509_KEY_TYPE_EC_DER erwartet.
		string[] args = kind == "cert" ? ["x509", "-in", pemPath, "-outform", "DER"] : ["ec", "-in", pemPath, "-outform", "DER"];
		return ProcessRunner.RunBinary("openssl", args);
	}

	// OpenSSL's Default-Datumsformat, z.B. "Jul 24 08:44:47 2026 GMT" (einstellige Tage mit
	// fuehrendem Leerzeichen statt "0", z.B. "Jul  2 ..."). Explizites ParseExact statt
	// DateTimeOffset.Parse (das dieses Format anders als JS's Date-Parser nicht zuverlaessig
	// erkennt) -- mehrfache Leerzeichen vorab auf eins normalisieren, damit "d" (nicht "dd") passt.
	private static long ToEpochSeconds(string opensslDate)
	{
		var normalized = System.Text.RegularExpressions.Regex.Replace(opensslDate.Trim(), @"\s+", " ");
		var dt = DateTime.ParseExact(
			normalized, "MMM d HH:mm:ss yyyy 'GMT'",
			System.Globalization.CultureInfo.InvariantCulture,
			System.Globalization.DateTimeStyles.AssumeUniversal | System.Globalization.DateTimeStyles.AdjustToUniversal);
		return new DateTimeOffset(dt, TimeSpan.Zero).ToUnixTimeSeconds();
	}

	// Liest Aussteller + Gueltigkeitszeitraum direkt aus dem (wiederverwendeten oder frisch
	// erzeugten) Zertifikat -- keine separate Buchfuehrung noetig, das Zertifikat selbst ist die
	// Quelle der Wahrheit.
	private static CertificateInfo ReadCertificateInfo(string crtPath)
	{
		var output = ProcessRunner.Run("openssl", ["x509", "-in", crtPath, "-noout", "-issuer", "-startdate", "-enddate"]);
		var issuerMatch = System.Text.RegularExpressions.Regex.Match(output, @"^issuer=(.+)$", System.Text.RegularExpressions.RegexOptions.Multiline);
		var startMatch = System.Text.RegularExpressions.Regex.Match(output, @"^notBefore=(.+)$", System.Text.RegularExpressions.RegexOptions.Multiline);
		var endMatch = System.Text.RegularExpressions.Regex.Match(output, @"^notAfter=(.+)$", System.Text.RegularExpressions.RegexOptions.Multiline);
		if (!issuerMatch.Success || !startMatch.Success || !endMatch.Success)
		{
			throw new InvalidOperationException($"Konnte Zertifikatsinfo nicht aus openssl-Ausgabe lesen:\n{output}");
		}
		return new CertificateInfo(
			issuerMatch.Groups[1].Value.Trim(),
			ToEpochSeconds(startMatch.Groups[1].Value.Trim()),
			ToEpochSeconds(endMatch.Groups[1].Value.Trim()));
	}

	private static string RenderDeviceIdsHh(string hostname, uint[] words, byte[] netMac, byte[] ethMac)
	{
		var (w0, w1, w2) = (words[0], words[1], words[2]);
		var serialString = string.Concat(w0.ToString("X8"), w1.ToString("X8"), w2.ToString("X8"));
		var netMacString = string.Concat(netMac.Select(ToHex2));
		var netMacInit = string.Join(", ", netMac.Select(b => $"0x{ToHex2(b)}"));
		var ethMacInit = string.Join(", ", ethMac.Select(b => $"0x{ToHex2(b)}"));

		return $$"""
			#pragma once
			// Generiert von builder/Phases/ReadHardwareIds.cs -- nicht von Hand editieren. Neu erzeugen
			// per "dotnet run --project builder -- ReadHardwareIDsAndGenerateFilesAndCertsLazy"
			// bzw. "...Forced" (liest/erzeugt Zertifikat + Board-Identitaet fuer das aktuell am ST-Link
			// angeschlossene Board). Alles hier ist fix pro physischem Board (aus dessen 96-Bit Chip-UID
			// abgeleitet) -- deshalb als Konstanten eincompiliert statt bei jedem Boot per snprintf()/
			// Bit-Fummelei neu berechnet.
			// App::SetupBeforeThreadX() (app.cc) prueft beim Start, ob DEVICE_CHIP_UID_W0/1/2 noch zur
			// tatsaechlichen Chip-UID des angeschlossenen Boards passen, und haelt sonst an
			// (Error_Handler()) -- verhindert, dass fuer Board A gebaute Firmware unbemerkt auf Board B
			// mit dessen falscher MAC-Adresse/Seriennummer laeuft.
			//
			// Sowohl von C++-Uebersetzungseinheiten (app.cc/net_setup.cpp/webserver.cpp) ALS AUCH von
			// main.c (reines C, CubeMX-generiert, s. dortiger MX_ETH_Init()/USER-CODE-Block MACADDRESS)
			// eingebunden -- DEVICE_IDS_CONST expandiert deshalb je nach Sprache zu constexpr (C++) oder
			// static const (C, kein constexpr vor C23), exakt einmal definiert statt die Werte doppelt
			// pflegen zu muessen.
			#ifdef __cplusplus
			#include <cstdint>
			#define DEVICE_IDS_CONST constexpr
			#else
			#include <stdint.h>
			#define DEVICE_IDS_CONST static const
			#endif
			DEVICE_IDS_CONST char DEVICE_HOSTNAME[] = "{{hostname}}";
			DEVICE_IDS_CONST uint32_t DEVICE_CHIP_UID_W0 = 0x{{w0.ToString("X8")}};
			DEVICE_IDS_CONST uint32_t DEVICE_CHIP_UID_W1 = 0x{{w1.ToString("X8")}};
			DEVICE_IDS_CONST uint32_t DEVICE_CHIP_UID_W2 = 0x{{w2.ToString("X8")}};
			// 24 Hex-Ziffern (3x uint32_t Chip-UID) -- USB-Seriennummer-Stringdeskriptor.
			DEVICE_IDS_CONST char DEVICE_USB_SERIAL_STRING[] = "{{serialString}}";
			// Locally-administered MAC (Byte 0 = 0x02, s. IEEE 802-2014 Tabelle 8-1) fuer die virtuelle
			// USB-CDC-NCM-NIC.
			DEVICE_IDS_CONST uint8_t DEVICE_USB_NCM_MAC[6] = {{{netMacInit}}};
			// Dieselben 6 Byte als 12 Grossbuchstaben-Hexziffern ohne Trennzeichen -- CDC1.2 Ethernet
			// Networking Functional Descriptor's iMACAddress-String-Format (s. usbd_device.c).
			DEVICE_IDS_CONST char DEVICE_USB_NCM_MAC_STRING[] = "{{netMacString}}";
			// Locally-administered MAC fuer die physische Ethernet-NIC (LAN8742, s. main.c MX_ETH_Init()) --
			// ANDERE Adresse als DEVICE_USB_NCM_MAC oben.
			DEVICE_IDS_CONST uint8_t DEVICE_ETH_MAC[6] = {{{ethMacInit}}};
			#undef DEVICE_IDS_CONST

			""";
	}

	public static HardwareIdsResult Run(bool forced)
	{
		Console.WriteLine("Lese Chip-ID vom angeschlossenen Board...");
		var (uid, words) = ReadUniqueId();
		var shortId = uid[^6..].ToLowerInvariant();
		var hostname = $"factory-box-{shortId}";
		var netMac = ComputeUsbNcmMac(words);
		var ethMac = ComputeEthMac(words);
		var boardId = $"{shortId}_{uid.ToLowerInvariant()}";
		// Board-Type-Zuordnung per Board-Archiv (board.json) statt (wie frueher) aus dem
		// CMake-Preset abgeleitet -- ein noch nie erfolgreich geflashtes Board faellt auf den
		// konfigurierten Default zurueck (s. BoardStore.TryGetBoardTypeName).
		var boardName = BoardStore.TryGetBoardTypeName(boardId) ?? BuilderOptions.Current.DefaultBoardTypeName;
		Console.WriteLine(
			$"Chip-ID: {uid} -> Hostname: {hostname}, Board-Type: {boardName}, USB-NCM-MAC: {string.Join(":", netMac.Select(ToHex2))}, " +
			$"Ethernet-MAC: {string.Join(":", ethMac.Select(ToHex2))}");

		var boardDir = BoardArchive.BoardArchiveDir(boardId);
		var generatedDir = BoardArchive.BoardGeneratedDir(boardId);
		Directory.CreateDirectory(boardDir);
		Directory.CreateDirectory(generatedDir);

		var existingKey = Path.Combine(boardDir, $"{hostname}.pem.key");
		var existingCrt = Path.Combine(boardDir, $"{hostname}.pem.crt");

		string keyPath, crtPath;
		if (!forced && File.Exists(existingKey) && File.Exists(existingCrt))
		{
			Console.WriteLine($"Vorhandenes Zertifikat gefunden (Lazy): {boardDir}");
			keyPath = existingKey;
			crtPath = existingCrt;
		}
		else
		{
			Console.WriteLine($"{(forced ? "Forced" : "Kein Zertifikat fuer dieses Board gefunden")} -- erzeuge neues Zertifikat in: {boardDir}");
			(keyPath, crtPath) = OpensslCreateCertificate(hostname, boardDir);
		}

		var certDer = PemToDer(crtPath, "cert");
		var keyDer = PemToDer(keyPath, "key");
		var certificate = ReadCertificateInfo(crtPath);
		File.WriteAllBytes(Path.Combine(generatedDir, "device_certificate.der"), certDer);
		File.WriteAllBytes(Path.Combine(generatedDir, "device_key.der"), keyDer);
		File.WriteAllText(Path.Combine(generatedDir, "device_ids.hh"), RenderDeviceIdsHh(hostname, words, netMac, ethMac));

		// Maschinenlesbares Gegenstueck zu device_ids.hh (analog zu gitstatus.json fuer
		// gitconstants.hh) -- damit ReadGitStatus Hostname/Chip-UID/MAC-Adressen/Zertifikatsdaten
		// fuer build-info.ts nicht per Regex aus dem C++-Header/DER-Binary zurueckparsen muss.
		var deviceIds = new DeviceIdentity(
			hostname,
			uid,
			string.Join(":", netMac.Select(ToHex2)),
			string.Join(":", ethMac.Select(ToHex2)),
			boardName,
			certificate);
		File.WriteAllText(
			Path.Combine(generatedDir, "device-ids.json"),
			System.Text.Json.JsonSerializer.Serialize(deviceIds, Json.Options) + "\n");

		BoardContext.WriteCachedBoardId(boardId);

		Console.WriteLine(
			$"Geschrieben: {generatedDir} (device_ids.hh, device-ids.json, device_certificate.der [{certDer.Length} B], device_key.der [{keyDer.Length} B])");
		return new HardwareIdsResult(boardId, hostname, uid);
	}
}
