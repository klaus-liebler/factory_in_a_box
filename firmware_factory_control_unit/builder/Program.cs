using Builder;
using FirmwareBuilder.Common;

BuildStepRunner.Run(args, a => Stm32Toolchain.CreateBuildContext(a, x => new Stm32BuildContext(x)), typeof(Builder.Program));

namespace Builder
{
	public static class Program
	{
		[BuildStep]
		public static void Info(IBuildContextStm32 ctx) => Stm32ConsoleReport.WriteBoardInfo(ctx);

		[BuildStep]
		public static void GitStatus(IBuildContextStm32 ctx) => BuilderConsoleReport.WriteGitStatus(ctx.Git);

		[BuildStep]
		public static void PrepareContextWithRealHardware(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.ReadHardwareIds(
				ctx,
				defaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
				resolveBoardTypeNameByBoardId: boardId => BoardStateStore.TryGetBoardTypeName(BuilderSettings.Current.BoardStorage, boardId));

		// Offline-Pendant: baut die Board-Identitaet ohne angeschlossene Hardware rein aus --board auf
		// (das Flag existierte vorher schon als Override fuer BEREITS archivierte Boards -- hier wird
		// es zum vollwertigen Ersatz des Live-Reads). Schreibt wie PrepareContextWithRealHardware die
		// Cache-Datei, damit nachfolgende Schritte (auch ohne erneutes --board) denselben Kontext sehen.
		[BuildStep]
		public static void PrepareContextWithCommandLineArguments(IBuildContextStm32 ctx)
		{
			if (!Directory.Exists(ctx.BoardArchiveDir))
			{
				throw new InvalidOperationException(
					$"Kein Board-Archiv unter {ctx.BoardArchiveDir} -- \"--board {ctx.BoardUid}\" ist unbekannt (noch nie erfolgreich geflasht?).");
			}
			BoardArchiveContext.WriteCachedBoardId(ctx.BuildDir, ctx.BoardIdCacheFile, ctx.BoardUid);
			Console.WriteLine($"Board-Kontext aus --board uebernommen: {ctx.BoardUid} ({ctx.BoardArchiveDir})");
		}

		[BuildStep]
		public static void GenerateCertificatesLazy(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateCertificates(ctx, force: false, httpsEcLeaf: true);

		[BuildStep]
		public static void GenerateCertificatesForced(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateCertificates(ctx, force: true, httpsEcLeaf: true);

		[BuildStep]
		public static void GenerateDeviceArtifacts(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateDeviceArtifacts(ctx);

		// Web UI login: the board's one-time password from the board archive (created once, reused
		// afterwards; --force-recreate-one-time-web-ui-password makes a new one), its PBKDF2 record for
		// the firmware (generated/device_credentials.hh) and, for a new password, the label.
		[BuildStep]
		public static void GenerateWebCredentials(IBuildContextStm32 ctx)
		{
			ApplyLabelOverrides(ctx);
			var hadPassword = !string.IsNullOrEmpty(ctx.WebAdminPassword);
			var device = WebCredentials.Generate(ctx);
			if ((!hadPassword || Cli.HasFlag(ctx.Args, WebCredentials.ForceFlag) ||
			     BuilderSettings.Current.WebAuth.ForceRecreateOneTimeWebUiPassword) && OperatingSystem.IsWindows())
			{
				DeviceLabel.Write(ctx.BoardsDir, device, Cli.HasFlag(ctx.Args, "--dry-run"));
			}
		}

		// The label again with the stored password, read only from the board archive (never creates a
		// password): the current board (build/.last-board-id), --board <id or prefix> (repeatable) or
		// --all; printed as LabelPrinter configures it (docs/labels.md), --dry-run writes the print job
		// to <BoardsDir>/<board id>/labels/ only.
		[BuildStep]
		public static void PrintLabel(IBuildContextStm32 ctx)
		{
			if (!OperatingSystem.IsWindows())
			{
				throw new PlatformNotSupportedException("the label needs System.Drawing (Windows)");
			}
			var dryRun = Cli.HasFlag(ctx.Args, "--dry-run");
			foreach (var device in SelectLabelBoards(ctx))
			{
				DeviceLabel.Write(BuilderSettings.Current.BoardStorage.BoardsDir, device, dryRun);
			}
		}

		// The labels of several boards on A4 at true scale with cut marks, for any laser printer:
		// <BoardsDir>/labels_A4.pdf (--out <file>). Boards as PrintLabel (current, --board ..., --all),
		// --copies <n> per board.
		[BuildStep]
		public static void PrintLabelSheet(IBuildContextStm32 ctx)
		{
			if (!OperatingSystem.IsWindows())
			{
				throw new PlatformNotSupportedException("the label needs System.Drawing (Windows)");
			}
			var devices = SelectLabelBoards(ctx);
			var o = BuilderSettings.Current.LabelPrinter;
			var copies = int.Parse(Cli.GetOptionalArgValue(ctx.Args, "--copies") ?? "1", System.Globalization.CultureInfo.InvariantCulture);
			var path = Cli.GetOptionalArgValue(ctx.Args, "--out") ?? Labels.BoardArchiveLabels.SheetPath(BuilderSettings.Current.BoardStorage.BoardsDir);
			var pages = Labels.LabelSheet.Write(path, devices, o.SheetCellWidthMm, o.SheetCellHeightMm, copies);
			Console.WriteLine($"A4-Bogen: {path} ({devices.Count} Board(s) x {copies}, {pages} Seite(n), " +
			                  $"Zelle {o.SheetCellWidthMm} x {o.SheetCellHeightMm} mm) -- mit 100 % drucken.");
		}

		// One-off overrides of LabelPrinter from the command line (try a label or printer without
		// editing appsettings.json): --label-type Zebra|BrotherQL|None, --ql-label 62|29x90|17x54,
		// --ql-backend WindowsRaw|WindowsDriver|Tcp|BrotherQlCli|File, --ql-target <queue|host:port|id>.
		private static void ApplyLabelOverrides(IBuildContextStm32 ctx)
		{
			var o = BuilderSettings.Current.LabelPrinter;
			o.Type = Cli.GetOptionalArgValue(ctx.Args, "--label-type") ?? o.Type;
			o.BrotherQL.Label = Cli.GetOptionalArgValue(ctx.Args, "--ql-label") ?? o.BrotherQL.Label;
			o.BrotherQL.Backend = Cli.GetOptionalArgValue(ctx.Args, "--ql-backend") ?? o.BrotherQL.Backend;
			o.BrotherQL.Target = Cli.GetOptionalArgValue(ctx.Args, "--ql-target") ?? o.BrotherQL.Target;
		}

		private static List<WebCredentials.Device> SelectLabelBoards(IBuildContextStm32 ctx)
		{
			ApplyLabelOverrides(ctx);
			var storage = BuilderSettings.Current.BoardStorage;
			var defaults = BuilderSettings.Current.BoardDefaults;
			var all = Cli.HasFlag(ctx.Args, "--all");
			var requested = Cli.GetAllArgValues(ctx.Args, "--board");
			var ids = all ? Labels.BoardArchiveLabels.AllBoardIds(storage.BoardsDir)
				: requested.Count > 0 ? requested.Select(b => Labels.BoardArchiveLabels.Resolve(storage.BoardsDir, b)).Distinct().ToList()
				: [ctx.BoardUid];
			var devices = new List<WebCredentials.Device>();
			foreach (var id in ids)
			{
				var d = Labels.BoardArchiveLabels.TryLoad(storage.BoardsDir, id, defaults.DefaultBoardTypeName, defaults.DefaultBoardTypeVersion, out var whyNot);
				if (d is null)
				{
					if (!all)
					{
						throw new InvalidOperationException($"Board {id}: {whyNot}.");
					}
					Console.WriteLine($"Board {id} uebersprungen: {whyNot}.");
					continue;
				}
				devices.Add(d);
			}
			if (devices.Count == 0)
			{
				throw new InvalidOperationException("Kein Board mit Web-Passwort gefunden.");
			}
			return devices;
		}

		[BuildStep]
		public static void GenerateRegisterAccessFiles(IBuildContextStm32 ctx) =>
			UniversalRegisterAccessBuildService.Run(new UniversalRegisterAccessBuildRequest(
				RootDir: ctx.RootDir,
				DefaultSchemaDirectory: Stm32BuildContext.RegisterMapSchemaDir,
				CoreGeneratedDir: ctx.FirmwareGeneratedDir,
				WebGeneratedDir: ctx.WebGeneratedDir,
				Sources: [],
				OpcUaRootName: "FactoryControlUnit",
				DocsGeneratedDir: Path.Combine(ctx.RootDir, "docs", "generated"),
				// EtherCAT identity and process RAM layout of the LAN9253 (8 KB at 1000h): mailboxes, then
				// outputs (3 buffers) and inputs (3 buffers). Vendor 0: no ETG vendor ID of our own yet.
				EtherCat: new UniversalRegisterAccess.Generators.EtherCatOptions(VendorId: 0x0000_0000,
					ProductCode: 0x0046_4355 /* "FCU" */, Revision: 1, Name: "Factory Control Unit", Group: "FactoryInABox",
					MailboxOutStart: 0x1000, MailboxOutSize: 128, MailboxInStart: 0x1080, MailboxInSize: 128,
					OutputsStart: 0x1100, InputsStart: 0x1800,
					PdiControl: 0x82 /* SPI EtherCAT Direct Mapped: the LAN9253 straps (EE_EMUL 100) */, TypeName: "FactoryControlUnit")));

		[BuildStep]
		public static void GenerateBestBinaryBufferFiles(IBuildContextStm32 ctx) =>
			WsProtocolBuildService.Generate(
				ctx,
				sourceDirs: Cli.GetAllArgValues(ctx.Args, "--best-binary-buffer-schema-path") is { Count: > 0 } sources
					? sources
					: [ctx.BestBinaryBuffersSchemaDir],
				cppOutputDir: ctx.FirmwareGeneratedDir,
				tsOutputDir: ctx.WebGeneratedDir);

		[BuildStep]
		public static void GenerateTerminalFiles(IBuildContextStm32 ctx) =>
			TerminalCatalog.Generate(
				ctx,
				Path.Combine(ctx.RootDir, "schema", "terminal"),
				cppOutputDir: ctx.FirmwareGeneratedDir,
				tsOutputDir: ctx.WebGeneratedDir);

		[BuildStep]
		public static void ReadGitStatusAndGenerateFiles(IBuildContextStm32 ctx)
		{
			var fw = FirmwareVersionReader.Read(Path.Combine(ctx.RootDir, "firmware-version.json"));
			GitBuildArtifactsService.Generate(
				ctx,
				defaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
				firmwareVersionMajor: fw.Major,
				firmwareVersionMinor: fw.Minor,
				firmwareVersionPatch: fw.Patch);
		}

		// Nur noch fuer die Zertifikat-Ausnahme (DER-Dateien) -- device_ids.hh/gitconstants.hh/
		// firmware_constants.hh/board_info.json/register_map.hpp/register_map_opcua.hpp/
		// ws_protocol.hh/terminal_commands.hh/register-map.ts/build-info.ts/ws-protocol.ts
		// werden inzwischen direkt ins Projekt generiert (s. Projektgedaechtnis "generierte Dateien
		// nur im Projekt"), brauchen also keinen Kopierschritt aus dem Board-Archiv mehr.
		[BuildStep]
		public static void CopyGeneratedFilesToBuildDirectory(IBuildContextStm32 ctx) =>
			GeneratedArtifactsCopyService.CopyToBuildDirectories(
				ctx,
				assetsDir: Stm32BuildContext.AssetsDir,
				coreFiles: [],
				webFiles: [],
				assetFiles: ["device_certificate.der", "device_key.der", "root_ca.der",
					"device_https_certificate.der", "device_https_key.der"]);

		[BuildStep]
		public static void BuildWebApp(IBuildContextStm32 ctx) => WebAppBuildService.Run(
			ctx,
			["--config", Path.Combine(ctx.WebRoot, "vite.config.ts")],
			Path.Combine(Stm32BuildContext.AssetsDir, "index.html.br"));

		// Die Web-App ohne Hardware (web_sim/: SPS-Entwicklung und HMI mit dem Simulator) als statische
		// Seite nach web_sim/dist/. Braucht nur die Registerkarte, kein Board und keine Zertifikate.
		[BuildStep]
		public static void BuildSimulationWebApp(IBuildContextStm32 ctx)
		{
			BuildStepRunner.Invoke(ctx, GenerateRegisterAccessFiles);
			var simDir = Path.Combine(ctx.RootDir, "web_sim");
			WebAppBuildService.Run(ctx, ["--config", Path.Combine(simDir, "vite.config.ts")], Path.Combine(simDir, "dist", "index.html"), simDir);
		}

		[BuildStep]
		public static void BuildFirmware(IBuildContextStm32 ctx) => CmakeFirmwareBuildService.Run(ctx);

		[BuildStep]
		public static void FlashFirmware(IBuildContextStm32 ctx) => FlashFirmwarePipelineService.Run(ctx);

		[BuildStep]
		public static void PipelineLazy(IBuildContextStm32 ctx)
		{
			BuildStepRunner.Invoke(ctx, PrepareContextWithRealHardware);
			GenerateAllAndBuild(ctx, GenerateCertificatesLazy);
		}

		// Wie PipelineLazy, aber ohne angeschlossene Hardware: Board aus --board, sonst das zuletzt
		// angeschlossene (build/.last-board-id); das Preset folgt ohne --preset dessen board_type
		// (s. Stm32BuildContext.DefaultPreset).
		[BuildStep]
		public static void PipelineLastBoard(IBuildContextStm32 ctx)
		{
			BuildStepRunner.Invoke(ctx, PrepareContextWithCommandLineArguments);
			GenerateAllAndBuild(ctx, GenerateCertificatesLazy);
		}

		[BuildStep]
		public static void PipelineForced(IBuildContextStm32 ctx)
		{
			BuildStepRunner.Invoke(ctx, PrepareContextWithRealHardware);
			GenerateAllAndBuild(ctx, GenerateCertificatesForced);
		}

		private static void GenerateAllAndBuild(IBuildContextStm32 ctx, Action<IBuildContextStm32> generateCertificates)
		{
			Console.WriteLine($"Board {ctx.BoardUid} ({ctx.BoardTypeName} {ctx.BoardTypeVersion}), Preset {ctx.Preset}");
			BuildStepRunner.Invoke(ctx, generateCertificates);
			BuildStepRunner.Invoke(ctx, GenerateDeviceArtifacts);
			BuildStepRunner.Invoke(ctx, GenerateWebCredentials);
			BuildStepRunner.Invoke(ctx, GenerateRegisterAccessFiles);
			BuildStepRunner.Invoke(ctx, GenerateBestBinaryBufferFiles);
			BuildStepRunner.Invoke(ctx, GenerateTerminalFiles);
			BuildStepRunner.Invoke(ctx, ReadGitStatusAndGenerateFiles);
			BuildStepRunner.Invoke(ctx, CopyGeneratedFilesToBuildDirectory);
			BuildStepRunner.Invoke(ctx, BuildWebApp);
			BuildStepRunner.Invoke(ctx, BuildFirmware);
		}

		[BuildStep]
		public static void PipelineFlash(IBuildContextStm32 ctx)
		{
			BuildStepRunner.Invoke(ctx, PipelineLazy);
			BuildStepRunner.Invoke(ctx, FlashFirmware);
		}
	}
}
