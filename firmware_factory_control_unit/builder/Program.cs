using Builder;
using FirmwareBuilder.Common;

BuildStepRunner.Run(args, a => new Stm32BuildContext(a), typeof(Builder.Program));

namespace Builder
{
	public static class Program
	{
		[BuildStep]
		public static void Info(IBuildContextStm32 ctx) => Stm32ConsoleReport.WriteBoardInfo(ctx);

		[BuildStep]
		public static void GitStatus(IBuildContextStm32 ctx) => BuilderConsoleReport.WriteGitStatus(ctx.Git);

		[BuildStep]
		public static void PrepareContextWithRealHardware(IBuildContextStm32 ctx)
		{
			var request = new ReadHardwareIdsRequest(
				BoardStorage: BuilderSettings.Current.BoardStorage,
				Stm32Programmer: ctx.Stm32Programmer,
				BuildDir: ctx.BuildDir,
				BoardIdCacheFile: Stm32BuildContext.BoardIdCacheFile,
				RootDir: ctx.RootDir,
				DefaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
				ResolveBoardTypeNameByBoardId: boardId => BoardStateStore.TryGetBoardTypeName(BuilderSettings.Current.BoardStorage, boardId));
			Stm32BoardProvisioningService.ReadHardwareIds(request);
		}

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
			BoardArchiveContext.WriteCachedBoardId(ctx.BuildDir, Stm32BuildContext.BoardIdCacheFile, ctx.BoardUid);
			Console.WriteLine($"Board-Kontext aus --board uebernommen: {ctx.BoardUid} ({ctx.BoardArchiveDir})");
		}

		[BuildStep]
		public static void GenerateCertificatesLazy(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateCertificates(new GenerateCertificatesRequest(
				BoardStorage: BuilderSettings.Current.BoardStorage,
				Certificates: ctx.Certificates,
				BoardIdCacheFile: Stm32BuildContext.BoardIdCacheFile,
				RootDir: ctx.RootDir,
				ExplicitBoardId: Cli.GetOptionalArgValue(ctx.Args, "--board"),
				Force: false));

		[BuildStep]
		public static void GenerateCertificatesForced(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateCertificates(new GenerateCertificatesRequest(
				BoardStorage: BuilderSettings.Current.BoardStorage,
				Certificates: ctx.Certificates,
				BoardIdCacheFile: Stm32BuildContext.BoardIdCacheFile,
				RootDir: ctx.RootDir,
				ExplicitBoardId: Cli.GetOptionalArgValue(ctx.Args, "--board"),
				Force: true));

		[BuildStep]
		public static void GenerateDeviceArtifacts(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateDeviceArtifacts(new GenerateDeviceArtifactsRequest(
				BoardStorage: BuilderSettings.Current.BoardStorage,
				BoardIdCacheFile: Stm32BuildContext.BoardIdCacheFile,
				CoreGeneratedDir: ctx.FirmwareGeneratedDir,
				ExplicitBoardId: Cli.GetOptionalArgValue(ctx.Args, "--board")));

		[BuildStep]
		public static void GenerateRegisterAccessFiles(IBuildContextStm32 ctx) =>
			UniversalRegisterAccessBuildService.Run(new UniversalRegisterAccessBuildRequest(
				RootDir: ctx.RootDir,
				DefaultSchemaDirectory: Stm32BuildContext.RegisterMapSchemaDir,
				CoreGeneratedDir: ctx.FirmwareGeneratedDir,
				WebGeneratedDir: ctx.WebGeneratedDir,
				Sources: []));

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
		public static void ReadGitStatusAndGenerateFiles(IBuildContextStm32 ctx)
		{
			var fw = FirmwareVersionReader.Read(Path.Combine(ctx.RootDir, "firmware-version.json"));
			GitBuildArtifactsService.Generate(new GitBuildArtifactsRequest(
				BoardStorage: BuilderSettings.Current.BoardStorage,
				BoardIdCacheFile: Stm32BuildContext.BoardIdCacheFile,
				RootDir: ctx.RootDir,
				CoreGeneratedDir: ctx.FirmwareGeneratedDir,
				WebGeneratedDir: ctx.WebGeneratedDir,
				DefaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
				FirmwareVersionMajor: fw.Major,
				FirmwareVersionMinor: fw.Minor,
				FirmwareVersionPatch: fw.Patch,
				ExplicitBoardId: Cli.GetOptionalArgValue(ctx.Args, "--board")));
		}

		// Nur noch fuer die Zertifikat-Ausnahme (DER-Dateien) -- device_ids.hh/gitconstants.hh/
		// firmware_constants.hh/board_info.json/modbus_registers_generated.hh/
		// opcua_registers_generated.hh/ws_protocol.hh/register-map.ts/build-info.ts/ws-protocol.ts
		// werden inzwischen direkt ins Projekt generiert (s. Projektgedaechtnis "generierte Dateien
		// nur im Projekt"), brauchen also keinen Kopierschritt aus dem Board-Archiv mehr.
		[BuildStep]
		public static void CopyGeneratedFilesToBuildDirectory(IBuildContextStm32 ctx) =>
			GeneratedArtifactsCopyService.CopyToBuildDirectories(new GeneratedArtifactsCopyRequest(
				BoardStorage: BuilderSettings.Current.BoardStorage,
				BoardIdCacheFile: Stm32BuildContext.BoardIdCacheFile,
				ExplicitBoardId: Cli.GetOptionalArgValue(ctx.Args, "--board"),
				CoreGeneratedDir: ctx.FirmwareGeneratedDir,
				WebGeneratedDir: ctx.WebGeneratedDir,
				AssetsDir: Stm32BuildContext.AssetsDir,
				CoreFiles: [],
				WebFiles: [],
				AssetFiles: ["device_certificate.der", "device_key.der", "root_ca.der"]));

		[BuildStep]
		public static void BuildWebApp(IBuildContextStm32 ctx) => WebAppBuildService.Run(
			ctx,
			["--config", Path.Combine(ctx.WebRoot, "vite.config.ts")],
			Path.Combine(Stm32BuildContext.AssetsDir, "index.html.br"));

		[BuildStep]
		public static void BuildFirmware(IBuildContextStm32 ctx) => CmakeFirmwareBuildService.Run(ctx);

		[BuildStep]
		public static void FlashFirmware(IBuildContextStm32 ctx) =>
			FlashFirmwarePipelineService.Run(new FlashFirmwarePipelineRequest(
				BoardStorage: BuilderSettings.Current.BoardStorage,
				Stm32Programmer: ctx.Stm32Programmer,
				RootDir: ctx.RootDir,
				BoardIdCacheFile: Stm32BuildContext.BoardIdCacheFile,
				BuildOutputDirectory: ctx.BuildDir,
				Preset: ctx.Preset,
				DefaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
				ExplicitBoardId: Cli.GetOptionalArgValue(ctx.Args, "--board")));

		[BuildStep]
		public static void PipelineLazy(IBuildContextStm32 ctx)
		{
			BuildStepRunner.Invoke(ctx, PrepareContextWithRealHardware);
			BuildStepRunner.Invoke(ctx, GenerateCertificatesLazy);
			BuildStepRunner.Invoke(ctx, GenerateDeviceArtifacts);
			BuildStepRunner.Invoke(ctx, GenerateRegisterAccessFiles);
			BuildStepRunner.Invoke(ctx, GenerateBestBinaryBufferFiles);
			BuildStepRunner.Invoke(ctx, ReadGitStatusAndGenerateFiles);
			BuildStepRunner.Invoke(ctx, CopyGeneratedFilesToBuildDirectory);
			BuildStepRunner.Invoke(ctx, BuildWebApp);
			BuildStepRunner.Invoke(ctx, BuildFirmware);
		}

		[BuildStep]
		public static void PipelineForced(IBuildContextStm32 ctx)
		{
			BuildStepRunner.Invoke(ctx, PrepareContextWithRealHardware);
			BuildStepRunner.Invoke(ctx, GenerateCertificatesForced);
			BuildStepRunner.Invoke(ctx, GenerateDeviceArtifacts);
			BuildStepRunner.Invoke(ctx, GenerateRegisterAccessFiles);
			BuildStepRunner.Invoke(ctx, GenerateBestBinaryBufferFiles);
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
