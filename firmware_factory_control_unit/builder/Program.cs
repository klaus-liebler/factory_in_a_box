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
			Stm32BoardProvisioningService.GenerateCertificates(ctx, force: false, httpsEcLeaf: true, sshHostKey: true);

		[BuildStep]
		public static void GenerateCertificatesForced(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateCertificates(ctx, force: true, httpsEcLeaf: true, sshHostKey: true);

		[BuildStep]
		public static void GenerateDeviceArtifacts(IBuildContextStm32 ctx) =>
			Stm32BoardProvisioningService.GenerateDeviceArtifacts(ctx);

		[BuildStep]
		public static void GenerateRegisterAccessFiles(IBuildContextStm32 ctx) =>
			UniversalRegisterAccessBuildService.Run(new UniversalRegisterAccessBuildRequest(
				RootDir: ctx.RootDir,
				DefaultSchemaDirectory: Stm32BuildContext.RegisterMapSchemaDir,
				CoreGeneratedDir: ctx.FirmwareGeneratedDir,
				WebGeneratedDir: ctx.WebGeneratedDir,
				Sources: [],
				OpcUaRootName: "FactoryControlUnit",
				DocsGeneratedDir: Path.Combine(ctx.RootDir, "docs", "generated")));

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
			GitBuildArtifactsService.Generate(
				ctx,
				defaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
				firmwareVersionMajor: fw.Major,
				firmwareVersionMinor: fw.Minor,
				firmwareVersionPatch: fw.Patch);
		}

		// Nur noch fuer die Zertifikat-Ausnahme (DER-Dateien) -- device_ids.hh/gitconstants.hh/
		// firmware_constants.hh/board_info.json/modbus_registers_generated.hh/
		// opcua_registers_generated.hh/ws_protocol.hh/register-map.ts/build-info.ts/ws-protocol.ts
		// werden inzwischen direkt ins Projekt generiert (s. Projektgedaechtnis "generierte Dateien
		// nur im Projekt"), brauchen also keinen Kopierschritt aus dem Board-Archiv mehr.
		[BuildStep]
		public static void CopyGeneratedFilesToBuildDirectory(IBuildContextStm32 ctx)
		{
			GeneratedArtifactsCopyService.CopyToBuildDirectories(
				ctx,
				assetsDir: Stm32BuildContext.AssetsDir,
				coreFiles: [],
				webFiles: [],
				assetFiles: ["device_certificate.der", "device_key.der", "root_ca.der",
					"device_https_certificate.der", "device_https_key.der", "ssh_host_key.der"]);
			WriteSshAuthorizedKeys(Path.Combine(Stm32BuildContext.AssetsDir, "ssh_authorized_keys"));
		}

		// Log terminal (ssh log@<hostname>.local): the public keys that may log in, one OpenSSH
		// line each (ssh-ed25519 / ecdsa-sha2-nistp256), from the files in Ssh.AuthorizedKeyFiles
		// (wildcards in the file name, "~" = the user profile). Not per board: every board of
		// this builder accepts them. At most 8 keys (rtos::net::SshServerContext).
		static void WriteSshAuthorizedKeys(string outputPath)
		{
			var home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
			var lines = new List<string>();
			foreach (var pattern in BuilderSettings.Current.Ssh.AuthorizedKeyFiles)
			{
				var path = pattern.StartsWith('~') ? home + pattern[1..] : pattern;
				var dir = Path.GetDirectoryName(path)!;
				if (!Directory.Exists(dir))
					continue;
				foreach (var file in Directory.GetFiles(dir, Path.GetFileName(path)).Order())
				{
					lines.AddRange(File.ReadAllLines(file).Select(l => l.Trim())
						.Where(l => l.StartsWith("ssh-ed25519 ") || l.StartsWith("ecdsa-sha2-nistp256 ")));
				}
			}
			if (lines.Count == 0)
				Console.WriteLine("WARNUNG: keine SSH-Schluessel (Ssh.AuthorizedKeyFiles) -- das Log-Terminal nimmt niemanden an");
			if (lines.Count > 8)
				throw new InvalidOperationException($"{lines.Count} SSH-Schluessel, die Firmware nimmt hoechstens 8");
			Directory.CreateDirectory(Path.GetDirectoryName(outputPath)!);
			File.WriteAllText(outputPath, string.Join("\n", lines) + "\n");
			Console.WriteLine($"{lines.Count} SSH-Schluessel -> {outputPath}");
		}

		[BuildStep]
		public static void BuildWebApp(IBuildContextStm32 ctx) => WebAppBuildService.Run(
			ctx,
			["--config", Path.Combine(ctx.WebRoot, "vite.config.ts")],
			Path.Combine(Stm32BuildContext.AssetsDir, "index.html.br"));

		[BuildStep]
		public static void BuildFirmware(IBuildContextStm32 ctx) => CmakeFirmwareBuildService.Run(ctx);

		[BuildStep]
		public static void FlashFirmware(IBuildContextStm32 ctx) => FlashFirmwarePipelineService.Run(ctx);

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
