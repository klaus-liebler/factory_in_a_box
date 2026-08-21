using Builder;

// TODO: Zum Builder-Programm: Mir gefallen die String-Konstanten nicht! Jeder Service implementiert eine 
// IBuildService-Interface (Methode "RunService()"). 
// Bei der Registrierung soll es eher so aussehen registry.Add(()=>new XYZService(arg1, arg2)). 
// Der Service wird dann unter seinem Klassennamen verfügbar. 
// Sehr ähnliche Services können dann durch Ableitungen von abstakten Basisklassen mit leicht veränderten 
// Funktionen oder Werten in Konstruktorparametern erstellt werden. 
// Unter ihrem Klassennamen sind sie dann auch im Builder verfügbar.
//Pipeline-Definitionen können dann auch auf die Klassennamen der Services verweisen, anstatt auf String-Konstanten.

static ReadHardwareIdsRequest CreateReadHardwareIdsRequest() =>
	new(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		Stm32Programmer: BuilderSettings.Current.Stm32Programmer,
		BuildDir: Paths.BuildDir,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		RootDir: Paths.RootDir,
		DefaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
		ResolveBoardTypeNameByBoardId: boardId => BoardStateStore.TryGetBoardTypeName(BuilderSettings.Current.BoardStorage, boardId));

static GenerateCertificatesRequest CreateGenerateCertificatesRequest(string? board, bool force) =>
	new(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		Certificates: BuilderSettings.Current.Certificates,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		RootDir: Paths.RootDir,
		ExplicitBoardId: board,
		Force: force);

static GenerateDeviceArtifactsRequest CreateGenerateDeviceArtifactsRequest(string? board) =>
	new(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		ExplicitBoardId: board);

static UniversalRegisterAccessBuildRequest CreateRegisterAccessRequest(string? board) =>
	new(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		RootDir: Paths.RootDir,
		DefaultSchemaDirectory: Paths.RegisterMapSchemaDir,
		CoreGeneratedDir: Paths.CoreGeneratedDir,
		WebGeneratedDir: Paths.WebGeneratedDir,
		ExplicitBoardId: board,
		Sources: []);

static BestBinaryBufferBuildRequest CreateBestBinaryBufferRequest(string? board, IReadOnlyList<string> schemaPaths) =>
	new(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		RootDir: Paths.RootDir,
		DefaultSchemaDirectory: Paths.BestBinaryBuffersSchemaDir,
		CoreGeneratedDir: Paths.CoreGeneratedDir,
		WebGeneratedDir: Paths.WebGeneratedDir,
		ExplicitBoardId: board,
		Sources: schemaPaths);

static GitBuildArtifactsRequest CreateGitBuildRequest(string? board)
{
	var fwVersionPath = Path.Combine(Paths.RootDir, "firmware-version.json");
	var fw = FirmwareVersionReader.Read(fwVersionPath);
	return new GitBuildArtifactsRequest(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		RootDir: Paths.RootDir,
		CoreGeneratedDir: Paths.CoreGeneratedDir,
		WebGeneratedDir: Paths.WebGeneratedDir,
		DefaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
		FirmwareVersionMajor: fw.Major,
		FirmwareVersionMinor: fw.Minor,
		FirmwareVersionPatch: fw.Patch,
		ExplicitBoardId: board);
}

static GeneratedArtifactsCopyRequest CreateCopyRequest(string? board) =>
	new(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		ExplicitBoardId: board,
		CoreGeneratedDir: Paths.CoreGeneratedDir,
		WebGeneratedDir: Paths.WebGeneratedDir,
		AssetsDir: Paths.AssetsDir,
		CoreFiles: ["device_ids.hh", "gitconstants.hh", "firmware_constants.hh", "board-variant.json", "modbus_registers_generated.hh", "opcua_registers_generated.hh", "ws_protocol.hh"],
		WebFiles: ["register-map.ts", "build-info.ts", "ws-protocol.ts"],
		AssetFiles: ["device_certificate.der", "device_key.der", "root_ca.der"]);

static FlashFirmwarePipelineRequest CreateFlashRequest(string preset, string? board) =>
	new(
		BoardStorage: BuilderSettings.Current.BoardStorage,
		Stm32Programmer: BuilderSettings.Current.Stm32Programmer,
		RootDir: Paths.RootDir,
		BoardIdCacheFile: Paths.BoardIdCacheFile,
		BuildOutputDirectory: Paths.BuildDir,
		Preset: preset,
		DefaultBoardTypeName: BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName,
		ExplicitBoardId: board);

static Args ParseArgs(string[] argv)
{
	if (argv.Length == 0)
	{
		throw new ArgumentException("Kein Phasenname angegeben. Beispiel: dotnet run --project builder -- GenerateRegisterAccessFiles");
	}
	var phase = argv[0];

	// Release is the default, not Debug: SecurityPolicy#Basic256Sha256's software RSA-2048 (no
	// hardware PKA on this board's STM32H563, see the OPC UA security project notes) causes
	// 13-17s ThreadX thread-starvation stalls per handshake in an unoptimized Debug build --
	// severe enough that the board appears hung to anyone testing it. Pass --preset Debug
	// explicitly when actual step-debugging is needed.
	var preset = "Release";
	string? board = null;
	var schemaPaths = new List<string>();
	for (var i = 1; i < argv.Length; i += 1)
	{
		if (argv[i] == "--preset" && i + 1 < argv.Length)
		{
			preset = argv[i + 1];
			i += 1;
		}
		else if (argv[i] == "--board" && i + 1 < argv.Length)
		{
			board = argv[i + 1];
			i += 1;
		}
		else if (argv[i] == "--best-binary-buffer-schema-path" && i + 1 < argv.Length)
		{
			schemaPaths.Add(argv[i + 1]);
			i += 1;
		}
	}

	if (!CmakeFirmwareBuildService.IsValidPreset(preset))
	{
		throw new ArgumentException($"Unbekanntes Preset \"{preset}\". Gueltig: Debug, Release, Debug-Nucleo.");
	}

	return new Args(phase, preset, board, schemaPaths);
}

static void RunPhase(string phase, string preset, string? board, IReadOnlyList<string> schemaPaths)
{
	var registry = new CommandPipelineRegistry(StringComparer.Ordinal);


	registry.AddCommand("ReadHardwareIds", () => Stm32BoardProvisioningService.ReadHardwareIds(CreateReadHardwareIdsRequest()));
	registry.AddCommand("GenerateCertificatesLazy", () => Stm32BoardProvisioningService.GenerateCertificates(CreateGenerateCertificatesRequest(board, force: false)));
	registry.AddCommand("GenerateCertificatesForced", () => Stm32BoardProvisioningService.GenerateCertificates(CreateGenerateCertificatesRequest(board, force: true)));
	registry.AddCommand("GenerateDeviceArtifacts", () => Stm32BoardProvisioningService.GenerateDeviceArtifacts(CreateGenerateDeviceArtifactsRequest(board)));
	registry.AddCommand("GenerateRegisterAccessFiles", () => UniversalRegisterAccessBuildService.Run(CreateRegisterAccessRequest(board)));
	registry.AddCommand("GenerateBestBinaryBufferFiles", () => BestBinaryBufferBuildService.Run(CreateBestBinaryBufferRequest(board, schemaPaths)));
	registry.AddCommand("ReadGitStatusAndGenerateFiles", () => GitBuildArtifactsService.Generate(CreateGitBuildRequest(board)));
	registry.AddCommand("CopyGeneratedFilesToBuildDirectory", () => GeneratedArtifactsCopyService.CopyToBuildDirectories(CreateCopyRequest(board)));
	registry.AddCommand("BuildWebApp", () => WebAppBuildService.Run(Paths.RootDir, Paths.WebDir, Paths.AssetsDir));
	registry.AddCommand("BuildFirmware", () => CmakeFirmwareBuildService.Run(Paths.RootDir, preset));
	registry.AddCommand("FlashFirmware", () => FlashFirmwarePipelineService.Run(CreateFlashRequest(preset, board)));

	registry.AddPipeline("pipeline:lazy",
	[
		"ReadHardwareIds",
		"GenerateCertificatesLazy",
		"GenerateDeviceArtifacts",
		"GenerateRegisterAccessFiles",
		"GenerateBestBinaryBufferFiles",
		"ReadGitStatusAndGenerateFiles",
		"CopyGeneratedFilesToBuildDirectory",
		"BuildWebApp",
		"BuildFirmware",
	]);

	registry.AddPipeline("pipeline:forced",
	[
		"ReadHardwareIds",
		"GenerateCertificatesForced",
		"GenerateDeviceArtifacts",
		"GenerateRegisterAccessFiles",
		"GenerateBestBinaryBufferFiles",
		"ReadGitStatusAndGenerateFiles",
		"CopyGeneratedFilesToBuildDirectory",
		"BuildWebApp",
		"BuildFirmware",
	]);

	registry.AddPipeline("pipeline:flash",
	[
		"pipeline:lazy",
		"FlashFirmware",
	]);

	registry.Run(phase);
}

try
{
	var (phase, preset, board, schemaPaths) = ParseArgs(args);
	RunPhase(phase, preset, board, schemaPaths);
}
catch (Exception err)
{
	Console.Error.WriteLine($"\nFehler: {err.Message}");
	Environment.Exit(1);
}

sealed record Args(string Phase, string Preset, string? Board, List<string> SchemaPaths);
