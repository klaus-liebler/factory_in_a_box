// Phase "BuildFirmware" (s. docs/build-process.md Abschnitt 2) -- reiner CMake-Aufruf ueber die
// bestehenden CMakePresets.json-Presets. Erwartet, dass CopyGeneratedFilesToBuildDirectory und BuildWebApp
// vorher gelaufen sind (Core/generated/*, build/assets/* muessen existieren, sonst schlaegt entweder das
// #include oder der objcopy-Custom-Command in CMakeLists.txt fehl).
namespace Builder.Phases;

public static class BuildFirmware
{
	public static readonly string[] ValidPresets = ["Debug", "Release", "Debug-Nucleo"];

	public static bool IsValidPreset(string preset) => ValidPresets.Contains(preset);

	public static void Run(string preset)
	{
		ProcessRunner.RunInherit("cmake", ["--preset", preset], Paths.RootDir);
		ProcessRunner.RunInherit("cmake", ["--build", "--preset", preset], Paths.RootDir);
	}
}
