// Phase "BuildFirmware" (s. docs/build-process.md Abschnitt 6) -- reiner CMake-Aufruf ueber die
// bestehenden CMakePresets.json-Presets. Erwartet, dass CopyGeneratedFilesToBuildDirectory und
// BuildWebApp vorher gelaufen sind (Core/generated/*, assets/* muessen existieren, sonst schlaegt
// entweder das #include oder der objcopy-Custom-Command in CMakeLists.txt fehl).
import { execFileSync } from "node:child_process";
import { rootDir } from "../paths.ts";

export const VALID_PRESETS = ["Debug", "Release", "Debug-Nucleo"] as const;
export type Preset = (typeof VALID_PRESETS)[number];

export function isValidPreset(preset: string): preset is Preset {
	return (VALID_PRESETS as readonly string[]).includes(preset);
}

export function buildFirmware(preset: Preset): void {
	execFileSync("cmake", ["--preset", preset], { cwd: rootDir, stdio: "inherit" });
	execFileSync("cmake", ["--build", "--preset", preset], { cwd: rootDir, stdio: "inherit" });
}
