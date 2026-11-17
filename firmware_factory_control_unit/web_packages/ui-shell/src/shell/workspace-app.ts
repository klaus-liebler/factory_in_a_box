// The lifecycle of a workspace (an app element the shell mounts, shell/app-shell.ts). Every app element is
// created once and stays in the DOM (only "hidden" changes), so its state survives switching workspaces;
// onShow()/onHide() replace connectedCallback()/disconnectedCallback() as the start/stop signal for polling.
export interface WorkspaceApp {
	onShow(): void;
	onHide(): void;
}

export function hasWorkspaceLifecycle(el: HTMLElement): el is HTMLElement & WorkspaceApp {
	return typeof (el as Partial<WorkspaceApp>).onShow === "function";
}
