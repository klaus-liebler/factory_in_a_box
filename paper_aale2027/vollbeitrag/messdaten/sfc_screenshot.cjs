// usage: node sfc.cjs <out.png> <dsf> <edit|online> <waitMs> <selectStep>
const { chromium } = require("C:/Users/mail/AppData/Local/npm-cache/_npx/e41f203b7505f1fb/node_modules/playwright-core");
const out = process.argv[2];
const dsf = Number(process.argv[3] || 1);
const online = process.argv[4] === "online";
const waitMs = Number(process.argv[5] || 3000);
const select = process.argv[6] || "";

let n = 0;
const id = () => "k" + (++n);
const step = (name, actions = []) => ({ kind: "step", id: id(), name, actions });
const trans = (condition) => ({ kind: "trans", id: id(), condition });
const chart = [
  step("Init", [{ qualifier: "N", body: "ConveyorPwm := 0" }]),
  trans("Init.T > T#1s"),
  step("BandAn", [{ qualifier: "N", body: "ConveyorPwm := 600" }]),
  { kind: "branch", id: id(), type: "alt", branches: [
    [trans("Lightbarrier2"), step("Sortieren", [{ qualifier: "N", body: "Valve1" }, { qualifier: "P", body: "Sortiert := Sortiert + 1" }]), trans("Sortieren.T >= T#300ms")],
    [trans("BandAn.T > T#4s"), step("Durchlassen", [{ qualifier: "P", body: "Durchgelassen := Durchgelassen + 1" }]), trans("NOT Lightbarrier1")],
  ] },
  step("Zurueck", [{ qualifier: "N", body: "ConveyorPwm := 0" }]),
  trans("Zurueck.T > T#500ms"),
  { kind: "jump", id: id(), target: "Init" },
];
const decl = "PROGRAM SFC_PRG\nVAR\n    Sortiert : INT;         (* ausgeschleuste Teile *)\n    Durchgelassen : INT;    (* durchgelassene Teile *)\nEND_VAR";

(async () => {
  const browser = await chromium.launch({ executablePath: "C:/Users/mail/AppData/Local/ms-playwright/chromium-1234/chrome-win64/chrome.exe" });
  const ctx = await browser.newContext({ viewport: { width: Number(process.env.VW || 1600), height: Number(process.env.VH || 900) }, deviceScaleFactor: dsf, colorScheme: "light" });
  const page = await ctx.newPage();
  page.on("console", m => { if (m.type() === "error") console.log("console:", m.text()); });
  await page.goto("http://localhost:5174/");
  await page.waitForTimeout(1500);
  await page.evaluate(async ({ chart, decl }) => {
    const p = JSON.parse(localStorage.getItem("fcu.plc.project") || "null")
      ?? (await import("/@fs/C:/repos/factory_in_a_box/firmware_factory_control_unit/web_packages/plc/src/ide/project.ts")).exampleProject();
    p.name = "Sortieranlage";
    const o = p.objects.find(x => x.name === "SFC_PRG");
    o.chart = chart; o.decl = decl;
    localStorage.setItem("fcu.plc.project", JSON.stringify(p));
  }, { chart, decl });
  await page.reload();
  await page.waitForTimeout(1500);
  await page.getByText("SFC_PRG", { exact: true }).first().click();
  await page.waitForTimeout(500);
  // ribbon collapsed (double click on a tab: the shell's own feature)
  await page.getByText("Projekt", { exact: true }).first().dblclick();
  await page.waitForTimeout(300);
  if (online) {
    await page.getByText("Allein im Simulator ausführen").first().click();
    await page.waitForTimeout(waitMs);
  }
  if (process.env.TAB) { await page.getByText(process.env.TAB, { exact: true }).first().click(); await page.waitForTimeout(Number(process.env.WAIT2 || 500)); }
  if (select) {
    await page.locator(`:text-is("${select}"):visible`).first().click();
    await page.waitForTimeout(300);
  }
  if (process.env.SERIES) { for (let i = 0; i < Number(process.env.SERIES); i++) { const act = await page.evaluate(() => { const r = []; const walk = (n) => { if (n.shadowRoot) walk(n.shadowRoot); for (const c of n.children ?? []) walk(c); if (n.classList?.contains("sfc-active")) r.push((n.querySelector("input")?.value ?? "?") + " " + n.textContent.trim()); }; walk(document.body); return r.join("|"); }); console.log(i, act); await page.screenshot({ path: out.replace(".png", "_" + String(i).padStart(2, "0") + ".png") }); await page.waitForTimeout(300); } }
  else await page.screenshot({ path: out });
  await browser.close();
})();
