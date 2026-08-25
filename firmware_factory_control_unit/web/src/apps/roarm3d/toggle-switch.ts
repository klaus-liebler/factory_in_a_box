// Kleiner, abgerundeter Zwei-Wege-Umschalter (wie ein iOS-Segmented-Control) fuers
// Gizmo-Overlay -- ersetzt zwei separate Knoepfe durch einen einzigen Schalter mit
// gleitendem Farb-"Daumen" hinter der aktiven Beschriftung. Rein inline gestylt (kein
// externes Stylesheet noetig), passend zum Rest von roarm-3d-view.ts' selbstenthaltenen
// Overlay-Widgets.

export interface ToggleSwitchHandle {
  el: HTMLElement
  /** Aktive Option von aussen setzen (z.B. bei programmatischer Zustandsaenderung), ohne onChange auszuloesen. */
  setIndex(index: 0 | 1): void
}

const TRACK_BG = 'rgba(255, 255, 255, 0.85)'
const THUMB_BG = '#2f6fed'
const TEXT_ACTIVE = '#ffffff'
const TEXT_INACTIVE = '#33363c'

export function createToggleSwitch(labels: [string, string], initialIndex: 0 | 1, onChange: (index: 0 | 1) => void): ToggleSwitchHandle {
  const el = document.createElement('div')
  el.style.cssText =
    'position:relative;display:inline-flex;border-radius:8px;border:1px solid rgba(0,0,0,0.15);' +
    `background:${TRACK_BG};padding:2px;gap:2px;user-select:none;`

  const thumb = document.createElement('div')
  thumb.style.cssText =
    `position:absolute;top:2px;bottom:2px;left:2px;width:calc(50% - 3px);border-radius:6px;` +
    `background:${THUMB_BG};transition:transform 0.16s ease;pointer-events:none;`
  el.appendChild(thumb)

  let index = initialIndex
  const buttons = labels.map((label, i) => {
    const btn = document.createElement('button')
    btn.type = 'button'
    btn.textContent = label
    // Feste Breite (statt am Textinhalt haengend) -- so werden alle Umschalter im UI gleich
    // gross, unabhaengig davon, ob ihre beiden Labels lang ("Bewegen"/"Drehen") oder kurz
    // ("Welt"/"Lokal") sind, s. deren gemeinsamen Container in roarm-3d-view.ts.
    btn.style.cssText =
      'position:relative;font:inherit;font-size:12px;padding:4px 0;width:56px;text-align:center;border:none;' +
      'background:transparent;cursor:pointer;border-radius:6px;transition:color 0.16s ease;white-space:nowrap;'
    btn.addEventListener('click', () => {
      if (index === i) return
      index = i as 0 | 1
      applyVisual()
      onChange(index)
    })
    el.appendChild(btn)
    return btn
  })

  function applyVisual(): void {
    thumb.style.transform = index === 0 ? 'translateX(0)' : 'translateX(calc(100% + 3px))'
    buttons.forEach((btn, i) => {
      btn.style.color = i === index ? TEXT_ACTIVE : TEXT_INACTIVE
    })
  }
  applyVisual()

  return {
    el,
    setIndex(i: 0 | 1): void {
      if (index === i) return
      index = i
      applyVisual()
    },
  }
}
