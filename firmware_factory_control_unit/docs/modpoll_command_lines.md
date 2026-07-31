# Modpoll Command Lines

### 📊 Diagnosedaten auslesen
Liest komplett 13 Halteregister (Holding Registers) ab der Adresse 0 aus.
```bash
# -0: Nullbasierte Adressierung
# -t 3: Datentyp 16-Bit Holding Register
# -r 0: Startadresse 0
# -c 13: Anzahl der Register (13 Stück)
modpoll -0 -t 3 -r 0 -c 13 COM47
```

### ⚡ Stromversorgung auslesen
Liest 8 Register ab Adresse 110 aus (z. B. Busspannung, Stromstärke).
```bash
# -r 110: Startadresse 110
# -c 8: Anzahl der Register (8 Stück)
modpoll -0 -t 3 -r 110 -c 8 COM47
```

### 🔴 LED-Steuerung (LED 1)
Schreibt Werte in das Register 100, um den Zustand der LED 1 zu ändern.
```bash
# -t 4: Schreibbares 16-Bit Register (Preset Single Register)

# LED1 auf ROT einschalten (Wert 1)
modpoll -0 -t 4 -r 100 COM47 1

# LED1 ausschalten / dunkel (Wert 0)
modpoll -0 -t 4 -r 100 COM47 0
```