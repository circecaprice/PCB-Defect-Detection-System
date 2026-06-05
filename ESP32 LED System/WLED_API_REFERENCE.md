# WLED Serial Control — Local Reference

A local copy of the WLED JSON-API details this app relies on, so you don't need
the WLED web UI or internet access to understand/extend the controller.

Sources:
- WLED serial interface: https://kno.wled.ge/interfaces/serial/
- WLED JSON API: https://kno.wled.ge/interfaces/json-api/
- Effect/palette names (index-ordered): WLED firmware ~0.14.x
  (`wled00/FX.h` `JSON_mode_names`, `wled00/FX.cpp` `JSON_palette_names`);
  mirrored locally in `wled_data.py`.

## Files in this folder

| File | Purpose |
|------|---------|
| `ESP32_driver.py` | `WLEDDriver` — pyserial JSON client (connect, send_json, query/get_state, set_* commands). |
| `wled_ui.py` | Standalone PyQt5 app (`python wled_ui.py`). |
| `wled_data.py` | Index-ordered `EFFECTS` / `PALETTES` name lists + `EFFECTS_SOURCE_VERSION`. |
| `WLED_API_REFERENCE.md` | This document. |

## Transport

- **115200 baud, 8N1**, over the ESP32's USB serial port.
- Send a **JSON object terminated with `\n`**, e.g. `{"on":true,"bri":128}\n`.
- Send `{"v":true}\n` to get the full state back. WLED replies with a
  **newline-terminated** JSON object containing `state` and `info`.
- ⚠️ **GPIO caveat:** if WLED's LED output is assigned to **GPIO1 or GPIO3**
  (the UART0 TX/RX pins), the USB serial JSON interface is unavailable — the port
  opens but no replies arrive. The app surfaces this as "no state reply". Move LED
  output to another data pin (e.g. GPIO2/GPIO16) in WLED's LED settings.

## Commands this app sends

| Action | JSON emitted |
|--------|--------------|
| Power on/off | `{"on":true}` / `{"on":false}` (`{"on":"t"}` toggles) |
| Brightness | `{"bri":0..255}` |
| Primary color | `{"seg":[{"id":0,"col":[[r,g,b]]}]}` |
| Effect | `{"seg":[{"id":0,"fx":<index>}]}` |
| Palette | `{"seg":[{"id":0,"pal":<index>}]}` |
| Effect speed | `{"seg":[{"id":0,"sx":0..255}]}` |
| Effect intensity | `{"seg":[{"id":0,"ix":0..255}]}` |
| Recall preset | `{"ps":<1..250>}` |
| Save preset | `{"psave":<1..250>}` |
| LED range (which LEDs on) | `{"seg":[{"id":0,"start":a,"stop":b}]}` (stop exclusive) |
| Individual LEDs | `{"seg":[{"id":0,"i":[idx,[r,g,b], idx2,[r,g,b], ...]}]}` |
| Read state | `{"v":true}` |

## State / info fields used on connect (`{"v":true}` reply)

- `state.on` (bool), `state.bri` (0..255), `state.ps` (current preset id)
- `state.seg[0]`: `fx`, `pal`, `sx`, `ix`, `start`, `stop`, `col` (`[[r,g,b], ...]`)
- `info.leds.count` — number of LEDs (sets the range spin-box bounds)
- `info.ver` — firmware version (compared to `EFFECTS_SOURCE_VERSION`; a mismatch
  warns that effect/palette index→name mapping may be off)
- `info.name` — device name

## Notes / limitations

- **Effect & palette indices are version-specific.** `wled_data.py` matches WLED
  ~0.14.x. If `info.ver` differs, names may not line up with indices; the console
  logs a warning. Update `wled_data.py` from the matching firmware source if so.
- Reserved/removed effect slots are kept as `"RSVD"` in `EFFECTS` to preserve
  index alignment — leave them in place.
- Only the **primary RGB** color is handled (`col[0]`). For RGBW strips, the
  white channel would be `col:[[r,g,b,w]]` (not exposed here).
- A mid-session USB unplug isn't actively detected; reconnect via the button.
