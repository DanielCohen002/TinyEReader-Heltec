# TinyEReader (Heltec Vision Master E213)

A tiny Wi-Fi e-reader firmware for the [Heltec Vision Master E213](https://heltec.org/project/vision-master-e213/) (ESP32-S3 + SX1262 LoRa + 2.13" e-paper). This is a from-scratch port of [TinyEReader](https://github.com/DanielCohen002/TinyEReader-CrowPanel), originally built for the Elecrow CrowPanel ESP32 E-Paper HMI 2.13", to this different board.

Upload `.txt` books over Wi-Fi, read them on the e-paper screen, jump between chapters, and it remembers your place in each book.

Built and compile-tested without physical access to the board at first (see `hardware-research/heltec-e213` for the schematic/datasheet/factory-test sources every pin number and API call is drawn from), now running on real hardware. Anything the code comments still mark "verify on hardware" or "a guess" is worth double-checking against your actual board.

## Features

- Multi-book library: each upload adds a book rather than replacing one, up to 5MB per book (limited by free space, not a book-count cap)
- Remembers your reading position per book; boots to the Home screen (not straight into reading)
- True sequential page-up/page-down (always the actual neighboring page, computed by replaying pagination rather than an undo-last-jump stack)
- Chapter skip, using a marker character a book can contain — see [Chapters](#chapters-and-the-epub-converter) below
- Real battery percentage on the Home screen, read from the board's own ADC
- Web upload page with drag-and-drop, multi-file queueing (pre-checked against free space so a book that won't fit is skipped instead of failing mid-batch), a live library list, and per-book delete
- Wi-Fi only powers on while the Connect to Wi-Fi screen is open — off the rest of the time (including at boot) to save battery
- Light-sleeps the ESP32 and puts the e-paper panel to sleep after a minute of inactivity, wakes on any button press

## Hardware differences from the CrowPanel build

- Only 2 software-readable buttons exist (not the CrowPanel's 5 inputs). The board has a 3rd physical button (Reset), but it's wired straight to the chip's hardware EN pin, not a GPIO — pressing it just restarts the chip, so software can't read it as a normal button.
- Real battery voltage sensing exists (GPIO7, ADC) — the CrowPanel had none at all.
- The display driver is Heltec's own library (proportional fonts, built-in word-wrap) rather than a fixed-width character blitter.
- This board ships with one of two possible e-paper controller chips depending on manufacturing batch — detected at runtime via a raw SPI chip-ID probe, exactly as Heltec's own example does it.
- 16MB flash (double the CrowPanel's 8MB), so the custom partition table here gives LittleFS roughly 13.9MB.
- Both boards' charge-status LEDs (orange on this one) are wired directly to the charger chip's status pins, not to any ESP32 GPIO — there's no way for firmware to read or control charging state on either board.

## Buttons

Only 2 real buttons exist: **Left** = GPIO0/BOOT, **Right** = GPIO21/USER. Which physical side is actually "left" vs "right" was a guess made without the board in hand — swap the `BTN_LEFT`/`BTN_RIGHT` `#define`s near the top of the `.ino` if it's backwards on yours.

| | Left tap | Left hold | Right tap | Right hold | Both held |
| --- | --- | --- | --- | --- | --- |
| Reading a book | Previous page | Previous chapter | Next page | Next chapter | Home menu |
| Home / Choose Book / Wi-Fi / delete dialog | Move selection | Open delete dialog (Choose Book only) | Move selection | Select / confirm | Home menu (also cancels delete without deleting) |

## Display driver

Uses Heltec's own `HT_lCMEN2R13EFC1`/`HT_E0213A367` driver classes (bundled from their Arduino library, trimmed down in `hardware-research/heltec-e213/heltec-esp32-lib-trimmed` to just the files this sketch actually needs — the full library pulls in unrelated display drivers with dependencies that aren't installed).

Every render is currently a full-screen refresh — Heltec's library only exposes true partial/windowed refresh via low-level single-character calls that would need to be driven directly instead of through the normal buffer+`drawString` API. Worth revisiting once this feels slow in practice.

## Storage

The board has 16MB of flash. `firmware/HeltecVisionMasterE213/partitions.csv` gives the app 2MB and LittleFS the remaining ~13.9MB. Building/flashing with `PartitionScheme=custom` in the FQBN picks this file up automatically.

## Chapters and the EPUB converter

Plain `.txt` has no concept of chapters, so the firmware looks for a form-feed byte (`0x0C`, `'\f'`) in the book to mark chapter boundaries — invisible in the rendered text, just a jump point for the buttons while reading. A hand-typed `.txt` with no form feeds just has one implicit chapter.

`tools/epub_to_txt.py` converts an EPUB to a `.txt` with these markers already inserted, using the book's real chapter headings (`<h1>`/`<h2>`/`<h3>`) when present, or one marker per internal chapter file as a fallback:

```powershell
py tools\epub_to_txt.py yourbook.epub
```

Writes `yourbook.txt` next to it — upload that the normal way.

## Building

Requires the Arduino IDE (for its bundled `arduino-cli`) plus the trimmed Heltec library already checked into `hardware-research/heltec-e213/heltec-esp32-lib-trimmed`.

```powershell
$cli = "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
& $cli compile --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=custom,CDCOnBoot=cdc" --library hardware-research\heltec-e213\heltec-esp32-lib-trimmed --output-dir firmware\HeltecVisionMasterE213\build firmware\HeltecVisionMasterE213
```

### Board settings (if building from the Arduino IDE instead)

- Board: **ESP32S3 Dev Module**
- Flash Size: **16MB**
- Partition Scheme: **Custom** (picks up `firmware/HeltecVisionMasterE213/partitions.csv`)
- PSRAM: **OPI PSRAM**
- USB CDC On Boot: **Enabled**

## Flashing

```powershell
$cli = "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
& $cli upload -p COMx --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=custom,CDCOnBoot=cdc" --input-dir firmware\HeltecVisionMasterE213\build firmware\HeltecVisionMasterE213
```

Replace `COMx` with the board's port (check Device Manager).

## Using it

1. Flash the firmware.
2. On the device, hold both buttons to open the Home menu, tap either button to move to **Connect to Wi-Fi**, hold Right to select it.
3. Connect to `PocketReader`, password `12345678`.
4. Open `http://192.168.4.1` and upload one or more `.txt` files (or converted EPUBs — see [Chapters](#chapters-and-the-epub-converter)). Uploading doesn't open a book automatically — pick one from the device's Choose Book screen.
5. Hold Right (or Both) to back out of the Wi-Fi screen when done — Wi-Fi turns off automatically.
6. Tap Left/Right to turn pages, hold either to jump chapters, hold both for the Home menu.
