/*
  Tiny E-Reader -- Heltec Vision Master E213 port
  ESP32-S3 + 2.13" e-paper (250x122), SX1262 LoRa radio present but unused.

  Ported from the CrowPanel-based TinyEReader to different hardware -- see
  hardware-research/heltec-e213 in this repo for the schematic/datasheet
  sources every pin number and API call here is drawn from. Now running on
  real hardware; anything still marked "verify on hardware" is an educated
  guess pending further testing, not a confirmed fact.

  Key hardware differences from the CrowPanel build this was ported from:
  - Only 2 software-readable buttons exist (not the CrowPanel's 5 inputs).
    The board has a 3rd physical button (Reset), but it's wired straight to
    the chip's hardware EN pin, not a GPIO -- pressing it just restarts the
    chip from scratch, so software can't read it as a normal button. See
    the button scheme below.
  - This board has a battery ADC pin (GPIO7) the CrowPanel doesn't, but the
    percentage it produced didn't hold up in practice and was removed --
    see the git history if it's worth revisiting with a better formula.
  - The display driver is Heltec's own library (proportional fonts, built-in
    word-wrap) rather than Elecrow's fixed-width character blitter. Per
    Heltec's own board datasheet, the panel is always LCMEN2R13EFC1 (the
    HT_ICMEN2R13EFC1 driver class) -- the alternate HT_E0213A367 class the
    chip-ID probe can also select is defensive/untested, kept only because
    Heltec's own example code handles both.
  - Both the CrowPanel and this board's charge-status LED are wired
    straight to their charger chip's status pins, not to any ESP32 GPIO --
    there's no way for firmware to read or control charging state on
    either board.
  - 16MB flash (double the CrowPanel's 8MB), so the custom partition table
    here gives LittleFS roughly 13.9MB.

  Button scheme (only 2 real buttons: Left=GPIO21/USER, Right=GPIO0/BOOT --
  confirmed on real hardware; this was backwards in the initial guess):

  Reading a book:
    Left tap        -> previous page      Right tap        -> next page
    Left double-tap  -> previous chapter   Right double-tap  -> next chapter
    Left hold        -> keep paging back   Right hold        -> keep paging forward
                         (auto-repeats, like the CrowPanel build's Top/Bottom hold)
    Both held        -> Home menu

  Everywhere else (Home / Choose Book / Wi-Fi / delete dialog):
    Left tap    -> move selection      Right tap    -> move selection
    Left hold   -> open delete dialog (Choose Book only, on a highlighted book)
    Right hold  -> select / confirm
    Both held   -> Home menu (also cancels the delete dialog without deleting)

  A tap only fires once its own double-tap window passes without a second
  tap following -- see DOUBLE_TAP_WINDOW_MS -- so every single tap has a
  small (currently 250ms) built-in delay before the page turns. Worth
  shortening if that lag is more annoying in practice than double-tap
  chapter-skipping is useful.

  Both-held ("double push") is intentionally forgiving about how
  simultaneously the two buttons actually go down -- see the comment on
  bothPressStart in pollButtons() -- since two physical buttons are rarely
  pressed within the same tens of milliseconds of each other.

  Refresh strategy: every render is a full black-flash refresh. An attempt
  at a genuine partial/fast refresh (adapted from the vendor library's own
  dis_img_Partial_Refresh() example) didn't visibly help even after fixing
  a real bug in its window parameters, so it was pulled back out rather
  than keep guessing blind at undocumented LUT/waveform behavior with no
  way to see the panel. See git history if it's worth revisiting with
  actual hardware-debugging tools (logic analyzer, etc).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "HT_lCMEN2R13EFC1.h"
#include "HT_E0213A367.h"

// 1-bit bitmaps (Home screen icons, Wi-Fi screen QR codes) packed
// LSB-first for ScreenDisplay::drawXbm() -- generated with
// tools/image_to_epd.py --format xbm from hand-drawn Piskel files /
// QR PNGs.
#include "generated/icon_book.h"
#include "generated/icon_bookshelf.h"
#include "generated/icon_wifi.h"
#include "generated/qr_wifi.h"
#include "generated/qr_webpage.h"

// ---------------- HARDWARE CONFIG ----------------
// E-paper SPI pins and Vext (peripheral power) pins are all taken directly
// from Heltec's own factory-test sketch and example code for this exact
// board (see hardware-research/heltec-e213), not guessed.
#define PIN_EINK_RST   3
#define PIN_EINK_DC    2
#define PIN_EINK_CS    5
#define PIN_EINK_BUSY  1
#define PIN_EINK_SCLK  4
#define PIN_EINK_MOSI  6

#define VEXT_CTRL_1  18
#define VEXT_CTRL_2  46

// Only two software-readable buttons exist -- see the button scheme in the
// file header. Confirmed backwards on real hardware from the initial guess,
// so BOOT is actually the right-hand button and USER is the left-hand one.
#define BTN_LEFT   21  // USER
#define BTN_RIGHT  0   // BOOT

// ---------------- APP CONFIG ----------------
const char* AP_NAME = "PocketReader";
const char* AP_PASS = "12345678";

// Per-book sanity cap -- the real limit is whatever free space
// handleUpload() finds at upload time. See partitions.csv: LittleFS gets
// roughly 13.9MB of this board's 16MB flash.
constexpr size_t MAX_BOOK_SIZE = 5 * 1024 * 1024;
constexpr uint32_t SLEEP_AFTER_MS = 60000;
// Book count is bounded by flash space, not an arbitrary limit -- see
// uploadSizeLimit in handleUpload(). MAX_BOOKS is just the bookList array's
// capacity (RAM, not flash). The Choose Book screen only fits
// CHOOSE_VISIBLE_ROWS at a time and scrolls to show more.
constexpr uint8_t MAX_BOOKS = 64;
constexpr uint8_t CHOOSE_VISIBLE_ROWS = 6;  // one row is a free-space header, see renderChooseBook()

constexpr unsigned long HOLD_THRESHOLD_MS = 500;  // tap vs. hold cutoff for both buttons
constexpr unsigned long PAGE_REPEAT_DELAY_MS = 500;     // hold Left/Right this long to start auto-paging (reading screen only)
constexpr unsigned long PAGE_REPEAT_INTERVAL_MS = 300;  // then advance one page this often while held

// Reading screen layout. Uses ArialMT_Plain_16 for readable text, but at
// 7 lines/page rather than the font's own 19px line height (which only
// fits 6 and leaves the screen looking sparse) -- 17px is tight enough to
// shave 2px of inter-line clearance off the font's nominal spacing, which
// risks clipping a descender (a 'y'/'g'/'p') on some line pairs. Worth
// checking for on real hardware; if it looks clipped, drop back to 19/6.
constexpr uint16_t SCREEN_W = 250;
constexpr uint16_t SCREEN_H = 122;
constexpr uint16_t BOOK_LEFT_MARGIN = 2;
constexpr uint16_t BOOK_TOP_MARGIN = 1;
constexpr uint16_t BOOK_LINE_HEIGHT = 17;
constexpr uint16_t BOOK_USABLE_WIDTH = SCREEN_W - 2 * BOOK_LEFT_MARGIN;
constexpr uint8_t BOOK_MAX_LINES = 7;  // (122-1)/17 = 7.1, rounds down to 7

// Menu/selection screens (Home, Choose Book, delete confirm) use the same
// font and line height as the reading screen for visual consistency --
// the Wi-Fi screen is left at the smaller font since it's QR-code-driven,
// not a navigable list, and the QR codes there are positioned close
// enough to the header text that the bigger font would collide with them.
constexpr uint16_t MENU_LEFT_MARGIN = 2;
constexpr uint16_t MENU_LINE_HEIGHT = BOOK_LINE_HEIGHT;

WebServer server(80);

enum AppScreen : uint8_t { SCREEN_READING, SCREEN_HOME, SCREEN_CHOOSE_BOOK, SCREEN_WIFI_INFO, SCREEN_CONFIRM_DELETE };
AppScreen currentScreen = SCREEN_READING;

// Declared up here (rather than next to pollButtons()/dispatchGesture() below)
// because the Arduino preprocessor auto-generates function prototypes and
// inserts them right after the #include block -- a prototype referencing
// ButtonGesture needs the enum already visible at that point, or it fails
// with "does not name a type".
enum ButtonGesture : uint8_t {
  GESTURE_NONE,
  GESTURE_LEFT_TAP,
  GESTURE_LEFT_HOLD,
  GESTURE_RIGHT_TAP,
  GESTURE_RIGHT_HOLD,
  GESTURE_BOTH_HOLD
};

constexpr uint8_t HOME_ITEM_COUNT = 3;
const uint8_t* HOME_ICONS[HOME_ITEM_COUNT] = { iconBook, iconBookshelf, iconWifi };
const char* HOME_SHORT_LABELS[HOME_ITEM_COUNT] = { "Read", "Books", "Wi-Fi" };
uint8_t homeSelection = 0;

// Icon-first layout: a row of three 48x48 icons with a small label under
// each, centered as a block in the 250x122 screen, selection shown as a
// border box rather than a "> " prefix (there's no list to prefix here) --
// same layout as the CrowPanel build's Home screen.
constexpr uint16_t HOME_ICON_SIZE = 48;
constexpr uint16_t HOME_ICON_Y = 30;
constexpr uint8_t HOME_LABEL_FONT_SIZE = 19;  // ArialMT_Plain_16's cell height, matching MENU_LINE_HEIGHT
constexpr uint16_t HOME_LABEL_Y = HOME_ICON_Y + HOME_ICON_SIZE + 4;
const uint16_t HOME_ICON_X[HOME_ITEM_COUNT] = { 26, 100, 174 };  // evenly spaced, symmetric margins

String bookList[MAX_BOOKS];
uint8_t bookCount = 0;
uint8_t bookFileTotal = 0;  // like bookCount but not capped at MAX_BOOKS -- see listBooks()
uint8_t chooseSelection = 0;
uint8_t chooseWindowStart = 0;  // first bookList index shown on screen, see scrollChooseWindow()
bool confirmDeleteYes = false;  // defaults to No -- the safe choice

String currentBookName;  // filename only, e.g. "MyNovel.txt" -- lives under /books/

File book;
size_t fileSize = 0;
uint32_t pageStart = 0;
uint32_t pageEnd = 0;
unsigned long lastActivity = 0;

constexpr uint16_t MAX_CHAPTERS = 200;
uint32_t chapterOffsets[MAX_CHAPTERS];
uint16_t chapterCount = 0;

constexpr uint16_t MAX_CACHED_PAGES = 400;
uint32_t cachedPageStarts[MAX_CACHED_PAGES];
uint16_t cachedPageCount = 0;
uint32_t cachedChapterStart = 0xFFFFFFFF;

const char uploadPage[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>PocketReader</title>
  <style>
    :root { color-scheme: light dark; }
    body {
      font-family: system-ui, sans-serif; line-height: 1.4; margin: 0;
      padding: 2rem 1rem; background: #f2efe9; color: #1c1a17;
      display: flex; justify-content: center;
    }
    .card {
      width: 100%; max-width: 26rem; background: #fffdf9;
      border-radius: 0.75rem; padding: 1.5rem 1.5rem 1.75rem;
      box-shadow: 0 1px 3px rgba(0,0,0,0.12);
    }
    h1 { font-size: 1.4rem; margin: 0 0 0.25rem; }
    .sub { color: #6b6357; font-size: 0.9rem; margin: 0 0 1.25rem; }
    .drop {
      display: block; box-sizing: border-box;
      border: 2px dashed #c9c0b0; border-radius: 0.6rem; padding: 1.5rem 1rem;
      text-align: center; cursor: pointer; transition: border-color 0.15s, background 0.15s;
    }
    .drop.drag { border-color: #8a7a5c; background: #f6f1e6; }
    .drop p { margin: 0.25rem 0; }
    .drop .hint { font-size: 0.85rem; color: #8a8072; }
    input[type=file] { display: none; }
    button {
      font: inherit; margin-top: 1rem; width: 100%; padding: 0.65rem;
      border: none; border-radius: 0.5rem; background: #3d3527; color: #fff;
      cursor: pointer;
    }
    button:disabled { background: #b6ac9a; cursor: not-allowed; }
    .files { margin-top: 1rem; font-size: 0.9rem; }
    .file-row { display: flex; justify-content: space-between; gap: 0.5rem; padding: 0.15rem 0; }
    .file-row .status { color: #8a8072; white-space: nowrap; }
    .file-row.done .status { color: #3c7a3c; }
    .file-row.error .status { color: #b03a2e; }
    h2 { font-size: 1rem; margin: 1.75rem 0 0.5rem; }
    .lib-meta { font-size: 0.85rem; color: #6b6357; margin: 0 0 0.5rem; }
    ul.lib { list-style: none; margin: 0; padding: 0; font-size: 0.9rem; }
    ul.lib li { display: flex; align-items: center; gap: 0.5rem; padding: 0.35rem 0; border-top: 1px solid #ece7dc; }
    ul.lib li .name { flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    ul.lib li .size { color: #8a8072; white-space: nowrap; }
    ul.lib li .del {
      flex: none; width: auto; margin-top: 0; padding: 0.2rem 0.55rem; font-size: 0.8rem;
      border: 1px solid #c9c0b0; border-radius: 0.35rem; background: transparent; color: #8a6b5c;
    }
    ul.lib li .del:hover { border-color: #b03a2e; color: #b03a2e; }
    .empty { color: #8a8072; font-size: 0.9rem; }
    @media (prefers-color-scheme: dark) {
      body { background: #171512; color: #ece7dc; }
      .card { background: #201d19; box-shadow: none; }
      .sub, .lib-meta, .empty, .drop .hint, .file-row .status { color: #a49b8a; }
      .drop { border-color: #4a4438; }
      .drop.drag { border-color: #8a7a5c; background: #2a251e; }
      button { background: #ece7dc; color: #201d19; }
      button:disabled { background: #4a4438; color: #a49b8a; }
      ul.lib li { border-top-color: #332e27; }
      ul.lib li .del { border-color: #4a4438; color: #c9b8a8; background: transparent; }
      ul.lib li .del:hover { border-color: #d47a6a; color: #d47a6a; }
    }
  </style>
</head>
<body>
  <div class="card">
    <h1>PocketReader</h1>
    <p class="sub">Upload one or more .txt books. Open a book from the device's Choose Book screen.</p>

    <form id="form">
      <label class="drop" id="drop" for="file">
        <p id="dropLabel">Tap to choose books</p>
        <p class="hint">or drag &amp; drop .txt files here</p>
      </label>
      <input type="file" id="file" name="file" accept=".txt,text/plain" multiple>
      <button type="submit" id="go" disabled>Upload</button>
    </form>
    <div class="files" id="files"></div>

    <h2>Library</h2>
    <p class="lib-meta" id="libMeta">Loading...</p>
    <ul class="lib" id="libList"></ul>
  </div>

  <script>
    var fileInput = document.getElementById('file');
    var drop = document.getElementById('drop');
    var dropLabel = document.getElementById('dropLabel');
    var go = document.getElementById('go');
    var filesBox = document.getElementById('files');
    var form = document.getElementById('form');
    var picked = [];

    function fmtSize(n) {
      if (n > 1024 * 1024) return (n / (1024 * 1024)).toFixed(1) + 'MB';
      if (n > 1024) return Math.round(n / 1024) + 'KB';
      return n + 'B';
    }

    function refreshLibrary() {
      fetch('/books').then(function (r) { return r.json(); }).then(function (data) {
        var meta = data.count + ' book' + (data.count === 1 ? '' : 's') + ' - ' + data.free;
        if (data.hidden > 0) {
          meta += ' (+' + data.hidden + ' more on the device you can\'t see or delete here - ' +
            'delete a visible book to make room, which will reveal one of them)';
        }
        document.getElementById('libMeta').textContent = meta;
        var list = document.getElementById('libList');
        list.innerHTML = '';
        if (data.books.length === 0) {
          var li = document.createElement('li');
          li.className = 'empty';
          li.textContent = 'No books yet.';
          list.appendChild(li);
        }
        data.books.forEach(function (b) {
          var li = document.createElement('li');
          var name = document.createElement('span');
          name.className = 'name';
          name.textContent = b.name;
          var size = document.createElement('span');
          size.className = 'size';
          size.textContent = fmtSize(b.size);
          var del = document.createElement('button');
          del.type = 'button';
          del.className = 'del';
          del.textContent = 'Delete';
          del.addEventListener('click', function () { deleteBook(b.name); });
          li.appendChild(name);
          li.appendChild(size);
          li.appendChild(del);
          list.appendChild(li);
        });
      }).catch(function () {
        document.getElementById('libMeta').textContent = '';
      });
    }

    function deleteBook(name) {
      if (!confirm('Delete "' + name + '"? This can\'t be undone.')) return;
      fetch('/delete?name=' + encodeURIComponent(name), { method: 'POST' })
        .then(refreshLibrary)
        .catch(refreshLibrary);
    }

    function setPicked(fileList) {
      picked = Array.prototype.filter.call(fileList, function (f) {
        return f.name.toLowerCase().endsWith('.txt');
      });
      dropLabel.textContent = picked.length === 0
        ? 'Tap to choose books'
        : picked.length + ' book' + (picked.length > 1 ? 's' : '') + ' selected';
      go.disabled = picked.length === 0;
      filesBox.innerHTML = '';
    }

    fileInput.addEventListener('change', function () { setPicked(fileInput.files); });

    ['dragenter', 'dragover'].forEach(function (evt) {
      drop.addEventListener(evt, function (e) { e.preventDefault(); drop.classList.add('drag'); });
    });
    ['dragleave', 'drop'].forEach(function (evt) {
      drop.addEventListener(evt, function (e) { e.preventDefault(); drop.classList.remove('drag'); });
    });
    drop.addEventListener('drop', function (e) {
      setPicked(e.dataTransfer.files);
    });

    function addFileRow(file) {
      var row = document.createElement('div');
      row.className = 'file-row';
      var name = document.createElement('span');
      name.textContent = file.name;
      var status = document.createElement('span');
      status.className = 'status';
      row.appendChild(name);
      row.appendChild(status);
      filesBox.appendChild(row);
      return { row: row, status: status };
    }

    function uploadOne(file) {
      return new Promise(function (resolve) {
        var r = addFileRow(file);
        r.status.textContent = '0%';

        var xhr = new XMLHttpRequest();
        xhr.open('POST', '/upload');
        xhr.upload.onprogress = function (e) {
          if (e.lengthComputable) r.status.textContent = Math.round((e.loaded / e.total) * 100) + '%';
        };
        xhr.onload = function () {
          if (xhr.status === 200) {
            r.row.classList.add('done');
            r.status.textContent = 'Done';
          } else {
            r.row.classList.add('error');
            r.status.textContent = xhr.status === 413 ? 'Too large' : 'Failed';
          }
          resolve();
        };
        xhr.onerror = function () {
          r.row.classList.add('error');
          r.status.textContent = 'Failed';
          resolve();
        };
        var data = new FormData();
        data.append('file', file);
        xhr.send(data);
      });
    }

    // Mirrors handleUpload()'s own acceptance check (min(maxSize, freeBytes -
    // 8192), freeBytes shrinking as earlier books in the queue are counted)
    // so books that can't possibly fit are rejected up front instead of
    // failing partway through a batch after using up real upload time.
    function planQueue(files, freeBytes, maxSize) {
      var available = freeBytes;
      return files.map(function (file) {
        var allowed = available > 8192 ? Math.min(maxSize, available - 8192) : 0;
        var fits = file.size <= allowed;
        if (fits) available -= file.size;
        return fits;
      });
    }

    form.addEventListener('submit', function (e) {
      e.preventDefault();
      if (picked.length === 0) return;
      go.disabled = true;
      filesBox.innerHTML = '';

      fetch('/books').then(function (r) { return r.json(); }).then(function (data) {
        var fits = planQueue(picked, data.freeBytes, data.maxSize);
        var chain = Promise.resolve();
        picked.forEach(function (file, i) {
          if (fits[i]) {
            chain = chain.then(function () { return uploadOne(file); });
          } else {
            var r = addFileRow(file);
            r.row.classList.add('error');
            r.status.textContent = "Won't fit - skipped";
          }
        });
        return chain;
      }).then(function () {
        picked = [];
        fileInput.value = '';
        dropLabel.textContent = 'Tap to choose books';
        refreshLibrary();
      });
    });

    refreshLibrary();
  </script>
</body>
</html>
)rawliteral";

// ---------------- DISPLAY ----------------
ScreenDisplay *display;
bool displayNeedsReinit = true;

void VextON(void) {
  pinMode(VEXT_CTRL_1, OUTPUT);
  digitalWrite(VEXT_CTRL_1, HIGH);
  pinMode(VEXT_CTRL_2, OUTPUT);
  digitalWrite(VEXT_CTRL_2, HIGH);
}

void VextOFF(void) {
  pinMode(VEXT_CTRL_1, OUTPUT);
  digitalWrite(VEXT_CTRL_1, LOW);
  pinMode(VEXT_CTRL_2, OUTPUT);
  digitalWrite(VEXT_CTRL_2, LOW);
}

// This board ships with one of two possible e-paper controller chips
// depending on manufacturing batch. This is Heltec's own detection
// sequence (a raw bit-banged SPI read of a chip-ID register before the
// display object exists), ported verbatim from their Global_Simple.ino
// example rather than reverse-engineered.
void probeAndCreateDisplay() {
  pinMode(PIN_EINK_SCLK, OUTPUT);
  pinMode(PIN_EINK_DC, OUTPUT);
  pinMode(PIN_EINK_CS, OUTPUT);
  pinMode(PIN_EINK_RST, OUTPUT);

  digitalWrite(PIN_EINK_RST, LOW);
  delay(20);
  digitalWrite(PIN_EINK_RST, HIGH);
  delay(20);

  digitalWrite(PIN_EINK_DC, LOW);
  digitalWrite(PIN_EINK_CS, LOW);

  uint8_t cmd = 0x2F;
  pinMode(PIN_EINK_MOSI, OUTPUT);
  digitalWrite(PIN_EINK_SCLK, LOW);
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_EINK_MOSI, (cmd & 0x80) ? HIGH : LOW);
    cmd <<= 1;
    digitalWrite(PIN_EINK_SCLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_EINK_SCLK, LOW);
    delayMicroseconds(1);
  }
  delay(10);

  digitalWrite(PIN_EINK_DC, HIGH);
  pinMode(PIN_EINK_MOSI, INPUT_PULLUP);

  uint8_t chipId = 0;
  for (int8_t b = 7; b >= 0; b--) {
    digitalWrite(PIN_EINK_SCLK, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_EINK_SCLK, HIGH);
    delayMicroseconds(1);
    if (digitalRead(PIN_EINK_MOSI)) chipId |= (1 << b);
  }
  digitalWrite(PIN_EINK_CS, HIGH);

  if ((chipId & 0x03) != 0x01) {
    display = new HT_ICMEN2R13EFC1(PIN_EINK_RST, PIN_EINK_DC, PIN_EINK_CS, PIN_EINK_BUSY,
                                    PIN_EINK_SCLK, PIN_EINK_MOSI, -1, 6000000);
  } else {
    display = new HT_E0213A367(PIN_EINK_RST, PIN_EINK_DC, PIN_EINK_CS, PIN_EINK_BUSY,
                                PIN_EINK_SCLK, PIN_EINK_MOSI, -1, 6000000);
  }
}

// The display IC loses power entirely when Vext is off (see maybeSleep()),
// so it needs a full re-init -- not just a redraw -- the first time
// something tries to render after waking. Mirrors the CrowPanel build's
// epdNeedsFullInit flag.
void ensureDisplayAwake() {
  if (!displayNeedsReinit) return;
  VextON();
  delay(100);
  display->init();
  displayNeedsReinit = false;
}

// ---------------- UTIL ----------------
bool pressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void touchActivity() {
  lastActivity = millis();
}

String bookPath(const String& name) {
  return "/books/" + name;
}

String posPath(const String& name) {
  return "/books/" + name + ".pos";
}

String freeSpaceLabel() {
  size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  char buf[24];
  snprintf(buf, sizeof(buf), "%uKB free", (unsigned)(freeBytes / 1024));
  return String(buf);
}

// Truncates to fit maxWidth, measured against whatever font is currently
// active -- a fixed character-count cutoff (like "26 chars") doesn't scale
// correctly across ArialMT_Plain_10 vs _16 since it's a proportional font,
// not the CrowPanel build's fixed-width character blitter.
String truncateToWidth(const String& text, uint16_t maxWidth) {
  if (display->getStringWidth(text) <= maxWidth) return text;
  String truncated = text;
  while (truncated.length() > 1 && display->getStringWidth(truncated + "...") > maxWidth) {
    truncated.remove(truncated.length() - 1);
  }
  return truncated + "...";
}

void showMessage(const String& message) {
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_10);
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  uint16_t y = 2;
  int start = 0;
  while (start < (int)message.length()) {
    int nl = message.indexOf('\n', start);
    String line = (nl == -1) ? message.substring(start) : message.substring(start, nl);
    display->drawString(2, y, line);
    y += BOOK_LINE_HEIGHT;
    start = (nl == -1) ? message.length() : nl + 1;
  }

  display->update(BLACK_BUFFER);
  display->display();
}

// ---------------- LIBRARY ----------------
// Everything below in this section is unchanged from the CrowPanel build --
// LittleFS/book-library management doesn't touch the display or buttons at
// all, so it ported over verbatim.
void saveCurrentBookName() {
  File f = LittleFS.open("/current.txt", "w");
  if (!f) return;
  f.print(currentBookName);
  f.close();
}

String loadCurrentBookName() {
  File f = LittleFS.open("/current.txt", "r");
  if (!f) return "";
  String name = f.readString();
  f.close();
  name.trim();
  return name;
}

void savePosition() {
  if (currentBookName.length() == 0) return;
  File pos = LittleFS.open(posPath(currentBookName), "w");
  if (!pos) return;
  pos.print(pageStart);
  pos.close();
}

uint32_t loadSavedPosition(const String& name) {
  File pos = LittleFS.open(posPath(name), "r");
  if (!pos) return 0;
  uint32_t saved = pos.parseInt();
  pos.close();
  return saved;
}

void reopenBookAt(uint32_t offset) {
  if (book) book.close();
  if (currentBookName.length() == 0) {
    fileSize = 0;
    pageStart = 0;
    return;
  }
  book = LittleFS.open(bookPath(currentBookName), "r");
  if (!book) return;
  fileSize = book.size();
  if (offset > fileSize) offset = 0;
  pageStart = offset;
  book.seek(pageStart);
}

void listBooks() {
  bookCount = 0;
  bookFileTotal = 0;
  File dir = LittleFS.open("/books");
  if (!dir || !dir.isDirectory()) return;

  // Keeps counting past MAX_BOOKS (into bookFileTotal) even once bookList is
  // full, so any book that can't fit in the RAM array is surfaced instead of
  // silently vanishing while still eating flash space.
  File f = dir.openNextFile();
  while (f) {
    String name = String(f.name());
    int slash = name.lastIndexOf('/');
    if (slash != -1) name = name.substring(slash + 1);
    if (name.length() > 0 && !name.endsWith(".pos")) {
      if (bookCount < MAX_BOOKS) bookList[bookCount++] = name;
      bookFileTotal++;
    }
    f = dir.openNextFile();
  }
}

void renderChooseBook();
void scrollChooseWindow();

// Shared by the on-device delete flow and the web /delete endpoint. Returns
// false if the book didn't exist (nothing to delete).
bool deleteBookFile(const String& name) {
  if (!LittleFS.exists(bookPath(name))) return false;

  if (name == currentBookName) {
    if (book) book.close();
    currentBookName = "";
    LittleFS.remove("/current.txt");
  }

  LittleFS.remove(bookPath(name));
  LittleFS.remove(posPath(name));
  return true;
}

void deleteSelectedBook() {
  if (bookCount == 0) return;
  deleteBookFile(bookList[chooseSelection]);

  listBooks();
  if (bookCount == 0) {
    chooseSelection = 0;
  } else if (chooseSelection >= bookCount) {
    chooseSelection = bookCount - 1;
  }
  scrollChooseWindow();
  renderChooseBook();
}

void renderPageAt(uint32_t offset);

void indexChapters() {
  chapterCount = 0;
  chapterOffsets[chapterCount++] = 0;

  if (!book) return;

  book.seek(0);
  uint32_t offset = 0;
  while (book.available() && chapterCount < MAX_CHAPTERS) {
    int c = book.read();
    offset++;
    if (c == '\f') {
      chapterOffsets[chapterCount++] = offset;
    }
  }
}

void nextChapter() {
  for (uint16_t i = 0; i < chapterCount; i++) {
    if (chapterOffsets[i] > pageStart) {
      renderPageAt(chapterOffsets[i]);
      return;
    }
  }
}

void previousChapter() {
  for (int16_t i = (int16_t)chapterCount - 1; i >= 0; i--) {
    if (chapterOffsets[i] < pageStart) {
      renderPageAt(chapterOffsets[i]);
      return;
    }
  }
  if (chapterCount > 0) renderPageAt(chapterOffsets[0]);
}

void openBook(const String& name) {
  // indexChapters() does a byte-by-byte scan of the whole file, which can take
  // a couple of seconds on a large book -- show a message first so this reads
  // as "working" rather than a frozen screen.
  showMessage("Opening book...");

  currentBookName = name;
  saveCurrentBookName();
  uint32_t saved = loadSavedPosition(name);
  reopenBookAt(saved);
  indexChapters();
  currentScreen = SCREEN_READING;
  renderPageAt(saved);
}

// ---------------- WIFI + WEB ----------------
void setupAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_NAME, AP_PASS);
}

void handleRoot() {
  touchActivity();
  server.send(200, "text/html", uploadPage);
}

// Hand-built JSON (no ArduinoJson dependency) so the upload page can show
// the current library without a page reload. bookCount is capped at
// MAX_BOOKS by listBooks(), so this stays small and fast even though it
// opens each file just to read its size.
void handleBooksList() {
  touchActivity();
  listBooks();

  String json = "{\"books\":[";
  for (uint8_t i = 0; i < bookCount; i++) {
    if (i > 0) json += ",";
    File f = LittleFS.open(bookPath(bookList[i]), "r");
    size_t size = f ? f.size() : 0;
    if (f) f.close();
    json += "{\"name\":\"" + bookList[i] + "\",\"size\":" + String(size) + "}";
  }
  // freeBytes/maxSize let the upload page simulate handleUpload()'s own
  // acceptance math (min(MAX_BOOK_SIZE, freeSpace - 8192)) client-side, so
  // it can reject queued books that won't fit before even starting the
  // upload instead of finding out partway through a batch.
  size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  json += "],\"count\":" + String(bookCount) +
          ",\"hidden\":" + String(bookFileTotal > bookCount ? bookFileTotal - bookCount : 0) +
          ",\"free\":\"" + freeSpaceLabel() + "\"" +
          ",\"freeBytes\":" + String(freeBytes) +
          ",\"maxSize\":" + String(MAX_BOOK_SIZE) + "}";

  server.send(200, "application/json", json);
}

String sanitizeFilename(String name) {
  name.trim();
  String out;
  for (size_t i = 0; i < name.length() && out.length() < 40; i++) {
    char c = name[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
              c == '-' || c == '_' || c == '.' || c == ' ';
    if (ok) out += c;
  }
  if (out.length() == 0) out = "book";
  if (!out.endsWith(".txt")) out += ".txt";
  return out;
}

bool tooLarge = false;
String uploadName;
size_t uploadSizeLimit = 0;

void handleUpload() {
  HTTPUpload& upload = server.upload();
  static File incoming;
  touchActivity();

  if (upload.status == UPLOAD_FILE_START) {
    tooLarge = false;
    uploadName = sanitizeFilename(upload.filename);

    if (!LittleFS.exists("/books")) LittleFS.mkdir("/books");

    size_t freeSpace = LittleFS.totalBytes() - LittleFS.usedBytes();
    uploadSizeLimit = min((size_t)MAX_BOOK_SIZE, freeSpace > 8192 ? freeSpace - 8192 : 0);
    if (uploadSizeLimit == 0) {
      tooLarge = true;
      return;
    }

    if (LittleFS.exists(posPath(uploadName))) LittleFS.remove(posPath(uploadName));
    incoming = LittleFS.open(bookPath(uploadName), "w");
  }

  if (upload.status == UPLOAD_FILE_WRITE && incoming && !tooLarge) {
    if (incoming.size() + upload.currentSize > uploadSizeLimit) {
      tooLarge = true;
      incoming.close();
      LittleFS.remove(bookPath(uploadName));
      return;
    }
    incoming.write(upload.buf, upload.currentSize);
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (incoming) incoming.close();
  }
}

void handleUploadComplete() {
  if (tooLarge) {
    server.send(413, "text/plain", "File too large (limit 5MB per book, or not enough free space)");
    return;
  }

  // Deliberately doesn't open the book -- uploading just adds it to the
  // library so the page stays put for uploading more. Open it from the
  // device's Choose Book screen.
  server.send(200, "text/plain", "Upload complete: " + uploadName);
}

void handleDeleteBook() {
  touchActivity();
  // Reuses the same sanitizer as uploads so this can't be pointed outside
  // /books via the name query param.
  String name = sanitizeFilename(server.arg("name"));
  if (!deleteBookFile(name)) {
    server.send(404, "text/plain", "Book not found: " + name);
    return;
  }
  server.send(200, "text/plain", "Deleted: " + name);
}

void registerWebRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/books", HTTP_GET, handleBooksList);
  server.on("/delete", HTTP_POST, handleDeleteBook);
  server.on("/upload", HTTP_POST, handleUploadComplete, handleUpload);
}

void setupWebServer() {
  server.begin();
}

// ---------------- READER ----------------
// UTF-8 handling: Heltec's own utf8ascii() (used internally by drawString)
// already fails safe on anything it doesn't recognize -- it silently drops
// the character rather than corrupting the rest of the line the way the
// CrowPanel's raw font renderer did. This transliteration pass is purely a
// readability upgrade (curly quotes/dashes/ellipses become their plain
// ASCII equivalents instead of vanishing), not a correctness fix.
String utf8ToAsciiReplacement(uint8_t leadByte) {
  uint8_t extraBytes;
  uint32_t codepoint;
  if ((leadByte & 0xE0) == 0xC0) {
    extraBytes = 1;
    codepoint = leadByte & 0x1F;
  } else if ((leadByte & 0xF0) == 0xE0) {
    extraBytes = 2;
    codepoint = leadByte & 0x0F;
  } else if ((leadByte & 0xF8) == 0xF0) {
    extraBytes = 3;
    codepoint = leadByte & 0x07;
  } else {
    return "";
  }

  for (uint8_t i = 0; i < extraBytes && book.available(); i++) {
    uint8_t cont = (uint8_t)book.peek();
    if ((cont & 0xC0) != 0x80) break;
    book.read();
    codepoint = (codepoint << 6) | (cont & 0x3F);
  }

  switch (codepoint) {
    case 0x2018: case 0x2019: return "'";
    case 0x201C: case 0x201D: return "\"";
    case 0x2013: return "-";
    case 0x2014: return "--";
    case 0x2026: return "...";
    case 0x00A0: return " ";
    default: return "";
  }
}

// Same greedy word-wrap architecture as the CrowPanel build (lone '\n' is a
// word separator, not a break; two-or-more is a real paragraph break; '\f'
// is a chapter marker and a hard page-ending break), except the fit test
// uses the font's actual pixel width (display->getStringWidth) instead of
// a fixed character count, since this board's fonts are proportional.
String readWrappedLine(bool* hitChapterBreak = nullptr) {
  String line;
  if (hitChapterBreak) *hitChapterBreak = false;

  while (true) {
    while (book.available() && (book.peek() == ' ' || book.peek() == '\r')) book.read();

    if (!book.available()) break;

    if (book.peek() == '\f') {
      book.read();
      if (hitChapterBreak) *hitChapterBreak = true;
      break;
    }

    if (book.peek() == '\n') {
      book.read();
      while (book.available() && book.peek() == '\r') book.read();
      if (book.available() && book.peek() == '\n') {
        while (book.available() && (book.peek() == '\n' || book.peek() == '\r')) book.read();
        break;
      }
      continue;
    }

    uint32_t wordStart = book.position();
    String word;
    while (book.available()) {
      uint8_t c = (uint8_t)book.peek();
      if (c == ' ' || c == '\n' || c == '\r' || c == '\f') break;
      book.read();

      if (c >= ' ' && c <= '~') {
        word += (char)c;
      } else if (c >= 0x80) {
        word += utf8ToAsciiReplacement(c);
      }
    }

    if (display->getStringWidth(word) > BOOK_USABLE_WIDTH) {
      if (line.length() > 0) {
        book.seek(wordStart);
        break;
      }
      // Single word wider than the whole line (long URL, etc): trim from
      // the end until it fits rather than looping forever.
      while (word.length() > 1 && display->getStringWidth(word) > BOOK_USABLE_WIDTH) {
        word.remove(word.length() - 1);
      }
      book.seek(wordStart + word.length());
      return word;
    }

    String candidate = line;
    if (candidate.length() > 0) candidate += ' ';
    candidate += word;

    if (display->getStringWidth(candidate) > BOOK_USABLE_WIDTH) {
      book.seek(wordStart);
      break;
    }

    line = candidate;
  }

  return line;
}

// Shared by the real render path and findPreviousPageStart()'s
// forward-replay simulation -- both need to agree exactly on where pages
// break. Assumes display->setFont() has already selected the body font,
// since readWrappedLine measures pixel widths against whatever font is
// currently active.
uint32_t computePageEnd(uint32_t offset, String* outLines, uint8_t* outLineCount) {
  if (!book) return offset;
  if (offset > fileSize) offset = 0;
  book.seek(offset);

  uint8_t lineCount = 0;
  while (book.available() && lineCount < BOOK_MAX_LINES) {
    bool hitChapterBreak = false;
    String line = readWrappedLine(&hitChapterBreak);
    if (outLines) outLines[lineCount] = line;
    if (line.length() > 0 || book.available()) lineCount++;
    if (hitChapterBreak) break;
  }

  if (outLineCount) *outLineCount = lineCount;
  return book.position();
}

// True sequential "page up": replays forward pagination from the current
// chapter's start (or the previous chapter, if already at a chapter's first
// page) up to the current page, and returns the start of the page right
// before it. Cached so repeated/held presses don't redo the replay from
// scratch every time. Same approach as the CrowPanel build.
uint32_t findPreviousPageStart(uint32_t currentStart) {
  if (currentStart == 0) return 0;

  int16_t chapterIndex = 0;
  for (uint16_t i = 0; i < chapterCount; i++) {
    if (chapterOffsets[i] <= currentStart) chapterIndex = i;
    else break;
  }

  uint32_t replayFrom = chapterOffsets[chapterIndex];
  uint32_t replayTo = currentStart;
  if (replayFrom == currentStart) {
    if (chapterIndex == 0) return 0;
    replayFrom = chapterOffsets[chapterIndex - 1];
    replayTo = chapterOffsets[chapterIndex];
  }

  bool cacheCoversTarget = cachedChapterStart == replayFrom && cachedPageCount > 0 &&
                            cachedPageStarts[cachedPageCount - 1] >= replayTo;
  if (!cacheCoversTarget) {
    cachedChapterStart = replayFrom;
    cachedPageCount = 0;
    cachedPageStarts[cachedPageCount++] = replayFrom;

    uint32_t simulated = replayFrom;
    while (simulated < replayTo && cachedPageCount < MAX_CACHED_PAGES) {
      uint32_t nextSimulated = computePageEnd(simulated, nullptr, nullptr);
      if (nextSimulated <= simulated) break;
      simulated = nextSimulated;
      cachedPageStarts[cachedPageCount++] = simulated;
    }
  }

  uint32_t previousStart = replayFrom;
  for (uint16_t i = 0; i < cachedPageCount; i++) {
    if (cachedPageStarts[i] >= replayTo) break;
    previousStart = cachedPageStarts[i];
  }
  return previousStart;
}

void renderPageAt(uint32_t offset) {
  reopenBookAt(offset);
  if (!book) {
    showMessage("Book open failed");
    return;
  }

  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_16);

  String lines[BOOK_MAX_LINES];
  uint8_t lineCount = 0;
  pageEnd = computePageEnd(offset, lines, &lineCount);

  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  for (uint8_t i = 0; i < lineCount; i++) {
    display->drawString(BOOK_LEFT_MARGIN, BOOK_TOP_MARGIN + i * BOOK_LINE_HEIGHT, lines[i]);
  }
  display->update(BLACK_BUFFER);
  display->display();

  savePosition();
  touchActivity();
}

void nextPage() {
  if (!book || pageEnd >= fileSize) return;
  renderPageAt(pageEnd);
}

void previousPage() {
  if (!book || pageStart == 0) return;
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_16);  // findPreviousPageStart's replay measures against this
  renderPageAt(findPreviousPageStart(pageStart));
}

// ---------------- MENUS ----------------
void renderHome() {
  ensureDisplayAwake();
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_16);  // matches the reading/menu screens' text size

  for (uint8_t i = 0; i < HOME_ITEM_COUNT; i++) {
    uint16_t x = HOME_ICON_X[i];
    display->drawXbm(x, HOME_ICON_Y, HOME_ICON_SIZE, HOME_ICON_SIZE, HOME_ICONS[i]);

    const char* label = HOME_SHORT_LABELS[i];
    uint16_t labelWidth = display->getStringWidth(String(label));
    uint16_t labelX = x + (HOME_ICON_SIZE - labelWidth) / 2;
    display->drawString(labelX, HOME_LABEL_Y, label);

    if (i == homeSelection) {
      display->drawRect(x - 4, HOME_ICON_Y - 4, HOME_ICON_SIZE + 8,
                         (HOME_LABEL_Y + HOME_LABEL_FONT_SIZE + 2) - (HOME_ICON_Y - 4));
    }
  }

  // Only Right-hold does anything on this screen (selects the highlighted
  // item); both-hold is a no-op here since Home is already Home. Kept at
  // the small font -- it's a static caption, not a navigable line, and
  // there's no vertical room left for it at the bigger size anyway.
  display->setFont(ArialMT_Plain_10);
  display->drawString(4, 102, "Tap=Move  Hold Right=Select");

  display->update(BLACK_BUFFER);
  display->display();
}

// Keeps chooseSelection inside [chooseWindowStart, chooseWindowStart +
// CHOOSE_VISIBLE_ROWS) by scrolling the window the minimum amount needed --
// called whenever chooseSelection or bookCount changes.
void scrollChooseWindow() {
  if (chooseSelection < chooseWindowStart) {
    chooseWindowStart = chooseSelection;
  } else if (chooseSelection >= chooseWindowStart + CHOOSE_VISIBLE_ROWS) {
    chooseWindowStart = chooseSelection - CHOOSE_VISIBLE_ROWS + 1;
  }
  uint8_t maxStart = (bookCount > CHOOSE_VISIBLE_ROWS) ? bookCount - CHOOSE_VISIBLE_ROWS : 0;
  if (chooseWindowStart > maxStart) chooseWindowStart = maxStart;
}

// Row 0 is a free-space header (plus ^/v hints when the list scrolls); the
// book list starts one row down and shows at most CHOOSE_VISIBLE_ROWS at a
// time.
void renderChooseBook() {
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_16);
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  String header = freeSpaceLabel();
  if (bookCount > CHOOSE_VISIBLE_ROWS) {
    header += "  ";
    header += (chooseWindowStart > 0) ? "^" : " ";
    header += (chooseWindowStart + CHOOSE_VISIBLE_ROWS < bookCount) ? "v" : " ";
  }
  display->drawString(MENU_LEFT_MARGIN, 2, header);

  if (bookCount == 0) {
    display->drawString(MENU_LEFT_MARGIN, 2 + MENU_LINE_HEIGHT, "No books yet.");
    display->drawString(MENU_LEFT_MARGIN, 2 + 2 * MENU_LINE_HEIGHT, "Upload one first.");
  } else {
    uint8_t windowEnd = min((uint8_t)(chooseWindowStart + CHOOSE_VISIBLE_ROWS), bookCount);
    for (uint8_t i = chooseWindowStart; i < windowEnd; i++) {
      uint8_t row = i - chooseWindowStart;
      String label = (i == chooseSelection) ? "> " : "  ";
      label += truncateToWidth(bookList[i], BOOK_USABLE_WIDTH - display->getStringWidth(label));
      display->drawString(MENU_LEFT_MARGIN, 2 + (row + 1) * MENU_LINE_HEIGHT, label);
    }
  }

  display->update(BLACK_BUFFER);
  display->display();
}

void renderConfirmDelete() {
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_16);
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  display->drawString(MENU_LEFT_MARGIN, 2, "Delete this book?");
  String name = (chooseSelection < bookCount) ? bookList[chooseSelection] : "";
  name = truncateToWidth(name, BOOK_USABLE_WIDTH);
  display->drawString(MENU_LEFT_MARGIN, 2 + MENU_LINE_HEIGHT, name);

  String yesLabel = confirmDeleteYes ? "> Yes" : "  Yes";
  String noLabel = !confirmDeleteYes ? "> No" : "  No";
  display->drawString(MENU_LEFT_MARGIN, 2 + 3 * MENU_LINE_HEIGHT, yesLabel);
  display->drawString(MENU_LEFT_MARGIN, 2 + 4 * MENU_LINE_HEIGHT, noLabel);

  display->update(BLACK_BUFFER);
  display->display();
}

void renderWifiInfo() {
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_10);
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  constexpr uint16_t leftX = 4;
  constexpr uint16_t rightX = leftX + QRWIFI_W + 8;
  constexpr uint16_t qrY = 20;

  display->drawString(leftX, 2, "Wi-Fi");
  display->drawXbm(leftX, qrY, QRWIFI_W, QRWIFI_H, qrWifi);

  display->drawString(rightX, 2, "Web");
  display->drawXbm(rightX, qrY, QRWEBPAGE_W, QRWEBPAGE_H, qrWebpage);

  display->drawString(leftX, qrY + QRWIFI_H + 4, "192.168.4.1");

  display->update(BLACK_BUFFER);
  display->display();
}

void enterWifiInfo() {
  currentScreen = SCREEN_WIFI_INFO;
  setupAP();
  setupWebServer();
  touchActivity();
  renderWifiInfo();
}

void exitWifiInfo() {
  server.stop();
  WiFi.mode(WIFI_OFF);
}

void enterHome() {
  currentScreen = SCREEN_HOME;
  homeSelection = 0;
  renderHome();
}

void enterConfirmDelete() {
  confirmDeleteYes = false;
  currentScreen = SCREEN_CONFIRM_DELETE;
  renderConfirmDelete();
}

void selectHomeItem() {
  switch (homeSelection) {
    case 0:  // Resume Last Book
      if (currentBookName.length() > 0) {
        currentScreen = SCREEN_READING;
        renderPageAt(pageStart);
      }
      break;
    case 1:  // Choose Book
      listBooks();
      chooseSelection = 0;
      chooseWindowStart = 0;
      currentScreen = SCREEN_CHOOSE_BOOK;
      renderChooseBook();
      break;
    case 2:  // Connect to Wi-Fi
      enterWifiInfo();
      break;
  }
}

// ---------------- BUTTONS + SLEEP ----------------

// How much of a head start the earlier-pressed button gets credited with
// once the second one joins -- see the bothPressStart comment below.
constexpr unsigned long BOTH_HOLD_STAGGER_GRACE_MS = 300;

// Distinguishes tap/hold/both-hold from two plain buttons. "Both held"
// takes priority the moment a second button joins in -- an individual
// button's hold timer is ignored for as long as both are down, and only
// resumes counting from the original press time if the other one lets go
// first. A tap fires on release only if nothing was already resolved
// (hold or both-hold) during that press.
ButtonGesture pollButtons() {
  static bool lastLeft = false, lastRight = false;
  static unsigned long leftPressStart = 0, rightPressStart = 0, bothPressStart = 0;
  static bool leftResolved = false, rightResolved = false, bothResolved = false;

  bool nowLeft = pressed(BTN_LEFT);
  bool nowRight = pressed(BTN_RIGHT);
  unsigned long now = millis();

  if (nowLeft && !lastLeft) { leftPressStart = now; leftResolved = false; }
  if (nowRight && !lastRight) { rightPressStart = now; rightResolved = false; }
  if (nowLeft && nowRight && !(lastLeft && lastRight)) {
    bothResolved = false;
    // Two physical buttons are rarely pressed within the same tens of
    // milliseconds of each other -- crediting only the moment the second
    // one joins would mean the user has to hold both together for the
    // *entire* HOLD_THRESHOLD_MS, on top of however long they'd already
    // been juggling getting both fingers down. Backdating the start time
    // to the earlier button's own press (capped at BOTH_HOLD_STAGGER_GRACE_MS
    // of credit) forgives a staggered press without making a brief
    // incidental overlap during an unrelated long single-button hold
    // (e.g. while auto-repeating pages) instantly count as a chord.
    unsigned long earlier = min(leftPressStart, rightPressStart);
    unsigned long earliestCreditable = now - BOTH_HOLD_STAGGER_GRACE_MS;
    bothPressStart = (earlier > earliestCreditable) ? earlier : earliestCreditable;
  }

  ButtonGesture gesture = GESTURE_NONE;

  if (nowLeft && nowRight) {
    if (!bothResolved && now - bothPressStart >= HOLD_THRESHOLD_MS) {
      gesture = GESTURE_BOTH_HOLD;
      bothResolved = true;
      leftResolved = true;
      rightResolved = true;
    }
  } else {
    if (nowLeft && !leftResolved && now - leftPressStart >= HOLD_THRESHOLD_MS) {
      gesture = GESTURE_LEFT_HOLD;
      leftResolved = true;
    }
    if (nowRight && !rightResolved && now - rightPressStart >= HOLD_THRESHOLD_MS) {
      gesture = GESTURE_RIGHT_HOLD;
      rightResolved = true;
    }
  }

  if (!nowLeft && lastLeft && !leftResolved) gesture = GESTURE_LEFT_TAP;
  if (!nowRight && lastRight && !rightResolved) gesture = GESTURE_RIGHT_TAP;

  lastLeft = nowLeft;
  lastRight = nowRight;
  return gesture;
}

// Handles every screen except SCREEN_READING, which has its own gesture
// vocabulary (double-tap, hold-to-repeat) driven by handleReadingButtons()
// below instead of this plain tap/hold/both-hold dispatch.
void dispatchGesture(ButtonGesture gesture) {
  switch (currentScreen) {
    case SCREEN_HOME:
      switch (gesture) {
        case GESTURE_LEFT_TAP:
          homeSelection = (homeSelection + HOME_ITEM_COUNT - 1) % HOME_ITEM_COUNT;
          renderHome();
          break;
        case GESTURE_RIGHT_TAP:
          homeSelection = (homeSelection + 1) % HOME_ITEM_COUNT;
          renderHome();
          break;
        case GESTURE_RIGHT_HOLD: selectHomeItem(); break;
        case GESTURE_BOTH_HOLD: enterHome(); break;
        default: break;
      }
      break;

    case SCREEN_CHOOSE_BOOK:
      if (bookCount > 0) {
        switch (gesture) {
          case GESTURE_LEFT_TAP:
            chooseSelection = (chooseSelection + bookCount - 1) % bookCount;
            scrollChooseWindow();
            renderChooseBook();
            break;
          case GESTURE_RIGHT_TAP:
            chooseSelection = (chooseSelection + 1) % bookCount;
            scrollChooseWindow();
            renderChooseBook();
            break;
          case GESTURE_RIGHT_HOLD: openBook(bookList[chooseSelection]); break;
          case GESTURE_LEFT_HOLD: enterConfirmDelete(); break;
          default: break;
        }
      }
      if (gesture == GESTURE_BOTH_HOLD) enterHome();
      break;

    case SCREEN_WIFI_INFO:
      if (gesture == GESTURE_RIGHT_HOLD || gesture == GESTURE_BOTH_HOLD) {
        exitWifiInfo();
        enterHome();
      }
      break;

    case SCREEN_CONFIRM_DELETE:
      switch (gesture) {
        case GESTURE_LEFT_TAP:
        case GESTURE_RIGHT_TAP:
          confirmDeleteYes = !confirmDeleteYes;
          renderConfirmDelete();
          break;
        case GESTURE_RIGHT_HOLD:
          currentScreen = SCREEN_CHOOSE_BOOK;
          if (confirmDeleteYes) {
            deleteSelectedBook();  // re-renders Choose Book itself
          } else {
            renderChooseBook();
          }
          break;
        case GESTURE_BOTH_HOLD:
          enterHome();
          break;
        default: break;
      }
      break;

    default: break;  // SCREEN_READING is handled separately
  }
}

constexpr unsigned long DOUBLE_TAP_WINDOW_MS = 250;  // see file header for the latency tradeoff this implies

// Reading screen only: a tap only fires (page turn) once DOUBLE_TAP_WINDOW_MS
// passes without a second tap on the same button following it, in which
// case that pair fires a chapter skip instead. Left/Right-hold from
// pollButtons() are ignored here (their single-fire nature doesn't suit
// "keep paging while held") in favor of directly polling the raw pin state
// below for continuous auto-repeat, the same way the CrowPanel build's
// Top/Bottom hold-to-repeat works.
void handleReadingButtons(ButtonGesture gesture) {
  static bool leftTapPending = false, rightTapPending = false;
  static unsigned long leftTapTime = 0, rightTapTime = 0;
  unsigned long now = millis();

  if (leftTapPending && now - leftTapTime >= DOUBLE_TAP_WINDOW_MS) {
    leftTapPending = false;
    previousPage();
  }
  if (rightTapPending && now - rightTapTime >= DOUBLE_TAP_WINDOW_MS) {
    rightTapPending = false;
    nextPage();
  }

  static unsigned long leftHoldStart = 0, leftLastRepeat = 0;
  static unsigned long rightHoldStart = 0, rightLastRepeat = 0;
  static bool wasLeftDown = false, wasRightDown = false;
  bool nowLeftDown = pressed(BTN_LEFT);
  bool nowRightDown = pressed(BTN_RIGHT);

  if (nowLeftDown && !wasLeftDown) { leftHoldStart = now; leftLastRepeat = now; }
  if (nowRightDown && !wasRightDown) { rightHoldStart = now; rightLastRepeat = now; }

  // Suppressed while both are down so a both-held-for-Home attempt doesn't
  // also spray page turns while the user is getting both fingers on.
  if (!(nowLeftDown && nowRightDown)) {
    if (nowLeftDown && now - leftHoldStart >= PAGE_REPEAT_DELAY_MS &&
        now - leftLastRepeat >= PAGE_REPEAT_INTERVAL_MS) {
      leftLastRepeat = now;
      touchActivity();
      previousPage();
    }
    if (nowRightDown && now - rightHoldStart >= PAGE_REPEAT_DELAY_MS &&
        now - rightLastRepeat >= PAGE_REPEAT_INTERVAL_MS) {
      rightLastRepeat = now;
      touchActivity();
      nextPage();
    }
  }

  wasLeftDown = nowLeftDown;
  wasRightDown = nowRightDown;

  switch (gesture) {
    case GESTURE_LEFT_TAP:
      if (leftTapPending) {
        leftTapPending = false;
        touchActivity();
        previousChapter();
      } else {
        leftTapPending = true;
        leftTapTime = now;
      }
      break;
    case GESTURE_RIGHT_TAP:
      if (rightTapPending) {
        rightTapPending = false;
        touchActivity();
        nextChapter();
      } else {
        rightTapPending = true;
        rightTapTime = now;
      }
      break;
    case GESTURE_BOTH_HOLD:
      touchActivity();
      enterHome();
      break;
    default: break;  // LEFT_HOLD/RIGHT_HOLD: repeat is handled above instead
  }
}

void handleButtons() {
  ButtonGesture gesture = pollButtons();

  // The reading screen needs to run its repeat/double-tap timing checks
  // every tick, not just when a new gesture fires, so it's dispatched
  // unconditionally instead of gated behind gesture != GESTURE_NONE below.
  if (currentScreen == SCREEN_READING) {
    handleReadingButtons(gesture);
    return;
  }

  if (gesture == GESTURE_NONE) return;
  touchActivity();
  dispatchGesture(gesture);
}

void maybeSleep() {
  if (millis() - lastActivity < SLEEP_AFTER_MS) return;

  bool wifiActive = (currentScreen == SCREEN_WIFI_INFO);

  VextOFF();
  displayNeedsReinit = true;
  if (wifiActive) exitWifiInfo();
  btStop();

  uint64_t wakeMask = (1ULL << BTN_LEFT) | (1ULL << BTN_RIGHT);
  esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_light_sleep_start();

  touchActivity();
  if (wifiActive) {
    setupAP();
    setupWebServer();
  }
}

// ---------------- ARDUINO ----------------
void setup() {
  Serial.begin(115200);

  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  VextON();
  delay(100);
  probeAndCreateDisplay();
  display->init();
  displayNeedsReinit = false;

  if (!LittleFS.begin(true)) {
    showMessage("LittleFS failed");
    while (true) delay(1000);
  }
  if (!LittleFS.exists("/books")) LittleFS.mkdir("/books");

  registerWebRoutes();
  WiFi.mode(WIFI_OFF);

  currentBookName = loadCurrentBookName();
  if (currentBookName.length() == 0 || !LittleFS.exists(bookPath(currentBookName))) {
    listBooks();
    currentBookName = (bookCount > 0) ? bookList[0] : "";
  }

  // Loads/indexes the last book (if any) so Resume Last Book is instant,
  // but boots to the Home screen rather than straight into reading.
  uint32_t saved = currentBookName.length() > 0 ? loadSavedPosition(currentBookName) : 0;
  reopenBookAt(saved);
  indexChapters();
  currentScreen = SCREEN_HOME;
  renderHome();
}

void loop() {
  server.handleClient();
  handleButtons();
  maybeSleep();
  delay(20);
}
