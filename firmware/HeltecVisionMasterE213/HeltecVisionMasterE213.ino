/*
  Tiny E-Reader -- Heltec Vision Master E213 port
  ESP32-S3 + 2.13" e-paper (250x122), SX1262 LoRa radio present but unused.

  This is a from-scratch port of the CrowPanel-based TinyEReader to
  different hardware, built and compile-tested without physical access to
  the board -- see hardware-research/heltec-e213 in this repo for the
  schematic/datasheet/factory-test sources every pin number and API call
  here is drawn from. Treat anything marked "verify on hardware" as an
  educated guess pending real testing, not a confirmed fact.

  Key hardware differences from the CrowPanel build this was ported from:
  - Only 2 software-readable buttons exist (not the CrowPanel's 5 inputs).
    The board has a 3rd physical button (Reset), but it's wired straight to
    the chip's hardware EN pin, not a GPIO -- pressing it just restarts the
    chip from scratch, so software can't read it as a normal button. See
    the button scheme below.
  - Real battery voltage sensing exists (GPIO7, ADC) -- the CrowPanel had
    none at all.
  - The display driver is Heltec's own library (proportional fonts, built-in
    word-wrap) rather than Elecrow's fixed-width character blitter.
  - This board ships with one of two possible e-paper controller chips
    depending on manufacturing batch (same kind of ambiguity as the
    CrowPanel's SSD1680/JD79661 situation) -- detected at runtime via a raw
    SPI chip-ID probe, exactly as Heltec's own example does it.
  - 16MB flash (double the CrowPanel's 8MB), so the custom partition table
    here gives LittleFS roughly 13.9MB.

  Button scheme (only 2 real buttons: Left=GPIO0/BOOT, Right=GPIO21/USER --
  which physical side is actually "left" vs "right" is a guess without the
  board in hand; swap the two #defines below if it's backwards):

  Reading a book:
    Left tap    -> previous page       Right tap    -> next page
    Left hold   -> previous chapter    Right hold   -> next chapter
    Both held   -> Home menu

  Everywhere else (Home / Choose Book / Wi-Fi / delete dialog):
    Left tap    -> move selection      Right tap    -> move selection
    Left hold   -> open delete dialog (Choose Book only, on a highlighted book)
    Right hold  -> select / confirm
    Both held   -> Home menu (also cancels the delete dialog without deleting)

  Refresh strategy: every render is a full-screen refresh. Heltec's display
  library only exposes true partial/windowed refresh via low-level
  single-character calls that would need to be driven directly instead of
  through the normal buffer+drawString API -- reverse-engineering that
  without hardware to validate against was a risk not worth taking for a
  first port. Once real hardware is available and this feels slow, that's
  the next thing to optimize (the CrowPanel build went through the same
  full-refresh-first, partial-refresh-later progression).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <esp_system.h>
#include "HT_lCMEN2R13EFC1.h"
#include "HT_E0213A367.h"

// ---------------- HARDWARE CONFIG ----------------
// E-paper SPI pins, Vext (peripheral power) pins, and the battery ADC pin
// are all taken directly from Heltec's own factory-test sketch and example
// code for this exact board (see hardware-research/heltec-e213), not
// guessed.
#define PIN_EINK_RST   3
#define PIN_EINK_DC    2
#define PIN_EINK_CS    5
#define PIN_EINK_BUSY  1
#define PIN_EINK_SCLK  4
#define PIN_EINK_MOSI  6

#define VEXT_CTRL_1  18
#define VEXT_CTRL_2  46

#define BATTERY_ADC_PIN 7

// Only two software-readable buttons exist -- see the button scheme in the
// file header. "Left"/"Right" is a guess at physical position; swap these
// two values if it's backwards once you can see the board.
#define BTN_LEFT   0   // BOOT
#define BTN_RIGHT  21  // USER

// ---------------- APP CONFIG ----------------
const char* AP_NAME = "PocketReader";
const char* AP_PASS = "12345678";

// Per-book sanity cap -- the real limit is whatever free space
// handleUpload() finds at upload time. See partitions.csv: LittleFS gets
// roughly 13.9MB of this board's 16MB flash.
constexpr size_t MAX_BOOK_SIZE = 5 * 1024 * 1024;
constexpr uint32_t SLEEP_AFTER_MS = 60000;
constexpr uint8_t MAX_BOOKS = 8;  // Choose Book's list isn't scrollable yet

constexpr unsigned long HOLD_THRESHOLD_MS = 500;  // tap vs. hold cutoff for both buttons

// Reading screen layout. ArialMT_Plain_10's line height is inferred from
// spacing used in Heltec's own demo code (not a documented constant) --
// verify once real hardware is available and adjust if lines overlap or
// leave too much gap.
constexpr uint16_t SCREEN_W = 250;
constexpr uint16_t SCREEN_H = 122;
constexpr uint16_t BOOK_LEFT_MARGIN = 2;
constexpr uint16_t BOOK_TOP_MARGIN = 1;
constexpr uint16_t BOOK_LINE_HEIGHT = 13;
constexpr uint16_t BOOK_USABLE_WIDTH = SCREEN_W - 2 * BOOK_LEFT_MARGIN;
constexpr uint8_t BOOK_MAX_LINES = 9;  // (122-1)/13, generous vs. the CrowPanel's 6

constexpr uint16_t MENU_LEFT_MARGIN = 2;
constexpr uint16_t MENU_LINE_HEIGHT = 13;

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
const char* HOME_LABELS[HOME_ITEM_COUNT] = { "Resume Last Book", "Choose Book", "Connect to Wi-Fi" };
uint8_t homeSelection = 0;

String bookList[MAX_BOOKS];
uint8_t bookCount = 0;
uint8_t chooseSelection = 0;
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
    body { font-family: system-ui, sans-serif; margin: 2rem; line-height: 1.35; }
    input, button { font: inherit; margin-top: 1rem; }
  </style>
</head>
<body>
  <h1>PocketReader</h1>
  <p>Uploading adds a new book to your library. It becomes the active book right away.</p>
  <form method="POST" action="/upload" enctype="multipart/form-data">
    <input type="file" name="file" accept=".txt,text/plain">
    <button type="submit">Upload TXT</button>
  </form>
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

constexpr float BATTERY_ADC_RESOLUTION = 1.0f / 4096.0f;
constexpr float BATTERY_ADC_REF_V = 3.3f;
constexpr float BATTERY_DIVIDER_COEFFICIENT = 4.01f;  // from Heltec's own factory-test formula

float readBatteryVoltage() {
  analogReadResolution(12);
  int raw = analogRead(BATTERY_ADC_PIN);
  return raw * BATTERY_ADC_RESOLUTION * BATTERY_ADC_REF_V * BATTERY_DIVIDER_COEFFICIENT;
}

// Linear approximation of a LiPo's 3.3V (empty) - 4.2V (full) range. Real
// LiPo discharge curves aren't linear, but without hardware to calibrate
// against, this is a reasonable first pass -- worth revisiting once you can
// compare the reported percentage against a real charge/discharge cycle.
uint8_t batteryPercent() {
  float v = readBatteryVoltage();
  float pct = (v - 3.3f) / (4.2f - 3.3f) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)(pct + 0.5f);
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
  File dir = LittleFS.open("/books");
  if (!dir || !dir.isDirectory()) return;

  File f = dir.openNextFile();
  while (f && bookCount < MAX_BOOKS) {
    String name = String(f.name());
    int slash = name.lastIndexOf('/');
    if (slash != -1) name = name.substring(slash + 1);
    if (name.length() > 0 && !name.endsWith(".pos")) {
      bookList[bookCount++] = name;
    }
    f = dir.openNextFile();
  }
}

void renderChooseBook();

void deleteSelectedBook() {
  if (bookCount == 0) return;
  String name = bookList[chooseSelection];

  if (name == currentBookName) {
    if (book) book.close();
    currentBookName = "";
    LittleFS.remove("/current.txt");
  }

  LittleFS.remove(bookPath(name));
  LittleFS.remove(posPath(name));

  listBooks();
  if (bookCount == 0) {
    chooseSelection = 0;
  } else if (chooseSelection >= bookCount) {
    chooseSelection = bookCount - 1;
  }
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
    server.send(413, "text/plain", "File too large or library full");
    return;
  }

  openBook(uploadName);
  server.send(200, "text/plain", "Upload complete: " + uploadName);
}

void registerWebRoutes() {
  server.on("/", HTTP_GET, handleRoot);
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
  if (currentBookName.length() == 0 || !LittleFS.exists(bookPath(currentBookName))) {
    showMessage("Upload a TXT at\nhttp://192.168.4.1\nor hold both buttons\nfor the Home menu");
    return;
  }

  reopenBookAt(offset);
  if (!book) {
    showMessage("Book open failed");
    return;
  }

  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_10);

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
  display->setFont(ArialMT_Plain_10);  // findPreviousPageStart's replay measures against this
  renderPageAt(findPreviousPageStart(pageStart));
}

// ---------------- MENUS ----------------
void renderHome() {
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_16);
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  constexpr uint16_t lineH = 22;
  for (uint8_t i = 0; i < HOME_ITEM_COUNT; i++) {
    String label = (i == homeSelection) ? "> " : "  ";
    label += HOME_LABELS[i];
    display->drawString(4, 6 + i * lineH, label);
  }

  display->setFont(ArialMT_Plain_10);
  char buf[24];
  snprintf(buf, sizeof(buf), "Battery: %u%%", batteryPercent());
  display->drawString(4, 6 + HOME_ITEM_COUNT * lineH + 6, buf);

  display->update(BLACK_BUFFER);
  display->display();
}

void renderChooseBook() {
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_10);
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  char freeBuf[24];
  snprintf(freeBuf, sizeof(freeBuf), "%uKB free", (unsigned)(freeBytes / 1024));
  display->drawString(MENU_LEFT_MARGIN, 2, freeBuf);

  if (bookCount == 0) {
    display->drawString(MENU_LEFT_MARGIN, 2 + MENU_LINE_HEIGHT, "No books yet.");
    display->drawString(MENU_LEFT_MARGIN, 2 + 2 * MENU_LINE_HEIGHT, "Upload one first.");
  } else {
    for (uint8_t i = 0; i < bookCount; i++) {
      String label = (i == chooseSelection) ? "> " : "  ";
      String name = bookList[i];
      if (name.length() > 34) name = name.substring(0, 31) + "...";
      label += name;
      display->drawString(MENU_LEFT_MARGIN, 2 + (i + 1) * MENU_LINE_HEIGHT, label);
    }
  }

  display->update(BLACK_BUFFER);
  display->display();
}

void renderConfirmDelete() {
  ensureDisplayAwake();
  display->setFont(ArialMT_Plain_10);
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  display->drawString(MENU_LEFT_MARGIN, 2, "Delete this book?");
  String name = (chooseSelection < bookCount) ? bookList[chooseSelection] : "";
  if (name.length() > 34) name = name.substring(0, 31) + "...";
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

  display->drawString(MENU_LEFT_MARGIN, 2, "Connect to Wi-Fi:");
  String ssidLine = "SSID: ";
  ssidLine += AP_NAME;
  String passLine = "Pass: ";
  passLine += AP_PASS;
  display->drawString(MENU_LEFT_MARGIN, 2 + MENU_LINE_HEIGHT, ssidLine);
  display->drawString(MENU_LEFT_MARGIN, 2 + 2 * MENU_LINE_HEIGHT, passLine);
  display->drawString(MENU_LEFT_MARGIN, 2 + 3 * MENU_LINE_HEIGHT, "Then open:");
  display->drawString(MENU_LEFT_MARGIN, 2 + 4 * MENU_LINE_HEIGHT, "192.168.4.1");
  display->drawString(MENU_LEFT_MARGIN, 2 + 6 * MENU_LINE_HEIGHT, "(hold Right to go back)");

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
      currentScreen = SCREEN_CHOOSE_BOOK;
      renderChooseBook();
      break;
    case 2:  // Connect to Wi-Fi
      enterWifiInfo();
      break;
  }
}

// ---------------- BUTTONS + SLEEP ----------------

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

  if (nowLeft && !lastLeft) { leftPressStart = millis(); leftResolved = false; }
  if (nowRight && !lastRight) { rightPressStart = millis(); rightResolved = false; }
  if (nowLeft && nowRight && !(lastLeft && lastRight)) { bothPressStart = millis(); bothResolved = false; }

  ButtonGesture gesture = GESTURE_NONE;

  if (nowLeft && nowRight) {
    if (!bothResolved && millis() - bothPressStart >= HOLD_THRESHOLD_MS) {
      gesture = GESTURE_BOTH_HOLD;
      bothResolved = true;
      leftResolved = true;
      rightResolved = true;
    }
  } else {
    if (nowLeft && !leftResolved && millis() - leftPressStart >= HOLD_THRESHOLD_MS) {
      gesture = GESTURE_LEFT_HOLD;
      leftResolved = true;
    }
    if (nowRight && !rightResolved && millis() - rightPressStart >= HOLD_THRESHOLD_MS) {
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

void dispatchGesture(ButtonGesture gesture) {
  switch (currentScreen) {
    case SCREEN_READING:
      switch (gesture) {
        case GESTURE_LEFT_TAP: previousPage(); break;
        case GESTURE_LEFT_HOLD: previousChapter(); break;
        case GESTURE_RIGHT_TAP: nextPage(); break;
        case GESTURE_RIGHT_HOLD: nextChapter(); break;
        case GESTURE_BOTH_HOLD: enterHome(); break;
        default: break;
      }
      break;

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
            renderChooseBook();
            break;
          case GESTURE_RIGHT_TAP:
            chooseSelection = (chooseSelection + 1) % bookCount;
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
  }
}

void handleButtons() {
  ButtonGesture gesture = pollButtons();
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

  // One-shot: if this boot was caused by physically pressing the Reset
  // button (not a normal power-on), land on the Home menu instead of
  // resuming the last book -- a hard "start over" escape hatch, since
  // Reset can't be read as a normal button during a running session (see
  // file header). ESP_RST_EXT is the specific reason code for an external
  // reset pin toggle; whether this board's reset circuit actually reports
  // that (vs. e.g. ESP_RST_POWERON) wasn't something I could verify without
  // hardware -- if it turns out wrong, this simply never fires rather than
  // firing on every normal power-on, which is the safer failure direction.
  bool bootedFromResetButton = (esp_reset_reason() == ESP_RST_EXT);

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

  uint32_t saved = currentBookName.length() > 0 ? loadSavedPosition(currentBookName) : 0;
  reopenBookAt(saved);
  indexChapters();

  if (bootedFromResetButton && currentBookName.length() > 0) {
    enterHome();
  } else {
    currentScreen = SCREEN_READING;
    renderPageAt(saved);
  }
}

void loop() {
  server.handleClient();
  handleButtons();
  maybeSleep();
  delay(20);
}
