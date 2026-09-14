#include "At328Link.h"

#include <stdio.h>
#include <string.h>

#include "LinkLine.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "at328";

// 4096 is about 1.6 s of cover at the measured 2.5 kB/s of stream traffic
// (151 frames/s, mean line 16.7 bytes with ATS0).
//
// It is sized for OUR stalls, not for bus bursts -- chiefly an NVS commit,
// which disables the flash cache for tens of milliseconds. And it is what
// keeps backpressure off the gateway: the 328 has no frame queue at all and
// its Serial.write() busy-waits, so if we stop draining, its main loop stalls,
// the MCP2515's two receive buffers overflow, and frames are lost on the car
// side where no counter will ever show it.
#define AT328_RX_BUF 4096

namespace {

int hexVal(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  return -1;
}

bool sameText(const char* line, const char* want) {
  return strcmp(line, want) == 0;
}

}  // namespace

At328Link::At328Link(uart_port_t port, int txPin, int rxPin, uint32_t baud)
    : port_(port),
      txPin_(txPin),
      rxPin_(rxPin),
      baud_(baud),
      reqId_(0u),
      respId_(0u),
      targetValid_(false),
      lastRxMs_(0u),
      ready_(false),
      monitoring_(false),
      streaming_(false),
      sink_(nullptr),
      frameQ_(nullptr),
      replyQ_(nullptr),
      reader_(nullptr) {}

uint32_t At328Link::nowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void At328Link::writeLine(const char* s) {
  uart_write_bytes(port_, s, strlen(s));
  uart_write_bytes(port_, "\r", 1);
}

void At328Link::drain() {
  uart_flush_input(port_);
}

// Reads until CR, or returns immediately when the '>' prompt appears. The
// prompt is not CR-terminated, so it has to be recognised as a terminator in
// its own right or every transaction would end in a timeout.
LinkStatus At328Link::readLine(char* out, uint16_t cap, uint16_t& len,
                               bool& promptSeen, uint32_t deadlineMs) {
  len = 0u;
  promptSeen = false;
  if (out == nullptr || cap == 0u) {
    return LINK_BAD_PARAM;
  }
  out[0] = '\0';

  for (;;) {
    const uint32_t now = nowMs();
    if (static_cast<int32_t>(deadlineMs - now) <= 0) {
      return LINK_NO_DATA;
    }
    const uint32_t remaining = deadlineMs - now;

    uint8_t c = 0u;
    const int got = uart_read_bytes(port_, &c, 1,
                                    pdMS_TO_TICKS(remaining ? remaining : 1u));
    if (got != 1) {
      return LINK_NO_DATA;   // deadline reached with nothing more to read
    }

    if (c == '>') {
      promptSeen = true;
      out[len] = '\0';
      return LINK_OK;
    }
    if (c == '\r' || c == '\n') {
      if (len == 0u) {
        continue;            // blank line: the adapter's own line endings
      }
      out[len] = '\0';
      return LINK_OK;
    }
    if (c < 0x20u || c > 0x7Eu) {
      continue;              // ignore stray control bytes rather than fail
    }
    if (len + 1u >= cap) {
      // A line longer than anything the protocol defines: discard it rather
      // than return a truncated frame that would decode into nonsense.
      len = 0u;
      out[0] = '\0';
      continue;
    }
    out[len++] = static_cast<char>(c);
  }
}

// ---------------------------------------------------------------------------
// The reader task.
//
// Exactly one task ever reads the UART. It classifies each line and routes it:
// a frame carrying the id we are currently waiting on goes to frameQ_ for
// whoever called readFrame(); every other frame is a broadcast and goes
// straight to the sink; anything that is not a frame is a command reply and
// goes to replyQ_.
//
// That split is what makes requests and broadcasts able to overlap. With the
// 2.1 adapter's AT@SM stream the two genuinely arrive interleaved, and the old
// alternation -- listen for a window, go deaf, spend a few requests -- is gone.
// ---------------------------------------------------------------------------

void At328Link::readerTrampoline(void* self) {
  static_cast<At328Link*>(self)->readerLoop();
}

void At328Link::readerLoop() {
  char line[AT328_LINE_MAX];

  for (;;) {
    uint16_t len = 0u;
    bool prompt = false;

    // A one-second deadline rather than an infinite block: readLine() is
    // written around a deadline, and coming back round an idle loop costs
    // nothing.
    // ONLY on LINK_OK. A deadline reached mid-line leaves a partial in the
    // buffer, and classifying that would turn one truncated frame into a
    // plausible-looking shorter one.
    if (readLine(line, sizeof(line), len, prompt, nowMs() + 1000u) != LINK_OK) {
      continue;
    }

    // Stamped for every complete line, whatever it turns out to be. An adapter
    // that is answering at all is an adapter that is still there.
    lastRxMs_ = nowMs();

    if (len != 0u) {
      if (linkClassifyLine(line, len) == LINK_LINE_FRAME) {
        CanFrame f;
        if (parseFrameLine(line, len, f)) {
          // targetValid_ is false whenever no request is addressed, so in the
          // ATMA fallback every frame reaches the sink exactly as before.
          // Broadcast ids (0x000..0x210) never collide with a response id
          // (0x500..0x516), so nothing is ever misrouted.
          if (targetValid_ && f.id == respId_) {
            // Never block. A response nobody is waiting for must not stall the
            // reader, because every broadcast behind it would stall too.
            (void)xQueueSend(frameQ_, &f, 0);
          } else if (sink_ != nullptr) {
            sink_(f, nowMs());
          }
        }
      } else {
        At328Reply r;
        strncpy(r.text, line, sizeof(r.text) - 1u);
        r.text[sizeof(r.text) - 1u] = '\0';
        r.prompt = false;
        (void)xQueueSend(replyQ_, &r, 0);
      }
    }

    if (prompt) {
      At328Reply r;
      r.text[0] = '\0';
      r.prompt = true;
      (void)xQueueSend(replyQ_, &r, 0);
    }
  }
}

LinkStatus At328Link::command(const char* cmd, uint32_t timeoutMs) {
  if (cmd == nullptr) {
    return LINK_BAD_PARAM;
  }
  if (replyQ_ == nullptr) {
    return LINK_NOT_READY;
  }

  // Was a uart_flush_input(): with the reader task owning the port, stale
  // lines are discarded here instead. Frames are unaffected -- they never
  // reach this queue.
  xQueueReset(replyQ_);
  writeLine(cmd);

  const uint32_t deadline = nowMs() + timeoutMs;

  // Bounded: a wedged adapter must not spin here even if it keeps emitting.
  // Raised from 8 to 16 because AT@EV event lines can now arrive in between.
  for (uint8_t i = 0u; i < 16u; ++i) {
    const uint32_t now = nowMs();
    if (static_cast<int32_t>(deadline - now) <= 0) {
      return LINK_TIMEOUT;
    }
    At328Reply r;
    if (xQueueReceive(replyQ_, &r, pdMS_TO_TICKS(deadline - now)) != pdTRUE) {
      return LINK_TIMEOUT;
    }
    if (r.prompt) {
      // Prompt with no OK seen. ATZ answers with its identity and then a
      // prompt, so treat reaching the prompt as success for those commands.
      return LINK_OK;
    }
    if (sameText(r.text, "OK")) {
      return LINK_OK;
    }
    if (sameText(r.text, "?")) {
      ESP_LOGW(TAG, "adapter rejected: %s", cmd);
      return LINK_PROTOCOL;
    }
    if (sameText(r.text, "CAN ERROR")) {
      return LINK_CAN_ERROR;
    }
    if (sameText(r.text, "BUFFER FULL")) {
      return LINK_OVERFLOW;
    }
    // Anything else (an identity string, a status line) is informational.
  }
  return LINK_PROTOCOL;
}

LinkStatus At328Link::begin() {
  ready_ = false;
  targetValid_ = false;
  monitoring_ = false;
  // Cleared here, not just set below: a retry after a failed begin() must not
  // inherit the previous adapter's answer. Swapping a 2.1 board for a 2.0 one
  // would otherwise leave us streaming against a device that never started.
  streaming_ = false;

  uart_config_t cfg = {};
  cfg.baud_rate = static_cast<int>(baud_);
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;

  if (!uart_is_driver_installed(port_)) {
    if (uart_driver_install(port_, AT328_RX_BUF, 0, 0, nullptr, 0) != ESP_OK) {
      ESP_LOGE(TAG, "uart_driver_install failed");
      return LINK_NOT_READY;
    }
  }
  if (uart_param_config(port_, &cfg) != ESP_OK ||
      uart_set_pin(port_, txPin_, rxPin_, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE) != ESP_OK) {
    ESP_LOGE(TAG, "uart config failed");
    return LINK_NOT_READY;
  }

  // The 328 is not reset by us opening the port -- it is flashed over ICSP and
  // has no bootloader on these pins -- so it may be mid-line from a previous
  // session. A bare CR closes whatever was half-typed.
  //
  // It may also still be STREAMING from before an ESP32 reset, in which case
  // frames are arriving right now. That is harmless: the reader routes them to
  // the sink, and command() waits on a queue frames never reach.
  writeLine("");
  vTaskDelay(pdMS_TO_TICKS(50));

  if (reader_ == nullptr) {
    // Discard the ROM bootloader noise BEFORE the reader starts, so it never
    // sees it. On a retry the reader is already draining and this is skipped.
    drain();

    frameQ_ = xQueueCreate(AT328_FRAME_Q_DEPTH, sizeof(CanFrame));
    replyQ_ = xQueueCreate(AT328_REPLY_Q_DEPTH, sizeof(At328Reply));
    if (frameQ_ == nullptr || replyQ_ == nullptr) {
      ESP_LOGE(TAG, "queue allocation failed");
      return LINK_NOT_READY;
    }
    // Core 0 beside carTask, one priority above it: the reader must win, or a
    // poll waiting on a response could hold off the frames that answer it.
    if (xTaskCreatePinnedToCore(readerTrampoline, "at328rx", 3072, this, 6,
                                &reader_, 0) != pdPASS) {
      reader_ = nullptr;
      ESP_LOGE(TAG, "reader task creation failed");
      return LINK_NOT_READY;
    }
  } else {
    xQueueReset(replyQ_);
    xQueueReset(frameQ_);
  }

  // ATZ resets the AT layer only; it deliberately does not re-initialise the
  // adapter's CAN controller, so this cannot disturb the car's VES announce.
  if (command("ATZ", AT328_RESET_TIMEOUT_MS) != LINK_OK) {
    ESP_LOGE(TAG, "no response to ATZ -- check wiring, 5V/3.3V level shift, baud");
    return LINK_NOT_READY;
  }

  struct { const char* cmd; const char* what; } kInit[] = {
      {"ATE0",   "echo off"},
      {"ATL0",   "no linefeeds"},
      {"ATS0",   "no spaces"},
      {"ATH1",   "headers on"},
      {"ATCAF0", "no auto formatting"},
      {"ATST32", "200 ms response timeout"},   // ELM units of 4 ms, 0x32 = 50
  };
  for (const auto& step : kInit) {
    const LinkStatus st = command(step.cmd, AT328_CMD_TIMEOUT_MS);
    if (st != LINK_OK) {
      ESP_LOGE(TAG, "init step failed: %s (%s)", step.cmd, step.what);
      return st;
    }
  }

  ready_ = true;

  // Ask for the background frame stream. A 2.0 adapter has never heard of
  // AT@SM and answers '?', which is NOT an error -- it means fall back to the
  // ATMA alternation. Probing rather than assuming is what lets the two
  // firmwares be flashed on separate trips to the radio.
  streaming_ = (command("AT@SM1", AT328_CMD_TIMEOUT_MS) == LINK_OK);

  ESP_LOGI(TAG, "adapter ready on UART%d tx=%d rx=%d @%u",
           static_cast<int>(port_), txPin_, rxPin_,
           static_cast<unsigned>(baud_));
  ESP_LOGI(TAG, "%s", streaming_
      ? "AT@SM1 accepted: streaming, requests and broadcasts overlap"
      : "no AT@SM (adapter older than 2.1): falling back to ATMA windows");
  return LINK_OK;
}

LinkStatus At328Link::setTarget(uint16_t requestId, uint16_t responseId) {
  if (requestId > 0x7FFu || responseId > 0x7FFu) {
    return LINK_BAD_PARAM;
  }
  if (!ready_) {
    return LINK_NOT_READY;
  }
  // Skip the round trip when nothing changed: two commands per poll on an
  // 83.3 kbps bus is worth avoiding.
  if (targetValid_ && requestId == reqId_ && responseId == respId_) {
    return LINK_OK;
  }

  char cmd[16];
  snprintf(cmd, sizeof(cmd), "ATSH%03X", requestId);
  LinkStatus st = command(cmd, AT328_CMD_TIMEOUT_MS);
  if (st != LINK_OK) {
    return st;
  }
  snprintf(cmd, sizeof(cmd), "ATCRA%03X", responseId);
  st = command(cmd, AT328_CMD_TIMEOUT_MS);
  if (st != LINK_OK) {
    return st;
  }

  reqId_ = requestId;
  respId_ = responseId;
  targetValid_ = true;
  return LINK_OK;
}

// The frame's own id is ignored: the adapter transmits with whatever ATSH set,
// which is what setTarget() configured. Only the payload travels.
LinkStatus At328Link::sendFrame(const CanFrame& frame) {
  if (frame.len == 0u || frame.len > CAN_MAX_DLC) {
    return LINK_BAD_PARAM;
  }
  if (!ready_) {
    return LINK_NOT_READY;
  }
  if (!targetValid_) {
    return LINK_BAD_PARAM;   // setTarget() first, or the adapter's stale
                             // header would address the wrong module
  }

  char line[CAN_MAX_DLC * 2u + 1u];
  uint8_t n = 0u;
  for (uint8_t i = 0u; i < frame.len; ++i) {
    static const char kHex[] = "0123456789ABCDEF";
    line[n++] = kHex[(frame.data[i] >> 4) & 0x0Fu];
    line[n++] = kHex[frame.data[i] & 0x0Fu];
  }
  line[n] = '\0';

  // Discard any late response left over from a transaction that already timed
  // out, so it cannot be mis-attributed to this one. This replaces the
  // uart_flush_input() that used to do the job -- flushing the port would now
  // throw away broadcasts too, and the reader may already have parsed them.
  xQueueReset(frameQ_);

  writeLine(line);
  return LINK_OK;
}

// With ATH1 + ATS0 a frame line is exactly three id nibbles followed by an
// even number of data nibbles, no separators.
bool At328Link::parseFrameLine(const char* line, uint16_t len, CanFrame& out) {
  if (line == nullptr || len < 3u) {
    return false;
  }
  const uint16_t dataChars = static_cast<uint16_t>(len - 3u);
  if ((dataChars & 1u) != 0u || (dataChars / 2u) > CAN_MAX_DLC) {
    return false;
  }

  uint16_t id = 0u;
  for (uint8_t i = 0u; i < 3u; ++i) {
    const int v = hexVal(line[i]);
    if (v < 0) {
      return false;
    }
    id = static_cast<uint16_t>((id << 4) | static_cast<uint16_t>(v));
  }

  out.id = id;
  out.len = static_cast<uint8_t>(dataChars / 2u);
  for (uint8_t i = 0u; i < out.len; ++i) {
    const int hi = hexVal(line[3u + i * 2u]);
    const int lo = hexVal(line[4u + i * 2u]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    out.data[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  for (uint8_t i = out.len; i < CAN_MAX_DLC; ++i) {
    out.data[i] = 0u;
  }
  return true;
}

LinkStatus At328Link::readFrame(CanFrame& out, uint32_t timeoutMs) {
  if (!ready_ || frameQ_ == nullptr) {
    return LINK_NOT_READY;
  }
  // The reader task has already parsed the line and decided this frame carries
  // the id we asked about, so there is nothing left to do but wait for it.
  //
  // CAN ERROR and BUFFER FULL are no longer seen here -- they are command
  // replies and go to replyQ_. A transaction that hits one now times out
  // instead of being told why, which is a fair trade for never having a
  // broadcast and a response compete for the same read.
  if (xQueueReceive(frameQ_, &out, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
    return LINK_OK;
  }
  return LINK_NO_DATA;
}

// ATMA streams every frame until ANY character reaches the adapter. The filter
// has to be cleared first or we would only be shown whatever the last request
// set -- ATCRA with no argument does that.
LinkStatus At328Link::monitorBegin() {
  if (!ready_) {
    return LINK_NOT_READY;
  }
  if (streaming_) {
    // Nothing to begin: with AT@SM the frames are already arriving, and ATMA
    // would only make the adapter deaf to the requests we still want to send.
    return LINK_OK;
  }
  if (monitoring_) {
    return LINK_OK;
  }
  drain();
  if (command("ATCRA", AT328_CMD_TIMEOUT_MS) != LINK_OK) {
    return LINK_PROTOCOL;
  }
  // No reply to wait for: the adapter answers this by streaming.
  writeLine("ATMA");
  monitoring_ = true;
  // The filter is gone, so the next request must reprogram its addressing.
  // It also means the reader sees no id matching respId_, so every frame in
  // the window goes to the sink -- which is exactly what the window is for.
  targetValid_ = false;
  return LINK_OK;
}

LinkStatus At328Link::monitorEnd() {
  if (streaming_ || !monitoring_) {
    return LINK_OK;
  }
  // Any character stops the stream; the adapter consumes it rather than
  // executing it, so a bare CR is the cheapest way to say stop.
  writeLine("");
  monitoring_ = false;

  // At 83 kbps there is a long tail of already-transmitted frames in flight
  // after the stop character. Left in the buffer, the next command reads them
  // as its own reply and the adapter answers "?" -- that is what produced
  // "adapter rejected: ATSH600" on the first request after every window.
  //
  // This used to drain the port here until it went quiet. It cannot any more:
  // the reader task owns the port and we would be racing it for bytes. So give
  // the reader time to absorb the tail into the sink where it belongs, then
  // discard whatever it classified as replies. 120 ms is generous -- the 328's
  // TX ring is 128 bytes, about 11 ms of backlog at 115200.
  vTaskDelay(pdMS_TO_TICKS(120));
  xQueueReset(replyQ_);
  return LINK_OK;
}

LinkStatus At328Link::status(char* out, uint16_t cap) {
  if (out == nullptr || cap == 0u) {
    return LINK_BAD_PARAM;
  }
  out[0] = '\0';
  if (!ready_ || replyQ_ == nullptr) {
    return LINK_NOT_READY;
  }

  xQueueReset(replyQ_);
  writeLine("AT@ST");
  const uint32_t deadline = nowMs() + AT328_CMD_TIMEOUT_MS;

  for (uint8_t i = 0u; i < 16u; ++i) {
    const uint32_t now = nowMs();
    if (static_cast<int32_t>(deadline - now) <= 0) {
      return LINK_TIMEOUT;
    }
    At328Reply r;
    if (xQueueReceive(replyQ_, &r, pdMS_TO_TICKS(deadline - now)) != pdTRUE) {
      return LINK_TIMEOUT;
    }
    if (r.prompt) {
      return out[0] != '\0' ? LINK_OK : LINK_NO_DATA;
    }
    if (sameText(r.text, "?")) {
      return LINK_PROTOCOL;
    }
    if (r.text[0] != '\0' && out[0] == '\0') {
      strncpy(out, r.text, cap - 1u);
      out[cap - 1u] = '\0';
    }
  }
  return LINK_PROTOCOL;
}
