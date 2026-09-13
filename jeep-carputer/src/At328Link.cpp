#include "At328Link.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "at328";

// Generous enough that a burst of monitor traffic cannot overrun us between
// reads, small enough to be irrelevant on a part with this much RAM.
#define AT328_RX_BUF 1024

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
      ready_(false) {}

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

LinkStatus At328Link::command(const char* cmd, uint32_t timeoutMs) {
  if (cmd == nullptr) {
    return LINK_BAD_PARAM;
  }
  writeLine(cmd);

  const uint32_t deadline = nowMs() + timeoutMs;
  char line[AT328_LINE_MAX];
  uint16_t len = 0u;
  bool prompt = false;

  // Bounded: a wedged adapter must not spin here even if it keeps emitting.
  for (uint8_t i = 0u; i < 8u; ++i) {
    const LinkStatus st = readLine(line, sizeof(line), len, prompt, deadline);
    if (st != LINK_OK) {
      return LINK_TIMEOUT;
    }
    if (prompt) {
      // Prompt with no OK seen. ATZ answers with its identity and then a
      // prompt, so treat reaching the prompt as success for those commands.
      return LINK_OK;
    }
    if (sameText(line, "OK")) {
      return LINK_OK;
    }
    if (sameText(line, "?")) {
      ESP_LOGW(TAG, "adapter rejected: %s", cmd);
      return LINK_PROTOCOL;
    }
    if (sameText(line, "CAN ERROR")) {
      return LINK_CAN_ERROR;
    }
    if (sameText(line, "BUFFER FULL")) {
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
  writeLine("");
  vTaskDelay(pdMS_TO_TICKS(50));
  drain();

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
  ESP_LOGI(TAG, "adapter ready on UART%d tx=%d rx=%d @%u",
           static_cast<int>(port_), txPin_, rxPin_,
           static_cast<unsigned>(baud_));
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

  drain();          // discard anything left from a previous transaction
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
  if (!ready_) {
    return LINK_NOT_READY;
  }
  const uint32_t deadline = nowMs() + timeoutMs;
  char line[AT328_LINE_MAX];
  uint16_t len = 0u;
  bool prompt = false;

  // Bounded: an adapter in ATMA mode would otherwise stream forever.
  for (uint8_t i = 0u; i < 32u; ++i) {
    const LinkStatus st = readLine(line, sizeof(line), len, prompt, deadline);
    if (st != LINK_OK) {
      return LINK_NO_DATA;
    }
    if (prompt) {
      return LINK_NO_DATA;   // the adapter is done with this transaction
    }
    if (sameText(line, "NO DATA")) {
      return LINK_NO_DATA;
    }
    if (sameText(line, "CAN ERROR")) {
      return LINK_CAN_ERROR;
    }
    if (sameText(line, "BUFFER FULL")) {
      return LINK_OVERFLOW;
    }
    if (sameText(line, "OK") || sameText(line, "?")) {
      continue;              // leftover reply from a previous command
    }
    if (parseFrameLine(line, len, out)) {
      return LINK_OK;
    }
    ESP_LOGD(TAG, "unparsed line: '%s'", line);
  }
  return LINK_PROTOCOL;
}

// ATMA streams every frame until ANY character reaches the adapter. The filter
// has to be cleared first or we would only be shown whatever the last request
// set -- ATCRA with no argument does that.
LinkStatus At328Link::monitorBegin() {
  if (!ready_) {
    return LINK_NOT_READY;
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
  targetValid_ = false;
  return LINK_OK;
}

LinkStatus At328Link::monitorEnd() {
  if (!monitoring_) {
    return LINK_OK;
  }
  // Any character stops the stream; the adapter consumes it rather than
  // executing it, so a bare CR is the cheapest way to say stop.
  writeLine("");
  monitoring_ = false;

  // Drain until the line goes QUIET, not merely until a prompt appears.
  //
  // At 83 kbps there can be a long tail of already-transmitted frames in
  // flight, and reading a fixed number of lines leaves the rest of them sitting
  // in the buffer -- where the next command reads them as its own reply and
  // the adapter answers "?". That is what produced "adapter rejected: ATSH600"
  // on the first request after every monitor window.
  const uint32_t giveUp = nowMs() + 400u;
  uint32_t lastByte = nowMs();
  uint8_t byte = 0u;
  while (nowMs() < giveUp) {
    int n = uart_read_bytes(port_, &byte, 1, pdMS_TO_TICKS(10));
    if (n > 0) {
      lastByte = nowMs();
      continue;
    }
    if ((nowMs() - lastByte) >= 40u) {
      break;              // 40 ms of silence: the stream has genuinely stopped
    }
  }
  drain();
  return LINK_OK;
}

LinkStatus At328Link::status(char* out, uint16_t cap) {
  if (out == nullptr || cap == 0u) {
    return LINK_BAD_PARAM;
  }
  out[0] = '\0';
  if (!ready_) {
    return LINK_NOT_READY;
  }

  writeLine("AT@ST");
  const uint32_t deadline = nowMs() + AT328_CMD_TIMEOUT_MS;
  char line[AT328_LINE_MAX];
  uint16_t len = 0u;
  bool prompt = false;

  for (uint8_t i = 0u; i < 8u; ++i) {
    if (readLine(line, sizeof(line), len, prompt, deadline) != LINK_OK) {
      return LINK_TIMEOUT;
    }
    if (prompt) {
      return out[0] != '\0' ? LINK_OK : LINK_NO_DATA;
    }
    if (sameText(line, "?")) {
      return LINK_PROTOCOL;
    }
    if (len != 0u && out[0] == '\0') {
      strncpy(out, line, cap - 1u);
      out[cap - 1u] = '\0';
    }
  }
  return LINK_PROTOCOL;
}
