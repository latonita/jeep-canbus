#include "At328Link.h"

#include <stdio.h>
#include <string.h>

#include "LinkLine.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "at328";

// 4096 is about 1.2 s of cover at the 3.4 kB/s AT@SML2 costs, and the figure
// MOVED when the stream did: a stamped LAWICEL line is
// `t` + 3 id + 1 DLC + 2n data + 4 stamp + CR, six bytes more than the ELM
// line it replaces.
//
// Measured rather than assumed, over a 220673-frame capture (mean DLC 6.34):
//
//   AT@SM1  (was)  16.7 B/line -> 2.5 kB/s -> 22 % of the link, 1.6 s of cover
//   AT@SML2 (now)  22.7 B/line -> 3.4 kB/s -> 30 % of the link, 1.2 s of cover
//
// So the timestamps cost eight points of link headroom and a quarter of the
// buffer's cover, which is the price of the whole change and is worth stating
// in one place. 30 % still leaves the link far from saturated, and 1.2 s is
// still an eternity against the stalls this is sized for.
//
// It is sized for OUR stalls, not for bus bursts -- chiefly an NVS commit,
// which disables the flash cache for tens of milliseconds. And it is what
// keeps backpressure off the gateway: the 328 has no frame queue at all and
// its Serial.write() busy-waits, so if we stop draining, its main loop stalls,
// the MCP2515's two receive buffers overflow, and frames are lost on the car
// side where no counter will ever show it.
#define AT328_RX_BUF 4096

namespace {

bool sameText(const char* line, const char* want) {
  return strcmp(line, want) == 0;
}

// THE ONE PLACE a stream level and a timestamp width are related.
//
// The levels mirror each other across the two formats -- AT@SM<n> and AT@SML<n>
// carry the same widths for the same n -- so this is one function and not two,
// which is also what stops the ladder in begin() from having to remember which
// mapping goes with which rung.
//
// What the width is FOR differs sharply between the two, though, and that is
// the whole reason this host moved:
//
//   LAWICEL -- the width is CHECKED. Every line's own DLC nibble says how many
//              data bytes follow, so linkParseStreamLine() derives the width
//              from the line and hands it back; this value is only what we
//              ASKED for, and a disagreement is counted and logged.
//   ELM     -- the width is OBEYED, because it cannot be measured. It is the
//              only thing that says where the data stops and the time starts,
//              and being wrong is silent in both directions.
uint8_t stampCharsForLevel(uint8_t level) {
  switch (level) {
    case 2u:  return LINK_LINE_TS_SHORT;   // 4 digits, millis() % 60000
    case 3u:  return LINK_LINE_TS_LONG;    // 8 digits, raw millis()
    default:  return LINK_LINE_TS_NONE;    // 1 = unstamped stream, 0 = no stream
  }
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
      wire_(LINK_WIRE_NONE),
      streamLevel_(0u),
      stampChars_(LINK_LINE_TS_NONE),
      lastStampMs_(0u),
      haveStamp_(false),
      stampMismatch_(0u),
      // LAWICEL level 2 is the DEFAULT and the whole point: stamped frames, and
      // a wire whose stamp width the reader can verify rather than assume.
      wireReq_(LINK_WIRE_LAWICEL),
      streamLevelReq_(2u),
      ready_(false),
      monitoring_(false),
      streaming_(false),
      sink_(nullptr),
      frameQ_(nullptr),
      replyQ_(nullptr),
      reader_(nullptr) {
  streamClockInit(clock_, LINK_LINE_TS_NONE);
}

uint32_t At328Link::nowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void At328Link::writeLine(const char* s) {
  uart_write_bytes(port_, s, strlen(s));
  uart_write_bytes(port_, "\r", 1);
}

void At328Link::drain() {
  // uart_flush_input() ONLY while nothing else owns the port.
  //
  // THIS HUNG THE LINK DEAD, and the failure was completely silent. Once the
  // reader task exists it sits inside uart_read_bytes(), which HOLDS the
  // driver's rx mutex for the whole time it waits -- and uart_flush_input()
  // wants that same mutex. The reader runs one priority ABOVE carTask on the
  // SAME core, so it re-takes the mutex the instant it is released and the
  // flush never wins it. carTask stops for good, inside begin(), having sent
  // three bytes -- so there is no "no response to ATZ", no "link not ready,
  // retrying", nothing on the console at all. The panel comes up and shows
  // dashes for ever, which is indistinguishable from an unplugged gateway.
  //
  // command() already learned this and discards stale lines by resetting its
  // queue instead ("Was a uart_flush_input()" there). Two call sites did not:
  // the one in begin() after streamStop(), which is on EVERY bring-up, and the
  // one in monitorBegin(), which is only on the ATMA rung and so had never been
  // reached. The rule lives HERE now, where it cannot be missed a third time.
  //
  // Draining the QUEUES is the honest equivalent once the reader owns the
  // port: what a caller wants discarded is lines from before whatever it just
  // did, and after the reader starts those are lines, not bytes.
  if (reader_ == nullptr) {
    uart_flush_input(port_);
    return;
  }
  if (replyQ_ != nullptr) {
    xQueueReset(replyQ_);
  }
  if (frameQ_ != nullptr) {
    xQueueReset(frameQ_);
  }
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
// Exactly one task ever reads the UART. It classifies each line and routes it
// three ways, and under AT@SML2 the three are told apart by the SHAPE of the
// line rather than by arithmetic on it:
//
//   a LAWICEL line ('t' ...) is a background STREAM frame. It goes to the sink,
//     always, with its stamp mapped onto our clock. It is never a response --
//     the gateway answers requests in the ELM shape, so a stream line cannot be
//     part of the transaction in flight.
//   an ELM-shaped line is a request RESPONSE. It goes to frameQ_ when it carries
//     the id we are waiting on. (On the AT@SM fallback and inside an ATMA window
//     this shape is every frame, which is why the id test is still here.)
//   anything else is a command reply and goes to replyQ_.
//
// That split is what makes requests and broadcasts able to overlap. With the
// 2.1 adapter's stream the two genuinely arrive interleaved, and the old
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
      // ONE read of the ELM stamp width per line, shared by the classifier and
      // that parser. Read it twice and a begin() landing in between would let
      // them split the same characters differently -- 19 chars accepted as 8
      // unstamped data bytes and then decoded as 6 bytes plus a stamp, or the
      // reverse -- and both halves would look like they had done their job.
      //
      // Under LAWICEL this is ZERO whatever the stream level is, because the
      // only ELM-shaped lines left are request responses and the gateway never
      // stamps those (AtServer.h, "which lines get which format"). The LAWICEL
      // branch measures its own width and ignores this entirely.
      const bool lawicel = (wire_ == LINK_WIRE_LAWICEL);
      const uint8_t wantTs = stampChars_;
      const uint8_t elmTs = lawicel ? LINK_LINE_TS_NONE : wantTs;

      const LinkLineKind kind = linkClassifyLine(line, len, elmTs);

      if (kind == LINK_LINE_STREAM) {
        // A LAWICEL line, so a BACKGROUND STREAM frame -- and never a response,
        // which is worth stating because the old code had to guess from the id.
        // The gateway routes a frame matching the active filter to the ELM
        // response path while a request is collecting, and streams it otherwise,
        // so a stamped LAWICEL line is by construction not part of the
        // transaction in flight. Sending one to frameQ_ would mis-attribute a
        // late or unfiltered frame to a request that did not ask for it.
        CanFrame f;
        uint32_t stamp = 0u;
        uint8_t got = 0u;
        if (linkParseStreamLine(line, len, f, stamp, got)) {
          uint32_t t = nowMs();
          if (got != LINK_LINE_TS_NONE) {
            lastStampMs_ = stamp;
            haveStamp_ = true;
            // MAPPED, not merely latched. This is the change the format switch
            // was for: the sink gets the time the GATEWAY saw the frame, put on
            // our timebase by core/StreamClock, so a fuel integral and an
            // acceleration split are measured on the car's intervals instead of
            // on whenever this task got round to the line. See StreamClock.h for
            // why the mapping is a minimum-filtered offset and why that is what
            // keeps `nowMs() - t` a real age.
            t = streamClockMap(clock_, stamp, t);
          }
          if (got != wantTs) {
            // The adapter appended a width we did not order. The frame above is
            // still decoded CORRECTLY -- that is what a self-describing wire
            // buys -- so this is a complaint, not a drop. On the ELM shape the
            // same disagreement was unobservable and showed up as two data bytes
            // read as a time.
            // Read-modify-write rather than `++`, which C++20 deprecates on a
            // volatile. The reader task is the only writer, so this needs no
            // atomicity beyond the single word it already has.
            stampMismatch_ = stampMismatch_ + 1u;
          }
          if (sink_ != nullptr) {
            sink_(f, t);
          }
        }
      } else if (kind == LINK_LINE_FRAME) {
        // The ELM shape. Under LAWICEL that is a request RESPONSE and nothing
        // else; on the AT@SM fallback and in an ATMA window it is every frame,
        // which is why the id test below stays exactly as it was.
        CanFrame f;
        uint32_t stamp = 0u;
        if (linkParseFrameLine(line, len, elmTs, f, stamp)) {
          if (elmTs != LINK_LINE_TS_NONE) {
            lastStampMs_ = stamp;
            haveStamp_ = true;
          }
          // targetValid_ is false whenever no request is addressed, so in the
          // ATMA fallback every frame reaches the sink exactly as before.
          // Broadcast ids (0x000..0x210) never collide with a response id
          // (0x500..0x516), so nothing is ever misrouted.
          if (targetValid_ && f.id == respId_) {
            // Never block. A response nobody is waiting for must not stall the
            // reader, because every broadcast behind it would stall too.
            (void)xQueueSend(frameQ_, &f, 0);
          } else if (sink_ != nullptr) {
            // The HOST clock here, and honestly so: an ELM line carries no
            // stamp on the LAWICEL path, and on the AT@SM path a stamp is not
            // measurable so mapping one would be building a clock on a width
            // nobody could check.
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
  //
  // The level and the width it implies go with it, and for a sharper version of
  // the same reason: a width left over from a session that is gone would have
  // the reader stripping four characters off lines nothing is appending to.
  streaming_ = false;
  wire_ = LINK_WIRE_NONE;
  streamLevel_ = 0u;
  stampChars_ = LINK_LINE_TS_NONE;
  // The clock goes with them, and for a sharper version of the same reason: it
  // holds an offset between OUR clock and a particular gateway's, and a
  // reconnect may be to a board that has been power-cycled since. A stale
  // offset would map every frame to a time minutes away from now, which every
  // staleness test downstream would read as "no data" -- a dashboard of dashes
  // on a healthy link.
  streamClockInit(clock_, LINK_LINE_TS_NONE);
  haveStamp_ = false;
  stampMismatch_ = 0u;

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

  // BREADCRUMBS, at DEBUG so they cost nothing at the default INFO level.
  //
  // They stay because the rx-mutex deadlock in drain() was invisible for want
  // of exactly these five lines: begin() is loud at both its EXITS and silent
  // in between, so a hang anywhere inside it looked identical to a gateway that
  // was never plugged in. Raise the level for this tag to get them back.
  ESP_LOGD(TAG, "bringup: uart configured");

  // The 328 is not reset by us opening the port -- it is flashed over ICSP and
  // has no bootloader on these pins -- so it may be mid-line from a previous
  // session. A bare CR closes whatever was half-typed.
  writeLine("");
  ESP_LOGD(TAG, "bringup: CR written");
  vTaskDelay(pdMS_TO_TICKS(50));

  if (reader_ == nullptr) {
    // Discard the ROM bootloader noise BEFORE the reader starts, so it never
    // sees it. On a retry the reader is already draining and this is skipped.
    drain();
    ESP_LOGD(TAG, "bringup: drained");

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

  // SILENCE THE STREAM BEFORE CONFIGURING ANYTHING.
  //
  // It may still be streaming from before an ESP32 reset, in which case frames
  // are arriving right now -- and the comment this replaces called that
  // harmless, which it was only while both ends agreed on one shape. They no
  // longer do: the format in force is now something this session NEGOTIATES, so
  // an adapter left in AT@SM2 by an older build would have every line judged
  // against the LAWICEL branch, fail it, and be filed as a command reply. At
  // 151 frames/s that is replyQ_ -- eight slots -- overflowing continuously,
  // and command()'s sixteen-line budget spent on frame lines before the answer
  // to ATZ can reach it. A bring-up that fails on a working link.
  //
  // Through streamStop() rather than beside it, so there is ONE place that
  // knows which commands stop a stream. It sends both levels -- they are
  // mutually exclusive in the gateway and we do not know which one a previous
  // session left running -- and ignores both answers: '?' from a 2.0 board means
  // there was no stream to stop, which is the state we were asking for.
  //
  // ATZ below would also clear the level, but only once its own reply has been
  // FOUND, and finding it is the thing this makes possible.
  ESP_LOGD(TAG, "bringup: reader up, stopping any stream");
  (void)streamStop();
  drain();
  ESP_LOGD(TAG, "bringup: stream stopped, sending ATZ");

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

  // ---- negotiate the stream: LAWICEL, then ELM, then windows ----
  //
  // Asked for rather than assumed, and in that order, because each rung is a
  // real adapter and the drop between them costs something specific. See
  // setStreamFormat()'s comment for the ladder; what is here is the I/O.
  //
  // No AT@ST parsing to discover what the adapter is doing: we are the side that
  // DECLARES it, and the silencing above plus ATZ plus this command re-declare
  // it on every bring-up. Interrogating would mean parsing a status line to
  // decide how to parse every line after it.
  char cmdBuf[12];
  const char* asked = "";

  if (wireReq_ == LINK_WIRE_LAWICEL) {
    snprintf(cmdBuf, sizeof(cmdBuf), "AT@SML%u",
             static_cast<unsigned>(streamLevelReq_));
    if (command(cmdBuf, AT328_CMD_TIMEOUT_MS) == LINK_OK) {
      wire_ = LINK_WIRE_LAWICEL;
      streamLevel_ = streamLevelReq_;
      asked = cmdBuf;
    } else {
      ESP_LOGW(TAG, "%s rejected: falling back to the ELM-shaped stream",
               cmdBuf);
    }
  }

  // The ELM rung, reached either because LAWICEL was refused or because a
  // caller asked for it. Level 1 only: the stamped ELM levels are deliberately
  // NOT tried, because their width cannot be checked against the line and a
  // timestamp we cannot verify is worth less than none -- that asymmetry is the
  // reason the LAWICEL rung is above this one at all.
  if (wire_ == LINK_WIRE_NONE) {
    if (command("AT@SM1", AT328_CMD_TIMEOUT_MS) == LINK_OK) {
      wire_ = LINK_WIRE_ELM;
      streamLevel_ = 1u;
      asked = "AT@SM1";
    }
  }

  streaming_ = (wire_ != LINK_WIRE_NONE);
  if (!streaming_) {
    streamLevel_ = 0u;
  }
  stampChars_ = stampCharsForLevel(streamLevel_);
  streamClockInit(clock_, stampChars_);

  ESP_LOGI(TAG, "adapter ready on UART%d tx=%d rx=%d @%u",
           static_cast<int>(port_), txPin_, rxPin_,
           static_cast<unsigned>(baud_));
  if (wire_ == LINK_WIRE_LAWICEL) {
    ESP_LOGI(TAG, "%s accepted: LAWICEL stream, requests and broadcasts "
                  "overlap, %u-char rolling timestamp mapped onto our clock",
             asked, static_cast<unsigned>(stampChars_));
  } else if (wire_ == LINK_WIRE_ELM) {
    ESP_LOGI(TAG, "%s accepted: ELM-shaped stream, requests and broadcasts "
                  "overlap, no timestamps -- broadcast times are host receive "
                  "times", asked);
  } else {
    ESP_LOGI(TAG, "no AT@SML and no AT@SM (adapter older than 2.1): falling "
                  "back to ATMA windows");
  }
  return LINK_OK;
}

// AT@SML0, or AT@SM0 when the ELM fallback is what is running.
//
// Sends the one that matches, then the other, and ignores both answers. The
// levels are mutually exclusive in the gateway, so the second command can only
// ever be a no-op -- but "which one is running" is exactly the thing that is
// unreliable when this is most needed (a bring-up against an adapter a previous
// session configured), and one redundant command is cheaper than a stream left
// running in a shape nobody is parsing.
LinkStatus At328Link::streamStop() {
  if (replyQ_ == nullptr) {
    return LINK_NOT_READY;
  }
  const bool lawicelFirst = (wire_ != LINK_WIRE_ELM);
  (void)command(lawicelFirst ? "AT@SML0" : "AT@SM0", AT328_CMD_TIMEOUT_MS);
  (void)command(lawicelFirst ? "AT@SM0" : "AT@SML0", AT328_CMD_TIMEOUT_MS);
  wire_ = LINK_WIRE_NONE;
  streamLevel_ = 0u;
  stampChars_ = LINK_LINE_TS_NONE;
  streaming_ = false;
  // The mapping belongs to a stream that is no longer running. Left standing it
  // would be re-used against whatever the next stream's epoch turns out to be.
  streamClockInit(clock_, LINK_LINE_TS_NONE);
  haveStamp_ = false;
  return LINK_OK;
}

// Levels 1..3 only. 0 is not offered here because it would turn the stream off
// by a route that looks like configuration, and the way to run without the
// stream is an adapter that refuses the command -- or streamStop(), which says
// what it does. Anything else is a caller's mistake, and on the ELM path it is
// one that would show up as a dead dashboard on a healthy link: the width would
// not match what the adapter appends, so every frame line would fail the
// classifier's length set and be filed as a command reply.
//
// LINK_WIRE_NONE is refused for the same reason as level 0. There is no "ask for
// no stream" here.
void At328Link::setStreamFormat(LinkWire w, uint8_t level) {
  if (w != LINK_WIRE_LAWICEL && w != LINK_WIRE_ELM) {
    ESP_LOGE(TAG, "stream wire %u is neither LAWICEL nor ELM -- ignored, "
                  "staying at %u/%u", static_cast<unsigned>(w),
             static_cast<unsigned>(wireReq_),
             static_cast<unsigned>(streamLevelReq_));
    return;
  }
  if (level < 1u || level > 3u) {
    ESP_LOGE(TAG, "stream level %u out of 1..3 -- ignored, staying at %u/%u",
             static_cast<unsigned>(level),
             static_cast<unsigned>(wireReq_),
             static_cast<unsigned>(streamLevelReq_));
    return;
  }
  wireReq_ = static_cast<uint8_t>(w);
  streamLevelReq_ = level;
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
