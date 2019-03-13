// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/sensor/button.hpp>

namespace nrfcxx {
namespace sensor {

namespace {

/** State value when we don't know the button state yet. */
constexpr uint8_t UNCONFIGURED = 0U;

/** State value when button is known to be released.
 *
 * Neither #HOLD_BIT nor #STUCK_BIT will be co-set. */
constexpr uint8_t RELEASED = 1U;

/** State value when button was last observed as pressed. */
constexpr uint8_t PRESSED = 2U;

/** State value when button was last observed as unpressed, but
 * release has not been confirmed. */
constexpr uint8_t UNPRESSED = 3U;

/** Bit set when in PRESSED or UNPRESSED state. */
constexpr uint8_t xPRESSED_bit = 0x02U;

/** Mask to isolate the primary state. */
constexpr uint8_t STATE_mask = 0x03U;

/** Bit set to indicate that an alarm was scheduled during the last
 * process. */
constexpr uint8_t ALARM_bit = 0x08U;

/** Bit set to indicate in a hold substate of xPRESSED. */
constexpr uint8_t HOLD_bit = 0x40U;

/** Bit set to indicate in a stuck substate of xPRESSED. */
constexpr uint8_t STUCK_bit = 0x80U;

/** The lower bound for setting the debounce, hold, or stuck
 * timeouts. */
constexpr unsigned int MINIMUM_utt = contact::utt_from_ts(1);

} // namespace

const char*
Button::eventstr (unsigned int evt)
{
  static const char* const str[] = {
    "release",
    "click",
    "hold",
    "stuck",
  };
  if (evt < (sizeof(str)/sizeof(*str))) {
    return str[evt];
  }
  return "?";
}

int
Button::debounce_utt (unsigned int utt)
{
  if (MINIMUM_utt < utt) {
    return -1;
  }
  debounce_utt_ = utt;
  return 0;
}

int
Button::hold_utt (unsigned int utt)
{
  if ((MINIMUM_utt > utt)
      || (utt > stuck_utt_)) {
    return -1;
  }
  hold_utt_ = utt;
  return 0;
}

int
Button::stuck_utt (unsigned int utt)
{
  if ((MINIMUM_utt > utt)
      || (utt < hold_utt_)) {
    return -1;
  }
  stuck_utt_ = utt;
  return 0;
}

Button::Button (uint8_t psel,
                const notifier_type& notify,
                event_callback callback,
                bool active_low) :
  notify_{notify},
  contact_{psel, [this](auto ordinal) {
      this->notify_();
    }, active_low ? gpio::PIN_CNF_ACTIVE_LOW : gpio::PIN_CNF_ACTIVE_HIGH},
  listener_{contact_.sense_listener()},
  callback_{callback},
  alarm_{alarm_callback, this}
{
  static bool initialized;
  using nrfcxx::sensor::contact;
  if (!initialized) {
    periph::GPIOTE::enable_sense();
    initialized = true;
  }

  /* Enable the listener, then synchronize the sense with the state.
   * This may trigger events when the mutex region exits. */
  listener_.enable();
  {
    periph::GPIOTE::mutex_type mutex;
    gpio::update_sense_bi(psel);
  }

  /* Capture the current state and use it to set the initial state.
   * If events are pending, this state will be updated by the
   * application in the usual way. */
  snapshot_ = contact_.snapshot();

  bool pressed = state_xor ^ contact::state_from_ordinal(snapshot_.ordinal);
  process_utt = contact::utt_from_ts(snapshot_.ordinal_ts());
  if (pressed) {
    /* If pressed on construction record it as being stuck. */
    state_ = PRESSED | STUCK_bit | HOLD_bit;
    press_utt = process_utt;
  } else {
    state_ = RELEASED;
  }
}

unsigned int
Button::state () const
{
  if (STUCK_bit & state_) {
    return EVT_STUCK;
  }
  if (HOLD_bit & state_) {
    return EVT_HOLD;
  }
  if (RELEASED == state_) {
    return EVT_RELEASE;
  }
  return EVT_CLICK;
}

unsigned int
Button::state (uint64_t& dur) const
{
  auto rv = state();
  if (EVT_RELEASE == rv) {
    dur = 0;
  } else {
    dur = nrfcxx::clock::uptime::now() - press_utt;
  }
  return rv;
}

void
Button::process ()
{
  using nrfcxx::clock::uptime;
  using nrfcxx::sensor::contact;

  bool alarm_scheduled = (ALARM_bit & state_);
  bool alarm_fired = false;
  if (alarm_scheduled) {
    /* Might be here because of a contact event or because of the
     * alarm.  If the alarm was scheduled when we cancel it, then it
     * didn't fire; otherwise we infer that it did.  In that case
     * there may be time-triggered state changes we'll need to
     * synthesize. */
    alarm_fired = (nrfcxx::clock::alarm::ST_scheduled != alarm_.cancel());
    state_ &= ~ALARM_bit;
  }

  uint64_t alarm_utt = 0;
  auto ss = contact_.snapshot();
  if (ss.ordinal != snapshot_.ordinal) {
    /* We have unprocessed contact events.  Process them.  These
     * supersede any time-triggered changes that may have also
     * occurred, so the alarm is ignored. */
    ss.process_changes(snapshot_, [this,&alarm_utt](auto eo, auto ts) {
        alarm_utt = process_(eo, ts);
      }, false);
    snapshot_ = ss;
  } else if (alarm_fired) {
    /* No unprocessed contact (e.g., app notification event was set
     * before we got around to taking the snapshot so we've already
     * processed it).  But an alarm did fire, so process
     * time-triggered changes. */
    process_utt = contact::utt_from_ts(ss.captured_ts());
    if (RELEASED == state_) {
      // No time-related changes from released state
    } else if (UNPRESSED == (STATE_mask & state_)) {
      state_ = RELEASED;
      callback_(EVT_RELEASE, process_utt - press_utt);
    } else if (HOLD_bit & state_) {
      state_ |= STUCK_bit;
      callback_(EVT_STUCK, process_utt - press_utt);
    } else {
      state_ |= HOLD_bit;
      callback_(EVT_HOLD, process_utt - press_utt);
      alarm_utt = press_utt + stuck_utt_;
    }
  } else if (alarm_scheduled) {
    /* A no-op contact event caused cancellation of an alarm.  The
     * deadline is unchanged so just restart it. */
    state_ |= ALARM_bit;
    alarm_.schedule();
  }
  if (alarm_utt) {
    state_ |= ALARM_bit;
    alarm_
      .set_deadline(alarm_utt)
      .schedule();
  }
}

uint64_t
Button::process_ (nrfcxx::sensor::contact::ordinal_type eo,
                  nrfcxx::sensor::contact::timestamp_type ts)
{
  using nrfcxx::sensor::contact;
  using nrfcxx::clock::uptime;

  uint64_t alarm_utt = 0;
  process_utt = contact::utt_from_ts(ts);
  bool pressed = state_xor ^ contact::state_from_ordinal(eo);
  if (RELEASED == state_) {
    if (!pressed) {
      puts("*** ERR release initiated while released");
      return 0;
    }
    state_ = PRESSED;
    press_utt = process_utt;
    callback_(EVT_CLICK, 0);
  } else if (xPRESSED_bit & state_) {
    if (PRESSED == (STATE_mask & state_)) {
      if (pressed) {
        puts("*** ERR press initiated while pressed");
      }
      /* Mark as unpressed and schedule debounce alarm */
      state_ = (~STATE_mask & state_) | UNPRESSED;
      alarm_utt = process_utt + debounce_utt_;
    } else {
      if (!pressed) {
        puts("*** ERR release initiated while unpressed");
      }
      /* Mark as pressed and check for changes in HOLD/STUCK state */
      state_ = (~STATE_mask & state_) | PRESSED;
      auto dur_utt = process_utt - press_utt;
      if (STUCK_bit & state_) {
        // no transition to apply
      } else if (HOLD_bit & state_) {
        // transition to stuck if held long enough
        if (stuck_utt_ <= dur_utt) {
          state_ |= STUCK_bit;
          callback_(EVT_STUCK, dur_utt);
        }
      } else if (hold_utt_ <= dur_utt) {
        // transition to hold if held long enough
        state_ |= HOLD_bit;
        callback_(EVT_HOLD, dur_utt);
      }
    }
  } else {
    puts("*** ERR unhandled state");
  }
  auto prb = (HOLD_bit | STUCK_bit | STATE_mask) & state_;
  /* Schedule alarm to detect passive transitions to HOLD/STUCK
   * states. */
  if (PRESSED == prb) {
    alarm_utt = press_utt + hold_utt_;
  } else if ((PRESSED | HOLD_bit) == prb) {
    alarm_utt = press_utt + stuck_utt_;
  }
  return alarm_utt;
}

bool
Button::alarm_callback (nrfcxx::clock::alarm& alarm)
{
  static_cast<Button *>(alarm.metadata)->notify_();
  return 0;
}

} // ns sensor
} // ns nrfcxx
