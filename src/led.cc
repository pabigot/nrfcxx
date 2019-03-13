// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2019 Peter A. Bigot

#include <nrfcxx/led.hpp>

namespace nrfcxx {
namespace led {

/* NB: Static definitions that depend on board-specific information
 * are in the board source file. */

led_type led_type::no_led{};

int
Pattern::configure (unsigned int pattern,
                    unsigned int interval_utt,
                    int reps)
{
  if (FLAG_ACTIVE & flags_) {
    // Can't configure when active.
    return -1;
  }
  pattern_ = pattern;
  if (MIN_INTERVAL_utt > interval_utt) {
    return -2;
  }
  interval_ = interval_utt;
  bool loop = (0 > reps);
  flags_ &= ~FLAG_LOOP;
  if (loop) {
    flags_ |= FLAG_LOOP;
  }
  reps_ = reps;
  if ((!loop)
      && ((reps_ != reps)
          || (0 == reps))) {
    /* Error if not looping and reps is zero or too many for storage
     * size. */
    reps_ = 0;
    return -3;
  } else if (loop) {
    reps_ = 0;
  }
  initial_reps_ = reps_;
  return 0;
}

int
Pattern::set_deadline (unsigned int deadline)
{
  int rv = 0;
  if (FLAG_ACTIVE & flags_) {
    rv = -1;
  } else {
    alarm_.set_deadline(deadline);
  }
  return rv;
}

int
Pattern::start (int delay_utt)
{
  if (FLAG_ACTIVE & flags_) {
    return -1;
  }
  if (FLAG_LOOP & flags_) {
    reps_ = 0;
  } else if (!initial_reps_) {
    return -2;
  } else {
    reps_ = initial_reps_ - 1;
  }
  idx_ = 32;
  flags_ |= FLAG_ACTIVE;
  led.enable();
  if (0 > delay_utt) {
    alarm_.schedule();
  } else {
    alarm_.schedule_offset(delay_utt);
  }
  return 0;
}

void
Pattern::cancel ()
{
  alarm_.cancel();
  complete_(false);
}

void
Pattern::complete_ (bool notify)
{
  led.off();
  led.disable();
  flags_ &= ~FLAG_ACTIVE;
  reps_ = 0;
  idx_ = 0;
  if (notify && notify_complete_) {
    notify_complete_();
  }
}

bool
Pattern::process_ ()
{
  if (0 == idx_) {
    complete_(true);
    return false;
  }
  unsigned int span = 0;
  unsigned int idx = idx_ - 1;
  unsigned int bit = (1U << idx);
  bool on = (pattern_ & bit);
  led.set(on);
  do {
    ++span;
    bit >>= 1;
  } while (bit && (on == !!(pattern_ & bit)));
  alarm_.set_deadline(alarm_.deadline() + span * interval_);
  if (bit) {
    idx_ -= span;
  } else if ((FLAG_LOOP & flags_) || (0 != reps_--)) {
    idx_ = 32;
  } else {
    idx_ = 0;
    reps_ = 0;
    if (!on) {
      complete_(true);
      return false;
    }
  }
  return true;
}

bool
Pattern::alarm_callback (nrfcxx::clock::alarm& alarm)
{
  return static_cast<Pattern *>(alarm.metadata)->process_();
}

} // namespace led
} // namespace nrfcxx
