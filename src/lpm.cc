// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/lpm.hpp>

namespace nrfcxx {
namespace lpm {

void
state_machine::cancel_delay ()
{
  alarm_.cancel();
  state_ &= ~(STATE_BLOCKING_ALARM | STATE_FALLBACK_ALARM);
}

void
state_machine::reset ()
{
  alarm_.cancel();
  state_ = MS_OFF;
  error_ = 0;
}

void
state_machine::set_lost ()
{
  // Preserve the full unrecognized state, including flags.
  error_ = state_;
  // Keep the flags, add HAS_ERROR, and set the state to MS_LOST.
  state_ = (state_ & ~STATE_MASK) | STATE_HAS_ERROR | MS_LOST;
}

bool
state_machine::alarm_callback_ (clock::alarm& alarm)
{
  state_machine * self = static_cast<state_machine *>(alarm.metadata);
  self->state_ &= ~(STATE_BLOCKING_ALARM | STATE_FALLBACK_ALARM);
  self->post_event();
  return false;
}

void
state_machine::set_delay (int delay)
{
  if (0 == delay) {
    return;
  }
  if (0 < delay) {
    state_ |= STATE_BLOCKING_ALARM;
  } else {
    state_ |= STATE_FALLBACK_ALARM;
    delay = -delay;
  }
  alarm_.schedule_offset(delay);
}

int
lpsm_capable::lpsm_start ()
{
  using lpm::state_machine;
  if (state_machine::MS_OFF != machine_.state()) {
    return -1;
  }
  machine_.set_state(state_machine::MS_ENTRY_START, true);
  return 0;
}

int
lpsm_capable::lpsm_sample ()
{
  using lpm::state_machine;
  if (state_machine::MS_IDLE != machine_.state()) {
    return -1;
  }
  machine_.set_state(state_machine::MS_ENTRY_SAMPLE, true);
  return 0;
}

lpm::state_machine::process_flags_type
lpsm_capable::lpsm_process ()
{
  using lpm::state_machine;
  int delay = 0;
  process_flags_type pf = 0;

  if (state_machine::MS_OFF == machine_.state()) {
    pf |= state_machine::PF_OFF;
  }
  if (machine_.fallback_alarm_pending()) {
    machine_.cancel_delay();
  }
  int rc = 0;
  if (!(machine_.has_error()
         || machine_.blocking_alarm_pending()
         || (state_machine::MS_OFF == machine_.state()))) {
    rc = lpsm_process_(delay, pf);
    // printf("sm %d : delay %u pf %x\n", rc, delay, pf);
  }
  if (state_machine::MS_FAILED == machine_.state()) {
    pf |= state_machine::PF_FAILED;
  } else if (state_machine::MS_OFF == machine_.state()) {
    pf |= state_machine::PF_OFF;
  }
  process_rc_ = rc;
  if (0 > rc) {
    machine_.set_error(rc);
  } else if (delay) {
    machine_.set_delay(delay);
  }
  /* Clear flags on error so we don't waste time processing things
   * that aren't accurate. */
  if (machine_.has_error()) {
    pf = 0;
  }
  return pf;
}

} // namespace lpm
} // namespace nrfcxx
