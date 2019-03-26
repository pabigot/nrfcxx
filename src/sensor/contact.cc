// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <nrfcxx/sensor/contact.hpp>

namespace nrfcxx {
namespace sensor {

contact::reduced_timestamp_type
contact::state_type::time_of_change (ordinal_type ci) const
{
  static_assert(0 == (HISTORY_LENGTH & (HISTORY_LENGTH - 1)),
                "HISTORY_LENGTH must be a power of 2");
  return event_ts[(HISTORY_LENGTH - 1) & ci];
}

contact::reduced_timestamp_type
contact::state_type::time_of_change () const
{
  return event_ts[(HISTORY_LENGTH - 1) & ordinal];
}

contact::timestamp_type
contact::state_type::ordinal_ts () const
{
  return captured_ts_ - (static_cast<reduced_timestamp_type>(captured_ts_) - time_of_change());
}

contact::contact (uint8_t psel,
                  change_callback_bi callback,
                  uint32_t pin_cnf) :
  psel{psel},
  callback_{callback},
  state_bi_{},
  pinref_{gpio::pin_reference::create(psel)}
{
  state_bi_.ordinal = -1;
  state_bi_.captured_ts_ = now();
  pinref_.configure(pin_cnf);
  record_change_bi(live_state());
}

contact::state_type
contact::snapshot () const
{
  mutex_type mutex;
  auto rv = const_cast<state_type&>(state_bi_);
  rv.live_state = live_state();
  rv.captured_ts_ = now();
  return rv;
}

void
contact::gpiote_sense_bi_ (const periph::GPIOTE::sense_status_type* sp)
{
  while ((0 <= sp) && (sp->psel < psel)) {
    ++sp;
  }
  if (sp->psel == psel) {
    const unsigned int updates = sp->counter_state;
    bool level_changed = ((1 & updates) != recorded_state_bi_());
    /* Update if the level changed from what's recorded, or if at
     * least one level change was counted during the update (tests at
     * least one bit above 0x01 is set).  Assess a double-tap if level
     * did not change. */
    if (level_changed || (1 < updates)) {
      record_change_bi(!level_changed);
    }
  }
}

contact::timestamp_type
contact::state_type::process_changes (const state_type& prev,
                                      notify_fn notify,
                                      bool collapse) const
{
  ordinal_type const nevt = ordinal - prev.ordinal;

  /* If no events since previous snapshot there's nothing to do. */
  if (0 == nevt) {
    return 0;
  }

  /* Calculated absolute time of latest event. */
  const auto ordinal_ts = this->ordinal_ts();

  /* We only have to do work if somebody cares about the individual
   * events. */
  if (notify) {
    /* We need the absolute time of the first unprocessed event.
     * Start by assuming there's only one. */
    auto evt_ts = ordinal_ts;

    /* Store as many inter-unprocessed-event deltas as we can
     * reconstruct.  The limit is one less than the event timestamp
     * history. */
    contact::reduced_timestamp_type delta[contact::HISTORY_LENGTH - 1];
    contact::ordinal_type ndelta{};

    if (nevt < contact::HISTORY_LENGTH) {
      /* This snapshot carries the necessary information for all
       * unprocessed events. */
      ndelta = nevt - 1;
      if (0 < ndelta) {
        /* There are deltas.  Calculate them from most recent down to
         * oldest, adjusting the event timestamp so it ends up correct
         * for the oldest unprocessed event. */
        contact::ordinal_type ci{ordinal};
        contact::ordinal_type ndi = ndelta - 1;
        auto lts = time_of_change();
        do {
          auto pts = time_of_change(--ci);
          delta[ndi] = lts - pts;
          lts = pts;
          evt_ts -= delta[ndi];
        } while (0 != ndi--);
      }
    } else {
      /* We lost some unprocessed events.  Just collapse everything
       * between the last of the previous snapshot and the last of
       * this snapshot. */
      if (contact::state_from_ordinal(ordinal) == contact::state_from_ordinal(prev.ordinal)) {
        /* No effective state change, treat it like a double tap. */
        delta[ndelta++] = 0;
      }
    }

    /* Associate the first unprocessed event with the oldest absolute
     * unprocessed timestamp we have. */
    auto eo = prev.ordinal;
    notify(++eo, evt_ts);

    /* If we lost events, synthesize them at that same timestamp. */
    contact::ordinal_type neo = ordinal - ndelta;
    if (collapse) {
      eo = neo;
    } else {
      while (eo != neo) {
        notify(++eo, evt_ts);
      }
    }
    /* Finally emit events for the remaining unprocessed timestamps
     * belonging to this snapshot. */
    for (auto di = 0U; di < ndelta; ++di) {
      evt_ts += delta[di];
      ++eo;
      if ((!collapse)
          || delta[di]
          || ((ndelta - 1U) == di)) {
        notify(eo, evt_ts);
      }
    }
  }

  /* Return the elapsed time between the previous and latest
   * events. */
  return ordinal_ts - prev.ordinal_ts();
}

contact::timestamp_type
contact::epoch_ts () const
{
  // No mutex because this field of state_bi_ is never changed after
  // construction.
  return const_cast<state_type&>(state_bi_).captured_ts();
}

void
contact::record_change_bi (bool double_tap)
{
  unsigned int hi = (HISTORY_LENGTH - 1) & ++state_bi_.ordinal;
  state_bi_.event_ts[hi] = timestamp();
  if (double_tap) {
    auto const ts = state_bi_.event_ts[hi];
    hi = (HISTORY_LENGTH - 1) & ++state_bi_.ordinal;
    state_bi_.event_ts[hi] = ts;
  }
  if (callback_) {
    callback_(state_bi_.ordinal);
  }
}

} // ns sensor
} // ns nrfcxx
