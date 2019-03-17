/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2019 Peter A. Bigot */

/** Support for voltage regulators.
 *
 * @file */

#ifndef NRFCXX_MISC_REGULATOR_HPP
#define NRFCXX_MISC_REGULATOR_HPP
#pragma once

#include <nrfcxx/gpio.hpp>

namespace nrfcxx {
namespace misc {

/** Generic API for a voltage regulator.
 *
 * On startup the source is unpowered.  Clients can @ref request
 * power; the device remains powered until the last client invokes
 * release().
 *
 * Client count is maintained internally.  There is no allocated
 * resource to associate specific clients with their requests, so it
 * is possible for mismanagement to result in the supply being left
 * on, or being powered off while a client is dependent on it.
 *
 * The class also provides @ref delay_utt as a convenience in cases
 * where a significant delay may be required before the source
 * stabilizes.
 *
 * @note This base class must be extended to provide the mechanism for
 * controlling the supply.  See, for example @ref
 * gpio_controlled_voltage. */
class controlled_voltage
{
public:
 /** Construct the instance.
   *
   * @param delay a delay, measured in uptime ticks, necessary before
   * the supply stabilizes.  This is made available through @ref
   * delay_utt. */
  controlled_voltage (unsigned int delay_utt = 0) :
    delay_utt{delay_utt}
  { }

  controlled_voltage (const controlled_voltage&) = delete;
  controlled_voltage& operator=(const controlled_voltage&) = delete;
  controlled_voltage (controlled_voltage&&) = delete;
  controlled_voltage& operator=(controlled_voltage&&) = delete;

  virtual ~controlled_voltage () = default;

  /** The delay necessary before the voltage stabilizes.
   *
   * The units are consistent with per clock::uptime::now().
   *
   * @note There is no API to determine whether the voltage supply has
   * already stabilized, or how long since it was first requested.  If
   * the application is unable to centrally manage regulator requests
   * it should probably respect the delay after each localized
   * request, in case the supply is still unstable. */
  const unsigned int delay_utt;

  /** Indicate whether the regulator is supplying voltage. */
  bool enabled () const
  {
    return 0 < clients_;
  }

  /** Request that the regulator supply voltage. */
  void request ()
  {
    primask mutex;

    if (0 == clients_++) {
      assert_();
    }
  }

  /** Release a request that the regulator supply voltage. */
  void release ()
  {
    primask mutex;

    if (0 == --clients_) {
      deassert_();
    }
  }

protected:
  virtual void assert_ ()
  { }

  virtual void deassert_ ()
  { }

private:
  unsigned int clients_ = 0;
};

/** A voltage regulator controlled by a generic GPIO with a designated
 * active level.
 *
 * This class is designed to support a supply controlled through a
 * generic GPIO, e.g. a PNP-based high-side transistor or an NPN-based
 * low-side transistor.
 *
 * @tparam SIGNAL a class conforming to the instantiated behavior of
 * @ref gpio::active_signal.  This can usually be inferred from the
 * constructor parameter. */
template <typename SIGNAL>
class gpio_controlled_voltage : public controlled_voltage
{
  using super = controlled_voltage;

public:
  using active_signal_type = SIGNAL;

  /** Construct the instance.
   *
   * @param ctrl signal controlling the voltage supply.  The supply
   * should be on when @p ctrl is asserted, and off when it is
   * deasserted.
   *
   * @param delay a delay, measured in uptime ticks, necessary before
   * the supply stabilizes. */
  gpio_controlled_voltage (const active_signal_type& ctrl,
                           unsigned int delay_utt = 0) :
    super{delay_utt},
    ctrl_{ctrl}
  {
    ctrl_.deassert();
    ctrl_.enable();
  }

private:
  void assert_ () override
  {
    ctrl_.assert();
  }

  void deassert_ () override
  {
    ctrl_.deassert();
  }

  const active_signal_type ctrl_;
};

} // ns misc
} // namespace

#endif /* NRFCXX_MISC_REGULATOR_HPP */
