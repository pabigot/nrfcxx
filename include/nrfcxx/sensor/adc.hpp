/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2017-2019 Peter A. Bigot */

/** Specializations of nrfcxx::periph::ADC for common ADC operations.
 *
 * @file */

#ifndef NRFCXX_SENSOR_ADC_HPP
#define NRFCXX_SENSOR_ADC_HPP
#pragma once

#include <cstring>
#include <initializer_list>

#include <nrfcxx/periph.hpp>
#include <nrfcxx/lpm.hpp>

namespace nrfcxx {
namespace sensor {

/** Specializations of nrfcxx::periph::ADC for common ADC
 * operations. */
namespace adc {

/** ADC instance to measure board Vdd.
 */
class vdd : public periph::ADCClient
{
  using super = periph::ADCClient;

public:
  /** Type for a function invoked from the ADC IRQ to provide
   * the calculated VDD.
   *
   * @note Functions of this type will be invoked by the ADC IRQ and
   * so are treated as though within a @link mutex_type
   * mutex-protected scope@endlink.
   *
   * @note Unlike the callback provided to a base {@link periph::ADC
   * ADC} module there is no provision to control the ADC within the
   * callback.
   *
   * @param vdd_mV the calculated Vdd expressed in millivolts. */
  using vdd_callback_bi = std::function<void(unsigned int vdd_mV)>;

  /** Construct an instance that measures board Vdd.
   *
   * @param callback optional callback to provide the result to the
   * application.  This is invoked after the result has been stored in
   * the instance.  If no callback is provided there is no
   * notification that the result has completed. */
  vdd (vdd_callback_bi callback = nullptr) :
    super{},
    vdd_callback_{callback},
    vdd_adc16_{}
  { }

  /** Convert a measured voltage to VDD voltage.
   *
   * @param vdd_adc16 the measured voltage in 16-bit ADC counts.
   *
   * @return The estimated digital voltage in millivolts. */
  int convert_adc16_mV (unsigned int vdd_adc16) const;

  /** Return the result of the most recent measurement.
   *
   * The value is zeroed on successful invocation of trigger(), and is
   * set to the measured Vdd value only when conversion completes.
   *
   * @return the measured Vdd in millivolts. */
  unsigned int vdd_mV () const
  {
    return convert_adc16_mV(vdd_adc16_);
  }

private:
  int configure_bi_ () override;
  void complete_bi_ () override;

  vdd_callback_bi vdd_callback_;
  volatile uint16_t vdd_adc16_;
};

/** ADC instance for voltage dividers.
 *
 * This class supports two use cases:
 * * If both the upper and lower resistor are nonzero, the class
 *   provides a measurement of the divider's input voltage.  This
 *   would be used, for example, to measure the voltage of an external
 *   power source such as a LiPo battery.
 * * If one of the resistors is zero, the class provides a measurement
 *   of the resistance at the missing leg.  This would be used, for
 *   example, when a thermistor or photo transistor is placed in the
 *   divider.
 */
class voltage_divider : public nrfcxx::periph::ADCClient
{
  using super = nrfcxx::periph::ADCClient;

  using packed_type = uint32_t;
  static constexpr auto AIN_WIDTH = 4U;
  static constexpr packed_type AIN_MASK = (1U << AIN_WIDTH) - 1;

  /** Maximum number of observations that can be stored internally. */
  static constexpr auto MAX_INTERNAL = 2;

  /** Additional flags for custom client. */
  enum flags_enum : flags_type {

    /** Flag set when the constructor provided external storage for raw
     * results. */
    FL_RAW_EXTERNAL = (FL_SUBCLASS_BASE << 0),

    /** Flag set when the sensor is to calculate the resistance of one
     * leg of the divider.
     *
     * When clear the sensor is to calculate the voltage of the divider
     * input. */
    FL_MEASURE_RESISTANCE = (FL_SUBCLASS_BASE << 1),
  };

  /** Helper to pack a sequence of channel specifications into a
   * 4-octet object. */
  template <typename Iterator>
  int8_t pack_values (packed_type& dest,
                      Iterator begin,
                      Iterator end)
  {
    static constexpr int8_t max_count = (8 * sizeof(dest)) / AIN_WIDTH;
    int8_t count = 0;
    unsigned int shift = 0;

    dest = 0;
    while ((begin != end) && (count < max_count)) {
      dest |= (AIN_MASK & *begin) << shift;
      shift += AIN_WIDTH;
      ++begin;
      ++count;
    }
    if (begin != end) {
      count = -1;
    }
    return count;
  }

  volatile uint16_t* raw_adc16_ ()
  {
    if (FL_RAW_EXTERNAL & flags_) {
      return raw_.ptr;
    }
    return raw_.val;
  }

  volatile const uint16_t* raw_adc16_ () const
  {
    if (FL_RAW_EXTERNAL & flags_) {
      return raw_.ptr;
    }
    return raw_.val;
  }

  void configure_instance_ (int8_t channel_count,
                            volatile uint16_t* raw);

public:

  /** The resistance of the upper leg of the voltage divider,
   * connected to the input voltage.
   *
   * If zero, a variable-resistance sensor is assumed to be on this
   * leg. */
  const unsigned int r1_Ohm;

  /** The resistance of the lower leg of the voltage divider,
   * connected to ground.
   *
   * If zero, a variable-resistance sensor is assumed to be on this
   * leg. */
  const unsigned int r2_Ohm;

  /** Generic constructor for an iterable set of channels and optional
   * external storage.
   *
   * @param r1_Ohm initializes #r1_Ohm
   *
   * @param r2_Ohm initializes #r2_Ohm
   *
   * @param ains_begin beginning iterator for AIN indexes sampled by the client
   *
   * @param ains_end ending iterator for AIN indexes sampled by the client
   *
   * @param raw pointer to external storage for raw ADC measurements.
   * This must be provided if the ains range exceeds two entries, and
   * the referenced memory must hold as many values as inputs are
   * provided by the ains range. */
  template <typename Iterator>
  voltage_divider (unsigned int r1_Ohm,
                   unsigned int r2_Ohm,
                   Iterator ains_begin,
                   Iterator ains_end,
                   volatile uint16_t* raw = nullptr) :
    r1_Ohm{r1_Ohm},
    r2_Ohm{r2_Ohm}
  {
    configure_instance_(pack_values(channel_ains_, ains_begin, ains_end), raw);
  }

  /** Constructor for common case of a single channel with internal
   * storage. */
  voltage_divider (unsigned int r1_Ohm,
                   unsigned int r2_Ohm,
                   uint8_t ain) :
    voltage_divider{r1_Ohm, r2_Ohm, &ain, 1 + &ain}
  { }

  /** Constructor for an explicit list of channels. */
  voltage_divider (unsigned int r1_Ohm,
                   unsigned int r2_Ohm,
                   std::initializer_list<uint8_t> ains,
                   volatile uint16_t* raw = nullptr) :
    voltage_divider{r1_Ohm, r2_Ohm, ains.begin(), ains.end(), raw}
  { }

  /** The number of channels used by this client. */
  size_t count () const
  {
    return channel_count_;
  }

  /** Extract the channel associated with the provided index.
   *
   * @param ci a zero-based index into the channels associated with the collection.
   *
   * @return the non-negative AIN index for the desired channel, or a
   * negative number if @p ci does not identify a configured channel. */
  int channel_ain (size_t ci) const
  {
    int rv = -1;
    if (ci < channel_count_) {
      rv = AIN_MASK & (channel_ains_ >> (ci * AIN_WIDTH));
    }
    return rv;
  }

  /** Retrieve the latest raw measurement for a channel.
   *
   * @param ci the zero-based channel index of interest.
   *
   * @return the ADC count measurement at the divider output,
   * expressed as a normalized unsigned 16-bit ADC measurement.  A
   * negative value is returned if the channel is not enabled. */
  int sample_adc16 (size_t ci = 0) const
  {
    int rv = -1;
    if (ci < channel_count_) {
      rv = raw_adc16_()[ci];
    }
    return rv;
  }

  /** Retrieve the latest output voltage measurement for a channel.
   *
   * @param ci the zero-based channel index of interest.
   *
   * @return the estimated voltage at the divider output.  A negative
   * value is returned if the channel is not enabled. */
  int sample_mV (size_t ci = 0,
                 bool truncate = false) const;

  /** Retrieve the latest input voltage measurement for a channel.
   *
   * @param ci the zero-based channel index of interest.
   *
   * @return the estimated voltage at the divider output.  A negative
   * value is returned if the channel is not enabled, or if the
   * divider was configured to measure resistance. */
  int input_mV (size_t ci = 0) const;

  /** Convert a raw ADC measurement to mV.
   *
   * The base class implementation assumes the reference is the
   * ADC-specific fixed reference, which is correct only when
   * configured to measure voltage (i.e. both legs of the voltage
   * divider are non-zero constants).  Subclasses may override for
   * other configurations.
   *
   * @note When measuring resistance the sampling reference and gain
   * are configured so that effective reference is Vdd, which is
   * system-dependent. */
  virtual int measurement_mV (uint16_t m16) const
  {
#if (NRF51 - 0)
    return (3 * peripheral::VBG_mV * m16) >> 16;
#else
    return (6 * peripheral::VBG_mV * m16) >> 16;
#endif
  }

  /** Convert a raw ADC measurement to the estimated resistance.
   *
   * @param m16 the raw ADC measurement of the divider output.
   *
   * @return the positive resistance of the variable resistance leg of
   * the divider, or a negative value if the ADC measurement is
   * zero indicating an open leg on the divider. */
  int measurement_Ohm (uint16_t m16) const
  {
    if (0 == m16) {
      return -1;
    }
    if (0 == r1_Ohm) {
      return r2_Ohm * ((uint64_t{1} << 16) - m16) / m16;
    }
    return r1_Ohm * static_cast<uint64_t>(m16) / ((1U << 16) - m16);
  }

  /** Retrieve the latest estimated resistance for a channel.
   *
   * @param ci the zero-based channel index of interest.
   *
   * @param filter_high_negative if `true` a large positive value that
   * might have been produced as the unsigned interpretation of a
   * @link nrf5::series::SAADC_Peripheral::near_zero small negative
   * value@endlink will be treated as zero.  This affects SAADC where
   * a negative voltage can be measured at ground due to error.
   *
   * @return the estimated resistance of the varying leg of the
   * divider.  A negative value is returned if the channel is not
   * enabled; the divider was configured to measure input voltage; or
   * no voltage was measured at the divider (open leg). */
  int sample_Ohm (size_t ci = 0,
                  bool filter_high_negative = false) const;

private:

  /** Storage to pack up to eight 4-bit input channel specifiers. */
  uint32_t channel_ains_ = 0;

  /** The number of input channels to be collected for each sample. */
  uint8_t channel_count_ = 0;

  /** For nRF51 this maintains the index of the channel that is
   * currently being collected. */
  uint8_t channel_idx_ = 0;

  /** The base ADC configuration for the desired measurement. */
  uint32_t config_ = 0;

  /** Storage for the ADC raw values, or pointer to external raw values.
   *
   * Up to two values can be stored internally.  Alternatively, the
   * constructor may be given a pointer to persistent storage that can
   * be used.  The #ptr field is valid iff #FL_RAW_EXTERNAL is set in
   * #flags_. */
  union {
    volatile uint16_t* ptr;
    uint16_t volatile val[MAX_INTERNAL];
  } raw_ = {nullptr};

  int configure_bi_ () override;
  int nrf51_next_bi_ (size_t ci) override;
};

/** Type holding Steinhart-Hart coefficients and extreme values.
 *
 * These coefficients are used to calculate the temperature given the
 * measured resistance of an NTC thermistor.  Steinhart-Hart is
 * significantly more accurate across a wide range of temperatures
 * than linear interpolation, and generally requires less space than
 * table lookup.
 *
 * The class also provides calculations for a thermistor installation
 * where Vcc leads to a voltage divider where the top resistor is the
 * thermistor, the bottom resistor is #ref_Ohm, and the ADC is
 * measuring the divided voltage.  In this configuration an open
 * thermistor allows no current flow (divided voltage is zero), and a
 * shorted one allows maximum flow (divided voltage is Vcc).
 *
 * For generality it is assumed a 16-bit ADC is used. */
class SteinhartHart
{
public:
  /** Store the coefficients and reference values.
   *
   * Default values correspond to the values for a full-scale (-40 Cel
   * to 200 Cel) measurement with the <a
   * href="https://adafru.it/372">Adafruit #372</a> NTC thermistor.
   *
   * @param a the coefficient scaling ln(R)^0 in the Steinhart-Hart
   * equation.
   *
   * @param b the coefficient scaling ln(R)^1 in the Steinhart-Hart
   * equation.
   *
   * @param c the coefficient scaling ln(R)^3 in the Steinhart-Hart
   * equation.
   *
   * @param open_adc16 the value used to initialize #open_adc16.
   *
   * @param short_adc16 the value used to initialize #short_adc16.
   *
   * @param ref_Ohm the resistance of the top resistor in the measured
   * voltage divider. */
  constexpr SteinhartHart (float a = 1.19438315714902408e-03,
                           float b = 2.19852114398042317e-04,
                           float c = 1.72427510143621963e-07,
                           uint16_t open_adc16 = 2281,
                           uint16_t short_adc16 = 65133) :
    a{a},
    b{b},
    c{c},
    open_adc16{open_adc16},
    short_adc16{short_adc16}
  { }

  /** Coefficient of ln(R)^0 */
  const float a;

  /** Coefficient of ln(R)^1 */
  const float b;

  /** Coefficient of ln(R)^3 */
  const float c;

  /** 16-bit ADC upper threshold for open thermistor.
   *
   * An open thermistor has an infinite resistance corresponding to an
   * extremely low temperature.
   *
   * If the measured 16-bit ADC value is at or below this value assume
   * the thermistor is open. */
  const uint16_t open_adc16;

  /** 16-bit ADC lower threshold for shorted thermistor.
   *
   * A shorted thermistor has a tiny resistance corresponding to an
   * extremely high temperature.
   *
   * If the measured 16-bit ADC value is at or above this value assume
   * the thermistor is shorted. */
  const uint16_t short_adc16;

  /** Calculate the temperature corresponding to a measured
   * thermistor resistance.
   *
   * @param therm_Ohm the measured resistance of the thermistor, in
   * Ohms.
   *
   * @return the estimated temperature of the thermistor in hundredths
   * of a degree Kelvin. */
  int temperature_cK (unsigned int therm_Ohm) const;
};

/** Steinhart-Hart configurations for common thermistors and
 * applications. */
namespace steinhartHart {
/** Constants for full-scale measurement with Adafruit thermistor.
 *
 * This range is calculated for -40 Cel to 200 Cel with a midpoint at
 * 25 Cel.
 *
 * @see https://adafru.it/372 Adafruit #372 Thermistor */
extern const SteinhartHart adafruit372fullScale;

/** Constants for HVAC measurement with Adafruit thermistor.
 *
 * This range is calculated for -30 Cel to 75 Cel with a midpoint at
 * 20 Cel.  This is optimized for residential ductwork carrying cooled
 * and heated forced air.
 *
 * Open and short thresholds correspond to 125 Cel and -40 Cel
 * respectively.
 *
 * @see https://adafru.it/372 Adafruit #372 Thermistor */
extern const SteinhartHart adafruit372hvac;

/** Constants for full-scale measurement with Adafruit thermistor.
 *
 * This range is calculated for -30 Cel to 10 Cel with a midpoint at 0
 * Cel.  This is optimized for non-commercial refrigerator and freezer
 * compartments.
 *
 * Open and short thresholds correspond to 125 Cel and -40 Cel
 * respectively.
 *
 * @see https://adafru.it/372 Adafruit #372 Thermistor */
extern const SteinhartHart adafruit372refrigerator;
} // ns steinhartHart

/** Class supporting thermistor measurements.
 *
 * Each thermistor is expected to be positioned as the upper resistor
 * in a voltage divider.  The value of the lower resistor and the
 * Steinhart-Hart thermistor coefficients must be the same for all
 * channels sampled by a given instance. */
class ntcThermistor : public voltage_divider
{
  using super = voltage_divider;

public:
  /** Constant used to convert between Kelvin and Celsius */
  static constexpr int ABSOLUTE_ZERO_cCel = -27315;

  /** An flag value for invalid temperatures.
   *
   * The value is chosen to allow its positive and negative values to
   * be stored in an `int16_t` with magnitudes that are not reasonable
   * for true observed measurements.  Any observation with a magnitude
   * equal to or greater than this is invalid.
   *
   * Distinct values may indicate different types of failure.  For
   * example the base value indicates a reading that was not
   * performed, while increasing the magnitude by one identifies
   * thermistor inputs that are faulted @link SHORT_cCel short@endlink
   * or @link OPEN_cCel open@endlink. */
  static constexpr int INVALID_cCel = 30000;

  /** Measurement returned by temperature_cCel() for a shorted
   * thermistor.
   *
   * A shorted thermistor is diagnosed when the measured voltage
   * across the thermistor is at or above #short_adc16.  This is in
   * the direction of extremely high temperatures.
   *
   * @note The magnitude of this value is greater than @ref
   * INVALID_cCel and is equal to the magnitude of @ref OPEN_cCel.
   * The value is positive. */
  static constexpr int SHORT_cCel = INVALID_cCel + 1;

  /** Measurement returned by temperature_cCel() for an open
   * thermistor.
   *
   * An open thermistor is diagnosed when the measured voltage across
   * the thermistor is at or below #open_adc16.  This is in the
   * direction of extremely low temperatures.
   *
   * @note The magnitude of this value is greater than @ref
   * INVALID_cCel and is equal to the magnitude of @ref SHORT_cCel.
   * The value is negative.  */
  static constexpr int OPEN_cCel = - SHORT_cCel;

  /** Generic constructor for an iterable set of thermistor channels
   * and optional external storage.
   *
   * @param ains_begin as with voltage_divider::voltage_divider.
   *
   * @param ains_end as with voltage_divider::voltage_divider.
   *
   * @param raw as with voltage_divider::voltage_divider.
   *
   * @param coeff pointer to the thermistor coefficients.
   *
   * @param ref_Ohm the value of the reference resistor in the
   * thermistor voltage divider. */
  template <typename Iterator>
  ntcThermistor(Iterator ains_begin,
                Iterator ains_end,
                volatile uint16_t*raw = nullptr,
                const SteinhartHart* coeff = &steinhartHart::adafruit372fullScale,
                unsigned int ref_Ohm = 10000) :
    super{0, ref_Ohm, ains_begin, ains_end, raw},
    steinhartHart{coeff}
  { }

  /** Constructor for an explicit list of channels. */
  ntcThermistor(std::initializer_list<uint8_t> ains,
                volatile uint16_t*raw = nullptr,
                const SteinhartHart* coeff = &steinhartHart::adafruit372fullScale,
                unsigned int ref_Ohm = 10000) :
    ntcThermistor(ains.begin(), ains.end(), raw, coeff, ref_Ohm)
  { }

  /** Convert a measured voltage to a temperature.
   *
   * @param therm_adc16 the measured voltage in 16-bit ADC counts.  A
   * value of 65535 would correspond to Vcc (a shorted thermistor); a
   * value of zero would correspond to no voltage (an open
   * thermistor).
   *
   * @return The measured temperature of the thermistor in hundredths
   * of a degree Celsius, or #OPEN_cCel or #SHORT_cCel for a broken
   * thermistor. */
  int convert_adc16_cCel (unsigned int therm_adc16);

  /** Convert the stored measured voltage to a thermisor.
   *
   * @param ci the thermistor channel to use.
   *
   * @return as with convert_adc16_cCel() */
  int temperature_cCel (size_t ci = 0);

  /** Return the last raw 16-bit ADC thermistor reading. */
  unsigned int therm_adc16 (size_t ci = 0) const;

  /** Return the estimated thermistor resistance, in Ohms. */
  unsigned int therm_Ohm (size_t ci = 0) const;

  /** The Steinhart-Hart coefficients and extreme values for
   * the thermistor in the circuit. */
  const SteinhartHart* const steinhartHart;
};

/** A class that implements an @link lpm::lpsm_capable LPM state
 * machine@endlink around a periph::ADCClient.
 *
 * The state machine handles coordination of sampling and calibration,
 * provides API to initiate a sample or calibration, and ensures
 * client-specific operations to @link periph::ADCClient::sample_setup
 * prepare for@endlink and @link periph::ADCClient::sample_teardown
 * cleanup after@endlink measurements are executed at the appropriate
 * time.
 *
 * lpm::lpsm_capable::lpsm_process() for this client returns the
 * following flags:
 * * #PF_CALIBRATED when ADC calibration completes, allowing
 *   lpm::lpsm_capable::lpsm_sample() to be successfully invoked;
 * * lpm::state_machine::PF_OBSERVATION when ADC sampling completes.
 *   Subclasses must provide API to retrieve the measurement(s) when
 *   lpm::state_machine::PF_OBSERVATION is returned.
 *
 * lpsm_calibrate() should be invoked by the application on startup
 * and whenever the ambient temperature changes by more than 10 Cel
 * from the last calibration.  Alternatively, applications may invoke
 * lpsm_bypass_calibration() to indicate that calibration is managed
 * externally. */
class lpsm_wrapper : public lpm::lpsm_capable
{
  using super_lpsm = lpm::lpsm_capable;

  /* State transitions:
   *
   * * ENTRY_START transitions to IDLE.
   *
   * * ENTRY_CALIBRATE claims the ADC, invokes sample_setup_(), and
   *   transitions to CALIBRATE (after a setup delay if necessary).
   *
   * * CALIBRATE queues an ADC calibration and falls into
   *   EXIT_CALIBRATE.
   *
   * * EXIT_CALIBRATE loops until calibration resolved, then records
   *   state of calibration and transitions to either IDLE (invoking
   *   sample_teardown_() and releasing the ADC) or ENTRY_SAMPLE
   *   depending on the source of the calibration request.
   *
   * * ENTRY_SAMPLE if not calibrated jumps to ENTRY_CALIBRATE.
   *   Otherwise it claims the ADC, invokes sample_setup_(), and
   *   transitions to SAMPLE (after a setup delay if necessary).
   *
   * * SAMPLE queues an ADC collection and transitions to EXIT_SAMPLE.
   *
   * * EXIT_SAMPLE is blocked until the ADC completes, then it invokes
   *   sample_teardown_() and processes the sample, emitting
   *   #PF_OBSERVATION if the collection was valid.  It then release
   *   the ADC and transitions to IDLE.
   *
   * Calibration and sampling are queued ADCClient operations.
   * Failure to successfully initiate the queued operation translates
   * to a machine error.
   *
   * NOTE: The design of this machine is influenced by the inability
   * to queue multiple operations on the same client.  As a result the
   * calibrate and sample operations cannot be made to occur within
   * the same claim session.  This is a problem only when multiple
   * clients are managed with distinct LPSM wrappers, and calibration
   * may be performed with both single-ended and differential sampling
   * configurations producing different offsets, as documented in the
   * devzone posting below.
   *
   * See: https://devzone.nordicsemi.com/f/nordic-q-a/43316/nrf52832-saadc-configuration-effects-on-calibration */
  static constexpr auto MS_ENTRY_CALIBRATE = lpm::state_machine::MS_ENTRY_SAMPLE + 1;
  static constexpr auto MS_CALIBRATE = lpm::state_machine::MS_SAMPLE + 1;
  static constexpr auto MS_EXIT_CALIBRATE = lpm::state_machine::MS_EXIT_SAMPLE + 1;

  /** Type for #flags_bi_. */
  using flags_type = uint8_t;

  /** Defined flags for #flags_bi_. */
  enum flags_enum : flags_type
  {
    /** Bit set in flags_bi_ to indicate that a queued calibration
     * operation is in pending or in progress. */
    FL_CALIBRATING = 0x01,

    /** Bit set in flags_bi_ to indicate that a queued sampling
     * operation is in pending or in progress. */
    FL_SAMPLING = 0x02,

    /** Bit set in flags_bi_ to indicate that a queued operation is
     * pending. */
    FL_PENDING = 0x08,

    /** Bit set in flags_bi_ to indicate that calibration has been
     * successfully performed. */
    FL_CALIBRATED = 0x10,

    /** Bit set in flags_bi_ to indicate that a sample should be
     * initiated as soon as calibration completes. */
    FL_THEN_SAMPLE = 0x20,
  };

public:
  /** Mutex domain is that of the ADC. */
  using mutex_type = periph::ADCClient::mutex_type;

  /** Sensor-specific indication from
   * lpm::lpsm_capable::lpsm_process() that the ADC has been
   * calibrated. */
  static constexpr auto PF_CALIBRATED = lpm::state_machine::PF_APP_BASE << 0;

  /** Construct an instance.
   *
   * @param notify as with lpm::state_machine::state_machine. */
  lpsm_wrapper (notifier_type notify,
                periph::ADCClient& client) :
    super_lpsm{notify},
    client_{client}
  { }

  template <typename Client = periph::ADCClient>
  Client& client () const
  {
    return reinterpret_cast<Client&>(client_);
  }

  /** Initiate a calibration.
   *
   * @return zero on success, or an error if the machine is not in a
   * state where calibration is allowed. */
  int lpsm_calibrate ();

  /** Set the internal flag that indicates calibration has been performed.
   *
   * Applications may want to do this when calibration is managed
   * through an external process.
   *
   * @return zero on success, or an error if the machine is not in a
   * state where calibration is allowed. */
  int lpsm_bypass_calibration ();

private:
  void lpsm_reset_ () override
  {
    mutex_type mutex;
    flags_bi_ = 0;
  }

  int lpsm_process_ (int& delay,
                     process_flags_type& lpf) override;

  periph::ADCClient& client_;
  int queue_rc_ = 0;
  flags_type flags_bi_ = 0;
};

} // ns adc
} // ns sensor
} // ns nrfcxx

#endif /* NRFCXX_SENSOR_ADC_HPP */
