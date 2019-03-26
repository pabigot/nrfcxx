// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Peter A. Bigot

#include <cstring>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/crc.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/newlib/system.h>

extern "C" {
extern char __stext;          ///< symbol at start of application text
extern char __etext;          ///< symbol at end of application text
extern char __data_start__;   ///< symbol at start of application rodata
extern char __data_end__;     ///< symbol at end of application rodata
} // extern "C"

/* Provide a default implementation of the heap overallocation failure
 * function */
void*
__attribute__((__weak__))
_nrfcxx_sbrk_error (void* brk,
                    ptrdiff_t current,
                    ptrdiff_t increment)
{
  nrfcxx::failsafe(nrfcxx::FailSafeCode::HEAP_OVERRUN);
}

extern "C" {
/* This little joy loads r0 with the main or process stack pointer then
 * jumps to the systemState watchdog IRQ handler, which will then
 * pull the exception return address out and use that as the value
 * of last_pc showing where the program was when the reset occurred.
 *
 * See https://stackoverflow.com/questions/38618440
 */
__attribute__((__naked__))
void WDT_IRQHandler ()
{
  __ASM (
#if (NRF51 - 0) /* Cortex M0/M1 */
         "mrs r0, msp\n\t"
         "mov r1, lr\n\t"
         "mov r2, #4\n\t"
         "tst r1, r2\n\t"
         "beq 1f\n\t"
         "mrs r0, psp\n"
#else /* Cortex M3/M4 */
         "tst lr, #4\n\t"
         "ite eq\n\t"
         "mrseq r0, msp\n\t"
         "mrsne r0, psp\n\t"
#endif /* ARM Cortext Variant */
         "1:\tldr r1, =%0\n\t"
         "bx r1\n"
         : // no outputs
         : "i" (nrfcxx::systemState::wdt_irqhandler)
         );
}
} // extern "C"

namespace {

/** Set the OFF-mode retention bits for the RAM bank that contains the
 * provided address.
 *
 * @note We assume that the material to be preserved does not cross a
 * retained RAM boundary.  Since this is expected to be invoked for
 * material in a noinit section, and those are placed at the base of
 * the RAM region for the application, this is reasonably likely. */
void retain_address (const void* ptr)
{
  using namespace nrfcxx::nrf5;
  static constexpr unsigned int ram_base = 0x20000000;

  auto addr = reinterpret_cast<uintptr_t>(ptr);
  if (ram_base > addr) {
    return;
  }
#if (NRF51 - 0)
  /* NRF51x22 use 8 x 4 KiBy sections.
   *
   * For nRF51 there are four blocks of 8 KiBy RAM, each supporting
   * two AHB slaves, each of which has one section of 4 KiBy RAM.
   * Blocks 0 and 1 are controlled by RAMON and blocks 2 and 3 are
   * controlled by RAMONB.  Retention is managed at the block
   * level. */
  unsigned int block = (addr - ram_base) / 0x2000;
  auto& ramon = (2 <= block) ? POWER->RAMONB : POWER->RAMON;
  ramon |= (POWER_RAMON_OFFRAM0_RAM0On << ((block & 1) + POWER_RAMON_OFFRAM0_Pos));
#elif (NRF52832 - 0) || (NRF52840 - 0)
#if (NRF52840 - 0)
  /* nRF52840 splits RAM into two areas.  The low 64 KiBy has the same
   * layout as the nRF52832.  The remainder of the memory is handled
   * by its own AHB under RAM8, comprising six sections of 32 KiBy
   * each.  If the address is in that section, it's handled here. */
  static constexpr unsigned int ram8_base = 0x20010000;
  if (ram8_base <= addr) {
    // RAM8 uses 8 x 32 KiBy sections ; NRF52840 only
    unsigned int section = (addr - ram8_base) / 0x8000;
    POWER->RAM[8].POWERSET = (POWER_RAM_POWERSET_S0RETENTION_On << (section + POWER_RAM_POWERSET_S0RETENTION_Pos));
    return;
  }
#endif /* NRF52840 */
  /** nRF52832 (and low nRF52840) is similar to nRF51, but here the
   * eight AHB slaves each support two 4 KiBy sections that are
   * individually controlled. */
  unsigned int section = (addr - ram_base) / 0x1000;
  unsigned int block = section / 2;
  POWER->RAM[block].POWERSET = (POWER_RAM_POWERSET_S0RETENTION_On << ((1 & section) + POWER_RAM_POWERSET_S0RETENTION_Pos));
#else
#error Unsupported MCU
#endif
}

} // ns anonymous

namespace nrfcxx {

void
sleep_ms (unsigned int dur_ms)
{
  using clock::uptime;

  /* from_ms truncates toward zero.  Add a tick to ensure we sleep for
   * the requested period (may be 30.6 us longer, live with it). */
  int dur_utt = 1 + uptime::from_ms(dur_ms);
  do {
    dur_utt = uptime::sleep(dur_utt);
  } while (0 < dur_utt);
}

unsigned int
stack_fill_unused (unsigned int marker)
{
  auto sp = reinterpret_cast<unsigned int*>(&__HeapLimit);
  auto ep = reinterpret_cast<unsigned int*>(__get_MSP());
  while (sp < ep) {
    *sp++ = marker;
  }
  return (&__StackTop - reinterpret_cast<char*>(ep));
}

unsigned int
stack_infer_highwater (unsigned int marker)
{
  auto sp = reinterpret_cast<unsigned int*>(&__HeapLimit);
  auto ep = reinterpret_cast<unsigned int*>(__get_MSP());
  while ((sp < ep) && (marker == *sp)) {
    ++sp;
  }
  return (&__StackTop - reinterpret_cast<char*>(sp));
}

uint32_t
application_crc32 (bool exclude_data)
{
  /* The checksum that includes the data section is cached the first
   * time it is calculated. */
  static uint32_t precalc;
  if ((!exclude_data)
      && (0 != precalc)) {
    return precalc;
  }
  /* All code is between __stext and __etext. */
  auto calc = crc::crc32.append(&__stext, &__etext);
  if (!exclude_data) {
    /* Data is in RAM, but what initializes it immediately follows the
     * code at __etext. */
    auto len = &__data_end__ - &__data_start__;
    calc = crc::crc32.append(&__etext, len + &__etext, calc);
  }
  uint32_t rv = crc::crc32.finalize(calc);
  if (!exclude_data) {
    precalc = rv;
  }
  return rv;
}

systemState::state_type* systemState::statep_;
systemState::app_handler_type systemState::app_handler_;
unsigned int systemState::wfe_count_;

systemState::systemState (state_type& state,
                          uint32_t magic,
                          app_handler_type app_handler) :
  state_{state},
  magic_(magic ^ (state_type::DECL_MAGIC * sizeof(state)))
{
  /* Only one instance can be active in any system.  If this isn't the
   * first, it does nothing. */
  if (statep_) {
    return;
  }
  statep_ = &state_;
  app_handler_ = app_handler;

  /* Set so the memory segment holding the persisted state is retained
   * in system off mode.  This configuration was probably cleared by
   * the reset handler on power-up. */
  retain_address(statep_);

  /* If the magic number isn't set correctly, clear everything. */
  bool retained = (state_.magic == magic_);
  if (retained) {
    ++state_.reset_count;
    uint64_t uptime = 0;
    const uint64_t* omtp = state_.om_total;
    const uint64_t* const omtpe = omtp + sizeof(state_.om_total) / sizeof(*omtp);
    while (omtp < omtpe) {
      uptime += *omtp++;
    }
    state_.last_uptime = uptime;
    state_.total_uptime += uptime;
    memset(state_.om_total, 0, sizeof(state_.om_total));
    state.om_value = 0;
    state.om_updated = 0;
  } else {
    memset(&state_, 0, sizeof(state_));
    state_.magic = magic_;
  }
  stack_fill_unused(magic_);

  /* Read and clear the RESETREAS register.  Transfer its data into
   * the reset_reas field. */
  unsigned int reset_reas = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = -1;
  state_.reset_reas = (0x0F & reset_reas) | (0x01F0 & (reset_reas >> 12));
  /* If reset was not due to a watchdog clear the watchdog status for
   * this session. */
  if (!(state_type::RESET_REAS_DOG & reset_reas)) {
    state_.wdt_status = 0;
  }

  /* Copy programmatically-recorded reasons from previous session,
   * excluding the internal reset flag, then clear the flags for this
   * session. */
  if (state_type::RESET_REAS_CONTROLLED & state_.reset_reas_) {
    state_.reset_reas |= (state_.reset_reas_ & ~state_type::RESET_REAS_INTERNAL);
  }
  state_.reset_reas_ = 0;

  /* Copy an assigned code from the previous session. */
  if ((state_type::RESET_REAS_PROGRAMMATIC
       | state_type::RESET_REAS_SDFAULT
       | state_type::RESET_REAS_FAILSAFE) & state_.reset_reas) {
    state_.code = state_.code_;
  } else {
    state_.code = 0;
  }
  state_.code_ = 0;

  /* Clear sdfault_id if it wasn't set in the previous session */
  if (!(state_type::RESET_REAS_SDFAULT & state_.reset_reas)) {
    state_.sdfault_id = 0;
  }

  /* Clear last_pc unless it was set by the previous session. */
  if (!((state_type::RESET_REAS_PROGRAMMATIC
         | state_type::RESET_REAS_SDFAULT
         | state_type::RESET_REAS_WDTBARKED
         | state_type::RESET_REAS_FAILSAFE) & state_.reset_reas)) {
    state_.last_pc = 0;
  }

  if (app_handler_) {
    app_handler_(state_, false, retained);
  }
}

uint64_t
systemState::operationalModeBreakdown (uint64_t &sleep_utt,
                                       uint64_t &radio_utt) const
{
  uint64_t total = 0;
  uint64_t* omt = state_.om_total;
  sleep_utt = 0;
  radio_utt = 0;
  for (auto omi = 0U; omi < sizeof(state_.om_total) / sizeof(*omt); ++omi) {
    total += omt[omi];
    if (OM_SLEEP & omi) {
      sleep_utt += omt[omi];
    }
    if (OM_RADIO & omi) {
      radio_utt += omt[omi];
    }
  }
  return total;
}

unsigned int
systemState::stack_reserved () const
{
  return &__StackTop - &__StackLimit;
}

unsigned int
systemState::stack_used () const
{
  return stack_infer_highwater(magic_);
}

unsigned int
systemState::heap_reserved () const
{
  return &__HeapLimit - &__HeapBase;
}

unsigned int
systemState::heap_used () const
{
  return _nrfcxx_heap_used();
}

void
systemState::controlledResetPrep_ (unsigned int preserve)
{
  __disable_irq();
  updateOperationalMode(0, 0);
  if (statep_) {
    state_type::reset_reas_type reset_reas = state_type::RESET_REAS_CONTROLLED;
    if (preserve & state_type::RESET_REAS_FAILSAFE & statep_->reset_reas) {
      reset_reas |= state_type::RESET_REAS_FAILSAFE;
      // lastpc already set
      statep_->code_ = statep_->code;
    } else if (preserve & state_type::RESET_REAS_SDFAULT & statep_->reset_reas) {
      reset_reas |= state_type::RESET_REAS_SDFAULT;
      // lastpc, sdfault_id already set
      statep_->code_ = statep_->code;
    } else if (preserve & state_type::RESET_REAS_PROGRAMMATIC & statep_->reset_reas) {
      reset_reas |= state_type::RESET_REAS_PROGRAMMATIC;
      // lastpc already set
      statep_->code_ = statep_->code;
    }
    if (app_handler_) {
      app_handler_(*statep_, true, true);
    }
    statep_->reset_reas_ |= reset_reas;
  }
}

[[noreturn]]
void
systemState::systemOff (unsigned int preserve,
                        int button_psel)
{
  if (0 <= button_psel) {
    auto button_pr = gpio::pin_reference::create(button_psel);
    auto pin_cnf = gpio::PIN_CNF_RDONLY;
    pin_cnf |= ((GPIO_PIN_CNF_SENSE_High ^ board::button_active_low) << GPIO_PIN_CNF_SENSE_Pos);
    button_pr.configure(pin_cnf);
  }
  controlledResetPrep_(preserve);
  __DSB();
  nrf5::POWER->SYSTEMOFF = true;
  while (true) {
    __WFE();
  }
}

[[noreturn]]
void
systemState::controlledReset_ (unsigned int preserve,
                               bool bypass_watchdog)
{
  controlledResetPrep_(preserve);
  if (bypass_watchdog
      || (!NRF_WDT->RUNSTATUS)) {
    NVIC_SystemReset();
  }
  while (true) {
    __WFE();
  }
}

unsigned int
systemState::updateOperationalMode (unsigned int om_clear,
                                    unsigned int om_set)
{
  auto sp = statep_;
  unsigned int span = 0;
  if (sp) {
    primask mutex;
    unsigned int updated = nrfcxx::clock::uptime::now24();
    span = nrfcxx::clock::uptime::delta24(sp->om_updated, updated);
    sp->om_total[sp->om_value] += span;
    sp->om_value |= om_set;
    sp->om_value &= (NUM_OPERATIONAL_MODES - 1) & ~om_clear;
    sp->om_updated = updated;
  }
  return span;
}

void
systemState::WFE ()
{
  auto sleeper = make_scoped_sleeper();
  __WFE();
}

const char* const systemState::operationalModeText[NUM_OPERATIONAL_MODES] = {
  // OM_SLEEP cleared is A set is S
  // OM_HFCLK cleared is - set is H
  // OM_RADIO cleared is - set is R
  "A--",
  "S--",
  "AH-",
  "SH-",
  "A-R",
  "S-R",
  "AHR",
  "SHR",
};

bool
systemState::watchdogActive () const
{
  return NRF_WDT->RUNSTATUS;
}

void
systemState::watchdogFeed (unsigned int channel) const
{
  if (8 > channel) {
    NRF_WDT->RR[channel] = WDT_RR_RR_Reload;
  }
}

void
systemState::watchdogFeedMulti (unsigned int channel_mask) const
{
  unsigned int ci = 0;
  uint8_t bit = 1;
  while (bit && channel_mask) {
    if (channel_mask & bit) {
      channel_mask &= ~bit;
      NRF_WDT->RR[ci] = WDT_RR_RR_Reload;
    }
    bit <<= 1;
    ci += 1;
  }
}

int
systemState::watchdogInit (unsigned int delay_32KiHz,
                           unsigned int channel_mask,
                           bool run_in_sleep,
                           bool run_in_debug) const
{
  auto ext_mask = WATCHDOG_MASK_EXTENDED
    | (1U << watchdog_extended_channel::WATCHDOG_CHANNEL_COMMON);
  if (statep_ != &state_) {
    return -1;
  }
  if (ext_mask == (channel_mask & ext_mask)) {
    // Can't combine user use of common channel with extended use.
    failsafe(FailSafeCode::API_VIOLATION);
  } else if (WATCHDOG_MASK_EXTENDED & channel_mask) {
    // Convert extended channel to common channel
    channel_mask &= ~WATCHDOG_MASK_EXTENDED;
    channel_mask |= (1U << watchdog_extended_channel::WATCHDOG_CHANNEL_COMMON);
  }

  unsigned int config = 0;
  if (run_in_sleep) {
    config |= (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  }
  if (run_in_debug) {
    config |= (WDT_CONFIG_HALT_Run << WDT_CONFIG_HALT_Pos);
  }
  if (NRF_WDT->RUNSTATUS) {
    if ((delay_32KiHz == NRF_WDT->CRV)
        && (channel_mask == NRF_WDT->RREN)
        && (config == NRF_WDT->CONFIG)) {
      // Configuration matches previous run.  Feed everything and keep
      // going.
      watchdogFeedMulti(channel_mask);
    } else {
      // Configuration is different.  Let the watchdog timeout,
      // preserving carried-over state from the previous session.
      if (statep_) {
        statep_->reset_reas = state_type::RESET_REAS_INTERNAL;
      }
      controlledReset_((state_type::RESET_REAS_FAILSAFE
                        | state_type::RESET_REAS_SDFAULT
                        | state_type::RESET_REAS_PROGRAMMATIC), false);
      //NOTREACHED
    }
  } else {
    NRF_WDT->CRV = delay_32KiHz;
    NRF_WDT->RREN = channel_mask;
    NRF_WDT->CONFIG = config;
  }
  NRF_WDT->INTENCLR = -1;
  NVIC_SetPriority(WDT_IRQn, nrfcxx::IRQ_PRIORITY_APP_HIGH);
  NVIC_EnableIRQ(WDT_IRQn);
  NRF_WDT->INTENSET = (WDT_INTENSET_TIMEOUT_Enabled << WDT_INTENSET_TIMEOUT_Pos);
  NRF_WDT->TASKS_START = 1;
  return 0;
}

bool watchdog_extended_channel::failed_;
watchdog_extended_channel::chain_type watchdog_extended_channel::chain_;

watchdog_extended_channel::watchdog_extended_channel (unsigned int interval_utt) :
  interval_utt{interval_utt}
{
  chain_.link_back(*this);
  feed();
}

watchdog_extended_channel::~watchdog_extended_channel ()
{
  chain_.unlink(*this);
}

void
watchdog_extended_channel::feed ()
{
  last_fed_ = clock::uptime::now();
}

bool
systemState::watchdogCheckExtended ()
{
  if (!watchdog_extended_channel::failed_) {
    bool failed = false;
    unsigned int now = clock::uptime::now();
    for (auto& ch: watchdog_extended_channel::chain_) {
      if (!ch.check(now)) {
        failed = true;
        break;
      }
    }
    if (failed) {
      watchdog_extended_channel::failed_ = true;
    } else {
      watchdogFeed(watchdog_extended_channel::WATCHDOG_CHANNEL_COMMON);
    }
  }
  return !watchdog_extended_channel::failed_;
}

void
systemState::reset_ (uint32_t pc,
                     unsigned int code,
                     bool bypass_watchdog)
{
  if (statep_) {
    statep_->reset_reas_ = state_type::RESET_REAS_PROGRAMMATIC;
    statep_->last_pc = pc;
    statep_->code_ = code;
  }
  controlledReset_(0, bypass_watchdog);
}

void
systemState::failsafe_ (uint32_t pc,
                        unsigned int code)
{
  if (statep_) {
    statep_->reset_reas_ = state_type::RESET_REAS_FAILSAFE;
    statep_->last_pc = pc;
    statep_->code_ = code;
  }
  controlledReset_(0, false);
}

void
systemState::sd_fault_handler (uint32_t id,
                               uint32_t pc,
                               uint32_t info)
{
  auto sp = statep_;
  if (sp) {
    sp->reset_reas_ = state_type::RESET_REAS_SDFAULT;
    sp->sdfault_id = id;
    sp->last_pc = pc;
    sp->code_ = info;
  }
  controlledReset_(0, false);
}

uint64_t
systemState::total_now ()
{
  auto rv = clock::uptime::now();
  if (statep_) {
    rv += statep_->total_uptime;
  }
  return rv;
}

uint32_t
systemState::current_pc ()
{
  return reinterpret_cast<uint32_t>(__builtin_return_address(0));
}

void
systemState::wdt_irqhandler (void* sp)
{
  auto statep = nrfcxx::systemState::statep_;
  if (statep) {
    struct exception_frame_type {
      uint32_t r0;
      uint32_t r1;
      uint32_t r2;
      uint32_t r3;
      uint32_t r12;
      uint32_t lr;
      uint32_t pc;
      uint32_t xpsr;
    } const * const efp = reinterpret_cast<const exception_frame_type*>(sp);

    statep->wdt_status = NRF_WDT->REQSTATUS;
    if (efp) {
      statep->last_pc = efp->lr;
      statep->reset_reas_ = state_type::RESET_REAS_WDTBARKED;
    }
  }
  controlledReset_(0, false);
}

namespace board {

__attribute__((__weak__))
int
initialize (bool enable_hfxt)
{
  return clock::initialize(enable_hfxt);
}

} // ns board
} // ns nrfcxx
