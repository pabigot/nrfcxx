// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Demonstrates the persist API. */

#include <cstdio>
#include <cstring>

#include <nrfcxx/clock.hpp>
#include <nrfcxx/impl.hpp>
#include <nrfcxx/utility.hpp>

namespace {

using nrfcxx::utility::Persist;

__attribute__((__section__(".persist.page0")))
uint32_t page0_data[Persist::PAGE_SIZE / sizeof(uint32_t)];

__attribute__((__section__(".persist.zznoise")))
uint32_t noise;

Persist persisted{nrfcxx::FailSafeCode::APPLICATION_BASE + 23};
Persist page0;

Persist::header_type backup[1024];

__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

nrfcxx::systemState cs{core_state, 2018101411};

} // anonymous

void check_assertion (bool condition,
                      const char* msg,
                      unsigned int lineno,
                      nrfcxx::systemState& cs)
{
  if (!condition) {
    printf("%u assertion failed: %s\n", lineno, msg);
    nrfcxx::delay_us(100000);
    cs.reset(lineno);
  }
  printf("ok: %s\n", msg);
}

#define ASSERT(_cond) do {                              \
    const bool res = (_cond);                           \
    check_assertion(res, #_cond, __LINE__, cs);         \
  } while (0)

int
main (void)
{
  using namespace nrfcxx;
  board::initialize();

  auto p0b = page0_data;
  auto const p0e = p0b + sizeof(page0_data) / sizeof(*page0_data);

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n\n" __FILE__ " " __DATE__ " " __TIME__);

  /* Display information about the cause of the restart. */
  {
    nrfcxx::clock::uptime::text_type buf;
    printf("boot %u with reset_reas %X from %08" PRIX32 ", up %s\n",
           cs.state().reset_count, cs.state().reset_reas,
           cs.state().last_pc,
           nrfcxx::clock::uptime::as_text(buf, cs.state().last_uptime));
    printf("total uptime %s\n",
           nrfcxx::clock::uptime::as_text(buf, cs.state().total_uptime));
  }

  auto reset_count = cs.state().reset_count;
  bool barked = systemState::state_type::RESET_REAS_DOG & cs.state().reset_reas;
  if (!(systemState::state_type::RESET_REAS_CONTROLLED & cs.state().reset_reas)) {
    // Any state copied from the previous instance is incomplete or
    // invalid.
    puts("***UNCONTROLLED RESET\n");
  }
  if (systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    printf("- due to failsafe, code %x, pc %08lx\n", cs.state().code, cs.state().last_pc);
    delay_us(10000);
    systemState::systemOff(0, -1);
  } else if (systemState::state_type::RESET_REAS_PROGRAMMATIC & cs.state().reset_reas) {
    if (barked && (cs.state().magic == cs.state().code)) {
      puts("- to put previous watchdog to sleep");
    } else {
      printf("- due to ASSERT failed at line %u\n", cs.state().code);
      utility::display_data(p0b, (p0e - p0b), reinterpret_cast<uintptr_t>(p0b));
      memset(&core_state, 0, sizeof(core_state));
      return EXIT_FAILURE;
    }
  } else if (barked) {
    printf("- due to watchdog, unloaded channels: %X\n", cs.state().wdt_status);
  } else if (systemState::state_type::RESET_REAS_SREQ & cs.state().reset_reas) {
    printf("- due to direct system reset\n");
  }

  int rc;
  if (2 >= (0x03 & reset_count)) {
    rc = page0.configure(p0b, p0e);
    ASSERT(0 == rc);
    printf("Initialization save OK: %d\n", rc);
  } else {
    printf("Inducing mis-use by failing to configure...");
  }
  if (1 == (0x03 & reset_count)) {
    printf("Attempting to overallocate...");
    rc = page0.configure(&noise, &noise + 1);
    ASSERT(0 > rc);
    printf("rejected: %d\n", rc);
  }
  delay_us(10000);

  printf("cleared %d, reclaimable %u, unused %u\n", page0.is_cleared(),
         page0.erased(),
         page0.available());

  delay_us(10000);
  auto ts0 = clock::uptime::timestamp24{};
  page0.clear();
  printf("erase took %u us\n", (unsigned int)clock::uptime::to_us(ts0.delta()));

  const uint16_t tag = 0x1234;
  ASSERT(nullptr == page0.find(tag));
  const char test[] = "something";
  auto hp = page0.addReplace(0x1234, test, sizeof(test));
  ASSERT(p0b == reinterpret_cast<const uint32_t*>(hp));
  ASSERT(0x12340004 == *p0b);
  utility::display_data(p0b, 8, reinterpret_cast<uintptr_t>(p0b));
  ASSERT(0 == page0.erased());
  ASSERT(1004 == page0.available());
  printf("cleared %d, reclaimable %u, unused %u\n", page0.is_cleared(),
         page0.erased(),
         page0.available());
  ASSERT(hp == page0.find(tag));
  const auto hp2 = page0.addReplace(0x1234, test, sizeof(test));
  ASSERT((hp + 4) == hp2);
  ASSERT(0x00000004 == *p0b);
  utility::display_data(p0b, 8, reinterpret_cast<uintptr_t>(p0b));

  hp = page0.addReplace(cs.state().magic & 0xFFFF, &reset_count, sizeof(reset_count));
  printf("recorded reset_count as %04x for %u\n", hp->tag, hp->span);
  printf("cleared %d, reclaimable %u, unused %u\n", page0.is_cleared(),
         page0.erased(),
         page0.available());

  rc = page0.extract();
  printf("extract requires %d\n", rc);
  rc = page0.extract(backup, rc);
  printf("extract produced %d\n", rc);
  utility::display_data(reinterpret_cast<uint32_t*>(backup), rc, 0);
  page0.clear();
  rc = page0.restore(backup, rc);
  printf("restore got %d:\n", rc);
  utility::display_data(p0b, 8, reinterpret_cast<uintptr_t>(p0b));

  hp = page0.addReplace(0x1234, &cs.state(), sizeof(cs.state()));
  printf("replaced tag 1234 with state, %u erased:\n", page0.erased());
  utility::display_data(p0b, 36, reinterpret_cast<uintptr_t>(p0b));
  rc = page0.gc();
  printf("gc got %d:\n", rc);
  utility::display_data(p0b, 36, reinterpret_cast<uintptr_t>(p0b));

  return 0;
}
