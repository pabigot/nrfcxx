// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Example of overriding allocator.
 *
 * Shows how to provide a custom memory allocator to underly sbrk()
 * and hence all heap allocation.  This one simply wraps the default,
 * retaining information about each call.  It also preserves
 * information across an out-of-memory failure so we can see how much
 * memory is available.
 *
 * Also shows how to use the stack high water tracking
 * infrastructure. */

#include <cinttypes>
#include <cstdio>
#include <cstdlib>

#include <nrfcxx/core.hpp>
#include <nrfcxx/newlib/system.h>

/* Tweak this to change the resolution of the size checking code. */
static constexpr size_t block_size = 1024U;

extern "C" {
size_t heap_calls;
ptrdiff_t heap_incrs[8];
void* _use_sbrk (ptrdiff_t increment)
{
  static ptrdiff_t* hip = heap_incrs;
  ++heap_calls;
  if (hip < (heap_incrs + sizeof(heap_incrs) / sizeof(*heap_incrs))) {
    *hip++ = increment;
  }
  return _nrfcxx_sbrk_heap(increment);
}

void* _sbrk (ptrdiff_t increment) __attribute__((__alias__("_use_sbrk")));

} // extern C

namespace {
/* Best-practice idiom for managing state across sessions.  This
 * comes before anything else. */
__attribute__((__section__(".noinit.core_state")))
nrfcxx::systemState::state_type core_state;

__attribute__((__section__(".noinit.heap_used")))
size_t heap_used;

void
app_state_handler (const nrfcxx::systemState::state_type& ss,
                   bool is_reset,
                   bool retained)
{
  using nrfcxx::systemState;

  if (is_reset) {
  } else if (retained
             && (systemState::state_type::RESET_REAS_CONTROLLED & ss.reset_reas)) {
  } else {
    // Uncontrolled or unretained reset.  Clear everything.
    heap_used = 0;
  }
}

nrfcxx::systemState cs{core_state, 2018100509, app_state_handler};

} // ns anonymous

int
main (void)
{
  auto s0 = nrfcxx::stack_fill_unused(core_state.magic);

  size_t hc[16];
  size_t* hcp = hc;

  // i0 zero means pre-main didn't use the heap
  *hcp++ = heap_calls;

  char buf[128];
  char* bp = buf;
  bp += snprintf(bp, sizeof(buf) - (bp - buf), "boot %u magic %08" PRIx32 " at %p\nwith reset_reas %X code %X from %08" PRIX32 "\n",
                 cs.state().reset_count, cs.state().magic, &core_state,
                 cs.state().reset_reas, cs.state().code,
                 cs.state().last_pc);
  bp += snprintf(bp, sizeof(buf) - (bp - buf), "last heap_used %u\n", heap_used);

  // i1 zero shows that snprintf doesn't allocate from heap
  *hcp++ = heap_calls;

  puts("\n" __FILE__ " " __DATE__ " " __TIME__);
  // i2 three here shows stdio does allocate from heap:
  // * one test allocation of zero bytes
  // * an allocation of 436 bytes for the nosys state
  // * an allocation of 1032 bytes for a stdio buffer
  *hcp++ = heap_calls;

  printf("heap reserved: %u\n", &__HeapLimit - &__HeapBase);
  printf("stack reserved: %u\n", &__StackTop - &__StackLimit);
  printf("initial stack high water: %u\n", s0);
  printf("current stack high water: %u\n", nrfcxx::stack_infer_highwater(core_state.magic));

  printf("heap calls:");
  for (auto hp = hc; hp < hcp; ++hp) {
    printf(" %u", *hp);
  }
  printf("\nheap incrs:");
  for (auto hi = 0U; hi < heap_calls; ++hi) {
    printf(" %u", heap_incrs[hi]);
  }
  printf("\nheap used %u\n", _nrfcxx_heap_used());
  puts(buf);
  if (nrfcxx::FailSafeCode::HEAP_OVERRUN != static_cast<nrfcxx::FailSafeCode>(cs.state().code)) {
    printf("Allocating to maximum\n");
    nrfcxx::delay_us(100000);
    while (true) {
      heap_used = _nrfcxx_heap_used();
      auto incr = heap_used & (block_size - 1U);
      if (!incr) {
        incr = block_size;
      }
      printf("malloc %u got %p\n", incr, malloc(incr));
      nrfcxx::delay_us(10000);
    };
    puts("failed\n");
  }
  return 0;
}
