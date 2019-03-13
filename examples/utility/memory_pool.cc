// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2018-2019 Peter A. Bigot

/** Demonstrate memory_pool API.
 *
 * Mostly this demonstrates how API violations cause failsafe
 * resets. */

#include <nrfcxx/impl.hpp>
#include <nrfcxx/utility.hpp>
#include <nrfcxx/newlib/system.h>
#include <cstring>
#include <cstdio>
#include <alloca.h>

#ifndef STACK_EXTENSION
#define STACK_EXTENSION 512
#endif /* STACK_EXTENSION */

namespace {
nrfcxx::utility::memory_pool pool2;
nrfcxx::utility::memory_pool<0> pool0;

/** Demonstrate reserving additional space on the stack. */
__attribute__((__section__(".stack.extension")))
volatile uint32_t stack_plus_512_words[512];

/** Demonstrate reserving additional space on the heap. */
__attribute__((__section__(".heap.extension")))
volatile uint32_t heap_plus_512_words[512];

void* volatile ap;
void recurse (void* ptr = 0) {
  int sr = nrfcxx::stack_space_remaining();
  ap = alloca(16);
  if (0 > nrfcxx::stack_space_remaining()) {
    printf("recursion has blown stack, last %d\n", sr);
    nrfcxx::delay_us(10000);
  }
  nrfcxx::validate_stack_pointer();
  recurse(ap);
}

} // anonymous

int
main (void)
{
  static __attribute__((__section__(".noinit.core_state")))
    nrfcxx::systemState::state_type core_state;
  union {
    char time[10];              // HH:MM:SS0
    struct {
      char skip[4];
      uint32_t magic;           // SS:M backwards as integer
    };
  } u = { __TIME__ };
  nrfcxx::systemState cs{core_state, u.magic};

  using namespace nrfcxx::utility;

  setvbuf(stdout, NULL, _IONBF, 0);
  puts("\n" __FILE__ " " __DATE__ " " __TIME__);
  auto reset_count = cs.state().reset_count;
  printf("Start %u magic %lx\n", reset_count, u.magic);
  if (nrfcxx::systemState::state_type::RESET_REAS_FAILSAFE & cs.state().reset_reas) {
    printf("Failsafe %x at %lx\n", cs.state().code, cs.state().last_pc);
  }

  printf("initial: p0 %u %u %u\n", pool0.size(), pool0.capacity(), pool0.align_bits);
  printf("initial: p2 %u %u %u\n", pool2.size(), pool2.capacity(), pool2.align_bits);

  if (0 == (7 & reset_count)) {
    puts("allocate from unconfigured\n");
    nrfcxx::delay_us(10000);
    pool2.allocate(12);
  }

  uint8_t region[32];
  pool0.configure(region + 1, region + sizeof(region)/sizeof(*region) - 1, 0x0a00);
  pool2.configure(region + 1, region + sizeof(region)/sizeof(*region) - 1, 0x0a02);
  printf("configured: p0 %u+%u/%u sb 0+30/30\n", pool0.size(), pool0.available(), pool0.capacity());
  printf("configured: p2 %u+%u/%u sb 0+24/24\n", pool2.size(), pool2.available(), pool2.capacity());

  auto a0a = static_cast<uint8_t*>(pool0.allocate(19));
  auto a2a = static_cast<uint8_t*>(pool2.allocate(19));
  printf("allocation: p0 %x %u %u %u sb 1 19 11 0\n", a0a - region, pool0.size(), pool0.available(), pool0.align_bits);
  printf("allocation: p2 %x %u %u %u sb 4 20 4 2\n", a2a - region, pool2.size(), pool2.available(), pool2.align_bits);

  if (1 == (7 & reset_count)) {
    puts("overallocate from pool0");
    nrfcxx::delay_us(10000);
    pool0.allocate(12);
  } else if (2 == (7 & reset_count)) {
    puts("overallocate from pool2");
    nrfcxx::delay_us(10000);
    pool2.allocate(12);
  } else if (3 == (7 & reset_count)) {
    pool2.clear();
    printf("cleared: p2 %u+%u/%u sb 24 24\n", pool2.size(), pool2.available(), pool2.capacity());
    nrfcxx::delay_us(10000);
    cs.reset(reset_count);
  } else if (4 == (7 & reset_count)) {
    const int heap_reserved{&__HeapLimit - &__HeapBase};
    const int heap_maximum{&__StackLimit - &__HeapBase};
    size_t hs = _nrfcxx_heap_used();
    int ha = 1 + heap_maximum - hs;
    printf("%d reserved heap from %p to %p extending to %p\n", heap_reserved, &__HeapBase, &__HeapLimit, &__StackLimit);
    const char* ehp = &__HeapBase + hs + ha;
    printf("%d of maximum %d heap used; allocating %d more to %p\n", hs, heap_maximum, ha, ehp);
    nrfcxx::delay_us(10000);
    void* mp = malloc(ha);
    printf("unexpectedly succeeded: %p\n", mp);
    memset(&core_state, 0, sizeof(core_state));
    while (true);
  } else if (5 == (7 & reset_count)) {
    printf("%d remaining of %u byte stack before recursing\n",
           nrfcxx::stack_space_remaining(), (unsigned int)(&__StackTop - &__StackLimit));
    recurse();
  } else if (6 == (7 & reset_count)) {
    puts("constructing scope from in-use pool");
    nrfcxx::delay_us(10000);
    auto scope = pool0.make_scoped();
  } else if (7 == (7 & reset_count)) {
    puts("constructing scope from free pool");
    nrfcxx::delay_us(10000);
    pool0.clear();
    {
      auto scope = pool0.make_scoped();
      a0a = static_cast<uint8_t*>(pool0.allocate(5));
      printf("in-scope p0 %x %u+%u/%u\n", a0a - region, pool0.size(), pool0.available(), pool0.capacity());
    }
    printf("past-scope p0 %u+%u/%u\n", pool0.size(), pool0.available(), pool0.capacity());
  }

  puts("completed examples");
  memset(&core_state, 0, sizeof(core_state));
}
