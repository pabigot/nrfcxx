// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2019 Peter A. Bigot

#include <cinttypes>
#include <cstdio>
#include <cstring>

#include <nrfcxx/impl.hpp>
#include <nrfcxx/utility.hpp>

namespace nrfcxx {
namespace utility {

namespace {

inline void
emit_address (uintptr_t addr)
{
  printf("%08" PRIxPTR " ", addr);
}

inline uintptr_t
align_base (uintptr_t base)
{
  unsigned int offset = base & 0x0F;
  if (0 != offset) {
    emit_address(base - offset);
    return offset;
  }
  return 0;
}

} // anonymous

namespace internal {

template<>
void
display_data<uint8_t> (const uint8_t* dp,
                       size_t count,
                       uintptr_t base)
{
  static const char data_text_spacer[] = "  ";
  const uint8_t* const edp = dp + count;
  const uint8_t* adp = dp;
  bool with_nl = false;
  unsigned int skips = align_base(base);

  /* If the base is not 16-byte aligned, backfill for the material
   * that belongs in the first partial line. */
  if (skips) {
    unsigned int n;
    for (n = 0; n < skips; ++n) {
      unsigned int nsp = 3 + (8 == n);
      while (0 < nsp--) {
        putchar(' ');
      }
    }
    with_nl = true;
  }
  while (dp < edp) {
    if (0 == (base & 0x0F)) {
      if (adp < dp) {
        printf(data_text_spacer);
        while (skips) {
          putchar(' ');
          --skips;
        }
        while (adp < dp) {
          putchar(isprint(*adp) ? *adp : '.');
          ++adp;
        }
      }
      adp = dp;
      if (with_nl) {
        putchar('\n');
      } else {
        with_nl = true;
      }
      emit_address(base);
    } else if (0 == (base & 0x07)) {
      putchar(' ');
    }
    printf(" %02x", *dp++);
    ++base;
  }
  if (adp < dp) {
    while (base & 0x0F) {
      if (0 == (base & 0x07)) {
        putchar(' ');
      }
      printf("   ");
      ++base;
    }
    printf("  ");
    while (adp < dp) {
      putchar(isprint(*adp) ? *adp : '.');
      ++adp;
    }
  }
  putchar('\n');
}

template<>
void
display_data<uint32_t> (const uint32_t* dp,
                        size_t count,
                        uintptr_t base)
{
  const uint32_t* const edp = dp + count;
  bool with_nl = false;
  unsigned int skips;

  /* Force word alignment.  Otherwise things get ugly with the skip
   * logic. */
  base &= ~(uintptr_t)3;
  skips = align_base(base);
  if (skips) {
    unsigned int n;
    skips /= 4;
    for (n = 0; n < skips; ++n) {
      unsigned int nsp = 9;
      while (0 < nsp--) {
        putchar(' ');
      }
    }
    with_nl = true;
  }
  while (dp < edp) {
    if (0 == (base & 0x0F)) {
      if (with_nl) {
        putchar('\n');
      } else {
        with_nl = true;
      }
      emit_address(base);
    }
    printf(" %08" PRIx32, *dp++);
    base += sizeof(*dp);
  }
  putchar('\n');
}

} // namespace internal

int
Persist::configure (const uint32_t* begin,
                    const uint32_t* end)
{
  if (begin_) {
    return -1;
  }
  if (!begin) {
    begin = &__PersistBase;
  }
  if (!end) {
    end = &__PersistLimit;
  }
  if (reinterpret_cast<uintptr_t>(begin) & (PAGE_SIZE - 1)) {
    return -2;
  }
  if (reinterpret_cast<uintptr_t>(end) & (PAGE_SIZE - 1)) {
    return -3;
  }
  unsigned int span = end - begin;
  if (0 >= span) {
    return -4;
  }

  begin_ = reinterpret_cast<const header_type*>(begin);
  end_ = reinterpret_cast<const header_type*>(end);
  first_ = nullptr;
  erased_ = 0;
  bool force_clear = false;
  auto hp = reinterpret_cast<const header_type*>(begin_);
  while ((hp < end_)
         && (TAG_UNUSED != hp->tag)) {
    /* An entry with a span of zero indicates a corrupt region.  @todo
     * Clear it. */
    if (0 == hp->span) {
      force_clear = true;
      hp = begin_;
      break;
    }
    /* Record the spans that are no longer in use. */
    if (TAG_ERASED == hp->tag) {
      erased_ += hp->span;
    } else if (!first_) {
      first_ = hp;
    }
    hp = hp->next();
  }
  unused_ = (hp < end_) ? hp : nullptr;
  if (force_clear) {
    clear();
  }
  return 0;
}

void
Persist::clear ()
{
  if (!begin_) {
    failsafe(code_);
  }
  uint32_t pp = reinterpret_cast<uint32_t>(begin_);
  uint32_t ppe = reinterpret_cast<uint32_t>(end_);
  nrf5::NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
  while (pp < ppe) {
    nrf5::NVMC->ERASEPAGE = pp;
    pp += PAGE_SIZE;
  }
  nrf5::NVMC->CONFIG = 0;
  erased_ = 0;
  first_ = nullptr;
  unused_ = begin_;
}

const Persist::header_type*
Persist::find_ (uint16_t tag)
{
  if (!begin_) {
    failsafe(code_);
  }
  const header_type* hp = nullptr;
  if ((TAG_ERASED != tag)
      && (TAG_UNUSED != tag)) {
    hp = first_;
    while (hp) {
      /* Not found if past the end or found the unused record. */
      if ((end_ <= hp)
          || (TAG_UNUSED == hp->tag)) {
        hp = nullptr;
        break;
      }
      /* Found if the tag matches.  If somebody wants to find the first
       * TAG_ERASED record this way that's ok. */
      if (tag == hp->tag) {
        break;
      }
      hp = hp->next();
    }
  }
  return hp;
}

void
Persist::erase_ (const header_type* hp)
{
  union {
    uint32_t u32;
    header_type hdr;
  } u;
  u.hdr = *hp;
  u.hdr.tag = TAG_ERASED;
  erased_ += hp->span;
  auto dp = reinterpret_cast<uint32_t*>(const_cast<header_type*>(hp));
  nrf5::NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  *dp = u.u32;
  nrf5::NVMC->CONFIG = 0;
}

const Persist::header_type*
Persist::addReplace (uint16_t tag,
                     const void* data,
                     size_t count)
{
  const header_type* rv = nullptr;

  /* Locate the previous record with this tag, if any.  Note that this
   * call will failsafe if the region has not been configured. */
  auto ep = find(tag);

  /* Can't add if there's no unused record.  (Might be able to add if
   * that record doesn't allow for any data.) */
  if (!unused_) {
    return rv;
  }

  /* Can't add if the space available for data doesn't accommodate the
   * aligned record content. */
  size_t data_span = (count + sizeof(uint32_t) - 1) / sizeof(uint32_t);
  if (data_span > available()) {
    return rv;
  }

  /* Can't add if we're supposed to provide record data but the caller
   * didn't give us any. */
  if (data_span && (!data)) {
    return rv;
  }

  /* We're committed.  If there's a previous record mark it erased. */
  if (ep) {
    erase_(ep);
  }

  /* Construct the header for the record we're going to add. */
  union {
    uint32_t u32;
    header_type hdr;
  } u;
  u.hdr.tag = tag;
  u.hdr.span = 1 + data_span;

  /* Get a uint32_t pointer to where the record will be stored, and
   * update unused_ to be past it. */
  rv = unused_;
  auto dp = reinterpret_cast<uint32_t*>(const_cast<header_type*>(unused_));
  unused_ += u.hdr.span;
  if (end_ == unused_) {
    unused_ = nullptr;
  }

  /* Write the header first, then the data, one word at a time. */
  nrf5::NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  *dp++ = u.u32;
  if (data && count) {
    auto sp = reinterpret_cast<const uint8_t*>(data);
    auto spe = sp + count;
    while (sp < spe) {
      size_t n = (spe - sp);
      if (n > sizeof(u.u32)) {
        n = sizeof(u.u32);
      }
      /* For consistency leave unused bytes as 0xFF. */
      u.u32 = -1;
      memcpy(&u.u32, sp, n);
      *dp++ = u.u32;
      sp += n;
    }
  }
  nrf5::NVMC->CONFIG = 0;

  /* If we don't have a first used record, set it. */
  if (!first_) {
    first_ = rv;
  }
  return rv;
}

ptrdiff_t
Persist::extract (header_type* buffer,
                  size_t length) const
{
  auto dhp = buffer;
  auto const dhpe = dhp + length;
  const header_type* shp = first_;
  size_t stored = 0;
  while (shp && (shp < end_)) {
    if (TAG_UNUSED == shp->tag) {
      break;
    }
    if (TAG_ERASED != shp->tag) {
      if (dhp) {
        if ((dhp + shp->span) > dhpe) {
          return -1;
        }
        memcpy(dhp, shp, shp->span * sizeof(*shp));
        dhp += shp->span;
      }
      stored += shp->span;
    }
    shp = shp->next();
  }
  return stored;
}

int
Persist::restore (const header_type* buffer,
                  size_t length)
{
  auto shp = buffer;
  auto const shpe = shp + length;
  int nr = 0;
  while (shp && (shp < shpe)) {
    if (addReplace(shp->tag, shp->data, (shp->span - 1) * sizeof(*shp))) {
      ++nr;
    } else {
      nr = -1;
      break;
    }
    shp += shp->span;
  }
  return nr;
}

int
Persist::gc ()
{
  auto nw = extract();
  if (0 > nw) {
    return nw;
  }
  header_type buf[nw];
  if (nw) {
    extract(buf, nw);
  }
  clear();
  int rv = 0;
  if (nw) {
    rv = restore(buf, nw);
  }
  return rv;
}

} // namespace utility
} // namespace nrfcxx
