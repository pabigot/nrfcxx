/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2019 Peter A. Bigot */

/** @file
 * Assorted utility types and functions */
#ifndef NRFCXX_UTILITY_HPP
#define NRFCXX_UTILITY_HPP
#pragma once

#include <nrfcxx/core.hpp>
#include <type_traits>
#include <array>
#include <limits>

/** @cond DOXYGEN_EXCLUDE */
extern "C" {
  extern const uint32_t __PersistBase;
  extern const uint32_t __PersistLimit;
}
/** @endcond */

namespace nrfcxx {

/** Various utilities, probably not dependent on nRF51 at all */
namespace utility {

/** Class supporting allocation from a fixed-size region.
 *
 * This capability is intended to support allocation of aligned
 * variable-sized blocks of memory from an externally supplied region.
 * A prime use case is populating runtime state structures
 * corresponding to enabled capabilities when an application is being
 * initialized.
 *
 * Allocation is done from a region that is supplied externally during
 * configuration.  Attempts to allocate more space than is available
 * will result in a @link failsafe fail-safe reset@endlink using a
 * code provided along with the region.  API is available to release
 * all allocations from the region at once, but individual allocations
 * cannot be released.
 *
 * @tparam ALIGN_BITS initializes #align_bits. */
template <unsigned int ALIGN_BITS = 2>
class memory_pool {
  /** Mask to isolate bits that are below the alignment bit.
   *
   * This also serves as the largest value that can be added to an
   * aligned span without increasing it to a different aligned
   * span. */
  static constexpr size_t ignore_mask{(1U << ALIGN_BITS) - 1};

  /** Mask to isolate bits at and above the alignment bit. */
  static constexpr uintptr_t align_mask = ~static_cast<uintptr_t>(ignore_mask);

public:
  using pool_type = memory_pool<ALIGN_BITS>;

  /** The number of bits used for alignment of the return
   * pointers. */
  static constexpr unsigned int align_bits = ALIGN_BITS;

  /** RAII class to maintain a shared memory pool within a block scope
   *
   * This is used when there is a global "scratch" pool available for
   * transient use by a variety of components.  It ensures that the
   * pool is empty when the holder instance is constructed, and
   * cleared when the holder instance is destructed.
   *
   * Usage:
   *
   *     memory_pool scratch{};
   *     scratch.configure(...);
   *     // ...
   *     {
   *        auto scope = scratch.make_scoped();
   *        auto buffer = static_cast<uint8_t*>(scratch.allocate(23));
   *        // ...
   *     }
   *     // pool is again unused
   */
  class scoped {
  public:
    /** Contained pool is cleared on destruction */
    ~scoped ()
    {
      pool_.clear();
    }

  protected:
    /** @cond DOXYGEN_EXCLUDE */
    friend memory_pool;
    scoped (pool_type& pool) :
      pool_{pool}
    { }

    scoped (const scoped&) = delete;
    scoped& operator= (const scoped&) = delete;
    scoped (scoped&&) = delete;
    scoped& operator= (scoped&) = delete;
    /** @endcond */

  private:
    pool_type& pool_;
  };

  /** Construct an RAII holder that maintains the instance as a scratch pool.
   *
   * The system will failsafe if the pool has allocations when the
   * holder is constructed.
   *
   * The pool will be @link clear cleared@endlink when the holder is
   * destructed. */
  scoped make_scoped ()
  {
    if (size()) {
      failsafe(code_);
    }
    return {*this};
  }

  /** Construct the instance.
   *
   * A newly-constructed instance does not have an associated region.
   * configure() must be called before allocate() is invoked.
   * Violation of this requirement will produce a failsafe reset. */
  constexpr memory_pool () :
    begin_{},
    end_{},
    head_{},
    code_{static_cast<unsigned int>(FailSafeCode::MEMORY_POOL)}
  { }

  /** @cond DOXYGEN_EXCLUDE */
  memory_pool (const memory_pool&) = delete;
  memory_pool& operator= (const memory_pool&) = delete;
  memory_pool (memory_pool&&) = delete;
  memory_pool& operator= (memory_pool&&) = delete;
  /** @endcond */

  /** Provide the allocation region and failsafe code.
   *
   * This method must be invoked before allocate().
   *
   * @param begin a pointer to the start of the space available for allocation.
   *
   * @param end a pointer just past the end of the space available for allocation.
   *
   * @param code the `code` value to pass to failsafe() if an
   * unsatisfiable allocation is made. */
  void configure (void* begin,
                  void* end,
                  unsigned int code = FailSafeCode::MEMORY_POOL)
  {
    if (begin_) {
      failsafe(code_);
    }
    code_ = code;

    /* The region begins at the aligned address no less than the
     * provided address. */
    begin_ = reinterpret_cast<unsigned char*>(align_mask & (ignore_mask + reinterpret_cast<uintptr_t>(begin)));

    /* The region ends at the aligned address no greater than the
     * provided address. */
    end_ = reinterpret_cast<unsigned char*>(align_mask & reinterpret_cast<uintptr_t>(end));

    /* If somebody does something stupid, including configuring an
     * empty region, failsafe. */
    if (end_ <= begin_) {
      failsafe(code_);
    }
    head_ = begin_;
  }

  /** Release all allocations from the pool.
   *
   * After invoking this pointers returned from allocate() are
   * invalidated and must not be dereferenced. */
  void clear ()
  {
    head_ = begin_;
  }

  /** Return the total number of bytes in the region (allocated
   * plus unallocated). */
  size_t capacity () const
  {
    return (end_ - begin_);
  }

  /** Return the number of allocated bytes in the region. */
  size_t size () const
  {
    return (head_ - begin_);
  }

  /** Return the number of unallocated bytes in the region. */
  size_t available () const
  {
    return (end_ - head_);
  }

  /** Allocate a block of at least `span` bytes.
   *
   * If the allocation cannot be satisfied the system will @link
   * failsafe failsafe reset@endlink using the `code` passed
   * in configure().
   *
   * @return A pointer to an #align_bits-bit aligned block of
   * memory. */
  void* allocate (size_t span)
  {
    /* Increase the span so all pointers are aligned. */
    span = align_mask & (span + ignore_mask);
    if ((!head_) || ((head_ + span) > end_)) {
      failsafe(code_);
    }
    void* rv = reinterpret_cast<void *>(head_);
    head_ += span;
    return rv;
  }

private:
  /* Pointer to the aligned start of the configured region. */
  unsigned char* begin_;

  /* Pointer to the aligned address just past the start of the
   * unallocated portion of the configured region. */
  unsigned char* end_;

  /* Pointer to the aligned start of the unallocated portion of the
   * configured region. */
  unsigned char* head_;

  /* The code to be used in the failsafe reset if allocation fails. */
  unsigned int code_;
};

/** @cond DOXYGEN_EXCLUDE */
namespace internal {

/* Generic API underlying display_data */
template <typename T>
void display_data (const T* dp,
                   size_t count,
                   uintptr_t base);

/* Known specialization for octets.  ASCII values are displayed in
 * right column. */
template <>
void display_data<uint8_t> (const uint8_t* dp,
                            size_t count,
                            uintptr_t base);

/* Known specialization for words. */
template <>
void display_data<uint32_t> (const uint32_t* dp,
                             size_t count,
                             uintptr_t base);

} // namespace internal
/** @endcond */

/** Display a block of data on the console.
 *
 * @tparam T the type of the underlying data.  Must be an integral
 * type.  Any `volatile` qualifier is explicitly removed.
 *
 * @param dp pointer to the data to display
 *
 * @param count the number of data elements to display
 *
 * @param base the base address for the data at @p *dp.  This affects
 * the address column of the output and need not be related to the
 * actual value of @p dp. */
template <typename T>
inline void display_data (const T* dp,
                          size_t count,
                          uintptr_t base)
{
  using base_type = typename std::remove_volatile<T>::type;
  internal::display_data(const_cast<const base_type*>(dp), count, base);
}

/** Class supporting persistence of tagged records to
 * non-volatile memory.
 *
 * Page-aligned regions of flash memory may be reserved by defining a
 * non-empty `PERSIST` region in the linker memory map and defining
 * objects that are placed in the `.persist` (or a `.persist.*`)
 * section.  Instantiating an instance of Persist and configuring it
 * to reference one of those regions allows tagged records to be
 * stored in flash and retrieved by their tag.
 *
 * * Tags must be unique to each persisted record (type).  Attempting
 *   to add a record (instance) with the same tag as an existing
 *   record will cause the previous record (instance) to be removed.
 * * The system does not automatically perform garbage collection.
 *   When there is insufficient space to store a record the
 *   application must use extract() to extract the active records,
 *   clear() to erase the region, and restore() to put the records
 *   back leaving the maximum possible space for additional records.
 * * On nRF52 the flash is restricted to a certain number of writes
 *   per 128-word block before additional writes to the block may
 *   cause data loss.  If more than 53 records are stored in a single
 *   block this limitation may be exceeded for nRF52832.  The system
 *   does not detect or mitigate against violating this requirement.
 * * Retrieved records provide information that describes their
 *   content length in aligned words.  If it is necessary to get byte-
 *   or half-word lengths of record data that information must be
 *   extracted from the record or other external sources.
 */
class Persist {
public:

  /** header_type::tag value for a record that has been
   * erased. */
  static constexpr uint16_t TAG_ERASED = 0;

  /** header_type::tag value for the placeholder header of the
   * next record to be written. */
  static constexpr uint16_t TAG_UNUSED = 0xFFFF;

  /** pointer to the first word of the `PERSIST` memory region.
   *
   * This may be used as the `begin` parameter to configure() when a
   * single persist instance that covers the region is desired. */
  static constexpr const uint32_t* REGION_BEGIN = &__PersistBase;

  /** pointer to the first word after the `PERSIST` memory
   * region.
   *
   * This may be used as the `end` parameter to configure() when a
   * single persist instance that covers the region is desired.  */
  static constexpr const uint32_t* REGION_END = &__PersistLimit;

  /** Header identifying record content and its span. */
  struct header_type {
    /** Span of the record in words, including the header. */
    uint16_t span;

    /** Tag identifying the record content. */
    uint16_t tag;

    /** The record content. */
    uint8_t data[0];

    /** Get a pointer to the following record.
     *
     * If `tag` is #TAG_UNUSED the resulting pointer will be at or
     * past the end of the containing region.  The caller is
     * responsible for detecting this situation either before or after
     * such a pointer is produced. */
    const header_type* next () const
    {
      /* By construction `span` cannot be zero. */
      static_assert(sizeof(*this) == sizeof(uint32_t),
                    "header is too big");
      return this + span;
    }
  };

#if (51 == NRF_SERIES) || (NRFCXX_DOXYGEN - 0)
  /** Number of bytes in a flash page.
   *
   * @note This constant is nRF5 series-specific; this value is for
   * nRF51. */
  static constexpr unsigned int PAGE_SIZE = 1024;

  /** Number of bytes in a flash block.
   *
   * @note This constant is nRF5 series-specific; this value is for
   * nRF51. */
  static constexpr unsigned int BLOCK_SIZE = PAGE_SIZE;
#elif (52 == NRF_SERIES)
  static constexpr unsigned int PAGE_SIZE = 4096;
  static constexpr unsigned int BLOCK_SIZE = 512;
#endif

  /** Construct the wrapper class.
   *
   * @param code the @link failsafe failsafe code@endlink to be
   * generated when an unrecoverable error is detected using the
   * Persist API. */
  constexpr Persist (FailSafeCode code = FailSafeCode::PERSIST_VIOLATION) :
    code_{code},
    begin_{},
    end_{},
    first_{},
    unused_{},
    erased_{}
  { }

  /** Alternative constructor for uncast failsafe codes */
  constexpr Persist (unsigned int code) :
    Persist(static_cast<FailSafeCode>(code))
  { }

  /** @cond DOXYGEN_EXCLUDE */
  Persist (const Persist&) = delete;
  Persist& operator= (const Persist&) = delete;
  Persist (Persist&& ) = delete;
  Persist& operator= (Persist&) = delete;
  /** @endcond */

  /** Configure the instance to operate on a specific persisted
   * region.
   *
   * An instance can be successfully configured at most once.
   *
   * @param begin page-aligned pointer to NVMC data.
   *
   * @param end page-aligned pointer to NVMC data, at or above `begin`
   *
   * @return zero if the configuration was acceptable, otherwise a
   * negative error code. */
  int configure (const uint32_t* begin,
                 const uint32_t* end);

  /** Find the record with the given tag, if it is present.
   *
   * @param tag the header_type::tag value for a matching record.
   *
   * @return a pointer to the record if it is found, otherwise a null
   * pointer. */
  const header_type* find (uint16_t tag)
  {
    return find_(tag);
  }

  /** Remove the record with the given tag.
   *
   * @param tag the header_type::tag value for a matching record.
   *
   * @return zero if the record was found and removed, or a negative
   * error code. */
  int erase (uint16_t tag)
  {
    int rv{-1};
    const header_type* hp{find_(tag)};
    if (hp) {
      erase_(hp);
      rv = 0;
    }
    return rv;
  }

  /** Add or replace the record with the given tag.
   *
   * @param tag the header_type::tag value for the record.  If a
   * record with this tag is already present it will be erased prior
   * to creating the new record.
   *
   * @param data pointer to the data to be associated with the record.
   * If a null pointer is passed the record will have no data.
   *
   * @param count the number of bytes at `data` that should be
   * preserved at header_type::data in the resulting record.
   *
   * @return a null pointer if the record could not be created,
   * otherwise a pointer to the new record.  If a null pointer is
   * returned any previous record with `tag` is left unmodified. */
  const header_type* addReplace (uint16_t tag,
                                  const void* data,
                                  size_t count);

  /** Extract the active records from the region.
   *
   * @param buffer where to store the record data.  If a null pointer
   * is passed the records will not be extracted, but the return value
   * will indicate the buffer length required to successfully extract
   * the data.
   *
   * @param length the number of words that may be stored in `buffer`
   * when that is a non-null pointer.
   *
   * @return the number of words stored in `buffer` (or that would be
   * stored in `buffer` if it were not null), or a negative error code
   * if the provided non-null `buffer` and `length` cannot accommodate
   * all records. */
  ptrdiff_t extract (header_type* buffer = nullptr,
                     size_t length = 0) const;

  /** Replay @link extract extracted@endlink records into the
   * persisted region.
   *
   * If a record has the same tag as a record in the region, it
   * supersedes the existing record as with addReplace().
   *
   * @param buffer pointer to a record set extracted from a Persist
   * instance using extract().
   *
   * @param length the non-negative size returned from the extract()
   * call that filled `buffer`.
   *
   * @return A non-negative count of the number of records restored,
   * if all records were restored.  A negative error code indicates
   * that the region did not have sufficient unused space to store the
   * records. */
  int restore (const header_type* buffer,
               size_t length);

  /** Extract, clear, and restore the page contents.
   *
   * This uses the stack to store the extracted information.
   *
   * @return as with restore(). */
  int gc ();

  /** Return `true` iff the region is configured and starts
   * with a used record. */
  bool is_cleared () const
  {
    return begin_ && (TAG_UNUSED == begin_->tag);
  }

  /** Return the number of bytes available for payload in a new
   * record.
   *
   * @note This is the maximum payload length for a single record.  It
   * does not include the length of the headers for additional
   * records. */
  size_t available () const
  {
    size_t rv{};
    if (unused_) {
      rv = (end_ - unused_ - 1) * sizeof(*unused_);
    }
    return rv;
  }

  /** Return the number of bytes that can be reclaimed if the
   * region is @link extract extracted@endlink and @link restore
   * restored@endlink. */
  size_t erased () const
  {
    return erased_ * sizeof(uint32_t);
  }

  /** Clear all data from the persisted region. */
  void clear ();

protected:
  /** Mark an existing record as erased. */
  void erase_ (const header_type* hp);

  /** Locate a record by tag. */
  const header_type* find_ (uint16_t tag);

  /** Code used in failsafe resets related to persistence failures. */
  const FailSafeCode code_;

  /** Inclusive lower bound of the persisted region. */
  const header_type* begin_;

  /** Exclusive upper bound of the persisted region. */
  const header_type* end_;

  /** Pointer to the first used and unerased record in the
   * region.
   *
   * If the region has never had a record added the pointer is
   * null. */
  const header_type* first_;

  /** Pointer to the region's unused record.
   *
   * If there is no unused space in the region the pointer is null.
   *
   * @note An unused record has tag #TAG_UNUSED and span of 0xFFFF.
   * There is at most one of these in the region, and it should
   * generally not be exposed to the application. */
  const header_type* unused_;

  /** Number of words in the region covered by erased records.
   * This includes the header word as well as record content. */
  unsigned int erased_;
};

} // namespace utility
} // namespace nrfcxx

#endif /* NRFCXX_UTILITY_HPP */
