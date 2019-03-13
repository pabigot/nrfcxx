// SPDX-License-Identifier: CC-BY-SA-4.0
// Copyright 2019 Peter A. Bigot

/** Extract the Serial Flash Discoverable Parameter block from a
 * jedec,spi-nor device connected to SPI2/SPIM2/QSPI.
 *
 * Also tests deep power down and release from DPD. */

#include <nrfcxx/periph.hpp>
#include <nrfcxx/clock.hpp>
#include <nrfcxx/gpio.hpp>
#include <nrfcxx/utility.hpp>
#include <cstdio>
#include <cstring>

#if 1
#include <nrfcxx/console/cstdio.hpp>
#else
#include <nrfcxx/console/null.hpp>
#endif

#define RELEASE_DP 1
#define DISPLAY_SFDP 1
#define POWERDOWN_DELAY_s 0

#if (NRFCXX_BOARD_IS_XENON - 0)
#define SPINOR_CSn NRFCXX_BOARD_PSEL_EFL_CSn
#elif (NRFCXX_BOARD_IS_PCA10056 - 0)
#define SPINOR_CSn NRFCXX_BOARD_PSEL_EFL_CSn
#endif

namespace {

struct qspi_cfg
{
  uint32_t ifconfig0 = 0;
  uint32_t ifconfig1 = 0;
};

struct parameter_header_type
{
  uint8_t id_lsb_;
  uint8_t rev_minor;
  uint8_t rev_major;
  uint8_t len_dw;
  uint8_t ptp_[3];
  uint8_t id_msb_;

  uint16_t id () const
  {
    return id_msb_ << 8 | id_lsb_;
  }

  uint32_t ptp () const
  {
    return (ptp_[2] << 16) | (ptp_[1] << 8) | ptp_[0];
  }
} __attribute__((__packed__));

/* All fields are little-endian. */
struct sfdp_header_type
{
  union {
    uint8_t u8[4];
    uint32_t u32;
  } sfdp;
  uint8_t rev_minor;
  uint8_t rev_major;
  uint8_t nph;
  uint8_t access_protocol;
  parameter_header_type ph[1];
} __attribute__((__packed__));

} // anonymous

int
main (void)
{
  using namespace nrfcxx;
  using clock::uptime;
  int rc;
  board::initialize();

  csetvbuf();
  cputs("\n\n" __FILE__ " " __DATE__ " " __TIME__);
  nrfcxx::periph::UART::instance().autoenable(1);

  auto& spi = periph::SPI::instance(2);
  rc = spi.bus_configure(NRFCXX_BOARD_PSEL_SPI2_SCK,
                         NRFCXX_BOARD_PSEL_SPI2_MOSI,
                         NRFCXX_BOARD_PSEL_SPI2_MISO,
                         SPI_FREQUENCY_FREQUENCY_M8,
                         spi.config_from_mode(0));

  cprintf("init %d for SPI2 SCK %u ; MOSI %u; MISO %u ; IO2 %u ; IO3 %u; CSn %u\n",
         rc,
         NRFCXX_BOARD_PSEL_SPI2_SCK,
         NRFCXX_BOARD_PSEL_SPI2_MOSI,
         NRFCXX_BOARD_PSEL_SPI2_MISO,
         NRFCXX_BOARD_PSEL_QSPI_IO2,
         NRFCXX_BOARD_PSEL_QSPI_IO3,
         SPINOR_CSn);

  auto csn = gpio::pin_reference::create(SPINOR_CSn);
  cprintf("PIN %u within %u mask %08lx\n", csn.local_psel, csn.global_psel, csn.local_bit);
  csn.set();
  csn.configure(gpio::PIN_CNF_WRONLY);

#if (RELEASE_DP - 0)
  /* Release any existing deep power down from a previous
   * invocation. */
  if (auto enabler = spi.scoped_enable()) {
    /* MX25L32 uses RDP=RES to wakeup.  MX25R64 only requires CS
     * asserted for tCRDP=20ns.  We'll send RDP which is much more
     * than 20 ns. */
    // cmd, 3-byte address, ID
    uint8_t buffer[5] = {0xAB};
    csn.clear();
    rc = spi.tx_rx(buffer, 1, 4, buffer);
    csn.set();
    /* MX25L32 requires CS held high t_RES2=100 us to complete a wakeup.
     * MX25R64 requires tRDP=35 us. */
    delay_us(100);
    cprintf("RDP got %02x: %d\n", buffer[4], rc);
  } else {
    cprintf("RDP failed: %d\n", enabler.result());
  }
#endif

  if (auto enabler = spi.scoped_enable()) {
    uint8_t cmd = 0x9F;
    uint8_t id[sizeof(cmd) + 3]{};
    csn.clear();
    rc = spi.tx_rx(&cmd, sizeof(cmd), sizeof(id) - sizeof(cmd), id);
    csn.set();
    if (0 > rc) {
      cprintf("Failed to read ID: %d\n", rc);
      return -1;
    }
    cprintf("JEDEC ID: %02x %02x %02x\n", id[1], id[2], id[3]);
  }

  if (auto enabler = spi.scoped_enable()) {
    uint8_t buf[2] = {0x05};
    csn.clear();
    rc = spi.tx_rx(buf, 1, 1, buf);
    csn.set();
    printf("SPI RDSR %02x: %d\n", buf[1], rc);
  } else {
    printf("SPI enable failed: %d\n", enabler.result());
  }

#if (DISPLAY_SFDP - 0)
  /* Fetch and display the serial flash discoverable parameters. */
  if (auto enabler = spi.scoped_enable()) {
    uint8_t id[512]{};
    uint8_t cmd = 0x5A;

    csn.clear();
    rc = spi.tx_rx(&cmd, sizeof(cmd), 4, nullptr);
    uint8_t* dp = id;
    if (5 != rc) {
      cprintf("Failed to transmit SFDP command: %d\n", rc);
      return -1;
    }
    rc = spi.tx_rx(nullptr, 0, 8, id);
    if (8 != rc) {
      cprintf("Failed to read SFDP hdr: %d\n", rc);
      return -1;
    }
    dp += rc;
    const sfdp_header_type*hp = reinterpret_cast<sfdp_header_type*>(id);
    unsigned int len = (1 + hp->nph) * sizeof(hp->ph[1]);
    rc = spi.tx_rx(nullptr, 0, len, dp);
    csn.set();
    if (len != static_cast<unsigned int>(rc)) {
      cprintf("Failed to read parameter headers: %d\n", rc);
      return -1;
    }
    dp += rc;

    cprintf("Raw SFDP through parameter headers:\n");
    utility::display_data(id, dp - id, 0);
    cprintf("HEADER: %08lx, rev %u.%u, access %02x, %u parameter blocks\n",
           hp->sfdp.u32, hp->rev_major, hp->rev_minor, hp->access_protocol,
           1 + hp->nph);
    for (auto phi = 0U; phi <= hp->nph; ++phi) {
      const parameter_header_type* php = hp->ph + phi;
      uint32_t record[php->len_dw];
      {
        cmd = 0x5A;
        csn.clear();
        rc = spi.tx_rx(&cmd, sizeof(cmd), 0, nullptr);
        if (sizeof(cmd) == rc) {
          /* Convert little-endian to big-endian */
          uint8_t addr[3] = {
            php->ptp_[2],
            php->ptp_[1],
            php->ptp_[0],
          };
          rc = spi.tx_rx(addr, sizeof(addr), 0, nullptr);
        }
        if (sizeof(php->ptp_) == rc) {
          rc = spi.tx_rx(&cmd, sizeof(cmd), 0, nullptr);
        }
        if (sizeof(cmd) == rc) {
          rc = spi.tx_rx(nullptr, 0, sizeof(record), reinterpret_cast<uint8_t*>(record));
        }
        csn.set();
        if (sizeof(record) != static_cast<size_t>(rc)) {
          cprintf("PH%u read failed: %d\n", phi, rc);
          return -1;
        }
      }
      cprintf("\nPH%u: id %04x, rev %u.%u, %u words at [%lx, %lx)\n",
             phi, php->id(), php->rev_major, php->rev_minor,
             php->len_dw, php->ptp(), php->ptp() + sizeof(record));
      utility::display_data(record, php->len_dw, php->ptp());

      if (0xFF00 == php->id()) {
        qspi_cfg qspi;

        cprintf("Basic Flash Parameters\n");
        /* Basic parameters. */
        cprintf("  %lu Mbit flash\n", record[1] >> 20);
        if (9 >= php->len_dw) {
          cprintf("  Legacy page 256 bytes\n");
        } else {
          const auto page_By = 1U << (0x0F & (record[10] >> 4));
          cprintf("  Page size %u bytes\n", page_By);
          if (512 == page_By) {
            qspi.ifconfig0 |= (QSPI_IFCONFIG0_PPSIZE_512Bytes << QSPI_IFCONFIG0_PPSIZE_Pos);
          }
        }
        uint8_t etsz = 0xFF & (record[7] >> 0);
        uint8_t etcmd = 0xFF & (record[7] >> 8);
        cprintf(" Erase type 1 cmd %02x size %u KiBy\n",
               etcmd, (1 << (etsz - 10)));
        etsz = 0xFF & (record[7] >> 16);
        if (0 != etsz) {
          etcmd = 0xFF & (record[7] >> 24);
          cprintf(" Erase type 2 cmd %02x size %u KiBy\n",
                 etcmd, (1 << (etsz - 10)));
        }
        etsz = 0xFF & (record[8] >> 0);
        if (0 != etsz) {
          etcmd = 0xFF & (record[8] >> 8);
          cprintf(" Erase type 3 cmd %02x size %u KiBy\n",
                 etcmd, (1 << (etsz - 10)));
        }
        etsz = 0xFF & (record[8] >> 16);
        if (0 != etsz) {
          etcmd = 0xFF & (record[8] >> 24);
          cprintf(" Erase type 4 cmd %02x size %u KiBy\n",
                 etcmd, (1 << (etsz - 10)));
        }

        /* Inferred QSPI configuration */

        /* Select fastest supported READOC based on BFP[0].  BFP
         * doesn't include PP support; assume that if READ is
         * supported so is PP. */
        // default: FASTREAD ~ 1-1-1 : 0x0B
        if ((1U << 21) & record[0]) {
          // 4IO ~ 1-4-4 : READ 0xEB , PP 0x38
          qspi.ifconfig0 |=
            (QSPI_IFCONFIG0_READOC_READ4IO << QSPI_IFCONFIG0_READOC_Pos)
            | (QSPI_IFCONFIG0_WRITEOC_PP4IO << QSPI_IFCONFIG0_WRITEOC_Pos)
            ;
        } else if ((1U << 22) & record[0]) {
          // 4O ~ 1-1-4 : READ 0x6B , PP 0x32
          qspi.ifconfig0 |=
            (QSPI_IFCONFIG0_READOC_READ4O << QSPI_IFCONFIG0_READOC_Pos)
            | (QSPI_IFCONFIG0_WRITEOC_PP4O << QSPI_IFCONFIG0_WRITEOC_Pos)
            ;
        } else if ((1U << 20) & record[0]) {
          // 2IO ~ 1-2-2 : READ 0xBB , PP 0xA2 (NB: This is PP2O, no PP2IO support)
          qspi.ifconfig0 |=
            (QSPI_IFCONFIG0_READOC_READ4IO << QSPI_IFCONFIG0_READOC_Pos)
            | (QSPI_IFCONFIG0_WRITEOC_PP2O << QSPI_IFCONFIG0_WRITEOC_Pos)
            ;
        } else if ((1U << 16) & record[0]) {
          // 2O ~ 1-1-2 : READ 0x3B , PP 0xA2
          qspi.ifconfig0 |=
            (QSPI_IFCONFIG0_READOC_READ4IO << QSPI_IFCONFIG0_READOC_Pos)
            | (QSPI_IFCONFIG0_WRITEOC_PP2O << QSPI_IFCONFIG0_WRITEOC_Pos)
            ;
        }
        /* Address byte support.  Assume 3-byte unless device requires 4-byte. */
        if (0x02 == (0x03 & (record[0] >> 17))) {
          qspi.ifconfig0 |= (QSPI_IFCONFIG0_ADDRMODE_32BIT << QSPI_IFCONFIG0_ADDRMODE_Pos);
        }

        /* More information available with JESD216A/B (16 words) and
         * JESD216C/D (20 words) */
        if (9 < php->len_dw) {
          /* DPM support.  Nordic doesn't provide configurable DPM
           * commands, and doesn't document what commands are used for
           * DPM, so assume they're the "standard" DP=0xB9; RDP=0xAB
           * and support the feature only if they're what's available.
           *
           * DPMDUR counts in units of 16 us, and the default register
           * value -1 corresponds to 1.048 s for both ENTER and EXIT,
           * which is hellalong.
           *
           * JESD216A+ uses four different units, and only provides
           * the EXIT duration.  Zero is 128 ns, with a maximum
           * duration of 4.096 us; the others are basically multipls
           * of 2^du us.
           *
           * Display the exit duration in us, but don't bother trying
           * to set DPM_DUR.  We can't read all the information, and
           * the MX25L32 is at rev 1.0 of JESD216 and doesn't even
           * provide this word.
           *
           * Further, MX25R64 supports DPM_DUR and provides a value
           * here (16 us), but it doesn't match the data sheet
           * (t_RDP=35 us) and the documented way of releasing DP
           * doesn't involve an RDP command.  So this is all pretty
           * much useless and we'll need to either hard-code the
           * DPMDUR value based on the JEDEC ID, or execute DP/RDP
           * with custom commands.  In either case we might as well
           * provide lookups for the other values too. */
          const uint8_t dur_units = 0x03 & (record[13] >> 13);
          const uint8_t dur_count = 0x1F & (record[13] >> 8);
          unsigned int dur_ns{(1000U * dur_count) << dur_units};
          if (0 == dur_units) {
            dur_ns = dur_count << 7;
          }
          printf("DPM %08lx %02lx %02lx ; dur %u %u = %u ns\n", record[13],
                 (0xFF & (record[13] >> 23)),
                 (0xFF & (record[13] >> 15)),
                 dur_units, dur_count, dur_ns);
          if ((!((1U << 31) & record[13]))
              && (0xB9 == (0xFF & (record[13] >> 23)))
              && (0xAB == (0xFF & (record[13] >> 15)))) { // RDP
            qspi.ifconfig0 |= (QSPI_IFCONFIG0_DPMENABLE_Enable << QSPI_IFCONFIG0_DPMENABLE_Pos);
          }
        }

        cprintf("QSPI: IFCONFIG0 %08" PRIx32 " ; IFCONFIG1 %08" PRIx32 "\n",
                qspi.ifconfig0, qspi.ifconfig1);
      } else if (0xFF84 == php->id()) {
        cprintf("4-byte Address Instruction Table\n");
      } else if (0xFFC2 == php->id()) {
        cprintf("GigaDevice/Macronix Flash Parameter Table\n");
      } else {
        cprintf("Undecoded parameter header %04x\n", php->id());
      }
    }
  } else {
    cprintf("SFDP failed: %d\n", enabler.result());
  }
#endif /* DISPLAY SFDP */

#if (POWERDOWN_DELAY_s - 0)
  cprintf("Sleeping %u s\n", POWERDOWN_DELAY_s);
  sleep_ms(POWERDOWN_DELAY_s * 1000);
  cputs("Powering down EFL");

  if (auto enabler = spi.scoped_enable()) {
    uint8_t cmd = 0xB9;
    csn.clear();
    rc = spi.tx_rx(&cmd, sizeof(cmd), 0, nullptr);
    csn.set();
    cprintf("Deep Power Down got %d\n", rc);
  } else {
    cprintf("DPD failed: %d\n", enabler.result());
  }
#endif

  while (true) {
    systemState::WFE();
  }

  return 0;
}
