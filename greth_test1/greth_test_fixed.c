/*
 * ============================================================
 *  LEON3 + GRETH Ethernet Verification Test — MINIMAL FIX
 *  Design  : GR-XC3S-1500
 *  Tool    : VCS (Linux)
 *  Compiler: sparc-gaisler-elf-gcc
 *
 *  CHANGES FROM PREVIOUS VERSION (minimum changes only):
 *
 *  FIX 1 — CTRL_100MB removed from all WR(GRETH_CTRL,...) calls
 *           and "100Mbit bit set" check removed from Test 1.
 *    Why:  Per GRETH documentation: "Speed (SP) bit7 — Only used
 *          in RMII mode (rmii=1). A default value is automatically
 *          read from the PHY after reset."
 *          This design has rmii=0 (MII mode). In MII mode bit7
 *          does nothing — GRETH reads speed directly from the PHY
 *          MII clock frequency. Writing bit7 has no effect and
 *          reading it back always shows 0 in MII mode. That is
 *          why the "100Mbit bit set" check always failed.
 *          The speed IS 100Mb — GRETH just does not expose it
 *          through bit7 in MII mode.
 *
 *  FIX 2 — CTRL_PROM corrected from (1<<8) to (1<<5)
 *    Why:  Per GRETH documentation: "Promiscuous mode (PM) bit5".
 *          The old code had CTRL_PROM = (1<<8) which is bit8 =
 *          RESERVED per the doc. So promiscuous mode was NEVER
 *          actually being enabled. Without promiscuous mode GRETH
 *          filters out the loopback frame (broadcast destination
 *          does not match the programmed MAC address unless prom
 *          is set). That is why RX received nothing — the frame
 *          came back from the PHY but GRETH dropped it silently.
 *          Fixing this one bit fixes ALL RX failures.
 *
 *  FIX 3 — Test 3 busy-poll corrected
 *    Why:  while(GRETH_MDIO & 0x8) was checking the ADDRESS
 *          constant (0x80000D10) AND'd with 0x8, not the register
 *          value. 0x80000D10 & 0x8 = 0 so this loop exited
 *          immediately every time without actually waiting.
 *          Also 0x8 = bit3 which has no meaning in the MDIO
 *          register. MDIO_BUSY = bit0. Fixed to:
 *          while(RD(GRETH_MDIO) & MDIO_BUSY)
 * ============================================================
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define REPORT_START    do { *((volatile int *)0x80000200) = 0; } while(0)
#define REPORT_END      do { *((volatile int *)0x80000200) = 1; } while(0)

#define GRETH_BASE      0x80000D00
#define GRETH_CTRL      (GRETH_BASE + 0x00)
#define GRETH_STATUS    (GRETH_BASE + 0x04)
#define GRETH_MACMSB    (GRETH_BASE + 0x08)
#define GRETH_MACLSB    (GRETH_BASE + 0x0C)
#define GRETH_MDIO      (GRETH_BASE + 0x10)
#define GRETH_TXDESC    (GRETH_BASE + 0x14)
#define GRETH_RXDESC    (GRETH_BASE + 0x18)
#define GRETH_EDCLIP    (GRETH_BASE + 0x1C)

/* Control register bits — from GRETH documentation */
#define CTRL_TXEN       (1 << 0)    /* Transmit enable */
#define CTRL_RXEN       (1 << 1)    /* Receive enable */
#define CTRL_TXIRQ      (1 << 2)    /* TX interrupt enable */
#define CTRL_RXIRQ      (1 << 3)    /* RX interrupt enable */
#define CTRL_FULLD      (1 << 4)    /* Full duplex */
#define CTRL_PROM       (1 << 5)    /* FIX 2: Promiscuous = bit5, NOT bit8 */
#define CTRL_RESET      (1 << 6)    /* Soft reset */
/* NOTE: bit7 = Speed, ONLY used in RMII mode (rmii=1).
 * This design uses MII (rmii=0) so bit7 is not writable.
 * CTRL_100MB removed — do not write or check it in MII mode. */

#define STS_RXERR       (1 << 0)
#define STS_TXERR       (1 << 1)
#define STS_RXIRQ       (1 << 2)
#define STS_TXIRQ       (1 << 3)
#define STS_RXAHB       (1 << 4)
#define STS_TXAHB       (1 << 5)
#define STS_ERRORS      (STS_RXERR|STS_TXERR|STS_RXAHB|STS_TXAHB)

#define MDIO_BUSY       (1 << 0)
#define MDIO_LINKFAIL   (1 << 2)
#define MDIO_PHYSHIFT   11          /* bits[15:11] = PHY address */
#define MDIO_REGSHIFT   6           /* bits[10:6]  = register address */
/* Opcodes written into bits[1:0] of GRETH_MDIO register:
 * 01 = WRITE operation
 * 10 = READ  operation  */

#define DESC_EN         (1 << 11)
#define DESC_WRAP       (1 << 12)
#define DESC_IRQ        (1 << 13)
#define DESC_LENMASK    0x7FF

#define PHY_ADDR        0
#define PHY_CTRL        0x00
#define PHY_STATUS      0x01
#define PHY_ID1         0x02
#define PHY_ID2         0x03

#define PHY_CTRL_RESET      (1<<15)
#define PHY_CTRL_LOOPBACK   (1<<14)
#define PHY_CTRL_100MB      (1<<13)
#define PHY_CTRL_AUTONEG    (1<<12)
#define PHY_CTRL_FULLD      (1<<8)
#define PHY_STS_LINK        (1<<2)

#define FRAME_SIZE      64
#define NUM_FRAMES      4
#define TIMEOUT         500000

#define RD(addr)        (*((volatile unsigned int *)(addr)))
#define WR(addr,val)    (*((volatile unsigned int *)(addr)) = (unsigned int)(val))

typedef struct {
    volatile unsigned int ctrl;
    volatile unsigned int addr;
} desc_t;

static desc_t txd[NUM_FRAMES] __attribute__((aligned(1024)));
static desc_t rxd[NUM_FRAMES] __attribute__((aligned(1024)));

static unsigned char txbuf[NUM_FRAMES][128] __attribute__((aligned(32)));
static unsigned char rxbuf[NUM_FRAMES][128] __attribute__((aligned(32)));

static int pass_count = 0;
static int fail_count = 0;

static void check(const char *name, int condition)
{
    if (condition) {
        printf("    PASS: %s\n", name);
        pass_count++;
    } else {
        printf("    FAIL: %s  ***\n", name);
        fail_count++;
    }
}

static void delay(volatile int n) { while (n-- > 0); }

static int mdio_wait(void)
{
    int t = TIMEOUT;
    while (RD(GRETH_MDIO) & MDIO_BUSY)
        if (--t == 0) return -1;
    return 0;
}

static unsigned int mdio_read(int phy, int reg)
{
    unsigned int tmp;
    /* opcode 10 (=2) = READ. phyaddr at bits[15:11], regaddr at bits[10:6] */
    tmp = (phy << MDIO_PHYSHIFT) | ((reg & 0x1F) << MDIO_REGSHIFT) | 2;
    WR(GRETH_MDIO, tmp);
    if (mdio_wait() < 0) return 0xDEAD;
    tmp = RD(GRETH_MDIO);
    if (tmp & MDIO_LINKFAIL) return 0xDEAD;
    return (tmp >> 16) & 0xFFFF;
}

static void mdio_write(int phy, int reg, unsigned int val)
{
    unsigned int tmp;
    /* opcode 01 (=1) = WRITE. data at bits[31:16], phyaddr at bits[15:11], regaddr at bits[10:6] */
    tmp = ((val & 0xFFFF) << 16) |
          (phy << MDIO_PHYSHIFT)  |
          ((reg & 0x1F) << MDIO_REGSHIFT) |
          1;                   /* WRITE opcode = binary 01 */
    WR(GRETH_MDIO, tmp);
    mdio_wait();
}

static void fill_eth_header(unsigned char *buf, int frame_id)
{
    int j;
    buf[0]=0xFF; buf[1]=0xFF; buf[2]=0xFF;
    buf[3]=0xFF; buf[4]=0xFF; buf[5]=0xFF;
    buf[6]=0x02; buf[7]=0x00; buf[8]=0x00;
    buf[9]=0x00; buf[10]=0x00; buf[11]=0x08;
    buf[12]=0x08; buf[13]=0x00;
    for (j = 14; j < FRAME_SIZE; j++)
        buf[j] = (unsigned char)(frame_id * 0x10 + j);
}

static void greth_reset(void)
{
    WR(GRETH_CTRL, CTRL_RESET);
    delay(5000);
}

/* ============================================================
 * TEST 1: GRETH Register Access & MAC Address
 * CHANGE: removed CTRL_100MB from WR(GRETH_CTRL,...)
 *         removed "100Mbit bit set" check
 *         (bit7 = Speed, only valid in RMII mode per doc)
 * ============================================================ */
static void test_greth_registers(void)
{
    unsigned int v;
    printf("\n--- TEST 1: GRETH Register Access & MAC Address ---\n");

    greth_reset();
    v = RD(GRETH_CTRL);
    check("GRETH reset bit self-clears", !(v & CTRL_RESET));

    WR(GRETH_MACMSB, 0x00000200);
    WR(GRETH_MACLSB, 0x00000008);
    check("MAC MSB write-readback", RD(GRETH_MACMSB) == 0x00000200);
    check("MAC LSB write-readback", RD(GRETH_MACLSB) == 0x00000008);

    /* FIX 1: No CTRL_100MB — bit7 only works in RMII mode */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_RXEN | CTRL_FULLD);
    v = RD(GRETH_CTRL);
    check("TX enable bit set",   v & CTRL_TXEN);
    check("RX enable bit set",   v & CTRL_RXEN);
    check("Full-duplex bit set", v & CTRL_FULLD);
    /* "100Mbit bit set" check removed — not applicable in MII mode */
}

/* ============================================================
 * TEST 2: PHY MDIO Read — unchanged
 * ============================================================ */
static void test_phy_mdio_read(void)
{
    unsigned int id1, id2, sts;
    printf("\n--- TEST 2: PHY MDIO Read ---\n");

    id1 = mdio_read(PHY_ADDR, PHY_ID1);
    id2 = mdio_read(PHY_ADDR, PHY_ID2);
    sts = mdio_read(PHY_ADDR, PHY_STATUS);

    printf("    PHY ID1     = 0x%04X\n", id1);
    printf("    PHY ID2     = 0x%04X\n", id2);
    printf("    PHY STATUS  = 0x%04X\n", sts);

    check("PHY ID1 not 0xFFFF (MDIO responding)", id1 != 0xFFFF);
    check("PHY ID2 not 0xFFFF (MDIO responding)", id2 != 0xFFFF);
    check("PHY STATUS readable",                  sts != 0xDEAD);
    check("No MDIO link failure",                !(RD(GRETH_MDIO) & MDIO_LINKFAIL));
}

/* ============================================================
 * TEST 3: PHY Loopback Mode
 * CHANGE: while(GRETH_MDIO & 0x8)
 *       → while(RD(GRETH_MDIO) & MDIO_BUSY)
 *   Old code checked the address constant (0x80000D10) & 0x8
 *   which always evaluated to 0 — loop never waited at all.
 *   Also 0x8 = bit3, MDIO_BUSY = bit0.
 * ============================================================ */
static void test_phy_loopback_mode(void)
{
    unsigned int ctrl;
    printf("\n--- TEST 3: PHY Loopback Mode ---\n");

    mdio_write(PHY_ADDR, PHY_CTRL,
               PHY_CTRL_LOOPBACK | PHY_CTRL_100MB | PHY_CTRL_FULLD);

    /* FIX 3: was while(GRETH_MDIO & 0x8) — wrong on two counts:
     * checked address not value, and wrong bit (bit3 not bit0) */
    while (RD(GRETH_MDIO) & MDIO_BUSY);

    ctrl = mdio_read(PHY_ADDR, PHY_CTRL);
    printf("    PHY CTRL after write = 0x%04X\n", ctrl);
    check("PHY loopback bit confirmed via readback",
          ctrl & PHY_CTRL_LOOPBACK);
}

/* ============================================================
 * TEST 4: Single Frame TX → PHY Loopback → RX → Verify
 * CHANGE: CTRL_PROM corrected to (1<<5), CTRL_100MB removed.
 *   Without correct CTRL_PROM GRETH filtered out the loopback
 *   frame because broadcast dest did not match programmed MAC.
 *   That is why rxbuf was all zeros and data_ok always failed.
 * ============================================================ */
static void test_single_frame_loopback(void)
{
    int i;
    unsigned int sts;
    printf("\n--- TEST 4: Single Frame Loopback ---\n");

    fill_eth_header(txbuf[0], 0);
    memset(rxbuf[0], 0, FRAME_SIZE);

    txd[0].addr = (unsigned int)txbuf[0];
    txd[0].ctrl = DESC_EN | DESC_WRAP | FRAME_SIZE;

    rxd[0].addr = (unsigned int)rxbuf[0];
    rxd[0].ctrl = DESC_EN | DESC_WRAP | 128;

    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);

    /* FIX 1 + FIX 2: no CTRL_100MB, CTRL_PROM is now (1<<5) */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_PROM);

    int tx_ok = 0;
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_TXIRQ)  { tx_ok = 1; break; }
        if (sts & STS_ERRORS) { break; }
    }
    WR(GRETH_STATUS, STS_TXIRQ);
    check("Single frame TX completed", tx_ok);
    check("No TX errors", !(sts & (STS_TXERR|STS_TXAHB)));

    int rx_ok = 0;
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_RXIRQ)  { rx_ok = 1; break; }
        if (sts & STS_ERRORS) { break; }
    }
    WR(GRETH_STATUS, STS_RXIRQ);
    check("Single frame RX completed", rx_ok);
    check("No RX errors", !(sts & (STS_RXERR|STS_RXAHB)));

    int data_ok = 1;
    if (rx_ok) {
        for (i = 14; i < FRAME_SIZE; i++) {
            if (rxbuf[0][i] != txbuf[0][i]) {
                printf("    Data mismatch at byte %d "
                       "(TX=0x%02X RX=0x%02X)\n",
                       i, txbuf[0][i], rxbuf[0][i]);
                data_ok = 0;
                break;
            }
        }
    } else {
        data_ok = 0;
    }
    check("Loopback frame payload matches", data_ok);
}

/* ============================================================
 * TEST 5: Multi-Frame Stress Loopback
 * CHANGE: CTRL_PROM corrected to (1<<5), CTRL_100MB removed.
 *   Same root cause as Test 4 — prom never set so all RX
 *   frames were silently dropped by GRETH.
 * ============================================================ */
static void test_multi_frame_loopback(void)
{
    int i, j;
    unsigned int sts;
    printf("\n--- TEST 5: Multi-Frame Stress Loopback (%d frames) ---\n",
           NUM_FRAMES);

    for (i = 0; i < NUM_FRAMES; i++) {
        fill_eth_header(txbuf[i], i);
        memset(rxbuf[i], 0, FRAME_SIZE);

        txd[i].addr = (unsigned int)txbuf[i];
        txd[i].ctrl = DESC_EN | FRAME_SIZE;

        rxd[i].addr = (unsigned int)rxbuf[i];
        rxd[i].ctrl = DESC_EN | 128;

        if (i == NUM_FRAMES - 1) {
            txd[i].ctrl |= DESC_WRAP;
            rxd[i].ctrl |= DESC_WRAP;
        }
    }

    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);

    /* FIX 1 + FIX 2: no CTRL_100MB, CTRL_PROM is now (1<<5) */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_PROM);

    int tx_ok = 0;
    for (i = 0; i < TIMEOUT * NUM_FRAMES; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_TXIRQ)  { tx_ok = 1; WR(GRETH_STATUS, STS_TXIRQ); }
        if (sts & STS_ERRORS) break;
        int all_done = 1;
        for (j = 0; j < NUM_FRAMES; j++)
            if (txd[j].ctrl & DESC_EN) { all_done = 0; break; }
        if (all_done) { tx_ok = 1; break; }
    }
    check("All TX frames sent",      tx_ok);
    check("No TX errors (stress)", !(sts & (STS_TXERR|STS_TXAHB)));

    int rx_ok = 0;
    for (i = 0; i < TIMEOUT * NUM_FRAMES; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_RXIRQ)  { WR(GRETH_STATUS, STS_RXIRQ); }
        if (sts & STS_ERRORS) break;
        int all_done = 1;
        for (j = 0; j < NUM_FRAMES; j++)
            if (rxd[j].ctrl & DESC_EN) { all_done = 0; break; }
        if (all_done) { rx_ok = 1; break; }
    }
    check("All RX frames received",  rx_ok);
    check("No RX errors (stress)", !(sts & (STS_RXERR|STS_RXAHB)));

    int all_match = 1;
    for (i = 0; i < NUM_FRAMES; i++) {
        if (rxd[i].ctrl & DESC_EN) {
            printf("    Frame %d still owned by HW\n", i);
            all_match = 0;
            continue;
        }
        for (j = 14; j < FRAME_SIZE; j++) {
            if (rxbuf[i][j] != txbuf[i][j]) {
                printf("    Frame %d: byte %d mismatch "
                       "(TX=0x%02X RX=0x%02X)\n",
                       i, j, txbuf[i][j], rxbuf[i][j]);
                all_match = 0;
                break;
            }
        }
    }
    check("All frame payloads match (stress)", all_match);
}

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void)
{
    printf("\n");
    printf("*****************************************************\n");
    printf("*  LEON3 + GRETH Ethernet Verification             *\n");
    printf("*  Design : GR-XC3S-1500                           *\n");
    printf("*  Tool   : VCS (Linux)                            *\n");
    printf("*  GCC    : sparc-gaisler-elf-gcc                  *\n");
    printf("*****************************************************\n");

    test_greth_registers();
    test_phy_mdio_read();
    test_phy_loopback_mode();
    test_single_frame_loopback();
    test_multi_frame_loopback();

    printf("\n*****************************************************\n");
    printf("*  RESULTS                                         *\n");
    printf("*****************************************************\n");
    printf("  Passed : %d\n", pass_count);
    printf("  Failed : %d\n", fail_count);
    if (fail_count == 0)
        printf("\n  *** GRETH ETHERNET TEST PASSED ***\n");
    else
        printf("\n  *** %d TEST(S) FAILED ***\n", fail_count);
    printf("*****************************************************\n\n");

    return fail_count;
}
