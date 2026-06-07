/*
 * ============================================================
 *  LEON3 + GRETH Ethernet Verification Test — FIXED
 *  Design  : GR-XC3S-1500
 *  Tool    : VCS (Linux)
 *  Compiler: sparc-gaisler-elf-gcc
 *
 *  FIXES IN THIS VERSION:
 *
 *  FIX 1 — Separate TX and RX enable to avoid AHB bus conflict.
 *    Previous code enabled CTRL_TXEN|CTRL_RXEN together and then
 *    re-wrote both inside the TX IRQ handler. This caused GRETH's
 *    TX and RX DMA engines to compete for the same AHB bus at the
 *    same time — resulting in AHB errors on frame 2 onwards.
 *    Fix: Enable only CTRL_TXEN first, wait for ALL TX to finish,
 *    then enable only CTRL_RXEN for RX. The loopback frames
 *    accumulate in the PHY/MAC receive FIFO while TX runs and
 *    are drained when RX is enabled after.
 *
 *  FIX 2 — Re-assert only the relevant enable bit on each IRQ.
 *    When STS_TXIRQ fires, only re-write CTRL_TXEN (not RXEN).
 *    When STS_RXIRQ fires, only re-write CTRL_RXEN (not TXEN).
 *    This stops each engine from interfering with the other.
 *
 *  FIX 3 — Break immediately on AHB errors, not on STS_ERRORS.
 *    Writing STS_ERRORS to STATUS clears the error bits but
 *    GRETH has already disabled itself — TXIRQ/RXIRQ never fires
 *    so the poll loop hangs for TIMEOUT*NUM_FRAMES cycles.
 *    Fix: check each error bit individually and break instantly
 *    with a descriptive print so we know exactly what failed.
 *
 *  FIX 4 — tx_ok flag set correctly.
 *    Previously tx_ok=1 was set inside if(STS_TXIRQ) which only
 *    fires once per frame — not reliable for multi-frame.
 *    tx_ok is now only set when ALL descriptor EN bits are clear.
 *    Same fix applied to rx_ok.
 *
 *  FIX 5 — Test 1 does not write CTRL_TXEN|CTRL_RXEN.
 *    Without descriptors GRETH self-clears TE/RE immediately.
 *    Test 1 only checks register access so only CTRL_FULLD
 *    is written to verify bit4 sticks.
 *
 *  FIX 6 — RX descriptor ctrl has no length field.
 *    PDF: LENGTH in RX descriptor word 0 is written BY GRETH
 *    after reception to show bytes received. It should be 0
 *    when we set up the descriptor — we do not pre-fill it.
 * ============================================================
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define REPORT_START  do { *((volatile int *)0x80000200) = 0; } while(0)
#define REPORT_END    do { *((volatile int *)0x80000200) = 1; } while(0)

#define GRETH_BASE    0x80000D00
#define GRETH_CTRL    (GRETH_BASE + 0x00)
#define GRETH_STATUS  (GRETH_BASE + 0x04)
#define GRETH_MACMSB  (GRETH_BASE + 0x08)
#define GRETH_MACLSB  (GRETH_BASE + 0x0C)
#define GRETH_MDIO    (GRETH_BASE + 0x10)
#define GRETH_TXDESC  (GRETH_BASE + 0x14)
#define GRETH_RXDESC  (GRETH_BASE + 0x18)
#define GRETH_EDCLIP  (GRETH_BASE + 0x1C)

/* CTRL bits — PDF Table 797 */
#define CTRL_TXEN     (1 << 0)
#define CTRL_RXEN     (1 << 1)
#define CTRL_FULLD    (1 << 4)
#define CTRL_PROM     (1 << 5)
#define CTRL_RESET    (1 << 6)

/* STATUS bits — PDF Table 798 */
#define STS_RXERR     (1 << 0)
#define STS_TXERR     (1 << 1)
#define STS_RXIRQ     (1 << 2)
#define STS_TXIRQ     (1 << 3)
#define STS_RXAHB     (1 << 4)
#define STS_TXAHB     (1 << 5)

/* MDIO bits — PDF Table 801 */
#define MDIO_BUSY     (1 << 3)
#define MDIO_LINKFAIL (1 << 2)
#define MDIO_READ     (1 << 1)
#define MDIO_WRITE    (1 << 0)
#define MDIO_PHYSHIFT 11
#define MDIO_REGSHIFT 6

/* Descriptor bits — PDF Table 787/789 */
#define DESC_EN       (1 << 11)
#define DESC_WRAP     (1 << 12)

/* PHY */
#define PHY_ADDR          0
#define PHY_CTRL          0x00
#define PHY_STATUS        0x01
#define PHY_ID1           0x02
#define PHY_ID2           0x03
#define PHY_CTRL_RESET    (1 << 15)
#define PHY_CTRL_LOOPBACK (1 << 14)
#define PHY_CTRL_100MB    (1 << 13)
#define PHY_CTRL_AUTONEG  (1 << 12)
#define PHY_CTRL_FULLD    (1 << 8)

#define FRAME_SIZE    64
#define NUM_FRAMES    4
#define TIMEOUT       500000

#define RD(addr)      (*((volatile unsigned int *)(addr)))
#define WR(addr,val)  (*((volatile unsigned int *)(addr)) = (unsigned int)(val))

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
    if (mdio_wait() < 0) return 0xDEAD;
    tmp = (phy << MDIO_PHYSHIFT) |
          ((reg & 0x1F) << MDIO_REGSHIFT) |
          MDIO_READ;
    WR(GRETH_MDIO, tmp);
    if (mdio_wait() < 0) return 0xDEAD;
    tmp = RD(GRETH_MDIO);
    if (tmp & MDIO_LINKFAIL) return 0xDEAD;
    return (tmp >> 16) & 0xFFFF;
}

static void mdio_write(int phy, int reg, unsigned int val)
{
    unsigned int tmp;
    mdio_wait();
    tmp = ((val & 0xFFFF) << 16) |
          (phy  << MDIO_PHYSHIFT)  |
          ((reg & 0x1F) << MDIO_REGSHIFT) |
          MDIO_WRITE;
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
 * TEST 1: Register Access & MAC Address
 * FIX 5: No CTRL_TXEN/RXEN — self-clear without descriptors.
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

    /* FIX 5: Only FULLD — no TE/RE without descriptors */
    WR(GRETH_CTRL, CTRL_FULLD);
    v = RD(GRETH_CTRL);
    check("Full-duplex bit set", v & CTRL_FULLD);
}

/* ============================================================
 * TEST 2: PHY MDIO Read
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

    check("PHY ID1 not 0xFFFF", id1 != 0xFFFF);
    check("PHY ID2 not 0xFFFF", id2 != 0xFFFF);
    check("PHY STATUS readable", sts != 0xDEAD);
    check("No MDIO link failure", !(RD(GRETH_MDIO) & MDIO_LINKFAIL));
}

/* ============================================================
 * TEST 3: PHY Loopback Mode
 * ============================================================ */
static void test_phy_loopback_mode(void)
{
    unsigned int ctrl;
    printf("\n--- TEST 3: PHY Loopback Mode ---\n");

    mdio_write(PHY_ADDR, PHY_CTRL,
               PHY_CTRL_LOOPBACK | PHY_CTRL_100MB | PHY_CTRL_FULLD);
    while (RD(GRETH_MDIO) & MDIO_BUSY);

    ctrl = mdio_read(PHY_ADDR, PHY_CTRL);
    printf("    PHY CTRL after write = 0x%04X\n", ctrl);
    check("PHY loopback bit confirmed via readback",
          ctrl & PHY_CTRL_LOOPBACK);
}

/* ============================================================
 * TEST 4: Single Frame TX → Loopback → RX → Verify
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

    /* FIX 6: RX descriptor — no length, GRETH fills it in */
    rxd[0].addr = (unsigned int)rxbuf[0];
    rxd[0].ctrl = DESC_EN | DESC_WRAP;

    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);
    WR(GRETH_STATUS, 0xFF);

    /* FIX 1: Enable TX only first */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_FULLD | CTRL_PROM);

    int tx_ok = 0;
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_TXIRQ)  { tx_ok = 1; break; }
        if (sts & STS_TXAHB)  {
            printf("    TX AHB error (STATUS=0x%08X)\n", sts);
            break;
        }
        if (sts & STS_TXERR)  { break; }
    }
    WR(GRETH_STATUS, STS_TXIRQ | STS_TXERR | STS_TXAHB);
    check("Single frame TX completed", tx_ok);
    check("No TX errors", !(sts & (STS_TXERR|STS_TXAHB)));

    /* FIX 1: Now enable RX only */
    WR(GRETH_CTRL, CTRL_RXEN | CTRL_FULLD | CTRL_PROM);

    int rx_ok = 0;
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_RXIRQ)  { rx_ok = 1; break; }
        if (sts & STS_RXAHB)  {
            printf("    RX AHB error (STATUS=0x%08X)\n", sts);
            break;
        }
        if (sts & STS_RXERR)  { break; }
    }
    WR(GRETH_STATUS, STS_RXIRQ | STS_RXERR | STS_RXAHB);
    check("Single frame RX completed", rx_ok);
    check("No RX errors", !(sts & (STS_RXERR|STS_RXAHB)));

    int data_ok = 1;
    if (rx_ok) {
        for (i = 14; i < FRAME_SIZE; i++) {
            if (rxbuf[0][i] != txbuf[0][i]) {
                printf("    Byte %d mismatch TX=0x%02X RX=0x%02X\n",
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
 *
 * FIX 1: TX phase uses CTRL_TXEN only.
 *         RX phase uses CTRL_RXEN only.
 *         This prevents TX/RX DMA from fighting over AHB bus.
 *
 * FIX 2: Re-assert only the relevant enable bit per IRQ.
 *
 * FIX 3: Break immediately on AHB error with a print.
 *
 * FIX 4: tx_ok/rx_ok only set when ALL descriptors are done.
 * ============================================================ */
static void test_multi_frame_loopback(void)
{
    int i, j;
    unsigned int sts = 0;
    printf("\n--- TEST 5: Multi-Frame Stress Loopback (%d frames) ---\n",
           NUM_FRAMES);

    /* Print buffer addresses to confirm they are in AHB-reachable SRAM */
    printf("    txd  @ 0x%08X\n", (unsigned int)txd);
    printf("    rxd  @ 0x%08X\n", (unsigned int)rxd);
    printf("    txbuf@ 0x%08X\n", (unsigned int)txbuf);
    printf("    rxbuf@ 0x%08X\n", (unsigned int)rxbuf);

    /* Clear descriptor memory */
    memset(txd, 0, sizeof(txd));
    memset(rxd, 0, sizeof(rxd));

    /* Set up all descriptors */
    for (i = 0; i < NUM_FRAMES; i++) {
        fill_eth_header(txbuf[i], i);
        memset(rxbuf[i], 0, FRAME_SIZE);

        txd[i].addr = (unsigned int)txbuf[i];
        txd[i].ctrl = DESC_EN | FRAME_SIZE;

        /* FIX 6: RX descriptor — no length field */
        rxd[i].addr = (unsigned int)rxbuf[i];
        rxd[i].ctrl = DESC_EN;

        if (i == NUM_FRAMES - 1) {
            txd[i].ctrl |= DESC_WRAP;
            rxd[i].ctrl |= DESC_WRAP;
        }
    }

    /* Write descriptor pointers after memset */
    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);
    WR(GRETH_STATUS, 0xFF);

    /* --------------------------------------------------------
     * TX PHASE — CTRL_TXEN only, no RXEN
     * FIX 1: Only TX DMA runs, no AHB contention with RX DMA
     * -------------------------------------------------------- */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_FULLD | CTRL_PROM);

    int tx_ok = 0;
    for (i = 0; i < TIMEOUT * NUM_FRAMES; i++) {
        sts = RD(GRETH_STATUS);

        if (sts & STS_TXIRQ) {
            WR(GRETH_STATUS, STS_TXIRQ);
            /* FIX 2: re-assert only TXEN so GRETH continues */
            WR(GRETH_CTRL, CTRL_TXEN | CTRL_FULLD | CTRL_PROM);
        }

        /* FIX 3: break immediately on AHB error */
        if (sts & STS_TXAHB) {
            printf("    TX AHB error at frame %d (STATUS=0x%08X)\n",
                   i, sts);
            WR(GRETH_STATUS, STS_TXAHB);
            break;
        }
        if (sts & STS_TXERR) {
            printf("    TX error (STATUS=0x%08X)\n", sts);
            WR(GRETH_STATUS, STS_TXERR);
            break;
        }

        /* FIX 4: tx_ok only when ALL EN bits cleared */
        int all_done = 1;
        for (j = 0; j < NUM_FRAMES; j++)
            if (txd[j].ctrl & DESC_EN) { all_done = 0; break; }
        if (all_done) { tx_ok = 1; break; }
    }

    /* Print each TX descriptor status for debug */
    for (j = 0; j < NUM_FRAMES; j++)
        printf("    txd[%d].ctrl = 0x%08X\n", j, txd[j].ctrl);

    check("All TX frames sent",      tx_ok);
    check("No TX errors (stress)", !(sts & (STS_TXERR|STS_TXAHB)));

    /* --------------------------------------------------------
     * RX PHASE — CTRL_RXEN only, no TXEN
     * FIX 1: Loopback frames waiting in RX FIFO are now drained.
     * Only RX DMA runs — no AHB contention with TX DMA.
     * -------------------------------------------------------- */
    WR(GRETH_STATUS, 0xFF);
    WR(GRETH_CTRL, CTRL_RXEN | CTRL_FULLD | CTRL_PROM);

    int rx_ok = 0;
    for (i = 0; i < TIMEOUT * NUM_FRAMES; i++) {
        sts = RD(GRETH_STATUS);

        if (sts & STS_RXIRQ) {
            WR(GRETH_STATUS, STS_RXIRQ);
            /* FIX 2: re-assert only RXEN so GRETH continues */
            WR(GRETH_CTRL, CTRL_RXEN | CTRL_FULLD | CTRL_PROM);
        }

        /* FIX 3: break immediately on AHB error */
        if (sts & STS_RXAHB) {
            printf("    RX AHB error at frame %d (STATUS=0x%08X)\n",
                   i, sts);
            WR(GRETH_STATUS, STS_RXAHB);
            break;
        }
        if (sts & STS_RXERR) {
            printf("    RX error (STATUS=0x%08X)\n", sts);
            WR(GRETH_STATUS, STS_RXERR);
            break;
        }

        /* FIX 4: rx_ok only when ALL EN bits cleared */
        int all_done = 1;
        for (j = 0; j < NUM_FRAMES; j++)
            if (rxd[j].ctrl & DESC_EN) { all_done = 0; break; }
        if (all_done) { rx_ok = 1; break; }
    }

    /* Print each RX descriptor status for debug */
    for (j = 0; j < NUM_FRAMES; j++)
        printf("    rxd[%d].ctrl = 0x%08X  (len=%d bytes)\n",
               j, rxd[j].ctrl, rxd[j].ctrl & 0x7FF);

    check("All RX frames received",  rx_ok);
    check("No RX errors (stress)", !(sts & (STS_RXERR|STS_RXAHB)));

    /* Verify all frames byte by byte */
    int all_match = 1;
    for (i = 0; i < NUM_FRAMES; i++) {
        if (rxd[i].ctrl & DESC_EN) {
            printf("    Frame %d still owned by HW\n", i);
            all_match = 0;
            continue;
        }
        for (j = 14; j < FRAME_SIZE; j++) {
            if (rxbuf[i][j] != txbuf[i][j]) {
                printf("    Frame %d byte %d mismatch "
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
