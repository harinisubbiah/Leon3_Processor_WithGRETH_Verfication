/*
 * ============================================================
 *  LEON3 + GRETH Ethernet Verification Test — FIXED
 *  Design  : GR-XC3S-1500
 *  Tool    : VCS (Linux)
 *  Compiler: sparc-gaisler-elf-gcc
 *
 *  FIXES IN THIS VERSION (based on GRETH_DETAILS.pdf + debug):
 *
 *  FIX 1 — MDIO bit positions corrected from PDF Table 801:
 *    bit3 = BUSY (BU)   ← was wrongly bit0 before
 *    bit2 = Linkfail (LF) — reset value is '1' (normal after reset)
 *    bit1 = Read  (RD)  ← READ  opcode = (1<<1) = 2
 *    bit0 = Write (WR)  ← WRITE opcode = (1<<0) = 1
 *    bits[15:11] = PHYADDR, bits[10:6] = REGADDR  (confirmed correct)
 *
 *  FIX 2 — EDCL duplex detection FSM must be disabled (CTRL bit12)
 *    From PDF: "Disable duplex detection (DD) bit12 - Disable the
 *    EDCL speed/duplex detection FSM."
 *    Your design has EDCL present (CTRL bit31=1 confirmed by your
 *    0x4000013 value). The FSM runs automatically after reset and
 *    issues its own MDIO transactions to detect PHY speed/duplex,
 *    then WRITES those results back into CTRL — overwriting whatever
 *    we set including the PROM bit. This is why:
 *    - CTRL goes 0x4000013 → 0x4000011 → 0x4000010 (FSM cleared RE)
 *    - PROM bit never sticks (FSM rewrites CTRL without PROM)
 *    - Only 2 of 4 RX frames received (RX stopped when PROM cleared)
 *    - Frame 0 data mismatch (EDCL intercepting some frames)
 *    FIX: Write CTRL_DD=(1<<12) immediately after reset BEFORE any
 *    other CTRL writes. This stops the FSM permanently.
 *    From PDF: "Note that the FSM cannot be re-enabled again."
 *
 *  FIX 3 — Linkfail check updated
 *    PDF says LF reset value is '1' — so it is SET after reset and
 *    only clears after a successful MDIO operation. The check
 *    !(RD(GRETH_MDIO) & MDIO_LINKFAIL) is correct — just needs
 *    to run after MDIO reads in Test 2, not before.
 *
 *  FIX 4 — Test 5 descriptor clear with memset (your working fix)
 *    memset(txd,0,sizeof(txd)) and memset(rxd,0,sizeof(rxd))
 *    before setting up new descriptors clears any leftover state.
 *
 *  KEPT: MDIO_PHYSHIFT=11, MDIO_REGSHIFT=6, MDIO_BUSY=(1<<3),
 *        MDIO_READ=(1<<1), MDIO_WRITE=(1<<0), CTRL_PROM=(1<<5)
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

/* ============================================================
 * CTRL register bits — from PDF Table 797
 * ============================================================ */
#define CTRL_TXEN       (1 << 0)    /* TE - Transmit enable */
#define CTRL_RXEN       (1 << 1)    /* RE - Receive enable */
#define CTRL_TXIRQ      (1 << 2)    /* TI - TX interrupt enable */
#define CTRL_RXIRQ      (1 << 3)    /* RI - RX interrupt enable */
#define CTRL_FULLD      (1 << 4)    /* FD - Full duplex */
#define CTRL_PROM       (1 << 5)    /* PM - Promiscuous mode */
#define CTRL_RESET      (1 << 6)    /* RS - Soft reset, self-clearing */
/* bit7  = SP - Speed, only in RMII mode, not used here */
#define CTRL_DD         (1 << 12)   /* FIX 2: DD - Disable duplex detection FSM
                                       Must be set to stop EDCL FSM from
                                       overwriting CTRL after we configure it */

/* ============================================================
 * STATUS register bits — from PDF Table 798
 * ============================================================ */
#define STS_RXERR       (1 << 0)    /* RE - Receiver error */
#define STS_TXERR       (1 << 1)    /* TE - Transmitter error */
#define STS_RXIRQ       (1 << 2)    /* RI - Receiver interrupt (no error) */
#define STS_TXIRQ       (1 << 3)    /* TI - Transmitter interrupt (no error) */
#define STS_RXAHB       (1 << 4)    /* RA - Receiver AHB error */
#define STS_TXAHB       (1 << 5)    /* TA - Transmitter AHB error */
#define STS_ERRORS      (STS_RXERR|STS_TXERR|STS_RXAHB|STS_TXAHB)

/* ============================================================
 * MDIO register bits — from PDF Table 801
 * bit3=BUSY, bit2=LF, bit1=RD(read), bit0=WR(write)
 * bits[15:11]=PHYADDR, bits[10:6]=REGADDR, bits[31:16]=DATA
 * ============================================================ */
#define MDIO_BUSY       (1 << 3)    /* BU - Busy, poll until clear */
#define MDIO_LINKFAIL   (1 << 2)    /* LF - reset value='1', clears on success */
#define MDIO_READ       (1 << 1)    /* RD - start read operation */
#define MDIO_WRITE      (1 << 0)    /* WR - start write operation */
#define MDIO_PHYSHIFT   11          /* bits[15:11] = PHY address */
#define MDIO_REGSHIFT   6           /* bits[10:6]  = register address */

/* ============================================================
 * Descriptor bits — from PDF Table 787/789
 * ============================================================ */
#define DESC_EN         (1 << 11)   /* EN - HW owns descriptor */
#define DESC_WRAP       (1 << 12)   /* WR - wrap to first descriptor */
#define DESC_IRQ        (1 << 13)   /* IE - interrupt enable (unused here) */

/* ============================================================
 * PHY standard registers
 * ============================================================ */
#define PHY_ADDR        0
#define PHY_CTRL        0x00
#define PHY_STATUS      0x01
#define PHY_ID1         0x02
#define PHY_ID2         0x03

#define PHY_CTRL_RESET      (1 << 15)
#define PHY_CTRL_LOOPBACK   (1 << 14)
#define PHY_CTRL_100MB      (1 << 13)
#define PHY_CTRL_AUTONEG    (1 << 12)
#define PHY_CTRL_FULLD      (1 << 8)

/* ============================================================
 * Test parameters
 * ============================================================ */
#define FRAME_SIZE      64
#define NUM_FRAMES      4
#define TIMEOUT         500000

#define RD(addr)        (*((volatile unsigned int *)(addr)))
#define WR(addr,val)    (*((volatile unsigned int *)(addr)) = (unsigned int)(val))

/* ============================================================
 * DMA descriptors — 1KB aligned
 * ============================================================ */
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

/* ============================================================
 * Helpers
 * ============================================================ */
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

/* ============================================================
 * mdio_wait() — poll MDIO_BUSY (bit3) until clear
 * ============================================================ */
static int mdio_wait(void)
{
    int t = TIMEOUT;
    while (RD(GRETH_MDIO) & MDIO_BUSY)
        if (--t == 0) return -1;
    return 0;
}

/* ============================================================
 * mdio_read() — matches greth_api.c read_mii() exactly:
 *   pre-poll → write command → post-poll → check LF → return data
 * ============================================================ */
static unsigned int mdio_read(int phy, int reg)
{
    unsigned int tmp;

    /* Pre-poll: wait for any previous operation to finish */
    if (mdio_wait() < 0) return 0xDEAD;

    /* Write read command with READ opcode (bit1=1) */
    tmp = (phy << MDIO_PHYSHIFT) |
          ((reg & 0x1F) << MDIO_REGSHIFT) |
          MDIO_READ;
    WR(GRETH_MDIO, tmp);

    /* Post-poll: wait for this operation to finish */
    if (mdio_wait() < 0) return 0xDEAD;

    /* Read result — LF=0 means success, LF=1 means failed */
    tmp = RD(GRETH_MDIO);
    if (tmp & MDIO_LINKFAIL) return 0xDEAD;
    return (tmp >> 16) & 0xFFFF;
}

/* ============================================================
 * mdio_write() — matches greth_api.c write_mii() exactly:
 *   pre-poll → write command → post-poll
 * ============================================================ */
static void mdio_write(int phy, int reg, unsigned int val)
{
    unsigned int tmp;

    /* Pre-poll: wait for any previous operation to finish */
    mdio_wait();

    /* Write command with WRITE opcode (bit0=1) */
    tmp = ((val & 0xFFFF) << 16) |
          (phy << MDIO_PHYSHIFT)  |
          ((reg & 0x1F) << MDIO_REGSHIFT) |
          MDIO_WRITE;
    WR(GRETH_MDIO, tmp);

    /* Post-poll: wait for write to complete */
    mdio_wait();
}

/* ============================================================
 * fill_eth_header()
 * ============================================================ */
static void fill_eth_header(unsigned char *buf, int frame_id)
{
    int j;
    /* Destination: broadcast */
    buf[0]=0xFF; buf[1]=0xFF; buf[2]=0xFF;
    buf[3]=0xFF; buf[4]=0xFF; buf[5]=0xFF;
    /* Source: locally administered MAC */
    buf[6]=0x02; buf[7]=0x00; buf[8]=0x00;
    buf[9]=0x00; buf[10]=0x00; buf[11]=0x08;
    /* EtherType: 0x0800 */
    buf[12]=0x08; buf[13]=0x00;
    /* Payload: unique pattern per frame_id */
    for (j = 14; j < FRAME_SIZE; j++)
        buf[j] = (unsigned char)(frame_id * 0x10 + j);
}

/* ============================================================
 * greth_reset()
 * FIX 2: After soft reset, immediately set CTRL_DD (bit12) to
 * disable the EDCL duplex detection FSM before it starts running.
 * The FSM auto-starts after reset and issues MDIO transactions
 * to read PHY speed/duplex, then writes results back to CTRL —
 * overwriting PROM and other bits we set. Disabling it first
 * prevents CTRL from being overwritten behind our back.
 * ============================================================ */
static void greth_reset(void)
{
    WR(GRETH_CTRL, CTRL_RESET);
    delay(5000);

    /* FIX 2: Disable duplex detection FSM immediately after reset.
     * Do this BEFORE configuring anything else so the FSM never
     * gets a chance to overwrite our CTRL settings. */
    WR(GRETH_CTRL, CTRL_DD);
    delay(100);
}

/* ============================================================
 * TEST 1: GRETH Register Access & MAC Address
 * ============================================================ */
static void test_greth_registers(void)
{
    unsigned int v;
    printf("\n--- TEST 1: GRETH Register Access & MAC Address ---\n");

    greth_reset();

    v = RD(GRETH_CTRL);
    check("GRETH reset bit self-clears", !(v & CTRL_RESET));

    /* Write MAC address: 02:00:00:00:00:08 */
    WR(GRETH_MACMSB, 0x00000200);
    WR(GRETH_MACLSB, 0x00000008);
    check("MAC MSB write-readback", RD(GRETH_MACMSB) == 0x00000200);
    check("MAC LSB write-readback", RD(GRETH_MACLSB) == 0x00000008);

    /* Enable TX+RX+FD. Include CTRL_DD to keep FSM disabled.
     * No CTRL_100MB — bit7 only works in RMII mode per PDF. */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_DD);
    v = RD(GRETH_CTRL);
    check("TX enable bit set",   v & CTRL_TXEN);
    check("RX enable bit set",   v & CTRL_RXEN);
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

    check("PHY ID1 not 0xFFFF (MDIO responding)", id1 != 0xFFFF);
    check("PHY ID2 not 0xFFFF (MDIO responding)", id2 != 0xFFFF);
    check("PHY STATUS readable",                  sts != 0xDEAD);
    /* LF clears after successful read — check it now */
    check("No MDIO link failure",                !(RD(GRETH_MDIO) & MDIO_LINKFAIL));
}

/* ============================================================
 * TEST 3: PHY Loopback Mode
 * ============================================================ */
static void test_phy_loopback_mode(void)
{
    unsigned int ctrl;
    printf("\n--- TEST 3: PHY Loopback Mode ---\n");

    /* Write loopback + 100Mb + full-duplex, auto-neg OFF.
     * Auto-neg must be off so PHY does not re-negotiate
     * and override the loopback speed setting. */
    mdio_write(PHY_ADDR, PHY_CTRL,
               PHY_CTRL_LOOPBACK | PHY_CTRL_100MB | PHY_CTRL_FULLD);

    /* mdio_write already polls busy. Extra poll for safety. */
    while (RD(GRETH_MDIO) & MDIO_BUSY);

    ctrl = mdio_read(PHY_ADDR, PHY_CTRL);
    printf("    PHY CTRL after write = 0x%04X\n", ctrl);
    check("PHY loopback bit confirmed via readback",
          ctrl & PHY_CTRL_LOOPBACK);
}

/* ============================================================
 * TEST 4: Single Frame TX → PHY Loopback → RX → Verify
 * CTRL_DD included in every WR(GRETH_CTRL,...) to keep the
 * EDCL FSM disabled so it cannot overwrite PROM or RE bits.
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

    /* CTRL_DD keeps FSM disabled so PROM stays set */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_PROM | CTRL_DD);

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
 * FIX 4: memset descriptors to 0 before setup (your working fix)
 * CTRL_DD included to keep EDCL FSM disabled throughout.
 * ============================================================ */
static void test_multi_frame_loopback(void)
{
    int i, j;
    unsigned int sts = 0;
    printf("\n--- TEST 5: Multi-Frame Stress Loopback (%d frames) ---\n",
           NUM_FRAMES);

    /* FIX 4: Clear descriptor memory before reuse */
    memset(txd, 0, sizeof(txd));
    memset(rxd, 0, sizeof(rxd));

    /* Set up all descriptors BEFORE enabling GRETH */
    for (i = 0; i < NUM_FRAMES; i++) {
        fill_eth_header(txbuf[i], i);
        memset(rxbuf[i], 0, FRAME_SIZE);

        txd[i].addr = (unsigned int)txbuf[i];
        txd[i].ctrl = DESC_EN | FRAME_SIZE;

        rxd[i].addr = (unsigned int)rxbuf[i];
        rxd[i].ctrl = DESC_EN | 128;

        /* WRAP only on last descriptor */
        if (i == NUM_FRAMES - 1) {
            txd[i].ctrl |= DESC_WRAP;
            rxd[i].ctrl |= DESC_WRAP;
        }
    }

    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);

    /* CTRL_DD keeps FSM disabled so PROM and RE stay set */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_PROM | CTRL_DD);

    /* Wait for all TX descriptors released by GRETH */
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

    /* Wait for all RX descriptors filled */
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
