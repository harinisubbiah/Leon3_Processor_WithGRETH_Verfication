/*
 * ============================================================
 *  LEON3 + GRETH Ethernet Verification Test — FIXED VERSION
 *  Design  : GR-XC3S-1500
 *  Tool    : VCS (Linux)
 *  Compiler: sparc-gaisler-elf-gcc
 *
 *  ROOT CAUSE OF ALL YOUR FAILURES — CTRL FLICKERING:
 *  ---------------------------------------------------
 *  The original test wrote CTRL_100MB directly into GRETH_CTRL
 *  BEFORE the PHY had finished its reset + auto-negotiation
 *  sequence. GRETH hardware monitors the PHY negotiation result
 *  and overwrites the speed bit (bit 7) in CTRL to match.
 *  So whatever you wrote got immediately overwritten → bit never
 *  sticks → 100MB check fails → speed mismatch → MDIO link fail
 *  → loopback frames never come back → RX all zeros.
 *
 *  THE FIX — mirrors greth_api.c greth_init() exactly:
 *  1. Soft-reset GRETH (safe — no EDCL in this design)
 *  2. Read PHY address from GRETH_MDIO bits[15:11]
 *  3. Reset PHY via MDIO (write 0x8000 to PHY reg 0)
 *  4. Wait for PHY reset bit to self-clear
 *  5. Wait for auto-negotiation to complete (PHY status bit 5)
 *  6. Read negotiated speed + duplex from PHY reg 0
 *  7. Write GRETH_CTRL ONCE with the result — no race, no flicker
 *
 *  PHY configuration (from phy.vhd + testbench.vhd):
 *  - base1000_t_fd=0, base1000_t_hd=0 → 10/100 only
 *  - On reset: speedsel="10" (100Mb), anegen=1, duplexmode=0
 *  - After auto-neg (anegcnt reaches 10): duplexmode=1 (full)
 *    because base100_x_fd=1 and tech_ability[3]=1
 *  - Expected result: 100Mb full-duplex
 *
 *  PHY ID values (from phy.vhd register decode):
 *  - PHY ID1 (reg 2) = 0xBBCD
 *  - PHY ID2 (reg 3) = 0x9C83
 *
 *  PHY loopback (from phy.vhd loopback_sel process):
 *  - r.ctrl.loopback must be '1'
 *  - Auto-neg must be OFF (anegen=0) so speed does not get
 *    overwritten by re-negotiation after we force it
 *  - Write: loopback | speedsel_100 | fullduplex (no aneg bit)
 * ============================================================
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * GRLIB testbench report — testbench.vhd watches 0x80000200
 * ============================================================ */
#define REPORT_START  (*((volatile unsigned int *)0x80000200) = 0)
#define REPORT_END    (*((volatile unsigned int *)0x80000200) = 1)

/* ============================================================
 * GRETH APB base address (APB slave 13)
 * ============================================================ */
#define GRETH_BASE    0x80000D00

/* Register offsets */
#define GRETH_CTRL    (GRETH_BASE + 0x00)
#define GRETH_STATUS  (GRETH_BASE + 0x04)
#define GRETH_MACMSB  (GRETH_BASE + 0x08)
#define GRETH_MACLSB  (GRETH_BASE + 0x0C)
#define GRETH_MDIO    (GRETH_BASE + 0x10)
#define GRETH_TXDESC  (GRETH_BASE + 0x14)
#define GRETH_RXDESC  (GRETH_BASE + 0x18)

/* ============================================================
 * GRETH_CTRL bits (greth_api.c: GRETH_RESET, GRETH_TXEN etc.)
 * ============================================================ */
#define CTRL_TXEN     (1 << 0)
#define CTRL_RXEN     (1 << 1)
#define CTRL_FULLD    (1 << 4)
#define CTRL_RESET    (1 << 6)   /* self-clearing */
#define CTRL_100MB    (1 << 7)   /* set AFTER reading PHY — never before */
#define CTRL_PROM     (1 << 8)   /* promiscuous — needed for loopback RX */

/* ============================================================
 * GRETH_STATUS bits
 * ============================================================ */
#define STS_RXIRQ     (1 << 2)
#define STS_TXIRQ     (1 << 3)
#define STS_TXERR     (1 << 1)
#define STS_RXERR     (1 << 0)
#define STS_TXAHB     (1 << 5)
#define STS_RXAHB     (1 << 4)
#define STS_ERRORS    (STS_TXERR | STS_RXERR | STS_TXAHB | STS_RXAHB)

/* ============================================================
 * GRETH_MDIO bits (greth_api.c: GRETH_MII_BUSY, GRETH_MII_NVALID)
 * ============================================================ */
#define MDIO_BUSY     (1 << 0)
#define MDIO_WRITE    (1 << 1)
#define MDIO_NVALID   (1 << 2)
#define MDIO_PHYSHIFT 11
#define MDIO_REGSHIFT 6

/* ============================================================
 * Descriptor bits (greth_api.c: GRETH_BD_EN, GRETH_BD_WR)
 * ============================================================ */
#define DESC_EN       (1 << 11)   /* HW owns descriptor */
#define DESC_WRAP     (1 << 12)   /* last in ring — wrap to start */

/* ============================================================
 * PHY standard MII registers
 * ============================================================ */
#define PHY_REG_CTRL    0
#define PHY_REG_STATUS  1
#define PHY_REG_ID1     2
#define PHY_REG_ID2     3

/* PHY CTRL register bits — from phy.vhd wdata decode */
#define PHY_CTRL_RESET   (1 << 15)
#define PHY_CTRL_LOOP    (1 << 14)  /* r.ctrl.loopback */
#define PHY_CTRL_100MB   (1 << 13)  /* r.ctrl.speedsel(1) */
#define PHY_CTRL_ANEG    (1 << 12)  /* r.ctrl.anegen */
#define PHY_CTRL_FULLD   (1 << 8)   /* r.ctrl.duplexmode */
#define PHY_CTRL_SPEED0  (1 << 6)   /* r.ctrl.speedsel(0) */

/* PHY STATUS register bits */
#define PHY_STS_ANEGCMPT (1 << 5)   /* auto-neg complete */
#define PHY_STS_LINK     (1 << 2)

/* Expected PHY IDs from phy.vhd register decode block */
#define PHY_EXPECT_ID1   0xBBCD
#define PHY_EXPECT_ID2   0x9C83

/* ============================================================
 * Test parameters
 * ============================================================ */
#define FRAME_SIZE    64
#define NUM_FRAMES    4
#define TIMEOUT       2000000

/* ============================================================
 * Register access macros
 * ============================================================ */
#define RD(a)      (*((volatile unsigned int *)(a)))
#define WR(a,v)    (*((volatile unsigned int *)(a)) = (unsigned int)(v))

/* ============================================================
 * DMA descriptors — 1 KB aligned (GRETH AHB DMA requirement)
 * ============================================================ */
typedef struct {
    volatile unsigned int ctrl;
    volatile unsigned int addr;
} desc_t;

static desc_t txd[NUM_FRAMES] __attribute__((aligned(1024)));
static desc_t rxd[NUM_FRAMES] __attribute__((aligned(1024)));

static unsigned char txbuf[NUM_FRAMES][128] __attribute__((aligned(32)));
static unsigned char rxbuf[NUM_FRAMES][128] __attribute__((aligned(32)));

/* Pass / fail counters */
static int pass_count = 0;
static int fail_count = 0;

/* PHY address — read from hardware after reset (never hardcoded) */
static int g_phyaddr = 0;

/* Negotiated speed and duplex — read from PHY after auto-neg */
static int g_speed_100  = 0;
static int g_fullduplex = 0;

/* ============================================================
 * check()
 * ============================================================ */
static void check(const char *name, int cond)
{
    if (cond) {
        printf("    PASS: %s\n", name);
        pass_count++;
    } else {
        printf("    FAIL: %s  ***\n", name);
        fail_count++;
    }
}

/* ============================================================
 * delay()
 * ============================================================ */
static void delay(volatile int n) { while (n-- > 0); }

/* ============================================================
 * mdio_wait()
 * Poll MDIO_BUSY until clear. Matches greth_api.c do-while
 * loops in read_mii() and write_mii().
 * Returns 0 OK, -1 timeout.
 * ============================================================ */
static int mdio_wait(void)
{
    int t = TIMEOUT;
    while (RD(GRETH_MDIO) & MDIO_BUSY)
        if (--t == 0) return -1;
    return 0;
}

/* ============================================================
 * mdio_read()
 * Exact translation of greth_api.c read_mii():
 *   poll busy → issue read command → poll busy → check NVALID
 * ============================================================ */
static unsigned int mdio_read(int phy, int reg)
{
    unsigned int tmp;

    /* greth_api: do { tmp=load(mdio); } while (tmp & BUSY); */
    if (mdio_wait() < 0) return 0xDEAD;

    tmp = (phy << MDIO_PHYSHIFT) | ((reg & 0x1F) << MDIO_REGSHIFT) | 2;
    WR(GRETH_MDIO, tmp);

    /* greth_api: do { tmp=load(mdio); } while (tmp & BUSY); */
    if (mdio_wait() < 0) return 0xDEAD;

    tmp = RD(GRETH_MDIO);

    /* greth_api: if (!(tmp & GRETH_MII_NVALID)) return (tmp>>16)&0xFFFF */
    if (tmp & MDIO_NVALID) return 0xDEAD;
    return (tmp >> 16) & 0xFFFF;
}

/* ============================================================
 * mdio_write()
 * Exact translation of greth_api.c write_mii():
 *   poll busy → issue write command → poll busy
 * ============================================================ */
static void mdio_write(int phy, int reg, unsigned int val)
{
    unsigned int tmp;

    /* greth_api: do { tmp=load(mdio); } while (tmp & BUSY); */
    mdio_wait();

    tmp = ((val & 0xFFFF) << 16) |
          (phy << MDIO_PHYSHIFT) |
          ((reg & 0x1F) << MDIO_REGSHIFT) |
          MDIO_WRITE;
    WR(GRETH_MDIO, tmp);

    /* greth_api: do { tmp=load(mdio); } while (tmp & BUSY); */
    mdio_wait();
}

/* ============================================================
 * greth_init_from_phy()
 *
 * This is the function that fixes the CTRL flickering.
 * It mirrors greth_api.c greth_init() step by step:
 *
 * Step 1 — Soft reset GRETH (safe, no EDCL)
 * Step 2 — Read PHY address from GRETH_MDIO bits[15:11]
 * Step 3 — Reset PHY via MDIO write 0x8000 to reg 0
 * Step 4 — Wait for PHY reset bit to self-clear (phy.vhd rstcnt>19)
 * Step 5 — If auto-neg enabled, wait for anegcmpt (status bit 5)
 *           phy.vhd: anegcnt reaches 10 → anegcmpt='1'
 * Step 6 — Read PHY ctrl reg 0 for actual speed and duplex
 *           phy.vhd speedsel: bit13=speedsel(1), bit6=speedsel(0)
 *           "10" = 100Mb,  "00" = 10Mb,  "01" = Gbit (not here)
 * Step 7 — Write GRETH_CTRL ONCE with detected values
 *           This is why CTRL no longer flickers: we only write
 *           it after the PHY has fully settled, so GRETH hardware
 *           has nothing to override.
 * ============================================================ */
static void greth_init_from_phy(void)
{
    unsigned int tmp;
    int i;

    /* Step 1 — Soft reset GRETH (greth_api.c: save GRETH_RESET) */
    printf("    Soft-resetting GRETH...\n");
    WR(GRETH_CTRL, CTRL_RESET);
    do {
        tmp = RD(GRETH_CTRL);
    } while (tmp & CTRL_RESET);
    printf("    GRETH reset complete. CTRL = 0x%08X\n", tmp);

    /* Step 2 — PHY address from GRETH_MDIO bits[15:11]
     * greth_api.c: greth->phyaddr = ((tmp >> 11) & 0x1F); */
    tmp = RD(GRETH_MDIO);
    g_phyaddr = (tmp >> 11) & 0x1F;
    printf("    PHY address = %d\n", g_phyaddr);

    /* Step 3 — Reset PHY
     * greth_api.c: write_mii(phyaddr, 0, 0x8000, regs); */
    mdio_write(g_phyaddr, PHY_REG_CTRL, PHY_CTRL_RESET);

    /* Step 4 — Wait for PHY reset bit to self-clear
     * greth_api.c: while (read_mii(phyaddr,0,regs) & 0x8000);
     * phy.vhd: reset clears when rstcnt > 19 (20 MDC cycles) */
    i = 0;
    do {
        tmp = mdio_read(g_phyaddr, PHY_REG_CTRL);
        i++;
    } while ((tmp & PHY_CTRL_RESET) && (i < 50000));
    printf("    PHY reset cleared after %d reads. PHY CTRL = 0x%04X\n",
           i, tmp);

    /* Step 5 — Wait for auto-negotiation to complete if enabled
     * greth_api.c: if (tmp & 0x1000) while(!(read_mii(..,1,..) & 0x20))
     * phy.vhd: anegcmpt set after anegcnt reaches 10 MDC cycles */
    if (tmp & PHY_CTRL_ANEG) {
        i = 0;
        while (!(mdio_read(g_phyaddr, PHY_REG_STATUS) & PHY_STS_ANEGCMPT)) {
            i++;
            if (i > 50000) {
                printf("    WARNING: auto-neg timeout\n");
                break;
            }
        }
        printf("    Auto-neg complete after %d polls\n", i);
    }

    /* Step 6 — Read actual speed and duplex from PHY CTRL
     * greth_api.c: tmp = read_mii(phyaddr, 0, regs);
     *              speed  = ((tmp >> 13) & 1);   bit13 = speedsel(1)
     *              duplex = (tmp >> 8) & 1;       bit8  = duplexmode
     * phy.vhd after auto-neg with base1000_t_fd=0, base1000_t_hd=0,
     * base100_x_fd=1: speedsel="10" (100Mb), duplexmode=1 (full) */
    tmp = mdio_read(g_phyaddr, PHY_REG_CTRL);
    printf("    PHY CTRL after auto-neg = 0x%04X\n", tmp);

    g_speed_100  = (tmp >> 13) & 1;   /* bit13 = speedsel(1) */
    g_fullduplex = (tmp >> 8)  & 1;   /* bit8  = duplexmode  */

    printf("    Detected: %s, %s duplex\n",
           g_speed_100 ? "100Mb" : "10Mb",
           g_fullduplex ? "full" : "half");

    /* Step 7 — Write GRETH_CTRL ONCE after PHY has settled.
     * greth_api.c: save(&regs->control,
     *                    (duplex<<4) | (speed<<7) | (gbit<<8));
     * This is the key fix: writing CTRL only after the PHY is
     * stable means GRETH hardware has no reason to override it. */
    WR(GRETH_CTRL,
       (g_fullduplex ? CTRL_FULLD : 0) |
       (g_speed_100  ? CTRL_100MB : 0));
    printf("    GRETH_CTRL set to 0x%08X\n", RD(GRETH_CTRL));
}

/* ============================================================
 * fill_eth_header()
 * ============================================================ */
static void fill_eth_header(unsigned char *buf, int frame_id)
{
    int j;
    /* Destination: broadcast FF:FF:FF:FF:FF:FF */
    buf[0]=0xFF; buf[1]=0xFF; buf[2]=0xFF;
    buf[3]=0xFF; buf[4]=0xFF; buf[5]=0xFF;
    /* Source: 02:00:00:00:00:08 (locally administered) */
    buf[6]=0x02; buf[7]=0x00; buf[8]=0x00;
    buf[9]=0x00; buf[10]=0x00; buf[11]=0x08;
    /* EtherType: 0x0800 */
    buf[12]=0x08; buf[13]=0x00;
    /* Payload: deterministic pattern per frame_id
     * frame 0: 0x0E 0x0F 0x10 ...
     * frame 1: 0x1E 0x1F 0x20 ...
     * frame 2: 0x2E 0x2F 0x30 ...
     * frame 3: 0x3E 0x3F 0x40 ... */
    for (j = 14; j < FRAME_SIZE; j++)
        buf[j] = (unsigned char)(frame_id * 0x10 + j);
}

/* ============================================================
 * TEST 1 — GRETH Register Access & MAC Address
 * ============================================================ */
static void test_greth_registers(void)
{
    unsigned int v;
    printf("\n--- TEST 1: GRETH Register Access & MAC Address ---\n");

    /* Full init: reset GRETH, reset PHY, wait for auto-neg,
     * read speed/duplex, write CTRL once. No flicker. */
    greth_init_from_phy();

    /* Verify reset bit is clear */
    v = RD(GRETH_CTRL);
    check("GRETH reset bit self-clears", !(v & CTRL_RESET));

    /* Write and read back MAC address 02:00:00:00:00:08
     * MSB register holds bytes 0-1: 0x0200
     * LSB register holds bytes 2-5: 0x00000008 */
    WR(GRETH_MACMSB, 0x00000200);
    WR(GRETH_MACLSB, 0x00000008);

    check("MAC MSB write-readback", RD(GRETH_MACMSB) == 0x00000200);
    check("MAC LSB write-readback", RD(GRETH_MACLSB) == 0x00000008);

    /* Enable TX + RX using speed/duplex detected from PHY.
     * We do NOT hardcode CTRL_100MB — we use g_speed_100 which
     * was read from the PHY AFTER it settled. This is why the
     * bit sticks now: GRETH already knows the PHY speed, so
     * setting the matching bit does not trigger a hardware
     * override. */
    WR(GRETH_CTRL,
       CTRL_TXEN | CTRL_RXEN |
       (g_fullduplex ? CTRL_FULLD : 0) |
       (g_speed_100  ? CTRL_100MB : 0));

    v = RD(GRETH_CTRL);
    printf("    GRETH_CTRL after enable = 0x%08X\n", v);

    check("TX enable bit set",  v & CTRL_TXEN);
    check("RX enable bit set",  v & CTRL_RXEN);
    check("Full-duplex bit set", v & CTRL_FULLD);
    check("100Mbit bit set",     v & CTRL_100MB);
}

/* ============================================================
 * TEST 2 — PHY MDIO Read (ID & Status)
 * ============================================================ */
static void test_phy_mdio_read(void)
{
    unsigned int id1, id2, sts;
    printf("\n--- TEST 2: PHY MDIO Read ---\n");
    printf("    PHY address = %d\n", g_phyaddr);

    id1 = mdio_read(g_phyaddr, PHY_REG_ID1);
    id2 = mdio_read(g_phyaddr, PHY_REG_ID2);
    sts = mdio_read(g_phyaddr, PHY_REG_STATUS);

    printf("    PHY ID1    = 0x%04X  (expect 0x%04X)\n", id1, PHY_EXPECT_ID1);
    printf("    PHY ID2    = 0x%04X  (expect 0x%04X)\n", id2, PHY_EXPECT_ID2);
    printf("    PHY STATUS = 0x%04X\n", sts);

    check("PHY ID1 = 0xBBCD",            id1 == PHY_EXPECT_ID1);
    check("PHY ID2 = 0x9C83",            id2 == PHY_EXPECT_ID2);
    check("PHY STATUS readable",          sts != (unsigned int)0xDEAD);
    check("No MDIO link failure",        !(RD(GRETH_MDIO) & MDIO_NVALID));
}

/* ============================================================
 * TEST 3 — PHY Loopback Mode
 *
 * From phy.vhd loopback_sel process:
 *   if r.ctrl.loopback = '1' then
 *       rxd  <= lb_rxd;
 *       rx_dv <= lb_rxdv;
 *
 * Two requirements for loopback to work:
 * 1. PHY_CTRL_LOOP (bit14) must be written via MDIO
 * 2. PHY_CTRL_ANEG (bit12) must be CLEARED so the PHY does
 *    not re-run auto-neg and overwrite the speed setting that
 *    loopback needs. phy.vhd will keep running anegcnt if
 *    anegen='1', which re-sets speedsel and duplexmode.
 * ============================================================ */
static void test_phy_loopback_mode(void)
{
    unsigned int ctrl;
    printf("\n--- TEST 3: PHY Loopback Mode ---\n");

    /* Write loopback + 100Mb + full-duplex.
     * Bit 12 (ANEG) is deliberately NOT set = auto-neg disabled.
     * Without this, phy.vhd keeps re-negotiating after we set
     * loopback and the speed flips back to the negotiated value,
     * breaking the echo path. */
    mdio_write(g_phyaddr, PHY_REG_CTRL,
               PHY_CTRL_LOOP | PHY_CTRL_100MB | PHY_CTRL_FULLD);

    /* mdio_write() already polls MDIO_BUSY after the write.
     * Give phy.vhd extra MDC clock cycles to clock the new
     * ctrl value into r.ctrl.loopback. The PHY model is clocked
     * on MDC edges only (see phy.vhd reg process). */
    delay(500000);

    ctrl = mdio_read(g_phyaddr, PHY_REG_CTRL);
    printf("    PHY CTRL after loopback write = 0x%04X\n", ctrl);
    printf("    Loopback bit (14) = %d  (expect 1)\n", (ctrl >> 14) & 1);
    printf("    Auto-neg bit (12) = %d  (expect 0)\n", (ctrl >> 12) & 1);
    printf("    Speed bit    (13) = %d  (expect 1 = 100Mb)\n", (ctrl >> 13) & 1);
    printf("    Duplex bit    (8) = %d  (expect 1 = full)\n", (ctrl >> 8)  & 1);

    check("PHY loopback bit set via readback", ctrl & PHY_CTRL_LOOP);
    check("PHY auto-neg OFF for loopback",    !(ctrl & PHY_CTRL_ANEG));

    /* Update GRETH_CTRL to match loopback speed + promiscuous mode.
     * CTRL_PROM is needed so GRETH accepts the broadcast-destination
     * loopback frame back on the RX path. */
    WR(GRETH_CTRL,
       CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_100MB | CTRL_PROM);
    printf("    GRETH_CTRL for loopback = 0x%08X\n", RD(GRETH_CTRL));
}

/* ============================================================
 * TEST 4 — Single Frame TX → PHY Loopback → RX → Verify
 * ============================================================ */
static void test_single_frame_loopback(void)
{
    int i;
    unsigned int sts;
    int tx_ok = 0, rx_ok = 0, data_ok = 0;

    printf("\n--- TEST 4: Single Frame Loopback ---\n");

    /* Build frame */
    fill_eth_header(txbuf[0], 0);
    memset(rxbuf[0], 0, sizeof(rxbuf[0]));

    /* Setup descriptors */
    txd[0].addr = (unsigned int)txbuf[0];
    txd[0].ctrl = DESC_EN | DESC_WRAP | FRAME_SIZE;

    rxd[0].addr = (unsigned int)rxbuf[0];
    rxd[0].ctrl = DESC_EN | DESC_WRAP | 128;

    /* Write descriptor pointers BEFORE enabling GRETH.
     * If CTRL is enabled first, GRETH may miss the first
     * incoming frame before RXDESC is loaded. */
    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);

    /* Clear any stale status bits */
    WR(GRETH_STATUS, 0xFF);

    /* Enable GRETH — speed already confirmed in Test 3 */
    WR(GRETH_CTRL,
       CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_100MB | CTRL_PROM);

    /* Poll for TX complete */
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_TXIRQ) { tx_ok = 1; break; }
        if (sts & STS_ERRORS) break;
    }
    WR(GRETH_STATUS, STS_TXIRQ);
    printf("    STATUS after TX poll = 0x%08X\n", sts);
    check("Single frame TX completed", tx_ok);
    check("No TX errors", !(sts & (STS_TXERR | STS_TXAHB)));

    /* Poll for RX complete */
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_RXIRQ) { rx_ok = 1; break; }
        if (sts & STS_ERRORS) break;
    }
    WR(GRETH_STATUS, STS_RXIRQ);
    printf("    STATUS after RX poll = 0x%08X\n", sts);
    check("Single frame RX completed", rx_ok);
    check("No RX errors", !(sts & (STS_RXERR | STS_RXAHB)));

    /* Verify payload byte by byte (skip 14-byte Ethernet header) */
    if (rx_ok && !(rxd[0].ctrl & DESC_EN)) {
        data_ok = 1;
        for (i = 14; i < FRAME_SIZE; i++) {
            if (rxbuf[0][i] != txbuf[0][i]) {
                printf("    Byte %d mismatch: TX=0x%02X RX=0x%02X\n",
                       i, txbuf[0][i], rxbuf[0][i]);
                data_ok = 0;
                break;
            }
        }
    } else {
        printf("    RX descriptor still owned by HW — no data to verify\n");
    }
    check("Loopback frame payload matches", data_ok);
}

/* ============================================================
 * TEST 5 — Multi-Frame Stress Loopback (NUM_FRAMES = 4)
 * ============================================================ */
static void test_multi_frame_loopback(void)
{
    int i, j;
    unsigned int sts = 0;
    int tx_ok = 0, rx_ok = 0, all_match = 1;

    printf("\n--- TEST 5: Multi-Frame Stress Loopback (%d frames) ---\n",
           NUM_FRAMES);

    /* Build and set up all descriptors before enabling GRETH */
    for (i = 0; i < NUM_FRAMES; i++) {
        fill_eth_header(txbuf[i], i);
        memset(rxbuf[i], 0, sizeof(rxbuf[i]));

        txd[i].addr = (unsigned int)txbuf[i];
        txd[i].ctrl = DESC_EN | FRAME_SIZE;

        rxd[i].addr = (unsigned int)rxbuf[i];
        rxd[i].ctrl = DESC_EN | 128;

        /* WRAP set only on last descriptor — tells GRETH to
         * wrap back to descriptor 0 after this one */
        if (i == NUM_FRAMES - 1) {
            txd[i].ctrl |= DESC_WRAP;
            rxd[i].ctrl |= DESC_WRAP;
        }
    }

    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);
    WR(GRETH_STATUS, 0xFF);

    WR(GRETH_CTRL,
       CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_100MB | CTRL_PROM);

    /* Wait until ALL TX descriptors are released by GRETH
     * (DESC_EN cleared = GRETH finished with that descriptor) */
    for (i = 0; i < TIMEOUT * NUM_FRAMES; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_TXIRQ) WR(GRETH_STATUS, STS_TXIRQ);
        if (sts & STS_ERRORS) break;
        tx_ok = 1;
        for (j = 0; j < NUM_FRAMES; j++) {
            if (txd[j].ctrl & DESC_EN) { tx_ok = 0; break; }
        }
        if (tx_ok) break;
    }
    check("All TX frames sent",      tx_ok);
    check("No TX errors (stress)", !(sts & (STS_TXERR | STS_TXAHB)));

    /* Wait until ALL RX descriptors are filled */
    for (i = 0; i < TIMEOUT * NUM_FRAMES; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_RXIRQ) WR(GRETH_STATUS, STS_RXIRQ);
        if (sts & STS_ERRORS) break;
        rx_ok = 1;
        for (j = 0; j < NUM_FRAMES; j++) {
            if (rxd[j].ctrl & DESC_EN) { rx_ok = 0; break; }
        }
        if (rx_ok) break;
    }
    check("All RX frames received",  rx_ok);
    check("No RX errors (stress)", !(sts & (STS_RXERR | STS_RXAHB)));

    /* Verify all frames */
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
    REPORT_START;

    printf("\n");
    printf("*****************************************************\n");
    printf("*  LEON3 + GRETH Ethernet Verification (FIXED)     *\n");
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

    REPORT_END;
    return fail_count;
}
