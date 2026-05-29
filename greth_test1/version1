/* VERSION 1
 * ============================================================
 *  LEON3 + GRETH Ethernet Verification Test
 *  For GR-XC3S-1500 Template Design
 *  Simulator : VCS (Linux)
 *  Compiler  : sparc-gaisler-elf-gcc
 *
 *  Usage:
 *    1. Copy this file as systest.c in your design folder
 *    2. sparc-gaisler-elf-gcc -O1 -o systest greth_test.c
 *       OR just: make soft  (uses Makefile default)
 *    3. make vcs-launch
 *
 *  Tests included:
 *    [1] GRETH Register reset & MAC address
 *    [2] PHY MDIO read (PHY ID, status)
 *    [3] PHY loopback mode via MDIO write
 *    [4] Single frame TX → loopback → RX → verify
 *    [5] Multi-frame stress loopback
 * ============================================================
 */

/* ============================================================
 * Standard includes — available in sparc-gaisler-elf-gcc
 * ============================================================ */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * GRLIB report macros
 * These match the GRLIB testbench monitoring system.
 * The testbench.vhd watches for these memory writes
 * to detect pass/fail and print to VCS transcript.
 * ============================================================ */
#define REPORT_START    do { \
    *((volatile int *)0x80000200) = 0; \
} while(0)

#define REPORT_END      do { \
    *((volatile int *)0x80000200) = 1; \
} while(0)

/* ============================================================
 * GRETH Base Address
 * From config.vhd: apbctrl slv13 at 0x80000D00
 * ============================================================ */
#define GRETH_BASE      0x80000D00

/* ============================================================
 * GRETH Register Map (APB offsets)
 * ============================================================ */
#define GRETH_CTRL      (GRETH_BASE + 0x00)
#define GRETH_STATUS    (GRETH_BASE + 0x04)
#define GRETH_MACMSB    (GRETH_BASE + 0x08)
#define GRETH_MACLSB    (GRETH_BASE + 0x0C)
#define GRETH_MDIO      (GRETH_BASE + 0x10)
#define GRETH_TXDESC    (GRETH_BASE + 0x14)
#define GRETH_RXDESC    (GRETH_BASE + 0x18)
#define GRETH_EDCLIP    (GRETH_BASE + 0x1C)

/* ============================================================
 * GRETH Control Register Bits
 * ============================================================ */
#define CTRL_TXEN       (1 << 0)    /* Enable transmitter */
#define CTRL_RXEN       (1 << 1)    /* Enable receiver */
#define CTRL_TXIRQ      (1 << 2)    /* TX interrupt enable */
#define CTRL_RXIRQ      (1 << 3)    /* RX interrupt enable */
#define CTRL_FULLD      (1 << 4)    /* Full duplex */
#define CTRL_RESET      (1 << 6)    /* Soft reset */
#define CTRL_100MB      (1 << 7)    /* 100 Mbit mode */
#define CTRL_PROM       (1 << 8)    /* Promiscuous mode */

/* ============================================================
 * GRETH Status Register Bits
 * ============================================================ */
#define STS_RXERR       (1 << 0)
#define STS_TXERR       (1 << 1)
#define STS_RXIRQ       (1 << 2)
#define STS_TXIRQ       (1 << 3)
#define STS_RXAHB       (1 << 4)
#define STS_TXAHB       (1 << 5)
#define STS_ERRORS      (STS_RXERR|STS_TXERR|STS_RXAHB|STS_TXAHB)

/* ============================================================
 * GRETH MDIO Register Bits
 * ============================================================ */
#define MDIO_BUSY       (1 << 0)
#define MDIO_WRITE      (1 << 1)
#define MDIO_LINKFAIL   (1 << 2)
#define MDIO_PHYSHIFT   10
#define MDIO_REGSHIFT   5

/* ============================================================
 * GRETH Descriptor Bits
 * ============================================================ */
#define DESC_EN         (1 << 11)   /* HW owns descriptor */
#define DESC_WRAP       (1 << 12)   /* Last descriptor */
#define DESC_IRQ        (1 << 13)   /* Interrupt on done */
#define DESC_LENMASK    0x7FF       /* Length bits [10:0] */

/* ============================================================
 * PHY Standard MII Registers
 * ============================================================ */
#define PHY_ADDR        0           /* PHY address on MDIO */
#define PHY_CTRL        0x00        /* Control */
#define PHY_STATUS      0x01        /* Status */
#define PHY_ID1         0x02        /* ID word 1 */
#define PHY_ID2         0x03        /* ID word 2 */

#define PHY_CTRL_RESET      (1<<15)
#define PHY_CTRL_LOOPBACK   (1<<14) /* Internal loopback */
#define PHY_CTRL_100MB      (1<<13)
#define PHY_CTRL_AUTONEG    (1<<12)
#define PHY_CTRL_FULLD      (1<<8)
#define PHY_STS_LINK        (1<<2)

/* ============================================================
 * Test frame config
 * ============================================================ */
#define FRAME_SIZE      64          /* Min Ethernet frame (bytes) */
#define NUM_FRAMES      4           /* Frames per test */
#define TIMEOUT         500000      /* Loop timeout count */

/* ============================================================
 * Register access
 * ============================================================ */
#define RD(addr)        (*((volatile unsigned int *)(addr)))
#define WR(addr,val)    (*((volatile unsigned int *)(addr)) = (unsigned int)(val))

/* ============================================================
 * DMA Descriptors — must be 1KB aligned for GRETH AHB DMA
 * ============================================================ */
typedef struct {
    volatile unsigned int ctrl;
    volatile unsigned int addr;
} desc_t;

static desc_t txd[NUM_FRAMES] __attribute__((aligned(1024)));
static desc_t rxd[NUM_FRAMES] __attribute__((aligned(1024)));

/* ============================================================
 * Frame buffers — 32-byte aligned for AHB
 * ============================================================ */
static unsigned char txbuf[NUM_FRAMES][128] __attribute__((aligned(32)));
static unsigned char rxbuf[NUM_FRAMES][128] __attribute__((aligned(32)));

/* ============================================================
 * Test counters
 * ============================================================ */
static int pass_count = 0;
static int fail_count = 0;

/* ============================================================
 * check() — evaluate and print a test result
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

/* ============================================================
 * delay() — simple busy-wait
 * ============================================================ */
static void delay(volatile int n)
{
    while (n-- > 0);
}

/* ============================================================
 * mdio_wait() — wait for MDIO operation to finish
 * ============================================================ */
static int mdio_wait(void)
{
    int t = TIMEOUT;
    while (RD(GRETH_MDIO) & MDIO_BUSY)
        if (--t == 0) return -1;
    return 0;
}

/* ============================================================
 * mdio_read() — read a PHY register over MDIO
 * ============================================================ */
static unsigned int mdio_read(int phy, int reg)
{
    WR(GRETH_MDIO, (phy << MDIO_PHYSHIFT) | (reg << MDIO_REGSHIFT));
    if (mdio_wait() < 0) return 0xDEAD;
    return (RD(GRETH_MDIO) >> 16) & 0xFFFF;
}

/* ============================================================
 * mdio_write() — write a PHY register over MDIO
 * ============================================================ */
static void mdio_write(int phy, int reg, unsigned int val)
{
    WR(GRETH_MDIO,
       MDIO_WRITE |
       (phy << MDIO_PHYSHIFT) |
       (reg << MDIO_REGSHIFT) |
       (val << 16));
    mdio_wait();
}

/* ============================================================
 * fill_eth_header()
 * Fills standard Ethernet header into buf:
 *   dst[6] | src[6] | ethertype[2]
 * ============================================================ */
static void fill_eth_header(unsigned char *buf, int frame_id)
{
    /* Destination: broadcast FF:FF:FF:FF:FF:FF */
    buf[0]=0xFF; buf[1]=0xFF; buf[2]=0xFF;
    buf[3]=0xFF; buf[4]=0xFF; buf[5]=0xFF;
    /* Source: 02:00:00:00:00:08 (locally administered) */
    buf[6]=0x02; buf[7]=0x00; buf[8]=0x00;
    buf[9]=0x00; buf[10]=0x00; buf[11]=0x08;
    /* EtherType: 0x0800 = IPv4 */
    buf[12]=0x08; buf[13]=0x00;
    /* Payload: known pattern per frame */
    int j;
    for (j = 14; j < FRAME_SIZE; j++)
        buf[j] = (unsigned char)(frame_id * 0x10 + j);
}

/* ============================================================
 * greth_reset() — soft reset GRETH and wait
 * ============================================================ */
static void greth_reset(void)
{
    WR(GRETH_CTRL, CTRL_RESET);
    delay(5000);
    /* Reset bit self-clears */
}

/* ============================================================
 * TEST 1: GRETH Register Access & MAC Address
 * ============================================================ */
static void test_greth_registers(void)
{
    unsigned int v;
    printf("\n--- TEST 1: GRETH Register Access & MAC Address ---\n");

    /* Soft reset */
    greth_reset();
    v = RD(GRETH_CTRL);
    check("GRETH reset bit self-clears", !(v & CTRL_RESET));

    /* Write MAC address */
    WR(GRETH_MACMSB, 0x00000200);   /* 02:00 */
    WR(GRETH_MACLSB, 0x00000008);   /* 00:00:00:08 */

    check("MAC MSB write-readback",
          RD(GRETH_MACMSB) == 0x00000200);
    check("MAC LSB write-readback",
          RD(GRETH_MACLSB) == 0x00000008);

    /* Enable TX+RX */
    WR(GRETH_CTRL, CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_100MB);
    v = RD(GRETH_CTRL);
    check("TX enable bit set",  v & CTRL_TXEN);
    check("RX enable bit set",  v & CTRL_RXEN);
    check("Full-duplex bit set",v & CTRL_FULLD);
    check("100Mbit bit set",    v & CTRL_100MB);
}

/* ============================================================
 * TEST 2: PHY MDIO Read (ID & Status)
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

    /* Check MDIO link failure bit in GRETH */
    check("No MDIO link failure",
          !(RD(GRETH_MDIO) & MDIO_LINKFAIL));
}

/* ============================================================
 * TEST 3: PHY Loopback Mode via MDIO Write
 * ============================================================ */
static void test_phy_loopback_mode(void)
{
    unsigned int ctrl;
    printf("\n--- TEST 3: PHY Loopback Mode ---\n");

    /* Enable PHY internal loopback:
     * PHY_CTRL_LOOPBACK | 100Mb | Full-duplex
     * In simulation the GRLIB eth_phy model honours this */
    mdio_write(PHY_ADDR, PHY_CTRL,
               PHY_CTRL_LOOPBACK | PHY_CTRL_100MB | PHY_CTRL_FULLD);
    delay(10000);

    ctrl = mdio_read(PHY_ADDR, PHY_CTRL);
    printf("    PHY CTRL after write = 0x%04X\n", ctrl);
    check("PHY loopback bit confirmed via readback",
          ctrl & PHY_CTRL_LOOPBACK);
}

/* ============================================================
 * TEST 4: Single Frame TX → PHY Loopback → RX → Verify
 * ============================================================ */
static void test_single_frame_loopback(void)
{
    int i;
    unsigned int sts;
    printf("\n--- TEST 4: Single Frame Loopback ---\n");

    /* Build one TX frame */
    fill_eth_header(txbuf[0], 0);
    memset(rxbuf[0], 0, FRAME_SIZE);

    /* TX descriptor: enable, length, wrap (only 1 desc) */
    txd[0].addr = (unsigned int)txbuf[0];
    txd[0].ctrl = DESC_EN | DESC_WRAP | FRAME_SIZE;

    /* RX descriptor */
    rxd[0].addr = (unsigned int)rxbuf[0];
    rxd[0].ctrl = DESC_EN | DESC_WRAP | 128;

    /* Point GRETH at descriptors */
    WR(GRETH_TXDESC, (unsigned int)txd);
    WR(GRETH_RXDESC, (unsigned int)rxd);

    /* Enable TX+RX+promiscuous */
    WR(GRETH_CTRL,
       CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_100MB | CTRL_PROM);

    /* Wait for TX done */
    int tx_ok = 0;
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_TXIRQ)  { tx_ok = 1; break; }
        if (sts & STS_ERRORS) { break; }
    }
    WR(GRETH_STATUS, STS_TXIRQ); /* clear */
    check("Single frame TX completed", tx_ok);
    check("No TX errors", !(sts & (STS_TXERR|STS_TXAHB)));

    /* Wait for RX done */
    int rx_ok = 0;
    for (i = 0; i < TIMEOUT; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_RXIRQ)  { rx_ok = 1; break; }
        if (sts & STS_ERRORS) { break; }
    }
    WR(GRETH_STATUS, STS_RXIRQ); /* clear */
    check("Single frame RX completed", rx_ok);
    check("No RX errors", !(sts & (STS_RXERR|STS_RXAHB)));

    /* Verify payload (bytes 14..FRAME_SIZE-1) */
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
 * ============================================================ */
static void test_multi_frame_loopback(void)
{
    int i, j;
    unsigned int sts;
    printf("\n--- TEST 5: Multi-Frame Stress Loopback (%d frames) ---\n",
           NUM_FRAMES);

    /* Build NUM_FRAMES TX frames with different patterns */
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
    WR(GRETH_CTRL,
       CTRL_TXEN | CTRL_RXEN | CTRL_FULLD | CTRL_100MB | CTRL_PROM);

    /* Wait for all TX */
    int tx_ok = 0;
    for (i = 0; i < TIMEOUT * NUM_FRAMES; i++) {
        sts = RD(GRETH_STATUS);
        if (sts & STS_TXIRQ)  { tx_ok = 1;
                                 WR(GRETH_STATUS, STS_TXIRQ); }
        if (sts & STS_ERRORS) break;
        /* Check if all TX descriptors released by HW */
        int all_done = 1;
        for (j = 0; j < NUM_FRAMES; j++)
            if (txd[j].ctrl & DESC_EN) { all_done = 0; break; }
        if (all_done) { tx_ok = 1; break; }
    }
    check("All TX frames sent", tx_ok);
    check("No TX errors (stress)", !(sts & (STS_TXERR|STS_TXAHB)));

    /* Wait for all RX */
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
    check("All RX frames received", rx_ok);
    check("No RX errors (stress)", !(sts & (STS_RXERR|STS_RXAHB)));

    /* Verify all frames */
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

    /* ---- Summary ---- */
    printf("\n*****************************************************\n");
    printf("*  RESULTS                                         *\n");
    printf("*****************************************************\n");
    printf("  Passed : %d\n", pass_count);
    printf("  Failed : %d\n", fail_count);

    if (fail_count == 0) {
        printf("\n  *** GRETH ETHERNET TEST PASSED ***\n");
    } else {
        printf("\n  *** %d TEST(S) FAILED ***\n", fail_count);
    }
    printf("*****************************************************\n\n");

    return fail_count;
}
