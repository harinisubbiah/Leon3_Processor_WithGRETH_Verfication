-- ============================================================
-- tb_bridge_top.vhd  (FIXED)
-- Testbench for DUT: eth_module (bridge_top)
-- Simulator: VCS
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.bridge_pkg.all;

entity tb_bridge_top is
end entity tb_bridge_top;

architecture sim of tb_bridge_top is

  -- ============================================================
  -- Constants
  -- ============================================================
  constant ABITS      : integer := 28;
  constant CLK_PERIOD : time    := 10 ns;

  -- ============================================================
  -- DUT port signals
  -- ============================================================
  signal clk  : std_logic := '0';
  signal rst  : std_logic := '0';

  -- AHB-Master outputs (TB drives IN to DUT)
  signal hbusreq : std_ulogic                    := '0';
  signal hlock   : std_ulogic                    := '0';
  signal htrans  : std_logic_vector(1 downto 0)  := "00";
  signal haddr   : std_logic_vector(31 downto 0) := (others => '0');
  signal hwrite  : std_ulogic                    := '0';
  signal hsize   : std_logic_vector(2 downto 0)  := "010";
  signal hburst  : std_logic_vector(2 downto 0)  := "000";
  signal hprot   : std_logic_vector(3 downto 0)  := "0011";
  signal hwdata  : std_logic_vector(31 downto 0) := (others => '0');

  -- AHB-Master inputs (DUT drives OUT to TB)
  signal hgrant  : std_ulogic;
  signal hready  : std_ulogic;
  signal hresp   : std_logic_vector(1 downto 0);
  signal hrdata  : std_logic_vector(31 downto 0);

  -- APB-Slave outputs (TB drives IN to DUT)
  signal prdata  : std_logic_vector(31 downto 0) := (others => '0');

  -- APB-Slave inputs (DUT drives OUT to TB)
  signal psel    : std_ulogic;
  signal penable : std_ulogic;
  signal paddr   : std_logic_vector(31 downto 0);
  signal pwrite  : std_ulogic;
  signal pwdata  : std_logic_vector(31 downto 0);

  -- SRAM outputs (TB drives IN to DUT)
  signal sram_rdata : std_logic_vector(31 downto 0) := (others => '0');

  -- SRAM inputs (DUT drives OUT to TB)
  signal sram_addr  : std_logic_vector(ABITS-1 downto 0);
  signal sram_wdata : std_logic_vector(31 downto 0);
  signal sram_ce1   : std_logic;
  signal sram_wen   : std_ulogic;
  signal sram_oen   : std_ulogic;

  -- Processor outputs (TB drives IN to DUT)
  signal proc_wdata : std_logic_vector(31 downto 0)      := (others => '0');
  signal proc_addr  : std_logic_vector(ABITS-1 downto 0) := (others => '0');
  signal proc_ce1   : std_logic := '1';
  signal proc_wen   : std_logic := '1';
  signal proc_oen   : std_logic := '1';

  -- Processor inputs (DUT drives OUT to TB)
  signal proc_rdata : std_logic_vector(31 downto 0);

  -- Scan-test + IRQ (DUT drives OUT, TB observes)
  signal testrst : std_ulogic;
  signal testen  : std_ulogic;
  signal testoen : std_ulogic;
  signal irq     : std_logic;

  -- ============================================================
  -- AHB Master internal handshake signals
  -- ============================================================
  -- These are used by the AHB master model process
  -- to coordinate address/data phases properly
  signal ahb_req_addr  : std_logic_vector(31 downto 0) := (others => '0');
  signal ahb_req_wdata : std_logic_vector(31 downto 0) := (others => '0');
  signal ahb_req_write : std_ulogic                    := '0';
  signal ahb_start     : std_logic                     := '0';
  signal ahb_done      : std_logic                     := '0';

  -- ============================================================
  -- Component Declaration
  -- ============================================================
  component eth_module is
    generic (
      constant SRAMBANKS : integer := 4;
      constant TACC      : integer := 10;
      constant ABITS     : integer := 28;
      constant sram_file : string  := "sram.srec"
    );
    port (
      clk, rst   : in  std_logic;
      hbusreq    : in  std_ulogic;
      hlock      : in  std_ulogic;
      htrans     : in  std_logic_vector(1 downto 0);
      haddr      : in  std_logic_vector(31 downto 0);
      hwrite     : in  std_ulogic;
      hsize      : in  std_logic_vector(2 downto 0);
      hburst     : in  std_logic_vector(2 downto 0);
      hprot      : in  std_logic_vector(3 downto 0);
      hwdata     : in  std_logic_vector(31 downto 0);
      hgrant     : out std_ulogic;
      hready     : out std_ulogic;
      hresp      : out std_logic_vector(1 downto 0);
      hrdata     : out std_logic_vector(31 downto 0);
      prdata     : in  std_logic_vector(31 downto 0);
      psel       : out std_ulogic;
      penable    : out std_ulogic;
      paddr      : out std_logic_vector(31 downto 0);
      pwrite     : out std_ulogic;
      pwdata     : out std_logic_vector(31 downto 0);
      sram_rdata : in  std_logic_vector(31 downto 0);
      sram_addr  : out std_logic_vector(ABITS-1 downto 0);
      sram_wdata : out std_logic_vector(31 downto 0);
      sram_ce1   : out std_logic;
      sram_wen   : out std_ulogic;
      sram_oen   : out std_ulogic;
      proc_wdata : in  std_logic_vector(31 downto 0);
      proc_addr  : in  std_logic_vector(ABITS-1 downto 0);
      proc_ce1   : in  std_logic;
      proc_wen   : in  std_logic;
      proc_oen   : in  std_logic;
      proc_rdata : out std_logic_vector(31 downto 0);
      testrst    : out std_ulogic;
      testen     : out std_ulogic;
      testoen    : out std_ulogic;
      irq        : out std_logic
    );
  end component;

begin

  -- ============================================================
  -- DUT Instantiation
  -- ============================================================
  DUT : eth_module
    generic map (
      SRAMBANKS => 4,
      TACC      => 10,
      ABITS     => 28,
      sram_file => "sram.srec"
    )
    port map (
      clk        => clk,
      rst        => rst,
      hbusreq    => hbusreq,
      hlock      => hlock,
      htrans     => htrans,
      haddr      => haddr,
      hwrite     => hwrite,
      hsize      => hsize,
      hburst     => hburst,
      hprot      => hprot,
      hwdata     => hwdata,
      hgrant     => hgrant,
      hready     => hready,
      hresp      => hresp,
      hrdata     => hrdata,
      prdata     => prdata,
      psel       => psel,
      penable    => penable,
      paddr      => paddr,
      pwrite     => pwrite,
      pwdata     => pwdata,
      sram_rdata => sram_rdata,
      sram_addr  => sram_addr,
      sram_wdata => sram_wdata,
      sram_ce1   => sram_ce1,
      sram_wen   => sram_wen,
      sram_oen   => sram_oen,
      proc_wdata => proc_wdata,
      proc_addr  => proc_addr,
      proc_ce1   => proc_ce1,
      proc_wen   => proc_wen,
      proc_oen   => proc_oen,
      proc_rdata => proc_rdata,
      testrst    => testrst,
      testen     => testen,
      testoen    => testoen,
      irq        => irq
    );

  -- ============================================================
  -- Clock Generation
  -- ============================================================
  clk_gen : process
  begin
    clk <= '0'; wait for CLK_PERIOD / 2;
    clk <= '1'; wait for CLK_PERIOD / 2;
  end process;

  -- ============================================================
  -- SRAM Model
  -- Responds to sram_ce1=0 and sram_oen=0 (read)
  -- Returns data based on address so we can verify on waveform
  -- ============================================================
  sram_model : process(sram_ce1, sram_oen, sram_wen, sram_addr)
  begin
    if sram_ce1 = '0' and sram_oen = '0' and sram_wen = '1' then
      -- READ: return address-tagged data
      -- e.g. addr=0x00000040 → rdata=0x40ABCDEF
      sram_rdata <= sram_addr(7 downto 0)  -- upper byte = addr[7:0]
                  & x"AB"
                  & x"CD"
                  & x"EF";
    else
      sram_rdata <= (others => '0');
    end if;
  end process;

  -- ============================================================
  -- APB Slave Model
  -- Two-cycle APB handshake:
  --   Cycle 1: psel=1, penable=0  → SETUP phase
  --   Cycle 2: psel=1, penable=1  → ENABLE phase → respond here
  -- ============================================================
  apb_slave_model : process(clk)
  begin
    if rising_edge(clk) then
      if psel = '1' and penable = '1' then
        if pwrite = '0' then
          -- READ: return address-tagged data
          -- e.g. paddr=0x20 → prdata=0x20FACADE
          prdata <= paddr(7 downto 0)
                  & x"FA"
                  & x"CA"
                  & x"DE";
        else
          -- WRITE: just acknowledge, prdata doesn't matter
          prdata <= (others => '0');
        end if;
      else
        prdata <= (others => '0');
      end if;
    end if;
  end process;

  -- ============================================================
  -- AHB Master Model
  -- Proper AHB protocol:
  --   1. Assert hbusreq
  --   2. Wait for hgrant=1 AND hready=1
  --   3. Drive address phase (htrans=NONSEQ, haddr, hwrite)
  --   4. Wait for hready=1 (address phase accepted)
  --   5. Drive data phase (hwdata for write)
  --      OR sample hrdata (for read)
  --   6. Go IDLE (htrans=00)
  --   7. Signal done to stimulus process
  -- ============================================================
  ahb_master_model : process
  begin
    -- Idle defaults
    hbusreq <= '0';
    hlock   <= '0';
    htrans  <= "00";   -- IDLE
    haddr   <= (others => '0');
    hwrite  <= '0';
    hsize   <= "010";  -- 32-bit
    hburst  <= "000";  -- SINGLE
    hprot   <= "0011";
    hwdata  <= (others => '0');
    ahb_done <= '0';

    -- Wait for stimulus to trigger a transaction
    wait until ahb_start = '1';

    -- ── Step 1: Request the bus ──────────────────────────────
    hbusreq <= '1';
    wait until rising_edge(clk);

    -- ── Step 2: Wait for grant ───────────────────────────────
    -- AHB spec: master can only start when hgrant=1 AND hready=1
    wait until rising_edge(clk) and
               hgrant = '1'     and
               hready = '1';

    -- ── Step 3: Address Phase ────────────────────────────────
    htrans  <= "10";              -- NONSEQ
    haddr   <= ahb_req_addr;
    hwrite  <= ahb_req_write;
    hsize   <= "010";
    hburst  <= "000";

    -- ── Step 4: Wait for address phase to be accepted ────────
    -- hready=1 means slave accepted the address phase
    wait until rising_edge(clk) and hready = '1';

    -- ── Step 5: Data Phase ───────────────────────────────────
    htrans <= "00";   -- IDLE (single transfer, no more after this)

    if ahb_req_write = '1' then
      -- WRITE: put data on hwdata this cycle
      hwdata <= ahb_req_wdata;
      -- Wait for slave to accept data
      wait until rising_edge(clk) and hready = '1';
      hwdata <= (others => '0');

    else
      -- READ: sample hrdata when hready=1
      -- hrdata is valid when hready=1 in the data phase
      wait until rising_edge(clk) and hready = '1';
      -- hrdata is now valid — waveform will show it here
    end if;

    -- ── Step 6: Release bus ──────────────────────────────────
    hbusreq <= '0';
    hwrite  <= '0';

    -- ── Step 7: Signal completion to stimulus ────────────────
    ahb_done <= '1';
    wait until rising_edge(clk);
    ahb_done <= '0';

    -- Loop back and wait for next transaction
    wait until ahb_start = '0';

  end process;

  -- ============================================================
  -- Stimulus Process
  -- Controls test sequences
  -- Triggers AHB master model via ahb_start/ahb_done handshake
  -- ============================================================
  stimulus : process

    -- Helper: trigger one AHB transaction and wait for it
    procedure do_ahb_transfer (
      addr  : in std_logic_vector(31 downto 0);
      wdata : in std_logic_vector(31 downto 0);
      wr    : in std_ulogic
    ) is
    begin
      ahb_req_addr  <= addr;
      ahb_req_wdata <= wdata;
      ahb_req_write <= wr;
      ahb_start     <= '1';
      wait until rising_edge(clk) and ahb_done = '1';
      ahb_start <= '0';
      wait for 2 * CLK_PERIOD;
    end procedure;

  begin

    -- ── Phase 0: Reset ────────────────────────────────────────
    rst       <= '0';
    ahb_start <= '0';
    wait for 5 * CLK_PERIOD;
    rst <= '1';              -- Release reset (active HIGH in your DUT)
    wait for 5 * CLK_PERIOD;

    report "=== RESET RELEASED ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 1: Processor WRITE via APB Bridge
    -- leon3 writes 0xDEADBEEF to address 0x0000010
    -- Expected on waveform:
    --   psel=1, penable=1, paddr=0x10, pwdata=0xDEADBEEF
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 1: Processor WRITE ===" severity note;
    wait until rising_edge(clk);

    proc_addr  <= std_logic_vector(to_unsigned(16#0000010#, ABITS));
    proc_wdata <= x"DEADBEEF";
    proc_ce1   <= '0';
    proc_wen   <= '0';   -- Active low = write
    proc_oen   <= '1';

    wait for 4 * CLK_PERIOD;  -- Hold through APB setup+enable

    proc_ce1   <= '1';
    proc_wen   <= '1';
    wait for 3 * CLK_PERIOD;

    report "=== TEST 1 DONE ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 2: Processor READ via APB Bridge
    -- leon3 reads from address 0x0000020
    -- Expected on waveform:
    --   psel=1, penable=1, pwrite=0
    --   proc_rdata = 0x20FACADE (from APB slave model)
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 2: Processor READ ===" severity note;
    wait until rising_edge(clk);

    proc_addr  <= std_logic_vector(to_unsigned(16#0000020#, ABITS));
    proc_wdata <= (others => '0');
    proc_ce1   <= '0';
    proc_wen   <= '1';   -- Deasserted = read
    proc_oen   <= '0';   -- Active low = output enable

    wait for 4 * CLK_PERIOD;

    proc_ce1   <= '1';
    proc_oen   <= '1';
    wait for 3 * CLK_PERIOD;

    report "=== TEST 2 DONE: proc_rdata should be 0x20FACADE ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 3: AHB Master WRITE → SRAM (via AHB Bridge)
    -- GRETH MAC writes 0xCAFEBABE to address 0x40000040
    -- Expected on waveform:
    --   sram_addr=0x0000040, sram_wdata=0xCAFEBABE, sram_wen=0
    --   hready=1 when done
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 3: AHB Master WRITE ===" severity note;

    do_ahb_transfer(
      addr  => x"40000040",
      wdata => x"CAFEBAABE",
      wr    => '1'
    );

    report "=== TEST 3 DONE: check sram_addr/wdata/wen ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 4: AHB Master READ ← SRAM (via AHB Bridge)
    -- GRETH MAC reads from address 0x40000080
    -- SRAM model returns: 0x80ABCDEF
    -- Expected on waveform:
    --   sram_addr=0x0000080, sram_oen=0
    --   hrdata=0x80ABCDEF when hready=1
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 4: AHB Master READ ===" severity note;

    do_ahb_transfer(
      addr  => x"40000080",
      wdata => x"00000000",  -- Don't care for read
      wr    => '0'
    );

    report "=== TEST 4 DONE: check hrdata=0x80ABCDEF ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 5: Back-to-back Processor Writes
    -- 3 consecutive writes, address increments by 4
    -- Expected: psel/penable pulsing, pwdata toggling
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 5: Back-to-back Processor Writes ===" severity note;

    wait until rising_edge(clk);
    proc_ce1 <= '0';
    proc_wen <= '0';
    proc_oen <= '1';

    proc_addr  <= std_logic_vector(to_unsigned(16#0000100#, ABITS));
    proc_wdata <= x"11111111";
    wait for 2 * CLK_PERIOD;

    proc_addr  <= std_logic_vector(to_unsigned(16#0000104#, ABITS));
    proc_wdata <= x"22222222";
    wait for 2 * CLK_PERIOD;

    proc_addr  <= std_logic_vector(to_unsigned(16#0000108#, ABITS));
    proc_wdata <= x"33333333";
    wait for 2 * CLK_PERIOD;

    proc_ce1   <= '1';
    proc_wen   <= '1';
    wait for 3 * CLK_PERIOD;

    report "=== TEST 5 DONE ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 6: AHB WRITE followed immediately by AHB READ
    -- Write 0xBEEFDEAD to 0x400000C0
    -- Then read back from same address
    -- Expected: hrdata = 0xC0ABCDEF (SRAM model ignores write,
    --           always returns address-tagged data on read)
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 6: AHB Write then Read same address ===" severity note;

    do_ahb_transfer(
      addr  => x"400000C0",
      wdata => x"BEEFDEAD",
      wr    => '1'
    );

    do_ahb_transfer(
      addr  => x"400000C0",
      wdata => x"00000000",
      wr    => '0'
    );

    report "=== TEST 6 DONE: hrdata should be 0xC0ABCDEF ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- END
    -- ══════════════════════════════════════════════════════════
    wait for 20 * CLK_PERIOD;
    report "=== ALL TESTS COMPLETE ===" severity note;
    report "Simulation finished" severity failure;

  end process stimulus;

end architecture sim;
