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

  -- AHB-Master (TB drives IN to DUT)
  signal hbusreq : std_ulogic                    := '0';
  signal hlock   : std_ulogic                    := '0';
  signal htrans  : std_logic_vector(1 downto 0)  := "00";
  signal haddr   : std_logic_vector(31 downto 0) := (others => '0');
  signal hwrite  : std_ulogic                    := '0';
  signal hsize   : std_logic_vector(2 downto 0)  := "010";
  signal hburst  : std_logic_vector(2 downto 0)  := "000";
  signal hprot   : std_logic_vector(3 downto 0)  := "0011";
  signal hwdata  : std_logic_vector(31 downto 0) := (others => '0');

  -- AHB-Master (DUT drives OUT to TB)
  signal hgrant  : std_ulogic;
  signal hready  : std_ulogic;
  signal hresp   : std_logic_vector(1 downto 0);
  signal hrdata  : std_logic_vector(31 downto 0);

  -- APB-Slave (TB drives IN to DUT)
  signal prdata  : std_logic_vector(31 downto 0) := (others => '0');

  -- APB-Slave (DUT drives OUT to TB)
  signal psel    : std_ulogic;
  signal penable : std_ulogic;
  signal paddr   : std_logic_vector(31 downto 0);
  signal pwrite  : std_ulogic;
  signal pwdata  : std_logic_vector(31 downto 0);

  -- SRAM (TB drives IN to DUT)
  signal sram_rdata : std_logic_vector(31 downto 0) := (others => '0');

  -- SRAM (DUT drives OUT to TB)
  signal sram_addr  : std_logic_vector(ABITS-1 downto 0);
  signal sram_wdata : std_logic_vector(31 downto 0);
  signal sram_ce1   : std_logic;
  signal sram_wen   : std_ulogic;
  signal sram_oen   : std_ulogic;

  -- Processor (TB drives IN to DUT)
  signal proc_wdata : std_logic_vector(31 downto 0)      := (others => '0');
  signal proc_addr  : std_logic_vector(ABITS-1 downto 0) := (others => '0');
  signal proc_ce1   : std_logic := '1';
  signal proc_wen   : std_logic := '1';
  signal proc_oen   : std_logic := '1';

  -- Processor (DUT drives OUT to TB)
  signal proc_rdata : std_logic_vector(31 downto 0);

  -- Scan-test and IRQ (DUT drives OUT, TB observes)
  signal testrst : std_ulogic;
  signal testen  : std_ulogic;
  signal testoen : std_ulogic;
  signal irq     : std_logic;

  -- ============================================================
  -- AHB Master Model internal handshake signals
  -- Used to coordinate between ahb_master_model and stimulus
  -- ============================================================
  signal ahb_start_write : std_logic := '0'; -- stimulus → model: start a write
  signal ahb_start_read  : std_logic := '0'; -- stimulus → model: start a read
  signal ahb_done        : std_logic := '0'; -- model → stimulus: transfer done
  signal ahb_req_addr    : std_logic_vector(31 downto 0) := (others => '0');
  signal ahb_req_data    : std_logic_vector(31 downto 0) := (others => '0');
  signal ahb_resp_data   : std_logic_vector(31 downto 0); -- captured hrdata

  -- ============================================================
  -- DUT Component Declaration
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
  -- Clock Generation: 100MHz
  -- ============================================================
  clk_gen : process
  begin
    clk <= '0'; wait for CLK_PERIOD / 2;
    clk <= '1'; wait for CLK_PERIOD / 2;
  end process;

  -- ============================================================
  -- SRAM Model
  -- Responds to sram_ce1=0, sram_oen=0 (read)
  -- Returns recognizable data: addr[7:0] & 0xABCDEF
  -- ============================================================
  sram_model : process(sram_ce1, sram_oen, sram_addr)
  begin
    if sram_ce1 = '0' and sram_oen = '0' then
      sram_rdata <= sram_addr(7 downto 0) & x"ABCDEF";
    else
      sram_rdata <= (others => '0');
    end if;
  end process;

  -- ============================================================
  -- APB Slave Model
  -- Phase 1 (psel=1, penable=0): SETUP  - latch address
  -- Phase 2 (psel=1, penable=1): ENABLE - drive prdata
  -- For writes: just acknowledge
  -- For reads:  return address-based data so we can verify
  -- ============================================================
  apb_slave_model : process(clk)
  begin
    if rising_edge(clk) then
      if psel = '1' and penable = '0' then
        -- SETUP phase: prepare data for next cycle
        if pwrite = '0' then
          -- Read: return recognizable pattern based on address
          prdata <= x"DA" &
                    paddr(15 downto 8) &
                    paddr(7  downto 0) &
                    x"A5";
        else
          prdata <= (others => '0');
        end if;
      end if;
    end if;
  end process;

  -- ============================================================
  -- AHB Master Model (FIXED)
  -- Properly follows AHB protocol:
  --   1. Assert hbusreq
  --   2. Wait for hgrant=1 AND hready=1
  --   3. Drive address phase (htrans=NONSEQ, haddr, hwrite)
  --   4. Wait for hready=1 (end of address phase)
  --   5. Drive data phase  (hwdata for write)
  --   6. Wait for hready=1 (end of data phase)
  --   7. Go IDLE, assert ahb_done
  -- ============================================================
  ahb_master_model : process
  begin

    -- Default: bus idle
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

    -- Wait forever, wake up when stimulus triggers a request
    loop
      -- Wait for stimulus to request a transfer
      wait until (ahb_start_write = '1' or ahb_start_read = '1')
                 and rising_edge(clk);

      -- ── Step 1: Request the bus ──────────────────────────
      hbusreq <= '1';

      -- ── Step 2: Wait for grant + ready ──────────────────
      -- The AHB arbiter (inside bridge) must grant us the bus
      wait until rising_edge(clk) and
                 hgrant = '1' and hready = '1';

      -- ── Step 3: Address Phase ────────────────────────────
      htrans <= "10";                  -- NONSEQ
      haddr  <= ahb_req_addr;          -- Address from stimulus

      if ahb_start_write = '1' then
        hwrite <= '1';                 -- Write
      else
        hwrite <= '0';                 -- Read
      end if;

      -- ── Step 4: Wait for hready (end of address phase) ──
      wait until rising_edge(clk) and hready = '1';

      -- ── Step 5: Data Phase ───────────────────────────────
      htrans <= "00";                  -- Back to IDLE (SINGLE burst done)

      if ahb_start_write = '1' then
        hwdata <= ahb_req_data;        -- Present write data
      end if;

      -- ── Step 6: Wait for hready (end of data phase) ─────
      wait until rising_edge(clk) and hready = '1';

      -- Capture read data
      if ahb_start_read = '1' then
        ahb_resp_data <= hrdata;
      end if;

      -- ── Step 7: Release bus, signal done ────────────────
      hbusreq <= '0';
      hwrite  <= '0';
      hwdata  <= (others => '0');

      ahb_done <= '1';
      wait until rising_edge(clk);
      ahb_done <= '0';

    end loop;
  end process;

  -- ============================================================
  -- Stimulus Process
  -- Controls test sequences, triggers AHB master model
  -- ============================================================
  stimulus : process

    -- Helper: do one AHB write
    procedure do_ahb_write (
      addr : in std_logic_vector(31 downto 0);
      data : in std_logic_vector(31 downto 0)
    ) is
    begin
      ahb_req_addr    <= addr;
      ahb_req_data    <= data;
      ahb_start_write <= '1';
      wait until rising_edge(clk);
      ahb_start_write <= '0';
      -- Wait for model to finish
      wait until ahb_done = '1';
      wait until rising_edge(clk);
      report "AHB WRITE done: addr=" & to_hstring(addr) &
             " data=" & to_hstring(data) severity note;
    end procedure;

    -- Helper: do one AHB read
    procedure do_ahb_read (
      addr : in std_logic_vector(31 downto 0)
    ) is
    begin
      ahb_req_addr   <= addr;
      ahb_start_read <= '1';
      wait until rising_edge(clk);
      ahb_start_read <= '0';
      -- Wait for model to finish
      wait until ahb_done = '1';
      wait until rising_edge(clk);
      report "AHB READ done: addr=" & to_hstring(addr) &
             " hrdata=" & to_hstring(ahb_resp_data) severity note;
    end procedure;

    -- Helper: do one processor write
    procedure do_proc_write (
      addr : in std_logic_vector(ABITS-1 downto 0);
      data : in std_logic_vector(31 downto 0)
    ) is
    begin
      wait until rising_edge(clk);
      proc_addr  <= addr;
      proc_wdata <= data;
      proc_ce1   <= '0';
      proc_wen   <= '0';
      proc_oen   <= '1';
      -- Hold for 2 APB cycles (setup + enable)
      wait for 2 * CLK_PERIOD;
      proc_ce1   <= '1';
      proc_wen   <= '1';
      wait for CLK_PERIOD;
      report "PROC WRITE done: addr=" & to_hstring("0000" & addr) &
             " data=" & to_hstring(data) severity note;
    end procedure;

    -- Helper: do one processor read
    procedure do_proc_read (
      addr : in std_logic_vector(ABITS-1 downto 0)
    ) is
    begin
      wait until rising_edge(clk);
      proc_addr  <= addr;
      proc_wdata <= (others => '0');
      proc_ce1   <= '0';
      proc_wen   <= '1';
      proc_oen   <= '0';
      wait for 2 * CLK_PERIOD;
      proc_ce1   <= '1';
      proc_oen   <= '1';
      wait for CLK_PERIOD;
      report "PROC READ done: addr=" & to_hstring("0000" & addr) &
             " proc_rdata=" & to_hstring(proc_rdata) severity note;
    end procedure;

  begin

    -- ── Phase 0: Reset ────────────────────────────────────────
    rst <= '0';
    wait for 5 * CLK_PERIOD;
    rst <= '1';
    wait for 5 * CLK_PERIOD;
    report "=== RESET RELEASED ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 1: Processor Write via APB Bridge
    -- Expect: psel=1, penable=1, paddr=0x10, pwdata=0xDEADBEEF
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 1: Processor WRITE ===" severity note;
    do_proc_write(
      std_logic_vector(to_unsigned(16#0000010#, ABITS)),
      x"DEADBEEF"
    );

    -- ══════════════════════════════════════════════════════════
    -- TEST 2: Processor Read via APB Bridge
    -- Expect: psel=1, penable=1, proc_rdata = APB slave response
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 2: Processor READ ===" severity note;
    do_proc_read(
      std_logic_vector(to_unsigned(16#0000020#, ABITS))
    );

    -- ══════════════════════════════════════════════════════════
    -- TEST 3: AHB Write → SRAM
    -- Expect: sram_addr driven, sram_wdata=0xCAFEBABE, sram_wen=0
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 3: AHB Master WRITE ===" severity note;
    do_ahb_write(x"40000040", x"CAFEBAABE");

    -- ══════════════════════════════════════════════════════════
    -- TEST 4: AHB Read ← SRAM
    -- Expect: sram_oen=0, hrdata = 0x80ABCDEF from SRAM model
    -- SRAM model returns: addr[7:0] & 0xABCDEF
    -- addr=0x40000080 → sram_addr lower bits = 0x80
    -- so hrdata should = 0x80ABCDEF
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 4: AHB Master READ ===" severity note;
    do_ahb_read(x"40000080");

    -- ══════════════════════════════════════════════════════════
    -- TEST 5: Back-to-back Processor Writes
    -- Expect: 3 APB write cycles back to back
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 5: Back-to-back Processor Writes ===" severity note;
    do_proc_write(
      std_logic_vector(to_unsigned(16#0000100#, ABITS)),
      x"11111111"
    );
    do_proc_write(
      std_logic_vector(to_unsigned(16#0000104#, ABITS)),
      x"22222222"
    );
    do_proc_write(
      std_logic_vector(to_unsigned(16#0000108#, ABITS)),
      x"33333333"
    );

    -- ══════════════════════════════════════════════════════════
    -- TEST 6: AHB Write followed immediately by AHB Read
    -- Write 0xABCD1234 then read it back
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 6: AHB Write then Read same address ===" severity note;
    do_ahb_write(x"400000C0", x"ABCD1234");
    do_ahb_read (x"400000C0");

    -- ══════════════════════════════════════════════════════════
    -- END
    -- ══════════════════════════════════════════════════════════
    wait for 10 * CLK_PERIOD;
    report "=== ALL TESTS COMPLETE ===" severity note;
    report "Simulation finished" severity failure;

  end process stimulus;

end architecture sim;
