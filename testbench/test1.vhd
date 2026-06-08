-- ============================================================
-- tb_bridge_top.vhd
-- Testbench for DUT: eth_module (bridge_top)
-- Simulator: VCS
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.bridge_pkg.all;

entity tb_bridge_top is
-- Testbench has no ports
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

  -- Clock and Reset
  signal clk  : std_logic := '0';
  signal rst  : std_logic := '0';

  -- AHB-Master outputs (TB drives these IN to DUT)
  signal hbusreq : std_ulogic                    := '0';
  signal hlock   : std_ulogic                    := '0';
  signal htrans  : std_logic_vector(1 downto 0)  := "00";
  signal haddr   : std_logic_vector(31 downto 0) := (others => '0');
  signal hwrite  : std_ulogic                    := '0';
  signal hsize   : std_logic_vector(2 downto 0)  := "010"; -- 32-bit word
  signal hburst  : std_logic_vector(2 downto 0)  := "000"; -- SINGLE
  signal hprot   : std_logic_vector(3 downto 0)  := "0011";
  signal hwdata  : std_logic_vector(31 downto 0) := (others => '0');

  -- AHB-Master inputs (DUT drives these OUT to TB)
  signal hgrant  : std_ulogic;
  signal hready  : std_ulogic;
  signal hresp   : std_logic_vector(1 downto 0);
  signal hrdata  : std_logic_vector(31 downto 0);

  -- APB-Slave outputs (TB drives prdata IN to DUT)
  signal prdata  : std_logic_vector(31 downto 0) := (others => '0');

  -- APB-Slave inputs (DUT drives these OUT to TB)
  signal psel    : std_ulogic;
  signal penable : std_ulogic;
  signal paddr   : std_logic_vector(31 downto 0);
  signal pwrite  : std_ulogic;
  signal pwdata  : std_logic_vector(31 downto 0);

  -- SRAM outputs (TB drives sram_rdata IN to DUT)
  signal sram_rdata : std_logic_vector(31 downto 0) := (others => '0');

  -- SRAM inputs (DUT drives these OUT to TB)
  signal sram_addr  : std_logic_vector(ABITS-1 downto 0);
  signal sram_wdata : std_logic_vector(31 downto 0);
  signal sram_ce1   : std_logic;
  signal sram_wen   : std_ulogic;
  signal sram_oen   : std_ulogic;

  -- Processor outputs (TB drives these IN to DUT)
  signal proc_wdata : std_logic_vector(31 downto 0)        := (others => '0');
  signal proc_addr  : std_logic_vector(ABITS-1 downto 0)   := (others => '0');
  signal proc_ce1   : std_logic                            := '1'; -- active low, deasserted
  signal proc_wen   : std_logic                            := '1'; -- active low, deasserted
  signal proc_oen   : std_logic                            := '1'; -- active low, deasserted

  -- Processor inputs (DUT drives this OUT to TB)
  signal proc_rdata : std_logic_vector(31 downto 0);

  -- Scan-test (DUT drives these OUT, TB just observes)
  signal testrst : std_ulogic;
  signal testen  : std_ulogic;
  signal testoen : std_ulogic;

  -- IRQ (DUT drives OUT, TB observes)
  signal irq : std_logic;

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
      -- AHB-Master output
      hbusreq    : in  std_ulogic;
      hlock      : in  std_ulogic;
      htrans     : in  std_logic_vector(1 downto 0);
      haddr      : in  std_logic_vector(31 downto 0);
      hwrite     : in  std_ulogic;
      hsize      : in  std_logic_vector(2 downto 0);
      hburst     : in  std_logic_vector(2 downto 0);
      hprot      : in  std_logic_vector(3 downto 0);
      hwdata     : in  std_logic_vector(31 downto 0);
      -- AHB-Master inputs
      hgrant     : out std_ulogic;
      hready     : out std_ulogic;
      hresp      : out std_logic_vector(1 downto 0);
      hrdata     : out std_logic_vector(31 downto 0);
      -- APB-Slave outputs
      prdata     : in  std_logic_vector(31 downto 0);
      -- APB-Slave inputs
      psel       : out std_ulogic;
      penable    : out std_ulogic;
      paddr      : out std_logic_vector(31 downto 0);
      pwrite     : out std_ulogic;
      pwdata     : out std_logic_vector(31 downto 0);
      -- SRAM outputs
      sram_rdata : in  std_logic_vector(31 downto 0);
      -- SRAM inputs
      sram_addr  : out std_logic_vector(ABITS-1 downto 0);
      sram_wdata : out std_logic_vector(31 downto 0);
      sram_ce1   : out std_logic;
      sram_wen   : out std_ulogic;
      sram_oen   : out std_ulogic;
      -- Processor outputs
      proc_wdata : in  std_logic_vector(31 downto 0);
      proc_addr  : in  std_logic_vector(ABITS-1 downto 0);
      proc_ce1   : in  std_logic;
      proc_wen   : in  std_logic;
      proc_oen   : in  std_logic;
      -- Processor inputs
      proc_rdata : out std_logic_vector(31 downto 0);
      -- Scan-test
      testrst    : out std_ulogic;
      testen     : out std_ulogic;
      testoen    : out std_ulogic;
      -- IRQ
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
  -- Clock Generation: 100 MHz (10ns period)
  -- ============================================================
  clk_gen : process
  begin
    clk <= '0';
    wait for CLK_PERIOD / 2;
    clk <= '1';
    wait for CLK_PERIOD / 2;
  end process;

  -- ============================================================
  -- Simple SRAM Model
  -- When sram_ce1=0 and sram_oen=0 (read), return fixed data
  -- ============================================================
  sram_model : process(sram_ce1, sram_oen, sram_addr)
  begin
    if sram_ce1 = '0' and sram_oen = '0' then
      -- Return address-dependent read data so we can verify
      sram_rdata <= std_logic_vector(
                      unsigned(sram_addr(7 downto 0)) &
                      x"AB" &
                      x"CD" &
                      x"EF"
                    );
    else
      sram_rdata <= (others => '0');
    end if;
  end process;

  -- ============================================================
  -- Simple APB Slave Model
  -- When psel=1 and penable=1, echo pwdata back as prdata
  -- ============================================================
  apb_slave_model : process(psel, penable, pwdata)
  begin
    if psel = '1' and penable = '1' then
      prdata <= pwdata; -- Echo write data back as read data
    else
      prdata <= x"FACE_CAFE";
    end if;
  end process;

  -- ============================================================
  -- Stimulus Process
  -- ============================================================
  stimulus : process
  begin

    -- ── Phase 0: Reset ────────────────────────────────────────
    rst <= '0';
    wait for 3 * CLK_PERIOD;
    rst <= '1';    -- Release reset (active high rst in DUT)
    wait for 2 * CLK_PERIOD;

    report "=== RESET RELEASED ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 1: PROCESSOR WRITE via APB Bridge
    -- Simulate leon3 writing 0xDEADBEEF to address 0x0000010
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 1: Processor WRITE ===" severity note;

    wait until rising_edge(clk);
    proc_addr  <= std_logic_vector(to_unsigned(16#0000010#, ABITS));
    proc_wdata <= x"DEADBEEF";
    proc_ce1   <= '0';   -- Assert chip enable (active low)
    proc_wen   <= '0';   -- Assert write enable (active low)
    proc_oen   <= '1';   -- OE deasserted during write

    -- Hold for 3 cycles (APB takes 2-cycle setup+enable)
    wait for 3 * CLK_PERIOD;

    -- Deassert
    proc_ce1   <= '1';
    proc_wen   <= '1';
    wait for 2 * CLK_PERIOD;

    report "=== TEST 1 DONE: check psel/penable/pwdata on waveform ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 2: PROCESSOR READ via APB Bridge
    -- Simulate leon3 reading from address 0x0000020
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 2: Processor READ ===" severity note;

    wait until rising_edge(clk);
    proc_addr  <= std_logic_vector(to_unsigned(16#0000020#, ABITS));
    proc_wdata <= (others => '0');
    proc_ce1   <= '0';   -- Assert chip enable
    proc_wen   <= '1';   -- WE deasserted = READ
    proc_oen   <= '0';   -- Assert output enable (active low)

    wait for 3 * CLK_PERIOD;

    -- Deassert
    proc_ce1   <= '1';
    proc_oen   <= '1';
    wait for 2 * CLK_PERIOD;

    report "=== TEST 2 DONE: check proc_rdata on waveform ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 3: AHB MASTER WRITE via AHB Bridge → SRAM
    -- Simulate GRETH MAC writing 0xCAFEBABE to AHB address
    -- AHB protocol: Address phase then Data phase
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 3: AHB Master WRITE ===" severity note;

    wait until rising_edge(clk);

    -- ── Address Phase ─────────────────────────────────────────
    hbusreq <= '1';
    hlock   <= '0';
    htrans  <= "10";          -- NONSEQ transfer
    haddr   <= x"40000040";   -- AHB address
    hwrite  <= '1';           -- Write
    hsize   <= "010";         -- 32-bit word
    hburst  <= "000";         -- SINGLE burst
    hprot   <= "0011";

    wait for CLK_PERIOD;      -- Address phase lasts 1 cycle

    -- ── Data Phase ────────────────────────────────────────────
    hwdata  <= x"CAFEBABBE";  -- Write data presented in data phase
    htrans  <= "00";          -- IDLE after single transfer

    wait for 2 * CLK_PERIOD;

    -- Deassert bus request
    hbusreq <= '0';
    hwrite  <= '0';
    wait for 2 * CLK_PERIOD;

    report "=== TEST 3 DONE: check sram_addr/sram_wdata/sram_wen on waveform ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 4: AHB MASTER READ via AHB Bridge ← SRAM
    -- Simulate GRETH MAC reading from AHB address
    -- SRAM model will return (addr[7:0] & 0xABCDEF)
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 4: AHB Master READ ===" severity note;

    wait until rising_edge(clk);

    -- ── Address Phase ─────────────────────────────────────────
    hbusreq <= '1';
    htrans  <= "10";          -- NONSEQ
    haddr   <= x"40000080";   -- AHB read address
    hwrite  <= '0';           -- Read
    hsize   <= "010";
    hburst  <= "000";

    wait for CLK_PERIOD;

    -- ── Data Phase ────────────────────────────────────────────
    htrans  <= "00";          -- IDLE

    wait for 3 * CLK_PERIOD;

    -- Deassert
    hbusreq <= '0';
    wait for 2 * CLK_PERIOD;

    report "=== TEST 4 DONE: check hrdata on waveform, expect 0x80ABCDEF ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- TEST 5: BACK-TO-BACK Processor Writes
    -- Two consecutive writes to different addresses
    -- ══════════════════════════════════════════════════════════
    report "=== TEST 5: Back-to-back Processor Writes ===" severity note;

    -- Write 1
    wait until rising_edge(clk);
    proc_addr  <= std_logic_vector(to_unsigned(16#0000100#, ABITS));
    proc_wdata <= x"11111111";
    proc_ce1   <= '0';
    proc_wen   <= '0';
    proc_oen   <= '1';
    wait for CLK_PERIOD;

    -- Write 2 (back to back, no gap)
    proc_addr  <= std_logic_vector(to_unsigned(16#0000104#, ABITS));
    proc_wdata <= x"22222222";
    wait for CLK_PERIOD;

    -- Write 3
    proc_addr  <= std_logic_vector(to_unsigned(16#0000108#, ABITS));
    proc_wdata <= x"33333333";
    wait for CLK_PERIOD;

    -- Deassert
    proc_ce1   <= '1';
    proc_wen   <= '1';
    wait for 3 * CLK_PERIOD;

    report "=== TEST 5 DONE: check psel/pwdata toggling on waveform ===" severity note;

    -- ══════════════════════════════════════════════════════════
    -- END OF TESTS
    -- ══════════════════════════════════════════════════════════
    wait for 10 * CLK_PERIOD;
    report "=== ALL TESTS COMPLETE ===" severity note;
    report "Simulation finished" severity failure; -- Forces VCS to stop

  end process stimulus;

  -- ============================================================
  -- VCD Dump for waveform viewing (VCS compatible)
  -- ============================================================
  -- Use $dumpfile/$dumpvars in a verilog initial block
  -- OR use the VCS command line: vcs -debug_all +vcs+dumpvars
  -- For VHDL-only VCS, use UCLI or the following:
  --   vcd on  / vcd file dump.vcd  (in UCLIscript)
  -- ============================================================

end architecture sim;
