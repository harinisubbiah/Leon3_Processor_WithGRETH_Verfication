------------------------------------------------------------------------------
--  TESTBENCH: tb_bridge_eth_sram
--
--  DUT:  AHB Memory Bridge (ahb_mem_bridge) + APB Memory Bridge (mem_apb_bridge)
--        + Ethernet SRAM (eth_sram model)
--
--  PURPOSE:
--    Standalone test to verify:
--      1. AHB side  – GRETH (as AHB master) READ and WRITE to ETH-SRAM via
--                     ahb_mem_bridge.  Results visible on HRDATA / HWDATA.
--      2. APB side  – Processor (LEON3 proxy) READ and WRITE to GRETH control
--                     & status registers via mem_apb_bridge.
--                     Results visible on PRDATA / PWDATA.
--
--  SIGNAL NAMING follows the bridge package definitions (Image 1):
--    AHB bridge package  (ahb_bridge_pkg):
--      ahbmo  : ahb_mst_out_type   -- GRETH drives this
--      ahbmi  : ahb_mst_in_type    -- GRETH receives this (HRDATA lives here)
--      memi   : mem_mst_in_type    -- SRAM read-data back to bridge
--      memo   : mem_mst_out_type   -- bridge drives SRAM (addr,data,wen,oen,ce)
--
--    APB bridge package  (apb_bridge_pkg):
--      apbo   : apb_slv_out_type   -- GRETH drives this (PRDATA lives here)
--      apbi   : apb_slv_in_type    -- processor drives this (PWDATA lives here)
--      memi   : mem_slv_in_type    -- processor read-data back to bridge
--      memo   : mem_slv_out_type   -- bridge drives GRETH regs (wxdata,oen…)
--
--  CLOCK: 40 MHz  (25 ns period) – representative of a 40 MHz AHB clock
--
--  TESTCASES:
--    TC1  AHB Write – GRETH writes 0xDEADBEEF to SRAM address 0x40000100
--    TC2  AHB Read  – GRETH reads  back from the same address → HRDATA check
--    TC3  APB Write – Processor writes CTRL register (offset 0x00) to enable
--                     TX and RX (PWDATA = 0x00000003)
--    TC4  APB Read  – Processor reads STATUS register (offset 0x04) → PRDATA check
--    TC5  AHB Burst – GRETH writes 4-word burst then reads it back
--
--  GRETH ↔ bridge protocol notes (AMBA AHB 2.0):
--    • Address phase  : HTRANS=NONSEQ, HWRITE, HADDR valid, HREADY=1
--    • Data phase     : HWDATA valid on next clock (writes); HRDATA sampled
--                       when slave asserts HREADY=1 (reads)
--    • Bridge responds with HREADY=0 (wait) for one cycle to model SRAM latency,
--      then HREADY=1 with valid HRDATA.
--
--  APB protocol notes (AMBA APB 2.0):
--    • SETUP  phase : PSEL=1, PENABLE=0
--    • ENABLE phase : PSEL=1, PENABLE=1  → data transferred
--
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

-- ============================================================
--  PACKAGE: local type definitions matching the bridge packages
--  (Mirrors Image 1 record definitions, self-contained so the TB
--   can compile without the full GRLIB library.)
-- ============================================================
package bridge_tb_pkg is

  constant ABITS : integer := 28;   -- memory address bits (from bridge pkg)
  constant DW    : integer := 32;   -- data width

  -- ---------- AHB memory bridge types ----------
  type mem_mst_out_type is record   -- bridge → SRAM
    mem_wdata : std_logic_vector(DW-1 downto 0);
    mem_addr  : std_logic_vector(ABITS-1 downto 0);
    mem_ce1   : std_logic;
    mem_wen   : std_logic;
    mem_oen   : std_logic;
  end record;

  type mem_mst_in_type is record    -- SRAM → bridge
    mem_rdata : std_logic_vector(DW-1 downto 0);
  end record;

  -- Simplified AHB master output (GRETH → bridge)
  type ahb_mst_out_type is record
    hbusreq : std_logic;
    htrans  : std_logic_vector(1 downto 0);
    haddr   : std_logic_vector(31 downto 0);
    hwrite  : std_logic;
    hsize   : std_logic_vector(2 downto 0);
    hburst  : std_logic_vector(2 downto 0);
    hwdata  : std_logic_vector(DW-1 downto 0);
  end record;

  -- Simplified AHB master input (bridge → GRETH)
  type ahb_mst_in_type is record
    hgrant  : std_logic;
    hready  : std_logic;
    hresp   : std_logic_vector(1 downto 0);
    hrdata  : std_logic_vector(DW-1 downto 0);
  end record;

  -- ---------- APB memory bridge types ----------
  type mem_slv_in_type is record    -- processor → APB bridge input
    mem_wxdata : std_logic_vector(DW-1 downto 0);
    mem_addr   : std_logic_vector(ABITS-1 downto 0);
    mem_ce1    : std_logic;
    mem_wen    : std_logic;
    mem_oen    : std_logic;
  end record;

  type mem_slv_out_type is record   -- APB bridge → processor (read data)
    mem_rxdata : std_logic_vector(DW-1 downto 0);
  end record;

  -- Simplified APB slave output (GRETH → processor)
  type apb_slv_out_type is record
    prdata : std_logic_vector(DW-1 downto 0);
  end record;

  -- Simplified APB slave input (processor → GRETH)
  type apb_slv_in_type is record
    psel    : std_logic;
    penable : std_logic;
    paddr   : std_logic_vector(31 downto 0);
    pwrite  : std_logic;
    pwdata  : std_logic_vector(DW-1 downto 0);
  end record;

  -- AHB HTRANS encoding
  constant HTRANS_IDLE   : std_logic_vector(1 downto 0) := "00";
  constant HTRANS_BUSY   : std_logic_vector(1 downto 0) := "01";
  constant HTRANS_NONSEQ : std_logic_vector(1 downto 0) := "10";
  constant HTRANS_SEQ    : std_logic_vector(1 downto 0) := "11";

  -- HRESP
  constant HRESP_OKAY    : std_logic_vector(1 downto 0) := "00";

  -- HSIZE
  constant HSIZE_WORD    : std_logic_vector(2 downto 0) := "010";

  -- HBURST
  constant HBURST_SINGLE : std_logic_vector(2 downto 0) := "000";
  constant HBURST_INCR4  : std_logic_vector(2 downto 0) := "011";

end package bridge_tb_pkg;

-- ============================================================
--  ENTITY: ahb_mem_bridge   (BEHAVIORAL MODEL of the DUT)
--
--  This is a behavioral implementation of the bridge shown in Image 1.
--  It accepts AHB master transactions from GRETH and translates them
--  into the mem_mst_out_type memory interface for the ETH-SRAM.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.bridge_tb_pkg.all;

entity ahb_mem_bridge is
  generic (
    SRAMDEPTH : integer := 19;
    ABITS     : integer := 28
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;
    -- AHB master port (from GRETH)
    ahbmo : in  ahb_mst_out_type;
    ahbmi : out ahb_mst_in_type;
    -- Memory port (to/from ETH SRAM)
    memi  : in  mem_mst_in_type;
    memo  : out mem_mst_out_type
  );
end ahb_mem_bridge;

architecture rtl of ahb_mem_bridge is

  type state_type is (IDLE, ADDR_PHASE, DATA_WRITE, DATA_READ, WAIT_ACK);
  signal state      : state_type := IDLE;

  signal reg_addr   : std_logic_vector(31 downto 0) := (others => '0');
  signal reg_write  : std_logic := '0';
  signal reg_htrans : std_logic_vector(1 downto 0) := HTRANS_IDLE;

  -- internal memory control
  signal s_ce1 : std_logic := '1';  -- active-low chip enable
  signal s_wen : std_logic := '1';  -- active-low write enable
  signal s_oen : std_logic := '1';  -- active-low output enable
  signal s_wdata : std_logic_vector(31 downto 0) := (others => '0');
  signal s_addr  : std_logic_vector(ABITS-1 downto 0) := (others => '0');

  signal s_hready : std_logic := '1';
  signal s_hrdata : std_logic_vector(31 downto 0) := (others => '0');

begin

  -- Drive AHB master input
  ahbmi.hgrant <= '1';          -- always granted (single master)
  ahbmi.hready <= s_hready;
  ahbmi.hresp  <= HRESP_OKAY;
  ahbmi.hrdata <= s_hrdata;

  -- Drive memory outputs
  memo.mem_wdata <= s_wdata;
  memo.mem_addr  <= s_addr;
  memo.mem_ce1   <= s_ce1;
  memo.mem_wen   <= s_wen;
  memo.mem_oen   <= s_oen;

  process(clk, rst)
  begin
    if rst = '0' then
      state    <= IDLE;
      s_hready <= '1';
      s_ce1    <= '1';
      s_wen    <= '1';
      s_oen    <= '1';
      s_hrdata <= (others => '0');

    elsif rising_edge(clk) then
      case state is

        -- -------------------------------------------------------
        when IDLE =>
          s_ce1    <= '1';
          s_wen    <= '1';
          s_oen    <= '1';
          s_hready <= '1';
          if ahbmo.htrans = HTRANS_NONSEQ or ahbmo.htrans = HTRANS_SEQ then
            -- Latch address phase
            reg_addr   <= ahbmo.haddr;
            reg_write  <= ahbmo.hwrite;
            reg_htrans <= ahbmo.htrans;
            s_addr     <= ahbmo.haddr(ABITS-1 downto 0);
            s_hready   <= '0';   -- insert one wait state (SRAM latency model)
            state      <= ADDR_PHASE;
          end if;

        -- -------------------------------------------------------
        -- Address phase captured; decide read or write on next clk
        when ADDR_PHASE =>
          s_ce1 <= '0';   -- select SRAM
          if reg_write = '1' then
            -- Write: latch HWDATA, assert WEN
            s_wdata  <= ahbmo.hwdata;
            s_wen    <= '0';
            s_oen    <= '1';
            state    <= DATA_WRITE;
          else
            -- Read: assert OEN, deassert WEN
            s_wen    <= '1';
            s_oen    <= '0';
            state    <= DATA_READ;
          end if;

        -- -------------------------------------------------------
        when DATA_WRITE =>
          -- Data written to SRAM; deassert strobes and ack GRETH
          s_wen    <= '1';
          s_ce1    <= '1';
          s_hready <= '1';
          -- Check if another transfer is queued
          if ahbmo.htrans = HTRANS_NONSEQ or ahbmo.htrans = HTRANS_SEQ then
            reg_addr  <= ahbmo.haddr;
            reg_write <= ahbmo.hwrite;
            s_addr    <= ahbmo.haddr(ABITS-1 downto 0);
            s_hready  <= '0';
            state     <= ADDR_PHASE;
          else
            state <= IDLE;
          end if;

        -- -------------------------------------------------------
        when DATA_READ =>
          -- Capture SRAM read data and return to GRETH
          s_hrdata <= memi.mem_rdata;
          s_oen    <= '1';
          s_ce1    <= '1';
          s_hready <= '1';
          if ahbmo.htrans = HTRANS_NONSEQ or ahbmo.htrans = HTRANS_SEQ then
            reg_addr  <= ahbmo.haddr;
            reg_write <= ahbmo.hwrite;
            s_addr    <= ahbmo.haddr(ABITS-1 downto 0);
            s_hready  <= '0';
            state     <= ADDR_PHASE;
          else
            state <= IDLE;
          end if;

        when others => state <= IDLE;
      end case;
    end if;
  end process;

end rtl;


-- ============================================================
--  ENTITY: mem_apb_bridge   (BEHAVIORAL MODEL of the DUT)
--
--  Translates APB slave transactions (from processor) into the
--  mem_slv_in_type memory interface used by GRETH registers.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.bridge_tb_pkg.all;

entity mem_apb_bridge is
  generic (
    ABITS : integer := 28
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;
    -- APB slave port (processor side)
    apbo  : out apb_slv_out_type;   -- GRETH register read-back
    apbi  : in  apb_slv_in_type;    -- processor writes/reads
    -- Memory interface (to/from GRETH register map model)
    memi  : in  mem_slv_out_type;   -- register read data
    memo  : out mem_slv_in_type     -- register address/data/strobes
  );
end mem_apb_bridge;

architecture rtl of mem_apb_bridge is
  signal s_prdata   : std_logic_vector(31 downto 0) := (others => '0');
  signal s_wxdata   : std_logic_vector(31 downto 0) := (others => '0');
  signal s_addr     : std_logic_vector(27 downto 0) := (others => '0');
  signal s_ce1      : std_logic := '1';
  signal s_wen      : std_logic := '1';
  signal s_oen      : std_logic := '1';
begin

  apbo.prdata      <= s_prdata;
  memo.mem_wxdata  <= s_wxdata;
  memo.mem_addr    <= s_addr;
  memo.mem_ce1     <= s_ce1;
  memo.mem_wen     <= s_wen;
  memo.mem_oen     <= s_oen;

  process(clk, rst)
  begin
    if rst = '0' then
      s_prdata <= (others => '0');
      s_ce1    <= '1';
      s_wen    <= '1';
      s_oen    <= '1';
    elsif rising_edge(clk) then
      -- Default: deassert strobes
      s_ce1 <= '1';
      s_wen <= '1';
      s_oen <= '1';

      -- APB ENABLE phase: transfer happens
      if apbi.psel = '1' and apbi.penable = '1' then
        s_addr <= apbi.paddr(27 downto 0);
        s_ce1  <= '0';
        if apbi.pwrite = '1' then
          -- Write: forward PWDATA to register memory
          s_wxdata <= apbi.pwdata;
          s_wen    <= '0';
        else
          -- Read: assert OEN, capture register data
          s_oen    <= '0';
          s_prdata <= memi.mem_rxdata;
        end if;
      end if;
    end if;
  end process;

end rtl;


-- ============================================================
--  ENTITY: eth_sram   (BEHAVIORAL SRAM MODEL – ETH SRAM)
--
--  Simple synchronous SRAM with parameterizable depth/width.
--  Represents the Ethernet SRAM sitting between the bridge and
--  the AHB bus.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.bridge_tb_pkg.all;

entity eth_sram is
  generic (
    DEPTH : integer := 512;   -- number of 32-bit words
    ABITS : integer := 28
  );
  port (
    clk    : in  std_logic;
    -- Memory interface (from bridge)
    addr   : in  std_logic_vector(ABITS-1 downto 0);
    wdata  : in  std_logic_vector(31 downto 0);
    rdata  : out std_logic_vector(31 downto 0);
    ce1    : in  std_logic;   -- active-low chip enable
    wen    : in  std_logic;   -- active-low write enable
    oen    : in  std_logic    -- active-low output enable
  );
end eth_sram;

architecture rtl of eth_sram is
  type ram_type is array (0 to DEPTH-1) of std_logic_vector(31 downto 0);
  signal ram : ram_type := (others => (others => '0'));
  signal s_rdata : std_logic_vector(31 downto 0) := (others => '0');
begin

  rdata <= s_rdata when oen = '0' else (others => 'Z');

  process(clk)
    variable idx : integer;
  begin
    if rising_edge(clk) then
      if ce1 = '0' then
        idx := to_integer(unsigned(addr(8 downto 0)));  -- 9-bit index for 512 words
        if wen = '0' then
          ram(idx) <= wdata;
        end if;
        s_rdata <= ram(idx);
      end if;
    end if;
  end process;

end rtl;


-- ============================================================
--  ENTITY: greth_reg_model   (BEHAVIORAL MODEL of GRETH registers)
--
--  A minimal register file modelling the GRETH APB register map
--  (CTRL at 0x00, STATUS at 0x04) so the APB bridge has something
--  to read and write.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.bridge_tb_pkg.all;

entity greth_reg_model is
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;
    memi  : in  mem_slv_in_type;    -- from APB bridge (write path)
    memo  : out mem_slv_out_type    -- to APB bridge   (read path)
  );
end greth_reg_model;

architecture rtl of greth_reg_model is
  -- GRETH CTRL register (offset 0x00) – reset value gives SP=1 per GRLIB spec
  signal reg_ctrl   : std_logic_vector(31 downto 0) := x"00000080";
  -- GRETH STATUS register (offset 0x04) – NRD=0b0000 (128 descriptors)
  signal reg_status : std_logic_vector(31 downto 0) := x"00000000";
  -- GRETH MAC MSB (offset 0x08)
  signal reg_macmsb : std_logic_vector(31 downto 0) := x"00001234";
  -- GRETH MAC LSB (offset 0x0C)
  signal reg_maclsb : std_logic_vector(31 downto 0) := x"56789ABC";

  signal s_rxdata : std_logic_vector(31 downto 0) := (others => '0');
begin

  memo.mem_rxdata <= s_rxdata;

  process(clk, rst)
    variable offset : integer;
  begin
    if rst = '0' then
      reg_ctrl   <= x"00000080";
      reg_status <= x"00000000";
    elsif rising_edge(clk) then
      if memi.mem_ce1 = '0' then
        offset := to_integer(unsigned(memi.mem_addr(7 downto 0)));
        -- Write path
        if memi.mem_wen = '0' then
          case offset is
            when 16#00# => reg_ctrl   <= memi.mem_wxdata;
            when 16#04# =>
              -- STATUS bits are write-1-to-clear; simulate a few flags being set
              reg_status <= reg_status and (not memi.mem_wxdata);
            when others => null;
          end case;
        end if;
        -- Read path
        if memi.mem_oen = '0' then
          case offset is
            when 16#00# => s_rxdata <= reg_ctrl;
            when 16#04# => s_rxdata <= reg_status;
            when 16#08# => s_rxdata <= reg_macmsb;
            when 16#0C# => s_rxdata <= reg_maclsb;
            when others => s_rxdata <= x"DEADC0DE";
          end case;
        end if;
      end if;
    end if;
  end process;

end rtl;


-- ============================================================
--  TESTBENCH TOP: tb_bridge_eth_sram
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.bridge_tb_pkg.all;

entity tb_bridge_eth_sram is
end tb_bridge_eth_sram;

architecture sim of tb_bridge_eth_sram is

  -- -------------------------------------------------------
  --  Clock / reset
  -- -------------------------------------------------------
  constant CLK_PERIOD : time := 25 ns;   -- 40 MHz AHB clock
  signal clk : std_logic := '0';
  signal rst : std_logic := '0';

  -- -------------------------------------------------------
  --  AHB bridge signals
  -- -------------------------------------------------------
  signal ahbmo : ahb_mst_out_type;      -- GRETH → AHB bridge
  signal ahbmi : ahb_mst_in_type;       -- AHB bridge → GRETH

  signal ahb_memi : mem_mst_in_type;    -- ETH SRAM → AHB bridge
  signal ahb_memo : mem_mst_out_type;   -- AHB bridge → ETH SRAM

  -- -------------------------------------------------------
  --  APB bridge signals
  -- -------------------------------------------------------
  signal apbi : apb_slv_in_type;        -- processor → APB bridge
  signal apbo : apb_slv_out_type;       -- APB bridge (GRETH) → processor

  signal apb_memi : mem_slv_out_type;   -- GRETH regs → APB bridge
  signal apb_memo : mem_slv_in_type;    -- APB bridge → GRETH regs

  -- -------------------------------------------------------
  --  Test observation aliases  (these are the signals called
  --  out in the test specification)
  -- -------------------------------------------------------
  alias HRDATA  : std_logic_vector(31 downto 0) is ahbmi.hrdata;
  alias HWDATA  : std_logic_vector(31 downto 0) is ahbmo.hwdata;
  alias PRDATA  : std_logic_vector(31 downto 0) is apbo.prdata;
  alias PWDATA  : std_logic_vector(31 downto 0) is apbi.pwdata;

  -- -------------------------------------------------------
  --  Test result tracking
  -- -------------------------------------------------------
  signal test_pass  : integer := 0;
  signal test_fail  : integer := 0;

  -- -------------------------------------------------------
  --  Helper: print to transcript
  -- -------------------------------------------------------
  procedure print(msg : string) is
    variable l : line;
  begin
    write(l, now, right, 12);
    write(l, string'("  "));
    write(l, msg);
    writeline(output, l);
  end procedure;

  -- -------------------------------------------------------
  --  Helper: check a value
  -- -------------------------------------------------------
  procedure check(
    signal pass : inout integer;
    signal fail : inout integer;
    actual   : std_logic_vector;
    expected : std_logic_vector;
    msg      : string) is
    variable l : line;
  begin
    if actual = expected then
      pass <= pass + 1;
      write(l, now, right, 12);
      write(l, string'("  PASS: ")); write(l, msg);
      writeline(output, l);
    else
      fail <= fail + 1;
      write(l, now, right, 12);
      write(l, string'("  FAIL: ")); write(l, msg);
      write(l, string'("  got="));
      write(l, actual);
      write(l, string'("  exp="));
      write(l, expected);
      writeline(output, l);
    end if;
  end procedure;

  -- -------------------------------------------------------
  --  Helper: AHB single-beat write transaction
  --  Models GRETH as AHB master issuing a NONSEQ write.
  -- -------------------------------------------------------
  procedure ahb_write(
    signal clk   : in  std_logic;
    signal ahbmo : out ahb_mst_out_type;
    signal ahbmi : in  ahb_mst_in_type;
    addr  : std_logic_vector(31 downto 0);
    data  : std_logic_vector(31 downto 0)) is
  begin
    -- Address phase
    wait until rising_edge(clk);
    ahbmo.hbusreq <= '1';
    ahbmo.htrans  <= HTRANS_NONSEQ;
    ahbmo.haddr   <= addr;
    ahbmo.hwrite  <= '1';
    ahbmo.hsize   <= HSIZE_WORD;
    ahbmo.hburst  <= HBURST_SINGLE;
    ahbmo.hwdata  <= data;

    -- Wait for bridge to assert HREADY
    loop
      wait until rising_edge(clk);
      exit when ahbmi.hready = '1';
    end loop;

    -- End transaction
    ahbmo.htrans  <= HTRANS_IDLE;
    ahbmo.hbusreq <= '0';
    ahbmo.hwrite  <= '0';
    wait until rising_edge(clk);
  end procedure;

  -- -------------------------------------------------------
  --  Helper: AHB single-beat read transaction
  -- -------------------------------------------------------
  procedure ahb_read(
    signal clk   : in  std_logic;
    signal ahbmo : out ahb_mst_out_type;
    signal ahbmi : in  ahb_mst_in_type;
    addr  : std_logic_vector(31 downto 0)) is
  begin
    -- Address phase
    wait until rising_edge(clk);
    ahbmo.hbusreq <= '1';
    ahbmo.htrans  <= HTRANS_NONSEQ;
    ahbmo.haddr   <= addr;
    ahbmo.hwrite  <= '0';
    ahbmo.hsize   <= HSIZE_WORD;
    ahbmo.hburst  <= HBURST_SINGLE;

    -- Wait for bridge to assert HREADY (data phase complete)
    loop
      wait until rising_edge(clk);
      exit when ahbmi.hready = '1';
    end loop;

    -- End transaction
    ahbmo.htrans  <= HTRANS_IDLE;
    ahbmo.hbusreq <= '0';
    wait until rising_edge(clk);
  end procedure;

  -- -------------------------------------------------------
  --  Helper: APB write  (SETUP + ENABLE phases)
  -- -------------------------------------------------------
  procedure apb_write(
    signal clk  : in  std_logic;
    signal apbi : out apb_slv_in_type;
    addr : std_logic_vector(31 downto 0);
    data : std_logic_vector(31 downto 0)) is
  begin
    -- SETUP phase
    wait until rising_edge(clk);
    apbi.psel    <= '1';
    apbi.penable <= '0';
    apbi.paddr   <= addr;
    apbi.pwrite  <= '1';
    apbi.pwdata  <= data;
    -- ENABLE phase
    wait until rising_edge(clk);
    apbi.penable <= '1';
    wait until rising_edge(clk);
    -- Deassert
    apbi.psel    <= '0';
    apbi.penable <= '0';
    apbi.pwrite  <= '0';
    wait until rising_edge(clk);
  end procedure;

  -- -------------------------------------------------------
  --  Helper: APB read
  -- -------------------------------------------------------
  procedure apb_read(
    signal clk  : in  std_logic;
    signal apbi : out apb_slv_in_type;
    addr : std_logic_vector(31 downto 0)) is
  begin
    -- SETUP phase
    wait until rising_edge(clk);
    apbi.psel    <= '1';
    apbi.penable <= '0';
    apbi.paddr   <= addr;
    apbi.pwrite  <= '0';
    apbi.pwdata  <= (others => '0');
    -- ENABLE phase
    wait until rising_edge(clk);
    apbi.penable <= '1';
    wait until rising_edge(clk);
    -- Deassert
    apbi.psel    <= '0';
    apbi.penable <= '0';
    wait until rising_edge(clk);
  end procedure;

begin

  -- ===================================================
  --  Clock generator
  -- ===================================================
  clk <= not clk after CLK_PERIOD / 2;

  -- ===================================================
  --  DUT instantiations
  -- ===================================================

  -- AHB Memory Bridge
  U_AHB_BRIDGE : entity work.ahb_mem_bridge
    port map (
      clk   => clk,
      rst   => rst,
      ahbmo => ahbmo,
      ahbmi => ahbmi,
      memi  => ahb_memi,
      memo  => ahb_memo
    );

  -- ETH SRAM (connected to AHB bridge)
  U_ETH_SRAM : entity work.eth_sram
    port map (
      clk   => clk,
      addr  => ahb_memo.mem_addr,
      wdata => ahb_memo.mem_wdata,
      rdata => ahb_memi.mem_rdata,
      ce1   => ahb_memo.mem_ce1,
      wen   => ahb_memo.mem_wen,
      oen   => ahb_memo.mem_oen
    );

  -- APB Memory Bridge
  U_APB_BRIDGE : entity work.mem_apb_bridge
    port map (
      clk   => clk,
      rst   => rst,
      apbo  => apbo,
      apbi  => apbi,
      memi  => apb_memi,
      memo  => apb_memo
    );

  -- GRETH Register Model (connected to APB bridge)
  U_GRETH_REGS : entity work.greth_reg_model
    port map (
      clk   => clk,
      rst   => rst,
      memi  => apb_memo,
      memo  => apb_memi
    );

  -- ===================================================
  --  STIMULUS PROCESS
  -- ===================================================
  STIMULUS : process
  begin

    -- ---------------------------------------------------
    --  Initialise all driven signals
    -- ---------------------------------------------------
    ahbmo.hbusreq <= '0';
    ahbmo.htrans  <= HTRANS_IDLE;
    ahbmo.haddr   <= (others => '0');
    ahbmo.hwrite  <= '0';
    ahbmo.hsize   <= HSIZE_WORD;
    ahbmo.hburst  <= HBURST_SINGLE;
    ahbmo.hwdata  <= (others => '0');

    apbi.psel    <= '0';
    apbi.penable <= '0';
    apbi.paddr   <= (others => '0');
    apbi.pwrite  <= '0';
    apbi.pwdata  <= (others => '0');

    -- ---------------------------------------------------
    --  Reset sequence  (active-low reset held for 4 clocks)
    -- ---------------------------------------------------
    rst <= '0';
    wait for CLK_PERIOD * 4;
    rst <= '1';
    wait for CLK_PERIOD * 2;

    print("=================================================");
    print("  TB_BRIDGE_ETH_SRAM  –  Starting test sequence ");
    print("=================================================");

    -- ===================================================
    --  TC1 – AHB WRITE
    --  GRETH (AHB master) writes 0xDEADBEEF to ETH SRAM
    --  at address 0x40000100.  Observe HWDATA on the bus.
    -- ===================================================
    print("--- TC1: AHB Write  0xDEADBEEF → addr 0x40000100 ---");
    ahb_write(clk, ahbmo, ahbmi,
              addr => x"40000100",
              data => x"DEADBEEF");

    -- Allow one idle cycle for SRAM to settle
    wait until rising_edge(clk);

    -- Check that HWDATA carried the expected value during write
    check(test_pass, test_fail,
          HWDATA, x"DEADBEEF",
          "TC1: HWDATA = 0xDEADBEEF during AHB write");

    -- ===================================================
    --  TC2 – AHB READ
    --  GRETH reads back from the same address.
    --  HRDATA should return 0xDEADBEEF.
    -- ===================================================
    print("--- TC2: AHB Read from addr 0x40000100 ---");
    ahb_read(clk, ahbmo, ahbmi, addr => x"40000100");

    check(test_pass, test_fail,
          HRDATA, x"DEADBEEF",
          "TC2: HRDATA = 0xDEADBEEF after AHB read");

    -- ===================================================
    --  TC3 – APB WRITE (CTRL register)
    --  Processor writes 0x00000003 to GRETH CTRL (offset 0x00)
    --  to enable TX (bit 0) and RX (bit 1).
    --  Observe PWDATA = 0x00000003 on the bus.
    -- ===================================================
    print("--- TC3: APB Write  CTRL ← 0x00000003 (RE | TE) ---");
    apb_write(clk, apbi,
              addr => x"00000000",    -- GRETH CTRL register offset 0x00
              data => x"00000003");

    check(test_pass, test_fail,
          PWDATA, x"00000003",
          "TC3: PWDATA = 0x00000003 during APB write to CTRL");

    -- ===================================================
    --  TC4 – APB READ (STATUS register)
    --  Processor reads GRETH STATUS (offset 0x04).
    --  Register model returns 0x00000000 (no errors/interrupts).
    -- ===================================================
    print("--- TC4: APB Read  STATUS register (offset 0x04) ---");
    apb_read(clk, apbi, addr => x"00000004");

    check(test_pass, test_fail,
          PRDATA, x"00000000",
          "TC4: PRDATA = 0x00000000 reading GRETH STATUS");

    -- ===================================================
    --  TC5 – AHB BURST WRITE then BURST READ (4-beat INCR4)
    --  GRETH writes four words starting at 0x40000200.
    --  Then reads them back. Verifies HRDATA for each word.
    -- ===================================================
    print("--- TC5: AHB 4-beat burst write ---");

    -- Beat 1 – NONSEQ
    wait until rising_edge(clk);
    ahbmo.hbusreq <= '1';
    ahbmo.htrans  <= HTRANS_NONSEQ;
    ahbmo.haddr   <= x"40000200";
    ahbmo.hwrite  <= '1';
    ahbmo.hsize   <= HSIZE_WORD;
    ahbmo.hburst  <= HBURST_INCR4;
    ahbmo.hwdata  <= x"AABBCCDD";
    loop
      wait until rising_edge(clk);
      exit when ahbmi.hready = '1';
    end loop;
    check(test_pass, test_fail,
          HWDATA, x"AABBCCDD",
          "TC5-b1: HWDATA burst beat 1");

    -- Beat 2 – SEQ
    ahbmo.htrans  <= HTRANS_SEQ;
    ahbmo.haddr   <= x"40000204";
    ahbmo.hwdata  <= x"11223344";
    loop
      wait until rising_edge(clk);
      exit when ahbmi.hready = '1';
    end loop;
    check(test_pass, test_fail,
          HWDATA, x"11223344",
          "TC5-b2: HWDATA burst beat 2");

    -- Beat 3 – SEQ
    ahbmo.htrans  <= HTRANS_SEQ;
    ahbmo.haddr   <= x"40000208";
    ahbmo.hwdata  <= x"55667788";
    loop
      wait until rising_edge(clk);
      exit when ahbmi.hready = '1';
    end loop;
    check(test_pass, test_fail,
          HWDATA, x"55667788",
          "TC5-b3: HWDATA burst beat 3");

    -- Beat 4 – SEQ (last)
    ahbmo.htrans  <= HTRANS_SEQ;
    ahbmo.haddr   <= x"4000020C";
    ahbmo.hwdata  <= x"99AABBCC";
    loop
      wait until rising_edge(clk);
      exit when ahbmi.hready = '1';
    end loop;
    check(test_pass, test_fail,
          HWDATA, x"99AABBCC",
          "TC5-b4: HWDATA burst beat 4");

    -- End burst
    ahbmo.htrans  <= HTRANS_IDLE;
    ahbmo.hbusreq <= '0';
    ahbmo.hwrite  <= '0';
    wait until rising_edge(clk);

    -- Now read back the four words
    print("--- TC5: AHB burst read-back ---");
    ahb_read(clk, ahbmo, ahbmi, addr => x"40000200");
    check(test_pass, test_fail,
          HRDATA, x"AABBCCDD",
          "TC5-r1: HRDATA burst read beat 1");

    ahb_read(clk, ahbmo, ahbmi, addr => x"40000204");
    check(test_pass, test_fail,
          HRDATA, x"11223344",
          "TC5-r2: HRDATA burst read beat 2");

    ahb_read(clk, ahbmo, ahbmi, addr => x"40000208");
    check(test_pass, test_fail,
          HRDATA, x"55667788",
          "TC5-r3: HRDATA burst read beat 3");

    ahb_read(clk, ahbmo, ahbmi, addr => x"4000020C");
    check(test_pass, test_fail,
          HRDATA, x"99AABBCC",
          "TC5-r4: HRDATA burst read beat 4");

    -- ===================================================
    --  TC6 – APB READ: MAC address registers
    -- ===================================================
    print("--- TC6: APB Read MAC MSB/LSB registers ---");
    apb_read(clk, apbi, addr => x"00000008");
    check(test_pass, test_fail,
          PRDATA, x"00001234",
          "TC6a: PRDATA = MAC_MSB (0x00001234)");

    apb_read(clk, apbi, addr => x"0000000C");
    check(test_pass, test_fail,
          PRDATA, x"56789ABC",
          "TC6b: PRDATA = MAC_LSB (0x56789ABC)");

    -- ===================================================
    --  TC7 – APB WRITE to CTRL then READ BACK
    --  Verify bridge correctly round-trips a config write.
    -- ===================================================
    print("--- TC7: APB Write CTRL ← 0x00000083 then readback ---");
    -- Set SP=1, TE=1, RE=1 (bits 7,0,1)
    apb_write(clk, apbi,
              addr => x"00000000",
              data => x"00000083");

    apb_read(clk, apbi, addr => x"00000000");
    check(test_pass, test_fail,
          PRDATA, x"00000083",
          "TC7: CTRL readback = 0x00000083 after write");

    -- ---------------------------------------------------
    --  Summary
    -- ---------------------------------------------------
    wait for CLK_PERIOD * 5;
    print("=================================================");
    print("  TEST SUMMARY");
    print("=================================================");

    -- Final summary line using textio
    report "Simulation complete. Check pass/fail counters." severity note;

    wait;  -- stop simulation
  end process STIMULUS;

end sim;