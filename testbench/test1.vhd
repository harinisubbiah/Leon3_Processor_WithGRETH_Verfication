-- ============================================================
-- bridge_top.vhd  –  DUT wrapper (the red box)
-- Contains: mem_apb_bridge + ahb_mem_bridge
-- Interfaces: flat AHB-Master pins, flat APB-Slave pins,
--             flat SRAM pins, flat Processor pins
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;

library grlib;
library gaisler;
use grlib.stdlib.all;
use grlib.amba.all;

library techmap;
use techmap.gencomp.all;
use gaisler.net.all;

-- ============================================================
-- Shared package  (mem_in_type / mem_out_type)
-- ============================================================
package bridge_pkg is

  constant ABITS : integer := 28;

  -- Master (uProc / AHB-bridge) → Memory Slave (SRAM)
  type mem_in_type is record
    mem_wdata : std_logic_vector(31 downto 0);
    mem_addr  : std_logic_vector(ABITS-1 downto 0);
    mem_ce1   : std_logic;
    mem_wen   : std_logic;
    mem_oen   : std_logic;
  end record;

  -- Memory Slave (SRAM) → Master (uProc / AHB-bridge)
  type mem_out_type is record
    mem_rdata : std_logic_vector(31 downto 0);
  end record;

end package;

-- ============================================================
-- APB Bridge package
-- ============================================================
package apb_bridge_pkg is

  constant ABITS : integer := 28;

  -- Signals coming from Master (uProc)  →  APB bridge slave port
  type mem_slv_in_type is record
    mem_wxdata : std_logic_vector(31 downto 0);
    mem_addr   : std_logic_vector(ABITS-1 downto 0);
    mem_ce1    : std_logic;
    mem_wen    : std_logic;
    mem_oen    : std_logic;
  end record;

  -- Signals driven to Master (uProc)  ←  APB bridge slave port
  type mem_slv_out_type is record
    mem_rxdata : std_logic_vector(31 downto 0);
  end record;

end package;

-- ============================================================
-- AHB Bridge package
-- ============================================================
package ahb_bridge_pkg is

  constant ABITS : integer := 28;

  -- Signals driven to Memory Slave (SRAM)
  type mem_mst_out_type is record
    mem_wdata : std_logic_vector(31 downto 0);
    mem_addr  : std_logic_vector(ABITS-1 downto 0);
    mem_ce1   : std_logic;
    mem_wen   : std_logic;
    mem_oen   : std_logic;
  end record;

  -- Signals coming from Memory Slave (SRAM)
  type mem_mst_in_type is record
    mem_rdata : std_logic_vector(31 downto 0);
  end record;

end package;

-- ============================================================
-- bridge_top  –  THE DUT  (the red box)
-- Port list taken verbatim from PDF page 2 (eth_module entity)
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

library grlib;
library gaisler;
use grlib.stdlib.all;
use grlib.amba.all;           -- apb_slv_in_type / apb_slv_out_type
                               -- ahb_mst_in_type  / ahb_mst_out_type

library techmap;
use techmap.gencomp.all;
use gaisler.net.all;

use work.bridge_pkg.all;

entity bridge_top is
  generic (
    SRAMBANKS : integer := 4;
    TACC      : integer := 10;
    ABITS     : integer := 28;
    sram_file : string  := "sram.srec"
  );
  port (
    clk  : in  std_logic;
    rst  : in  std_logic;

    -- ── AHB-Master outputs (driven by GRETH MAC master)
    hbusreq : in  std_ulogic;
    hlock   : in  std_ulogic;
    htrans  : in  std_logic_vector(1 downto 0);
    haddr   : in  std_logic_vector(31 downto 0);
    hwrite  : in  std_ulogic;
    hsize   : in  std_logic_vector(2 downto 0);
    hburst  : in  std_logic_vector(2 downto 0);
    hprot   : in  std_logic_vector(3 downto 0);
    hwdata  : in  std_logic_vector(31 downto 0);

    -- ── AHB-Master inputs (fed back to GRETH MAC master)
    hgrant  : out std_ulogic;
    hready  : out std_ulogic;
    hresp   : out std_logic_vector(1 downto 0);
    hrdata  : out std_logic_vector(31 downto 0);

    -- ── APB-Slave outputs (driven by GRETH MAC slave)
    prdata  : in  std_logic_vector(31 downto 0);

    -- ── APB-Slave inputs (driven into GRETH MAC slave)
    psel    : out std_ulogic;
    penable : out std_ulogic;
    paddr   : out std_logic_vector(31 downto 0);
    pwrite  : out std_ulogic;
    pwdata  : out std_logic_vector(31 downto 0);

    -- ── SRAM outputs (read data coming back from SRAM)
    sram_rdata : in  std_logic_vector(31 downto 0);

    -- ── SRAM inputs (control/address/data driven to SRAM)
    sram_addr  : out std_logic_vector(ABITS-1 downto 0);
    sram_wdata : out std_logic_vector(31 downto 0);
    sram_ce1   : out std_logic;
    sram_wen   : out std_ulogic;
    sram_oen   : out std_ulogic;

    -- ── Processor outputs (uProc → bridge)
    proc_wdata : in  std_logic_vector(31 downto 0);
    proc_addr  : in  std_logic_vector(ABITS-1 downto 0);
    proc_ce1   : in  std_logic;
    proc_wen   : in  std_logic;
    proc_oen   : in  std_logic;

    -- ── Processor inputs (bridge → uProc)
    proc_rdata : out std_logic_vector(31 downto 0);

    -- ── Scan-test
    testrst  : out std_ulogic;
    testen   : out std_ulogic;
    testoen  : out std_ulogic;

    -- ── IRQ
    irq : out std_logic
  );
end entity bridge_top;

-- ============================================================
architecture rtl of bridge_top is

  -- ── Internal record signals (connect sub-components)
  signal apbi  : apb_slv_in_type;
  signal apbo  : apb_slv_out_type;
  signal ahbmi : ahb_mst_in_type;
  signal ahbmo : ahb_mst_out_type;

  -- SRAM-facing record signals
  signal srami : mem_in_type;    -- bridge → SRAM  (address/ctrl/wdata)
  signal sramo : mem_out_type;   -- SRAM  → bridge (rdata)

  -- Processor-facing record signals
  signal uproco : mem_in_type;   -- uProc → APB bridge
  signal uproci : mem_out_type;  -- APB bridge → uProc

  -- ── Component declarations ─────────────────────────────────

  -- APB Bridge
  component mem_apb_bridge is
    generic (constant ABITS : integer := 28);
    port (
      clk, rst : in  std_logic;
      -- APB port i/o
      apbo     : in  apb_slv_out_type;
      apbi     : out apb_slv_in_type;
      -- Memory port i/o  (processor side)
      memi     : in  mem_in_type;
      memo     : out mem_out_type
    );
  end component;

  -- AHB Bridge
  component ahb_mem_bridge is
    generic (
      constant SRAMDEPTH : integer := 19;
      constant ABITS     : integer := 28
    );
    port (
      clk, rst : in  std_logic;
      -- AHB port i/o
      ahbmo    : in  ahb_mst_out_type;
      ahbmi    : out ahb_mst_in_type;
      -- Memory port i/o  (SRAM side)
      memi     : in  mem_out_type;    -- mem_mst_in  (rdata from SRAM)
      memo     : out mem_in_type      -- mem_mst_out (addr/ctrl/wdata to SRAM)
    );
  end component;

begin

  -- ══════════════════════════════════════════════════════════
  -- Instantiate AHB Bridge
  -- ══════════════════════════════════════════════════════════
  ahb_bridge0 : ahb_mem_bridge
    generic map (ABITS => ABITS)
    port map (
      clk   => clk,
      rst   => rst,
      ahbmo => ahbmo,
      ahbmi => ahbmi,
      memi  => sramo,   -- SRAM read-data → bridge
      memo  => srami    -- bridge → SRAM addr/ctrl/wdata
    );

  -- ══════════════════════════════════════════════════════════
  -- Instantiate APB Bridge
  -- ══════════════════════════════════════════════════════════
  apb_bridge0 : mem_apb_bridge
    generic map (ABITS => ABITS)
    port map (
      clk   => clk,
      rst   => rst,
      apbo  => apbo,
      apbi  => apbi,
      memi  => uproco,  -- uProc → APB bridge
      memo  => uproci   -- APB bridge → uProc
    );

  -- ══════════════════════════════════════════════════════════
  -- SRAM ↔ Bridge signal wiring
  -- ══════════════════════════════════════════════════════════
  sramo.mem_rdata <= sram_rdata;   -- SRAM → AHB bridge

  sram_addr  <= srami.mem_addr;    -- AHB bridge → SRAM
  sram_wdata <= srami.mem_wdata;
  sram_ce1   <= srami.mem_ce1;
  sram_wen   <= srami.mem_wen;
  sram_oen   <= srami.mem_oen;

  -- ══════════════════════════════════════════════════════════
  -- AHB-Master ↔ Bridge  (GRETH MAC AHB master side)
  -- ══════════════════════════════════════════════════════════
  ahbmo.hbusreq <= hbusreq;
  ahbmo.hlock   <= hlock;
  ahbmo.htrans  <= htrans;
  ahbmo.haddr   <= haddr;
  ahbmo.hwrite  <= hwrite;
  ahbmo.hsize   <= hsize;
  ahbmo.hburst  <= hburst;
  ahbmo.hprot   <= hprot;
  ahbmo.hwdata  <= hwdata;

  hgrant <= ahbmi.hgrant(ahbmo.hindex);
  hready <= ahbmi.hready;
  hresp  <= ahbmi.hresp;
  hrdata <= ahbmi.hrdata;

  -- ══════════════════════════════════════════════════════════
  -- APB-Slave ↔ Bridge  (GRETH MAC APB slave side)
  -- ══════════════════════════════════════════════════════════
  apbo.prdata <= prdata;           -- GRETH MAC → bridge

  psel    <= apbi.psel(apbo.pindex);
  penable <= apbi.penable;
  paddr   <= apbi.paddr;
  pwrite  <= apbi.pwrite;
  pwdata  <= apbi.pwdata;

  -- ══════════════════════════════════════════════════════════
  -- Processor ↔ Bridge  (leon3 / uProc side)
  -- ══════════════════════════════════════════════════════════
  uproco.mem_wdata <= proc_wdata;
  uproco.mem_addr  <= proc_addr;
  uproco.mem_ce1   <= proc_ce1;
  uproco.mem_wen   <= proc_wen;
  uproco.mem_oen   <= proc_oen;

  proc_rdata <= uproci.mem_rdata;

  -- ══════════════════════════════════════════════════════════
  -- Scan-test  (driven from AHB bridge internals via ahbmi)
  -- ══════════════════════════════════════════════════════════
  testen  <= ahbmi.testen;
  testrst <= ahbmi.testrst;
  testoen <= ahbmi.testoen;

  -- ══════════════════════════════════════════════════════════
  -- IRQ  (from AHB bridge)
  -- ══════════════════════════════════════════════════════════
  irq <= ahbmi.hirq(ahbmo.hindex);

end architecture rtl;
