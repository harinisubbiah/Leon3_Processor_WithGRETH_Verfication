sram_model : process(clk)
    variable registered_addr : std_logic_vector(ABITS-1 downto 0);
  begin
    if rising_edge(clk) then

      -- Stage 1: Register address every cycle
      -- (address is valid 1 cycle before control signals)
      registered_addr := sram_addr;

      -- Stage 2: Use registered address with current control signals
      if sram_ce1 = '0' then
        if sram_oen = '1' then
          -- WRITE: use registered address
          sram_mem(to_integer(
            unsigned(registered_addr(11 downto 2)))) <= sram_wdata;
          sram_rdata <= (others => '0');

        elsif sram_oen = '0' then
          -- READ: use registered address
          if unsigned(registered_addr) >= 16#F00000# then
            sram_rdata <= x"DEAD0000";
          else
            sram_rdata <= sram_mem(to_integer(
              unsigned(registered_addr(11 downto 2))));
          end if;
        end if;
      else
        sram_rdata <= (others => '0');
      end if;

    end if;
  end process;
