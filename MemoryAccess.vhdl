library ieee;
use ieee.std_logic_1164.all;
library work;

entity MemoryAccess is
port( 
	address 	: in std_logic_vector(15 downto 0);	-- lsreg_a in docs
	pc 		: in std_logic_vector(15 downto 0);
	pc_inc		: in std_logic_vector(15 downto 0);
	data_in		: in std_logic_vector(15 downto 0);
	ir		: in std_logic_vector(15 downto 0);
	control_word	: in std_logic_vector(15 downto 0);
	prop,enable	: in std_logic;
	clk : in std_logic;
	pc_out 		: out std_logic_vector(15 downto 0);
	pc_inc_out	: out std_logic_vector(15 downto 0);
	ir_out		: out std_logic_vector(15 downto 0);
	data_out	: out std_logic_vector(15 downto 0);
	control_word_out: out std_logic_vector(15 downto 0);
	zero_bit_out : out std_logic;
	lm_sm_in : in std_logic_vector(2 downto 0);
	lm_sm_out : out std_logic_vector(2 downto 0);
	r7_bit: in std_logic;
	r7_bit_out : out std_logic
);

end entity MemoryAccess;

architecture stage_mem of MemoryAccess is

component Memory_asyncread_syncwrite is 
	port (address,Mem_datain: in std_logic_vector(15 downto 0); 
	         clk,Mem_wrbar: in std_logic;
		   Mem_dataout: out std_logic_vector(15 downto 0));
end component;

signal data_out_temp : std_logic_vector(15 downto 0); 

begin

Mem_data: Memory_asyncread_syncwrite
	port map(address => address, Mem_datain => data_in, Mem_dataout => data_out_temp, clk => clk, Mem_wrbar => control_word(10)); 
	
----------Propagating

lm_sm_out <= lm_sm_in;
pc_out <= pc;
pc_inc_out <= pc_inc;
control_word_out <= control_word;
ir_out <= ir;
r7_bit_out <= r7_bit;

----- Zero 

zero_bit_out <= '1' when ir(15 downto 12)="0100" and data_out_temp=x"0000" else '0';
data_out <= data_out_temp when ir(15 downto 12)="0110" or ir(15 downto 12)="0100" else data_in;

end stage_mem;