library ieee;
use ieee.std_logic_1164.all;
library work;
use work.Gates.all;

entity InstructionFetch is
port( input_pc : in std_logic_vector(15 downto 0); clk, enable, prop : in std_logic;
	out_pc, out_instruction, out_pc_prev: out std_logic_vector(15 downto 0) );

end entity InstructionFetch;

architecture IR of InstructionFetch is


component Bit_adder_8 is
   port( input_vector1: in std_logic_vector(15 downto 0);
	      input_vector2 :in std_logic_vector(15 downto 0);
       	output_vector: out std_logic_vector(16 downto 0));
end component;

component Memory_asyncread_syncwrite is 
	port (address,Mem_datain: in std_logic_vector(15 downto 0); clk,Mem_wrbar: in std_logic;
				Mem_dataout: out std_logic_vector(15 downto 0));
end component;

signal out_pc_temp: std_logic_vector(16 downto 0);

begin

out_pc_prev <= input_pc;

B1 : Bit_adder_8
	port map(input_vector1 =>  input_pc , input_vector2 => "0000000000000001" , output_vector => out_pc_temp);
	
Mem1: Memory_asyncread_syncwrite
	  port map(address => input_pc, Mem_datain => "0000000000000000", Mem_dataout => out_instruction, clk => clk, Mem_wrbar => '1'); 


out_pc <= out_pc_temp(15 downto 0);


end IR;
