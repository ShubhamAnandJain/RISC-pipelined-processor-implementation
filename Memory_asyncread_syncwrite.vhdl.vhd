library std;
use std.standard.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;	 
use ieee.std_logic_unsigned.all;

-- since The Memory is asynchronous read, there is no read signal, but you can use it based on your preference.
-- this memory gives 16 Bit data in one clock cycle, so edit the file to your requirement.

entity Memory_asyncread_syncwrite is 
	port (address,Mem_datain: in std_logic_vector(15 downto 0); clk,Mem_wrbar: in std_logic;
				Mem_dataout: out std_logic_vector(15 downto 0));
end entity;

architecture Form of Memory_asyncread_syncwrite is 
type regarray is array(65535 downto 0) of std_logic_vector(15 downto 0);   -- defining a new type
signal Memory: regarray:=(
    0 => "1001101010000000",   --JALR R5 R2
    1 => "0000000001010000",   --ADD R0 R1 R2 
    2 => "0000001001010010",   --ADC R1 R1 R2
    3 => "0000110111101001",   --ADZ R6 R7 R5 
    4 => "0001100011011000",   --ADI R4 R3 011000 
    5 => "0010100001010000",   --NDU R4 R1 R2 
    6 => "0010010010011010",   --NDC R2 R2 R3
    7 => "0010001011010001",   --NDZ R1 R3 R2
    8 => "1001101010000000",   --JALR R5 R2
    9 => "0110111011011110",    --LM R7 0 11011110
    10 => "0110000001010000",   --LM R0 0 01010000 
    11 => "0111001000110001",   --SM R1 0 00110001 
    12 => "1100111101010101",   --BEQ R7 R5 010101 
    13 => "1000011011010101",   --JAL R3 011010101
    14 => "1001101010000000",   --JALR R5 R2
    15 => "0000000001010000",   --ADD R0 R1 R2  
    16 => "0000001001010010",   --ADC R1 R1 R2
    17 => "0000110111101001",   --ADZ R6 R7 R5 
    18 => "0001100011011000",   --ADI R4 R3 011000 
    19 => "0010100001010000",   --NDU R4 R1 R2 
    20 => "0010010010011010",   --NDC R2 R2 R3
    21 => "0010001011010001",   --NDZ R1 R3 R2
    22 => "0011111001010000",   --LHI R7 001010000 
    23 => "0100101011000111",   --LW R5 R3 000011 
    24 => "0101100100000001",   --SW R4 R4 000001
    25 => "0110100011011001",   --LM R4 0 11011001 
    26 => "0111011000110001",   --SM R3 0 00110001 
    27 => "1100011101010101",   --BEQ R3 R5 010101 
    28 => "1000011011010101",   --JAL R3 011010101
    29 => "1001101010000000",   --JALR R5 R2
    others => x"0000");
-- you can use the above mentioned way to initialise the memory with the instructions and the data as required to test your processor
begin
Mem_dataout <= Memory(conv_integer(address));
Mem_write:
process (Mem_wrbar,Mem_datain,address,clk)
	begin
	if(Mem_wrbar = '0') then
		if(rising_edge(clk)) then
			Memory(conv_integer(address)) <= Mem_datain;
		end if;
	end if;
	end process;
end Form;
