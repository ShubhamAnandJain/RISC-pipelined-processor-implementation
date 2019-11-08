library ieee;
use ieee.std_logic_1164.all;
--decoder block
entity threeToeight_Decoder is
port(input_line1 : in std_logic_vector(2 downto 0);
    output_line1: out std_logic_vector(7 downto 0)
		);
		
end entity;

architecture decode of threeToeight_Decoder is

begin

output_line1 <= "00000001" when input_line1 = "000" else
	       "00000010" when input_line1 = "001" else
	       "00000100" when input_line1 = "010" else
	       "00001000" when input_line1 = "011" else
	       "00010000" when input_line1 = "100" else
	       "00100000" when input_line1 = "101" else
	       "01000000" when input_line1 = "110" else
	       "10000000" when input_line1= "111" ;

end decode;
