library ieee;
use ieee.std_logic_1164.all;
--priority encoder
entity eightToThreeEnc is
port(input_line : in std_logic_vector(7 downto 0);
	  output_line : out std_logic_vector(2 downto 0)
		);
		
end entity;

architecture str of eightToThreeEnc is

begin
output_line(0) <= ((not input_line(6)) and (((not input_line(4)) and (not input_line(2)) and input_line(1)) or ((not input_line(4)) and input_line(3)) or input_line(5))) or input_line(7);
output_line(1) <= input_line(7) or input_line(6) or ((input_line(3) or input_line(2)) and (not input_line(4)) and (not input_line(5)));
output_line(2) <= input_line(4) or input_line(5) or input_line(6) or input_line(7);
end str;