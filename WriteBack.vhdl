library ieee;
use ieee.std_logic_1164.all;
library work;
use work.Gates.all;

entity WriteBack is
port( rf_write_data_1,rf_write_data_2,in_control_word,in_instruction : in std_logic_vector(15 downto 0);
		data_add_1,data_add_2 : in std_logic_vector(2 downto 0);
		prop,enab : in std_logic;
		rf_write_data_1_out,rf_write_data_2_out : out std_logic_vector(15 downto 0);
		data_add_1_out,data_add_2_out : out std_logic_vector(2 downto 0);
		prop_out,enab_out,write_out : out std_logic);

end entity WriteBack;

architecture WB of WriteBack is

signal opcode: std_logic_vector(3 downto 0);

begin

opcode <= in_instruction(15 downto 12);
rf_write_data_1_out<= rf_write_data_1;
rf_write_data_2_out<= rf_write_data_2;
data_add_1_out <= data_add_1 when opcode = "0110" else in_instruction(11 downto 9) when ((((not opcode(3)) and (not opcode(2)) and opcode(1) and opcode(0)) or (opcode(3) and (not opcode(2)) and (not opcode(1))) or ((not opcode(3)) and opcode(2) and (not opcode(0))))='1') else in_instruction(5 downto 3);
data_add_2_out <= data_add_2;
prop_out <= prop;
enab_out <= enab;
write_out <= in_control_word(7);

end WB;
