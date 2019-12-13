library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity InstructionDecode is

port(
		pc : in std_logic_vector(15 downto 0);
		pc_inc : in std_logic_vector(15 downto 0);
		ir : in std_logic_vector(15 downto 0);
		prop_bit : in std_logic;
		Enable_bit : in std_logic;
		SM_LM_mux_control : in std_logic;
		clk : in std_logic;
		
		out_pc : out std_logic_vector(15 downto 0);
		out_pc_inc : out std_logic_vector(15 downto 0);
		out_ir : out std_logic_vector(15 downto 0);
		control_word : out std_logic_vector(15 downto 0);
		BEQ_jump_address : out std_logic_vector(15 downto 0);
		Normal_jump_address : out std_logic_vector(15 downto 0);
		LsReg_a : out std_logic_vector(2 downto 0);
		IR_ID_enable : out std_logic;
		flag_bit : out std_logic;
		start_bit : out std_logic;
		prop_reg : out std_logic_vector(15 downto 0)
		);

end entity;

architecture decode of InstructionDecode is


component Bit_adder_1 is
   port( input_vector1: in std_logic_vector(15 downto 0);
	      input_vector2 :in std_logic_vector(15 downto 0);
			CarIn : in std_logic;
       	output_vector: out std_logic_vector(15 downto 0));
end component;

component eightToThreeEnc is
port(input_line : in std_logic_vector(7 downto 0);
	  output_line : out std_logic_vector(2 downto 0)
		);		
end component;

component threeToeight_Decoder is
port(input_line1 : in std_logic_vector(2 downto 0);
     output_line1 : out std_logic_vector(7 downto 0)
		);
		
end component;

signal Imm6, Imm9, Jmp_mag, jmp_address_signal : std_logic_vector(15 downto 0);
signal Pref6 : std_logic_vector(9 downto 0);
signal Pref9 : std_logic_vector(6 downto 0);
signal RegSelectImm, Decoder_output, feedback_to_enc, fbenc1 : std_logic_vector(7 downto 0);
signal ra_temp_add, ra_temp_add_1 : std_logic_vector(2 downto 0);
signal mux_select, bool_bit : std_logic;

begin

start_bit <= '1' when((ir(15 downto 12) = "0110" or ir(15 downto 12) = "0111") and SM_LM_mux_control = '0') else '0';

IR_ID_enable <= (not ra_temp_add_1(0)) and (not ra_temp_add_1(1)) and (not ra_temp_add_1(2));
ra_temp_add(2) <= not ra_temp_add_1(2);
ra_temp_add(1) <= not ra_temp_add_1(1);
ra_temp_add(0) <= not ra_temp_add_1(0);
mux_select <= ir(15) and ir(14);
LsReg_a <= ra_temp_add;
out_pc <= pc;
out_ir <= ir;
feedback_to_enc <= RegSelectImm and not(Decoder_output);

flag_bit <='1' when ((ir(15 downto 12) = "0111" or ir(15 downto 12) = "0110") and ir(0) = '1') else '0';
bool_bit <= '0' when ir(7 downto 0) = "00000000" else '1';

BEQ_jump_address <= (others => '0') when mux_select = '0' else
							jmp_address_signal when mux_select = '1';
							
Normal_jump_address <= (others => '0') when mux_select = '1' else
								jmp_address_signal when mux_select = '0' ;


Pref6 <= (others => ir(5));
Pref9 <= (others => ir(8));

Imm6 <=  Pref6 & ir(5 downto 0);
Imm9 <=  Pref9 & ir(8 downto 0);
RegSelectImm <= ir(7 downto 0) when SM_LM_mux_control = '0' else
					 fbenc1 when SM_LM_mux_control = '1';

process(clk)
begin
if(clk'event and clk = '1') then
fbenc1 <= feedback_to_enc;
			
end if;
end process;			 
Jmp_mag <= Imm6 when mux_select = '1' else
			  Imm9 when mux_select = '0' ;

Enc : eightToThreeEnc
port map(input_line => RegSelectImm , output_line => ra_temp_add_1);

Dec : threeToeight_Decoder
port map(input_line1 => ra_temp_add_1 , output_line1 => Decoder_output);

Adder : Bit_adder_1
port map(input_vector1=> Jmp_mag, input_vector2=> pc_inc, CarIn=> '0', output_vector=> jmp_address_signal);

out_pc_inc <= jmp_address_signal when (ir(15 downto 12) = "1000") else pc_inc;
prop_reg <= pc_inc;
--generation of control words
control_word(15) <= (not(ir(15)) and not(ir(14)) and (ir(13)) and not(ir(12)));
control_word(14) <= ir(15) and not(ir(14)) and not(ir(13)) and not(ir(12)) and (not(ir(11) and ir(10) and ir(9)));
control_word(13) <= ir(15) and not(ir(14)) and not(ir(13)) and ir(12) and (not(ir(11) and ir(10) and ir(9)));
control_word(12) <= not(ir(15)) and not(ir(14)) and not(ir(13));
control_word(11) <= (not(ir(15)) and not(ir(14)) and (ir(13) nand ir(12))) or (ir(14) and not(ir(15)) and (ir(13) nor ir(12)));
control_word(10) <= not(not(ir(15)) and (ir(14)) and (ir(12)));
control_word(9) <= not(ir(15)) and ir(14) and ir(13) and not(ir(12)) and bool_bit;
control_word(8) <= not(ir(15)) and ir(14) and ir(13) and ir(12) and bool_bit;
control_word(7) <= not(ir(14)) or (not(ir(12)) and not(ir(15)));
control_word(6) <= mux_select;
control_word(5) <= ir(15) and ir(14) and not(ir(13)) and not(ir(12));
control_word(4) <= not(ir(15)) and not(ir(14)) and not(ir(12)) and ir(1);
control_word(3) <= not(ir(15)) and not(ir(14)) and not(ir(12)) and ir(0) ;
control_word(2) <= not(ir(15)) and not(ir(14)) and ir(13) and ir(12);
control_word(1) <= not(ir(15)) and not(ir(14)) and not(ir(13)) and ir(12);
control_word(0) <= (ir(14) and ir(13)) or (not(ir(15)) and not(ir(14)) and not(ir(13)) and ir(12));




end decode;