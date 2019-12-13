library ieee;
use ieee.std_logic_1164.all;
library work;
use work.Gates.all;

entity RegisterRead is
port(pc,ir,pc_inc,jump_addr_in,wr_reg_data,wr_pc: in std_logic_vector(15 downto 0);
		lsreg_a,wr_reg_add: in std_logic_vector(2 downto 0);
		control_word_input: in std_logic_vector(15 downto 0);
		prop,enable,prop_wb,enable_wb,write_wb,clk,r: in std_logic;
		control_word_output: out std_logic_vector(15 downto 0);
		lsreg_a_out: out std_logic_vector(2 downto 0);
	 output_pc,output_pc_inc,output_ir,ra,rb,jump_addr_out: out std_logic_vector(15 downto 0);
	 O0, O1, O2, O3, O4, O5, O6, O7 : out std_logic_vector(15 downto 0);
	 start_bit_a:in std_logic; start_bit_out: out std_logic;
	 in_prop_r: in std_logic_vector(15 downto 0);
	 out_prop_r: out std_logic_vector(15 downto 0)
	 );
		
	
end entity RegisterRead;
	
architecture RR of RegisterRead is

component RF is
port(	clk : in std_logic;
		rst : in std_logic;
		Write_En : in std_logic_vector(1 downto 0);
		Read_En : in std_logic;
		Write_Data_1 : in std_logic_vector(15 downto 0);
		Write_Data_2 : in std_logic_vector(15 downto 0);
		Read_Data_1 : out std_logic_vector(15 downto 0);
		Read_Data_2 : out std_logic_vector(15 downto 0);
		In_select_1 : in std_logic_vector(2 downto 0);
		In_select_2 : in std_logic_vector(2 downto 0);
		Out_select_1 : in std_logic_vector(2 downto 0);
		Out_select_2 : in std_logic_vector(2 downto 0);
		O0, O1, O2, O3, O4, O5, O6, O7 : out std_logic_vector(15 downto 0));
end component;

component Bit_adder_8 is
   port( input_vector1: in std_logic_vector(15 downto 0);
	      input_vector2 :in std_logic_vector(15 downto 0);
       	output_vector: out std_logic_vector(16 downto 0));
end component;

signal Operand_select, temp_var, temp_var_1 : std_logic_vector(1 downto 0);
signal ra_rd_En, rb_rd_En, RF_En_sig : std_logic;
signal ra_temp,rb_temp, wr_pc_1,rb_jump_temp, jump_temp: std_logic_vector(15 downto 0);
signal wr_pc_inc : std_logic_vector(16 downto 0);
signal reg_select_a,reg_select_b,reg_select_a_temp,reg_select_b_temp : std_logic_vector(2 downto 0);


begin

--First of all, let us pass along the operands which are needed for further stages
out_prop_r <= in_prop_r;
start_bit_out <= start_bit_a;
output_pc <= pc;
output_ir <= ir;
jump_temp <= rb_jump_temp when (control_word_input(13)='1') else jump_addr_in;
jump_addr_out <= jump_temp;
control_word_output <= control_word_input;
lsreg_a_out <= lsreg_a;
temp_var_1 <= ((prop_wb and enable_wb and write_wb)&(prop_wb and enable_wb));

--Adding 1 to wr_pc

B1 : Bit_adder_8
	port map(input_vector1 =>  wr_pc , input_vector2 => "0000000000000001" , output_vector => wr_pc_inc);


--Handling the case of R7
temp_var <= '0' & temp_var_1(0) when wr_reg_add="111" else temp_var_1;
wr_pc_1 <= wr_reg_data when wr_reg_add="111" else wr_pc;


--Now let us get the values in registers a and b from the register file

reg_select_a_temp <= ir(11 downto 9) when control_word_input(8)='0' else lsreg_a;
reg_select_b_temp <= ir(11 downto 9) when control_word_input(8)='1' else ir(8 downto 6);

--Flipping for SM

reg_select_a <= reg_select_a_temp when control_word_input(8)='0' else reg_select_b_temp;
reg_select_b <= reg_select_b_temp when control_word_input(8)='0' else reg_select_a_temp;

--Giving input and output to register module

RF1: RF
port map(clk => clk,	rst => r, Write_En=> temp_var, Read_En=>'1',
		Write_Data_1 => wr_reg_data, Write_Data_2 => wr_pc_1,
		Read_Data_1 => ra_temp,
		Read_Data_2 => rb_temp,
		In_select_1 => wr_reg_add,
		In_select_2 => "111",
		Out_select_1 => reg_select_a,
		Out_select_2 => reg_select_b,
		O0 => O0, O1 => O1, O2 => O2, O3 => O3, O4 => O4, O5 => O5, O6 => O6, O7 => O7);

--Checking for dependencies and assigning final values to ra and rb

--ra <= wr_reg_data when ((temp_var(1)='1') and (reg_select_a = wr_reg_add)) else wr_pc when ((temp_var(0)='1') and (reg_select_a = "111")) else ra_temp; 
--rb <= wr_reg_data when ((temp_var(1)='1') and reg_select_b = wr_reg_add) else wr_pc when ((temp_var(0)='1') and reg_select_b = "111") else rb_temp;
ra <= pc when (reg_select_a = "111") else wr_reg_data when ((temp_var(1)='1') and reg_select_a = wr_reg_add) else ra_temp;  
rb_jump_temp <= pc when (reg_select_b = "111") else wr_reg_data when ((temp_var(1)='1') and reg_select_b = wr_reg_add) else rb_temp; 
rb <= rb_jump_temp;
	
output_pc_inc <= jump_temp when (ir(15 downto 12) = "1001") else pc_inc;	
	
end RR;