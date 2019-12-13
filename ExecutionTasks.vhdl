library ieee;
use ieee.std_logic_1164.all;
library work;

entity ExecutionTasks is
port( 
	
	lm_address 	: in std_logic_vector(15 downto 0);	-- lsreg_a in docs
	pc 		: in std_logic_vector(15 downto 0);
	pc_inc		: in std_logic_vector(15 downto 0);
	operand_1	: in std_logic_vector(15 downto 0);	--ra
	operand_2	: in std_logic_vector(15 downto 0);	--rb
	ir		: in std_logic_vector(15 downto 0);
	
	
	control_word	: in std_logic_vector(15 downto 0);
	jump_addr	: in std_logic_vector(15 downto 0);

	
	prop,enable	: in std_logic;
	zin,cin		: in std_logic;
	
--	lhi_sel		: in std_logic;
	
	lm_address_out	: out std_logic_vector(15 downto 0);
	pc_out 		: out std_logic_vector(15 downto 0);
	pc_inc_out	: out std_logic_vector(15 downto 0);
	alu_out		: out std_logic_vector(15 downto 0);
	ir_out		: out std_logic_vector(15 downto 0);
	jump_addr_out	: out std_logic_vector(15 downto 0);
	cout 		: out std_logic;
	zout		: out std_logic;
	wr_c_bit		: out std_logic;
	beq_condn_bit	: out std_logic;
	
	control_word_out: out std_logic_vector(15 downto 0);
	lm_sm_in : in std_logic_vector(2 downto 0);
	lm_sm_out : out std_logic_vector(2 downto 0);
    
    start_bit   : in std_logic;
    r7_bit      : out std_logic;
	 in_prop_e : in std_logic_vector(15 downto 0);
	 out_prop_e : out std_logic_vector(15 downto 0)
    );

end entity ExecutionTasks;

architecture OT of ExecutionTasks is

component ALU is
   port(input_vectorf1 : in std_logic_vector(15 downto 0);
	input_vectorf2 : in std_logic_vector(15 downto 0);
	Op_select : in std_logic; --0-Add 1-Nand
	output_vectorf : out std_logic_vector(15 downto 0);
	Carry_flag : out std_logic;
	Zero_flag : out std_logic);

end component;

signal alu_c, r7_bit_temp : std_logic;
signal alu_z, c_temp, z_temp,control_word_out_temp7 : std_logic;
signal alu_output : std_logic_vector(15 downto 0);
signal alu_out_temp : std_logic_vector(15 downto 0);
signal imm : std_logic_vector(15 downto 0);
signal lhi : std_logic_vector(15 downto 0);
signal operand_sel_1, opt, beq_condn_temp : std_logic;
signal operand_sel_2 : std_logic;
signal op1 : std_logic_vector(15 downto 0);
signal op2, jump_temp : std_logic_vector(15 downto 0);

begin
	
----------Propagating

pc_out <= pc;
out_prop_e <= in_prop_e;
control_word_out(15 downto 8) <= control_word(15 downto 8);
control_word_out(6 downto 0) <= control_word(6 downto 0);
lm_sm_out <= lm_sm_in;
ir_out <= ir;

------ JUMP ADDR ( For BEQ, R7)

jump_temp		 <= jump_addr when ir(15 downto 12)="1100" else
                alu_out_temp when ir(15 downto 14)="00" and r7_bit_temp='1' ;
jump_addr_out <= jump_temp;

r7_bit_temp <= '1' when (ir(15 downto 12)="0000" and ir(5 downto 3)="111" and control_word_out_temp7='1') or (ir(15 downto 12)="0001" and ir(8 downto 6)="111") or (ir(15 downto 12)="0010" and ir(5 downto 3)="111" and control_word_out_temp7='1') or (ir(15 downto 12)="0011" and ir(11 downto 9)="111")
                else '0';
-- BEQ condn evaluate
beq_condn_temp <= '1' when ((operand_1 = operand_2) and control_word(5)='1' and prop='1' and enable='1') or (r7_bit_temp='1')  else '0';
beq_condn_bit <= beq_condn_temp;
r7_bit <= '1' when (lm_sm_in="111" and ir(15 downto 12) = "0110") or (ir(15 downto 12)="0100" and ir(11 downto 9)="111") else '0';
--Setting the values of the operands and op_sel

operand_sel_1 <= '1' when ir(15 downto 13)="010" else '0' ;
operand_sel_2 <= '1' when ir(15 downto 13)="011" else '0' ;
opt <= '1' when (ir(15 downto 12)="0001") else '0';			

imm <= "1111111111"&ir(5 downto 0) when ir(5) = '1' else "0000000000"&ir(5 downto 0);

op2 <= operand_2 when operand_sel_2='0' and opt='0' else 
        imm when opt='1' else   x"0001";
	
op1 <= operand_1 when operand_sel_1='0' else imm;


--Doing ALU,LHI operation and output

lhi <= ir(8 downto 0)&"0000000";	
		 
Alu1: ALU
	  port map(input_vectorf1 => op1, input_vectorf2 => op2, Op_select => control_word(15), output_vectorf => alu_output, Carry_flag => alu_c, Zero_flag => alu_z);

alu_out_temp <= lhi when control_word(2)='1' else 
            pc_inc when ir(15 downto 13)="100" else
	    operand_2 when ir(15 downto 12)="0111" else 
            lm_address  when ir(15 downto 14)="01" else 
            alu_output;
	
alu_out <= alu_out_temp;

lm_address_out <= alu_output when ir(15 downto 13)="010" or (ir(15 downto 13)="011" and start_bit='0') else
                    op1 when ir(15 downto 13)="011" and start_bit='1' else
		    --operand_2 when ir(15 downto 13)="011" and start_bit='1' else
                    lm_address;	

--operand1-address


--control_word_out(7) <= '1' when (((control_word(4)='0' or c_temp='1') and (control_word(3)='0' or z_temp='1')) and control_word(7) = '1') else '0';
control_word_out_temp7 <= '0' when ((control_word(4)='1' and cin='0') or (control_word(3)='1' and zin='0')) else control_word(7);



control_word_out(7) <= control_word_out_temp7;

--This is also trial code, please change if wrong
c_temp <= alu_c when control_word(12)='1' and prop='1' and enable='1' and control_word_out_temp7='1' else cin;
z_temp <= alu_z when control_word(11)='1' and prop='1' and enable='1' and control_word_out_temp7='1' else zin;

cout <= c_temp;
zout <= z_temp;

pc_inc_out <= pc_inc when (beq_condn_temp = '0') else jump_temp;

end OT;
