library ieee;
use ieee.std_logic_1164.all;

entity TopLevel is
   port (r, clk: in std_logic);
end entity;

architecture Project1 of TopLevel is
  signal enable_if, enable_id, enable_rr, enable_ex, enable_mem, enable_wb : std_logic;
  signal pc_mux_control, lm_instruction : std_logic;
  signal rr_mux_control : std_logic_vector(15 downto 0);
  signal out_addrpc : std_logic_vector(15 downto 0);
  
component InstructionFetch is
port( input_pc : in std_logic_vector(15 downto 0); clk, enable, prop : in std_logic;
	out_pc, out_instruction, out_pc_prev: out std_logic_vector(15 downto 0) );
end component;

component InstructionDecode is
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
end component;  

-- Note that to_jump in InstructionDecode will be used by the TopLevel logic

component RegisterRead is
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
		
		
end component;

component ExecutionTasks is

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

end component;	

component MemoryAccess is

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
	r7_bit_out : out std_logic;
	in_prop_m : in std_logic_vector(15 downto 0);
	out_prop_m : out std_logic_vector(15 downto 0)
	 );

end component;	

component WriteBack is

port( rf_write_data_1,rf_write_data_2,in_control_word,in_instruction : in std_logic_vector(15 downto 0);
		data_add_1,data_add_2 : in std_logic_vector(2 downto 0);
		prop,enab : in std_logic;
		rf_write_data_1_out,rf_write_data_2_out : out std_logic_vector(15 downto 0);
		data_add_1_out,data_add_2_out : out std_logic_vector(2 downto 0);
		prop_out,enab_out,write_out : out std_logic;
		in_prop_w : in std_logic_vector(15 downto 0)
		);

end component;

component HazardDetectionUnit is
port( 
    ir_1    :   in std_logic_vector(15 downto 0);       -- IR_1 is the first instruction
    ir_2    :   in std_logic_vector(15 downto 0);       -- IR_2 is the second, possibly dependent instruction
    ra_bit  :   out std_logic;                          -- RegA of instruction 2 dependent
	 rb_bit  :   out std_logic;                           -- RegB of instruction 2 dependent
	 fake_rb : in std_logic_vector(2 downto 0)
	);
end component;

		
-- INTERFACE REGISTERS
signal check_PC : std_logic_vector(15 downto 0);
signal check_IR : std_logic_vector(15 downto 0);
signal check_cntrl_wrd : std_logic_vector(15 downto 0);
signal check_op_1 : std_logic_vector(15 downto 0);
signal check_op_2 : std_logic_vector(15 downto 0);
signal check_address : std_logic_vector(15 downto 0);
signal R0, R1, R2, R3, R4, R5, R6, R7 : std_logic_vector(15 downto 0);


signal IF_ID_enable : std_logic;
signal ID_RR_enable : std_logic;
signal RR_EX_enable : std_logic;
signal EX_MEM_enable : std_logic;
signal MEM_WR_enable : std_logic;

signal IF_ID_interface : std_logic_vector(50 downto 0);
signal ID_RR_interface : std_logic_vector(124 downto 0);
signal RR_EX_interface : std_logic_vector(131 downto 0);
signal EX_MEM_interface : std_logic_vector(98 downto 0);
signal MEM_WR_interface : std_logic_vector(71 downto 0);

-- INSTRUCTION FETCH : f

signal in_pc_f : std_logic_vector(15 downto 0);
signal out_pc_f : std_logic_vector(15 downto 0);
signal out_pc_inc_f : std_logic_vector(15 downto 0);
signal out_instruction_f : std_logic_vector(15 downto 0);
signal in_enable_f, in_prop_f : std_logic;
 
-- INSTRUCTION DECODE : d

signal out_pc_d, out_pc_inc_d, out_instruction_d, out_control_word_d, out_jump_add_d, out_normal_jump_d : std_logic_vector(15 downto 0);
signal out_ls_enable_d, out_zero_flag_d, out_start_bit_d : std_logic;
signal out_lsreg_a_d : std_logic_vector(2 downto 0);
-- REGISTER READ : r

signal out_pc_r, out_pc_inc_r, out_instruction_r, out_control_word_r, out_jump_add_r, out_ra_r, out_rb_r : std_logic_vector(15 downto 0);
signal out_lsreg_a_r : std_logic_vector(2 downto 0);
signal in_start_bit_r, out_start_bit_r : std_logic;

-- EXECUTION TASKS : e

signal out_pc_e, out_pc_inc_e, out_instruction_e, out_control_word_e, out_jump_add_e, out_lsreg_a_e : std_logic_vector(15 downto 0);
signal alu_out_e, lm_address_out_e : std_logic_vector(15 downto 0);
signal out_c_e, out_z_e, beq_condn_bit_e, wr_c_bit_e : std_logic;
signal in_lm_sm_address_e, out_lm_sm_address_e : std_logic_vector(2 downto 0);
signal in_start_bit_e, out_r7_bit_e : std_logic;

-- MEMORY ACCESS : m

signal out_pc_m, out_pc_inc_m, out_instruction_m, out_data_m, out_control_word_m : std_logic_vector(15 downto 0);
signal zero_flag_out_m : std_logic;
signal in_lm_sm_address_m, out_lm_sm_address_m : std_logic_vector(2 downto 0);
signal in_r7_bit_m, out_r7_bit_m : std_logic;

-- WRITE BACK : w

signal rf_write_data1_out_w, rf_write_data2_out_w : std_logic_vector(15 downto 0);
signal data_add_1_out_w, data_add_2_out_w : std_logic_vector(2 downto 0);
signal enable_w, prop_w, write_w : std_logic;

-- HAZARDS

signal bwb_rr_ra, bwb_rr_rb, bmem_bex_ra, bmem_bex_rb,
 bwb_bex_ra, bwb_bex_rb, awb_rr_ra, awb_rr_rb : std_logic;

-- General

signal q_var : std_logic_vector(1 downto 0);

  
-- Give inputs to IF, ID, RR, etc  

-- Curr pc+1 for JAL/JALR

signal out_prop_d, in_prop_r, out_prop_r, in_prop_e, out_prop_e, in_prop_m, out_prop_m, in_prop_w : std_logic_vector(15 downto 0);
  
  
begin

check_PC <= ID_RR_interface(124 downto 109);
check_IR <= out_instruction_d;
check_cntrl_wrd <= ID_RR_interface(22 downto 7);
check_op_1 <= rf_write_data1_out_w;
check_op_2 <= rf_write_data2_out_w;

-- INSTRUCTION FETCH
	
	IF_instance_1 : InstructionFetch port map(in_pc_f, clk, in_enable_f, in_prop_f, out_pc_inc_f, out_instruction_f, out_pc_f);
	
-- INSTRUCTION DECODE

	ID_instance_1 : InstructionDecode port map(IF_ID_interface(50 downto 35), IF_ID_interface(34 downto 19), 
	IF_ID_interface(18 downto 3), IF_ID_interface(2), IF_ID_interface(1), IF_ID_interface(0), clk,
	out_pc_d, out_pc_inc_d, out_instruction_d, out_control_word_d, out_jump_add_d, out_normal_jump_d,
	out_lsreg_a_d, out_ls_enable_d, out_zero_flag_d, out_start_bit_d, out_prop_d);
	
-- REGISTER READ

	RR_instance_1 : RegisterRead port map(ID_RR_interface(124 downto 109), ID_RR_interface(108 downto 93),
	ID_RR_interface(92 downto 77), ID_RR_interface(76 downto 61), ID_RR_interface(60 downto 45), 
	ID_RR_interface(44 downto 29), ID_RR_interface(28 downto 26), ID_RR_interface(25 downto 23), 
	ID_RR_interface(22 downto 7), ID_RR_interface(6), ID_RR_interface(5), ID_RR_interface(4), ID_RR_interface(3),
	ID_RR_interface(2), clk, ID_RR_interface(0), out_control_word_r, out_lsreg_a_r, out_pc_r, out_pc_inc_r,
	out_instruction_r, out_ra_r, out_rb_r, out_jump_add_r, R0, R1, R2, R3, R4, R5, R6, R7, in_start_bit_r, out_start_bit_r, in_prop_r, out_prop_r);
	
-- EXECUTION TASKS

	EX_instance_1 : ExecutionTasks port map(RR_EX_interface(131 downto 116), RR_EX_interface(115 downto 100), 
	RR_EX_interface(99 downto 84), RR_EX_interface(83 downto 68), RR_EX_interface(67 downto 52), RR_EX_interface(51 downto 36),
	RR_EX_interface(35 downto 20), RR_EX_interface(19 downto 4), RR_EX_interface(3), RR_EX_interface(2), RR_EX_interface(1),
	RR_EX_interface(0), lm_address_out_e, out_pc_e, out_pc_inc_e, alu_out_e, out_instruction_e, out_jump_add_e, out_c_e,
	out_z_e, wr_c_bit_e, beq_condn_bit_e, out_control_word_e, in_lm_sm_address_e, out_lm_sm_address_e, in_start_bit_e, out_r7_bit_e, in_prop_e, out_prop_e);

-- MEMORY ACCESS

	MEM_instance_1 : MemoryAccess port map(EX_MEM_interface(98 downto 83), EX_MEM_interface(82 downto 67), 
	EX_MEM_interface(66 downto 51), EX_MEM_interface(50 downto 35), EX_MEM_interface(34 downto 19), EX_MEM_interface(18 downto 3),
	EX_MEM_interface(2), EX_MEM_interface(1), clk, out_pc_m, out_pc_inc_m, out_instruction_m,
	out_data_m, out_control_word_m, zero_flag_out_m, in_lm_sm_address_m, out_lm_sm_address_m, in_r7_bit_m, out_r7_bit_m, in_prop_m, out_prop_m);

-- WRITE BACK	
	
	WB_instance_1 : WriteBack port map(MEM_WR_interface(39 downto 24), MEM_WR_interface(23 downto 8), MEM_WR_interface(55 downto 40), MEM_WR_interface(71 downto 56), MEM_WR_interface(7 downto 5),
	MEM_WR_interface(4 downto 2), MEM_WR_interface(1), MEM_WR_interface(0),
	rf_write_data1_out_w, rf_write_data2_out_w, data_add_1_out_w, data_add_2_out_w, prop_w, enable_w, write_w, in_prop_w);
	
-- TRANSFER FROM RR stage to WR stage	
	
	ID_RR_interface(60 downto 45) <= rf_write_data1_out_w;
	ID_RR_interface(44 downto 29) <= rf_write_data2_out_w; 
	ID_RR_interface(25 downto 23) <= data_add_1_out_w;
	ID_RR_interface(4) <= prop_w;
	ID_RR_interface(3) <= enable_w;
	ID_RR_interface(2) <= write_w;
	
-- HAZARD DETECTION
	
	HD_instance_1 : HazardDetectionUnit
	port map( 
    out_instruction_e, out_instruction_r, bmem_bex_ra, bmem_bex_rb, out_lsreg_a_r);

	HD_instance_2 : HazardDetectionUnit
	port map(
	 out_instruction_m, out_instruction_r, bwb_bex_ra, bwb_bex_rb, out_lsreg_a_r);	
		
	process(clk)
	
	variable nq_var : std_logic_vector(1 downto 0);
	variable en_if, en_id, en_rr, en_ex, en_mem, en_wr : std_logic;
	variable prop_if, prop_id, prop_rr, prop_ex, prop_mem, prop_wr : std_logic;
	variable out_pc : std_logic_vector(15 downto 0);
	variable id_ir_enable : std_logic;
	variable IF_ID : std_logic_vector(50 downto 0);
	variable ID_RR : std_logic_vector(124 downto 0);
	variable RR_EX : std_logic_vector(131 downto 0);
	variable EX_MEM : std_logic_vector(98 downto 0);
	variable MEM_WR : std_logic_vector(71 downto 0);
	variable ztake, ctake, sm_lm_mux_con : std_logic;
	variable load_dependency : std_logic_vector(3 downto 0);
	variable beq : std_logic;
	variable bmem_bex_hazard_ra, bmem_bex_hazard_rb, bwb_bex_hazard_ra, bwb_bex_hazard_rb : std_logic;
	variable bmem_add, bwb_add : std_logic_vector(15 downto 0);
	variable opcode_id, opcode_rr, opcode_f : std_logic_vector(3 downto 0);
	variable enable_bit_lm : std_logic;	
	variable next_pc : std_logic_vector(15 downto 0);
	variable beq_happens, jlr_happens, jal_happens : std_logic;
	variable beq_add, jlr_add, jal_add : std_logic_vector(15 downto 0);
	variable z_load, z_ok : std_logic;
	variable first_lm_sm : std_logic;
	variable lmsme, lmsmm, lmsmw : std_logic_vector(2 downto 0);
	variable flag_for_zero : std_logic;
	variable start_bit_r, start_bit_e : std_logic;
	variable r7_bit_e, r7_bit_m : std_logic;
	variable jump_mem : std_logic_vector(15 downto 0);
	variable out_prop_temp_d, out_prop_temp_r, out_prop_temp_e, out_prop_temp_m : std_logic_vector(15 downto 0);
	
	begin

	lmsme := out_lsreg_a_r;
	lmsmm := out_lm_sm_address_e;
	lmsmw := out_lm_sm_address_m;
	jump_mem := out_data_m;
	
	out_prop_temp_d := out_prop_d;
	out_prop_temp_r := out_prop_r;
	out_prop_temp_e := out_prop_e;
	out_prop_temp_m := out_prop_m;
	
	flag_for_zero := out_zero_flag_d;
	start_bit_r := out_start_bit_d;
	start_bit_e := out_start_bit_r;
	r7_bit_e := (out_r7_bit_e and prop_ex);
	r7_bit_m := (out_r7_bit_m and prop_mem);
	
	bmem_add := alu_out_e;
	bwb_add := out_data_m;
	-- sm_lm_mux_con := '0'; --CHANGE THIS. SHOULD CHANGE IN STATE 10 TO 1, AND 0 IN STATE 11
	
	ztake := out_z_e;
	ctake := out_c_e;
	z_ok := zero_flag_out_m;
	
	
	en_if := in_enable_f;
	prop_if := in_prop_f;
	
	en_id := IF_ID_interface(1);
	prop_id := IF_ID_interface(2);
	
	en_rr := ID_RR_interface(5);
	prop_rr := ID_RR_interface(6);
	
	en_ex := RR_EX_interface(2);
	prop_ex := RR_EX_interface(3);
	
	en_mem := EX_MEM_interface(1);
	prop_mem := EX_MEM_interface(2);
	
	en_wr := MEM_WR_interface(0);
	prop_wr := MEM_WR_interface(1);
	
	if(out_instruction_m(15 downto 12) = "0100" and prop_mem = '1') then
		z_load := '1';
	else z_load := '0';
	end if;	
	
	
	next_pc := out_pc_inc_f;
	
	if(r7_bit_m = '1' and prop_mem = '1') then
		next_pc := bwb_add;
		prop_if := '0';
		prop_id := '0';
		prop_rr := '0';
		prop_ex := '0';
		EX_MEM(2) := '0';
		RR_EX(3) := '0';
		ID_RR(6) := '0';
		IF_ID(2) := '0';
	
	elsif(r7_bit_e = '1' and prop_ex = '1') then
		prop_if := '0';
		prop_id := '0';
		prop_rr := '0';
		RR_EX(3) := '0';
		ID_RR(6) := '0';
		IF_ID(2) := '0';
	
	end if;
	
	
	bmem_bex_hazard_ra := (bmem_bex_ra) and prop_ex and out_control_word_e(7);
	bmem_bex_hazard_rb := (bmem_bex_rb) and prop_ex and out_control_word_e(7);
	bwb_bex_hazard_ra := (bwb_bex_ra) and prop_mem and out_control_word_m(7);
	bwb_bex_hazard_rb := (bwb_bex_rb) and prop_mem and out_control_word_m(7);
	
	beq_happens := (beq_condn_bit_e and prop_ex);
	jlr_happens := (out_control_word_r(13) and prop_rr);
	jal_happens := (out_control_word_d(14) and prop_id);
	
	beq_add := out_jump_add_e;
	jal_add := out_normal_jump_d;
	jlr_add := out_jump_add_r;
	
	IF_ID(50 downto 35) := out_pc_f;
	IF_ID(34 downto 19) := out_pc_inc_f; 
	IF_ID(18 downto 3) := out_instruction_f;
	IF_ID(2) := prop_if;
	IF_ID(1) := en_if;
	--IF_ID(0) := sm_lm_mux_con; ASSIGNED LATER NOW
	
	
	ID_RR(124 downto 109) := out_pc_d;
	ID_RR(108 downto 93) := out_instruction_d;
	ID_RR(92 downto 77) := out_pc_inc_d;
	ID_RR(76 downto 61) := out_jump_add_d;
	ID_RR(28 downto 26) := out_lsreg_a_d;
	ID_RR(22 downto 7) := out_control_word_d;
	ID_RR(6) := prop_id;
	ID_RR(5) := en_id;
	ID_RR(1) := clk;
	ID_RR(0) := r;
	
	
	RR_EX(131 downto 116) := out_ra_r;
	RR_EX(115 downto 100) := out_pc_r;
	RR_EX(99 downto 84) := out_pc_inc_r;
	RR_EX(83 downto 68) := out_ra_r;
	RR_EX(67 downto 52) := out_rb_r;
	RR_EX(51 downto 36) := out_instruction_r;
	RR_EX(35 downto 20) := out_control_word_r;
	RR_EX(19 downto 4) := out_jump_add_r;
	RR_EX(3) := prop_rr;
	RR_EX(2) := en_rr;
	RR_EX(1) := ztake;
	RR_EX(0) := ctake;
	
	
	EX_MEM(98 downto 83) := lm_address_out_e;
	EX_MEM(82 downto 67) := out_pc_e;
	EX_MEM(66 downto 51) := out_pc_inc_e;
	EX_MEM(50 downto 35) := alu_out_e;
	EX_MEM(34 downto 19) := out_instruction_e;
	EX_MEM(18 downto 3) := out_control_word_e;
	EX_MEM(2) := prop_ex;
	EX_MEM(1) := en_ex;
	EX_MEM(0) := clk;
	
	MEM_WR(71 downto 56) := out_instruction_m;
	MEM_WR(55 downto 40) := out_control_word_m;
	MEM_WR(39 downto 24) := out_data_m;
	MEM_WR(23 downto 8) := out_pc_inc_m;
	MEM_WR(7 downto 5) := out_lm_sm_address_m; --Data address 1 is not given
	MEM_WR(4 downto 2) := "111";
	MEM_WR(1) := prop_mem;
	MEM_WR(0) := en_mem;
	
	enable_bit_lm := out_ls_enable_d;
	opcode_id := out_instruction_d(15 downto 12);
	opcode_rr := out_instruction_r(15 downto 12);
	opcode_f := out_instruction_f(15 downto 12);
	
	if(bmem_bex_hazard_ra = '1') then
		RR_EX(83 downto 68) := bmem_add;
	elsif(bwb_bex_hazard_ra = '1') then
		RR_EX(83 downto 68) := bwb_add;
	end if;
	
	if(bmem_bex_hazard_rb = '1') then
		RR_EX(67 downto 52) := bmem_add;
	elsif(bwb_bex_hazard_rb = '1') then
		RR_EX(67 downto 52) := bwb_add;
	end if;
	
	if(bmem_bex_hazard_rb = '1' and jlr_happens = '1') then
		RR_EX(99 downto 84) := bmem_add;
		jlr_add := bmem_add;
	elsif(bwb_bex_hazard_rb = '1' and jlr_happens = '1') then
		RR_EX(99 downto 84) := bwb_add;
		jlr_add := bwb_add;
	end if;
	
	if((RR_EX(51 downto 48) = "0110" or RR_EX(51 downto 48) = "0111") and (EX_MEM(34 downto 31) = "0110" or EX_MEM(34 downto 31) = "0111") and prop_rr = '1' and prop_ex = '1')  then
		RR_EX(83 downto 68) := EX_MEM(98 downto 83);
	end if;

	if(beq_happens = '1' and prop_ex = '1') then
		next_pc := beq_add;
		prop_if := '0';
		prop_id := '0';
		prop_rr := '0';
		RR_EX(3) := '0';
		ID_RR(6) := '0';
		IF_ID(2) := '0';
	elsif(jlr_happens = '1' and prop_rr = '1') then
		next_pc := jlr_add;
		prop_if := '0';
		prop_id := '0';
		ID_RR(6) := '0';
		IF_ID(2) := '0';
	elsif(jal_happens = '1' and prop_id = '1') then
		next_pc := jal_add;
		prop_if := '0';
		IF_ID(2) := '0';
	end if;
	
	

	case q_var is
	
		when "00" =>
			sm_lm_mux_con := '0';
			first_lm_sm := '0';
			if((opcode_rr = "0100" or opcode_rr = "0011") and prop_rr = '1') then
				nq_var := "10";
				
			elsif((opcode_f = "0110" or opcode_f = "0111") and prop_if = '1') then
				first_lm_sm := '1';
				nq_var := "01";
			
			else nq_var := "00";
			
			end if;
		
		when "01" =>
			first_lm_sm := '0';
			sm_lm_mux_con := '1';
			if(enable_bit_lm = '0') then
--				first_lm_sm := '1';
				nq_var := "01";
			elsif(flag_for_zero = '1') then
				IF_ID(2) := '0';
				nq_var := "11";
			else
				IF_ID(2) := '0';
				ID_RR(6) := '0';
				nq_var := "00";
			end if;
				
		when "10" =>
			first_lm_sm := '0';
--			first_lm_sm := not(IF_ID_interface(0));
			sm_lm_mux_con := '0';
			if((opcode_f = "0110" or opcode_f = "0111") and prop_id = '1') then
				first_lm_sm := '1';
				nq_var := "01";
			else
				nq_var := "00";
			end if;
			
		when "11" =>	
			first_lm_sm := '0';
			sm_lm_mux_con := '0';
--			IF_ID(2) := '0';
			nq_var := "00";
		
		when others => null;
	
	end case;
	
	IF_ID(0) := sm_lm_mux_con;


	
	if(clk'event and clk = '1') then
		
			 ID_RR_interface(28 downto 26) <= ID_RR(28 downto 26);
			 in_lm_sm_address_e <= lmsme;
			 in_lm_sm_address_m <= lmsmm; 
			 in_start_bit_r <= start_bit_r;
			 in_start_bit_e <= start_bit_e;
			 in_r7_bit_m <= r7_bit_e;
			 
			 in_prop_r <= out_prop_d;
			 in_prop_e <= out_prop_r;
			 in_prop_m <= out_prop_e;
			 in_prop_w <= out_prop_m;
			 
          if(r = '1') then
            
				 in_enable_f <= '1';
				 in_prop_f <= '1';
				 next_pc := "0000000000000000";
				 in_pc_f <= next_pc;
				 IF_ID_interface(2) <= '0';
				 ID_RR_interface(6) <= '0';
				 RR_EX_interface(3) <= '0';
				 EX_MEM_interface(2) <= '0';
				 MEM_WR_interface(1) <= '0';
				 beq_happens := '0';
				 jlr_happens := '0';
				 jal_happens := '0';
				 q_var <= "00";
			    RR_EX_interface(1) <= '0';
				 RR_EX_interface(0) <= '0';
			
			 else

				 q_var <= nq_var;
				 
				 if(nq_var = "00") then
			 
					in_enable_f <= '1';
					in_prop_f <= '1';
					in_pc_f <= next_pc;
					IF_ID_interface <= IF_ID;
					ID_RR_interface(124 downto 61) <= ID_RR(124 downto 61);
					ID_RR_interface(22 downto 5) <= ID_RR(22 downto 5);
					ID_RR_interface(1 downto 0) <= ID_RR(1 downto 0);
					RR_EX_interface(131 downto 2) <= RR_EX(131 downto 2);
					if(z_load = '1') then
						RR_EX_interface(1) <= z_ok;
					else
						RR_EX_interface(1) <= RR_EX(1);
					end if;
					RR_EX_interface(0) <= RR_EX(0);
					EX_MEM_interface <= EX_MEM;
					MEM_WR_interface <= MEM_WR;
					
				 
				 elsif(nq_var = "01") then
				 
					if(first_lm_sm = '1') then
						IF_ID_interface <= IF_ID;
					end if;
				 
					--in_enable_f <= '0';
					IF_ID_interface(0) <= IF_ID(0);
					ID_RR_interface(124 downto 61) <= ID_RR(124 downto 61);
					ID_RR_interface(22 downto 5) <= ID_RR(22 downto 5);
					ID_RR_interface(1 downto 0) <= ID_RR(1 downto 0);
					RR_EX_interface(131 downto 2) <= RR_EX(131 downto 2);
						if(z_load = '1') then
							RR_EX_interface(1) <= z_ok;
						else
							RR_EX_interface(1) <= RR_EX(1);
						end if;
					RR_EX_interface(0) <= RR_EX(0);
					EX_MEM_interface <= EX_MEM;
					MEM_WR_interface <= MEM_WR;
				 
				 elsif(nq_var = "10") then
					
					ID_RR_interface(124 downto 61) <= ID_RR(124 downto 61);
					ID_RR_interface(22 downto 7) <= ID_RR(22 downto 7);
					ID_RR_interface(6) <= '0';
					ID_RR_interface(5) <= ID_RR(5);
					ID_RR_interface(1 downto 0) <= ID_RR(1 downto 0);
					RR_EX_interface(131 downto 2) <= RR_EX(131 downto 2);
						if(z_load = '1') then
							RR_EX_interface(1) <= z_ok;
						else
							RR_EX_interface(1) <= RR_EX(1);
						end if;
					RR_EX_interface(0) <= RR_EX(0);
					EX_MEM_interface <= EX_MEM;
					MEM_WR_interface <= MEM_WR;
					
				 else
--				 
					in_enable_f <= '1';
					in_prop_f <= '1';
					in_pc_f <= next_pc;
					
					IF_ID_interface <= IF_ID;
					ID_RR_interface(124 downto 61) <= ID_RR(124 downto 61);
					ID_RR_interface(22 downto 5) <= ID_RR(22 downto 5);
					ID_RR_interface(1 downto 0) <= ID_RR(1 downto 0);
					RR_EX_interface(131 downto 2) <= RR_EX(131 downto 2);
					if(z_load = '1') then
						RR_EX_interface(1) <= z_ok;
					else
						RR_EX_interface(1) <= RR_EX(1);
					end if;
					RR_EX_interface(0) <= RR_EX(0);
					EX_MEM_interface <= EX_MEM;
					MEM_WR_interface <= MEM_WR;
	
				 
				 end if;
					 
					 
				 end if;			 
			 
			 
			 
			 
     end if;
	
	
	end process;


end Project1;