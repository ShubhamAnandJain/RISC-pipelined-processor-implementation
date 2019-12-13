library ieee;
use ieee.std_logic_1164.all;

entity RF is
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

end entity;

architecture Wrk of RF is

type RegisterSet is  array(0 to 7) of std_logic_vector(15 downto 0);
signal RegisterNo : RegisterSet:=(0 => x"0001", 1 => x"0011", 2 => x"0001",others => x"0000") ;
signal readen,writeen,c0, c1, c2, c3, c4, c5, c6, c7, b0, b1, b2, b3, b4, b5, b6, b7 : std_logic_vector(15 downto 0);
type TempSet is  array(0 to 7) of std_logic_vector(15 downto 0);
signal TempNo : TempSet;


begin
readen <= (others => Read_En);

O0 <= RegisterNo(0);
O1 <= RegisterNo(1);
O2 <= RegisterNo(2);
O3 <= RegisterNo(3);
O4 <= RegisterNo(4);
O5 <= RegisterNo(5);
O6 <= RegisterNo(6);
O7 <= RegisterNo(7);

--reading first operand
c0 <= (others => ((not Out_select_1(0)) and (not Out_select_1(1)) and (not Out_select_1(2))));
c1 <= (others => ((Out_select_1(0)) and (not Out_select_1(1)) and (not Out_select_1(2))));
c2 <= (others => ((not Out_select_1(0)) and (Out_select_1(1)) and (not Out_select_1(2))));
c3 <= (others => ((Out_select_1(0)) and (Out_select_1(1)) and (not Out_select_1(2))));
c4 <= (others => ((not Out_select_1(0)) and (not Out_select_1(1)) and (Out_select_1(2))));
c5 <= (others => ((Out_select_1(0)) and (not Out_select_1(1)) and (Out_select_1(2))));
c6 <= (others => ((not Out_select_1(0)) and (Out_select_1(1)) and (Out_select_1(2))));
c7 <= (others => ((Out_select_1(0)) and (Out_select_1(1)) and (Out_select_1(2))));


	Read_Data_1 <= (readen and c0 and RegisterNo(0)) or (readen and c1 and RegisterNo(1))
or (readen and c2 and RegisterNo(2)) or (readen and c3 and RegisterNo(3))
or (readen and c4 and RegisterNo(4)) or (readen and c5 and RegisterNo(5))
or (readen and c6 and RegisterNo(6)) or (readen and c7 and RegisterNo(7));


--reading second operand
b0 <= (others => ((not Out_select_2(0)) and (not Out_select_2(1)) and (not Out_select_2(2))));
b1 <= (others => ((Out_select_2(0)) and (not Out_select_2(1)) and (not Out_select_2(2))));
b2 <= (others => ((not Out_select_2(0)) and (Out_select_2(1)) and (not Out_select_2(2))));
b3 <= (others => ((Out_select_2(0)) and (Out_select_2(1)) and (not Out_select_2(2))));
b4 <= (others => ((not Out_select_2(0)) and (not Out_select_2(1)) and (Out_select_2(2))));
b5 <= (others => ((Out_select_2(0)) and (not Out_select_2(1)) and (Out_select_2(2))));
b6 <= (others => ((not Out_select_2(0)) and (Out_select_2(1)) and (Out_select_2(2))));
b7 <= (others => ((Out_select_2(0)) and (Out_select_2(1)) and (Out_select_2(2))));


	Read_Data_2 <= (readen and b0 and RegisterNo(0)) or (readen and b1 and RegisterNo(1))
or (readen and b2 and RegisterNo(2)) or (readen and b3 and RegisterNo(3))
or (readen and b4 and RegisterNo(4)) or (readen and b5 and RegisterNo(5))
or (readen and b6 and RegisterNo(6)) or (readen and b7 and RegisterNo(7));


process(clk,rst)
--variable index : unsigned(2 downto 0);
begin
	if(clk'event and clk='1' and rst = '0') then
		if(Write_En(1) = '1') then
				case In_select_1 is
				 when "000" =>
				 RegisterNo(0) <= Write_Data_1 ;
				 
				 when "001" =>
				 RegisterNo(1) <= Write_Data_1 ;
				 
				 when "010" =>
				 RegisterNo(2) <= Write_Data_1 ;
				 
				 when "011" =>
				 RegisterNo(3) <= Write_Data_1 ;
				 
				 when "100" =>
				 RegisterNo(4) <= Write_Data_1 ;
				 
				 when "101" =>
				 RegisterNo(5) <= Write_Data_1 ;
				 
				 when "110" =>
				 RegisterNo(6) <= Write_Data_1 ;
				 
				 when "111" =>
				 RegisterNo(7) <= Write_Data_1; 
				 
				 when others => null;
			
			end case;
							 
	end if;
	
	if(Write_En(0) = '1') then
				case In_select_2 is
				 when "000" =>
				 RegisterNo(0) <= Write_Data_2 ;
				 
				 when "001" =>
				 RegisterNo(1) <= Write_Data_2 ;
				 
				 when "010" =>
				 RegisterNo(2) <= Write_Data_2 ;
				 
				 when "011" =>
				 RegisterNo(3) <= Write_Data_2 ;
				 
				 when "100" =>
				 RegisterNo(4) <= Write_Data_2 ;
				 
				 when "101" =>
				 RegisterNo(5) <= Write_Data_2 ;
				 
				 when "110" =>
				 RegisterNo(6) <= Write_Data_2 ;
				 
				 when "111" =>
				 RegisterNo(7) <= Write_Data_2 ; 
				 
				 when others => null;
			
			end case;
							 
	end if;
	
end if;
end process;


end Wrk;