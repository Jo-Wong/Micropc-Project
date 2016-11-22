/////////////////////////////////////////////
// testbench
//   Tests quad_motor_control
/////////////////////////////////////////////

module testbench();
    logic clk, reset, sck, sdi, cs;
    logic [31:0] motor_number;
    logic [8:0] i;
    logic [3:0] f_esc;
    logic [3:0] test_nums;
	 
    // device under test
    quad_motor_control dut(clk, reset, sck, sdi, cs, f_esc);

    // test case
    initial begin   
	 // Test case
        motor_number <= 32'hAAFF0077;
    end
    
    // generate clock and load signals
    initial 
        forever begin
            clk = 1'b0; #1;
            clk = 1'b1; #1;
        end
        
    initial begin
	reset = 1'b0;
	#1;
	reset = 1'b1;
	#1;
 	reset = 1'b0;
   	i = 0;
	test_nums = 0;
  	cs = 1'b1;
	sdi = 1'b0;
	sck = 1'b0;
    end 
    
    // shift in test vectors, wait until done, and shift out result
    always @(posedge clk) 
	 begin
			if (i<32)
			begin
				#1; sdi = motor_number[31-i];
				cs = 1'b0;
				#1; sck = 1; #2; sck = 0;
				i = i + 1;
			end 
			else if (i == 32)
			begin
				#1;
				cs = 1'b1;
				#3000000; // wait 
				reset = 1'b1;
				#1;
				reset = 1'b0;
				#1;
				test_nums = test_nums + 1;
            			i = 0;
			end
			else if (test_nums >= 2)
			begin
				$display("Testbench finished running.");
            $stop();
			end
    end
    
endmodule
