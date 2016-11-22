/////////////////////////////////////////////
// all_controls
// Takes SPI, outputs motor control signals
// {f1, f2, f3, f4} = front - left - right - back motors
// FPGA clk is 40MHz
/////////////////////////////////////////////
module quad_motor_control(input logic clk, input logic reset, input logic sck, input logic sdi, input logic cs,
									output logic[3:0] f_esc, output logic slowclk, output done);
	//logic slowclk;
	//logic done;
	logic[7:0] f1_motor, f2_motor, f3_motor, f4_motor;
	logic[31:0] allf;

	// Divide 40MHz FPGA clock into 256kHz clock (divide by about 2^7)
	generate_acc_divclk#8 clk_div1(clk, slowclk);
	
	spi_interface spi1(reset, sck, sdi, cs, allf, done);

	register f1_motor_reg(clk, done, allf[31:24], f1_motor);
	register f2_motor_reg(clk, done, allf[23:16], f2_motor);
	register f3_motor_reg(clk, done, allf[15:8], f3_motor);
	register f4_motor_reg(clk, done, allf[7:0], f4_motor);
	
	ESCcontrol motor1(slowclk, cs, f1_motor, f_esc[0]);
	ESCcontrol motor2(slowclk, cs, f2_motor, f_esc[1]);
	ESCcontrol motor3(slowclk, cs, f3_motor, f_esc[2]);
	ESCcontrol motor4(slowclk, cs, f4_motor, f_esc[3]);
endmodule

/////////////////////////////////////////////
// spi_interface
//   Half-duplex SPI interface. Shifts in 4 8-bit numbers, 
//   Can only receive, can't send (hence no sdo)
/////////////////////////////////////////////
module spi_interface(
					input logic reset,
					input  logic sck, 
               input  logic sdi,
               input  logic cs,
               output logic [31:0] motor_number,
					output logic done);
   logic [5:0] count;
	logic should_shift;
	
	// then deassert load, wait until done
	// apply (4 * 8 bit = 32) sclks to shift in motor_number[0]
   always_ff @(posedge sck)
	begin
        	if (should_shift)  motor_number[31:0] = {motor_number[30:0], sdi}; 
	end	
	
	// Counter here to count number of motor numbers done
	counter#6 motor_num_counter(sck, cs|done, count);
	
	assign done = (count == 6'b100000); // = 32
	
	assign should_shift = (~cs) & (count < 6'b100000);
endmodule

module register#(parameter N = 8)(input logic clk, input logic en, input logic [N-1:0] din, output logic [N-1:0] dout);
	always_ff@(posedge clk)
	begin
		if(en) dout <= din;
	end
endmodule

module ESCcontrol#(parameter COUNT_20MS = 16'd10240, COUNT_1MS = 16'd512)(input logic clk, input logic reset, input logic[7:0] motor_reg, output logic ESCout);
	// ESC neutral is at 1ms pulse, further counts bring up duty cycle

	logic reset_count;
	logic[15:0] count;
	//logic[12:0] t1, t2;
	// Use 256kHz clock to manage ESC counter, big enough to hold up to counts to 20ms
	counter#16 counter1(clk, reset_count|reset, count);

	// assign t1 = count + countoffset;
	// assign t2 = motor_reg + countoffset;
	assign reset_count = (count == COUNT_20MS);
	assign ESCout = (count < ((motor_reg<<1) + COUNT_1MS));
endmodule

//// Counter with neg-edge reset
//module nr_counter#(parameter N = 16)(input logic clk, reset, output logic[N-1:0] count);
//	always_ff@(negedge clk, negedge reset)
//	begin
//		if(~reset && clk) count <= 0;
//		else count <= count + 1;
//	end
//endmodule

module counter#(parameter N = 16)(input logic clk, reset, output logic[N-1:0] count);
	initial begin
		count <= 0;
	end	
	always_ff@(posedge clk, posedge reset)
	begin
		if(reset) count <= 0;
		else count <= count + 1;
	end
endmodule

module en_counter#(parameter N = 16)(input logic clk, en, output logic[N-1:0] count);
	initial begin
		count <= 0;
	end	
	always_ff@(posedge clk)
	begin
		if(en) count <= count + 1;
	end
endmodule

module nr_counter#(parameter N = 16)(input logic clk, output logic[N-1:0] count);
	initial begin
		count <= 0;
	end
	always_ff@(posedge clk)
	begin
		count <= count + 1;
	end
endmodule

// Slowed clock by a factor of 2^N 
module generate_divclk#(parameter N = 4)(input logic clk, output logic slowclk);
	logic[N-1:0] count;
	nr_counter#(N) cnt(clk, count);
	assign slowclk = count[N-1];
endmodule

// Slowed clock by more accurate method
// 40MHz -> 256kHz means 156.25 ticks, half on half off 78ticks each
module generate_acc_divclk#(parameter N = 8)(input logic clk, output logic slowclk);
	logic[N-1:0] count;
	logic reset;
	counter#(N) cnt(clk, reset, count);
	assign reset = (count == 8'b1001_1100);
	assign slowclk = (count < 8'b0100_1110);
endmodule
