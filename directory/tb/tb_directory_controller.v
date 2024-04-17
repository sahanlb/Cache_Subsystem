/** @module : tb_directory_controller
 *  @author : Adaptive & Secure Computing Systems (ASCS) Laboratory
 *  @author : Sahan Bandara

 *  Copyright (c) 2019 BRISC-V (ASCS/ECE/BU)
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

/*******************************************************************************
* Module: tb_directory_controller
*******************************************************************************/

module tb_directory_controller();

//define the log2 function
function integer log2;
input integer value;
begin
  value = value-1;
  for (log2=0; value>0; log2=log2+1)
    value = value >> 1;
end
endfunction

`include `INCLUDE_FILE


parameter OFFSET_BITS         =  2,
          DATA_WIDTH          = 32,
          ADDRESS_BITS        = 32,
          MSG_BITS            =  4,
		      MAX_OFFSET_BITS     =  3,
          REQ_BUF_DEPTH_BITS  =  2,
          RESP_BUF_DEPTH_BITS =  2,
          NUM_SHARERS_BITS    =  1, //log2(No. of pointers)
          SHARER_ID_BITS      =  3,
          DIR_INDEX_BITS      =  8,
          DIR_WAYS            =  2,
          DEFAULT_DEST        =  0,
          ACTIVE_REQS         =  2, 
          //Use default value in module instantiation for following parameters
          CACHE_WORDS         = 1 << OFFSET_BITS,
          CACHE_WIDTH         = DATA_WIDTH * CACHE_WORDS,
          WAY_BITS            = (DIR_WAYS > 1) ? log2(DIR_WAYS) : 1;


reg  clock;
reg  reset;
//interface with the NoC
reg  [MSG_BITS-1      :0] noc_msg_in;
reg  [ADDRESS_BITS-1  :0] noc_address_in;
reg  [CACHE_WIDTH-1   :0] noc_data_in;
reg  [SHARER_ID_BITS-1:0] noc_src_id;
reg  packetizer_busy;
wire [MSG_BITS-1      :0] noc_msg_out;
wire [ADDRESS_BITS-1  :0] noc_address_out;
wire [CACHE_WIDTH-1   :0] noc_data_out;
wire [SHARER_ID_BITS-1:0] noc_dest_id;
//memory interface
reg  [MSG_BITS-1      :0] mem_msg_in;
reg  [ADDRESS_BITS-1  :0] mem_address_in;
reg  [CACHE_WIDTH-1   :0] mem_data_in;
wire [MSG_BITS-1      :0] mem_msg_out;
wire [ADDRESS_BITS-1  :0] mem_address_out;
wire [CACHE_WIDTH-1   :0] mem_data_out;

reg  report;


//instantiate directory controller
directory_controller #(
  .OFFSET_BITS(OFFSET_BITS),
  .DATA_WIDTH(DATA_WIDTH),
  .ADDRESS_BITS(ADDRESS_BITS),
  .MSG_BITS(MSG_BITS),
  .REQ_BUF_DEPTH_BITS(REQ_BUF_DEPTH_BITS),
  .RESP_BUF_DEPTH_BITS(RESP_BUF_DEPTH_BITS),
  .NUM_SHARERS_BITS(NUM_SHARERS_BITS), 
  .SHARER_ID_BITS(SHARER_ID_BITS),
  .DIR_INDEX_BITS(DIR_INDEX_BITS),
  .DIR_WAYS(DIR_WAYS),
  .DEFAULT_DEST(DEFAULT_DEST),
  .ACTIVE_REQS(ACTIVE_REQS)
) DUT (
  .clock(clock),
  .reset(reset),
  .noc_msg_in(noc_msg_in),
  .noc_address_in(noc_address_in),
  .noc_data_in(noc_data_in),
  .noc_src_id(noc_src_id),
  .packetizer_busy(packetizer_busy),
  .noc_msg_out(noc_msg_out),
  .noc_address_out(noc_address_out),
  .noc_data_out(noc_data_out),
  .noc_dest_id(noc_dest_id),
  .mem_msg_in(mem_msg_in),
  .mem_address_in(mem_address_in),
  .mem_data_in(mem_data_in),
  .mem_msg_out(mem_msg_out),
  .mem_address_out(mem_address_out),
  .mem_data_out(mem_data_out),
  .report(report)
);


//clock generation
always #1 clock = ~clock;

//cycle counter
reg [31:0] cycles;

always @(posedge clock)
  cycles = cycles + 1;


//test inputs
initial begin
  clock           = 0;
  reset           = 0;
  cycles          = 0;
  noc_msg_in      = NoMsg;
  noc_address_in  = 0;
  noc_data_in     = 0;
  noc_src_id      = 0;
  packetizer_busy = 1'b0;
  mem_msg_in      = NO_REQ;
  mem_address_in  = 0;
  mem_data_in     = 0;
  report          = 0;

  repeat(1) @(posedge clock);
  @(posedge clock) reset <= 1'b1;
  repeat(1) @(posedge clock);
  @(posedge clock) reset <= 1'b0;

  //wait for the reset sequence
  repeat(256) @(posedge clock);

  //read request
  @(posedge clock)begin
    noc_msg_in     <= GetS;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b011;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
  end

  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  wait(noc_msg_out == Data & noc_address_out == 32'h00000100 & noc_dest_id == 3'd3 &
  noc_data_out == 128'h00000001_00000002_00000003_00000004);
  
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= GetS;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b111;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
  end

  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  wait(noc_msg_out == FwdGetS & noc_address_out == 32'h00000100 & noc_dest_id == 3'd3);
  //cache '3' demotes the line to S and reply with PutE
  repeat(4) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= PutE;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b011;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
  end

  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  wait(noc_msg_out == DataS & noc_address_out == 32'h00000100 & noc_dest_id == 3'd7 &
  noc_data_out == 128'h00000001_00000002_00000003_00000004);

  //3rd sharer request the same line
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= GetS;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b100;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
  end

  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  //check invalidation request sent out
  wait(noc_msg_out == Inv & noc_address_out == 32'h00000100 & noc_dest_id == 3'd3);

  //request for a different memory block
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= GetM;
    noc_address_in <= 32'h50000100; 
    noc_src_id     <= 3'b100;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
  end

  wait(mem_msg_out == R_REQ & mem_address_out == 32'h50000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h50000001_50000002_50000003_50000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  //check response to GetM
  wait(noc_msg_out == Data & noc_address_out == 32'h50000100 & noc_dest_id == 3'd4 &
  noc_data_out == 128'h50000001_50000002_50000003_50000004);

  //another cache requesting the line in modified state
  repeat(3) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= GetS;
    noc_address_in <= 32'h50000100; 
    noc_src_id     <= 3'b110;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
  end

  wait(mem_msg_out == R_REQ & mem_address_out == 32'h50000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h50000001_50000002_50000003_50000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  //check invalidation request sent to cache '4'
  wait(noc_msg_out == FwdGetS & noc_address_out == 32'h50000100 & noc_dest_id == 3'd4);

  //response from cache '3' to Inv request
  @(posedge clock)begin
    noc_msg_in     <= InvAck;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b011;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
  end

  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  wait(noc_msg_out == DataS & noc_address_out == 32'h00000100 & noc_dest_id == 3'd4 &
  noc_data_out == 128'h00000001_00000002_00000003_00000004);

  //cache '4' writes back the modified line in response to FwdGetS
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= RespPutM;
    noc_address_in <= 32'h50000100; 
    noc_src_id     <= 3'b100;
    noc_data_in    <= 128'h60000001_60000002_60000003_60000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
    noc_data_in    <= 0;
  end

  //writeback to memory
  wait(mem_msg_out == WB_REQ & mem_address_out == 32'h50000100 & 
  mem_data_out == 128'h60000001_60000002_60000003_60000004);

  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  //No PutAck for the RespPutM.

  //read memory
  wait(mem_msg_out == R_REQ & mem_address_out == 32'h50000100);

  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h60000001_60000002_60000003_60000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  //respond to cache '6' with Data
  wait(noc_msg_out == Data & noc_address_out == 32'h50000100 & noc_dest_id == 3'd6 &
  noc_data_out == 128'h60000001_60000002_60000003_60000004);

  //cache '1' request a line in shared state to modify (GetM)
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= GetM;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b001;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
    noc_data_in    <= 0;
  end
  
  //memory read
  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  //invalidation messages
  wait(noc_msg_out == Inv & noc_address_out == 32'h00000100 & noc_dest_id == 3'd4);
  wait(noc_msg_out == Inv & noc_address_out == 32'h00000100 & noc_dest_id == 3'd7);
  
  //memory read
  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  //two caches reply to invalidation requests
  @(posedge clock)begin
    noc_msg_in     <= InvAck;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b100;
  end
  @(posedge clock)begin
    noc_msg_in     <= InvAck;
    noc_address_in <= 32'h00000100; 
    noc_src_id     <= 3'b111;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0; 
    noc_src_id     <= 0;
    noc_data_in    <= 0;
  end

  //memory read
  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  wait(mem_msg_out == NO_REQ);

  //memory read
  wait(mem_msg_out == R_REQ & mem_address_out == 32'h00000100);
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    mem_msg_in     <= MEM_RESP;
    mem_address_in <= mem_address_out;
    mem_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    mem_msg_in     <= NO_REQ;
    mem_address_in <= 0;
    mem_data_in    <= 0;
  end

  wait(noc_msg_out == Data & noc_address_out == 32'h00000100 & noc_dest_id == 3'd1 &
  noc_data_out == 128'h00000001_00000002_00000003_00000004);

  #10;
  $display("\ntb_directory_controller --> Test Passed!\n\n");
  $stop;
end

//timeout
initial begin
  #1000;
  $display("\ntb_directory_controller --> Test Failed!\n\n");
  $stop;
end

endmodule
