/** @module : tb_message_handler
 *  @author : Adaptive & Secure Computing Systems (ASCS) Laboratory
 *  @author : Sahan Bandara

 *  Copyright (c) 2018 BRISC-V (ASCS/ECE/BU)
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
* Module: tb_message_handler
*******************************************************************************/

module tb_message_handler();

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
//`include "../params.h"

parameter CACHE_OFFSET_BITS   =  2,
          DATA_WIDTH          = 32,
          ADDRESS_BITS        = 32,
          MSG_BITS            =  4,
          REQ_BUF_DEPTH_BITS  =  2,
          RESP_BUF_DEPTH_BITS =  2,
          PARENT              = "CACHE",
          ID_BITS             =  3,
          DEFAULT_DEST        =  0,
          CACHE_WORDS         = 1 << CACHE_OFFSET_BITS,
          CACHE_WIDTH         = DATA_WIDTH * CACHE_WORDS,
          BUF_WIDTH           = CACHE_WIDTH + MSG_BITS + ADDRESS_BITS + ID_BITS;

/*DUT signals*/
reg clock, reset;
//interface with the NoC
reg [MSG_BITS-1    :0] noc_msg_in;
reg [ADDRESS_BITS-1:0] noc_address_in;
reg [CACHE_WIDTH-1 :0] noc_data_in;
reg [ID_BITS-1     :0] noc_src_id;
reg packetizer_busy;
wire [MSG_BITS-1    :0] noc_msg_out;
wire [ADDRESS_BITS-1:0] noc_address_out;
wire [CACHE_WIDTH-1 :0] noc_data_out;
wire [ID_BITS-1     :0] noc_dest_id;
//interface with controller of the parent module
reg [MSG_BITS-1    :0] ctrl_msg_in;
reg [ADDRESS_BITS-1:0] ctrl_address_in;
reg [CACHE_WIDTH-1 :0] ctrl_data_in;
reg [ID_BITS-1     :0] ctrl_dest_id;
wire intf_busy;
//exposed FIFO read interfaces
reg reqbuf_read;
reg respbuf_read;
wire reqbuf_empty;
wire reqbuf_full;
wire reqbuf_valid;
wire respbuf_empty;
wire respbuf_full;
wire respbuf_valid;
wire [BUF_WIDTH-1:0] reqbuf_data;
wire [BUF_WIDTH-1:0] respbuf_data;

/*instantiate DUT*/
message_handler #(
  .CACHE_OFFSET_BITS(CACHE_OFFSET_BITS),
  .DATA_WIDTH(DATA_WIDTH),
  .ADDRESS_BITS(ADDRESS_BITS),
  .MSG_BITS(MSG_BITS),
  .REQ_BUF_DEPTH_BITS(REQ_BUF_DEPTH_BITS),
  .RESP_BUF_DEPTH_BITS(RESP_BUF_DEPTH_BITS),
  .PARENT(PARENT),
  .ID_BITS(ID_BITS),
  .DEFAULT_DEST(DEFAULT_DEST)
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
  .ctrl_msg_in(ctrl_msg_in),
  .ctrl_address_in(ctrl_address_in),
  .ctrl_data_in(ctrl_data_in),
  .ctrl_dest_id(ctrl_dest_id),
  .intf_busy(intf_busy),
  .reqbuf_read(reqbuf_read),
  .respbuf_read(respbuf_read),
  .reqbuf_empty(reqbuf_empty),
  .reqbuf_full(reqbuf_full),
  .reqbuf_valid(reqbuf_valid),
  .respbuf_empty(respbuf_empty),
  .respbuf_full(respbuf_full),
  .respbuf_valid(respbuf_valid),
  .reqbuf_data(reqbuf_data),
  .respbuf_data(respbuf_data)
);

/*clock signal*/
always #1 clock = ~clock;

//cycle counter
reg [31:0] cycles;
always @(posedge clock)begin
  cycles <= cycles + 1;
end

/*test patterns*/
initial begin
  clock           = 0;
  reset           = 0;
  cycles          = 0;
  noc_msg_in      = NoMsg;
  noc_data_in     = 0;
  noc_address_in  = 0;
  noc_src_id      = 0;
  packetizer_busy = 0;
  ctrl_msg_in     = NoMsg;
  ctrl_data_in    = 0;
  ctrl_address_in = 0;
  ctrl_dest_id    = 0;
  reqbuf_read     = 0;
  respbuf_read    = 0;

  //reset the module
  @(posedge clock)begin
    reset <= 1;
  end
  @(posedge clock)begin
    reset <= 0;
  end

  //multiple requests and responses from NoC side interleaved.
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= FwdGetS;
    noc_address_in <= 32'h10000010;
  end
  @(posedge clock)begin
    noc_msg_in     <= FwdGetS;
    noc_address_in <= 32'h10000020;
  end
  @(posedge clock)begin
    noc_msg_in     <= Data;
    noc_address_in <= 32'h10000030;
    noc_data_in    <= 128'h11111111_22222222_33333333_44444444;
  end
  @(posedge clock)begin
    noc_msg_in     <= PutAck;
    noc_address_in <= 32'h10000040;
    noc_data_in    <= 128'h0;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0;
  end
  repeat(1) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h10000050;
    noc_src_id     <= 3'd2;
  end
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h10000060;
    noc_src_id     <= 3'd0;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0;
  end
  repeat(2) @(posedge clock);
  //check the contents of the buffer
  if(DUT.req_buffer.queue[0] !== {3'd0, FwdGetS , 32'h10000010, 128'h0} |
     DUT.req_buffer.queue[1] !== {3'd0, FwdGetS , 32'h10000020, 128'h0} |
     DUT.req_buffer.queue[2] !== {3'd2, Inv, 32'h10000050, 128'h0} |
     DUT.req_buffer.queue[3] !== {3'd0, Inv, 32'h10000060, 128'h0} )
  begin
    $display("Wrong values in buffers!\nCycles: %d", cycles);
    $display("%d, %d, %h, %h", DUT.req_buffer.queue[0][166:164],
      DUT.req_buffer.queue[0][163:160],
      DUT.req_buffer.queue[0][159:128],
      DUT.req_buffer.queue[0][127:0]);
    $display("%d, %d, %h, %h", DUT.req_buffer.queue[1][166:164],
      DUT.req_buffer.queue[1][163:160],
      DUT.req_buffer.queue[1][159:128],
      DUT.req_buffer.queue[1][127:0]);
    $display("%d, %d, %h, %h", DUT.req_buffer.queue[2][166:164],
      DUT.req_buffer.queue[2][163:160],
      DUT.req_buffer.queue[2][159:128],
      DUT.req_buffer.queue[2][127:0]);
    $display("%d, %d, %h, %h", DUT.req_buffer.queue[3][166:164],
      DUT.req_buffer.queue[3][163:160],
      DUT.req_buffer.queue[3][159:128],
      DUT.req_buffer.queue[3][127:0]);
    $stop;
  end
    
  //Send request when request buffer is full
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h20002000;
    noc_src_id     <= 3'd5;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0;
    noc_src_id     <= 3'd0;
  end
  repeat(2) @(posedge clock);
  //reject message from DUT
  if(noc_msg_out != NackC | noc_address_out != 32'h20002000 | ~intf_busy |
  noc_dest_id != 3'd5)
  begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Error with Nack message!\nCycles: %d", cycles);
    $stop;
  end

  //message from controller side
  @(posedge clock)begin
    ctrl_msg_in     <= PutS;
    ctrl_address_in <= 32'h40002000;
    ctrl_data_in    <= 128'h10000000_20000000_30000000_40000000;
  end
  @(posedge clock)begin
    ctrl_msg_in     <= NoMsg;
    ctrl_address_in <= 0;
    ctrl_data_in    <= 0;
  end
  //check output to NoC
  repeat(1) @(posedge clock);
  if(noc_msg_out != PutS | noc_address_out != 32'h40002000 |
     noc_data_out != 128'h10000000_20000000_30000000_40000000)begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Error passing controller message to NoC!\nCycles: %d", cycles);
    $stop;
  end
  
  //request while req buffer is full and contrller is sending a message.
  @(posedge clock)begin
    ctrl_msg_in     <= PutM;
    ctrl_address_in <= 32'h50005000;
    ctrl_data_in    <= 128'h10000001_20000002_30000003_40000004;
    ctrl_dest_id    <= 3'd7;
    noc_msg_in      <= Inv;
    noc_address_in  <= 32'h20002000;
  end
  @(posedge clock)begin
    ctrl_msg_in     <= NoMsg;
    ctrl_address_in <= 0;
    ctrl_data_in    <= 0;
    ctrl_dest_id    <= 3'd0;
    noc_msg_in      <= NoMsg;
    noc_address_in  <= 0;
  end
  //check outputs
  repeat(1) @(posedge clock);
  if(noc_msg_out != PutM | noc_address_out != 32'h50005000 | noc_dest_id != 3'd7
  | noc_data_out != 128'h10000001_20000002_30000003_40000004)begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Error passing controller message to NoC!\nCycles: %d", cycles);
    $stop;
  end
  repeat(1) @(posedge clock);
  if(noc_msg_out != NackC | noc_address_out != 32'h20002000 | ~intf_busy |
  noc_dest_id != 0)begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Error with Nack message!\nCycles: %d", cycles);
    $stop;
  end

  //controller request while responding with a NackB
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h20002000;
  end
  @(posedge clock)begin
    ctrl_msg_in     <= PutM;
    ctrl_address_in <= 32'h50005000;
    ctrl_data_in    <= 128'h10000001_20000002_30000003_40000004;
    noc_msg_in      <= NoMsg;
    noc_address_in  <= 0;
  end
  @(posedge clock)begin
    ctrl_msg_in     <= GetS;
    ctrl_address_in <= 32'h44005500;
    ctrl_data_in    <= 0;
  end
  repeat(1) @(posedge clock);
  //check outputs
  if(noc_msg_out != NackC | noc_address_out != 32'h20002000 | ~intf_busy)begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Error with Nack message!\nCycles: %d", cycles);
    $stop;
  end
  @(posedge clock)begin
    ctrl_msg_in     <= NoMsg;
    ctrl_address_in <= 0;
    ctrl_data_in    <= 0;
  end
  if(noc_msg_out != PutM | noc_address_out != 32'h50005000 |
     noc_data_out != 128'h10000001_20000002_30000003_40000004)begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Error passing controller message to NoC!\nCycles: %d", cycles);
    $stop;
  end
  repeat(1) @(posedge clock);
  if(noc_msg_out != GetS | noc_address_out != 32'h44005500 | noc_data_out != 0)begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Error passing controller message to NoC!\nCycles: %d", cycles);
    $stop;
  end

  //parent module reads from a buffer
  repeat(2) @(posedge clock) respbuf_read <= 1'b1;
  @(posedge clock) respbuf_read <= 1'b0;
  
  //check buffer empty signal
  if(~respbuf_empty)begin
    $display("\ntb_message_handler --> Test Failed!\n\n");
    $display("Buffer empty signal not set!\nCycles: %d", cycles);
    $stop;
  end
  
  //test packetizer busy signal
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    ctrl_msg_in     <= PutS;
    ctrl_address_in <= 32'h00000050;
    ctrl_data_in    <= 0;
    ctrl_dest_id    <= 0;
  end
  @(posedge clock)begin
    ctrl_msg_in     <= PutM;
    ctrl_address_in <= 32'h00000060;
    ctrl_data_in    <= 0;
    ctrl_dest_id    <= 0;
    packetizer_busy <= 1;
  end 
  @(posedge clock)begin
    ctrl_msg_in     <= GetM;
    ctrl_address_in <= 32'h00000060;
    ctrl_data_in    <= 0;
    ctrl_dest_id    <= 0;
  end
  repeat(5) @(posedge clock);
  @(posedge clock)begin
    packetizer_busy <= 1'b0;
  end
  wait(~intf_busy);
  @(posedge clock)begin
    ctrl_msg_in     <= PutS;
    ctrl_address_in <= 32'h00000100;
    ctrl_data_in    <= 0;
    ctrl_dest_id    <= 0;
  end
  @(posedge clock)begin
    ctrl_msg_in     <= NoMsg;
    ctrl_address_in <= 0;
    ctrl_data_in    <= 0;
    ctrl_dest_id    <= 0;
  end

  #10;
  $display("\ntb_message_handler --> Test Passed!\n\n");
  $finish;
end

//timeout
initial begin
  #400;
  $display("\ntb_message_handler --> Test Failed!\n\n");
  $stop;
end


endmodule
