/** @module : tb_two_level_hierarchy_noc_wrapper
 *  @author : Adaptive & Secure Computing Systems (ASCS) Laboratory
 *  @author : Sahan Bandara

 *  Copyright (c) 2020 BRISC-V (ASCS/ECE/BU)
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

module tb_two_level_hierarchy_noc_wrapper();

//Define the log2 function
function integer log2;
input integer value;
begin
  value = value-1;
  for(log2=0; value>0; log2=log2+1)
    value = value>>1;
  end
endfunction

parameter STATUS_BITS_L1      = 2,
          OFFSET_BITS_L1      = {32'd2, 32'd2, 32'd2, 32'd2},
          NUMBER_OF_WAYS_L1   = {32'd2, 32'd2, 32'd2, 32'd2},
          INDEX_BITS_L1       = {32'd6, 32'd6, 32'd6, 32'd6},
          REPLACEMENT_MODE_L1 = 1'b0,
          STATUS_BITS_L2      = 3,
          OFFSET_BITS_L2      = 2,
          NUMBER_OF_WAYS_L2   = 4,
          INDEX_BITS_L2       = 6,
          REPLACEMENT_MODE_L2 = 1'b0,
          L2_INCLUSION        = 1'b1,
          COHERENCE_BITS      = 2,
          DATA_WIDTH          = 32,
          ADDRESS_BITS        = 32,
          MSG_BITS            = 4,
          NUM_L1_CACHES       = 4,
          BUS_OFFSET_BITS     = 2,
          MAX_OFFSET_BITS     = 2,
          REQ_BUF_DEPTH_BITS  = 2,
          RESP_BUF_DEPTH_BITS = 2,
          CORE                = 0,
          CACHE_NO            = 0,
          CONTROLLER_TYPE     = "BLOCKING",
          ID_BITS             = 2,
          DEFAULT_DEST        = 0,
          //Use default value in module instantiation for following parameters
          L2_WORDS            = 1 << OFFSET_BITS_L2,
          L2_WIDTH            = L2_WORDS*DATA_WIDTH,
          L2_TAG_BITS         = ADDRESS_BITS - OFFSET_BITS_L2 - INDEX_BITS_L2,
          L2_WAY_BITS         = (NUMBER_OF_WAYS_L2 > 1) ? log2(NUMBER_OF_WAYS_L2) : 1,
          L2_MBITS            = COHERENCE_BITS + STATUS_BITS_L2;

localparam BUS_WORDS     = 1 << BUS_OFFSET_BITS;
localparam BUS_WIDTH     = BUS_WORDS*DATA_WIDTH;
localparam BUS_PORTS     = NUM_L1_CACHES + 1;
localparam MEM_PORT      = BUS_PORTS - 1;
localparam BUS_SIG_WIDTH = log2(BUS_PORTS);
localparam WIDTH_BITS    = log2(MAX_OFFSET_BITS) + 1;

// Define INCLUDE_FILE  to point to /includes/params.h. The path should be
// relative to your simulation/sysnthesis directory. You can add the macro
// when compiling this file in modelsim by adding the following argument to the
// vlog command that compiles this module:
// +define+INCLUDE_FILE="../../../includes/params.h"
`include `INCLUDE_FILE


reg  clock;
reg  reset;
//interface with processor pipelines
reg  [NUM_L1_CACHES-1:0] read, write, invalidate, flush;
reg  [NUM_L1_CACHES*DATA_WIDTH/8-1:0] w_byte_en;
reg  [ADDRESS_BITS-1:0] address_s [NUM_L1_CACHES-1:0];
wire [NUM_L1_CACHES*ADDRESS_BITS-1:0] address;
reg  [DATA_WIDTH-1  :0] data_in_s [NUM_L1_CACHES-1:0];
wire [NUM_L1_CACHES*DATA_WIDTH-1  :0] data_in;
wire [ADDRESS_BITS-1:0] out_address_s [NUM_L1_CACHES-1:0];
wire [NUM_L1_CACHES*ADDRESS_BITS-1:0] out_address;
wire [DATA_WIDTH-1  :0] data_out_s [NUM_L1_CACHES-1:0];
wire [NUM_L1_CACHES*DATA_WIDTH-1  :0] data_out;
wire [NUM_L1_CACHES-1:0] valid, ready;
//memory side interface
reg  [MSG_BITS-1    :0] noc_msg_in;
reg  [ADDRESS_BITS-1:0] noc_address_in;
reg  [L2_WIDTH-1    :0] noc_data_in;
reg  [ID_BITS-1     :0] noc_src_id;
reg  packetizer_busy;
wire [MSG_BITS-1    :0] noc_msg_out;
wire [ADDRESS_BITS-1:0] noc_address_out;
wire [L2_WIDTH-1    :0] noc_data_out;
wire [ID_BITS-1     :0] noc_dest_id;
wire interface_busy;

reg  scan;

//combine and split signals
genvar j;
generate
  for(j=0; j<NUM_L1_CACHES; j=j+1)begin
    assign address[j*ADDRESS_BITS +: ADDRESS_BITS] = address_s[j];
    assign data_in[j*DATA_WIDTH   +: DATA_WIDTH  ] = data_in_s[j];
  end
  for(j=0; j<NUM_L1_CACHES; j=j+1)begin
    assign data_out_s[j]    = data_out   [j*DATA_WIDTH   +: DATA_WIDTH  ];
    assign out_address_s[j] = out_address[j*ADDRESS_BITS +: ADDRESS_BITS];
  end
endgenerate

//Instantiate DUT
two_level_hierarchy_noc_wrapper #(
  .STATUS_BITS_L1(STATUS_BITS_L1),
  .OFFSET_BITS_L1(OFFSET_BITS_L1),
  .NUMBER_OF_WAYS_L1(NUMBER_OF_WAYS_L1),
  .INDEX_BITS_L1(INDEX_BITS_L1),
  .REPLACEMENT_MODE_L1(REPLACEMENT_MODE_L1),
  .STATUS_BITS_L2(STATUS_BITS_L2),
  .OFFSET_BITS_L2(OFFSET_BITS_L2),
  .NUMBER_OF_WAYS_L2(NUMBER_OF_WAYS_L2),
  .INDEX_BITS_L2(INDEX_BITS_L2),
  .REPLACEMENT_MODE_L2(REPLACEMENT_MODE_L2),
  .L2_INCLUSION(L2_INCLUSION),
  .COHERENCE_BITS(COHERENCE_BITS),
  .DATA_WIDTH(DATA_WIDTH),
  .ADDRESS_BITS(ADDRESS_BITS),
  .MSG_BITS(MSG_BITS),
  .NUM_L1_CACHES(NUM_L1_CACHES),
  .BUS_OFFSET_BITS(BUS_OFFSET_BITS),
  .MAX_OFFSET_BITS(MAX_OFFSET_BITS),
  .REQ_BUF_DEPTH_BITS(REQ_BUF_DEPTH_BITS),
  .RESP_BUF_DEPTH_BITS(RESP_BUF_DEPTH_BITS),
  .CORE(CORE),
  .CACHE_NO(CACHE_NO),
  .CONTROLLER_TYPE(CONTROLLER_TYPE),
  .ID_BITS(ID_BITS),
  .DEFAULT_DEST(DEFAULT_DEST)
) DUT (
  .clock(clock),
  .reset(reset),
  //interface with processor pipelines
  .read(read),
  .write(write),
  .invalidate(invalidate),
  .flush(flush),
  .w_byte_en(w_byte_en),
  .address(address),
  .data_in(data_in),
  .out_address(out_address),
  .data_out(data_out),
  .valid(valid),
  .ready(ready),
  //memory side interface
  .noc_msg_in(noc_msg_in),
  .noc_address_in(noc_address_in),
  .noc_data_in(noc_data_in),
  .noc_src_id(noc_src_id),
  .packetizer_busy(packetizer_busy),
  .noc_msg_out(noc_msg_out),
  .noc_address_out(noc_address_out),
  .noc_data_out(noc_data_out),
  .noc_dest_id(noc_dest_id),
  .interface_busy(interface_busy),
  
  .scan(scan)
);

integer i;

//clock signal
always #1 clock = ~clock;

//cycle counter
reg [31:0] cycles;
always @(posedge clock)begin
  cycles <= cycles + 1;
end


initial begin
  clock     = 1'b0;
  reset     = 1'b0;
  cycles    = 0;
  w_byte_en = {NUM_L1_CACHES*(DATA_WIDTH/8){1'b1}};
  for(i=0; i<NUM_L1_CACHES; i=i+1)begin
    address_s[i]  = 32'd0;
    data_in_s[i]  = 32'd0;
    read[i]       = 1'b0;
    write[i]      = 1'b0;
    flush[i]      = 1'b0;
    invalidate[i] = 1'b0;
  end
  noc_msg_in      = NoMsg;
  noc_address_in  = 32'd0;
  noc_data_in     = 128'd0;
  noc_src_id      = 0;
  packetizer_busy = 1'b0;

  scan = 1'b0;

  //reset
  repeat(1) @(posedge clock);
  @(posedge clock) reset <= 1;
  repeat(3) @(posedge clock);
  @(posedge clock) reset <= 0;

  //wait for L1 caches to finish reset sequence
  wait(DUT.cache_hier.L1INST[0].l1cache.cache.controller.state == 4'd0);
  $display("%0d> L1 caches finished reset sequence", cycles-1);

  //write request to L1cache 0
  @(posedge clock)begin
    write[0]     <= 1'b1;
    address_s[0] <= 32'hffffff0c;
    data_in_s[0] <= 32'h11112222;
  end
  @(posedge clock)begin
    write[0]     <= 1'b0;
    address_s[0] <= 32'h0;
    data_in_s[0] <= 32'h0;
  end

  wait(noc_msg_out == GetM & noc_address_out == 32'h3fffffc0);

  //L1cache 3 issuing a read request to the same address
  @(posedge clock)begin
    read[3]      <= 1'b1;
    address_s[3] <= 32'hffffff0c;
  end
  @(posedge clock)begin
    read[3]      <= 1'b0;
    address_s[3] <= 32'h0;
  end

  //memory respondong to the cache hierarchy
  repeat(3) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= Data;
    noc_address_in <= 32'h3fffffc0;
    noc_data_in    <= 128'h00040004_00030003_00020002_00010001;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 32'h0;
    noc_data_in    <= 128'h0;
  end

  //cache 0 completes the write request
  wait(ready[0]);

  //cache 3 returns requested data.
  wait(valid[3] & data_out_s[3] == 32'h11112222);

  //Flush request from memory side
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h3fffffc0;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <=32'h0;
  end

  //requested line is written back by the cache hierarchy
  wait(noc_msg_out == RespPutM & noc_address_out == 32'h3fffffc0 &
  noc_data_out == 128'h11112222_00030003_00020002_00010001);

  //Test passed
  #20;
  $display("\ntb_two_level_hierarchy_noc_wrapper --> Test Passed!\n\n");
  $stop;
end

//timeout
initial begin
  #500;
  $display("\ntb_two_level_hierarchy_noc_wrapper --> Test Failed!\n\n");
  $stop;
end

endmodule
