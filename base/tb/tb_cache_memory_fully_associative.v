/** @module : tb_cache_memory_fully_associative
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

module tb_cache_memory_fully_associative();

//define the log2 function
function integer log2;
input integer value;
begin
  value = value-1;
  for (log2=0; value>0; log2=log2+1)
    value = value >> 1;
end
endfunction

parameter STATUS_BITS           =  2,
          COHERENCE_BITS        =  2,
          OFFSET_BITS           =  2,
          DATA_WIDTH            = 32,
          NUMBER_OF_WAYS        =  4,
          REPLACEMENT_MODE      =  0,
          ADDRESS_BITS          = 32,
          INDEX_BITS            =  0;

parameter WORDS_PER_LINE = 1 << OFFSET_BITS;
parameter BLOCK_WIDTH    = DATA_WIDTH*WORDS_PER_LINE;
parameter SBITS          = COHERENCE_BITS + STATUS_BITS;
parameter TAG_BITS       = ADDRESS_BITS - OFFSET_BITS - INDEX_BITS;
parameter MBITS          = SBITS + TAG_BITS;
parameter WAY_BITS       = (NUMBER_OF_WAYS > 1) ? log2(NUMBER_OF_WAYS) : 1;
parameter CACHE_DEPTH    = 1 << INDEX_BITS;

`include `INCLUDE_FILE

reg clock, reset;
reg read0, write0, invalidate0;
reg  [INDEX_BITS-1    :0] index0;
reg  [TAG_BITS-1      :0] tag0;
reg  [SBITS-1         :0] meta_data0;
reg  [BLOCK_WIDTH-1   :0] data_in0;
reg  [WAY_BITS-1      :0] way_select0;
wire [BLOCK_WIDTH-1   :0] data_out0;
wire [TAG_BITS-1      :0] tag_out0;
wire [WAY_BITS-1      :0] matched_way0;
wire [COHERENCE_BITS-1:0] coh_bits0;
wire [STATUS_BITS-1   :0] status_bits0;
wire hit0;

reg read1, write1, invalidate1;
reg  [INDEX_BITS-1    :0] index1;
reg  [TAG_BITS-1      :0] tag1;
reg  [SBITS-1         :0] meta_data1;
reg  [BLOCK_WIDTH-1   :0] data_in1;
reg  [WAY_BITS-1      :0] way_select1;
wire [BLOCK_WIDTH-1   :0] data_out1;
wire [TAG_BITS-1      :0] tag_out1;
wire [WAY_BITS-1      :0] matched_way1;
wire [COHERENCE_BITS-1:0] coh_bits1;
wire [STATUS_BITS-1   :0] status_bits1;
wire hit1;

reg  report;


//instantiate DUT
cache_memory #(
  .STATUS_BITS(STATUS_BITS),
  .COHERENCE_BITS(COHERENCE_BITS),
  .OFFSET_BITS(OFFSET_BITS),
  .DATA_WIDTH(DATA_WIDTH),
  .NUMBER_OF_WAYS(NUMBER_OF_WAYS),
  .REPLACEMENT_MODE(REPLACEMENT_MODE),
  .ADDRESS_BITS(ADDRESS_BITS),
  .INDEX_BITS(INDEX_BITS),
  .READ_DURING_WRITE("OLD_DATA")
) DUT (
clock, reset,
//port 0
read0, write0, invalidate0,
index0,
tag0,
meta_data0,
data_in0,
way_select0,
data_out0,
tag_out0,
matched_way0,
coh_bits0,
status_bits0,
hit0,
//port 1
read1, write1, invalidate1,
index1,
tag1,
meta_data1,
data_in1,
way_select1,
data_out1,
tag_out1,
matched_way1,
coh_bits1,
status_bits1,
hit1,

report
);

//generate clock
always #1 clock = ~clock;

//cycle counter
reg [31:0] cycles;
always @(posedge clock)begin
  cycles <= cycles + 1;
end

// Test patterns
initial begin
  cycles = 0;
  clock = 0;
  reset = 0;
  read0 = 0;
  write0 = 0;
  invalidate0 = 0;
  index0 = 0;
  tag0 = 0;
  meta_data0 = 0;
  data_in0 = 0;
  way_select0 = 0;
  read1 = 0;
  write1 = 0;
  invalidate1 = 0;
  index1 = 0;
  tag1 = 0;
  meta_data1 = 0;
  data_in1 = 0;
  way_select1 = 0;
  report = 0;

  repeat(1) @(posedge clock);
  @(posedge clock) begin
    reset <= 1;
    $display("%0d> Start reset sequence.", cycles);
  end
  //repeat(CACHE_DEPTH)begin
  //  @(posedge clock) index0 <= index0 + 1;
  //end
  @(posedge clock) begin
    reset <= 0;
    index0 <= 0;
    $display("%0d> Deassert reset.", cycles);
  end

  repeat(1) @(posedge clock);
  //initialize content
  DUT.DATA[0].data_bram.RAM.ram[0] = 128'h11111111_22222222_33333333_44444444;
  DUT.DATA[1].data_bram.RAM.ram[0] = 128'h11115555_22226666_33337777_44448888;
  DUT.DATA[2].data_bram.RAM.ram[0] = 128'h11555555_22666666_33777777_44888888;
  DUT.DATA[3].data_bram.RAM.ram[0] = 128'h15555555_26666666_37777777_48888888;
  DUT.MDATA[0].mdata_bram.RAM.ram[0] = {2'b10, SHARED, 30'h3AAAAA};
  DUT.MDATA[1].mdata_bram.RAM.ram[0] = {2'b10, SHARED, 30'h2BBBBB};
  DUT.MDATA[2].mdata_bram.RAM.ram[0] = {2'b10, EXCLUSIVE, 30'h1CCCCC};
  DUT.MDATA[3].mdata_bram.RAM.ram[0] = {2'b10, MODIFIED, 30'h0EEEEE};
  //initialize LRU state
  DUT.REPLACE.replace_inst.lru_inst.lru_bram.RAM.ram[0] = 8'b10_00_11_01;

  //$stop;

  $display("Initial memory contents");
  $display("Way0: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[0].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[0].mdata_bram.RAM.ram[0][29:0], DUT.DATA[0].data_bram.RAM.ram[0]);
  $display("Way1: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[1].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[1].mdata_bram.RAM.ram[0][29:0], DUT.DATA[1].data_bram.RAM.ram[0]);
  $display("Way2: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[2].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[2].mdata_bram.RAM.ram[0][29:0], DUT.DATA[2].data_bram.RAM.ram[0]);
  $display("Way3: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[3].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[3].mdata_bram.RAM.ram[0][29:0], DUT.DATA[3].data_bram.RAM.ram[0]);

  //write request
  repeat(2) @(posedge clock);
  @(posedge clock)begin
    write0 <= 1;
    index0 <= 0;
    tag0 <= 30'h1CCCCC;
    meta_data0 <= {2'b10, MODIFIED};
    data_in0 <= 128'h11555555_99999999_33777777_00001000;
    way_select0 <= 2;
    $display("%0d> Write request.", cycles);
  end
  @(posedge clock)begin
    write0 <= 0;
    index0 <= 0;
    tag0 <= 0;
    meta_data0 <= 0;
    data_in0 <= 0;
    way_select0 <= 0;
  end

  //read request
  @(posedge clock)begin
	$display("%0d> Memory contents after the write.", cycles);
    $display("Way0: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[0].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[0].mdata_bram.RAM.ram[0][29:0], DUT.DATA[0].data_bram.RAM.ram[0]);
    $display("Way1: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[1].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[1].mdata_bram.RAM.ram[0][29:0], DUT.DATA[1].data_bram.RAM.ram[0]);
    $display("Way2: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[2].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[2].mdata_bram.RAM.ram[0][29:0], DUT.DATA[2].data_bram.RAM.ram[0]);
    $display("Way3: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[3].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[3].mdata_bram.RAM.ram[0][29:0], DUT.DATA[3].data_bram.RAM.ram[0]);

    if(DUT.DATA[2].data_bram.RAM.ram[0] != 128'h11555555999999993377777700001000
    | DUT.MDATA[2].mdata_bram.RAM.ram[1][33:30] != 4'b1010)begin
      $display("\ntb_cache_memory_fully_associative --> Test Failed!\n\n");
      $stop;
    end
    read1 <= 1;
    index1 <= 0;
    tag1 <= 30'h2BBBBB; //hit does not change LRU state
	  $display("%0d> Read request on port1. Index1=0, tag=30'h2BBBBB", cycles);
  end
  @(posedge clock)begin
    read1 <= 0;
    index1 <= 0;
    tag1 <= 0;
    read0 <= 1;
    index0 <= 0;
    tag0 <= 22'h2BBBBB; //hit. changes LRU state
	  $display("%0d> Read request on port0. Index=0, tag=30'h2BBBBB", cycles);
  end
  @(data_out1) 
    $display("%0d> Data returned on port1=%h", cycles-1, data_out1);
    $display("%0d> Data returned on port0=%h", cycles-1, data_out0);

  @(posedge clock)begin
    tag0 <= 30'h2BB123; //miss. should return 3 as matched way (replace way).
	  $display("%0d> Read request on port0. Index=0, tag=22'h2BB123", cycles);
  end
  @(data_out0) $display("%0d> Data returned on port0=%h, hit0:%b, matched way:%h", 
    cycles-1, data_out0, hit0, matched_way0);
  //invalidate request
  @(posedge clock)begin
    read0 <= 0;
    index0 <= 0;
    tag0 <= 0;
    invalidate1 <= 1;
    index1 <= 0;
    way_select1 <= 1;
	  $display("%0d> Invalidate request on port1. Index=0, way_select1=1", cycles);
  end
  @(posedge clock)begin
    invalidate1 <= 0;
    index1 <= 0;
    way_select1 <= 0;
  end

  //port 0 reading the invalidated line.
  @(posedge clock)begin
	$display("%0d> Memory contents after the invalidation.", cycles);
    $display("Way0: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[0].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[0].mdata_bram.RAM.ram[0][29:0], DUT.DATA[0].data_bram.RAM.ram[0]);
    $display("Way1: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[1].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[1].mdata_bram.RAM.ram[0][29:0], DUT.DATA[1].data_bram.RAM.ram[0]);
    $display("Way2: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[2].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[2].mdata_bram.RAM.ram[0][29:0], DUT.DATA[2].data_bram.RAM.ram[0]);
    $display("Way3: Meta data=%b | tag=%h | data=%h", 
      DUT.MDATA[3].mdata_bram.RAM.ram[0][33:30],
      DUT.MDATA[3].mdata_bram.RAM.ram[0][29:0], DUT.DATA[3].data_bram.RAM.ram[0]);

    if(DUT.MDATA[1].mdata_bram.RAM.ram[0][33:30] != 4'b0000 |
    DUT.MDATA[1].mdata_bram.RAM.ram[1][29:0] != 30'd0)begin
      $display("\ntb_cache_memory_fully_associative --> Test Failed!\n\n");
      $stop;
    end
    read0 <= 1;
    index0 <= 0;
    tag0 <= 30'h2BBBBB;
	$display("%0d> Read request on port0. Index=0, tag=30'h2BBBBB", cycles);
  end
  @(posedge clock)begin
    read0 <= 0;
    index0 <= 0;
    tag0 <= 0;
  end

  repeat(1) @(posedge clock);
  @(posedge clock)begin
    $display("%0d> Both ports writing to the same address.", cycles);
	  $display("Port0 data=128'h00000000_00000001_00000002_00000003");
	  $display("Port1 data=128'h11111110_11111111_11111112_11111113");
	  write0 <= 1;
	  write1 <= 1;
	  index0 <= 0;
	  index1 <= 0;
    tag0 <= 30'h1CCEEE;
	  tag1 <= 30'h1CCFFF;
    meta_data0 <= {2'b10, MODIFIED};
	  meta_data1 <= {2'b10, MODIFIED};
    data_in0 <= 128'h00000000_00000001_00000002_00000003;
	  data_in1 <= 128'h11111110_11111111_11111112_11111113;
    way_select0 <= 1;
	  way_select1 <= 1;
  end
  @(posedge clock)begin
	  write0 <= 0;
	  write1 <= 0;
	  index0 <= 0;
	  index1 <= 0;
    tag0 <= 0;
	  tag1 <= 0;
    meta_data0 <= 0;
	  meta_data1 <= 0;
    data_in0 <= 0;
	  data_in1 <= 0;
    way_select0 <= 0;
	  way_select1 <= 0;  
  end
  
  repeat(1) @(posedge clock);
  $display("%0d> Memory contents after the write.", cycles);
  $display("Way0: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[0].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[0].mdata_bram.RAM.ram[0][29:0], DUT.DATA[0].data_bram.RAM.ram[0]);
  $display("Way1: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[1].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[1].mdata_bram.RAM.ram[0][29:0], DUT.DATA[1].data_bram.RAM.ram[0]);
  $display("Way2: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[2].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[2].mdata_bram.RAM.ram[0][29:0], DUT.DATA[2].data_bram.RAM.ram[0]);
  $display("Way3: Meta data=%b | tag=%h | data=%h", 
    DUT.MDATA[3].mdata_bram.RAM.ram[0][33:30],
    DUT.MDATA[3].mdata_bram.RAM.ram[0][29:0], DUT.DATA[3].data_bram.RAM.ram[0]);  
  
  if(DUT.MDATA[0].mdata_bram.RAM.ram[0][33:30] != 4'b1011 | 
     DUT.MDATA[1].mdata_bram.RAM.ram[0][33:30] != 4'b1010 |
     DUT.MDATA[2].mdata_bram.RAM.ram[0][33:30] != 4'b1010 |
     DUT.MDATA[3].mdata_bram.RAM.ram[0][33:30] != 4'b1010 |
     DUT.MDATA[0].mdata_bram.RAM.ram[0][29:0] != 30'h3aaaaa |
     DUT.MDATA[1].mdata_bram.RAM.ram[0][29:0] != 30'h1ccfff |
     DUT.MDATA[2].mdata_bram.RAM.ram[0][29:0] != 30'h1ccccc |
     DUT.MDATA[3].mdata_bram.RAM.ram[0][29:0] != 30'h0eeeee |
     DUT.DATA[0].data_bram.RAM.ram[0] != 128'h11111111222222223333333344444444 |
     DUT.DATA[1].data_bram.RAM.ram[0] != 128'h11111110111111111111111211111113 |
     DUT.DATA[2].data_bram.RAM.ram[0] != 128'h11555555999999993377777700001000 |
     DUT.DATA[3].data_bram.RAM.ram[0] != 128'h15555555266666663777777748888888 )
  begin
    $display("\ntb_cache_memory_fully_associative --> Test Failed!\n\n");
    $stop;
  end

  #10;
  $display("\ntb_cache_memory_fully_associative --> Test Passed!\n\n");
  $finish;
end

//timeout
initial begin
  #1000;
  $display("\ntb_cache_memory_fully_associative --> Test Failed!\n\n");
  $stop;
end


endmodule
