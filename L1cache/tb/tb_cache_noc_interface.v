/** @module : tb_cache_noc_interface
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
* Module: tb_cache_noc_interface
*******************************************************************************/

module tb_cache_noc_interface();

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

parameter STATUS_BITS         =  2,
          COHERENCE_BITS      =  2,
          CACHE_OFFSET_BITS   =  2,
          DATA_WIDTH          = 32,
          NUMBER_OF_WAYS      =  4,
          ADDRESS_BITS        = 32,
          INDEX_BITS          =  8,
          MSG_BITS            =  4,
		      MAX_OFFSET_BITS     =  3,
          REQ_BUF_DEPTH_BITS  =  2,
          RESP_BUF_DEPTH_BITS =  2,
          CORE                =  0,
          CACHE_NO            =  0,
          CONTROLLER_TYPE     = "BLOCKING",
          ID_BITS             =  2,
          DEFAULT_DEST        =  0,
          CACHE_WORDS         = 1 << CACHE_OFFSET_BITS,
          CACHE_WIDTH         = DATA_WIDTH * CACHE_WORDS,
          WAY_BITS            = (NUMBER_OF_WAYS > 1) ? log2(NUMBER_OF_WAYS) : 1,
          TAG_BITS            = ADDRESS_BITS - INDEX_BITS - CACHE_OFFSET_BITS,
          SBITS               = COHERENCE_BITS + STATUS_BITS;


/*DUT signals*/
reg clock, reset;
//interface with the NoC
reg  [MSG_BITS-1    :0] noc_msg_in;
reg  [ADDRESS_BITS-1:0] noc_address_in;
reg  [CACHE_WIDTH-1 :0] noc_data_in;
reg  [ID_BITS-1     :0] noc_src_id;
reg  packetizer_busy;
wire [MSG_BITS-1    :0] noc_msg_out;
wire [ADDRESS_BITS-1:0] noc_address_out;
wire [CACHE_WIDTH-1 :0] noc_data_out;
wire [ID_BITS-1     :0] noc_dest_id;
//interface with cache controller
reg  [MSG_BITS-1    :0] cache_msg_in;
reg  [ADDRESS_BITS-1:0] cache_address_in;
reg  [CACHE_WIDTH-1 :0] cache_data_in;
wire [MSG_BITS-1    :0] cache_msg_out;
wire [ADDRESS_BITS-1:0] cache_address_out;
wire [CACHE_WIDTH-1 :0] cache_data_out;
wire busy;
//interface with cache memory
reg  [CACHE_WIDTH-1   :0] port1_read_data;
reg  [WAY_BITS-1      :0] port1_matched_way;
reg  [COHERENCE_BITS-1:0] port1_coh_bits;
reg  [STATUS_BITS-1   :0] port1_status_bits;
reg  port1_hit;
wire port1_read, port1_write, port1_invalidate;
wire [INDEX_BITS-1 :0] port1_index;
wire [TAG_BITS-1   :0] port1_tag;
wire [SBITS-1      :0] port1_metadata;
wire [CACHE_WIDTH-1:0] port1_write_data;
wire [WAY_BITS-1   :0] port1_way_select;


/*instantiate DUT*/
cache_noc_interface #(
  .STATUS_BITS(STATUS_BITS),
  .COHERENCE_BITS(COHERENCE_BITS),
  .CACHE_OFFSET_BITS(CACHE_OFFSET_BITS),
  .DATA_WIDTH(DATA_WIDTH),
  .NUMBER_OF_WAYS(NUMBER_OF_WAYS),
  .ADDRESS_BITS(ADDRESS_BITS),
  .INDEX_BITS(INDEX_BITS),
  .MSG_BITS(MSG_BITS),
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
  .noc_msg_in(noc_msg_in),
  .noc_address_in(noc_address_in),
  .noc_data_in(noc_data_in),
  .noc_src_id(noc_src_id),
  .packetizer_busy(packetizer_busy),
  .noc_msg_out(noc_msg_out),
  .noc_address_out(noc_address_out),
  .noc_data_out(noc_data_out),
  .noc_dest_id(noc_dest_id),
  .cache_msg_in(cache_msg_in),
  .cache_address_in(cache_address_in),
  .cache_data_in(cache_data_in),
  .cache_msg_out(cache_msg_out),
  .cache_address_out(cache_address_out),
  .cache_data_out(cache_data_out),
  .busy(busy),
  .port1_read_data(port1_read_data),
  .port1_matched_way(port1_matched_way),
  .port1_coh_bits(port1_coh_bits),
  .port1_status_bits(port1_status_bits),
  .port1_hit(port1_hit),
  .port1_read(port1_read),
  .port1_write(port1_write),
  .port1_invalidate(port1_invalidate),
  .port1_index(port1_index),
  .port1_tag(port1_tag),
  .port1_metadata(port1_metadata),
  .port1_write_data(port1_write_data),
  .port1_way_select(port1_way_select)
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
  clock             = 0;
  reset             = 0;
  cycles            = 0;
  noc_msg_in        = NoMsg;
  noc_address_in    = 0;
  noc_data_in       = 0;
  noc_src_id        = 0;
  packetizer_busy   = 1'b0;
  cache_msg_in      = NoMsg; //NoMsg and NO_REQ have same bit encoding.
  cache_address_in  = 0;
  cache_data_in     = 0;
  port1_read_data   = 0;
  port1_matched_way = 0;
  port1_coh_bits    = 0;
  port1_status_bits = 0;
  port1_hit         = 1'b0;



  //reset the module
  @(posedge clock)begin
    reset <= 1;
  end
  @(posedge clock)begin
    reset <= 0;
  end

  //read request from cache controller
  @(posedge clock)begin
    cache_msg_in     <= R_REQ; //same encoding as GetS
    cache_address_in <= 32'h00001000;
  end

  //directory responds
  wait(noc_msg_out == GetS & noc_address_out == 32'h00001000 & noc_dest_id == 0);
  repeat(3) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= DataS;
    noc_address_in <= 32'h00001000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0;
    noc_data_in    <= 0;
  end
  
  wait(cache_msg_out == MEM_RESP_S);
  @(posedge clock)begin
    cache_msg_in     <= NO_REQ;
    cache_address_in <= 0;
  end
  
  // Fill up the buffers while
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h00002000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h00003000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= FwdGetS;
    noc_address_in <= 32'h00004000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= DataS;
    noc_address_in <= 32'h00005000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= DataS;
    noc_address_in <= 32'h00006000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= DataS;
    noc_address_in <= 32'h00007000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h00008000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h00009000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= Inv;
    noc_address_in <= 32'h00010000;
    noc_data_in    <= 128'h00000001_00000002_00000003_00000004;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0;
    noc_data_in    <= 0;
  end
  
  repeat(10) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= FwdGetS;
    noc_address_in <= 32'h00010000;
  end
  @(posedge clock)begin
    noc_msg_in     <= NoMsg;
    noc_address_in <= 0;
    noc_data_in    <= 0;
  end
  repeat(8) @(posedge clock);
  @(posedge clock)begin
    packetizer_busy <= 1'b1;
  end
  
  repeat(3) @(posedge clock);
  @(posedge clock)begin
    cache_msg_in     <= WB_REQ;
    cache_address_in <= 32'h00020000;
    cache_data_in    <= 128'h50000001_50000002_50000003_50000004;
  end

  repeat(2) @(posedge clock);
  @(posedge clock)begin
    packetizer_busy <= 1'b0;
  end
  @(posedge clock)begin
    packetizer_busy <= 1'b1;
  end
  repeat(3) @(posedge clock);
  @(posedge clock)begin
    packetizer_busy <= 1'b0;
  end



end


//cache memory responses through port1
initial begin
  wait(port1_read & port1_index == 0 & port1_tag == 22'h000008);
  @(posedge clock)begin
    port1_read_data   <= 128'h20000001_20000002_20000003_20000004;
    port1_matched_way <= 3;
    port1_coh_bits    <= MODIFIED;
    port1_status_bits <= 2'b11;
    port1_hit         <= 1'b1;
  end  
  @(posedge clock)begin
    port1_read_data   <= 0;
    port1_matched_way <= 0;
    port1_coh_bits    <= INVALID;
    port1_status_bits <= 0;
    port1_hit         <= 0;
  end  
  //invalidate cache line
  @(posedge clock)begin
    if(port1_invalidate != 1'b1 | port1_index != 0 | port1_tag != 22'h000008 |
    port1_way_select != 3)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end
  //response to directoroy
  @(posedge clock)begin
    if(noc_msg_out != RespPutM | noc_address_out != 32'h00002000 | noc_dest_id != 0 |
    noc_data_out != 128'h20000001_20000002_20000003_20000004)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end
  wait(port1_read & port1_index == 0 & port1_tag == 22'h00000C);
  @(posedge clock)begin
    port1_read_data   <= 128'h30000001_30000002_30000003_30000004;
    port1_matched_way <= 2;
    port1_coh_bits    <= SHARED;
    port1_status_bits <= 2'b10;
    port1_hit         <= 1'b1;
  end  
  @(posedge clock)begin
    port1_read_data   <= 0;
    port1_matched_way <= 0;
    port1_coh_bits    <= INVALID;
    port1_status_bits <= 0;
    port1_hit         <= 0;
  end  
  //invalidate cache line
  @(posedge clock)begin
    if(port1_invalidate != 1'b1 | port1_index != 0 | port1_tag != 22'h00000C |
    port1_way_select != 2)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end
  //response to directoroy
  @(posedge clock)begin
    if(noc_msg_out != InvAck | noc_address_out != 32'h00003000 | noc_dest_id != 0 |
    noc_data_out != 0)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end

  wait(port1_read & port1_index == 0 & port1_tag == 22'h000010);
  @(posedge clock)begin
    port1_read_data   <= 128'h40000001_40000002_40000003_40000004;
    port1_matched_way <= 0;
    port1_coh_bits    <= EXCLUSIVE;
    port1_status_bits <= 2'b10;
    port1_hit         <= 1'b1;
  end  
  @(posedge clock)begin
    port1_read_data   <= 0;
    port1_matched_way <= 0;
    port1_coh_bits    <= INVALID;
    port1_status_bits <= 0;
    port1_hit         <= 0;
  end  

  //change cache line state
  @(posedge clock)begin
    if(port1_write != 1'b1 | port1_index != 0 | port1_tag != 22'h000010 |
    port1_way_select != 0 | port1_metadata != 4'b1011)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end
  //response to directoroy
  @(posedge clock)begin
    if(noc_msg_out != PutE | noc_address_out != 32'h00004000 | noc_dest_id != 0 |
    noc_data_out != 0)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end

  wait(port1_read & port1_index == 0 & port1_tag == 22'h000020);
  @(posedge clock)begin
    port1_read_data   <= 128'h80000001_80000002_80000003_80000004;
    port1_matched_way <= 0;
    port1_coh_bits    <= MODIFIED;
    port1_status_bits <= 2'b11;
    port1_hit         <= 1'b1;
  end  
  @(posedge clock)begin
    port1_read_data   <= 0;
    port1_matched_way <= 0;
    port1_coh_bits    <= INVALID;
    port1_status_bits <= 0;
    port1_hit         <= 0;
  end  
  //invalidate cache line
  @(posedge clock)begin
    if(port1_invalidate != 1'b1 | port1_index != 0 | port1_tag != 22'h000020 |
    port1_way_select != 0)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end
  //response to directoroy
  @(posedge clock)begin
    if(noc_msg_out != RespPutM | noc_address_out != 32'h00008000 | noc_dest_id != 0 |
    noc_data_out != 128'h80000001_80000002_80000003_80000004)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end

  wait(port1_read & port1_index == 0 & port1_tag == 22'h000024);
  @(posedge clock)begin
    port1_read_data   <= 128'h80000001_80000002_80000003_80000004;
    port1_matched_way <= 0;
    port1_coh_bits    <= SHARED;
    port1_status_bits <= 2'b10;
    port1_hit         <= 1'b0;
  end  
  @(posedge clock)begin
    port1_read_data   <= 0;
    port1_matched_way <= 0;
    port1_coh_bits    <= INVALID;
    port1_status_bits <= 0;
    port1_hit         <= 0;
  end  
  repeat(1) @(posedge clock);
  //response to directoroy
  @(posedge clock)begin
    if(noc_msg_out != InvAck | noc_address_out != 32'h00009000 | noc_dest_id != 0 |
    noc_data_out != 0)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end

  wait(port1_read & port1_index == 0 & port1_tag == 22'h000040);
  @(posedge clock)begin
    port1_read_data   <= 128'h40000001_40000002_40000003_40000004;
    port1_matched_way <= 0;
    port1_coh_bits    <= EXCLUSIVE;
    port1_status_bits <= 2'b10;
    port1_hit         <= 1'b1;
  end  
  @(posedge clock)begin
    port1_read_data   <= 0;
    port1_matched_way <= 0;
    port1_coh_bits    <= INVALID;
    port1_status_bits <= 0;
    port1_hit         <= 0;
  end  

  //change cache line state
  @(posedge clock)begin
    if(port1_write != 1'b1 | port1_index != 0 | port1_tag != 22'h000040 |
    port1_way_select != 0 | port1_metadata != 4'b1011)begin
      $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
      $stop;
    end 
  end

  //wait for the PutE message
  wait(noc_msg_out == PutE & noc_address_out == 32'h00010000);

  //wait for the GetM message
  wait(noc_msg_out == PutM & noc_address_out == 32'h00020000 &
  noc_data_out == 128'h50000001_50000002_50000003_50000004);

  repeat(2) @(posedge clock);
  @(posedge clock)begin
    noc_msg_in     <= PutAck;
    noc_address_in <= 32'h00020000;
  end
  
  wait(cache_msg_out == MEM_RESP & cache_address_out == 32'h00020000);

  #20;
  $display("\ntb_cache_noc_interface --> Test Passed!\n\n");
  $stop;  

end

//timeout
initial begin
  #1000;
  $display("\ntb_L1_bus_interface --> Test Failed!\n\n");
  $stop;
end

endmodule
