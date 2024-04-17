/** @module : two_level_hierarchy_noc_wrapper
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

/** Module description
* --------------------
 *  - Wrapper module for the two level cache hierarchy use when connecting the
 *    two level hierarchy to the NoC.
 *  - Instantiates the two_level_cache_hierarchy module and connects the
 *    memory side interface to a lx_noc_interface module.
 *
 *  Sub modules
 *  -----------
   *  two_level_cache_hierarchy
   *  lx_noc_interface
**/



module two_level_hierarchy_noc_wrapper #(
parameter STATUS_BITS_L1      = 2,
          OFFSET_BITS_L1      = {32'd2, 32'd2, 32'd2, 32'd2},
          NUMBER_OF_WAYS_L1   = {32'd2, 32'd2, 32'd2, 32'd2},
          INDEX_BITS_L1       = {32'd5, 32'd5, 32'd5, 32'd5},
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
          L2_WIDTH            = L2_WORDS*DATA_WIDTH
		  
)(
input  clock,
input  reset,
//processor side interface
input  [NUM_L1_CACHES-1:0] read, write, invalidate, flush,
input  [NUM_L1_CACHES*DATA_WIDTH/8-1:0] w_byte_en,
input  [NUM_L1_CACHES*ADDRESS_BITS-1:0] address,
input  [NUM_L1_CACHES*DATA_WIDTH-1  :0] data_in,
output [NUM_L1_CACHES*ADDRESS_BITS-1:0] out_address,
output [NUM_L1_CACHES*DATA_WIDTH-1  :0] data_out,
output [NUM_L1_CACHES-1:0] valid, ready,
//memory-side interface
input  [MSG_BITS-1    :0] noc_msg_in,
input  [ADDRESS_BITS-1:0] noc_address_in,
input  [L2_WIDTH-1    :0] noc_data_in,
input  [ID_BITS-1     :0] noc_src_id,
input  packetizer_busy,
output [MSG_BITS-1    :0] noc_msg_out,
output [ADDRESS_BITS-1:0] noc_address_out,
output [L2_WIDTH-1    :0] noc_data_out,
output [ID_BITS-1     :0] noc_dest_id,
output interface_busy,

input scan
);

//Define the log2 function
function integer log2;
input integer value;
begin
  value = value-1;
  for(log2=0; value>0; log2=log2+1)
    value = value>>1;
  end
endfunction

// Define INCLUDE_FILE  to point to /includes/params.h. The path should be
// relative to your simulation/sysnthesis directory. You can add the macro
// when compiling this file in modelsim by adding the following argument to the
// vlog command that compiles this module:
// +define+INCLUDE_FILE="../../../includes/params.h"
`include `INCLUDE_FILE

//local parameters
localparam L2_TAG_BITS = ADDRESS_BITS - OFFSET_BITS_L2 - INDEX_BITS_L2;
localparam L2_WAY_BITS = (NUMBER_OF_WAYS_L2 > 1) ? log2(NUMBER_OF_WAYS_L2) : 1;
localparam L2_MBITS    = COHERENCE_BITS + STATUS_BITS_L2;

//internal signals
genvar i;

wire [MSG_BITS-1    :0]     intf2cachehier_msg;
wire [ADDRESS_BITS-1:0] intf2cachehier_address;
wire [L2_WIDTH-1    :0]    intf2cachehier_data;
wire [MSG_BITS-1    :0]     cachehier2intf_msg;
wire [ADDRESS_BITS-1:0] cachehier2intf_address;
wire [L2_WIDTH-1    :0]    cachehier2intf_data;
wire [ADDRESS_BITS-1:0] llc_controller_address;
wire              llc_controller_address_valid;
wire mem_intf_busy;
wire [ADDRESS_BITS-1:0] mem_intf_address;
wire mem_intf_address_valid;

wire port1_read;
wire port1_write;
wire port1_invalidate;
wire [INDEX_BITS_L2-1 :0] port1_index;
wire [L2_TAG_BITS-1   :0] port1_tag;
wire [L2_MBITS-1      :0] port1_metadata;
wire [L2_WIDTH-1      :0] port1_write_data;
wire [L2_WAY_BITS-1   :0] port1_way_select;
wire [L2_WIDTH-1      :0] port1_read_data;
wire [L2_WAY_BITS-1   :0] port1_matched_way;
wire [COHERENCE_BITS-1:0] port1_coh_bits;
wire [STATUS_BITS_L2-1:0] port1_status_bits;
wire port1_hit;


//instantiate two_level_cache_hierarchy
two_level_cache_hierarchy #(
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
  .MAX_OFFSET_BITS(MAX_OFFSET_BITS)
) cache_hier (
  .clock(clock),
  .reset(reset),
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
  
  .mem2cachehier_msg(intf2cachehier_msg),
  .mem2cachehier_address(intf2cachehier_address),
  .mem2cachehier_data(intf2cachehier_data),
  .mem_intf_busy(mem_intf_busy),
  .mem_intf_address(mem_intf_address),
  .mem_intf_address_valid(mem_intf_address_valid),
  .cachehier2mem_msg(cachehier2intf_msg),
  .cachehier2mem_address(cachehier2intf_address),
  .cachehier2mem_data(cachehier2intf_data),
  
  .port1_read(port1_read),
  .port1_write(port1_write),
  .port1_invalidate(port1_invalidate),
  .port1_index(port1_index),
  .port1_tag(port1_tag),
  .port1_metadata(port1_metadata),
  .port1_write_data(port1_write_data),
  .port1_way_select(port1_way_select),
  .port1_read_data(port1_read_data),
  .port1_matched_way(port1_matched_way),
  .port1_coh_bits(port1_coh_bits),
  .port1_status_bits(port1_status_bits),
  .port1_hit(port1_hit),
  
  .scan(scan)
);


//instantiate lx_noc_interface
lx_noc_interface #(
  .STATUS_BITS(STATUS_BITS_L2),
  .COHERENCE_BITS(COHERENCE_BITS),
  .CACHE_OFFSET_BITS(OFFSET_BITS_L2),
  .DATA_WIDTH(DATA_WIDTH),
  .NUMBER_OF_WAYS(NUMBER_OF_WAYS_L2),
  .ADDRESS_BITS(ADDRESS_BITS),
  .INDEX_BITS(INDEX_BITS_L2),
  .MSG_BITS(MSG_BITS),
  .REQ_BUF_DEPTH_BITS(REQ_BUF_DEPTH_BITS),
  .RESP_BUF_DEPTH_BITS(RESP_BUF_DEPTH_BITS),
  .CORE(CORE),
  .CACHE_NO(CACHE_NO),
  .CONTROLLER_TYPE(CONTROLLER_TYPE),
  .ID_BITS(ID_BITS),
  .DEFAULT_DEST(DEFAULT_DEST)
) noc_intf (
  .clock(clock),
  .reset(reset),
  //interface with the NoC (via packetizer)
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
  //interface with cache controller
  .cache_msg_in(cachehier2intf_msg),
  .cache_address_in(cachehier2intf_address),
  .cache_data_in(cachehier2intf_data),
  //.controller_address(llc_controller_address),
  //.controller_address_valid(llc_controller_address_valid),
  .cache_msg_out(intf2cachehier_msg),
  .cache_address_out(intf2cachehier_address),
  .cache_data_out(intf2cachehier_data),
  .busy(mem_intf_busy),
  .current_address(mem_intf_address),
  .current_address_valid(mem_intf_address_valid),
  //interface with cache memory
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


endmodule
