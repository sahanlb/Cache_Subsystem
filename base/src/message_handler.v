/** @module : message_handler
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

/** Module description
* --------------------
 *  - Receives messages from the NoC and buffers them.
 *  - Separate FIFO buffers for requests and responses received.
 *  - FIFO read interfaces are exposed to the controller.
 *  - If the request buffer is full, message_receiver will send a NackB
 *    message to the requestor.
 *  - It is assumed that the response buffer is always given priority by the 
 *    controller because responses cannot be rejected.
 *  - Multiplex messages sent out to the NoC by the main controller in the 
 *    cache_noc_interface module and its own controller due to request buffer 
 *    being full.
 *  - Buffer output
 *    <Source_ID|Message|Address|Data>
 *
 *  Sub modules
 *  -----------
   * req_buffer
   * resp_buffer
 *
 *  Parameters
 *  ----------
   * PARENT - Indicate whether the parent module is a cache or a directory.
   *   This is used in filtering messages received and putting them in request
   *   and response FIFOs. 
   *   - Options "DIR/CACHE". (Default "CACHE".)
   * ID_BITS - Number of bits in source and destination IDs.
*/


module message_handler #(
parameter CACHE_OFFSET_BITS   =  2,
          DATA_WIDTH          = 32,
          ADDRESS_BITS        = 32,
          MSG_BITS            =  4,
          REQ_BUF_DEPTH_BITS  =  2,
          RESP_BUF_DEPTH_BITS =  2,
          PARENT              = "CACHE",
          ID_BITS             =  2,
          DEFAULT_DEST        =  0,
          //Use default value in module instantiation for following parameters
          CACHE_WORDS         = 1 << CACHE_OFFSET_BITS,
          CACHE_WIDTH         = DATA_WIDTH * CACHE_WORDS,
          BUF_WIDTH           = CACHE_WIDTH + MSG_BITS + ADDRESS_BITS + ID_BITS
)(
input  clock, reset,
//interface with the NoC
input  [MSG_BITS-1    :0] noc_msg_in,
input  [ADDRESS_BITS-1:0] noc_address_in,
input  [CACHE_WIDTH-1 :0] noc_data_in,
input  [ID_BITS-1     :0] noc_src_id,
input  packetizer_busy,
output [MSG_BITS-1    :0] noc_msg_out,
output [ADDRESS_BITS-1:0] noc_address_out,
output [CACHE_WIDTH-1 :0] noc_data_out,
output [ID_BITS-1     :0] noc_dest_id,
//interface with controller of the parent module
input  [MSG_BITS-1    :0] ctrl_msg_in,
input  [ADDRESS_BITS-1:0] ctrl_address_in,
input  [CACHE_WIDTH-1 :0] ctrl_data_in,
input  [ID_BITS-1     :0] ctrl_dest_id,
output reg intf_busy,
//exposed FIFO read interfaces
input  reqbuf_read,
input  respbuf_read,
output reqbuf_empty,
output reqbuf_full,
output reqbuf_valid,
output respbuf_empty,
output respbuf_full,
output respbuf_valid,
output [BUF_WIDTH-1:0] reqbuf_data,
output [BUF_WIDTH-1:0] respbuf_data
);

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

//internal variables
reg [MSG_BITS-1    :0] r_noc_msg_in;
reg [ADDRESS_BITS-1:0] r_noc_address_in;
reg [CACHE_WIDTH-1 :0] r_noc_data_in;
reg [ID_BITS-1     :0] r_noc_src_id;
reg [MSG_BITS-1    :0] r_noc_msg_out;
reg [ADDRESS_BITS-1:0] r_noc_address_out;
reg [CACHE_WIDTH-1 :0] r_noc_data_out;
reg [ID_BITS-1     :0] r_noc_dest_id;
reg [MSG_BITS-1    :0] r_ctrl_msg_in;
reg [ADDRESS_BITS-1:0] r_ctrl_address_in;
reg [CACHE_WIDTH-1 :0] r_ctrl_data_in;
reg [ID_BITS-1     :0] r_ctrl_dest_id;
reg input_buffer_valid;
reg [1:0] state;

wire recv_req, recv_resp;
wire reqbuf_write, respbuf_write;


/*control logic*/
//register inputs from NoC side
always @(posedge clock)begin
  if(reset)begin
    r_noc_msg_in     <= {MSG_BITS{1'b0}};
    r_noc_address_in <= {ADDRESS_BITS{1'b0}};
    r_noc_data_in    <= {CACHE_WIDTH{1'b0}};
    r_noc_src_id     <= {ID_BITS{1'b0}};
  end
  else begin
    r_noc_msg_in     <= noc_msg_in;
    r_noc_address_in <= noc_address_in;
    r_noc_data_in    <= noc_data_in;
    r_noc_src_id     <= noc_src_id;
  end
end

//output registers
always @(posedge clock)begin
  if(reset)begin
    r_noc_msg_out      <= NoMsg;
    r_noc_address_out  <= {ADDRESS_BITS{1'b0}};
    r_noc_data_out     <= {CACHE_WIDTH{1'b0}};
    r_noc_dest_id      <= DEFAULT_DEST;
    intf_busy          <= 1'b0;
    r_ctrl_msg_in      <= NoMsg;
    r_ctrl_address_in  <= 0;
    r_ctrl_data_in     <= 0;
    r_ctrl_dest_id     <= DEFAULT_DEST;
    input_buffer_valid <= 1'b0;
  end
  else if(packetizer_busy)begin
    if(~input_buffer_valid & ctrl_msg_in != NoMsg & ~intf_busy)begin
      r_ctrl_msg_in      <= ctrl_msg_in;
      r_ctrl_address_in  <= ctrl_address_in;
      r_ctrl_data_in     <= ctrl_data_in;
      r_ctrl_dest_id     <= ctrl_dest_id;
      input_buffer_valid <= 1'b1;
    end
    r_noc_msg_out     <= r_noc_msg_out;
    r_noc_address_out <= r_noc_address_out;
    r_noc_data_out    <= r_noc_data_out;
    r_noc_dest_id     <= r_noc_dest_id;
    intf_busy         <= 1'b1;
  end
  else begin
    if(recv_req & reqbuf_full)begin
      r_noc_msg_out     <= (PARENT == "CACHE") ? NackC : NackB;
      r_noc_address_out <= r_noc_address_in;
      r_noc_data_out    <= {CACHE_WIDTH{1'b0}};
      r_noc_dest_id     <= r_noc_src_id;
      intf_busy         <= 1'b1;
      if(ctrl_msg_in != NoMsg)begin
        r_ctrl_msg_in      <= ctrl_msg_in;
        r_ctrl_address_in  <= ctrl_address_in;
        r_ctrl_data_in     <= ctrl_data_in;
        r_ctrl_dest_id     <= ctrl_dest_id;
        input_buffer_valid <= 1'b1;
      end
    end
    else begin
      //if(input_buffer_valid)begin
      if(intf_busy)begin
        r_noc_msg_out      <= input_buffer_valid ? r_ctrl_msg_in : NoMsg;
        /*check for input_buffer_valid is only done for r_noc_msg_out since
        * address and data signals are ignored when message == NoMsg*/
        r_noc_address_out  <= r_ctrl_address_in;
        r_noc_data_out     <= r_ctrl_data_in;
        r_noc_dest_id      <= r_ctrl_dest_id;
        input_buffer_valid <= 1'b0;
        intf_busy          <= 1'b0;
      end
      else begin
        r_noc_msg_out     <= ctrl_msg_in;
        r_noc_address_out <= ctrl_address_in;
        r_noc_data_out    <= ctrl_data_in;
        r_noc_dest_id     <= ctrl_dest_id;
        intf_busy         <= 1'b0;
      end
    end
  end
end


//assignments
generate
  if(PARENT == "CACHE")begin
    assign recv_req  = (r_noc_msg_in == FwdGetS) | (r_noc_msg_in == Inv) ;
    
    assign recv_resp = (r_noc_msg_in == PutAck) | (r_noc_msg_in == Data ) |
                       (r_noc_msg_in == DataS ) | (r_noc_msg_in == NackB) ;
  end
  else begin
    assign recv_req  = (r_noc_msg_in == GetS) | (r_noc_msg_in == GetM) |
                       (r_noc_msg_in == PutM) | (r_noc_msg_in == PutS) ;
    
    assign recv_resp = (r_noc_msg_in == PutE  ) | (r_noc_msg_in == InvAck) |
                       (r_noc_msg_in == NackD ) | (r_noc_msg_in == NackC ) |
                       (r_noc_msg_in == RespPutM);
  end
endgenerate

assign reqbuf_write  = recv_req  & ~reqbuf_full;
assign respbuf_write = recv_resp & ~respbuf_full;

assign noc_msg_out     = r_noc_msg_out;
assign noc_address_out = r_noc_address_out;
assign noc_data_out    = r_noc_data_out;
assign noc_dest_id     = r_noc_dest_id;


//instantiate request buffer
fifo #(
  .DATA_WIDTH(BUF_WIDTH),
  .Q_DEPTH_BITS(REQ_BUF_DEPTH_BITS),
  .Q_IN_BUFFERS(0)
) req_buffer (
    .clk(clock),
    .reset(reset),
    .write_data({r_noc_src_id, r_noc_msg_in, r_noc_address_in, r_noc_data_in}),
    .wrtEn(reqbuf_write),
    .rdEn(reqbuf_read),
    .peek(1'b0),
    .read_data(reqbuf_data),
    .valid(reqbuf_valid),
    .full(reqbuf_full),
    .empty(reqbuf_empty)
);

//instantiate response buffer
fifo #(
  .DATA_WIDTH(BUF_WIDTH),
  .Q_DEPTH_BITS(RESP_BUF_DEPTH_BITS),
  .Q_IN_BUFFERS(0)
) resp_buffer (
    .clk(clock),
    .reset(reset),
    .write_data({r_noc_src_id, r_noc_msg_in, r_noc_address_in, r_noc_data_in}),
    .wrtEn(respbuf_write),
    .rdEn(respbuf_read),
    .peek(1'b0),
    .read_data(respbuf_data),
    .valid(respbuf_valid),
    .full(respbuf_full),
    .empty(respbuf_empty)
);

endmodule

