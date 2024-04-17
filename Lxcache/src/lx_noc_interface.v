/** @module : lx_noc_interface
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
 *  - Interface module which connects lx cache to a NoC.
 *  - Have two interfaces to;
 *    1) Interface with the cache
 *      - Interface with cache controller and service memory requests
 *      - Read cache memory for coherence operations
 *      - Forward invalidation requests from the directory to the cache controller
 *    2) Interface with the NoC
 *
 *  Sub modules
 *  -----------
   * message_handler - receives messages from the NoC and buffers those. 
   * controls outgoing messages to the NoC. Multiplex between messages from 
   * controller and any Nack messages from the handler itself.
 *  
 *  Parameters
 *  ----------
   * CONTROLLER_TYPE - Specify whether the cache controller is a blocking or
   *   non-blocking one. Blocking controller will hold the signals to the
   *   interface asserted until the interface responds. Non-blocking controller
   *   will deassert the sinals after one cycle unless the interface is busy in
   *   which case the interface will not register the message from the 
   *   contrller.
   * REQ_BUF_DEPTH_BITS - log2(request buffer depth)
   * RESP_BUF_DEPTH_BITS - log2(response buffer depth)
*/


module lx_noc_interface #(
parameter STATUS_BITS         =  3, //{valid, dirty, inclusion}
          COHERENCE_BITS      =  2,
          CACHE_OFFSET_BITS   =  2,
          DATA_WIDTH          = 32,
          NUMBER_OF_WAYS      =  4,
          ADDRESS_BITS        = 32,
          INDEX_BITS          =  8,
          MSG_BITS            =  4,
          REQ_BUF_DEPTH_BITS  =  2,
          RESP_BUF_DEPTH_BITS =  2,
          CORE                =  0,
          CACHE_NO            =  0,
          CONTROLLER_TYPE     = "BLOCKING",
          ID_BITS             =  2,
          DEFAULT_DEST        =  0,
          //Use default value in module instantiation for following parameters
          CACHE_WORDS         = 1 << CACHE_OFFSET_BITS,
          CACHE_WIDTH         = DATA_WIDTH * CACHE_WORDS,
          WAY_BITS            = (NUMBER_OF_WAYS > 1) ? log2(NUMBER_OF_WAYS) : 1,
          TAG_BITS            = ADDRESS_BITS - INDEX_BITS - CACHE_OFFSET_BITS,
          SBITS               = COHERENCE_BITS + STATUS_BITS,
          BUF_WIDTH           = CACHE_WIDTH + MSG_BITS + ADDRESS_BITS + ID_BITS
)(
input  clock, reset,
//interface with the NoC (via packetizer)
input  [MSG_BITS-1    :0] noc_msg_in,
input  [ADDRESS_BITS-1:0] noc_address_in,
input  [CACHE_WIDTH-1 :0] noc_data_in,
input  [ID_BITS-1     :0] noc_src_id,
input  packetizer_busy,
output [MSG_BITS-1    :0] noc_msg_out,
output [ADDRESS_BITS-1:0] noc_address_out,
output [CACHE_WIDTH-1 :0] noc_data_out,
output [ID_BITS-1     :0] noc_dest_id,
output interface_busy,

//interface with cache controller
input  [MSG_BITS-1    :0] cache_msg_in,
input  [ADDRESS_BITS-1:0] cache_address_in,
input  [CACHE_WIDTH-1 :0] cache_data_in,
output [MSG_BITS-1    :0] cache_msg_out,
output [ADDRESS_BITS-1:0] cache_address_out,
output [CACHE_WIDTH-1 :0] cache_data_out,
output busy,
output [ADDRESS_BITS-1:0] current_address,
output reg current_address_valid,

//interface with cache memory
input  [CACHE_WIDTH-1   :0] port1_read_data,
input  [WAY_BITS-1      :0] port1_matched_way,
input  [COHERENCE_BITS-1:0] port1_coh_bits,
input  [STATUS_BITS-1   :0] port1_status_bits,
input  port1_hit,
output port1_read, port1_write, port1_invalidate,
output [INDEX_BITS-1 :0] port1_index,
output [TAG_BITS-1   :0] port1_tag,
output [SBITS-1      :0] port1_metadata,
output [CACHE_WIDTH-1:0] port1_write_data,
output [WAY_BITS-1   :0] port1_way_select
);

`include `INCLUDE_FILE

//define the log2 function
function integer log2;
input integer value;
begin
  value = value-1;
  for (log2=0; value>0; log2=log2+1)
    value = value >> 1;
end
endfunction

//function to convert cache messages
function [MSG_BITS-1:0] convert_message;
input [MSG_BITS-1:0] cache_msg;
begin
  case(cache_msg)
    NO_REQ    : convert_message = NoMsg;
    R_REQ     : convert_message = GetS;
    WB_REQ    : convert_message = PutM;
    FLUSH     : convert_message = PutM; //Add real flush instruction to the NoC later
    FLUSH_S   : convert_message = PutS; //Add real flush instruction to the NoC later
    RFO_BCAST : convert_message = GetM;
    WS_BCAST  : convert_message = GetM;
    default   : convert_message = NoMsg;
  endcase
end
endfunction
//Messages from the cache controller are different from the NoC messages since 
//it uses the bus messages. Conversion from bus messages to NoC messages happens 
//within this module.


/*Local parameters*/
//state encoding
localparam IDLE        = 0,
           DIR_RESP    = 1,
           BACKOFF     = 2,
           MEM_READ    = 3,
           DIR_REQ     = 4,
           CACHE_REQ   = 5,
           NOC_RESP    = 6,
           FWD_CTRL    = 7;
           

//Internal signals
reg [2:0] state;
reg inservice; //Used only in BLOCKING mode to indicate that the interface is 
               //currently servicing a request from the cache controller.
reg [MSG_BITS-1    :0] r_msg_out;
reg [MSG_BITS-1    :0] r_cache_msg_out;
reg [ADDRESS_BITS-1:0] r_address_out;
reg [CACHE_WIDTH-1 :0] r_data_out;
reg [ID_BITS-1     :0] r_dest_id;
reg [MSG_BITS-1    :0] cur_request; //request picked to serve
reg [ADDRESS_BITS-1:0] cur_address;
reg [CACHE_WIDTH-1 :0] cur_data;
/*Following 3 signals are not used in the current implementation becase the
* cache controller blocks until a request is serviced by the noc interface*/
/*reg [MSG_BITS-1    :0] stored_request;
reg [ADDRESS_BITS-1:0] stored_address;
reg [CACHE_WIDTH-1 :0] stored_data;*/

reg r_port1_write, r_port1_invalidate;
reg [SBITS-1      :0] r_port1_metadata;
reg [CACHE_WIDTH-1:0] r_port1_write_data;
reg [WAY_BITS-1   :0] r_port1_way_select;
reg [STATUS_BITS-1:0] r_port1_status_bits;

wire w_intf_busy;
wire w_reqbuf_read, w_reqbuf_full, w_reqbuf_empty, w_reqbuf_valid;
wire [BUF_WIDTH-1:0] w_reqbuf_data;
wire w_respbuf_read, w_respbuf_full, w_respbuf_empty, w_respbuf_valid;
wire [BUF_WIDTH-1:0] w_respbuf_data;
wire [MSG_BITS-1 :0] w_cache_msg_out;
wire collision; //trying to access the same address as the cache controller



//Controller FSM
always @(posedge clock)begin
  if(reset)begin
    r_msg_out             <= NoMsg;
    r_cache_msg_out       <= NoMsg;
    r_address_out         <= 0;
    r_data_out            <= 0; 
    r_dest_id             <= DEFAULT_DEST; //At the moment always use default value.
    cur_request           <= NoMsg;
    cur_address           <= 0;
    cur_data              <= 0;
    inservice             <= 1'b0;
    r_port1_write         <= 1'b0;
    r_port1_invalidate    <= 1'b0;
    r_port1_way_select    <= 0;
    r_port1_metadata      <= 0;
    r_port1_write_data    <= 0;
    r_port1_status_bits   <= {STATUS_BITS{1'b0}};
    current_address_valid <= 1'b0;
    state                 <= IDLE;
  end
  else begin
    case(state)
      IDLE:begin
        if(~w_respbuf_empty)begin
          cur_request   <= w_respbuf_data[BUF_WIDTH-ID_BITS-1 -: MSG_BITS];
          cur_address   <= w_respbuf_data[CACHE_WIDTH +: ADDRESS_BITS];
          cur_data      <= w_respbuf_data[0 +: CACHE_WIDTH];
          state         <= DIR_RESP;
        end
        else if(~w_reqbuf_empty)begin
          cur_request <= w_reqbuf_data[BUF_WIDTH-ID_BITS-1 -: MSG_BITS];
          cur_address <= w_reqbuf_data[CACHE_WIDTH +: ADDRESS_BITS];
          cur_data    <= w_reqbuf_data[0 +: CACHE_WIDTH];
          //not registering the source ID because it is assumed that only the
          //directory will send messages and DEFAULT_DEST is set to directory
          //address.
          current_address_valid <= 1'b1;
          state                 <= collision ? BACKOFF : MEM_READ;
        end
        else if(!inservice & (cache_msg_in != NO_REQ))begin
          cur_request           <= cache_msg_in;
          cur_address           <= cache_address_in;
          cur_data              <= cache_data_in;
          inservice             <= (CONTROLLER_TYPE == "BLOCKING") ? 1'b1 : 1'b0;
          current_address_valid <= 1'b1;
          state                 <= CACHE_REQ;
        end
        else begin
          cur_request <= cur_request;
          cur_address <= cur_address;
          cur_data    <= cur_data;
          state       <= IDLE;
        end
      end
      DIR_RESP:begin
        if(cur_request == NackB & inservice)begin //directory rejected the request
          r_msg_out     <= cache_msg_in;          //by cache_controller
          r_address_out <= cache_address_in;
          r_data_out    <= cache_data_in;
          state         <= NOC_RESP;
          //assuming the blocking cache controller which keeps holding the
          //request until the noc_interface responds. For a non blocking
          //controller, logic should be added to figure out which request to
          //be reissued.
        end
        else begin
          cur_request <= NoMsg;
          cur_address <= 0;
          cur_data    <= 0;
          inservice   <= 1'b0;
          state       <= IDLE;
        end
      end
      MEM_READ:begin //memory is read using port1 during this state.
        //Backoff if there is an address collision
        state <= collision ? BACKOFF : DIR_REQ;
      end
      DIR_REQ:begin
        if(collision)begin
          state <= BACKOFF;
        end
        else begin
          case(cur_request)
            FwdGetS:begin
              if(port1_hit)begin
                if(port1_status_bits[0])begin //include bit is set.
                  r_cache_msg_out       <= cur_request; //TODO: never reset in NoMsg
                  r_port1_metadata      <= {3'b100, SHARED};
                  r_port1_write_data    <= port1_read_data;
                  r_port1_way_select    <= port1_matched_way;
                  r_data_out            <= port1_read_data;
                  r_port1_status_bits   <= port1_status_bits;
                  current_address_valid <= 1'b0;
                  state                 <= FWD_CTRL;
                end
                else begin
                  r_port1_metadata   <= {3'b100, SHARED};
                  r_port1_write_data <= port1_read_data;
                  r_port1_way_select <= port1_matched_way;
                  r_port1_write      <= 1'b1;
                  r_msg_out          <= (port1_status_bits[1]) ? RespPutM : PutE;
                  r_address_out      <= cur_address; 
                  r_data_out         <= port1_read_data;
                  state              <= NOC_RESP; 
                end
              end
              else begin //Don't have the line requested
                r_msg_out     <= NackD;
                r_address_out <= cur_address; 
                r_data_out    <= 0;
                state         <= NOC_RESP;
              end
            end
            Inv:begin
              if(port1_hit)begin
                if(port1_status_bits[0])begin //include bit is set.
                  r_cache_msg_out       <= cur_request;
                  r_port1_metadata      <= {3'b000, INVALID};
                  r_port1_write_data    <= 0;
                  r_port1_way_select    <= port1_matched_way;
                  r_data_out            <= port1_read_data;
                  r_port1_status_bits   <= port1_status_bits;
                  current_address_valid <= 1'b0;
                  state                 <= FWD_CTRL;
                end
                else begin
                  r_port1_metadata   <= {3'b000, INVALID};
                  r_port1_write_data <= 0;
                  r_port1_way_select <= port1_matched_way;
                  r_port1_invalidate <= 1'b1;
                  r_msg_out          <= (port1_status_bits[1]) ? RespPutM : InvAck;
                  r_address_out      <= cur_address; 
                  r_data_out         <= port1_read_data;
                  state              <= NOC_RESP; 
                end
              end
              else begin //Don't have the line requested
                r_msg_out     <= NackD;
                r_address_out <= cur_address; 
                r_data_out    <= 0;
                state         <= NOC_RESP;
              end
            end
          endcase
        end
      end
      FWD_CTRL:begin
        if(cache_msg_in == C_FLUSH)begin
          //r_port1_invalidate <= (cur_request == Inv) ? 1'b1 : 1'b0;
          //r_port1_write      <= (cur_request == Inv) ? 1'b0 : 1'b1;
          r_msg_out          <= RespPutM;
          r_address_out      <= cur_address; 
          r_data_out         <= cache_data_in;
          state              <= NOC_RESP; 
        end
        else if(cache_msg_in == EN_ACCESS)begin
          //r_port1_invalidate <= (cur_request == Inv) ? 1'b1 : 1'b0;
          //r_port1_write      <= (cur_request == Inv) ? 1'b0 : 1'b1;
          //r_msg_out          <= (r_port1_status_bits[1]) ? RespPutM : InvAck;
          r_msg_out          <= (cur_request == Inv) ? InvAck : PutE;
          r_address_out      <= cur_address; 
          state              <= NOC_RESP; 
        end
        else begin
          state <= FWD_CTRL;
        end
      end
      CACHE_REQ:begin
        r_msg_out     <= convert_message(cur_request);
        r_address_out <= cur_address;
        r_data_out    <= cur_data;
        state         <= NOC_RESP;
      end
      NOC_RESP:begin
        r_port1_write      <= 1'b0;
        r_port1_invalidate <= 1'b0;
        if(w_intf_busy)begin               
          r_msg_out     <= r_msg_out;     
          r_address_out <= r_address_out; 
          r_data_out    <= r_data_out;    
          cur_request   <= cur_request;
          cur_address   <= cur_address;
          cur_data      <= cur_data;
          state         <= NOC_RESP;
        end
        else begin
          r_msg_out             <= NoMsg;
          r_address_out         <= 0;
          r_data_out            <= 0;
          cur_request           <= NoMsg;
          cur_address           <= 0;
          cur_data              <= 0;
          current_address_valid <= 1'b0;
          state                 <= IDLE;
        end
      end
      BACKOFF:begin
        if(cache_msg_in != NO_REQ)begin
          cur_request <= cache_msg_in;
          cur_address <= cache_address_in;
          cur_data    <= cache_data_in;
          inservice   <= (CONTROLLER_TYPE == "BLOCKING") ? 1'b1 : 1'b0;
          state       <= CACHE_REQ;
        end
        else begin
          state <= collision ? BACKOFF : MEM_READ;
        end
      end
      default:begin
        cur_request   <= NoMsg;
        cur_address   <= 0;
        cur_data      <= 0;
        r_msg_out     <= NoMsg;
        r_address_out <= 0;
        r_data_out    <= 0; 
        state         <= IDLE;
      end
    endcase
  end
end


//Continuous assignments
//busy signal to the cache controller
assign busy = ~((state == IDLE) & w_reqbuf_empty & w_respbuf_empty);

assign w_respbuf_read = (state == DIR_RESP);
assign w_reqbuf_read  = (state == DIR_REQ) & ~collision;
/*Added the check for collision because if the cache controller issues
* a read or write request for the colliding address, noc interface has to
* first service that request before completing its current request from the
* directory. Form BACKOFF state, noc interface will go to servicing the
* request from the cache controller. Since we do not pop the first item in the
* request buffer we can come back to the same request after servicing the
* cache controller's request.*/

assign w_cache_msg_out = (cur_request == PutAck) ? MEM_RESP : 
                         (cur_request == Data & cache_msg_in == WS_BCAST) ? EN_ACCESS :
                         (cur_request == NackB ) ? NO_REQ : cur_request;

assign cache_msg_out     = (state == DIR_RESP) ? w_cache_msg_out : 
                           (state == FWD_CTRL) ? r_cache_msg_out :
                           NoMsg;
assign cache_address_out = cur_address;
assign cache_data_out    = cur_data;

assign port1_read       = (state == MEM_READ) ? 1'b1 : 1'b0;
assign port1_index      = cur_address[CACHE_OFFSET_BITS +: INDEX_BITS];
assign port1_tag        = cur_address[ADDRESS_BITS-1 -: TAG_BITS];
assign port1_write      = r_port1_write;
assign port1_invalidate = r_port1_invalidate;
assign port1_metadata   = r_port1_metadata;
assign port1_way_select = r_port1_way_select;
assign port1_write_data = r_port1_write_data;

//assign collision = (cur_address == controller_address) & controller_address_valid;
assign collision = 1'b0;

//busy signal to the NoC
assign interface_busy = w_reqbuf_full | w_respbuf_full;

//send current address to cache controller
assign current_address = cur_address;


//instantiate message handler
message_handler #(
  .CACHE_OFFSET_BITS(CACHE_OFFSET_BITS),
  .DATA_WIDTH(DATA_WIDTH),
  .ADDRESS_BITS(ADDRESS_BITS),
  .MSG_BITS(MSG_BITS),
  .REQ_BUF_DEPTH_BITS(REQ_BUF_DEPTH_BITS),
  .RESP_BUF_DEPTH_BITS(RESP_BUF_DEPTH_BITS),
  .PARENT("CACHE"),
  .ID_BITS(ID_BITS),
  .DEFAULT_DEST(DEFAULT_DEST)
) message_handler (
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
  .ctrl_msg_in(r_msg_out),
  .ctrl_address_in(r_address_out),
  .ctrl_data_in(r_data_out),
  .ctrl_dest_id(r_dest_id),
  .intf_busy(w_intf_busy),
  .reqbuf_read(w_reqbuf_read),
  .respbuf_read(w_respbuf_read),
  .reqbuf_empty(w_reqbuf_empty),
  .reqbuf_full(w_reqbuf_full),
  .reqbuf_valid(w_reqbuf_valid),
  .respbuf_empty(w_respbuf_empty),
  .respbuf_full(w_respbuf_full),
  .respbuf_valid(w_respbuf_valid),
  .reqbuf_data(w_reqbuf_data),
  .respbuf_data(w_respbuf_data)
);



endmodule
