/** @module : directory_controller
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
 *  - Stores coherence state for all the cached memory blocks in the directory 
 *    cache and manages cache coherence.
 *  - Memory interface can be connected to either main memory or a shared
 *    cache.
 *  - Respond to requests from the caches after checking the coherence state of
 *    a memory block (from directory cache) and getting the memory block
 *    through the memory interface.
 *
 *  Sub modules
 *  -----------
   * message_handler - receives messages from the NoC and buffers those. 
   *  controls outgoing messages to the NoC. Multiplex between messages from 
   *  controller and any Nack messages from the handler itself.
   * directory_cache - Holds coherence state for all cached memory blocks.
   *  Only a cache_memory module. Reads and writes controlled by directory
   *  controller.
   *  Cache line: <v|tag|sharer list>
   *  Limited pointer representation for sharers.
 *  
 *  Parameters
 *  ----------
   * REQ_BUF_DEPTH_BITS - log2(request buffer depth)
   * RESP_BUF_DEPTH_BITS - log2(response buffer depth)
   * OFFSET_BITS - (word) offset bits for sharer caches.
   * ACTIVE_REQS - Number of active requests handled by the directory before
   *  rejecting the next request. Determines how many sets of registers are
   *  available to store the status of active requests.
      * (Use a power of 2 for ACTIVE_REQS)
*/

module directory_controller #(
parameter OFFSET_BITS         =  2,
          DATA_WIDTH          = 32,
          ADDRESS_BITS        = 32,
          MSG_BITS            =  4,
          REQ_BUF_DEPTH_BITS  =  2,
          RESP_BUF_DEPTH_BITS =  2,
          NUM_SHARERS_BITS    =  2, //log2(No. of pointers)
          SHARER_ID_BITS      =  2,
          DIR_INDEX_BITS      =  8,
          DIR_WAYS            =  8,
          DEFAULT_DEST        =  0,
          ACTIVE_REQS         =  2, 
          //Use default value in module instantiation for following parameters
          CACHE_WORDS         = 1 << OFFSET_BITS,
          CACHE_WIDTH         = DATA_WIDTH * CACHE_WORDS,
          TAG_BITS            = ADDRESS_BITS - DIR_INDEX_BITS - OFFSET_BITS,
          BUF_WIDTH           = CACHE_WIDTH + MSG_BITS + ADDRESS_BITS + SHARER_ID_BITS,
          WAY_BITS            = (DIR_WAYS > 1) ? log2(DIR_WAYS) : 1,
          NUM_SHARERS         = 1 << NUM_SHARERS_BITS,
          DIR_WIDTH           = SHARER_ID_BITS * NUM_SHARERS,
          DIR_DEPTH           = 1 << DIR_INDEX_BITS

)(
input  clock,
input  reset,
//interface with the NoC
input  [MSG_BITS-1      :0] noc_msg_in,
input  [ADDRESS_BITS-1  :0] noc_address_in,
input  [CACHE_WIDTH-1   :0] noc_data_in,
input  [SHARER_ID_BITS-1:0] noc_src_id,
input  packetizer_busy,
output [MSG_BITS-1      :0] noc_msg_out,
output [ADDRESS_BITS-1  :0] noc_address_out,
output [CACHE_WIDTH-1   :0] noc_data_out,
output [SHARER_ID_BITS-1:0] noc_dest_id,
//memory interface
input  [MSG_BITS-1      :0] mem_msg_in,
input  [ADDRESS_BITS-1  :0] mem_address_in,
input  [CACHE_WIDTH-1   :0] mem_data_in,
output [MSG_BITS-1      :0] mem_msg_out,
output [ADDRESS_BITS-1  :0] mem_address_out,
output [CACHE_WIDTH-1   :0] mem_data_out,

input  report
);

integer i, k, l, found;
genvar j;

//define the log2 function
function integer log2;
input integer value;
begin
  value = value-1;
  for (log2=0; value>0; log2=log2+1)
    value = value >> 1;
  end
endfunction

//function to count sharers(pointer does not point to default destination)
function integer sharers;
input [DIR_WIDTH-1:0] line;
begin
  sharers = 0;
  for(k=0; k<NUM_SHARERS; k=k+1)begin
    if(line[k*SHARER_ID_BITS +: SHARER_ID_BITS] != DEFAULT_DEST)begin
      sharers = sharers + 1;
    end
  end
end
endfunction

//function to find the first free slot in sharer list
function integer free_slot;
input [DIR_WIDTH-1:0] line;
begin
  free_slot = 0;
  found = 0;
  for(k=0; k<NUM_SHARERS; k=k+1)begin
    if(line[k*SHARER_ID_BITS +: SHARER_ID_BITS] == DEFAULT_DEST & ~found)begin
      free_slot = k;
      found     = 1;
    end
  end
end
endfunction

//function to find the first sharer in the list
function integer first_sharer;
input [DIR_WIDTH-1:0] line;
begin
  first_sharer = 0;
  found = 0;
  for(l=0; l<NUM_SHARERS; l=l+1)begin
    if(line[l*SHARER_ID_BITS +: SHARER_ID_BITS] != DEFAULT_DEST & ~found)begin
      first_sharer = l;
      found        = 1;
    end
  end
end
endfunction

//function to give the used slots in the sharer list as a bit vector
function [NUM_SHARERS-1:0] sharer_bits;
input [DIR_WIDTH-1:0] line;
begin
  sharer_bits = 0;
  for(k=0; k<NUM_SHARERS; k=k+1)begin
    if(line[k*SHARER_ID_BITS +: SHARER_ID_BITS] != DEFAULT_DEST)
      sharer_bits[k] = 1'b1;
  end
end
endfunction

//function to remove current requestor from the sharer list
function [DIR_WIDTH-1:0] remove_src;
input [DIR_WIDTH-1:0] line;
input [SHARER_ID_BITS-1:0] cur_src;
begin
  for(k=0; k<NUM_SHARERS; k=k+1)begin
    if(line[k*SHARER_ID_BITS +: SHARER_ID_BITS] == cur_src)
      remove_src[k*SHARER_ID_BITS +: SHARER_ID_BITS] = DEFAULT_DEST;
    else
      remove_src[k*SHARER_ID_BITS +: SHARER_ID_BITS] = 
        line[k*SHARER_ID_BITS +: SHARER_ID_BITS];
  end
end
endfunction

//function to count ones in a bit vector
function integer ones;
input [ACTIVE_REQS-1:0] active;
begin
  ones = 0;
  for(k=0; k<ACTIVE_REQS; k=k+1)begin
    if(active[k])
      ones = ones + 1;
  end
end
endfunction


`include `INCLUDE_FILE

/*local parameters*/
//state encoding for main FSM
localparam IDLE         =  0,
           RESET        =  1, 
           DIR_ADDR     =  2,
           DIR_READ     =  3,
           READ_MEM     =  4,
           WRITE_MEM    =  5,
           HANDLE_REQ   =  6,
           HANDLE_RESP  =  7,
           CHECK_DATA   =  8,
           START_RECALL =  9,
           DIR_ACCESS2  = 10,
           SEND_RECALL  = 11,
           WAIT_INTF    = 12;
           
//state encoding for memory interface FSM
localparam M_IDLE  = 0,
           M_READ  = 1,
           M_WRITE = 2,
           M_WAIT  = 3;


/*internal signals*/
reg [3:0] state;
reg [1:0] mstate;
reg [3:0] goto_state;
reg goto;
reg [DIR_INDEX_BITS-1:0] reset_counter;

//status registers for holding active requests
reg [ACTIVE_REQS-1    :0] active;
reg [MSG_BITS-1       :0] active_req       [ACTIVE_REQS-1:0];
reg [ADDRESS_BITS-1   :0] active_address   [ACTIVE_REQS-1:0];
reg [CACHE_WIDTH-1    :0] active_data      [ACTIVE_REQS-1:0];
reg [SHARER_ID_BITS-1 :0] active_requestor [ACTIVE_REQS-1:0];
reg [DIR_WIDTH-1      :0] active_sharers   [ACTIVE_REQS-1:0];
reg [NUM_SHARERS-1    :0] active_responses [ACTIVE_REQS-1:0];

reg [log2(NUM_SHARERS):0] active_index;
reg [DIR_WIDTH-1      :0] dir_line; //line read from dir$.
reg [WAY_BITS-1       :0] dir_way; //way in dir$ active line is read from.
reg [TAG_BITS-1       :0] dir_tag;
reg dir_hit; //dir$ read is a hit.
reg dir_modified; //directory line has the modified bit set
reg dir_valid; //dir$ returned a valid line.
reg valid_response; //response is from a sharer from whome we are expecting one

reg [MSG_BITS-1      :0] cur_msg; // request/response picked from buffers
reg [ADDRESS_BITS-1  :0] cur_address;
reg [CACHE_WIDTH-1   :0] cur_data;
reg [SHARER_ID_BITS-1:0] cur_src; //source node of the current message

reg [MSG_BITS-1      :0] r_mem_msg_out;
reg [ADDRESS_BITS-1  :0] r_mem_address_out;
reg [CACHE_WIDTH-1   :0] r_mem_data_out;
reg [MSG_BITS-1      :0] r_mem_msg_in;
reg [ADDRESS_BITS-1  :0] r_mem_address_in;
reg [CACHE_WIDTH-1   :0] r_mem_data_in;
reg [MSG_BITS-1      :0] r_intf_msg_out;
reg [ADDRESS_BITS-1  :0] r_intf_address_out;
reg [CACHE_WIDTH-1   :0] r_intf_data_out;
reg [SHARER_ID_BITS-1:0] r_intf_dest; //destination node of the message sent

reg [DIR_WIDTH-1     :0] cache_data_in1;
reg [DIR_INDEX_BITS-1:0] cache_index1;
reg [TAG_BITS-1      :0] cache_tag_in1;
reg [WAY_BITS-1      :0] cache_way_select1;
reg [1               :0]cache_metadata1; //valid and modified bits
reg cache_read1, cache_write1;
reg cache_invalidate1;

reg handling_resp;
reg read_memory, write_memory, clear_memreq;

reg [log2(NUM_SHARERS)-1:0] request_count;
reg [ADDRESS_BITS-1  :0] recall_address;

reg from_active;
reg [log2(ACTIVE_REQS)-1:0] from_active_index;

reg r_reqbuf_read, r_respbuf_read;

wire intf_busy;
wire [BUF_WIDTH-1:0] w_reqbuf_data;
wire [BUF_WIDTH-1:0] w_respbuf_data;
wire w_reqbuf_full, w_reqbuf_empty, w_reqbuf_valid;
wire w_respbuf_full, w_respbuf_empty, w_respbuf_valid;
wire [DIR_WIDTH-1     :0] cache_data_in0;
wire [DIR_WIDTH-1     :0] cache_data_out0, cache_data_out1;
wire [DIR_INDEX_BITS-1:0] cache_index0;
wire [TAG_BITS-1      :0] cache_tag_in0, cache_tag_out0, cache_tag_out1;
wire [WAY_BITS-1      :0] cache_matched_way0, cache_matched_way1;
wire [WAY_BITS-1      :0] cache_way_select0;
wire [1               :0] cache_metadata0; //valid bit and modified bits
wire cache_status_bits0, cache_status_bits1; //valid bit
wire cache_coh_bits0, cache_coh_bits1; //modified bit
wire cache_hit0, cache_hit1;
wire cache_read0, cache_write0;
wire cache_invalidate0;
wire i_reset;

wire active_ready; //active request is ready to be serviced.
wire [ACTIVE_REQS-1      :0] w_ready_reg;
wire [log2(ACTIVE_REQS)-1:0] ready_reg; //which active request is ready
wire cur_match; //one of the active requests match the address of the current request
wire [ACTIVE_REQS-1      :0] w_cur_match;
wire [log2(ACTIVE_REQS)-1:0] cur_match_reg; //which active req matches current one.
wire free_reg; //free status reg available
wire [log2(ACTIVE_REQS)-1:0] free_index; //index of free status register
wire [log2(ACTIVE_REQS)  :0] free_reg_count;

wire [SHARER_ID_BITS-1:0] w_active_sharers [NUM_SHARERS-1:0];
wire [SHARER_ID_BITS-1:0] w_cache_line_slices [NUM_SHARERS-1:0];
wire [NUM_SHARERS-1   :0] w_active_index;
wire [log2(NUM_SHARERS)-1:0] w_active_index_binary;
wire w_active_index_valid;

wire [DIR_WIDTH-1:0] w_write_line1; //for cache responses
wire [DIR_WIDTH-1:0] w_write_line2; //directory update for GetS
wire [DIR_WIDTH-1:0] w_write_line3; //directory update for GetS
wire [DIR_WIDTH-1:0] w_write_line4; //directory update for GetS

//MESI flags and sharer count for dir_line
wire M, E, S, I;
wire [log2(NUM_SHARERS):0] sharer_count;

wire [ADDRESS_BITS-1:0] dir_address; //shifted address used to access directory

generate
//split signals
  for(j=0; j<NUM_SHARERS; j=j+1)begin:W_ACTIVE_SHARERS
    assign w_active_sharers[j] = active_sharers[cur_match_reg][j*SHARER_ID_BITS +: SHARER_ID_BITS];
  end
  for(j=0; j<NUM_SHARERS; j=j+1)begin:CACHE_LINE_SLICES
    assign w_cache_line_slices[j] = dir_line[j*SHARER_ID_BITS +: SHARER_ID_BITS];
  end

//build write line 1
  for(j=0; j<NUM_SHARERS; j=j+1)begin:WRITE_LINE1
    assign w_write_line1[j*SHARER_ID_BITS +: SHARER_ID_BITS] = 
      (j == active_index) ? DEFAULT_DEST : w_cache_line_slices[j];
    //Only InvAck causes the directory state to update. It removes the sharer
    //from the sharer list. Therefore, setting the pointer to default
    //destination.
  end

//build write line 2
  for(j=0; j<NUM_SHARERS; j=j+1)begin:WRITE_LINE2
    assign w_write_line2[j*SHARER_ID_BITS +: SHARER_ID_BITS] = 
      (j == free_slot(dir_line)) ? cur_src : w_cache_line_slices[j];
    //adds the current requestor (cur_src) to the first free slot in the
    //sharer list.
  end

//build write line 3
  for(j=0; j<NUM_SHARERS; j=j+1)begin:WRITE_LINE3
    assign w_write_line3[j*SHARER_ID_BITS +: SHARER_ID_BITS] = 
      (w_cache_line_slices[j] == cur_src) ? DEFAULT_DEST : w_cache_line_slices[j];
    //For PutS
    //Remove the current requestor (cur_src) from the sharer list.
  end

//build write line 4
  for(j=0; j<NUM_SHARERS; j=j+1)begin:WRITE_LINE4
    assign w_write_line4[j*SHARER_ID_BITS +: SHARER_ID_BITS] = 
      (j == 0) ? cur_src : DEFAULT_DEST;
    //For GetM: adds the current requestor (cur_src) to the first slot in the
    //sharer list and set everything else to default dest.
  end

//track active requests ready to be serviced
  for(j=0; j<ACTIVE_REQS; j=j+1)begin:READY_REG
    assign w_ready_reg[j] = (active_responses[j] == {NUM_SHARERS{1'b0}}) & active[j];
  end

//compare current address with active requests
//should be a one-hot signal because multiple active requests to the same
//memory block are rejected by the controller.
  for(j=0; j<ACTIVE_REQS; j=j+1)begin:CUR_MATCH
    assign w_cur_match[j] = (active_address[j] == cur_address) & active[j];
  end

//pick the active index
  for(j=0; j<NUM_SHARERS; j=j+1)begin:ACTIVE_INDEX
    //assign w_active_index[j] = ((w_active_sharers[cur_match_reg][j] == cur_src) &
    //cur_match & active_responses[cur_match_reg][j]) ? 1'b1 : 1'b0;
    assign w_active_index[j] = ((w_active_sharers[j] == cur_src) &
    cur_match & active_responses[cur_match_reg][j]) ? 1'b1 : 1'b0;
  end
  
endgenerate



// FSM
always @(posedge clock)begin
  if(reset & (state != RESET))begin
    for(i=0; i<ACTIVE_REQS; i=i+1)begin
      active[i]           <= 1'b0;
      active_req[i]       <= NoMsg;
      active_address[i]   <= {ADDRESS_BITS{1'b0}};
      active_data[i]      <= {CACHE_WIDTH{1'b0}};
      active_requestor[i] <= {SHARER_ID_BITS{1'b0}};
      active_sharers[i]   <= {DIR_WIDTH{1'b0}};
      active_responses[i] <= {NUM_SHARERS{1'b0}};
    end
    active_index       <= 0;
    dir_line           <= {DIR_WIDTH{1'b0}};
    dir_tag            <= {TAG_BITS{1'b0}};
    dir_way            <= {WAY_BITS{1'b0}};
    dir_hit            <= 1'b0;
    dir_modified       <= 1'b0;
    dir_valid          <= 1'b0;
    valid_response     <= 1'b0;
    r_intf_msg_out     <= NoMsg;
    r_intf_address_out <= {ADDRESS_BITS{1'b0}};
    r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
    r_intf_dest        <= DEFAULT_DEST;
    cur_msg            <= NoMsg;
    cur_address        <= {ADDRESS_BITS{1'b0}};
    cur_data           <= {CACHE_WIDTH{1'b0}};
    cur_src            <= {SHARER_ID_BITS{1'b0}};
    cache_read1        <= 1'b0;
    cache_write1       <= 1'b0;
    cache_invalidate1  <= 1'b0;
    cache_index1       <= {DIR_INDEX_BITS{1'b0}};
    cache_tag_in1      <= {TAG_BITS{1'b0}};
    cache_metadata1    <= 2'b00;
    cache_data_in1     <= {DIR_WIDTH{1'b0}};
    cache_way_select1  <= {WAY_BITS{1'b0}};
    handling_resp      <= 1'b0;
    read_memory        <= 1'b0;
    write_memory       <= 1'b0;
    clear_memreq       <= 1'b0;
    goto               <= 1'b0;
    goto_state         <= IDLE;
    r_reqbuf_read      <= 1'b0;
    r_respbuf_read     <= 1'b0;
    request_count      <= 0;
    recall_address     <= {ADDRESS_BITS{1'b0}};
    reset_counter      <= {DIR_INDEX_BITS{1'b0}};
    from_active        <= 1'b0;
    from_active_index  <= 0;
    state              <= RESET;
  end
  else begin
    case(state)
      RESET:begin
        if(reset_counter < DIR_DEPTH-1)begin
          reset_counter <= reset_counter + 1;
        end
        else if((reset_counter == DIR_DEPTH-1) & ~reset)begin
          reset_counter <= {DIR_INDEX_BITS{1'b0}};
          state         <= IDLE;
        end
        else
          state <= RESET;
      end
      IDLE:begin
        //unset signals to directory cache memory and memory interface FSM
        cache_index1      <= {DIR_INDEX_BITS{1'b0}};
        cache_data_in1    <= {DIR_WIDTH{1'b0}};
        cache_tag_in1     <= {TAG_BITS{1'b0}};
        cache_metadata1   <= 2'b00;
        cache_way_select1 <= {WAY_BITS{1'b0}};
        cache_write1      <= 1'b0;
        cache_read1       <= 1'b0;
        cache_invalidate1 <= 1'b0;
        clear_memreq      <= 1'b0;
        valid_response    <= 1'b0;
        from_active       <= 1'b0;
        if(~w_respbuf_empty)begin
          cur_msg        <= w_respbuf_data[BUF_WIDTH-SHARER_ID_BITS-1 -: MSG_BITS];
          cur_address    <= w_respbuf_data[CACHE_WIDTH +: ADDRESS_BITS];
          cur_data       <= w_respbuf_data[0 +: CACHE_WIDTH];
          cur_src        <= w_respbuf_data[BUF_WIDTH-1 -: SHARER_ID_BITS];
          handling_resp  <= 1'b1;
          r_respbuf_read <= 1'b1;
          state          <= DIR_ADDR;
        end
        else if(active_ready)begin //active request can be serviced.
          if(active_req[ready_reg] == Inv)begin //same encoding as REQ_FLUSH
            active[ready_reg] <= 1'b0;
            state             <= IDLE;
          end
          else begin
            cur_msg           <= active_req[ready_reg];
            cur_address       <= active_address[ready_reg];
            cur_data          <= active_data[ready_reg];
            cur_src           <= active_requestor[ready_reg];
            from_active       <= 1'b1; //one of the active requests are being serviced
            from_active_index <= ready_reg;
            state             <= DIR_ADDR;
          end
        end
        else if(~w_reqbuf_empty)begin //requests pending to be serviced.
          cur_msg        <= w_reqbuf_data[BUF_WIDTH-SHARER_ID_BITS-1 -: MSG_BITS];
          cur_address    <= w_reqbuf_data[CACHE_WIDTH +: ADDRESS_BITS];
          cur_data       <= w_reqbuf_data[0 +: CACHE_WIDTH];
          cur_src        <= w_reqbuf_data[BUF_WIDTH-1 -: SHARER_ID_BITS];
          r_reqbuf_read  <= 1'b1;
          state          <= DIR_ADDR;
        end
        else begin
          state <= IDLE;
        end
      end
      DIR_ADDR:begin
        r_reqbuf_read  <= 1'b0;
        r_respbuf_read <= 1'b0;
        //Address is sent to directory cache during this state
        //For GetM and GetS, also send a request to the memory interface
        if(cur_msg == GetS | cur_msg == GetM)
          read_memory <= 1'b1;
        //else if(cur_msg == PutM)
        else if(cur_msg == PutM | cur_msg == RespPutM)
          write_memory <= 1'b1;
        else begin
          read_memory  <= 1'b0;
          write_memory <= 1'b0;
        end
        //update active request tracking
        active_index   <= w_active_index_binary;
        valid_response <= w_active_index_valid;
        state <= DIR_READ;
      end
      DIR_READ:begin
        read_memory  <= 1'b0;
        write_memory <= 1'b0;
        //register readout from directory cache.
        dir_line     <= cache_data_out0;
        dir_tag      <= cache_tag_out0;
        dir_way      <= cache_matched_way0;
        dir_hit      <= cache_hit0;
        dir_modified <= cache_coh_bits0;
        dir_valid    <= cache_status_bits0;
        state        <= handling_resp ? HANDLE_RESP : READ_MEM;
      end
      HANDLE_RESP:begin
        //update active request tracking data
        //update dir$
        //respond if necessary
        handling_resp <= 1'b0;
        if(valid_response)begin
          case(cur_msg)
            PutE:begin
              //update active request tracking data
              //only one cache can respond with PutE. No other sharers to
              //check.
              active_responses[cur_match_reg] <= {NUM_SHARERS{1'b0}};
              //update dir$ -> Directory state does not have to be updated.
              //  respond if necessary -> PutE does not require a response at this 
              //  time. Cache goes form E->S. No cache to cache forwarding.
              state <= IDLE;
            end
            InvAck:begin
              //update active request tracking data
              active_responses[cur_match_reg][active_index] <= 1'b0;
              active_sharers[cur_match_reg][active_index]   <= DEFAULT_DEST;
              //update dir$
              cache_index1      <= dir_address[0 +: DIR_INDEX_BITS];
              cache_data_in1    <= w_write_line1;
              cache_tag_in1     <= dir_address[DIR_INDEX_BITS +: TAG_BITS];
              cache_metadata1   <= (sharers(dir_line) > 1) ? 2'b10 : 2'b00;
              cache_way_select1 <= dir_way;
              cache_write1      <= 1'b1;
              //respond if necessary -> not required
              state             <= IDLE;
            end
            RespPutM:begin
              //update active request tracking data
              //only one cache can respond with RespPutM. No other sharers to
              //check.
              active_responses[cur_match_reg] <= {NUM_SHARERS{1'b0}};
              cache_invalidate1  <= 1'b1;
              cache_index1       <= dir_address[0 +: DIR_INDEX_BITS];
              cache_tag_in1      <= dir_address[DIR_INDEX_BITS +: TAG_BITS];
              cache_metadata1    <= 2'b00;
              cache_way_select1  <= dir_way;
              state              <= WRITE_MEM;
            end
            NackC:begin
              //update active request tracking data -> no update
              //update dir$ -> request rejected. no update
              //respond if necessary -> resend request
              r_intf_msg_out     <= active_req[cur_match_reg]; 
              r_intf_address_out <= cur_address;
              r_intf_dest        <= cur_src;
              state              <= WAIT_INTF;
            end
            NackD:begin //cache does not have the cache line in question.
              //update active request tracking data
              active_responses[cur_match_reg][active_index] <= 1'b0;
              active_sharers[cur_match_reg][active_index*SHARER_ID_BITS +:
              SHARER_ID_BITS]    <= DEFAULT_DEST;
              //update dir$
              cache_index1       <= dir_address[0 +: DIR_INDEX_BITS];
              cache_data_in1     <= w_write_line1;
              cache_tag_in1      <= dir_address[DIR_INDEX_BITS +: TAG_BITS];
              cache_metadata1    <= 2'b00;
              cache_way_select1  <= dir_way;
              cache_write1       <= 1'b1;
              //respond if necessary -> not required
              state              <= IDLE;
            end
            //No default clause because this is fully specified.
          endcase
        end
        else begin //not expecting a response from this cache.
          cur_msg        <= NoMsg;
          cur_address    <= {ADDRESS_BITS{1'b0}};
          cur_data       <= {CACHE_WIDTH{1'b0}};
          cur_src        <= {SHARER_ID_BITS{1'b0}};
          active_index   <= 0;
          dir_line       <= {DIR_WIDTH{1'b0}};
          dir_way        <= {WAY_BITS{1'b0}};
          dir_hit        <= 1'b0;
          dir_valid      <= 1'b0;
          valid_response <= 1'b0;
          state          <= IDLE;
        end
      end
      READ_MEM:begin
        if(r_mem_msg_in != NO_REQ) //memory has responded
          state <= HANDLE_REQ;
        else
          state <= READ_MEM;
      end
      HANDLE_REQ:begin
        if(cur_match & ~from_active & (cur_msg != PutM))begin 
          //active request for the same memory block->reject new one
          //PutM should be allowed to go through because it is the writeback
          //correcponding to the active request.
          r_intf_msg_out     <= NackB; 
          r_intf_address_out <= cur_address;
          r_intf_dest        <= cur_src;
          state              <= WAIT_INTF;
        end
        else if(dir_hit)begin //there is a directory entry for this memory block.
          case(cur_msg)
            GetS:begin
              //check direcory status
              case({M, E, S, I})
                4'b1000:begin
                  if(~from_active & ~free_reg)begin //no free status registers
                    r_intf_msg_out     <= NackB; 
                    r_intf_address_out <= cur_address;
                    r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                    r_intf_dest        <= cur_src;
                    state              <= WAIT_INTF;
                  end
                  else if(from_active & ~free_reg)begin
                    clear_memreq <= 1'b1;
                    state        <= IDLE;
                  end
                  else begin
                  //send out the writeback request
                    r_intf_msg_out     <= FwdGetS; 
                    r_intf_address_out <= cur_address;
                    r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                    r_intf_dest        <= w_cache_line_slices[first_sharer(dir_line)];
                  //move the request to a status register
                    active[free_index]           <= 1'b1;
                    active_requestor[free_index] <= cur_src;
                    active_req[free_index]       <= cur_msg;
                    active_address[free_index]   <= cur_address;
                    active_data[free_index]      <= cur_data;
                    active_sharers[free_index]   <= dir_line;
                    active_responses[free_index] <= sharer_bits(dir_line);
                    state                        <= WAIT_INTF;
                  end
                end
                4'b0100:begin
                  if(from_active)begin //PutE has been received. send data to second sharer
                    state <= CHECK_DATA;
                  end
                  else begin //new request for a block in E state
                    if(~free_reg)begin
                      r_intf_msg_out     <= NackB; 
                      r_intf_address_out <= cur_address;
                      r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                      r_intf_dest        <= cur_src;
                      state              <= WAIT_INTF;
                    end
                    else begin
                    //send out the share request
                      r_intf_msg_out     <= FwdGetS; 
                      r_intf_address_out <= cur_address;
                      r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                      r_intf_dest        <= w_cache_line_slices[first_sharer(dir_line)];
                    //move the request to a status register
                      active[free_index]           <= 1'b1;
                      active_requestor[free_index] <= cur_src;
                      active_req[free_index]       <= cur_msg;
                      active_address[free_index]   <= cur_address;
                      active_data[free_index]      <= cur_data;
                      active_sharers[free_index]   <= dir_line;
                      active_responses[free_index] <= sharer_bits(dir_line);
                      state                        <= WAIT_INTF;
                    end
                  end 
                end
                4'b0010:begin
                  if(sharers(dir_line) == NUM_SHARERS)begin //no free slot
                    if(~from_active & ~free_reg)begin //no free status register
                      r_intf_msg_out     <= NackB; 
                      r_intf_address_out <= cur_address;
                      r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                      r_intf_dest        <= cur_src;
                      state              <= WAIT_INTF;
                    end
                    else if(from_active & ~free_reg)begin
                      clear_memreq <= 1'b1;
                      state        <= IDLE;
                    end
                    else begin
                      r_intf_msg_out     <= Inv;
                      r_intf_address_out <= cur_address;
                      r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                      r_intf_dest        <= w_cache_line_slices[first_sharer(dir_line)];
                      //change this later to recall a random sharer
                      active[free_index]           <= 1'b1;
                      active_requestor[free_index] <= cur_src;
                      active_req[free_index]       <= cur_msg;
                      active_address[free_index]   <= cur_address;
                      active_data[free_index]      <= cur_data;
                      active_sharers[free_index]   <= dir_line;
                      for(i=0; i<NUM_SHARERS; i=i+1)begin
                        if(i == first_sharer(dir_line))
                          active_responses[free_index][i] <= 1'b1;
                        else 
                          active_responses[free_index][i] <= 1'b0;
                      end
                      state <= WAIT_INTF;
                    end
                  end
                  else begin //free slot available
                    state <= CHECK_DATA;
                  end
                end
                4'b0001:begin
                  state <= CHECK_DATA;
                end
              endcase
            end
            GetM:begin
              if(I || (E & (w_cache_line_slices[first_sharer(dir_line)] == cur_src)))begin 
              //line is not cached by caches in the upper level or only cached
              //by the sharer requesting the line to modify.
                state <= CHECK_DATA;
              end
              else begin
                if(~from_active & ~free_reg)begin
                  r_intf_msg_out     <= NackB; 
                  r_intf_address_out <= cur_address;
                  r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                  r_intf_dest        <= cur_src;
                  state              <= WAIT_INTF;
                end
                else if(from_active & ~free_reg)begin
                  clear_memreq <= 1'b1;
                  state        <= IDLE;
                end
                else begin
                  //put current request in a status register
                  if(~from_active)begin
                    active[free_index]           <= 1'b1;
                    active_requestor[free_index] <= cur_src;
                    active_req[free_index]       <= cur_msg;
                    active_address[free_index]   <= cur_address;
                    active_data[free_index]      <= cur_data;
                    active_sharers[free_index]   <= {DIR_WIDTH{1'b0}};
                    active_responses[free_index] <= {NUM_SHARERS{1'b0}};
                  end
                  //remove current source from dir_line
                  dir_line <= remove_src(dir_line, cur_src);
                  state    <= START_RECALL;
                end
              end
            end
            PutM:begin
              //Assuming a inclusive cache or memory. 
              //Writeback cannot result in recall.
              r_intf_msg_out     <= PutAck;
              r_intf_address_out <= cur_address;
              r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
              r_intf_dest        <= cur_src;
              cache_invalidate1  <= 1'b1;
              cache_index1       <= dir_address[0 +: DIR_INDEX_BITS];
              cache_tag_in1      <= dir_address[DIR_INDEX_BITS +: TAG_BITS];
              cache_metadata1    <= 2'b00;
              cache_way_select1  <= dir_way;
              //update status register if there is an active request
              if(cur_match & valid_response)begin
                active_sharers[cur_match_reg][active_index*SHARER_ID_BITS   +:
                                            SHARER_ID_BITS]   <= DEFAULT_DEST;
                active_responses[cur_match_reg][active_index] <= 1'b0;
              end
              state              <= WAIT_INTF;
            end
            PutS:begin
              //update directory state
              cache_write1      <= 1'b1;
              cache_index1      <= dir_address[0 +: DIR_INDEX_BITS];
              cache_metadata1   <= 2'b10;
              cache_way_select1 <= dir_way;
              cache_tag_in1     <= dir_address[DIR_INDEX_BITS +: TAG_BITS];
              cache_data_in1    <= w_write_line3;
              //update status register if there is an active request
              if(cur_match & valid_response)begin
                active_sharers[cur_match_reg][active_index*SHARER_ID_BITS   +:
                                            SHARER_ID_BITS]   <= DEFAULT_DEST;
                active_responses[cur_match_reg][active_index] <= 1'b0;
              end
              //respond to the PutS with PutAck
              r_intf_msg_out     <= PutAck;
              r_intf_address_out <= cur_address;
              r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
              r_intf_dest        <= cur_src;
              state              <= WAIT_INTF;
            end
            default: state <= IDLE;
          endcase
        end
        else if(~dir_hit)begin //no directory entry for the memory block
          if(!dir_valid)begin //empty way in directory cache
            dir_line <= {NUM_SHARERS{DEFAULT_DEST}};
            state    <= CHECK_DATA;
          end
          else begin //no empty way -> recall
            if(from_active)begin
              if(~free_reg)begin
                clear_memreq <= 1'b1;
                state        <= IDLE;
              end
              else begin
                active[free_index]           <= 1'b1;
                active_requestor[free_index] <= DEFAULT_DEST;
                active_req[free_index]       <= Inv;
                active_address[free_index]   <= {dir_tag, dir_address[0 +: DIR_INDEX_BITS], 
                                                {OFFSET_BITS{1'b0}}};
                active_data[free_index]      <= {CACHE_WIDTH{1'b0}};
                active_sharers[free_index]   <= dir_line;
                active_responses[free_index] <= sharer_bits(dir_line);
                request_count                <= sharers(dir_line);
                recall_address               <= {dir_tag, dir_address[0 +: DIR_INDEX_BITS], 
                                                {OFFSET_BITS{1'b0}}};
                state                        <= SEND_RECALL;
              end
            end
            else begin
              if(~free_reg)begin
                r_intf_msg_out     <= NackB; 
                r_intf_address_out <= cur_address;
                r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
                r_intf_dest        <= cur_src;
                state              <= WAIT_INTF;
              end
              else begin
                active[free_index]           <= 1'b1;
                active_requestor[free_index] <= cur_src;
                active_req[free_index]       <= cur_msg;
                active_address[free_index]   <= cur_address;
                active_data[free_index]      <= {CACHE_WIDTH{1'b0}};
                active_sharers[free_index]   <= {DIR_WIDTH{1'b0}};
                active_responses[free_index] <= {NUM_SHARERS{1'b0}};
                state                        <= START_RECALL;
              end
            end
          end
        end
      end
      CHECK_DATA:begin
        if(r_mem_msg_in == MEM_RESP)begin //send data to requestor
          //update directory state
          cache_write1      <= 1'b1;
          cache_index1      <= dir_address[0 +: DIR_INDEX_BITS];
          cache_data_in1    <= (cur_msg == GetM) ? w_write_line4 : w_write_line2;
          cache_way_select1 <= dir_way;
          cache_metadata1   <= (cur_msg == GetM) ? 2'b11 : 2'b10;
          cache_tag_in1     <= dir_address[DIR_INDEX_BITS +: TAG_BITS];
          //send out data
          //r_intf_msg_out     <= (S || (E & from_active)) ? DataS : Data;
          r_intf_msg_out <= (I || (E & (cur_msg == GetM) & 
                            w_cache_line_slices[first_sharer(dir_line)] == cur_src)) ?
                            Data : DataS;
          r_intf_address_out <= cur_address;
          r_intf_data_out    <= r_mem_data_in;
          r_intf_dest        <= cur_src;
          if(from_active)
            active[from_active_index] <= 1'b0;
          state <= WAIT_INTF;
        end
        else if(r_mem_msg_in == REQ_FLUSH)begin
          if(from_active)begin
            //current request loaded from a status reg
            if(~free_reg)begin
              //cannot issue the RECALL now. Come back to this req later
              clear_memreq <= 1'b1;
              state        <= IDLE;
            end
            else begin
              //send address of the line recalled by the memory
              cache_read1   <= 1'b1;
              cache_index1  <= r_mem_address_in[OFFSET_BITS +: DIR_INDEX_BITS];
              cache_tag_in1 <= r_mem_address_in[ADDRESS_BITS-1 -: TAG_BITS];
              state         <= DIR_ACCESS2;
            end
          end
          else begin
            if(~free_reg)begin
              //no status reg available to send out the REQ_FLUSH or store the
              //current request -> NackB
              r_intf_msg_out     <= NackB; 
              r_intf_address_out <= cur_address;
              r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
              r_intf_dest        <= cur_src;
              state              <= WAIT_INTF;
            end
            else begin
              //first put the current request in a status register
              active[free_index]           <= 1'b1;
              active_requestor[free_index] <= cur_src;
              active_req[free_index]       <= cur_msg;
              active_address[free_index]   <= cur_address;
              active_data[free_index]      <= cur_data;
              active_sharers[free_index]   <= dir_line;
              active_responses[free_index] <= {NUM_SHARERS{1'b0}}; //not expecting 
                //responses for this block
              //send address of the line recalled by the memory to directory cache
              cache_read1   <= 1'b1;
              cache_index1  <= r_mem_address_in[OFFSET_BITS +: DIR_INDEX_BITS];
              cache_tag_in1 <= r_mem_address_in[ADDRESS_BITS-1 -: TAG_BITS];
              state         <= DIR_ACCESS2;
            end
          end
        end
      end
      START_RECALL:begin
        if(~free_reg)begin //cannot send Inv to the sharers
          clear_memreq <= 1'b1;
          state        <= IDLE;
        end
        else begin
          active[free_index]           <= 1'b1;
          active_requestor[free_index] <= DEFAULT_DEST;
          active_req[free_index]       <= Inv;
          active_address[free_index]   <= {dir_tag, dir_address[0 +: DIR_INDEX_BITS],
                                          {OFFSET_BITS{1'b0}}};
          active_data[free_index]      <= {CACHE_WIDTH{1'b0}};
          active_sharers[free_index]   <= dir_line;
          active_responses[free_index] <= sharer_bits(dir_line);
          recall_address               <= {dir_tag, dir_address[0 +: DIR_INDEX_BITS],
                                          {OFFSET_BITS{1'b0}}};
          request_count                <= sharers(dir_line);
          state                        <= SEND_RECALL; 
        end
      end
      DIR_ACCESS2:begin
        cache_read1 <= 1'b0;
        if(~free_reg)begin
        //no status reg available to send out the REQ_FLUSH -> goto IDLE,
        //handle original request later.
          clear_memreq <= 1'b1;
          state        <= IDLE;          
        end
        else begin
          active[free_index]           <= 1'b1;
          active_requestor[free_index] <= DEFAULT_DEST;
          active_req[free_index]       <= Inv;
          active_address[free_index]   <= r_mem_address_in;
          active_data[free_index]      <= {CACHE_WIDTH{1'b0}};
          active_sharers[free_index]   <= cache_data_out1;
          active_responses[free_index] <= sharer_bits(cache_data_out1);
          dir_line                     <= cache_data_out1;
          recall_address               <= r_mem_address_in;
          request_count                <= sharers(cache_data_out1);
          state                        <= SEND_RECALL; 
        end
      end
      SEND_RECALL:begin
        r_intf_msg_out                   <= Inv;
        r_intf_address_out               <= recall_address;
        r_intf_data_out                  <= {CACHE_WIDTH{1'b0}};
        r_intf_dest                      <= w_cache_line_slices[first_sharer(dir_line)];
        dir_line[SHARER_ID_BITS*first_sharer(dir_line) +: SHARER_ID_BITS] <= DEFAULT_DEST;
        if(request_count == 1)begin
          goto       <= 1'b0;
          goto_state <= IDLE;
        end
        else begin
          goto       <= 1'b1;
          goto_state <= SEND_RECALL;
        end
        request_count <= request_count - 1;
        state         <= WAIT_INTF;
      end
      WRITE_MEM:begin
        if(r_mem_msg_in == MEM_RESP)begin
          clear_memreq <= 1'b1;
          state <= IDLE;
        end
        else
          state <= WRITE_MEM;
      end
      WAIT_INTF:begin
        //unset signals to directory cache
        cache_index1      <= {DIR_INDEX_BITS{1'b0}};
        cache_data_in1    <= {DIR_WIDTH{1'b0}};
        cache_tag_in1     <= {TAG_BITS{1'b0}};
        cache_metadata1   <= 2'b00;
        cache_way_select1 <= {WAY_BITS{1'b0}};
        cache_write1      <= 1'b0;
        cache_read1       <= 1'b0;
        cache_invalidate1 <= 1'b0;
        valid_response    <= 1'b0;
        if(~intf_busy)begin
          //unset signals to the interface
          r_intf_msg_out     <= NoMsg; 
          r_intf_address_out <= {ADDRESS_BITS{1'b0}};
          r_intf_data_out    <= {CACHE_WIDTH{1'b0}};
          r_intf_dest        <= DEFAULT_DEST;
          if(goto)begin
            goto  <= 1'b0;
            state <= goto_state;
          end
          else begin
            clear_memreq <= 1'b1;
            state <= IDLE;
          end
        end
        else begin
          state <= WAIT_INTF;
        end
      end
      default:begin
        state <= IDLE;
      end
    endcase
  end
end


//state machine to interface with the memory interface. This will free up the
//main FSM from having to wait for memory to respond.
always @(posedge clock)begin
  if(reset)begin
    r_mem_msg_out     <= NoMsg;
    r_mem_address_out <= {ADDRESS_BITS{1'b0}};
    r_mem_data_out    <= {CACHE_WIDTH{1'b0}};
    r_mem_msg_in      <= NoMsg;
    r_mem_address_in  <= {ADDRESS_BITS{1'b0}};
    r_mem_data_in     <= {CACHE_WIDTH{1'b0}};
    mstate            <= M_IDLE;
  end
  else begin
    case(mstate)
      M_IDLE:begin
        if(read_memory)begin
          r_mem_msg_out     <= R_REQ;
          r_mem_address_out <= cur_address;
          mstate            <= M_READ;
        end
        else if(write_memory)begin
          r_mem_msg_out     <= WB_REQ;
          r_mem_address_out <= cur_address;
          r_mem_data_out    <= cur_data;
          mstate            <= M_WRITE;
        end
        else begin
          mstate <= M_IDLE;
        end
      end
      M_READ:begin
        if(mem_msg_in != NO_REQ)begin
          r_mem_msg_in      <= mem_msg_in;
          r_mem_address_in  <= mem_address_in;
          r_mem_data_in     <= mem_data_in;
          r_mem_msg_out     <= NO_REQ;
          r_mem_address_out <= 0;
          mstate            <= M_WAIT;
        end
        else 
          mstate <= M_READ;
      end
      M_WRITE:begin
        if(mem_msg_in != NO_REQ)begin
          r_mem_msg_in      <= mem_msg_in;
          r_mem_address_in  <= mem_address_in;
          r_mem_data_in     <= mem_data_in;
          r_mem_msg_out     <= NO_REQ;
          r_mem_address_out <= 0;
          r_mem_data_out    <= 0;
          mstate            <= M_WAIT;
        end
        else 
          mstate <= M_WRITE;
      end
      M_WAIT:begin
        if(clear_memreq)begin
          r_mem_msg_in     <= NoMsg;
          r_mem_address_in <= {ADDRESS_BITS{1'b0}};
          r_mem_data_in    <= {CACHE_WIDTH{1'b0}};
          mstate           <= M_IDLE;
        end
        else
          mstate <= M_WAIT;
      end
    endcase
  end
end



/*continuous assignments*/
//shifted address to track memory blocks in the directory. Can ignore the
//lower OFFSET_BITS bits because the directory tracks whole blocks.
assign dir_address = cur_address >> OFFSET_BITS;

assign mem_msg_out     = r_mem_msg_out;
assign mem_address_out = r_mem_address_out;
assign mem_data_out    = r_mem_data_out;

//always assign current address to port0 of directory cache.
assign cache_index0  = (state == RESET) ? reset_counter    : 
                       dir_address[0 +: DIR_INDEX_BITS];
assign cache_tag_in0 = (state == RESET) ? {TAG_BITS{1'b0}} : 
                       dir_address[DIR_INDEX_BITS +: TAG_BITS];
assign cache_read0   = (state == DIR_ADDR);
//ground unused connections to cache memory
assign cache_write0      = (state == RESET) ? 1'b1 : 1'b0;
assign cache_invalidate0 = 1'b0;
assign cache_metadata0   = 2'b00;
assign cache_way_select0 = {WAY_BITS{1'b0}};
assign cache_data_in0    = {DIR_WIDTH{1'b0}};

assign sharer_count   = sharers(dir_line);
assign free_reg_count = ACTIVE_REQS - ones(active);

//MESI flags for dir_line
assign M = sharer_count == 1 & dir_modified;
assign E = sharer_count == 1 & ~dir_modified;
assign S = sharer_count  > 1;
assign I = sharer_count == 0;

assign i_reset = reset | (state == RESET);


// Instantiate message handler
message_handler #(
  .CACHE_OFFSET_BITS(OFFSET_BITS),
  .DATA_WIDTH(DATA_WIDTH),
  .ADDRESS_BITS(ADDRESS_BITS),
  .MSG_BITS(MSG_BITS),
  .REQ_BUF_DEPTH_BITS(REQ_BUF_DEPTH_BITS),
  .RESP_BUF_DEPTH_BITS(RESP_BUF_DEPTH_BITS),
  .PARENT("DIRECTORY"),
  .ID_BITS(SHARER_ID_BITS),
  .DEFAULT_DEST(DEFAULT_DEST) //use own address as default destination.
) msg_handler (
  .clock(clock),
  .reset(reset),
//interface with the NoC
  .noc_msg_in(noc_msg_in),
  .noc_address_in(noc_address_in),
  .noc_data_in(noc_data_in),
  .noc_src_id(noc_src_id),
  .packetizer_busy(packetizer_busy),
  .noc_msg_out(noc_msg_out),
  .noc_address_out(noc_address_out),
  .noc_data_out(noc_data_out),
  .noc_dest_id(noc_dest_id),
//interface with controller of the parent module
  .ctrl_msg_in(r_intf_msg_out),
  .ctrl_address_in(r_intf_address_out),
  .ctrl_data_in(r_intf_data_out),
  .ctrl_dest_id(r_intf_dest),
  .intf_busy(intf_busy),
//exposed FIFO read interfaces
  .reqbuf_read(r_reqbuf_read),
  .respbuf_read(r_respbuf_read),
  .reqbuf_empty(w_reqbuf_empty),
  .reqbuf_full(w_reqbuf_full),
  .reqbuf_valid(w_reqbuf_valid),
  .respbuf_empty(w_respbuf_empty),
  .respbuf_full(w_respbuf_full),
  .respbuf_valid(w_respbuf_valid),
  .reqbuf_data(w_reqbuf_data),
  .respbuf_data(w_respbuf_data)
);


//instantiate directory cache
cache_memory #(
  .STATUS_BITS(1), //valid bit only
  .COHERENCE_BITS(1),
  .OFFSET_BITS(0),
  .DATA_WIDTH(DIR_WIDTH),
  .NUMBER_OF_WAYS(DIR_WAYS),
  .REPLACEMENT_MODE(1'b0), //LRU
  .ADDRESS_BITS(ADDRESS_BITS),
  .INDEX_BITS(DIR_INDEX_BITS),
  .TAG_BITS(TAG_BITS)
) dir_cache (
  .clock(clock),
  .reset(i_reset),
//port 0
  .read0(cache_read0),
  .write0(cache_write0),
  .invalidate0(cache_invalidate0),
  .index0(cache_index0),
  .tag0(cache_tag_in0),
  .meta_data0(cache_metadata0),
  .data_in0(cache_data_in0),
  .way_select0(cache_way_select0),
  .data_out0(cache_data_out0),
  .tag_out0(cache_tag_out0),
  .matched_way0(cache_matched_way0),
  .coh_bits0(cache_coh_bits0), //modified bit
  .status_bits0(cache_status_bits0),
  .hit0(cache_hit0),
//port 1
  .read1(cache_read1),
  .write1(cache_write1),
  .invalidate1(cache_invalidate1),
  .index1(cache_index1),
  .tag1(cache_tag_in1),
  .meta_data1(cache_metadata1),
  .data_in1(cache_data_in1),
  .way_select1(cache_way_select1),
  .data_out1(cache_data_out1),
  .tag_out1(cache_tag_out1),
  .matched_way1(cache_matched_way1),
  .coh_bits1(cache_coh_bits1), //modified bit
  .status_bits1(cache_status_bits1),
  .hit1(cache_hit1),

  .report(report)
);


//instantiate priority encoder find the next empty status register
priority_encoder #(
  .WIDTH(ACTIVE_REQS),
  .PRIORITY("LSB")
) free_reg_selector (
  .decode(~active),
  .encode(free_index),
  .valid(free_reg)
);

//instantiate arbiter to select the next status register ready to be serviced
arbiter #(
  .WIDTH(ACTIVE_REQS),
  .ARB_TYPE("PACKET")
) ready_reg_selector (
  .clock(clock),
  .reset(reset),
  .requests(w_ready_reg),
  .grant(ready_reg),
  .valid(active_ready)
);

//instantiate one-hot decoder to find the active request matching current
//address
one_hot_decoder #(
  .WIDTH(ACTIVE_REQS)
) cur_match_selector (
  .encoded(w_cur_match),
  .decoded(cur_match_reg),
  .valid(cur_match)
);

//instantiate one-hot decoder to find the index of the pointer which matches
//the current scr
one_hot_decoder #(
  .WIDTH(NUM_SHARERS)
) active_index_selector (
  .encoded(w_active_index),
  .decoded(w_active_index_binary),
  .valid(w_active_index_valid)
);

endmodule
