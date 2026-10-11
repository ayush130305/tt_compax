// compax_controller.v
// UART controller for raw storage and fixed-slot BDI compression.
//   'W' addr data        write one byte            -> 'K'
//   'R' addr             read one byte             -> data
//   'P' addr n d0..dn-1  page write, n bytes       -> 'K'   (address wraps at 32)
//   'G' addr n           page read, n bytes        -> n data bytes
//   anything else                                  -> '?'

`default_nettype none

module compax_controller (
    input  wire       clk,       // system clock
    input  wire       rst,       // active-high asynchronous reset
    input  wire [7:0] rx_data,   // received byte
    input  wire       rx_valid,  // received byte strobe
    output reg  [7:0] tx_data,   // byte to transmit
    output reg        tx_start,  // transmit strobe
    input  wire       tx_busy,   // transmitter busy
    output reg        mem_we,    // memory write strobe
    output reg  [4:0] mem_addr,  // memory address for both ports
    output reg  [7:0] mem_wdata, // memory write data
    input  wire [7:0] mem_rdata  // memory read data
);
    // State encodings.
    localparam [4:0] S_IDLE = 5'd0, S_W_ADDR = 5'd1, S_W_DATA = 5'd2, S_P_ADDR = 5'd3,
                     S_P_LEN = 5'd4, S_P_DATA = 5'd5, S_P_NEXT = 5'd6, S_R_ADDR = 5'd7,
                     S_G_ADDR = 5'd8, S_G_LEN = 5'd9, S_RD_SEND = 5'd10, S_RD_GAP = 5'd11,
                     S_REPLY = 5'd12, S_BLOCK_SLOT = 5'd13, S_BLOCK_FETCH = 5'd14,
                     S_BLOCK_TRANSFORM = 5'd15, S_BLOCK_WRITE = 5'd16, S_BLOCK_STEP = 5'd17,
                     S_META_SEND = 5'd18, S_META_GAP = 5'd19, S_SCAN = 5'd20,
                     S_CHECK = 5'd21, S_PACK = 5'd22, S_UNPACK = 5'd23, S_CODEC_FINISH = 5'd24;

    // Current state.
    reg [4:0] state;
    // Bytes left in a page transfer.
    reg [6:0] remaining;
    // Reply byte for single-byte replies.
    reg [7:0] reply;
    // Cycles waited after starting a transmission.
    reg [1:0] gap;

    reg [127:0] block_buffer;
    reg [2:0] block_tag [0:1];
    reg [3:0] block_mask [0:1];
    reg slot;
    reg [3:0] byte_index;
    reg [7:0] operation;
    reg [7:0] meta_length;
    reg reject_page_write;
    integer reset_index;
    function [7:0] payload_length;
        input [2:0] mode;
        begin
            case(mode)
                3'd1: payload_length=8'd0;
                3'd2: payload_length=8'd4;
                3'd3: payload_length=8'd8;
                3'd4: payload_length=8'd12;
                default: payload_length=8'd16;
            endcase
        end
    endfunction
    // One 32-bit datapath is reused for both delta subtraction and reconstruction.
    reg [1:0] word_index;
    reg wide_delta, found_base, all_zero, all_repeated, all_fit;
    reg [31:0] base_value, word_cursor;
    reg [3:0] candidate_mask;
    reg [2:0] candidate_tag;
    wire [31:0] selected_word = block_buffer[{word_index,5'b00000} +: 32];
    wire [31:0] alu_word = (state==S_PACK || state==S_UNPACK) ? word_cursor : selected_word;
    wire immediate_fit = wide_delta ?
        (alu_word=={{16{alu_word[15]}},alu_word[15:0]}) :
        (alu_word=={{24{alu_word[7]}},alu_word[7:0]});
    wire decoding = (state==S_UNPACK);
    wire use_zero_base = decoding ? block_mask[slot][word_index] : immediate_fit;
    wire [31:0] alu_base = use_zero_base ? 32'd0 : base_value;
    wire [31:0] alu_result;
    wire delta_fit = wide_delta ?
        (alu_result=={{16{alu_result[15]}},alu_result[15:0]}) :
        (alu_result=={{24{alu_result[7]}},alu_result[7:0]});
    wire [1:0] next_word_index = word_index + 2'd1;
    wire [1:0] previous_word_index = word_index - 2'd1;
    wire [7:0] previous_delta8 = block_buffer[7'd32+{2'b00,previous_word_index,3'b000} +: 8];
    wire [15:0] previous_delta16 = block_buffer[7'd32+{1'b0,previous_word_index,4'b0000} +: 16];
    bdi_word_alu u_codec_alu (.word_value(alu_word), .base_value(alu_base),
        .subtract(!decoding), .result(alu_result));

    // This always block is the command state machine.
    always @(posedge clk or posedge rst) begin
        // Asynchronous reset puts everything in its idle state.
        if (rst) begin
            word_index<=2'd0; wide_delta<=1'b0; found_base<=1'b0;
            all_zero<=1'b0; all_repeated<=1'b0; all_fit<=1'b0;
            candidate_mask<=4'd0; candidate_tag<=3'd0;
            // Datapath registers are loaded before use and need no reset.
            reject_page_write <= 1'b0;
            slot <= 2'd0; byte_index <= 4'd0; operation <= 8'd0; meta_length <= 8'd0;
            for (reset_index=0; reset_index<2; reset_index=reset_index+1) begin
                block_tag[reset_index] <= 3'd0; block_mask[reset_index] <= 4'd0;
            end
            state     <= S_IDLE;
            remaining <= 7'd0;
            reply     <= 8'd0;
            gap       <= 2'd0;
            tx_data   <= 8'd0;
            tx_start  <= 1'b0;
            mem_we    <= 1'b0;
            mem_addr  <= 5'd0;
            mem_wdata <= 8'd0;
        end else begin
            // Strobes last one cycle by default.
            tx_start <= 1'b0;
            mem_we   <= 1'b0;

            // Dispatch on the current state.
            case (state)
                // Wait for a command byte.
                S_IDLE: begin
                    // Only act when a byte has arrived.
                    if (rx_valid) begin
                        // Pick the command by its ASCII code.
                        case (rx_data)
                            8'h43, 8'h44, 8'h4D: begin operation<=rx_data; state<=S_BLOCK_SLOT; end
                            8'h57:   state <= S_W_ADDR;                       // 'W'
                            8'h52:   state <= S_R_ADDR;                       // 'R'
                            8'h50:   state <= S_P_ADDR;                       // 'P'
                            8'h47:   state <= S_G_ADDR;                       // 'G'
                            default: begin reply <= 8'h3F; state <= S_REPLY; end // '?'
                        endcase
                    end
                end
                // Single write: address byte.
                S_W_ADDR: if (rx_valid) begin
                    // Low six bits address the memory.
                    mem_addr <= rx_data[4:0];
                    state    <= S_W_DATA;
                end
                // Single write: data byte, then strobe the memory.
                S_W_DATA: if (rx_valid) begin
                    if (block_tag[mem_addr[4]] != 3'd0) reply<=8'h45; // E: decompress first
                    else begin mem_wdata<=rx_data; mem_we<=1'b1; reply<=8'h4B; end
                    state<=S_REPLY;
                end
                // Page write: start address.
                S_P_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[4:0];
                    state    <= S_P_LEN;
                end
                // Page write: byte count (0 means nothing to write).
                S_P_LEN: if (rx_valid) begin
                    remaining <= rx_data[6:0];
                    reject_page_write <= (block_tag[0]!=3'd0 || block_tag[1]!=3'd0);
                    reply <= (rx_data[6:0]!=7'd0 && (block_tag[0]!=3'd0 || block_tag[1]!=3'd0)) ? 8'h45 : 8'h4B;
                    state     <= (rx_data[6:0] == 7'd0) ? S_REPLY : S_P_DATA;
                end
                // Page write: one data byte, strobe the memory.
                S_P_DATA: if (rx_valid) begin
                    mem_wdata <= rx_data;
                    mem_we    <= !reject_page_write;
                    remaining <= remaining - 7'd1;
                    state     <= S_P_NEXT;
                end
                // Page write: advance the address after the write strobe cycle.
                S_P_NEXT: begin
                    mem_addr <= mem_addr + 5'd1;
                    state    <= (remaining == 7'd0) ? S_REPLY : S_P_DATA;
                end
                // Single read: address byte, then send one byte.
                S_R_ADDR: if (rx_valid) begin
                    mem_addr  <= rx_data[4:0];
                    remaining <= 7'd1;
                    state     <= S_RD_SEND;
                end
                // Page read: start address.
                S_G_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[4:0];
                    state    <= S_G_LEN;
                end
                // Page read: byte count.
                S_G_LEN: if (rx_valid) begin
                    remaining <= rx_data[6:0];
                    state     <= (rx_data[6:0] == 7'd0) ? S_IDLE : S_RD_SEND;
                end
                // Send the addressed byte once the transmitter is free.
                S_RD_SEND: if (!tx_busy) begin
                    tx_data   <= mem_rdata;
                    tx_start  <= 1'b1;
                    remaining <= remaining - 7'd1;
                    gap       <= 2'd0;
                    state     <= S_RD_GAP;
                end
                // Let the transmitter raise busy, then continue or finish.
                S_RD_GAP: begin
                    gap <= gap + 2'd1;
                    if (gap == 2'd2) begin
                        // Last byte done, or step to the next address.
                        if (remaining == 7'd0) state <= S_IDLE;
                        else begin
                            mem_addr <= mem_addr + 5'd1;
                            state    <= S_RD_SEND;
                        end
                    end
                end
                // Send the single reply byte once the transmitter is free.
                S_REPLY: if (!tx_busy) begin
                    tx_data  <= reply;
                    tx_start <= 1'b1;
                    state    <= S_IDLE;
                end
                // C/D/M take a slot index 0..1, not a byte address.
                S_BLOCK_SLOT: if (rx_valid) begin
                    if (rx_data > 8'd1) begin reply<=8'h3F; state<=S_REPLY; end
                    else begin
                        slot<=rx_data[0]; byte_index<=4'd0;
                        mem_addr<={rx_data[0],4'd0}; reply<=8'h4B;
                        if (operation==8'h4D) begin
                            tx_data<={1'b0,block_mask[rx_data[0]],block_tag[rx_data[0]]};
                            meta_length<=payload_length(block_tag[rx_data[0]]);
                            state<=S_META_SEND;
                        end else if ((operation==8'h43 && block_tag[rx_data[0]]!=3'd0) ||
                                     (operation==8'h44 && block_tag[rx_data[0]]==3'd0)) state<=S_REPLY;
                        else state<=S_BLOCK_FETCH;
                    end
                end
                S_BLOCK_FETCH: begin
                    block_buffer[byte_index*8 +: 8]<=mem_rdata;
                    if (byte_index==4'd15) state<=S_BLOCK_TRANSFORM;
                    else begin byte_index<=byte_index+4'd1; mem_addr<=mem_addr+5'd1; end
                end
                S_BLOCK_TRANSFORM: begin
                    word_index<=2'd0; wide_delta<=1'b0; found_base<=1'b0;
                    base_value<=block_buffer[31:0]; candidate_mask<=4'd0;
                    all_zero<=1'b1; all_repeated<=1'b1; all_fit<=1'b1;
                    if (operation==8'h43) state<=S_SCAN;
                    else begin
                        case(block_tag[slot])
                            3'd1: begin block_buffer<=128'd0; state<=S_CODEC_FINISH; end
                            3'd2: begin block_buffer<={4{block_buffer[31:0]}}; state<=S_CODEC_FINISH; end
                            default: begin
                                word_index<=2'd3; wide_delta<=(block_tag[slot]==3'd4);
                                word_cursor<=(block_tag[slot]==3'd4) ?
                                    {{16{block_buffer[95]}},block_buffer[95:80]} :
                                    {{24{block_buffer[63]}},block_buffer[63:56]};
                                state<=S_UNPACK;
                            end
                        endcase
                    end
                end
                // Scan one word each cycle, choosing the first non-immediate base.
                S_SCAN: begin
                    if (selected_word!=32'd0) all_zero<=1'b0;
                    if (selected_word!=block_buffer[31:0]) all_repeated<=1'b0;
                    if (!found_base && !immediate_fit) begin base_value<=selected_word; found_base<=1'b1; end
                    if (word_index==2'd3) begin word_index<=2'd0; state<=S_CHECK; end
                    else word_index<=next_word_index;
                end
                // Test all deltas before changing the input buffer.
                S_CHECK: begin
                    candidate_mask[word_index]<=immediate_fit;
                    if (!delta_fit) all_fit<=1'b0;
                    if (all_zero) begin block_buffer<=128'd0; candidate_tag<=3'd1; candidate_mask<=4'd0; state<=S_CODEC_FINISH; end
                    else if (all_repeated) begin block_buffer[127:32]<=96'd0; candidate_tag<=3'd2; candidate_mask<=4'd0; state<=S_CODEC_FINISH; end
                    else if (word_index==2'd3) begin
                        if (all_fit && delta_fit) begin
                            candidate_tag<=wide_delta ? 3'd4 : 3'd3;
                            word_index<=2'd0; word_cursor<=block_buffer[31:0]; state<=S_PACK;
                        end else if (!wide_delta) begin
                            wide_delta<=1'b1; found_base<=1'b0; base_value<=block_buffer[31:0];
                            word_index<=2'd0; all_fit<=1'b1; candidate_mask<=4'd0; state<=S_SCAN;
                        end else begin candidate_tag<=3'd0; candidate_mask<=4'd0; state<=S_CODEC_FINISH; end
                    end else word_index<=next_word_index;
                end
                // Capture the next source word before overlapping compacted writes.
                S_PACK: begin
                    if (wide_delta) block_buffer[7'd32+{1'b0,word_index,4'b0000} +: 16]<=alu_result[15:0];
                    else block_buffer[7'd32+{2'b00,word_index,3'b000} +: 8]<=alu_result[7:0];
                    word_cursor<=block_buffer[{next_word_index,5'b00000} +: 32];
                    if (word_index==2'd3) state<=S_CODEC_FINISH;
                    else word_index<=next_word_index;
                end
                // Expand backwards; save the next delta before an overlapping write.
                S_UNPACK: begin
                    block_buffer[{word_index,5'b00000} +: 32]<=alu_result;
                    word_cursor<=wide_delta ? {{16{previous_delta16[15]}},previous_delta16} :
                                             {{24{previous_delta8[7]}},previous_delta8};
                    if (word_index==2'd0) state<=S_CODEC_FINISH;
                    else word_index<=previous_word_index;
                end
                S_CODEC_FINISH: begin
                    if (operation==8'h43) begin
                        block_tag[slot]<=candidate_tag; block_mask[slot]<=candidate_mask;
                        if (candidate_tag==3'd3 || candidate_tag==3'd4) begin
                            block_buffer[31:0]<=base_value;
                            if (candidate_tag==3'd3) block_buffer[127:64]<=64'd0;
                            else block_buffer[127:96]<=32'd0;
                        end
                    end else begin block_tag[slot]<=3'd0; block_mask[slot]<=4'd0; end
                    byte_index<=4'd0; mem_addr<={slot,4'd0}; state<=S_BLOCK_WRITE;
                end
                // Write all 16 bytes: padding is zeroed for compressed formats.
                S_BLOCK_WRITE: begin
                    mem_wdata<=block_buffer[byte_index*8 +: 8]; mem_we<=1'b1;
                    state<=S_BLOCK_STEP;
                end
                S_BLOCK_STEP: begin
                    if (byte_index==4'd15) state<=S_REPLY;
                    else begin byte_index<=byte_index+4'd1; mem_addr<=mem_addr+5'd1; state<=S_BLOCK_WRITE; end
                end
                S_META_SEND: if (!tx_busy) begin
                    tx_start<=1'b1; gap<=2'd0; state<=S_META_GAP;
                end
                S_META_GAP: begin
                    gap<=gap+2'd1;
                    if (gap==2'd2) begin reply<=meta_length; state<=S_REPLY; end
                end
                // Any other state code returns to idle.
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
