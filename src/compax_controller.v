// compax_controller.v
// UART controller for raw storage and fixed-slot BDI compression.
//   'W' addr data        write one byte            -> 'K'
//   'R' addr             read one byte             -> data
//   'P' addr n d0..dn-1  page write, n bytes       -> 'K'   (address wraps at 64)
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
    output reg  [5:0] mem_addr,  // memory address for both ports
    output reg  [7:0] mem_wdata, // memory write data
    input  wire [7:0] mem_rdata  // memory read data
);
    // State encodings.
    localparam [4:0] S_IDLE = 5'd0, S_W_ADDR = 5'd1, S_W_DATA = 5'd2, S_P_ADDR = 5'd3,
                     S_P_LEN = 5'd4, S_P_DATA = 5'd5, S_P_NEXT = 5'd6, S_R_ADDR = 5'd7,
                     S_G_ADDR = 5'd8, S_G_LEN = 5'd9, S_RD_SEND = 5'd10, S_RD_GAP = 5'd11,
                     S_REPLY = 5'd12, S_BLOCK_SLOT = 5'd13, S_BLOCK_FETCH = 5'd14,
                     S_BLOCK_TRANSFORM = 5'd15, S_BLOCK_WRITE = 5'd16, S_BLOCK_STEP = 5'd17,
                     S_META_SEND = 5'd18, S_META_GAP = 5'd19;

    // Current state.
    reg [4:0] state;
    // Bytes left in a page transfer.
    reg [6:0] remaining;
    // Reply byte for single-byte replies.
    reg [7:0] reply;
    // Cycles waited after starting a transmission.
    reg [1:0] gap;

    reg [127:0] block_buffer;
    wire [127:0] encoded_payload, decoded_block;
    wire [2:0] encoded_tag;
    wire [3:0] encoded_mask;
    reg [2:0] block_tag [0:3];
    reg [3:0] block_mask [0:3];
    reg [1:0] slot;
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
    bdi_encoder u_encoder(.block_in(block_buffer), .payload(encoded_payload),
        .tag(encoded_tag), .immediate_mask(encoded_mask));
    bdi_decoder u_decoder(.payload(block_buffer), .tag(block_tag[slot]),
        .immediate_mask(block_mask[slot]), .block_out(decoded_block));

    // This always block is the command state machine.
    always @(posedge clk or posedge rst) begin
        // Asynchronous reset puts everything in its idle state.
        if (rst) begin
            reject_page_write <= 1'b0;
            slot <= 2'd0; byte_index <= 4'd0; operation <= 8'd0; meta_length <= 8'd0;
            for (reset_index=0; reset_index<4; reset_index=reset_index+1) begin
                block_tag[reset_index] <= 3'd0; block_mask[reset_index] <= 4'd0;
            end
            state     <= S_IDLE;
            remaining <= 7'd0;
            reply     <= 8'd0;
            gap       <= 2'd0;
            tx_data   <= 8'd0;
            tx_start  <= 1'b0;
            mem_we    <= 1'b0;
            mem_addr  <= 6'd0;
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
                    mem_addr <= rx_data[5:0];
                    state    <= S_W_DATA;
                end
                // Single write: data byte, then strobe the memory.
                S_W_DATA: if (rx_valid) begin
                    if (block_tag[mem_addr[5:4]] != 3'd0) reply<=8'h45; // E: decompress first
                    else begin mem_wdata<=rx_data; mem_we<=1'b1; reply<=8'h4B; end
                    state<=S_REPLY;
                end
                // Page write: start address.
                S_P_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[5:0];
                    state    <= S_P_LEN;
                end
                // Page write: byte count (0 means nothing to write).
                S_P_LEN: if (rx_valid) begin
                    remaining <= rx_data[6:0];
                    reject_page_write <= (block_tag[0]!=3'd0 || block_tag[1]!=3'd0 || block_tag[2]!=3'd0 || block_tag[3]!=3'd0);
                    reply <= (rx_data[6:0]!=7'd0 && (block_tag[0]!=3'd0 || block_tag[1]!=3'd0 || block_tag[2]!=3'd0 || block_tag[3]!=3'd0)) ? 8'h45 : 8'h4B;
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
                    mem_addr <= mem_addr + 6'd1;
                    state    <= (remaining == 7'd0) ? S_REPLY : S_P_DATA;
                end
                // Single read: address byte, then send one byte.
                S_R_ADDR: if (rx_valid) begin
                    mem_addr  <= rx_data[5:0];
                    remaining <= 7'd1;
                    state     <= S_RD_SEND;
                end
                // Page read: start address.
                S_G_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[5:0];
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
                            mem_addr <= mem_addr + 6'd1;
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
                // C/D/M take a slot index 0..3, not a byte address.
                S_BLOCK_SLOT: if (rx_valid) begin
                    if (rx_data > 8'd3) begin reply<=8'h3F; state<=S_REPLY; end
                    else begin
                        slot<=rx_data[1:0]; byte_index<=4'd0;
                        mem_addr<={rx_data[1:0],4'd0}; reply<=8'h4B;
                        if (operation==8'h4D) begin
                            tx_data<={1'b0,block_mask[rx_data[1:0]],block_tag[rx_data[1:0]]};
                            meta_length<=payload_length(block_tag[rx_data[1:0]]);
                            state<=S_META_SEND;
                        end else if ((operation==8'h43 && block_tag[rx_data[1:0]]!=3'd0) ||
                                     (operation==8'h44 && block_tag[rx_data[1:0]]==3'd0)) state<=S_REPLY;
                        else state<=S_BLOCK_FETCH;
                    end
                end
                S_BLOCK_FETCH: begin
                    block_buffer[byte_index*8 +: 8]<=mem_rdata;
                    if (byte_index==4'd15) state<=S_BLOCK_TRANSFORM;
                    else begin byte_index<=byte_index+4'd1; mem_addr<=mem_addr+6'd1; end
                end
                S_BLOCK_TRANSFORM: begin
                    if (operation==8'h43) begin
                        block_buffer<=encoded_payload;
                        block_tag[slot]<=encoded_tag; block_mask[slot]<=encoded_mask;
                    end else begin
                        block_buffer<=decoded_block;
                        block_tag[slot]<=3'd0; block_mask[slot]<=4'd0;
                    end
                    byte_index<=4'd0; mem_addr<={slot,4'd0}; state<=S_BLOCK_WRITE;
                end
                // Write all 16 bytes: padding is zeroed for compressed formats.
                S_BLOCK_WRITE: begin
                    mem_wdata<=block_buffer[byte_index*8 +: 8]; mem_we<=1'b1;
                    state<=S_BLOCK_STEP;
                end
                S_BLOCK_STEP: begin
                    if (byte_index==4'd15) state<=S_REPLY;
                    else begin byte_index<=byte_index+4'd1; mem_addr<=mem_addr+6'd1; state<=S_BLOCK_WRITE; end
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
