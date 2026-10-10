// cmd_fsm.v
// UART command interpreter for the memory trial.
//   'W' addr data        write one byte            -> 'K'
//   'R' addr             read one byte             -> data
//   'P' addr n d0..dn-1  page write, n bytes       -> 'K'   (address wraps at 64)
//   'G' addr n           page read, n bytes        -> n data bytes
//   anything else                                  -> '?'

`default_nettype none

module cmd_fsm (
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
    localparam [3:0] S_IDLE = 4'd0, S_W_ADDR = 4'd1, S_W_DATA = 4'd2, S_P_ADDR = 4'd3,
                     S_P_LEN = 4'd4, S_P_DATA = 4'd5, S_P_NEXT = 4'd6, S_R_ADDR = 4'd7,
                     S_G_ADDR = 4'd8, S_G_LEN = 4'd9, S_RD_SEND = 4'd10, S_RD_GAP = 4'd11,
                     S_REPLY = 4'd12;

    // Current state.
    reg [3:0] state;
    // Bytes left in a page transfer.
    reg [6:0] remaining;
    // Reply byte for single-byte replies.
    reg [7:0] reply;
    // Cycles waited after starting a transmission.
    reg [1:0] gap;

    // This always block is the command state machine.
    always @(posedge clk or posedge rst) begin
        // Asynchronous reset puts everything in its idle state.
        if (rst) begin
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
                    mem_wdata <= rx_data;
                    mem_we    <= 1'b1;
                    reply     <= 8'h4B; // 'K'
                    state     <= S_REPLY;
                end
                // Page write: start address.
                S_P_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[5:0];
                    state    <= S_P_LEN;
                end
                // Page write: byte count (0 means nothing to write).
                S_P_LEN: if (rx_valid) begin
                    remaining <= rx_data[6:0];
                    reply     <= 8'h4B; // 'K'
                    state     <= (rx_data[6:0] == 7'd0) ? S_REPLY : S_P_DATA;
                end
                // Page write: one data byte, strobe the memory.
                S_P_DATA: if (rx_valid) begin
                    mem_wdata <= rx_data;
                    mem_we    <= 1'b1;
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
                // Any other state code returns to idle.
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
