// latch_mem.v
// 64 x 8 latch-based memory. Each row is 8 transparent latches opened by a gated clock.
// Write protocol: present we/waddr/wdata during cycle N; the row is written during the high
// phase of cycle N+1 (so it can be read from cycle N+2). Reads are combinational.

`default_nettype none

module latch_mem #(
    parameter integer DEPTH = 64, // number of rows
    parameter integer AW    = 6   // address width, clog2(DEPTH)
) (
    input  wire          clk,   // system clock
    input  wire          we,    // write enable, valid for one cycle
    input  wire [AW-1:0] waddr, // write address
    input  wire [7:0]    wdata, // write data
    input  wire [AW-1:0] raddr, // read address
    output wire [7:0]    rdata  // read data (combinational)
);
    // Write data captured at the rising edge, so it is stable during the next high phase.
    reg [7:0] wd_q;

    // This always block captures the write data on every clock edge.
    always @(posedge clk) begin
        // Take the data the controller is presenting this cycle.
        wd_q <= wdata;
    end

    // Storage rows.
    reg [7:0] mem [0:DEPTH-1];

    // Generate variable for the row loop.
    genvar i;

    // This generate builds one enable latch, one clock gate and one 8-bit latch row per address.
    generate
        for (i = 0; i < DEPTH; i = i + 1) begin : g_row
            // Enable latch, transparent while clk is low (standard clock-gating structure).
            reg en_lat;
            // This always block holds the row enable stable through the high phase of clk.
            always @* begin
                // Follow the decoded write request only while clk is low.
                if (!clk) en_lat = we & (waddr == i);
            end
            // Gated clock for this row: high only in the high phase after a write request.
            wire gate = clk & en_lat;
            // This always block is the 8 storage latches of the row.
            always @* begin
                // Transparent while the gate is open, otherwise holds.
                if (gate) mem[i] = wd_q;
            end
        end
    endgenerate

    // Combinational read port.
    assign rdata = mem[raddr];
endmodule

`default_nettype wire
