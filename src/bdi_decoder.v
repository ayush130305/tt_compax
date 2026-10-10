`default_nettype none
module bdi_decoder (
    input wire [127:0] payload,
    input wire [2:0] tag,
    input wire [3:0] immediate_mask,
    output reg [127:0] block_out
);
    integer i;
    reg [31:0] delta_value, base_value;
    reg [7:0] delta8;
    reg [15:0] delta16;
    always @* begin
        block_out=payload; delta_value=32'd0; base_value=32'd0;
        delta8=8'd0; delta16=16'd0;
        case (tag)
            3'd1: block_out=128'd0;
            3'd2: block_out={4{payload[31:0]}};
            3'd3,3'd4: begin
                for (i=0; i<4; i=i+1) begin
                    delta8=payload[32+i*8 +: 8];
                    delta16=payload[32+i*16 +: 16];
                    delta_value=(tag==3'd3) ? {{24{delta8[7]}},delta8} : {{16{delta16[15]}},delta16};
                    base_value=immediate_mask[i] ? 32'd0 : payload[31:0];
                    block_out[i*32 +: 32]=base_value+delta_value;
                end
            end
            default: block_out=payload;
        endcase
    end
endmodule
`default_nettype wire
