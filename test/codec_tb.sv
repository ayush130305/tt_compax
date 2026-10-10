`timescale 1ns/1ps
module codec_tb;
reg [127:0] original;
wire [127:0] encoded, restored;
wire [2:0] tag;
wire [3:0] mask;
bdi_encoder e(original,encoded,tag,mask);
bdi_decoder d(encoded,tag,mask,restored);
integer i,j; reg [31:0] base, value;
task check; begin #1; if(restored!==original) $fatal(1,"codec mismatch %h tag %d mask %h",original,tag,mask); end endtask
initial begin
original=128'd0;check();if(tag!=1)$fatal;
original={4{32'h12345678}};check();if(tag!=2)$fatal;
original={32'd7,32'hffffff80,32'h1000007f,32'h10000000};check();if(tag!=3 || mask!=12)$fatal;
original={32'h10000080,32'hffff8000,32'h10007fff,32'h10000000};check();if(tag!=4 || mask!=4)$fatal;
original={32'hffffffff,32'h87654321,32'h12345678,32'd0};check();if(tag!=0)$fatal;
// Signed delta boundaries: +128 needs delta16; +32768 needs raw fallback.
original={32'h10000080,32'h10000000,32'h10000000,32'h10000000};check();if(tag!=4)$fatal;
original={32'h10008000,32'h10000000,32'h10000000,32'h10000000};check();if(tag!=0)$fatal;
original={32'h0fffff80,32'h10000000,32'h10000000,32'h10000000};check();if(tag!=3)$fatal;
original={32'h0fff8000,32'h10000000,32'h10000000,32'h10000000};check();if(tag!=4)$fatal;
for(i=0;i<10000;i=i+1)begin
 base=$random;
 for(j=0;j<4;j=j+1)begin
 case(i%4)
 0:value=$random;
 1:value=base+($random%128);
 2:value=base+($random%32768);
 3:value=(j%2) ? base+($random%64) : ($random%64);
 endcase
 original[j*32+:32]=value;
 end
 check();
end
$display("PASS: five formats and 10000 randomized lossless codec round trips");$finish;
end
endmodule
