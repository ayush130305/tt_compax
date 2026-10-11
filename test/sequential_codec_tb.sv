`timescale 1ns/1ps
module sequential_codec_tb;
reg clk=0;always #5 clk=~clk;
reg rst=1,rx_valid=0;reg[7:0]rx_data=0;
wire[7:0]tx_data,wdata,rdata;wire tx_start,we;wire[4:0]addr;
compax_controller c(clk,rst,rx_data,rx_valid,tx_data,tx_start,1'b0,we,addr,wdata,rdata);
flop_mem m(clk,we,addr,wdata,addr,rdata);
reg[127:0]original;
wire[127:0]expected;wire[2:0]expected_tag;wire[3:0]expected_mask;
bdi_reference_encoder reference_codec(original,expected,expected_tag,expected_mask);
integer replies=0;reg[7:0]last_reply;
always @(posedge clk)if(tx_start)begin replies=replies+1;last_reply=tx_data;end
task send(input[7:0]v);begin
 @(negedge clk);rx_data=v;rx_valid=1;
 @(negedge clk);rx_valid=0;repeat(2)@(negedge clk);
end endtask
task ack(input integer before_count);integer timeout;begin
 timeout=0;while(replies==before_count && timeout<200)begin @(negedge clk);timeout=timeout+1;end
 if(replies==before_count || last_reply!==8'h4b)$fatal(1,"reply failed");
end endtask
integer i,j,k,before_count;reg[31:0]base,value;
task trial;begin
 before_count=replies;send(8'h50);send(0);send(16);
 for(k=0;k<16;k=k+1)send(original[k*8+:8]);ack(before_count);
 before_count=replies;send(8'h43);send(0);ack(before_count);
 if(c.block_tag[0]!==expected_tag || c.block_mask[0]!==expected_mask)
  $fatal(1,"format mismatch case %d data %h expected %d/%h got %d/%h",i,original,expected_tag,expected_mask,c.block_tag[0],c.block_mask[0]);
 for(k=0;k<16;k=k+1)if(m.mem[k]!==expected[k*8+:8])$fatal(1,"payload mismatch case %d byte %d",i,k);
 before_count=replies;send(8'h44);send(0);ack(before_count);
 for(k=0;k<16;k=k+1)if(m.mem[k]!==original[k*8+:8])$fatal(1,"decode mismatch case %d byte %d",i,k);
 if(c.block_tag[0]!==0 || c.block_mask[0]!==0)$fatal(1,"metadata not cleared");
end endtask
initial begin
 repeat(3)@(negedge clk);rst=0;i=0;
 original=0;trial();original={4{32'h12345678}};trial();
 original={32'h10000080,32'h10000000,32'h10000000,32'h10000000};trial();
 original={32'h10008000,32'h10000000,32'h10000000,32'h10000000};trial();
 original={32'h0fffff80,32'h10000000,32'h10000000,32'h10000000};trial();
 original={32'h0fff8000,32'h10000000,32'h10000000,32'h10000000};trial();
 original={32'h0000007f,32'hffffff80,32'hffffffff,32'd0};trial();
 original={32'h10000001,32'h10000000,32'hffffff80,32'd0};trial();
 for(i=0;i<10000;i=i+1)begin
  base=$random;
  for(j=0;j<4;j=j+1)begin
   case(i%5)
    0:value=$random;
    1:value=base+($random%128);
    2:value=base+($random%32768);
    3:value=(j%2)?base+($random%64):($random%64);
    4:value=base;
   endcase
   original[j*32+:32]=value;
  end
  trial();
 end
 $display("PASS: 10000 randomized sequential encode/store/decode operations match reference tags, masks and every payload byte; signed boundary cases pass");$finish;
end
initial begin #50000000;$fatal(1,"timeout");end
endmodule
