`timescale 1ns/1ps
module uart_bdi_tb;
reg clk=0;
always #20 clk=~clk;
reg rst_n=0;
reg [7:0] ui=1;
wire [7:0] uo;
tt_um_ayush130305_compax dut(.clk(clk),.rst_n(rst_n),.ena(1'b1),.ui_in(ui),.uo_out(uo),.uio_in(8'd0),.uio_out(),.uio_oe());
reg [7:0] received[0:8191];
integer head=0,tail=0;
reg [7:0] sampled;
integer bitno;
initial forever begin
  @(negedge uo[0]);
  if(rst_n) begin
    #(8680+4340);
    for(bitno=0;bitno<8;bitno=bitno+1) begin
      sampled[bitno]=uo[0];
      #8680;
    end
    if(uo[0]!==1'b1) $fatal(1,"bad TX stop bit");
    received[tail]=sampled;
    tail=tail+1;
  end
end
task send(input [7:0] value);
integer b;
begin
  @(negedge clk); ui[0]=0;
  repeat(217) @(negedge clk);
  for(b=0;b<8;b=b+1) begin
    ui[0]=value[b]; repeat(217) @(negedge clk);
  end
  ui[0]=1; repeat(217) @(negedge clk);
end
endtask
task expect_byte(input [7:0] value);
integer timeout;
begin
 timeout=0;
 while(head==tail && timeout<100000) begin @(negedge clk);timeout=timeout+1;end
 if(head==tail) $fatal(1,"UART timeout");
 if(received[head]!==value) $fatal(1,"expected %h got %h at response %0d",value,received[head],head);
 head=head+1;
end
endtask
task write_one(input [7:0] addr,input [7:0] value);
begin send(8'h57);send(addr);send(value);expect_byte(8'h4b);end
endtask
task read_one(input [7:0] addr,input [7:0] value);
begin send(8'h52);send(addr);expect_byte(value);end
endtask
reg [127:0] vector;
integer i, n;
reg [7:0] pattern_byte;
task block_case(input sl, input [127:0] data, input [7:0] meta, input [7:0] length);
integer k;
begin
send(8'h50);send({3'd0,sl,4'd0});send(16);
for(k=0;k<16;k=k+1)send(data[k*8+:8]);expect_byte(8'h4b);
send(8'h43);send({7'd0,sl});expect_byte(8'h4b);
send(8'h4d);send({7'd0,sl});expect_byte(meta);expect_byte(length);
send(8'h43);send({7'd0,sl});expect_byte(8'h4b);
send(8'h44);send({7'd0,sl});expect_byte(8'h4b);
send(8'h44);send({7'd0,sl});expect_byte(8'h4b);
send(8'h4d);send({7'd0,sl});expect_byte(0);expect_byte(16);
send(8'h47);send({3'd0,sl,4'd0});send(16);
for(k=0;k<16;k=k+1)expect_byte(data[k*8+:8]);
end
endtask
initial begin
repeat(6)@(negedge clk);rst_n=1;repeat(4)@(negedge clk);
write_one(0,8'ha5);read_one(0,8'ha5);write_one(31,8'h5a);read_one(31,8'h5a);
send(8'h41);expect_byte(8'h3f);
for(n=0;n<3;n=n+1)begin
 send(8'h50);send(0);send(32);
 for(i=0;i<32;i=i+1)begin
  case(n)0:pattern_byte=i*37+11;1:pattern_byte=~(i*37+11);2:pattern_byte=i;endcase
  send(pattern_byte);
 end
 expect_byte(8'h4b);send(8'h47);send(0);send(32);
 for(i=0;i<32;i=i+1)begin
  case(n)0:pattern_byte=i*37+11;1:pattern_byte=~(i*37+11);2:pattern_byte=i;endcase
  expect_byte(pattern_byte);
 end
end
block_case(0,128'd0,1,0);
block_case(1,{4{32'h12345678}},2,4);
block_case(0,{32'd7,32'hffffff80,32'h1000007f,32'h10000000},8'h63,8);
block_case(1,{32'h10000080,32'hffff8000,32'h10007fff,32'h10000000},8'h24,12);
block_case(0,{32'hffffffff,32'h87654321,32'h12345678,32'd0},0,16);
// Retain one compressed slot while writing a different slot.
send(8'h50);send(16);send(16);for(i=0;i<16;i=i+1)send(0);expect_byte(8'h4b);
send(8'h43);send(1);expect_byte(8'h4b);write_one(15,8'h55);
send(8'h4d);send(1);expect_byte(1);expect_byte(0);
// Writes cannot silently destroy compressed data. Rejected page data is consumed.
send(8'h57);send(16);send(8'haa);expect_byte(8'h45);
send(8'h50);send(0);send(2);send(8'hbb);send(8'hcc);expect_byte(8'h45);
send(8'h4d);send(1);expect_byte(1);expect_byte(0);
send(8'h44);send(1);expect_byte(8'h4b);
send(8'h47);send(16);send(16);for(i=0;i<16;i=i+1)expect_byte(0);
// Invalid slots and legacy page wrap, zero lengths.
send(8'h43);send(2);expect_byte(8'h3f);
send(8'h44);send(255);expect_byte(8'h3f);
send(8'h4d);send(2);expect_byte(8'h3f);
send(8'h50);send(30);send(4);send(8'h11);send(8'h22);send(8'h33);send(8'h44);expect_byte(8'h4b);
send(8'h47);send(30);send(4);expect_byte(8'h11);expect_byte(8'h22);expect_byte(8'h33);expect_byte(8'h44);
send(8'h50);send(0);send(0);expect_byte(8'h4b);
send(8'h47);send(0);send(0);send(8'h41);expect_byte(8'h3f);
$display("PASS: UART raw access, five formats, metadata, repeated C/D, slot isolation, invalid slots and page wrap (%0d replies)",head);$finish;
end
initial begin #400000000; $fatal(1,"global timeout");end
endmodule
