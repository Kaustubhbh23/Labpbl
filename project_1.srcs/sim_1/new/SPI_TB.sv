`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.07.2025 10:45:25
// Design Name: 
// Module Name: SPI_TB
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns/1ps

module spi_flash_tb;

  logic clk;
  logic rst;
  logic start;
  logic [7:0] data_in;
  logic [1:0] operation;
  logic [7:0] data_out;
  logic done;
  logic sclk, cs_n, mosi, miso;

  // Clock generation
  initial clk = 0;
  always #5 clk = ~clk; // 100MHz clock

  // Instantiate SPI Flash Controller
  spi_flash_controller dut (
    .clk(clk),
    .rst(rst),
    .start(start),
    .data_in(data_in),
    .operation(operation),
    .data_out(data_out),
    .done(done),
    .sclk(sclk),
    .cs_n(cs_n),
    .mosi(mosi),
    .miso(miso)
  );

  // Instantiate SPI Flash Memory
  spi_flash_memory mem (
    .clk(clk),
    .sclk(sclk),
    .cs_n(cs_n),
    .mosi(mosi),
    .miso(miso)
  );

  // Instantiate Assertion Checker
  spi_assertions check (
    .clk(clk),
    .rst(rst),
    .cs_n(cs_n),
    .sclk(sclk),
    .mosi(mosi),
    .miso(miso),
    .start(start),
    .done(done)
  );

  // Stimulus
  initial begin
    $display("---- SPI Flash Controller Testbench ----");
    $dumpfile("spi_wave.vcd");
    $dumpvars(0, spi_flash_tb);

    // Reset
    rst = 1;
    start = 0;
    data_in = 8'h00;
    operation = 2'b00; // Read
    #40;
    rst = 0;

    // Stimulate: Perform read at address 0x00
    @(posedge clk);
    data_in = 8'h00; // Address
    start = 1;
    @(posedge clk);
    start = 0;

    // Wait for done
    wait (done == 1);
    @(posedge clk);

    if (data_out == 8'hA5)
      $display("PASS: Data read = %h (expected 0xA5)", data_out);
    else
      $fatal("FAIL: Data read = %h (expected 0xA5)", data_out);

    // Give time to observe final state
    #100;

    $finish;
  end

endmodule



