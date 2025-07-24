`timescale 1ns / 1ps

module tb_spi_flash_controller;

    logic clk;
    logic rst_n;
    logic start;
    logic [7:0] command;
    logic [23:0] address;
    logic [7:0] write_data;
    logic [7:0] read_data;
    logic done;

    logic cs;
    logic sclk;
    logic mosi;
    logic miso;

    // Instantiate DUT
    spi_flash_controller uut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .command(command),
        .address(address),
        .write_data(write_data),
        .read_data(read_data),
        .done(done),
        .cs(cs),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso)
    );

    // Clock generation
    always #5 clk = ~clk;

    // MISO simulation (dummy data returned during READ)
    // MISO returns 8'b10101010 = 0xAA
    always @(negedge sclk) begin
        if (!cs && command == 8'h03) begin
            case (uut.bit_cnt)
                7: miso = 1;
                6: miso = 0;
                5: miso = 1;
                4: miso = 0;
                3: miso = 1;
                2: miso = 0;
                1: miso = 1;
                0: miso = 0;
                default: miso = 0;
            endcase
        end else begin
            miso = 0;
        end
    end

    initial begin
        // Initialize
        clk = 0;
        rst_n = 0;
        start = 0;
        command = 8'h00;
        address = 24'h000000;
        write_data = 8'h00;
        #20;

        rst_n = 1;
        #20;

        // --------------------------
        // Test 1: WRITE command
        // --------------------------
        $display("Test 1: WRITE");
        command = 8'h02;           // WRITE
        address = 24'h0000A5;      // Target address
        write_data = 8'h5A;        // Data to write
        start = 1;
        #10;
        start = 0;

        wait (done);
        $display("WRITE done at time %0t", $time);
        #50;

        // --------------------------
        // Test 2: READ command
        // --------------------------
        $display("Test 2: READ");
        command = 8'h03;           // READ
        address = 24'h0000A5;
        start = 1;
        #10;
        start = 0;

        wait (done);
        $display("READ done at time %0t", $time);
        $display("Read Data: %0h", read_data);
        #50;

        // --------------------------
        // Test 3: ERASE command
        // --------------------------
        $display("Test 3: ERASE");
        command = 8'h20;           // ERASE
        address = 24'h0000A5;
        start = 1;
        #10;
        start = 0;

        wait (done);
        $display("ERASE done at time %0t", $time);
        #50;

        $finish;
    end

endmodule
