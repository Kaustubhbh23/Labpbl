// simple_spi_master_tb.sv
//
// Testbench for the simple_spi_master module.
// This testbench includes a basic behavioral model of an SPI Flash device
// to simulate the interaction between the master and the flash.

module simple_spi_master_tb;

    // =================================================================================================
    // Testbench Parameters
    // =================================================================================================
    parameter CLK_PERIOD = 10ns; // System clock period
    parameter FLASH_MEM_SIZE = 256; // Size of the simulated flash memory (bytes)
    parameter SPI_CLK_DIV_TB = 4; // Must match the parameter in simple_spi_master

    // =================================================================================================
    // Testbench Signals
    // =================================================================================================
    logic clk;
    logic rst_n;

    logic start_op;
    logic read_op;
    logic [7:0] spi_command;
    logic [23:0] spi_addr;
    logic [15:0] spi_data_len;

    logic [7:0] tx_data_in;
    logic tx_data_ready;
    logic [7:0] rx_data_out;
    logic rx_data_valid;

    logic busy;
    logic op_done;

    logic spi_sck;
    logic spi_cs_n;
    logic spi_mosi;
    logic spi_miso; // This will be driven by the flash model

    // =================================================================================================
    // Instantiate the Device Under Test (DUT)
    // =================================================================================================
    simple_spi_master #(
        .SPI_CLK_DIV_VALUE(SPI_CLK_DIV_TB)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start_op(start_op),
        .read_op(read_op),
        .spi_command(spi_command),
        .spi_addr(spi_addr),
        .spi_data_len(spi_data_len),
        .tx_data_in(tx_data_in),
        .tx_data_ready(tx_data_ready),
        .rx_data_out(rx_data_out),
        .rx_data_valid(rx_data_valid),
        .busy(busy),
        .op_done(op_done),
        .spi_sck(spi_sck),
        .spi_cs_n(spi_cs_n),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

    // =================================================================================================
    // Clock Generation
    // =================================================================================================
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // =================================================================================================
    // SPI Flash Behavioral Model
    // This is a simplified model to respond to the master's commands.
    // =================================================================================================
    logic [7:0] flash_mem [0:FLASH_MEM_SIZE-1];
    logic       wel_set; // Write Enable Latch
    logic [23:0] flash_addr_internal;
    logic [15:0] flash_data_counter;
    logic [7:0]  flash_command_internal; // To store the received command

    // States for the flash model's internal state machine
    typedef enum logic [2:0] {
        FLASH_IDLE,
        FLASH_RECV_CMD,
        FLASH_RECV_ADDR,
        FLASH_RECV_DATA,
        FLASH_SEND_DATA
    } flash_state_e;

    flash_state_e flash_current_state, flash_next_state;

    logic [7:0] flash_rx_byte; // Byte received by flash
    logic [7:0] flash_tx_byte_buffer; // Buffer for the byte to transmit from flash (loaded from memory)
    logic [2:0] flash_bit_cnt; // Bit counter for flash
    logic       flash_byte_done; // Indicates 8 bits transferred for flash

    // Internal signals for SPI communication with the flash
    logic flash_sck_q;
    logic flash_sck_rise_edge, flash_sck_fall_edge;

    // Edge detection for flash model
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flash_sck_q <= 1'b0;
        end else begin
            flash_sck_q <= spi_sck;
        end
    end
    assign flash_sck_rise_edge = spi_sck && ~flash_sck_q;
    assign flash_sck_fall_edge = ~spi_sck && flash_sck_q;

    // Flash internal shift register for MISO output
    logic [7:0] flash_miso_shift_reg; // <<< DECLARATION: This is where flash_miso_shift_reg is declared.
    logic [7:0] flash_shift_reg_rx;   // <<< DECLARATION: This is where flash_shift_reg_rx is declared.

    // Control spi_miso from flash model
    // MISO is driven by the MSB of the shift register when in SEND_DATA state, otherwise 0.
    assign spi_miso = (flash_current_state == FLASH_SEND_DATA) ? flash_miso_shift_reg[7] : 1'b0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flash_shift_reg_rx <= '0;
            flash_tx_byte_buffer <= '0;
            flash_miso_shift_reg <= '0;
            flash_bit_cnt      <= '0;
            flash_byte_done    <= 1'b0;
            flash_current_state <= FLASH_IDLE;
            wel_set            <= 1'b0;
            flash_addr_internal <= '0;
            flash_data_counter <= '0;
            flash_command_internal <= '0;
        end else begin
            flash_byte_done <= 1'b0; // Default to false
            flash_current_state <= flash_next_state; // Update state

            // Detect CS falling edge to reset flash state
            if (!spi_cs_n && $fell(spi_cs_n)) begin
                flash_current_state <= FLASH_RECV_CMD;
                wel_set <= 1'b0;
                flash_addr_internal <= '0;
                flash_data_counter <= '0;
                flash_command_internal <= '0;
                flash_miso_shift_reg <= '0; // Reset MISO shift register
                flash_shift_reg_rx <= '0; // Reset RX shift register
            end else if (spi_cs_n && $rose(spi_cs_n)) begin
                flash_current_state <= FLASH_IDLE;
            end

            if (flash_current_state != FLASH_IDLE) begin
                // Data reception (MOSI -> RX Shift Reg) - samples on rising edge
                if (flash_sck_rise_edge) begin
                    flash_shift_reg_rx <= {flash_shift_reg_rx[6:0], spi_mosi};
                    flash_bit_cnt <= flash_bit_cnt + 1'b1;
                    if (flash_bit_cnt == 3'd7) begin
                        flash_bit_cnt       <= '0;
                        flash_byte_done     <= 1'b1;
                        flash_rx_byte       <= flash_shift_reg_rx; // Capture received byte
                    end
                end

                // Data transmission (TX Byte -> MISO Shift Reg -> MISO)
                if (flash_current_state == FLASH_SEND_DATA) begin
                    // Load new byte into shift register if previous byte is done
                    // OR if just entered SEND_DATA state (indicated by flash_current_state == FLASH_RECV_ADDR
                    // and flash_next_state == FLASH_SEND_DATA from previous cycle)
                    if (flash_byte_done || ($fell(flash_current_state == FLASH_RECV_ADDR && flash_next_state == FLASH_SEND_DATA))) begin
                        if (flash_addr_internal < FLASH_MEM_SIZE) begin
                            flash_tx_byte_buffer = flash_mem[flash_addr_internal];
                        end else begin
                            flash_tx_byte_buffer = 8'hFF; // Send dummy if out of bounds
                        end
                        flash_miso_shift_reg <= flash_tx_byte_buffer; // Load the full byte
                    end

                    // Shift out bits on falling edge
                    if (flash_sck_fall_edge) begin
                        flash_miso_shift_reg <= {flash_miso_shift_reg[6:0], 1'b0}; // Shift out for next bit
                    end
                end
            end
        end
    end

    // Flash Next State Logic (always_comb)
    always_comb begin
        flash_next_state = flash_current_state;

        case (flash_current_state)
            FLASH_IDLE: begin
                // Wait for CS to go low
            end

            FLASH_RECV_CMD: begin
                if (flash_byte_done) begin
                    flash_command_internal = flash_rx_byte; // Store the command
                    case (flash_rx_byte) // This flash_rx_byte is the command
                        8'h06: begin // Write Enable
                            wel_set = 1'b1;
                            flash_next_state = FLASH_IDLE; // Command complete
                        end
                        8'h02: begin // Page Program
                            if (wel_set) begin
                                flash_next_state = FLASH_RECV_ADDR;
                            end else begin
                                $display("FLASH: Error - Page Program without Write Enable!");
                                flash_next_state = FLASH_IDLE; // Error, return to idle
                            end
                        end
                        8'h03: begin // Read Data
                            flash_next_state = FLASH_RECV_ADDR;
                        end
                        default: begin
                            $display("FLASH: Unknown command received: %h", flash_rx_byte);
                            flash_next_state = FLASH_IDLE; // Unknown command, return to idle
                        end
                    endcase
                end
            end

            FLASH_RECV_ADDR: begin
                if (flash_byte_done) begin
                    case (flash_data_counter)
                        16'd0: flash_addr_internal[23:16] = flash_rx_byte;
                        16'd1: flash_addr_internal[15:8]  = flash_rx_byte;
                        16'd2: begin
                            flash_addr_internal[7:0] = flash_rx_byte;
                            // Use flash_command_internal to decide next state
                            if (flash_command_internal == 8'h02) begin // If command was Page Program
                                flash_next_state = FLASH_RECV_DATA;
                            end else if (flash_command_internal == 8'h03) begin // If command was Read Data
                                flash_next_state = FLASH_SEND_DATA;
                                // The first byte for transmission will be loaded into flash_miso_shift_reg
                                // in the always_ff block when flash_current_state transitions to FLASH_SEND_DATA.
                            end else begin
                                flash_next_state = FLASH_IDLE; // Should not happen for these commands
                            end
                        end
                    endcase
                    flash_data_counter = flash_data_counter + 1'b1;
                end
            end

            FLASH_RECV_DATA: begin
                if (flash_byte_done) begin
                    if (flash_addr_internal < FLASH_MEM_SIZE) begin
                        flash_mem[flash_addr_internal] = flash_rx_byte;
                        $display("FLASH: Written 0x%h to address 0x%h", flash_rx_byte, flash_addr_internal);
                        flash_addr_internal = flash_addr_internal + 1'b1;
                        flash_data_counter = flash_data_counter + 1'b1;
                    end else begin
                        $display("FLASH: Write address 0x%h out of bounds!", flash_addr_internal);
                    end
                end
            end

            FLASH_SEND_DATA: begin
                if (flash_byte_done) begin
                    flash_addr_internal = flash_addr_internal + 1'b1;
                    flash_data_counter = flash_data_counter + 1'b1;
                    // The loading of flash_miso_shift_reg for the next byte is handled in the always_ff block
                    // based on flash_byte_done.
                end
            end
        endcase
    end


    // =================================================================================================
    // Test Sequence
    // =================================================================================================
    initial begin
        // Initialize signals
        rst_n = 1'b0;
        start_op = 1'b0;
        read_op = 1'b0;
        spi_command = '0;
        spi_addr = '0;
        spi_data_len = '0;
        tx_data_in = '0;

        // Initialize flash memory with some known pattern
        for (int i = 0; i < FLASH_MEM_SIZE; i++) begin
            flash_mem[i] = i; // Fill with sequential data
        end

        $display("--------------------------------------------------");
        $display("Starting Simple SPI Master Testbench Simulation");
        $display("--------------------------------------------------");

        // Apply reset
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        $display("Reset released.");

        repeat (10) @(posedge clk); // Wait for stability

        // --- Test Case 1: Write Enable ---
        $display("\n--- Test Case 1: Write Enable (0x06) ---");
        spi_command = 8'h06;
        read_op = 1'b0;
        spi_data_len = 16'd0; // No data for WE
        spi_addr = 24'h0; // Address doesn't matter for WE

        start_op = 1'b1;
        @(posedge clk);
        start_op = 1'b0; // Pulse start_op

        wait (op_done);
        $display("Master: Write Enable operation done. busy=%b, op_done=%b", busy, op_done);
        repeat (5) @(posedge clk); // Allow CS to de-assert

        // --- Test Case 2: Page Program (Write 16 bytes) ---
        $display("\n--- Test Case 2: Page Program (0x02) ---");
        spi_command = 8'h02;
        read_op = 1'b0;
        spi_addr = 24'h000010; // Start address for writing
        spi_data_len = 16'd16; // Write 16 bytes

        start_op = 1'b1;
        @(posedge clk);
        start_op = 1'b0;

        // Provide data to tx_data_in when tx_data_ready is asserted
        fork
            begin : tx_data_provider
                integer i;
                for (i = 0; i < spi_data_len; i = i + 1) begin
                    wait (tx_data_ready);
                    tx_data_in = 8'hA0 + i; // Data pattern
                    $display("Master: Providing TX data: 0x%h", tx_data_in);
                    @(posedge clk); // Wait one cycle for master to consume
                end
            end
        join

        wait (op_done);
        $display("Master: Page Program operation done. busy=%b, op_done=%b", busy, op_done);
        repeat (5) @(posedge clk);

        // --- Test Case 3: Read Data (Read 16 bytes from the same address) ---
        $display("\n--- Test Case 3: Read Data (0x03) ---");
        spi_command = 8'h03;
        read_op = 1'b1;
        spi_addr = 24'h000010; // Read from the same address
        spi_data_len = 16'd16; // Read 16 bytes

        start_op = 1'b1;
        @(posedge clk);
        start_op = 1'b0;

        // Consume data from rx_data_out when rx_data_valid is asserted
        fork
            begin : rx_data_consumer
                integer i;
                for (i = 0; i < spi_data_len; i = i + 1) begin
                    wait (rx_data_valid);
                    $display("Master: Received RX data: 0x%h", rx_data_out);
                    @(posedge clk); // Wait one cycle for master to clear valid
                end
            end
        join

        wait (op_done);
        $display("Master: Read Data operation done. busy=%b, op_done=%b", busy, op_done);
        repeat (5) @(posedge clk);

        // --- Test Case 4: Read Data (Read from original flash content) ---
        $display("\n--- Test Case 4: Read Data (0x03) from 0x00 ---");
        spi_command = 8'h03;
        read_op = 1'b1;
        spi_addr = 24'h000000; // Read from address 0x00
        spi_data_len = 16'd8; // Read 8 bytes

        start_op = 1'b1;
        @(posedge clk);
        start_op = 1'b0;

        fork
            begin : rx_data_consumer_2
                integer i;
                for (i = 0; i < spi_data_len; i = i + 1) begin
                    wait (rx_data_valid);
                    $display("Master: Received RX data from 0x00: 0x%h", rx_data_out);
                    @(posedge clk);
                end
            end
        join

        wait (op_done);
        $display("Master: Read Data operation done. busy=%b, op_done=%b", busy, op_done);
        repeat (5) @(posedge clk);


        $display("\n--------------------------------------------------");
        $display("Simulation Finished.");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
