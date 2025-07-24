// simple_spi_master.sv
//
// This module implements a simplified SPI Master for basic SPI Flash operations.
// It focuses on the core SPI protocol (Mode 0: CPOL=0, CPHA=0) and a basic state machine
// to demonstrate common flash commands like Write Enable, Page Program, and Read Data.
//
// This version removes the AXI4-Lite interface for easier simulation and understanding
// of the fundamental SPI communication.
//
// Features:
// - Configurable SPI clock frequency (prescaler)
// - Supports Write Enable (0x06), Page Program (0x02), and Read Data (0x03) commands
// - Direct inputs for command, address, and data length
// - Simple state machine for command and data transfer
//
// Limitations:
// - Only supports SPI Mode 0.
// - Basic error handling.
// - No data buffering (FIFOs) - data is provided/consumed one byte at a time.
// - No support for Quad SPI, Dual SPI, or other advanced features.

module simple_spi_master #(
    parameter SPI_CLK_DIV_VALUE = 4 // N for clk_spi = clk_sys / (2 * (N+1))
) (
    // System Signals
    input  logic                                 clk,         // System clock
    input  logic                                 rst_n,       // Asynchronous reset, active low

    // Control Signals (Direct Inputs for simplicity)
    input  logic                                 start_op,    // Start the SPI operation
    input  logic                                 read_op,     // 1=read, 0=write/program
    input  logic [7:0]                           spi_command, // 8-bit SPI command
    input  logic [23:0]                          spi_addr,    // 24-bit SPI address
    input  logic [15:0]                          spi_data_len, // Number of bytes to read/write

    // Data Interface (Single byte for simplicity, no FIFOs)
    input  logic [7:0]                           tx_data_in,  // Data byte to transmit
    output logic                                 tx_data_ready, // Indicates master is ready for next TX byte
    output logic [7:0]                           rx_data_out, // Data byte received
    output logic                                 rx_data_valid, // Indicates a valid RX byte is available

    // Status Signals
    output logic                                 busy,        // 1=controller is busy, 0=idle
    output logic                                 op_done,     // 1=operation complete, cleared when start_op goes high

    // SPI Interface
    output logic                                 spi_sck,     // SPI Serial Clock
    output logic                                 spi_cs_n,    // SPI Chip Select (active low)
    output logic                                 spi_mosi,    // SPI Master Out Slave In
    input  logic                                 spi_miso     // SPI Master In Slave Out
);

    // =================================================================================================
    // SPI Clock Generator
    // =================================================================================================
    logic [7:0] spi_clk_cnt; // Counter for SPI clock division (assuming SPI_CLK_DIV_VALUE fits)
    logic       spi_sck_int; // Internal SPI clock signal

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_clk_cnt <= '0;
            spi_sck_int <= 1'b0;
        end else begin
            if (spi_clk_cnt == SPI_CLK_DIV_VALUE) begin
                spi_clk_cnt <= '0;
                spi_sck_int <= ~spi_sck_int; // Toggle clock
            end else begin
                spi_clk_cnt <= spi_clk_cnt + 1'b1;
            end
        end
    end

    assign spi_sck = spi_sck_int; // Output the generated SPI clock

    // =================================================================================================
    // SPI Master State Machine
    // =================================================================================================

    // State definitions
    typedef enum logic [3:0] {
        IDLE,
        SEND_CMD,
        SEND_ADDR,
        SEND_DATA,
        RECV_DATA,
        OP_COMPLETE
    } spi_state_e;

    spi_state_e current_state, next_state;

    // Internal signals for SPI transfer
    logic [7:0] spi_tx_byte;
    logic [7:0] spi_rx_byte;
    logic       spi_tx_en;   // Enable SPI transmission
    logic       spi_rx_en;   // Enable SPI reception
    logic       spi_byte_done; // Indicates 8 bits have been transferred

    logic [2:0] bit_cnt; // Bit counter for 8-bit transfer
    logic       sck_rise_edge, sck_fall_edge; // Detect clock edges

    // Edge detection for SPI clock
    logic spi_sck_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_sck_q <= 1'b0;
        end else begin
            spi_sck_q <= spi_sck_int;
        end
    end
    assign sck_rise_edge = spi_sck_int && ~spi_sck_q; // CPOL=0, CPHA=0: Data sampled on rising edge
    assign sck_fall_edge = ~spi_sck_int && spi_sck_q; // Data changes on falling edge

    // SPI MOSI/MISO shift register
    logic [7:0] tx_shift_reg;
    logic [7:0] rx_shift_reg;

    assign spi_mosi = tx_shift_reg[7]; // MSB first

    // SPI bit-level transfer logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift_reg <= '0;
            rx_shift_reg <= '0;
            bit_cnt      <= '0;
            spi_byte_done <= 1'b0;
        end else begin
            spi_byte_done <= 1'b0; // Default to false
            rx_data_valid <= 1'b0; // Default to false

            if (current_state != IDLE && current_state != OP_COMPLETE) begin
                if (sck_fall_edge) begin // Data changes on falling edge (CPHA=0)
                    if (spi_tx_en) begin
                        tx_shift_reg <= {tx_shift_reg[6:0], 1'b0}; // Shift left
                    end
                end
                if (sck_rise_edge) begin // Data sampled on rising edge (CPOL=0, CPHA=0)
                    if (spi_rx_en) begin
                        rx_shift_reg <= {rx_shift_reg[6:0], spi_miso}; // Shift left
                    end
                    bit_cnt <= bit_cnt + 1'b1;
                    if (bit_cnt == 3'd7) begin // After 8 bits
                        bit_cnt       <= '0;
                        spi_byte_done <= 1'b1;
                        spi_rx_byte   <= rx_shift_reg; // Capture received byte
                        if (spi_rx_en) begin
                            rx_data_out   <= rx_shift_reg; // Output received byte
                            rx_data_valid <= 1'b1; // Indicate new data
                        end
                    end
                end
            end else begin
                bit_cnt <= '0;
            end
        end
    end

    // State machine registers
    logic [23:0] current_addr_internal;
    logic [15:0] bytes_transferred;
    logic [7:0]  current_command_internal;
    logic        internal_start_op; // Internal signal to capture start_op pulse

    // Chip Select control
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_cs_n <= 1'b1; // De-assert CS
        end else begin
            if (current_state == IDLE && next_state != IDLE) begin
                spi_cs_n <= 1'b0; // Assert CS when starting operation
            end else if (next_state == OP_COMPLETE) begin
                spi_cs_n <= 1'b1; // De-assert CS when operation is complete
            end
        end
    end

    // Status and Control Logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            busy          <= 1'b0;
            op_done       <= 1'b0;
            current_addr_internal <= '0;
            bytes_transferred <= '0;
            current_command_internal <= '0;
            internal_start_op <= 1'b0;
        end else begin
            current_state <= next_state;

            // Latch start_op and clear op_done
            if (start_op && !internal_start_op) begin // Detect rising edge of start_op
                internal_start_op <= 1'b1;
                busy <= 1'b1; // Set busy when operation starts
                op_done <= 1'b0; // Clear op_done
                current_addr_internal <= spi_addr; // Initialize address
                bytes_transferred <= '0; // Reset byte counter
                current_command_internal <= spi_command; // Store current command
            end else if (!start_op) begin
                internal_start_op <= 1'b0; // Clear internal start_op
            end

            if (next_state == OP_COMPLETE) begin
                busy <= 1'b0; // Clear busy when operation finishes
                op_done <= 1'b1; // Set op_done
            end
        end
    end

    // Next State Logic and Output Logic
    assign tx_data_ready = (current_state == SEND_DATA) && !spi_tx_en; // Ready for next byte when not currently transmitting
                                                                        // and in SEND_DATA state. This is simplistic,
                                                                        // a real FIFO would manage this better.

    always_comb begin
        next_state  = current_state;
        spi_tx_en   = 1'b0;
        spi_rx_en   = 1'b0;
        spi_tx_byte = '0;

        case (current_state)
            IDLE: begin
                if (internal_start_op) begin // Trigger on internal_start_op
                    next_state = SEND_CMD;
                end
            end

            SEND_CMD: begin
                spi_tx_en = 1'b1;
                tx_shift_reg = current_command_internal; // Load command into shift register
                if (spi_byte_done) begin
                    // If command is a read/write with address, go to SEND_ADDR
                    // Common commands: 0x03 (Read), 0x02 (Page Program)
                    if (current_command_internal == 8'h03 || current_command_internal == 8'h02) begin
                        next_state = SEND_ADDR;
                    end else if (current_command_internal == 8'h06) begin // Write Enable
                        next_state = OP_COMPLETE; // For simple commands, immediately complete
                    end else begin
                        // Handle other commands or error (e.g., Read Status Register 0x05)
                        // For simplicity, assume these also complete immediately if no data/addr
                        next_state = OP_COMPLETE;
                    end
                end
            end

            SEND_ADDR: begin
                spi_tx_en = 1'b1;
                case (bytes_transferred)
                    16'd0: tx_shift_reg = current_addr_internal[23:16]; // Send MSB of address
                    16'd1: tx_shift_reg = current_addr_internal[15:8];
                    16'd2: tx_shift_reg = current_addr_internal[7:0]; // Send LSB of address
                    default: tx_shift_reg = '0; // Should not happen
                endcase

                if (spi_byte_done) begin
                    bytes_transferred = bytes_transferred + 1'b1;
                    if (bytes_transferred == 16'd3) begin // All 3 address bytes sent
                        bytes_transferred = '0; // Reset for data transfer
                        if (read_op) begin
                            next_state = RECV_DATA;
                        end else begin
                            next_state = SEND_DATA;
                        end
                    end
                end
            end

            SEND_DATA: begin
                spi_tx_en = 1'b1;
                tx_shift_reg = tx_data_in; // Get data from input
                // In a real scenario, you'd wait for tx_data_ready to be asserted by the host
                // before loading tx_data_in. For this simplified version, we assume data is ready.

                if (spi_byte_done) begin
                    bytes_transferred = bytes_transferred + 1'b1;
                    if (bytes_transferred == spi_data_len) begin
                        next_state = OP_COMPLETE; // All data sent
                    end
                end
            end

            RECV_DATA: begin
                spi_rx_en = 1'b1;
                spi_tx_en = 1'b1; // Still need to clock out dummy data to receive
                tx_shift_reg = 8'hFF; // Send dummy byte to clock in data

                if (spi_byte_done) begin
                    // rx_data_out and rx_data_valid are handled in the always_ff block
                    bytes_transferred = bytes_transferred + 1'b1;
                    if (bytes_transferred == spi_data_len) begin
                        next_state = OP_COMPLETE; // All data received
                    end
                end
            end

            OP_COMPLETE: begin
                next_state = IDLE; // Return to IDLE
            end

            default: next_state = IDLE; // Should not happen
        endcase
    end

endmodule
