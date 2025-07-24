`timescale 1ns / 1ps

module spi_flash_controller (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [7:0]  command,     // 0x03 = READ, 0x02 = WRITE, 0x20 = ERASE
    input  logic [23:0] address,     // 3-byte address
    input  logic [7:0]  write_data,  // Data to write
    output logic [7:0]  read_data,   // Data read from flash
    output logic        done,        // Transaction complete

    // SPI interface
    output logic        cs,
    output logic        sclk,
    output logic        mosi,
    input  logic        miso
);

    typedef enum logic [2:0] {
        IDLE,
        SEND_CMD,
        SEND_ADDR,
        WRITE_BYTE,
        READ_BYTE,
        DONE
    } state_t;

    state_t state;

    logic [7:0] shift_reg;
    logic [2:0] byte_cnt;
    logic [2:0] bit_cnt;
    logic sclk_en;

    assign sclk = sclk_en ? clk : 1'b0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            cs         <= 1;
            sclk_en    <= 0;
            done       <= 0;
            mosi       <= 0;
            shift_reg  <= 0;
            bit_cnt    <= 0;
            byte_cnt   <= 0;
            read_data  <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done    <= 0;
                    cs      <= 1;
                    sclk_en <= 0;
                    if (start) begin
                        cs        <= 0;
                        shift_reg <= command;
                        bit_cnt   <= 3'd7;
                        state     <= SEND_CMD;
                    end
                end

                SEND_CMD: begin
                    sclk_en <= 1;
                    mosi    <= shift_reg[7];
                    shift_reg <= {shift_reg[6:0], 1'b0};
                    if (bit_cnt == 0) begin
                        bit_cnt   <= 3'd7;
                        byte_cnt  <= 0;
                        shift_reg <= address[23:16];
                        state     <= SEND_ADDR;
                    end else begin
                        bit_cnt <= bit_cnt - 1;
                    end
                end

                SEND_ADDR: begin
                    mosi <= shift_reg[7];
                    shift_reg <= {shift_reg[6:0], 1'b0};
                    if (bit_cnt == 0) begin
                        byte_cnt <= byte_cnt + 1;
                        bit_cnt  <= 3'd7;
                        case (byte_cnt)
                            0: shift_reg <= address[15:8];
                            1: shift_reg <= address[7:0];
                            2: begin
                                case (command)
                                    8'h02: begin
                                        shift_reg <= write_data;
                                        state     <= WRITE_BYTE;
                                    end
                                    8'h03: begin
                                        shift_reg <= 0;
                                        state     <= READ_BYTE;
                                    end
                                    8'h20: state <= DONE;
                                    default: state <= DONE;
                                endcase
                            end
                        endcase
                    end else begin
                        bit_cnt <= bit_cnt - 1;
                    end
                end

                WRITE_BYTE: begin
                    mosi <= shift_reg[7];
                    shift_reg <= {shift_reg[6:0], 1'b0};
                    if (bit_cnt == 0) begin
                        state <= DONE;
                    end else begin
                        bit_cnt <= bit_cnt - 1;
                    end
                end

                READ_BYTE: begin
                    shift_reg <= {shift_reg[6:0], miso};
                    if (bit_cnt == 0) begin
                        read_data <= {shift_reg[6:0], miso};
                        state     <= DONE;
                    end else begin
                        bit_cnt <= bit_cnt - 1;
                    end
                end

                DONE: begin
                    cs      <= 1;
                    sclk_en <= 0;
                    done    <= 1;
                    state   <= IDLE;
                end
            endcase
        end
    end

endmodule
