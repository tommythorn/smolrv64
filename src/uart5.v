// sifive,uart0 compatible UART Implementation with Avalon
// Memory-Mapped Interface
//
// Based on SiFive UART specification, except for an additional
// register 7.  When read, it returns the base frequency (from a
// module parameter) as a read-only register (as we don't yet have a
// device tree).  With a stunning lack of ambition, we are limited to
// 4 GHz.  When written, the lsb enables stretching the bps length by
// a cycle, every other cycle, thus effectively is a "0.5" of div.
// This enables, for example a 8.5 divisor which is needed for 3 Mb/s
// at 25 MHz.
//
// off | name   | write                         | read
// --------------------------------------------------------------------------
// 0   | txdata |              transmit_data:8  | full:1                 0:31
// 1   | rxdata |                            -  | empty:1 0:23 receive_data:8
// 2   | txctrl |  txcnt:3 -:14 nstop:1 txen:1  | (same)
// 3   | rxctrl |  rxcnt:3 -:15         rxen:1  | (same)
// 4   | ie     |          txwm_ie:1 rxwm_ie:1  | (same)
// 5   | ip     |                            -  |         txwm_ip:1 rxwm_ip:1
// 6   | div    |                     divm1:16  | (same)
// 7   | freq   |                   div_half:1  |        base_frequency_Hz:32
//
// Bits not specified ignore writes, reads undefined (really 0)
// txen/rxen enables the transmitted/receiver respectively
// txcnt/rxcnt are levels for the respective watermark interrupts
// ie are interrupt enables bits, ip interrupt pending bits
// the bit rate is given by f_base / (divm1 + 1)

`timescale 1ns/10ps
`default_nettype none

module uart5 #(
    parameter TX_FIFO_DEPTH = 8,
    parameter RX_FIFO_DEPTH = 8,
    parameter ADDR_WIDTH = 3,  // 8 registers, need 3 bits for word addressing
    parameter CLK_FREQUENCY = 0
)(
    // Clock and Reset
    input wire clk,
    input wire rst_n,

    // Avalon Memory-Mapped Slave Interface
    input wire [ADDR_WIDTH-1:0] avs_address,
    input wire avs_read,
    input wire avs_write,
    input wire [31:0] avs_writedata,
    output reg        avs_readdatavalid,
    output reg [31:0] avs_readdata,
    output wire avs_waitrequest,

    // UART Serial Interface
    input wire uart_rx,
    output wire uart_tx,

    // Interrupt
    output wire irq
);

    // Register addresses (word-aligned)
    localparam TXDATA_ADDR = 3'h0;  // 0x00
    localparam RXDATA_ADDR = 3'h1;  // 0x04
    localparam TXCTRL_ADDR = 3'h2;  // 0x08
    localparam RXCTRL_ADDR = 3'h3;  // 0x0C
    localparam IE_ADDR     = 3'h4;  // 0x10
    localparam IP_ADDR     = 3'h5;  // 0x14
    localparam DIV_ADDR    = 3'h6;  // 0x18
    localparam FREQ_ADDR   = 3'h7;  // 0x1C

    // Register bit definitions
    localparam TXDATA_FULL = 31;
    localparam RXDATA_EMPTY = 31;
    localparam TXCTRL_TXEN = 0;
    localparam TXCTRL_NSTOP = 1;
    localparam RXCTRL_RXEN = 0;
    localparam INT_TXWM = 0;
    localparam INT_RXWM = 1;

    // Registers
    reg [18:0] txctrl;  // bits [18:16] = txcnt, [1:0] = nstop, txen
    reg [18:0] rxctrl;  // bits [18:16] = rxcnt, [0] = rxen
    reg [1:0] ie;
    reg [15:0] div;
    reg        div_half;

    // TX FIFO
    reg [7:0] tx_fifo [0:TX_FIFO_DEPTH-1];
    reg [$clog2(TX_FIFO_DEPTH):0] tx_wr_ptr;
    reg [$clog2(TX_FIFO_DEPTH):0] tx_rd_ptr;
    wire [$clog2(TX_FIFO_DEPTH):0] tx_count;
    wire tx_full;
    wire tx_empty;

    // RX FIFO
    reg [7:0] rx_fifo [0:RX_FIFO_DEPTH-1];
    reg [$clog2(RX_FIFO_DEPTH):0] rx_wr_ptr;
    reg [$clog2(RX_FIFO_DEPTH):0] rx_rd_ptr;
    wire [$clog2(RX_FIFO_DEPTH):0] rx_count;
    wire rx_full;
    wire rx_empty;

    // UART TX/RX state machines
    reg [3:0] tx_state;
    reg [3:0] rx_state;
    reg [15:0] tx_baud_counter;
    reg [15:0] rx_baud_counter;
    reg [3:0] tx_bit_counter;
    reg [3:0] rx_bit_counter;
    reg [7:0] tx_shift_reg;
    reg [7:0] rx_shift_reg;
    reg uart_tx_reg;
    reg uart_rx_sync1, uart_rx_sync2;

    // Initialize FIFO memory to prevent X propagation
    integer init_i;
    initial begin
        for (init_i = 0; init_i < TX_FIFO_DEPTH; init_i = init_i + 1)
            tx_fifo[init_i] = 8'h00;
        for (init_i = 0; init_i < RX_FIFO_DEPTH; init_i = init_i + 1)
            rx_fifo[init_i] = 8'h00;
    end

    // TX state definitions
    localparam TX_IDLE = 4'd0;
    localparam TX_START = 4'd1;
    localparam TX_DATA = 4'd2;
    localparam TX_STOP = 4'd3;
    localparam TX_WAIT = 4'd4;

    localparam RX_IDLE = 4'd0;
    localparam RX_START = 4'd1;
    localparam RX_DATA = 4'd2;
    localparam RX_STOP = 4'd3;

    // FIFO count calculations
    assign tx_count = tx_wr_ptr - tx_rd_ptr;
    assign tx_full = tx_count == TX_FIFO_DEPTH;
    assign tx_empty = tx_count == 0;

    assign rx_count = rx_wr_ptr - rx_rd_ptr;
    assign rx_full = rx_count == RX_FIFO_DEPTH;
    assign rx_empty = rx_count == 0;

    // Watermark calculations
    wire [2:0] tx_watermark = txctrl[18:16];
    wire [2:0] rx_watermark = rxctrl[18:16];

    // Interrupt pending (registered for stability)
    reg ip_txwm_reg;
    reg ip_rxwm_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ip_txwm_reg <= 1'b0;
            ip_rxwm_reg <= 1'b0;
        end else begin
            ip_txwm_reg <= (tx_count <= tx_watermark);
            ip_rxwm_reg <= (rx_count > rx_watermark);
        end
    end

    wire [1:0] ip = {ip_rxwm_reg, ip_txwm_reg};

    // Interrupt output
    assign irq = |(ip & ie);

    // Avalon interface - no wait states
    assign avs_waitrequest = 1'b0;

    // UART TX output
    assign uart_tx = uart_tx_reg;

    // Avalon read interface
    always @(posedge clk or negedge rst_n) begin
       avs_readdatavalid <= 0;
        if (!rst_n) begin
            avs_readdata <= 32'h0;
        end else if (avs_read) begin
            avs_readdatavalid <= 1;
            case (avs_address)
                TXDATA_ADDR: avs_readdata <= {tx_full, 31'h0};
                RXDATA_ADDR: avs_readdata <= rx_empty ? {1'b1, 31'h0} : {1'b0, 24'h0, rx_fifo[rx_rd_ptr[$clog2(RX_FIFO_DEPTH)-1:0]]};
                TXCTRL_ADDR: avs_readdata <= {13'h0, txctrl};
                RXCTRL_ADDR: avs_readdata <= {13'h0, rxctrl};
                IE_ADDR:     avs_readdata <= {30'h0, ie};
                IP_ADDR:     avs_readdata <= {30'h0, ip};
                DIV_ADDR:    avs_readdata <= {16'h0, div};
                FREQ_ADDR:   avs_readdata <= CLK_FREQUENCY;
                default:     avs_readdata <= 32'hX; // This cannot happen
            endcase
        end
    end

    // Avalon write interface and register updates
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txctrl <= 19'h0;
            rxctrl <= 19'h0;
            ie <= 2'h0;
            div <= 16'h0;
            tx_wr_ptr <= {($clog2(TX_FIFO_DEPTH)+1){1'b0}};
        end else begin
            if (avs_write) begin
                case (avs_address)
                    TXDATA_ADDR: begin
                        if (!tx_full && txctrl[TXCTRL_TXEN]) begin
                            tx_fifo[tx_wr_ptr[$clog2(TX_FIFO_DEPTH)-1:0]] <= avs_writedata[7:0];
                            tx_wr_ptr <= tx_wr_ptr + 1;
                        end
                    end
                    TXCTRL_ADDR: txctrl <= avs_writedata[18:0] & 19'h70003;
                    RXCTRL_ADDR: rxctrl <= avs_writedata[18:0] & 19'h70001;
                    IE_ADDR:     ie <= avs_writedata[1:0];
                    DIV_ADDR:    div <= avs_writedata[15:0];
                    FREQ_ADDR:   div_half <= avs_writedata[0];
                    default: ;
                endcase
            end
        end
    end

    // RX FIFO read pointer update (on RXDATA read)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_rd_ptr <= {($clog2(RX_FIFO_DEPTH)+1){1'b0}};
        end else begin
            if (avs_read && avs_address == RXDATA_ADDR && !rx_empty) begin
                rx_rd_ptr <= rx_rd_ptr + 1;
            end
        end
    end

    // UART RX synchronizer
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_rx_sync1 <= 1'b1;
            uart_rx_sync2 <= 1'b1;
        end else begin
            uart_rx_sync1 <= uart_rx;
            uart_rx_sync2 <= uart_rx_sync1;
        end
    end

    // UART TX state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_baud_counter <= 16'h0;
            tx_bit_counter <= 4'h0;
            tx_shift_reg <= 8'h0;
            uart_tx_reg <= 1'b1;
            tx_rd_ptr <= {($clog2(TX_FIFO_DEPTH)+1){1'b0}};
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    uart_tx_reg <= 1'b1;
                    if (!tx_empty && txctrl[TXCTRL_TXEN]) begin
                        tx_shift_reg <= tx_fifo[tx_rd_ptr[$clog2(TX_FIFO_DEPTH)-1:0]];
`ifdef ECHO_TX
                        $write("%c", tx_fifo[tx_rd_ptr[$clog2(TX_FIFO_DEPTH)-1:0]]);
`endif
                        tx_rd_ptr <= tx_rd_ptr + 1;
                        tx_state <= TX_START;
                        tx_baud_counter <= div;
                        tx_bit_counter <= 0;
                    end
                end

                TX_START: begin
                    uart_tx_reg <= 1'b0;  // Start bit
                    if (tx_baud_counter == 0) begin
                        tx_state <= TX_DATA;
                        tx_bit_counter <= 0;
                        tx_baud_counter <= div;
                    end else begin
                        tx_baud_counter <= tx_baud_counter - 1;
                    end
                end

                TX_DATA: begin
                    uart_tx_reg <= tx_shift_reg[0];
                    if (tx_baud_counter == 0) begin
                        tx_bit_counter <= tx_bit_counter + 1;
                        tx_shift_reg <= {1'b1, tx_shift_reg[7:1]};
                        tx_baud_counter <= div + (div_half & tx_bit_counter[0]);
                        if (tx_bit_counter == 7) begin
                            tx_state <= TX_STOP;
                        end
                    end else begin
                        tx_baud_counter <= tx_baud_counter - 1;
                    end
                end

                TX_STOP: begin
                    uart_tx_reg <= 1'b1;  // Stop bit
                    if (tx_baud_counter == 0) begin
                        if (txctrl[TXCTRL_NSTOP] && tx_bit_counter == 8) begin
                            // Second stop bit
                            tx_bit_counter <= 9;
                            tx_baud_counter <= div;
                        end else begin
                            tx_state <= TX_WAIT;
                            tx_baud_counter <= div;
                        end
                    end else begin
                        tx_baud_counter <= tx_baud_counter - 1;
                    end
                end

                TX_WAIT: begin
                    uart_tx_reg <= 1'b1;
                    if (tx_baud_counter == 0) begin
                        tx_state <= TX_IDLE;
                    end else begin
                        tx_baud_counter <= tx_baud_counter - 1;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // UART RX state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            rx_baud_counter <= 16'h0;
            rx_bit_counter <= 4'h0;
            rx_shift_reg <= 8'h0;
            rx_wr_ptr <= {($clog2(RX_FIFO_DEPTH)+1){1'b0}};
        end else begin
            case (rx_state)
                RX_IDLE: begin
                    if (!uart_rx_sync2 && rxctrl[RXCTRL_RXEN]) begin
                        // Start bit detected
                        rx_state <= RX_START;
                        rx_baud_counter <= div/2;  // Sample at middle of bit
                    end
                end

                RX_START: begin
                    if (rx_baud_counter == 0) begin
                        if (!uart_rx_sync2) begin  // Verify start bit
                            rx_state <= RX_DATA;
                            rx_bit_counter <= 0;
                            rx_baud_counter <= div;
                        end else begin
                            rx_state <= RX_IDLE;  // False start
                        end
                    end else begin
                        rx_baud_counter <= rx_baud_counter - 1;
                    end
                end

                RX_DATA: begin
                    if (rx_baud_counter == 0) begin
                        rx_shift_reg <= {uart_rx_sync2, rx_shift_reg[7:1]};
                        rx_bit_counter <= rx_bit_counter + 1;
                        rx_baud_counter <= div + (div_half & rx_bit_counter[0]);
                        if (rx_bit_counter == 7) begin
                            rx_state <= RX_STOP;
                        end
                    end else begin
                        rx_baud_counter <= rx_baud_counter - 1;
                    end
                end

                RX_STOP: begin
                    if (rx_baud_counter == 0) begin
                        if (uart_rx_sync2) begin  // Valid stop bit
                            if (!rx_full) begin
                                rx_fifo[rx_wr_ptr[$clog2(RX_FIFO_DEPTH)-1:0]] <= rx_shift_reg;
                                rx_wr_ptr <= rx_wr_ptr + 1;
                            end
                        end
                        rx_state <= RX_IDLE;
                    end else begin
                        rx_baud_counter <= rx_baud_counter - 1;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end
endmodule
