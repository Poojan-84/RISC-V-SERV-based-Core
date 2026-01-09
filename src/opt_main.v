/* * top.v - Optimized SERV RISC-V Core Deployment for Shrike Lite
 * Resource-constrained version for FPGA-1K Tile
 */

(* top *) module top (
    (* iopad_external_pin, clkbuf_inhibit *) input clk,
    (* iopad_external_pin *) output clk_en,
    (* iopad_external_pin *) input rst_n,

    // SPI Interface Pins
    (* iopad_external_pin *) input  spi_ss_n,
    (* iopad_external_pin *) input  spi_sck,
    (* iopad_external_pin *) input  spi_mosi,
    (* iopad_external_pin *) output spi_miso,
    (* iopad_external_pin *) output spi_miso_en
);

    assign clk_en = 1'b1;
    wire rst = !rst_n;

    // --- SPI Target Bridge ---
    wire [7:0] mcu_rx_data;
    wire       mcu_rx_valid;
    wire       spi_tx_hold;
    wire       spi_miso_internal;
    wire       spi_miso_en_internal;
    reg [7:0]  spi_tx_data;
    
    // Minimal Read State Machine
    reg [2:0] rd_st;
    reg [4:0] rd_addr; // Reduced to 5 bits (32 words)
    
    // Combined SPI Logic to save CLBs
    reg [2:0]  spi_state;
    reg [4:0]  spi_addr;
    reg [23:0] spi_wdata_buf;

    always @(posedge clk) begin
        if (rst) begin
            spi_tx_data <= 8'hA5;
            rd_st       <= 0;
            spi_state   <= 0;
        end else if (mcu_rx_valid) begin
            if (mcu_rx_data == 8'hFF) begin
                rd_st <= 5; // Enter Read Mode
            end else if (rd_st == 5) begin
                rd_addr <= mcu_rx_data[4:0];
                rd_st   <= 1;
            end else if (rd_st == 0) begin
                // Write State Machine
                case (spi_state)
                    3'd0: begin spi_addr <= mcu_rx_data[4:0]; spi_state <= 1; end
                    3'd1: begin spi_wdata_buf[7:0]   <= mcu_rx_data; spi_state <= 2; end
                    3'd2: begin spi_wdata_buf[15:8]  <= mcu_rx_data; spi_state <= 3; end
                    3'd3: begin spi_wdata_buf[23:16] <= mcu_rx_data; spi_state <= 4; end
                    3'd4: begin 
                        ram[spi_addr] <= {mcu_rx_data, spi_wdata_buf[23:0]}; 
                        spi_state <= 0; 
                    end
                endcase
            end
        end else if (spi_tx_hold) begin
            case (rd_st)
                1: begin spi_tx_data <= ram[rd_addr][7:0];   rd_st <= 2; end
                2: begin spi_tx_data <= ram[rd_addr][15:8];  rd_st <= 3; end
                3: begin spi_tx_data <= ram[rd_addr][23:16]; rd_st <= 4; end
                4: begin spi_tx_data <= ram[rd_addr][31:24]; rd_st <= 0; end
                default: spi_tx_data <= 8'hA5;
            endcase
        end
    end
    
    assign spi_miso = spi_miso_internal;
    assign spi_miso_en = spi_miso_en_internal;
    
    spi_target mcu_bridge (
        .i_clk(clk), .i_rst_n(rst_n), .i_enable(1'b1),
        .i_ss_n(spi_ss_n), .i_sck(spi_sck), .i_mosi(spi_mosi),
        .o_miso(spi_miso_internal), .o_miso_oe(spi_miso_en_internal),
        .o_rx_data(mcu_rx_data), .o_rx_data_valid(mcu_rx_valid),
        .i_tx_data(spi_tx_data), .o_tx_data_hold(spi_tx_hold)
    );

    // --- SERV CPU Core (Optimized) ---
    wire [31:0] wb_dbus_adr, wb_dbus_dat, wb_dbus_rdt, wb_ibus_adr, wb_ibus_rdt;
    wire [3:0]  wb_dbus_sel;
    wire        wb_dbus_we, wb_dbus_cyc, wb_dbus_ack, wb_ibus_cyc, wb_ibus_ack;

    serv_rf_top #(
        .RESET_PC(32'h0000_0000),
        .WITH_CSR(0) // Logic Saving: Disabled CSRs
    ) cpu (
        .clk(clk), .i_rst(rst), .i_timer_irq(1'b0),
        .o_ibus_adr(wb_ibus_adr), .o_ibus_cyc(wb_ibus_cyc), .i_ibus_rdt(wb_ibus_rdt), .i_ibus_ack(wb_ibus_ack),
        .o_dbus_adr(wb_dbus_adr), .o_dbus_dat(wb_dbus_dat), .o_dbus_sel(wb_dbus_sel),
        .o_dbus_we(wb_dbus_we), .o_dbus_cyc(wb_dbus_cyc), .i_dbus_rdt(wb_dbus_rdt), .i_dbus_ack(wb_dbus_ack)
    );

    // --- Reduced Memory (32 Words = 128 Bytes) ---
    reg [31:0] ram [0:31];

    // --- Simple Memory Arbitration ---
    assign wb_ibus_rdt = ram[wb_ibus_adr[6:2]];
    assign wb_dbus_rdt = ram[wb_dbus_adr[6:2]];

    reg ibus_ack_reg, dbus_ack_reg;
    always @(posedge clk) begin
        ibus_ack_reg <= wb_ibus_cyc & !wb_dbus_cyc;
        if (wb_dbus_cyc && wb_dbus_we) begin
            if (wb_dbus_sel[0]) ram[wb_dbus_adr[6:2]][7:0]   <= wb_dbus_dat[7:0];
            if (wb_dbus_sel[1]) ram[wb_dbus_adr[6:2]][15:8]  <= wb_dbus_dat[15:8];
            if (wb_dbus_sel[2]) ram[wb_dbus_adr[6:2]][23:16] <= wb_dbus_dat[23:16];
            if (wb_dbus_sel[3]) ram[wb_dbus_adr[6:2]][31:24] <= wb_dbus_dat[31:24];
        end
        dbus_ack_reg <= wb_dbus_cyc;
    end
    assign wb_ibus_ack = ibus_ack_reg;
    assign wb_dbus_ack = dbus_ack_reg;

    // Bootloader: Simple loop (addi x1, x1, 1; jal x0, 0)
    initial begin
        ram[0] = 32'h00108093; 
        ram[1] = 32'hff9ff06f; 
    end
endmodule
