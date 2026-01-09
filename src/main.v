(* top *) module top (
    (* iopad_external_pin, clkbuf_inhibit *) input clk,
    (* iopad_external_pin *) output clk_en,
    (* iopad_external_pin *) input rst_n,

    // SPI Interface Pins (Connected to Shrike MCU)
    (* iopad_external_pin *) input  spi_ss_n,
    (* iopad_external_pin *) input  spi_sck,
    (* iopad_external_pin *) input  spi_mosi,
    (* iopad_external_pin *) output spi_miso,
    (* iopad_external_pin *) output spi_miso_en
);

    // Drive Clock Enable High to keep the logic active
    assign clk_en = 1'b1;

    // Internal Reset Logic
    wire rst = !rst_n;

    // --- SPI Target Instance ---
    wire [7:0] mcu_rx_data;
    wire       mcu_rx_valid;
    wire       spi_tx_hold;
    reg [7:0]  spi_tx_data;
    
    // SPI Read/Write Protocol State
    reg [1:0]  spi_state;        // 0=idle, 1=write_mode, 2=read_mode
    reg [7:0]  spi_read_addr;    // Address to read from
    reg [2:0]  spi_read_byte_cnt; // Which byte of 32-bit word to send
    
    // SPI transmit data logic
    always @(posedge clk) begin
        if (rst) begin
            spi_tx_data <= 8'hA5;
            spi_state <= 2'd0;
            spi_read_addr <= 8'd0;
            spi_read_byte_cnt <= 3'd0;
        end else if (mcu_rx_valid) begin
            // First byte determines command: 0xFF = read, else = write address
            if (spi_state == 2'd0) begin
                if (mcu_rx_data == 8'hFF) begin
                    // Read command - next byte is address
                    spi_state <= 2'd2;  // read_mode
                end else begin
                    // Write command - this byte is address
                    spi_state <= 2'd1;  // write_mode
                end
            end else if (spi_state == 2'd2) begin
                // Got read address, prepare to send data
                spi_read_addr <= mcu_rx_data;
                spi_read_byte_cnt <= 3'd0;
                spi_state <= 2'd0;  // Return to idle after address received
            end
        end else if (spi_tx_hold) begin
            // Send requested memory data or status byte
            if (spi_read_byte_cnt == 3'd0) begin
                // Send byte 0 of memory word
                spi_tx_data <= shared_mem[spi_read_addr][7:0];
                spi_read_byte_cnt <= 3'd1;
            end else if (spi_read_byte_cnt == 3'd1) begin
                // Send byte 1
                spi_tx_data <= shared_mem[spi_read_addr][15:8];
                spi_read_byte_cnt <= 3'd2;
            end else if (spi_read_byte_cnt == 3'd2) begin
                // Send byte 2
                spi_tx_data <= shared_mem[spi_read_addr][23:16];
                spi_read_byte_cnt <= 3'd3;
            end else if (spi_read_byte_cnt == 3'd3) begin
                // Send byte 3
                spi_tx_data <= shared_mem[spi_read_addr][31:24];
                spi_read_byte_cnt <= 3'd4;
            end else begin
                // Default: send status byte
                spi_tx_data <= 8'hA5;
                spi_read_byte_cnt <= 3'd0;
            end
        end
    end
    
    spi_target #(
        .WIDTH(8),
        .CPOL(0),
        .CPHA(0)
    ) mcu_bridge (
        .i_clk           (clk),
        .i_rst_n         (rst_n),
        .i_enable        (1'b1),
        .i_ss_n          (spi_ss_n),
        .i_sck           (spi_sck),
        .i_mosi          (spi_mosi),
        .o_miso          (spi_miso),
        .o_miso_oe       (spi_miso_en),
        .o_rx_data       (mcu_rx_data),
        .o_rx_data_valid (mcu_rx_valid),
        .i_tx_data       (spi_tx_data),
        .o_tx_data_hold  (spi_tx_hold)
    );

    // --- CPU Core Wires ---
    // Wishbone Memory Interface
    wire [31:0] wb_adr;
    wire [31:0] wb_dat;
    wire [31:0] wb_rdt;
    wire [3:0]  wb_sel;
    wire        wb_we;
    wire        wb_stb;
    wire        wb_cyc;
    wire        wb_ack;
    
    // Instruction Bus (separate from data bus for Harvard architecture)
    wire [31:0] ibus_adr;
    wire        ibus_cyc;
    wire        ibus_ack;
    wire [31:0] ibus_rdt;

    // Register File Interface (bit-serial)
    wire [4:0]  rf_waddr;
    wire [4:0]  rf_raddr0;
    wire [4:0]  rf_raddr1;
    wire        rf_wdata;
    wire        rf_rdata0;
    wire        rf_rdata1;
    wire        rf_wen;
    wire        rf_rreq;
    wire        rf_wreq;
    wire        rf_ready;

    // --- SERV CPU Core Instance ---
    serv_rf_top #(
        .RESET_PC       (32'h0000_0000),
        .RESET_STRATEGY ("MINI"),
        .WITH_CSR       (1)
    ) cpu (
        .clk            (clk),
        .i_rst          (rst),
        .i_timer_irq    (1'b0),
        
        // Instruction Bus (Wishbone)
        .o_ibus_adr     (ibus_adr),
        .o_ibus_cyc     (ibus_cyc),
        .i_ibus_rdt     (ibus_rdt),
        .i_ibus_ack     (ibus_ack),
        
        // Data Bus (Wishbone)
        .o_dbus_adr     (wb_adr),
        .o_dbus_dat     (wb_dat),
        .o_dbus_sel     (wb_sel),
        .o_dbus_we      (wb_we),
        .o_dbus_cyc     (wb_cyc),
        .i_dbus_rdt     (wb_rdt),
        .i_dbus_ack     (wb_ack)
    );

    // --- Memory System ---
    // Shared memory: 256 words (1KB total)
    // 0x0000-0x03FF: Instruction and Data memory
    reg [31:0] shared_mem [0:255]; 

    // Memory address decoding
    wire [7:0] imem_addr = ibus_adr[9:2];  // Word-aligned instruction address
    wire [7:0] dmem_addr = wb_adr[9:2];    // Word-aligned data address
    
    // Instruction fetch logic
    reg ibus_ack_r;
    reg [31:0] ibus_rdt_r;
    
    always @(posedge clk) begin
        if (rst) begin
            ibus_ack_r <= 1'b0;
            ibus_rdt_r <= 32'h0000_0013; // NOP instruction (addi x0, x0, 0)
        end else begin
            ibus_ack_r <= ibus_cyc & !ibus_ack_r;
            if (ibus_cyc & !ibus_ack_r) begin
                ibus_rdt_r <= shared_mem[imem_addr];
            end
        end
    end
    
    assign ibus_ack = ibus_ack_r;
    assign ibus_rdt = ibus_rdt_r;

    // Data memory read/write logic
    reg wb_ack_r;
    reg [31:0] wb_rdt_r;
    
    always @(posedge clk) begin
        if (rst) begin
            wb_ack_r <= 1'b0;
            wb_rdt_r <= 32'h0;
        end else begin
            wb_ack_r <= wb_stb & wb_cyc & !wb_ack_r;
            
            // Memory write (with byte enable)
            if (wb_we & wb_stb & wb_cyc & !wb_ack_r) begin
                if (wb_sel[0]) shared_mem[dmem_addr][7:0]   <= wb_dat[7:0];
                if (wb_sel[1]) shared_mem[dmem_addr][15:8]  <= wb_dat[15:8];
                if (wb_sel[2]) shared_mem[dmem_addr][23:16] <= wb_dat[23:16];
                if (wb_sel[3]) shared_mem[dmem_addr][31:24] <= wb_dat[31:24];
            end
            
            // Memory read
            if (!wb_we & wb_stb & wb_cyc & !wb_ack_r) begin
                wb_rdt_r <= shared_mem[dmem_addr];
            end
        end
    end
    
    assign wb_ack = wb_ack_r;
    assign wb_rdt = wb_rdt_r;

    // --- SPI to Memory Programming Interface ---
    // MCU can write to memory via SPI for program loading
    // Format: [7:2] = address (6 bits = 64 locations), [1:0] = control/data
    
    reg [7:0] spi_byte_count;
    reg [31:0] spi_write_buffer;
    reg [7:0] spi_target_addr;
    
    always @(posedge clk) begin
        if (rst) begin
            spi_byte_count <= 8'd0;
            spi_write_buffer <= 32'h0;
            spi_target_addr <= 8'd0;
        end else if (mcu_rx_valid) begin
            case (spi_byte_count[1:0])
                2'b00: begin
                    // First byte: target address
                    spi_target_addr <= mcu_rx_data;
                    spi_byte_count <= spi_byte_count + 1;
                end
                2'b01: begin
                    // Second byte: bits [7:0]
                    spi_write_buffer[7:0] <= mcu_rx_data;
                    spi_byte_count <= spi_byte_count + 1;
                end
                2'b10: begin
                    // Third byte: bits [15:8]
                    spi_write_buffer[15:8] <= mcu_rx_data;
                    spi_byte_count <= spi_byte_count + 1;
                end
                2'b11: begin
                    // Fourth byte: bits [23:16]
                    spi_write_buffer[23:16] <= mcu_rx_data;
                    spi_byte_count <= spi_byte_count + 1;
                end
            endcase
            
            // Write complete 32-bit word after 5th byte (bits [31:24])
            if (spi_byte_count[1:0] == 2'b11 && spi_byte_count >= 4) begin
                shared_mem[spi_target_addr] <= {mcu_rx_data, spi_write_buffer[23:0]};
                spi_byte_count <= 8'd0;
            end
        end
    end

    // --- Initial Memory Contents (Bootstrap) ---
    // Simple program to test the core
    integer i;
    initial begin
        // Initialize all memory to NOPs
        for (i = 0; i < 256; i = i + 1) begin
            shared_mem[i] = 32'h0000_0013; // NOP (addi x0, x0, 0)
        end
        
        // Simple test program at address 0x00
        // This program writes incrementing values to memory
        shared_mem[0]  = 32'h00000513; // addi x10, x0, 0    (x10 = 0, counter)
        shared_mem[1]  = 32'h01000593; // addi x11, x0, 16   (x11 = 16, limit)
        shared_mem[2]  = 32'h10000617; // auipc x12, 0x100   (x12 = base addr)
        shared_mem[3]  = 32'h00a60023; // sb x10, 0(x12)     (store byte)
        shared_mem[4]  = 32'h00150513; // addi x10, x10, 1   (counter++)
        shared_mem[5]  = 32'h00160613; // addi x12, x12, 1   (addr++)
        shared_mem[6]  = 32'hfeb54ce3; // blt x10, x11, -8   (loop if counter < limit)
        shared_mem[7]  = 32'h0000006f; // jal x0, 0          (infinite loop)
    end

endmodule