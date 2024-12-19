module NES_Urbana(
    input CLK, // 100MHz
    input logic reset_rtl_0, // active low reset (btn 0)
    
    //USB signals   
    input logic [0:0] gpio_usb_int_tri_i,
    output logic gpio_usb_rst_tri_o,
    input logic usb_spi_miso,
    output logic usb_spi_mosi,
    output logic usb_spi_sclk,
    output logic usb_spi_ss,
    
    //UART
    input logic uart_rtl_0_rxd,
    output logic uart_rtl_0_txd,
    
    //HDMI
    output logic hdmi_tmds_clk_n,
    output logic hdmi_tmds_clk_p,
    output logic [2:0]hdmi_tmds_data_n,
    output logic [2:0]hdmi_tmds_data_p,
    
    //HEX displays
    output logic [7:0] hex_segA,
    output logic [3:0] hex_gridA,
    output logic [7:0] hex_segB,
    output logic [3:0] hex_gridB
    
    `ifdef SIMULATION
    output logic [15:0] dbg_cpu_addr,
    output logic [7:0]  dbg_cpu_data,
    output logic        dbg_cpu_read
    `endif
);

logic reset_ah;
logic [2:0] reset_shift;

always_ff @(posedge clk_ibuf) begin
    reset_shift <= {reset_shift[1:0], ~reset_rtl_0};
    reset_ah <= reset_shift[2];
end

/* hex display debug */
hex_driver HexA (
    .clk(clk_ibuf),  
    .reset(reset_ah),
    .in({cpu_reset, cpu_enable, cpu_rdy, cpu_read}),
    .hex_seg(hex_segA),
    .hex_grid(hex_gridA)
);

logic [3:0] dbg [4];  // Debug signals for HexB
always_comb begin
    dbg[0] = {3'b0, locked};        // Clock wizard lock status
    dbg[1] = {3'b0, vde};           // Video Display Enable
    dbg[2] = {3'b0, VGA_HS};        // Horizontal sync
    dbg[3] = {3'b0, VGA_V  S};        // Vertical sync
end

hex_driver HexB (
    .clk(clk_ibuf),            
    .reset(reset_ah), 
    .in(dbg),
    .hex_seg(hex_segB),
    .hex_grid(hex_gridB)
);

/* Clock signals */
logic nes_clk, cpu_clk, ppu_clk, vga_clk;
logic clk_25MHz, clk_125MHz;

wire clk_ibuf;
IBUF clk_ibuf_inst (
    .I(CLK),
    .O(clk_ibuf)
);

clk_wiz_0 clk_wiz (
    .clk_out1(clk_25MHz),    // 25MHz for HDMI
    .clk_out2(clk_125MHz),   // 125MHz for HDMI
    .clk_out3(nes_clk),      // 
    .clk_out4(cpu_clk),      // 
    .reset(reset_rtl_0),
    .locked(locked),
    .clk_in1(clk_ibuf)       // 100MHz input
);

// CPU Clock divider (÷12)
logic [3:0] cpu_div_count;
always_ff @(posedge nes_clk or posedge reset_ah) begin
    if (reset_ah) begin
        cpu_div_count <= '0;
        cpu_clk <= 1'b0;
    end else begin
        if (cpu_div_count == 11) begin
            cpu_div_count <= '0;
            cpu_clk <= ~cpu_clk;
        end else begin
            cpu_div_count <= cpu_div_count + 1;
        end
    end
end

// PPU Clock divider (÷4)
logic [3:0] ppu_div_count;
always_ff @(posedge nes_clk or posedge reset_ah) begin
    if (reset_ah) begin
        ppu_div_count <= '0;
        ppu_clk <= 0;
    end else begin
        if (ppu_div_count == 3) begin
            ppu_div_count <= '0;
            ppu_clk <= ~ppu_clk;
        end else begin
            ppu_div_count <= ppu_div_count + 1;
        end
    end
end
    
/* HDMI */
logic [3:0] VGA_R, VGA_G, VGA_B;
logic VGA_HS, VGA_VS, vde;
logic locked;

hdmi_tx_0 vga_to_hdmi (
    //Clocking and Reset
    .pix_clk(clk_25MHz),
    .pix_clkx5(clk_125MHz),
    .pix_clk_locked(locked),
    .rst(reset_ah),
    //Color and Sync Signals
    .red(VGA_R), // VGA_R
    .green(VGA_G), // VGA_G
    .blue(VGA_B), // VGA_B
    .hsync(VGA_HS),
    .vsync(VGA_VS),
    .vde(vde),
    //aux Data (unused)
    .aux0_din(4'b0),
    .aux1_din(4'b0),
    .aux2_din(4'b0),
    .ade(1'b0),
    //Differential outputs
    .TMDS_CLK_P(hdmi_tmds_clk_p),          
    .TMDS_CLK_N(hdmi_tmds_clk_n),          
    .TMDS_DATA_P(hdmi_tmds_data_p),         
    .TMDS_DATA_N(hdmi_tmds_data_n)          
);

/* Memory Interface Signals */
logic [7:0] databus;
logic [10:0] mirrored_addr;  // For 2KB RAM mirroring

/* BRAM for 2KB internal RAM */
logic [7:0] bram_din, bram_dout;
logic [10:0] bram_addr;  // 11 bits for 2KB
logic bram_we;

// BRAM instance
blk_mem_gen_0 internal_ram (
    .addra(bram_addr),    // 11-bit address for 2KB
    .dina(bram_din),      // 8-bit data in
    .wea(bram_we),        // Write enable
    .clka(nes_clk),       // Clock
    .douta(bram_dout),    // 8-bit data out
    .ena(1'b1)            // always enabled
);

/* ROM interface */
logic [7:0] rom_data;
ROM game_rom (
    .clk(nes_clk),
    .reset(reset_ah),
    .rom_addr(cpu_addr),
    .dout(rom_data)
);

/* USB keycodes from MicroBlaze */
logic [31:0] keycode;
assign keycode = keycode0_gpio;

mb_snes mb_snes_i (
    .clk_100MHz(clk_ibuf),
    .gpio_usb_int_tri_i(gpio_usb_int_tri_i),
    .gpio_usb_keycode_0_tri_o(keycode0_gpio),
    .gpio_usb_keycode_1_tri_o(keycode1_gpio),
    .gpio_usb_rst_tri_o(gpio_usb_rst_tri_o),
    .reset_rtl_0(~reset_ah),
    .uart_rtl_0_rxd(uart_rtl_0_rxd),
    .uart_rtl_0_txd(uart_rtl_0_txd),
    .usb_spi_miso(usb_spi_miso),
    .usb_spi_mosi(usb_spi_mosi),
    .usb_spi_sclk(usb_spi_sclk),
    .usb_spi_ss(usb_spi_ss)
);

/* CPU */
logic cpu_reset, cpu_enable, cpu_rdy;
logic cpu_irq, cpu_nmi, cpu_read;
logic [15:0] cpu_addr;
logic [7:0] cpu_din, cpu_dout;

always_comb begin
    if (~cpu_read) begin    
        cpu_din = cpu_dout;  /* WRITING */
    end else begin          
        cpu_din = databus;   /* READING */
    end
    
    cpu_reset = reset_ah;
    cpu_enable = ~dma;
    cpu_rdy = 1'b1;
    cpu_irq = 1'b1;
    cpu_nmi = ~ppu_nmi;
end

T65 cpu (
    .mode(2'b0),        // 6502 mode
    .BCD_en(1'b0),      // BCD disabled
    .clk(cpu_clk),      // clock
    .res_n(reset_ah),  // cpu reset 
    .enable(cpu_enable),// enable
    .rdy(cpu_rdy),      // ready
    .IRQ_n(cpu_irq),    // interrupt request
    .NMI_n(cpu_nmi),    // non-maskable interrupt
    .R_W_n(cpu_read),   // read = 1, write = 0
    .A(cpu_addr),       // address
    .DI(cpu_din),       // data in
    .DO(cpu_dout)       // data out
);

/* PPU */
logic [15:0] ppu_addr;
logic [7:0] ppu_din, ppu_dout;
logic ppu_wren, ppu_nmi;

logic [7:0] ppu_oam_addr0, ppu_oam_addr1;
logic [7:0] ppu_oam_dout0, ppu_oam_dout1;
logic [7:0] ppu_oam_din1;
logic ppu_oam_wren1;

ppu ppu (
    .ppu_clk(ppu_clk),
    .nes_clk(nes_clk),
    .vga_clk(vga_clk),
    .cpu_clk(cpu_clk),
    .addr(ppu_addr),
    .din(ppu_din),
    .wren(ppu_wren),
    .dout(ppu_dout),
    .nmi(ppu_nmi),
    .oam_addr0(ppu_oam_addr0),
    .oam_dout0(ppu_oam_dout0),
    .oam_addr1(ppu_oam_addr1),
    .oam_din1(ppu_oam_din1),
    .oam_wren1(ppu_oam_wren1),
    .oam_dout1(ppu_oam_dout1),
    .VGA_R(VGA_R),
    .VGA_G(VGA_G),
    .VGA_B(VGA_B),
    .VGA_HS(VGA_HS),
    .VGA_VS(VGA_VS),
    .vde(vde),
    .debug(),
    .select(~reset_ah)
);

/* Controller */
logic [7:0] controller_dout;
controller controller (
    .clk(cpu_clk),
    .addr(cpu_addr),
    .keycode(keycode),
    .wren(~cpu_read),
    .dout(controller_dout)
);

/* OAM */
logic [7:0] oam_addr0, oam_addr1;
logic [7:0] oam_din0, oam_din1;
logic [7:0] oam_dout0, oam_dout1;
logic oam_wren0, oam_wren1;

OAM OAM (
    .clock_a(vga_clk),
    .address_a(oam_addr0),
    .data_a(oam_din0),
    .wren_a(oam_wren0),
    .q_a(oam_dout0),
    .clock_b(nes_clk),
    .address_b(oam_addr1),
    .data_b(oam_din1),
    .wren_b(oam_wren1),
    .q_b(oam_dout1)
); 

/* DMA */
logic [9:0] dma_count;
logic [7:0] dma_reg;
logic dma;
logic dma_finish;
logic [15:0] dma_ram_addr;
logic [7:0] dma_oam_addr;

always_ff @ (posedge cpu_clk) begin
	dma = ~dma_finish;
end

always_ff @ (posedge nes_clk) begin
	if (cpu_addr == 16'h4014 && ~dma) begin
		dma_finish <= 0;
		dma_count <= 0;
		dma_reg <= cpu_dout;
		dma_ram_addr <= {dma_reg, 8'd0};
		dma_oam_addr <= 0;
	end
	
	if (dma) begin
		if (dma_count == 10'h200) begin
			dma_finish <= 1;
		end else begin
			dma_count <= dma_count + 1'd1;
			case (dma_count[0])
				1'b0 :
					begin
						dma_ram_addr <= dma_ram_addr + 1'd1;
					end
				1'b1:
					begin
						dma_oam_addr <= dma_oam_addr + 1'd1;
					end
			endcase
		end
	end
end


// OAM connections
always_comb begin
    // Default PPU connections
    oam_addr0 = ppu_oam_addr0;
    oam_din0 = '0;
    oam_wren0 = '0;
    ppu_oam_dout0 = oam_dout0;
    
    // DMA or PPU control of OAM port 1
    if (dma) begin
        oam_addr1 = dma_oam_addr;
        oam_din1 = bram_dout;  // Data from RAM during DMA
        oam_wren1 = (dma_count < 10'h200);
    end else begin
        oam_addr1 = ppu_oam_addr1;
        oam_din1 = ppu_oam_din1;
        oam_wren1 = ppu_oam_wren1;
    end
    ppu_oam_dout1 = oam_dout1;
end

// Memory address mirroring logic
always_comb begin
    // Mirror RAM addresses (0x0000-0x07FF mirrors to 0x0800-0x1FFF)
    mirrored_addr = cpu_addr[10:0];
end

// Memory access logic
always_comb begin
    // Default to RAM access
    bram_addr = cpu_addr[10:0];  // RAM mirroring
    bram_din = cpu_dout;
    bram_we = ~cpu_read & ~dma;
    databus = bram_dout;
    
    // Override for specific memory ranges
    if (cpu_addr >= 16'h8000) begin
        // ROM access (including reset vector)
        databus = rom_data;
    end
    else if (cpu_addr >= 16'h2000 && cpu_addr <= 16'h3FFF) begin
        // PPU registers
        ppu_addr = {cpu_addr[15:3], 3'b000};  // Mirror every 8 bytes
        ppu_din = cpu_dout;
        ppu_wren = ~cpu_read & ~dma;
        databus = ppu_dout;
    end
    else if (cpu_addr >= 16'h4016 && cpu_addr <= 16'h4017) begin
        // Controller
        databus = controller_dout;
    end
    
    // DMA override
    if (dma) begin
        bram_addr = dma_ram_addr[10:0];
        bram_we = 1'b0;
    end
end

endmodule