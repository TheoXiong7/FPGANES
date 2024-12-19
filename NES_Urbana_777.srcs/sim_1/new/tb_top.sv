`timescale 1ns / 1ps

module NES_Urbana_tb();
    // Clock generation
    logic CLK;
    logic reset_rtl_0;
    
    // USB signals
    logic [0:0] gpio_usb_int_tri_i;
    logic gpio_usb_rst_tri_o;
    logic usb_spi_miso;
    logic usb_spi_mosi;
    logic usb_spi_sclk;
    logic usb_spi_ss;
    
    // UART
    logic uart_rtl_0_rxd;
    logic uart_rtl_0_txd;
    
    // HDMI
    logic hdmi_tmds_clk_n;
    logic hdmi_tmds_clk_p;
    logic [2:0] hdmi_tmds_data_n;
    logic [2:0] hdmi_tmds_data_p;
    
    // HEX displays
    logic [7:0] hex_segA;
    logic [3:0] hex_gridA;
    logic [7:0] hex_segB;
    logic [3:0] hex_gridB;

    // Instantiate NES module
    NES_Urbana dut (
        .CLK(CLK),
        .reset_rtl_0(reset_rtl_0),    // Direct connection, no inversion
        .gpio_usb_int_tri_i(gpio_usb_int_tri_i),
        .gpio_usb_rst_tri_o(gpio_usb_rst_tri_o),
        .usb_spi_miso(usb_spi_miso),
        .usb_spi_mosi(usb_spi_mosi),
        .usb_spi_sclk(usb_spi_sclk),
        .usb_spi_ss(usb_spi_ss),
        .uart_rtl_0_rxd(uart_rtl_0_rxd),
        .uart_rtl_0_txd(uart_rtl_0_txd),
        .hdmi_tmds_clk_n(hdmi_tmds_clk_n),
        .hdmi_tmds_clk_p(hdmi_tmds_clk_p),
        .hdmi_tmds_data_n(hdmi_tmds_data_n),
        .hdmi_tmds_data_p(hdmi_tmds_data_p),
        .hex_segA(hex_segA),
        .hex_gridA(hex_gridA),
        .hex_segB(hex_segB),
        .hex_gridB(hex_gridB)
    );

    // Clock generation - 100MHz
    always begin
        CLK = 0; #5;
        CLK = 1; #5;
    end

    // CPU signals for monitoring
    logic [15:0] monitored_cpu_addr;
    logic [7:0] monitored_cpu_data;
    logic monitored_cpu_read;
    logic monitored_cpu_clk;
    logic monitored_locked;

    // Connect monitoring signals
    assign monitored_cpu_addr = dut.cpu_addr[15:0];
    assign monitored_cpu_data = dut.cpu_din;
    assign monitored_cpu_read = dut.cpu_read;
    assign monitored_cpu_clk = dut.cpu_clk;
    assign monitored_locked = dut.locked;

    // Test stimulus
    initial begin
        // Initialize signals
        reset_rtl_0 = 1;  // Start with reset inactive
        gpio_usb_int_tri_i = 0;
        uart_rtl_0_rxd = 1;
        usb_spi_miso = 0;

        // Wait for initial settle
        #100;

        // Wait for clock wizard to lock with timeout
        repeat (1000) @(posedge CLK);
        if (!monitored_locked) begin
            $display("ERROR: Clock wizard failed to lock.");
            $finish;
        end
        $display("Clock wizard locked at time %0t", $time);
        #100;

        // Assert reset (active low)
        reset_rtl_0 = 0;
        $display("Reset asserted at time %0t", $time);
        
        // Wait for 10 CPU clock cycles
        repeat(10) @(posedge monitored_cpu_clk);
        
        // Release reset
        reset_rtl_0 = 1;
        $display("Reset released at time %0t", $time);

        // Monitor initial CPU operations
        repeat(20) begin
            @(posedge monitored_cpu_clk);
            $display("Time=%0t CPU Addr=%h Data=%h Read=%b",
                     $time, monitored_cpu_addr, monitored_cpu_data, monitored_cpu_read);
        end

        // Check reset vector access
        repeat(100) begin
            @(posedge monitored_cpu_clk);
            if (monitored_cpu_addr == 16'hFFFC || monitored_cpu_addr == 16'hFFFD) begin
                $display("Reset Vector Read at time %0t: Addr=%h Data=%h",
                    $time, monitored_cpu_addr, monitored_cpu_data);
            end
        end

        // Print final status
        $display("\nTest completed");
        $display("Final CPU state:");
        $display("  Address: %h", monitored_cpu_addr);
        $display("  Data:    %h", monitored_cpu_data);
        $display("  Read:    %b", monitored_cpu_read);
        $display("HexA display: %h", hex_segA);
        $display("HexB display: %h", hex_segB);
        
        #1000;
        $finish;
    end

    // Waveform dumping
    initial begin
        $dumpfile("nes_test.vcd");
        $dumpvars(0, NES_Urbana_tb);
    end

endmodule
