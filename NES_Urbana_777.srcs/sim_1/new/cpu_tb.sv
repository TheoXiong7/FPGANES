`timescale 1ns / 1ps
module T65_tb();
    // Clock and control signals
    logic clk;
    logic reset_n;
    logic enable;
    logic nmi_n;
    logic irq_n;
    logic rdy;
    logic [1:0] mode;  // 6502 mode = 0
    logic bcd_en;      // BCD disabled = 0
    logic r_w_n;       // Read = 1, Write = 0
    
    // Address and data buses
    logic [23:0] A;    // Using 24-bit address even though we only need 16
    logic [7:0] DI;    // Data input to CPU
    logic [7:0] DO;    // Data output from CPU
    
    // ROM signals
    logic [15:0] rom_addr;
    logic [7:0] rom_data;
    
    // Instantiate T65 CPU
    T65 cpu (
        .mode(2'b0),        // 6502 mode
        .BCD_en(1'b0),      // BCD disabled
        .clk(clk),
        .res_n(reset_n),    // Active low reset
        .enable(enable),
        .nmi_n(nmi_n),
        .irq_n(irq_n),
        .r_w_n(r_w_n),
        .A(A),
        .DI(DI),
        .DO(DO),
        .rdy(rdy)
    );
    
    // Instantiate ROM
    ROM rom (
        .clk(clk),
        .reset(~reset_n),
        .rom_addr(rom_addr),
        .dout(rom_data)
    );
    
    // Clock generation - 1MHz
    always begin
        clk = 0; #500;
        clk = 1; #500;
    end
    
    // Memory read handling
    assign rom_addr = A[15:0];
    
    always_comb begin
        if (r_w_n) begin  // Read operation
            if (A[15]) begin  // Upper memory space (ROM)
                DI = rom_data;
            end else begin
                DI = 8'h00;  // Default read value
            end
        end
    end
    
    // Test sequence
    initial begin
        // Initialize signals
        reset_n = 1'b0;  // Assert reset
        enable = 1'b1;
        nmi_n = 1'b1;
        irq_n = 1'b1;
        rdy = 1'b1;
        
        // Wait 5 clock cycles
        repeat(5) @(posedge clk);
        
        // Release reset
        reset_n = 1'b1;
        
        // Monitor CPU startup sequence
        repeat(20) begin
            @(posedge clk);
            $display("Time=%0t Reset=%b Addr=%h Data_In=%h R/W=%b",
                     $time, reset_n, A[15:0], DI, r_w_n);
        end
        
        // End simulation
        #10000;
        $finish;
    end
    
    // Additional monitoring
    initial begin
        $monitor("Time=%0t Reset=%b Enable=%b Addr=%h Data=%h R/W=%b",
                 $time, reset_n, enable, A[15:0], DI, r_w_n);
    end
    
    // Waveform dump
    initial begin
        $dumpfile("cpu_test.vcd");
        $dumpvars(0, T65_tb);
    end

endmodule