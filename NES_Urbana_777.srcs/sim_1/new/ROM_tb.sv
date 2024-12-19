`timescale 1ns / 1ps
module ROM_tb();
    // Test signals matching ROM module interface
    logic clk;
    logic reset;
    logic [15:0] rom_addr;
    logic [7:0] dout;
    
    // Instantiate ROM module
    ROM rom_inst (
        .clk(clk),
        .reset(reset),
        .rom_addr(rom_addr),
        .dout(dout)
    );
    
    // Clock generation
    always begin
        clk = 0; #5;
        clk = 1; #5;
    end
    
    // Test stimulus
    initial begin
        // Initialize signals
        reset = 1;
        rom_addr = 16'h0000;
        
        // Wait 100ns
        #100;
        
        // Release reset
        reset = 0;
        #20;
        
        // Test 1: Check reset vector location (0xFFFC-0xFFFD)
        $display("Test 1: Checking Reset Vector");
        rom_addr = 16'hFFFC;
        #10;
        $display("Reset Vector Low Byte: %h", dout);
        rom_addr = 16'hFFFD;
        #10;
        $display("Reset Vector High Byte: %h", dout);
        
        // Test 2: Read first few bytes of PRG ROM
        $display("\nTest 2: Reading PRG ROM start");
        rom_addr = 16'h8000;
        #10;
        $display("PRG ROM First Byte: %h", dout);
        rom_addr = 16'h8001;
        #10;
        $display("PRG ROM Second Byte: %h", dout);
        
        // Test 3: Read CHR ROM
        $display("\nTest 3: Reading CHR ROM");
        rom_addr = 16'h0000;
        #10;
        $display("CHR ROM First Byte: %h", dout);
        rom_addr = 16'h0010;
        #10;
        $display("CHR ROM at 0x0010: %h", dout);
        
        // Test 4: Test address mirroring
        $display("\nTest 4: Testing address mirroring");
        rom_addr = 16'h8000;
        #10;
        $display("Data at 0x8000: %h", dout);
        rom_addr = 16'hC000;  // Should mirror 0x8000
        #10;
        $display("Data at 0xC000 (mirror): %h", dout);
        
        // End simulation
        #100;
        $display("\nTestbench completed");
        $finish;
    end

    // Optional: Waveform dump
    initial begin
        $dumpfile("rom_test.vcd");
        $dumpvars(0, ROM_tb);
    end
    
    // Monitor address translations
    always @(rom_addr) begin
        #1; // Small delay to let combinatorial logic settle
        $display("Time=%0t ROM Access: Address=%h Data=%h", 
                 $time, rom_addr, dout);
    end

endmodule