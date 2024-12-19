
module ROM(
    input clk,
    input reset,
    input [15:0] rom_addr,    
    output logic [7:0] dout   
);
    localparam PRG_SIZE = 32768;    // 32KB PRG ROM
    localparam CHR_SIZE = 8192;     // 8KB CHR ROM
    localparam TOTAL_SIZE = PRG_SIZE + CHR_SIZE;
    
    // ROM storage
    logic [7:0] rom [0:TOTAL_SIZE-1]; 
    
    // Address translation
    logic [15:0] translated_addr;
    always_comb begin
        if (rom_addr >= 16'h8000) begin
            translated_addr = (rom_addr - 16'h8000);
            if (translated_addr >= PRG_SIZE) begin
                translated_addr = translated_addr & (PRG_SIZE - 1);
            end
        end else if (rom_addr < 16'h2000) begin
            translated_addr = rom_addr + PRG_SIZE;
        end else begin
            translated_addr = '0;
        end
    end

    // Load ROM data
    initial begin
        int fd;
        fd = $fopen("supermariobros.nes", "rb");
        if (fd == 0) begin
            $display("ERROR: Could not open ROM file!");
            $stop;
        end
        
        void'($fread(rom, fd));
        $fclose(fd);
        $display("First few bytes of PRG ROM: %h %h %h %h", 
                 rom[0], rom[1], rom[2], rom[3]);
        $display("Reset vector at 0xFFFC: %h %h", 
                 rom[32762], rom[32763]);
    end
    
    // Output logic
    always_comb begin
        if (reset) begin
            dout = 8'h00;
        end else if (translated_addr < TOTAL_SIZE) begin
            dout = rom[translated_addr];
        end else begin
            dout = 8'h00;
        end
    end
endmodule
