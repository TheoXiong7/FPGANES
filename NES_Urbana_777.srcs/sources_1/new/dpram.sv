// DP RAMMMMMMMMMMMMMMMM

module dual_port_ram_diff_clk #(
    parameter int ADDR_WIDTH_A = 8,
    parameter int DATA_WIDTH_A = 8,
    parameter int ADDR_WIDTH_B = 8,
    parameter int DATA_WIDTH_B = 8,
    parameter string INIT_FILE = ""
)(
    input logic clock_a,
    input logic clock_b,
    
    // Port A
    input logic [ADDR_WIDTH_A-1:0] address_a,
    input logic [DATA_WIDTH_A-1:0] data_a,
    input logic wren_a,
    output logic [DATA_WIDTH_A-1:0] q_a,
    
    // Port B
    input logic [ADDR_WIDTH_B-1:0] address_b,
    input logic [DATA_WIDTH_B-1:0] data_b,
    input logic wren_b,
    output logic [DATA_WIDTH_B-1:0] q_b
);
    // Xilinx Block Memory (BRAM) instantiation with different clocks
    (* ram_style = "block" *) logic [DATA_WIDTH_A-1:0] ram_memory_a [0:(2**ADDR_WIDTH_A)-1];
    (* ram_style = "block" *) logic [DATA_WIDTH_B-1:0] ram_memory_b [0:(2**ADDR_WIDTH_B)-1];
    
//    // Optional initialization from file
//    initial begin
//        if (INIT_FILE != "") begin
//            $readmemh(INIT_FILE, ram_memory_a);
//            $readmemh(INIT_FILE, ram_memory_b);
//        end
//    end
    
    // Port A Read/Write logic
    always_ff @(posedge clock_a) begin
        if (wren_a) begin
            ram_memory_a[address_a] <= data_a;
        end
        q_a <= ram_memory_a[address_a];
    end
    
    // Port B Read/Write logic
    always_ff @(posedge clock_b) begin
        if (wren_b) begin
            ram_memory_b[address_b] <= data_b;
        end
        q_b <= ram_memory_b[address_b];
    end
endmodule