`timescale 1 ps / 1 ps

// chr rom using dpram
module chrrom (
    input [12:0] address_a,
    input [12:0] address_b,
    input clock_a,
    input clock_b,
    input [7:0] data_a,
    input [7:0] data_b,
    input wren_a,
    input wren_b,
    output [7:0] q_a,
    output [7:0] q_b
);

    // Instantiate dual_port_ram_diff_clk with appropriate parameters
    dual_port_ram_diff_clk #(
        .ADDR_WIDTH_A(13),        // 13-bit address (8192 words)
        .DATA_WIDTH_A(8),         // 8-bit data
        .ADDR_WIDTH_B(13),        // 13-bit address
        .DATA_WIDTH_B(8),         // 8-bit data
        .INIT_FILE("../cart_init/chr.mif")  // Initialize from CHR ROM file
    ) chrrom_mem (
        .clock_a(clock_a),
        .clock_b(clock_b),
        
        // Port A
        .address_a(address_a),
        .data_a(data_a),
        .wren_a(wren_a),
        .q_a(q_a),
        
        // Port B
        .address_b(address_b),
        .data_b(data_b),
        .wren_b(wren_b),
        .q_b(q_b)
    );

endmodule