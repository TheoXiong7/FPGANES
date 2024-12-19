`timescale 1 ps / 1 ps

// OAM using DPRAMmmm
module OAM (
   input [7:0] address_a,
   input [7:0] address_b,
   input clock_a,
   input clock_b,
   input [7:0] data_a,
   input [7:0] data_b,
   input wren_a,
   input wren_b,
   output [7:0] q_a,
   output [7:0] q_b
);

   // Instantiate dual_port_ram_diff_clk with parameters for 256-byte OAM
   dual_port_ram_diff_clk #(
       .ADDR_WIDTH_A(8),     // 8-bit address (256 bytes)
       .DATA_WIDTH_A(8),     // 8-bit data
       .ADDR_WIDTH_B(8),     // 8-bit address
       .DATA_WIDTH_B(8)      // 8-bit data
   ) oam_mem (
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