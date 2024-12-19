module controller (
    input clk,
    input [15:0] addr,
    input [31:0] keycode,
    input wren,
    output logic [7:0] dout // Changed to `logic` to allow continuous assignment
);

logic [7:0] data;
logic [7:0] dout_reg; // Internal signal to drive `dout`

always_ff @(posedge clk) begin
    if (16'h4016 <= addr && addr <= 16'h4017) begin
        if (wren) begin
            data <= controller_din << 1;
            dout_reg <= {7'h00, controller_din[7]};
        end else begin
            dout_reg <= {7'h00, data[7]};
            data <= data << 1;
        end
    end
end

// Continuous assignment to drive dout
assign dout = dout_reg;

logic [7:0] controller_din;

always_comb begin
    controller_din = 0;
    
    if (keycode[31:24] == 8'h1A || keycode[23:16] == 8'h1A || keycode[15:8] == 8'h1A || keycode[7:0] == 8'h1A) begin
        controller_din[3] = 1; // W --> UP
    end
    if (keycode[31:24] == 8'h16 || keycode[23:16] == 8'h16 || keycode[15:8] == 8'h16 || keycode[7:0] == 8'h16) begin
        controller_din[2] = 1; // S --> DOWN
    end
    if (keycode[31:24] == 8'h04 || keycode[23:16] == 8'h04 || keycode[15:8] == 8'h04 || keycode[7:0] == 8'h04) begin
        controller_din[1] = 1; // A --> LEFT
    end
    if (keycode[31:24] == 8'h07 || keycode[23:16] == 8'h07 || keycode[15:8] == 8'h07 || keycode[7:0] == 8'h07) begin
        controller_din[0] = 1; // D --> RIGHT
    end
    if (keycode[31:24] == 8'h0F || keycode[23:16] == 8'h0F || keycode[15:8] == 8'h0F || keycode[7:0] == 8'h0F) begin
        controller_din[7] = 1; // L --> A
    end
    if (keycode[31:24] == 8'h0E || keycode[23:16] == 8'h0E || keycode[15:8] == 8'h0E || keycode[7:0] == 8'h0E) begin
        controller_din[6] = 1; // K --> B
    end
    if (keycode[31:24] == 8'h0A || keycode[23:16] == 8'h0A || keycode[15:8] == 8'h0A || keycode[7:0] == 8'h0A) begin
        controller_din[5] = 1; // G --> SELECT
    end
    if (keycode[31:24] == 8'h0B || keycode[23:16] == 8'h0B || keycode[15:8] == 8'h0B || keycode[7:0] == 8'h0B) begin
        controller_din[4] = 1; // H --> START
    end
end

endmodule
