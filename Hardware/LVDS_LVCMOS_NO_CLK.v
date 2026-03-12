`timescale 1ns / 1ps

module lvds_to_lvcmos_no_clk (
    // Differential LVDS Data Inputs
    input        LVDS_P,      // LVDS positive input (data)
    input        LVDS_N,
    input wire clk,      // LVDS negative input (data)
    
    // Differential LVDS Clock Inputs
        // LVDS negative input (clock)
    
    // Single-ended LVCMOS Outputs
    output    reg   LVCMOS_OUT // Single-ended LVCMOS data output
         // Single-ended LVCMOS clock output
);
wire lvds_data;
// Instantiate IBUFDS for the data conversion (LVDS to LVCMOS)
IBUFDS #(
    .DIFF_TERM("FALSE"),          // Differential Termination disabled for data
    .IBUF_LOW_PWR("FALSE"),       // Low power setting
    .IOSTANDARD("LVDS2.5")           // Specify the I/O standard
) ibufds_data_inst (
    .I(LVDS_P),                    // Differential positive input (data)
    .IB(LVDS_N),                   // Differential negative input (data)
    .O(lvds_data)                 // Single-ended output (data)
);

(* IOB = "TRUE" *)  // Force into IO register
    always @(posedge clk) begin
        LVCMOS_OUT <= lvds_data;
    end



endmodule
