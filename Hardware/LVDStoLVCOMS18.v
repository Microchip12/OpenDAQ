`timescale 1ns / 1ps

module lvds_to_lvcmos_with_clk (
    // Differential LVDS inputs
    input  wire LVDS_P,
    input  wire LVDS_N,
    input  wire CLK_P,
    input  wire CLK_N,
    input wire clock_en,
   
    
    // Outputs
    output reg  LVCMOS_OUT,
    output wire CLK_OUT
  
);
    wire lvds_data;
    wire lvds_clk_raw;
    wire lvds_clk_bufr;    // Regional clock for IOB capture
    wire lvds_clk_bufg;    // Global clock for distribution
    
    //=========================================================================
    // LVDS Input Buffers
    //=========================================================================
    IBUFDS #(
        .DIFF_TERM("FALSE"),      // External termination on board
        .IOSTANDARD("LVDS_25")
    ) ibufds_data (
        .I (LVDS_P),
        .IB(LVDS_N),
        .O (lvds_data)
    );
    
    IBUFDS #(
        .DIFF_TERM("FALSE"),
        .IOSTANDARD("LVDS_25")
    ) ibufds_clk (
        .I (CLK_P),
        .IB(CLK_N),
        .O (lvds_clk_raw)
    );
    
    //=========================================================================
    // BUFR - Regional Clock Buffer for Local IOB Capture
    // This stays in the same clock region as the input, providing low-skew
    // clock distribution for the IOB register
    //=========================================================================
    BUFR #(
        .BUFR_DIVIDE("BYPASS"),   // No division, pass through 400 MHz
        .SIM_DEVICE("7SERIES")    // For Zynq-7000 (Zedboard)
    ) bufr_lvds_clk (
        .I  (lvds_clk_raw),
        .O  (lvds_clk_bufr),
        .CE (clock_en),               // Always enabled
        .CLR(1'b0)                // Not cleared
    );
    
    //=========================================================================
    // Sample data on BUFR clock in IOB
    // This register stays in the same region as BUFR for optimal capture
    //=========================================================================
    (* IOB = "TRUE" *)  // Force into IO register
    always @(posedge lvds_clk_bufr) begin
        LVCMOS_OUT <= lvds_data;
    end
    
    //=========================================================================
    // BUFG - Global Clock Buffer for Chip-Wide Distribution
    // Takes the regional BUFR output and distributes it globally
    // This allows AXI_Adapter and other logic anywhere on chip to use the clock
    //=========================================================================
    
    
    // Output the global clock for rest of design
    assign CLK_OUT = lvds_clk_bufr;
    
endmodule