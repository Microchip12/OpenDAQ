module axi_frame_corrector #(
    parameter integer FRAME_WORDS = 8388608,   // beats per frame
    parameter integer DEPTH       = 64,     // small buffer depth (>= 10)
    parameter integer GAP_CYCLES  = 3       // enforced pause cycles after TLAST
)(
    input  wire        aclk,
    input  wire        aresetn,
    output reg [31:0] beats,

    // Upstream (from FSM)
    input  wire [31:0] s_tdata,
    input  wire        s_tvalid,
    input  wire        s_tlast,     // ignored
    output reg         s_tready,
    input wire [3:0] s_tkeep,

    // Downstream (to FIFO/DMA)
    output reg  [31:0] m_tdata,
    output reg         m_tvalid,
    output reg         m_tlast,
    output reg  [3:0]  m_tkeep,
    input  wire        m_tready
);

    // ----------------------------
    // Internal circular buffer
    // ----------------------------
    localparam PTRW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
    reg [31:0]    buf_data [0:DEPTH-1];
    reg [PTRW-1:0] wr_ptr, rd_ptr;
    reg [PTRW:0]   fill;

    // Frame and gap control
    reg [31:0] beat_count;
    reg [7:0]  gap_ctr;
    integer i;

    // ----------------------------
    // Main logic
    // ----------------------------
    always @(posedge aclk) begin
        if (!aresetn) begin
            wr_ptr     <= 0;
            rd_ptr     <= 0;
            fill       <= 0;
            beat_count <= 0;
            gap_ctr    <= 0;
            beats <=0;
            s_tready   <= 1'b1;

            m_tdata    <= 32'd0;
            m_tvalid   <= 1'b0;
            m_tlast    <= 1'b0;
            m_tkeep    <= 4'h0;
            for (i = 0; i < DEPTH; i = i + 1)begin
            buf_data[i] <= 32'd0;  //clear buf
            end

        end else begin
            //-----------------------------------
            // Always ready to take upstream data
            //-----------------------------------
            s_tready <= (fill < DEPTH);
            if (s_tvalid && s_tready) begin
                buf_data[wr_ptr] <= s_tdata;
                wr_ptr <= (wr_ptr == DEPTH-1) ? 0 : wr_ptr + 1'b1;
                fill   <= fill + 1'b1;
            end

            //-----------------------------------
            // Default downstream outputs
            //-----------------------------------
            m_tvalid <= 1'b0;
            m_tlast  <= 1'b0;
            m_tkeep  <= 4'h0;
            beats <= beat_count;

            //-----------------------------------
            // Handle enforced downstream gap
            //-----------------------------------
            if (gap_ctr != 0) begin
             m_tvalid <= 1'b0;
             m_tdata<=m_tdata;
                // No output beats during enforced idle
                gap_ctr <= gap_ctr - 1'b1;
                // (still receiving upstream data ? FIFO may fill slightly)
            end
            else if ((fill != 0) && m_tready) begin
                //-----------------------------------
                // Normal transmit path
                //-----------------------------------
                m_tdata  <= buf_data[rd_ptr];
                m_tvalid <= 1'b1;
                m_tkeep  <= 4'hF;
                m_tlast  <= (beat_count == FRAME_WORDS-1);

                // Pop one from FIFO
                rd_ptr <= (rd_ptr == DEPTH-1) ? 0 : rd_ptr + 1'b1;
                fill   <= fill - 1'b1;

                // TLAST and gap start
                if (beat_count == FRAME_WORDS-1) begin
                    beat_count <= 0;
                    gap_ctr    <= GAP_CYCLES[7:0] + 1;  // start enforced pause
                end else begin
                    beat_count <= beat_count + 1'b1;
                end
            end
        end
    end
endmodule