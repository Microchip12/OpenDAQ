
`timescale 1ns / 1ps
`default_nettype wire
module AXI_Adapter(
    input fclk,
    input Akadc,
    input tready,
    input reset,
    output reg [3:0] tkeep,
    output reg tlast,
    output reg [31:0] tdata,
    output reg tvalid
);

    parameter BURST_SIZE = 32;  
    parameter send_state = 2'b11;
    parameter collect_state = 2'b00;
    parameter catch_state = 2'b01;
    parameter num_words = 24'd16000000;

    parameter FRAME_WORDS = 8388608;
    reg post_last_holdoff;
    reg accepted_beat;

    reg [1:0] curr_state, next_state;
    reg [31:0] hold;
    reg hold4;
    reg [5:0] bitcounter;
    reg [23:0] word_counter;
    reg [3:0] packet_counter;  
    reg tlast_pending;  
    reg [4:0] cyclecount;
    reg transferred_flag;
    reg [31:0]hold5;
    reg clk_prev;
    reg [23:0]transfers;
    reg pending_valid;

    always @(*) begin
        case(curr_state)
            send_state:      next_state = catch_state;
            collect_state:   next_state = (bitcounter == 6'd31) ? send_state : collect_state;
            catch_state:     next_state = collect_state;
            default:         next_state = collect_state;
        endcase
    end

    always @(posedge fclk) begin
        if (reset) begin
            curr_state <= collect_state;
            cyclecount <= 5'd0;
        end else begin
            curr_state <= next_state;
        end
    end
       
    always @(posedge fclk) begin
        if (reset) begin
            hold <= 32'd0;
            transferred_flag <= 1'b0;
            hold5 <= 32'd0;
            bitcounter <= 6'd0;
            word_counter <= 0;
            packet_counter <= 0;
            tvalid <= 1'b0;
            tkeep <= 4'b0000;
            tlast <= 1'b0;
            tlast_pending <= 1'b0;
            tdata <= 32'd0;
            hold4 <= 1'b0;
            pending_valid <= 1'b0;
            transfers <= 1'd0;
            post_last_holdoff <= 1'b0;
            accepted_beat <= 1'b0;
        end else begin

        accepted_beat <= (tvalid && tready);
        
        if (tvalid && !tready) begin
            hold <= hold;
            bitcounter <= bitcounter;
            tvalid <= tvalid;
            tdata <= tdata;
            tkeep <= tkeep;
            tlast <= tlast;
            hold4 <= Akadc;
            pending_valid <= 1'b1;
        end else begin

            if (post_last_holdoff) begin
                tvalid <= 1'b0;
                tlast  <= 1'b0;
                post_last_holdoff <= 1'b0;
            end else begin
        
            case (curr_state)
                send_state: begin
                    
                    tkeep  <= 4'b1111; 
					tvalid <=1'b1;
                    if (tready) begin
                        tdata <= hold;
                        tkeep <= 4'hF;
						//tvalid <= 1'b1;
                        transfers <= transfers + 1'd1;

                        if (transfers == FRAME_WORDS - 1) begin
                            tlast        <= 1'b1;
                            word_counter <= 24'd0;
                            transfers    <= 24'd0;
                            post_last_holdoff <= 1'b1;
                        end else begin
                            tlast        <= 1'b0;
                            word_counter <= word_counter + 1'd1;
                        end

                        hold5 <= 32'b0;
                        hold4 <= Akadc;
                        bitcounter <= 6'd0;
                        transferred_flag <= 1'b1;
                    end
                end

                collect_state: begin
                    tvalid <= 1'b0;            // 
                    tlast  <= 1'b0;

                    if (transferred_flag) begin
                        // keep transfers stable here (counted only in send_state)
                        transferred_flag <= 1'b0;
                    end

                    if (pending_valid) begin
                        hold           <= {Akadc, hold4, hold[31:2]};
                        pending_valid  <= 1'b0;
                        bitcounter     <= bitcounter + 2;
                    end else begin
                        hold           <= {Akadc, hold[31:1]};
                        bitcounter     <= bitcounter + 1;
                    end
                end

                catch_state: begin
                    tvalid <= 1'b0;            // 
                    tlast  <= 1'b0;

                    hold <= {Akadc, hold4, 30'b0};
                    bitcounter <= bitcounter + 2'd2;
                end

                default: begin
                    tvalid <= 1'b0; 
                    tkeep  <= 4'b0000;
                    tlast  <= 1'b0;
                end
            endcase
            end
        end
        end
    end
endmodule