
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module can_rx(
    input        clk,
    input        rst,
    input        can_rx, // 1 = recessive , 0 = dominant
    output reg [10:0] rx_id,
    output reg [3:0]  rx_dlc,
    output reg [63:0] rx_data,
    output reg        rx_valid
);

// State encodings
localparam IDLE       = 4'd0;
localparam RX_ID      = 4'd1;
localparam RX_RTR     = 4'd2;
localparam RX_IDE     = 4'd3;
localparam RX_R0      = 4'd4;
localparam RX_DLC     = 4'd5;
localparam RX_DATA    = 4'd6;
localparam RX_CRC     = 4'd7;
localparam RX_ACK     = 4'd8;
localparam RX_ACK_DEL = 4'd9;
localparam RX_EOF     = 4'd10;

reg [3:0] state;
reg [3:0] bit_idx;
reg [3:0] byte_idx;
reg [3:0] dlc_index;
reg [3:0] crc_index;

//bit stuffing detection
reg [2:0] stuff_count;
reg       skip;
reg       last_bit;

// buffer for data/crc
reg [7:0] data_byte;
reg [14:0] rx_crc;
reg        rtr, ide, r0;

// Initialize outputs
initial begin
    rx_id = 11'b0;
    rx_dlc = 4'b0;
    rx_data = 64'b0;
    rx_valid = 1'b0;
end

// Main state machine
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= IDLE;
        rx_valid <= 1'b0;
        rx_id <= 11'b0;
        rx_dlc <= 4'b0;
        rx_data <= 64'b0;
        rtr <= 1'b0; 
        ide <= 1'b0; 
        r0 <= 1'b0;
        stuff_count <= 3'b0;
        skip <= 1'b0;
        last_bit <= 1'b1;
        bit_idx <= 4'b0;
        byte_idx <= 4'b0;
        dlc_index <= 4'd3;
        crc_index <= 4'd14;
        data_byte <= 8'b0;
        rx_crc <= 15'b0;
    end else begin
        // Auto-clear rx_valid after one cycle
        if (rx_valid) begin
            rx_valid <= 1'b0;
        end
        
        case (state)
            IDLE: begin
                // Wait for dominant SOF (0)
                if (can_rx === 1'b0) begin
                    state <= RX_ID;
                    stuff_count <= 3'b001;
                    skip <= 1'b0;
                    last_bit <= 1'b0;
                    bit_idx <= 4'd10;
                    rx_id <= 11'b0;
                    dlc_index <= 4'd3;
                    crc_index <= 4'd14;
                end
            end

            RX_ID: begin
                if (skip) begin
                    skip <= 1'b0;
                    stuff_count <= 3'b001;
                    last_bit <= can_rx;
                end else begin
                    if (can_rx === last_bit)
                        stuff_count <= stuff_count + 1;
                    else
                        stuff_count <= 3'b001;
                    
                    last_bit <= can_rx;
                    
                    if (stuff_count == 3'd5)
                        skip <= 1'b1;

                    if (can_rx === 1'b0 || can_rx === 1'b1) begin
                        rx_id[bit_idx] <= can_rx;
                    end
                    
                    if (bit_idx == 0) begin // WE RECEIVED LAST BIT OF  IDX
                        state <= RX_RTR;
                    end else begin
                        bit_idx <= bit_idx - 1;
                    end
                end
            end

            RX_RTR: begin
                if (skip) begin
                    skip <= 1'b0;
                    stuff_count <= 3'b001;
                    last_bit <= can_rx;
                end else begin
                    if (can_rx === last_bit)
                        stuff_count <= stuff_count + 1;
                    else
                        stuff_count <= 3'b001;
                    
                    last_bit <= can_rx;
                    
                    if (stuff_count == 3'd5)
                        skip <= 1'b1;

                    if (can_rx === 1'b0 || can_rx === 1'b1) begin
                        rtr <= can_rx;
                    end
                    state <= RX_IDE;
                end
            end

            RX_IDE: begin
                if (skip) begin
                    skip <= 1'b0;
                    stuff_count <= 3'b001;
                    last_bit <= can_rx;
                end else begin
                    if (can_rx === last_bit)
                        stuff_count <= stuff_count + 1;
                    else
                        stuff_count <= 3'b001;
                    
                    last_bit <= can_rx;
                    
                    if (stuff_count == 3'd5)
                        skip <= 1'b1;

                    if (can_rx === 1'b0 || can_rx === 1'b1) begin
                        ide <= can_rx;
                    end
                    state <= RX_R0;
                end
            end

            RX_R0: begin
                if (skip) begin
                    skip <= 1'b0;
                    stuff_count <= 3'b001;
                    last_bit <= can_rx;
                end else begin
                    if (can_rx === last_bit)
                        stuff_count <= stuff_count + 1;
                    else
                        stuff_count <= 3'b001;
                    
                    last_bit <= can_rx;
                    
                    if (stuff_count == 3'd5)
                        skip <= 1'b1;

                    if (can_rx === 1'b0 || can_rx === 1'b1) begin
                        r0 <= can_rx;
                    end
                    state <= RX_DLC;
                end
            end

            RX_DLC: begin
                if (skip) begin
                    skip <= 1'b0;
                    stuff_count <= 3'b001;
                    last_bit <= can_rx;
                end else begin
                    if (can_rx === last_bit)
                        stuff_count <= stuff_count + 1;
                    else
                        stuff_count <= 3'b001;
                    
                    last_bit <= can_rx;
                    
                    if (stuff_count == 3'd5)
                        skip <= 1'b1;
                               
                    if (can_rx === 1'b0 || can_rx === 1'b1) begin
                        rx_dlc[dlc_index] <= can_rx;
                    end
                    
                    if (dlc_index == 0) begin
                        if (rx_dlc == 4'b0000) begin
                            state <= RX_CRC;
                        end else begin
                            state <= RX_DATA;
                            byte_idx <= 4'b0;
                            bit_idx <= 4'd7;
                            rx_data <= 64'b0;
                            data_byte <= 8'b0;
                        end
                    end else begin
                        dlc_index <= dlc_index - 1;
                    end
                end
            end

            RX_DATA: begin
                if (skip) begin
                    skip <= 1'b0;
                    stuff_count <= 3'b001;
                    last_bit <= can_rx;
                end else begin
                    if (can_rx === last_bit)
                        stuff_count <= stuff_count + 1;
                    else
                        stuff_count <= 3'b001;
                    
                    last_bit <= can_rx;
                    
                    if (stuff_count == 3'd5)
                        skip <= 1'b1;

                    if (can_rx === 1'b0 || can_rx === 1'b1) begin
                        data_byte[bit_idx] <= can_rx;
                    end
                    
                    if (bit_idx == 0) begin
                        // Store the received byte
                        case(byte_idx)
                            0: rx_data[7:0]   <= data_byte;
                            1: rx_data[15:8]  <= data_byte;
                            2: rx_data[23:16] <= data_byte;
                            3: rx_data[31:24] <= data_byte;
                            4: rx_data[39:32] <= data_byte;
                            5: rx_data[47:40] <= data_byte;
                            6: rx_data[55:48] <= data_byte;
                            7: rx_data[63:56] <= data_byte;
                        endcase
            
                        if (byte_idx == (rx_dlc - 1)) begin
                            state <= RX_CRC;
                        end else begin
                            byte_idx <= byte_idx + 1;
                            data_byte <= 8'b0;
                            bit_idx <= 4'd7;
                        end
                    end else begin
                        bit_idx <= bit_idx - 1;
                    end
                end
            end

            RX_CRC: begin
                if (skip) begin
                    skip <= 1'b0;
                    stuff_count <= 3'b001;
                    last_bit <= can_rx;
                end else begin
                    if (can_rx === last_bit)
                        stuff_count <= stuff_count + 1;
                    else
                        stuff_count <= 3'b001;
                    
                    last_bit <= can_rx;
                    
                    if (stuff_count == 3'd5)
                        skip <= 1'b1;

                    if (can_rx === 1'b0 || can_rx === 1'b1) begin
                        rx_crc[crc_index] <= can_rx;
                    end
                    
                    if (crc_index == 0) begin
                        state <= RX_ACK;
                    end else begin 
                        crc_index <= crc_index - 1;
                    end
                end
            end

            RX_ACK: begin
                // ACK slot - wait one cycle
                state <= RX_ACK_DEL;
                stuff_count <= 3'b0;
                skip <= 1'b0;
            end 

            RX_ACK_DEL: begin
                state <= RX_EOF;
                bit_idx <= 4'd6;
            end

            RX_EOF: begin
                // EOF - 7 recessive bits
                if (bit_idx == 0) begin
                    rx_valid <= 1'b1;
                    state <= IDLE;
                end else begin
                    bit_idx <= bit_idx - 1;
                end
            end
            
            default: begin
                state <= IDLE;
                rx_valid <= 1'b0;
            end
        endcase
    end
end
endmodule
