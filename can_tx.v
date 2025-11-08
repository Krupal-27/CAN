`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/08/2025 11:03:12 AM
// Design Name: 
// Module Name: can_tx
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module can_tx(
    input        clk,
    input        rst,
    input        start,
    input [10:0] id, //  11 BIT CAN IDENTIFIER 
    input [3:0]  dlc, // DATA LENGTH CODE 0-8
    input [63:0] data,// UP TO &$ BITS DATA (8 BYTES)
    output reg   tx,
    output reg   busy,
    output reg   done
);

// STAGE 1 Frame Loader (fl)
reg f1_valid;
reg [10:0] f1_id;
reg [3:0]  f1_dlc;
reg [63:0] f1_data;

// STAGE 2 Frame Generator (f2) sg
reg f2_valid;
reg         f2_sof; // Start of Frame
reg [10:0]  f2_id;  // 11 BIT CAN IDENTIFIER
reg         f2_rtr; // Remote Transmission Request
reg         f2_ide; // Identifier 
reg         f2_r0;  // Reserved bit always 0
reg [3:0]   f2_dlc; // DATA LENGTH CODE 0-8
reg [63:0]  f2_data;// UP TO 64 BITS DATA (8 BYTES)

//STAGE 3 CRC calculator  
reg         f3_valid;
reg [10:0]  f3_id;
reg          f3_rtr;
reg          f3_ide;
reg          f3_r0;
reg [3:0]   f3_dlc;
reg [63:0]  f3_data;
reg [14:0]  f3_crc; // 15 BIT CRC

// STAGE 4 Bit Stuffing 
reg         f4_valid;
reg[10:0] f4_id;
reg         f4_rtr;
reg         f4_ide;
reg         f4_r0;
reg [3:0]  f4_dlc;
reg [63:0] f4_data;
reg [14:0] f4_crc;
reg [2:0]  cons_count; // consecutive bit counter counter same 5 bits 
reg        last_bit;   // last bit value
reg [6:0]  bit_index;   // which bit transfering now why 7 because 
                        // SOF+Header (19) + Data (64) + CRC (15) = 98 bits

reg bus_busy;

// Module-level declarations - FIXED
reg next_bit;               // next bit to transmit
reg [7:0] byte_index;       // for data byte calculation - CHANGED from 'byte'
reg [3:0] bit_position;     // for bit within a byte - CHANGED from 'bit'  
reg [3:0] crc_bit_index;    // for CRC bit calculation - CHANGED from 'crc_bit'

//STAGE 1 Frame Loader
always @(posedge clk or posedge rst) begin
    if(rst) begin
        f1_valid <= 0;
    end else if(start) begin
        f1_valid <= 1;
        f1_id    <= id;
        f1_dlc   <= dlc;
        f1_data  <= data;
    end else begin
       f1_valid <= 0;// CLEAR FLAG RESET  TO ZERO AGAIN  
end
end

// STAGE 2 Frame Generator
always @(posedge clk or posedge rst) begin
    if(rst) begin
        f2_valid <= 0;
    end else if(f1_valid) begin
        f2_valid <= 1'b1;
        f2_sof  <= 1'b0; // Start of Frame is always dominant (0)
        f2_id   <= f1_id;
        f2_rtr  <= 1'b0; // 0 = Data Frame, 1= Remote Frame
        f2_ide  <= 1'b0;  // 0 = Standard Identifier (11 bits)
        f2_r0   <= 1'b0; 
        f2_dlc  <= f1_dlc;
        f2_data <= f1_data;
    end else begin
        f2_valid <= 0;
    end
end

// STAGE 3 CRC Calculator
always @(posedge clk or posedge rst) begin
    if(rst) begin
        f3_valid <= 0;
        end else if(f2_valid) begin
        f3_valid <= 1'b1; // PASS TO NEXT STAGE
        f3_id    <= f2_id;
        f3_rtr   <= f2_rtr;
        f3_ide   <= f2_ide;
        f3_r0    <= f2_r0;
        f3_dlc   <= f2_dlc;
        f3_data  <= f2_data;
        f3_crc   <= {f2_id[0] ^ f2_dlc[0] ^ f2_data[0], 14'b0};
        // SIMPLIFIED CRC CALCULATION FOR DEMO PURPOSES 
        // ADD THIS CALCULATION WITH ACTUAL CRC 15 bit 
    end 
    else begin
        f3_valid <= 0;
    end
end

// stage 4 Bit Stuffing
always @(posedge clk or posedge rst) begin
    if(rst) begin
        f4_valid <= 0;
        cons_count <= 0;
        last_bit <= 1'b1; // Recessive stage  nothin is transmitted yet
        bit_index <= 0; // around 98 bits total, less than 127 - so 7 bits is plenty
         next_bit <= 1'b1; // ADD THIS LINE
    end else if(f3_valid) begin
        f4_valid <= 1'b1; // PASS TO NEXT STAGE
        f4_id    <= f3_id;
        f4_rtr   <= f3_rtr;
        f4_ide   <= f3_ide;
        f4_r0    <= f3_r0;
        f4_dlc   <= f3_dlc;
        f4_data  <= f3_data;
        f4_crc   <= f3_crc;
        // initialize stuff counter  
        last_bit <= 1'b1; // Recessive stage before SOF
        cons_count <= 3 'b0;
        bit_index <= 0; 
        end else if(f4_valid) begin
        f4_valid <= 1'b1; // KEEP VALID
        end else begin
        f4_valid <= 0;
    end
end

// Bus Arbitration & Transmit - FIXED VERSION
always @(posedge clk or posedge rst) begin
    if(rst) begin
        tx <= 1'b1; // Recessive state
        busy <= 1'b0;
        done <= 1'b0;
        bus_busy <= 1'b0;
        bit_index <= 0;
        cons_count <= 0;
        last_bit <= 1'b1;
        next_bit <= 1'b1;
    end else if(f4_valid && !bus_busy) begin
        // Begin transmission - first bit is SOF dominant (0)
        busy <= 1'b1;
        done <= 1'b0;
        bus_busy <= 1'b1;
        tx <= 1'b0; // SOF dominant 
        last_bit <= 1'b0;
        cons_count <= 3'b1;
        bit_index <= 7'b1; // move to next bit
        next_bit <= 1'b0;
    end else if (bus_busy) begin
        // Determine next bit to transmit
        if (bit_index == 0) begin
            next_bit = 1'b0; // SOF
        end else if (bit_index <= 11) begin
            next_bit = f4_id[11 - bit_index]; // ID MSB first
        end else if (bit_index == 12) begin
            next_bit = f4_rtr;
        end else if (bit_index == 13) begin
            next_bit = f4_ide;
        end else if (bit_index == 14) begin
            next_bit = f4_r0;
        end else if (bit_index >= 15 && bit_index < 19) begin
            next_bit = f4_dlc[18 - bit_index]; // DLC[3:0]
        end else if (bit_index >= 19 && bit_index < (19 + f4_dlc * 8)) begin
            // Data bits: compute byte and bit inside byte
            byte_index = (bit_index - 19) / 8;
            bit_position = 7 - ((bit_index - 19) % 8);
            next_bit = f4_data[byte_index*8 + bit_position];
        end else if (bit_index >= (19 + f4_dlc * 8) && bit_index < (19 + f4_dlc * 8 + 15)) begin
            // CRC bits [14:0]
            crc_bit_index = 14 - (bit_index - (19 + f4_dlc * 8));
            next_bit = f4_crc[crc_bit_index];
        end else begin
            next_bit = 1'b1; // Recessive after CRC (default)
        end

        // Bit Stuffing Logic
        if (next_bit == last_bit) begin
            cons_count <= cons_count + 1;
        end else begin
            cons_count <= 3'b1; // start new bit sequence
        end
        
        if (cons_count == 5) begin
            // Insert Stuff Bit
            tx <= ~last_bit; // stuff bit is opposite of last bit
            last_bit <= ~last_bit;
            cons_count <= 3'b1; 
            // Don't increment bit_index when stuffing
        end else begin
            tx <= next_bit;
            last_bit <= next_bit;
            bit_index <= bit_index + 1;
        end

        // End of frame condition  
        if (bit_index >= (19 + f4_dlc * 8 + 15 + 7)) begin // account for possible stuff bits
            busy <= 1'b0;
            done <= 1'b1;
            bus_busy <= 1'b0;
            tx <= 1'b1; // Recessive state
            bit_index <= 0;
        end
    end else begin
        if (done) begin
            done <= 1'b0; // clear done flag after one cycle
        end
        // When not busy, ensure TX is recessive
        if (!bus_busy) begin
            tx <= 1'b1;
        end
    end
end
endmodule


