module can_tx(
    input        clk,
    input        rst,
    input        start,
    input [10:0] id, //  11 BIT CAN IDENTIFIER 
    input [3:0]  dlc, // DATA LENGTH CODE 0-8
    input [63:0] data,// UP TO 64 BITS DATA (8 BYTES)
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
        bit_index <= 0; 
         next_bit <= 1'b1; 
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
        cons_count <= 3'b0;
        bit_index <= 0; 
        end else if(f4_valid) begin
        f4_valid <= 1'b1; 
        end else begin
        f4_valid <= 0;
    end
end

// Bus Arbitration & Transmit 
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
            next_bit = f4_id[11 - bit_index]; // MSB first bit_index = 1 → transmit f4_id[10]
        end else if (bit_index == 12) begin
            next_bit = f4_rtr;
        end else if (bit_index == 13) begin
            next_bit = f4_ide;
        end else if (bit_index == 14) begin
            next_bit = f4_r0;
        end else if (bit_index >= 15 && bit_index < 19) begin
            next_bit = f4_dlc[18 - bit_index]; // DLC[3:0] DLC bits 18 - bit index 15
        end else if (bit_index >= 19 && bit_index < (19 + f4_dlc * 8)) begin //  between 19 to  83 is  data field  bits
            // Data bits: compute byte and bit inside byte
            byte_index = (bit_index - 19) / 8;
            bit_position = 7 - ((bit_index - 19) % 8); //Determine the bit position inside that byte global frame bit index → correct data bit:
            next_bit = f4_data[byte_index*8 + bit_position]; // Pick that bit from the 64-bit data
        end else if (bit_index >= (19 + f4_dlc * 8) && bit_index < (19 + f4_dlc * 8 + 15)) begin
            // CRC bits [14:0]
            crc_bit_index = 14 - (bit_index - (19 + f4_dlc * 8)); // For sending msb first  14 - 83 -83
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
        if (bit_index >= (19 + f4_dlc * 8 + 15 + 7)) begin // last bit transmitted
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
        last_bit <= 1'b1; // recessive state 
        bit_idx <= 4'b0;
        byte_idx <= 4'b0;
        dlc_index <= 4'd3;
        crc_index <= 4'd14;
        data_byte <= 8'b0;
        rx_crc <= 15'b0;
    end else begin
        
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
                    
                    if (stuff_count == 3'd5)  // ignore it  because of stuff bits
                        skip <= 1'b1;

                    if (can_rx === 1'b0 || can_rx === 1'b1) begin // recived bits is 0 or 1 as sometime unknown and high-impedance
                        rx_id[bit_idx] <= can_rx;
                    end
                    
                    if (bit_idx == 0) begin
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
                        if (rx_dlc == 4'b0000) begin // There are no data bytes at all
                            state <= RX_CRC;
                        end else begin
                            state <= RX_DATA;  // if dlc have data then begin
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
                            bit_idx <= 4'd7; // Start receiving the next byte from MSB first
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
