`timescale 1ns / 1ps

module can_tb;

    // Clock and reset
    reg clk;
    reg rst;
    
    // Transmitter signals
    reg tx_start;
    reg [10:0] tx_id;
    reg [3:0] tx_dlc;
    reg [63:0] tx_data;
    wire tx_busy;
    wire tx_done;
    
    // Receiver signals
    wire [10:0] rx_id;
    wire [3:0] rx_dlc;
    wire [63:0] rx_data;
    wire rx_valid;
    
    // Instantiate top module
    can_top dut (
        .clk(clk),
        .rst(rst),
        .tx_start(tx_start),
        .tx_id(tx_id),
        .tx_dlc(tx_dlc),
        .tx_data(tx_data),
        .tx_busy(tx_busy),
        .tx_done(tx_done),
        .rx_id(rx_id),
        .rx_dlc(rx_dlc),
        .rx_data(rx_data),
        .rx_valid(rx_valid)
    );
    
    // Generate clock
    always #5 clk = ~clk;
    
    // Test sequence
    initial begin
        // Initialize
        clk = 0;
        rst = 1;
        tx_start = 0;
        tx_id = 0;
        tx_dlc = 0;
        tx_data = 0;
        
        // Reset
        #100;
        rst = 0;
        #50;
        
        // Test 1: Send 1 byte
        tx_id = 11'h123;
        tx_dlc = 1;
        tx_data = 64'hAA;
        tx_start = 1;
        #10;
        tx_start = 0;
        
        // Wait for completion
        wait(tx_done);
        #100;
        
        // Test 2: Send 4 bytes  
        tx_id = 11'h456;
        tx_dlc = 4;
        tx_data = 64'hDEADBEEF;
        tx_start = 1;
        #10;
        tx_start = 0;
        
        wait(tx_done);
        #100;
        
        // End test
        $finish;
    end
    
    // Save waveforms
    initial begin
        $dumpfile("can_simple.vcd");
        $dumpvars(0, can_tb);
    end

endmodule