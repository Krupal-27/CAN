`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/08/2025 11:04:13 AM
// Design Name: 
// Module Name: can_top
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


module can_top (
    input clk,
    input rst,
    input tx_start,
    input [10:0] tx_id,
    input [3:0] tx_dlc,
    input [63:0] tx_data,
    output tx_busy,
    output tx_done,
    output [10:0] rx_id,
    output [3:0] rx_dlc,
    output [63:0] rx_data,
    output rx_valid
);

wire can_signal;

can_tx tx_module (
    .clk(clk), .rst(rst),
    .start(tx_start), .id(tx_id), 
    .dlc(tx_dlc), .data(tx_data),
    .tx(can_signal), .busy(tx_busy), .done(tx_done)
);

can_rx rx_module (
    .clk(clk), .rst(rst),
    .can_rx(can_signal),
    .rx_id(rx_id), .rx_dlc(rx_dlc), 
    .rx_data(rx_data), .rx_valid(rx_valid)
);

endmodule
