`timescale 1ns / 1ns

module power_detector #(
    parameter int IN_WIDTH  = 18,  
    parameter int OUT_WIDTH = 37   
) (
    input logic                         clk,
    input logic                         rst,

    input logic signed [IN_WIDTH-1:0]   ifft_re_in,
    input logic signed [IN_WIDTH-1:0]   ifft_im_in,
    input logic                         ifft_valid_in,

    output logic [OUT_WIDTH-1:0]        power_out,
    output logic                        power_valid
);

    // =========================================================================
    // Stage 1: Расширение знака ДО умножения (защита от переполнения)
    // =========================================================================
    logic signed [IN_WIDTH*2-1:0] re_ext;
    logic signed [IN_WIDTH*2-1:0] im_ext;
    
    assign re_ext = ifft_re_in;
    assign im_ext = ifft_im_in;

    logic signed [IN_WIDTH*2-1:0] re_sq_signed;
    logic signed [IN_WIDTH*2-1:0] im_sq_signed;
    logic                         stg1_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            re_sq_signed <= '0;
            im_sq_signed <= '0;
            stg1_valid   <= 1'b0;
        end else begin
            re_sq_signed <= re_ext * re_ext;
            im_sq_signed <= im_ext * im_ext;
            stg1_valid   <= ifft_valid_in;
        end
    end

    // =========================================================================
    // Stage 2: Сложение квадратов
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            power_out   <= '0;
            power_valid <= 1'b0;
        end else begin
            power_out   <= unsigned'(re_sq_signed) + unsigned'(im_sq_signed);
            power_valid <= stg1_valid;
        end
    end

endmodule
