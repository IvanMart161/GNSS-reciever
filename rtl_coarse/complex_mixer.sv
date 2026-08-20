module complex_mixer #(
    parameter int DATA_WIDTH = 16,  
    parameter int ROM_DWIDTH = 8    
) (
    input logic                      clk,
    input logic                      rst,

    input logic signed [DATA_WIDTH-1:0]  buf_i,
    input logic signed [DATA_WIDTH-1:0]  buf_q,
    input logic                          buf_valid, 
    input logic signed [ROM_DWIDTH-1:0]  nco_cos,
    input logic signed [ROM_DWIDTH-1:0]  nco_sin,

    output logic signed [DATA_WIDTH-1:0] mix_i,
    output logic signed [DATA_WIDTH-1:0] mix_q,
    output logic                         mix_valid
);

    logic signed [24:0] mult_i_cos;
    logic signed [24:0] mult_q_sin;
    logic signed [24:0] mult_i_sin;
    logic signed [24:0] mult_q_cos;
    
    logic signed [25:0] sum_i;
    logic signed [25:0] sum_q;

    always_ff @(posedge clk) begin
        if (rst) begin
            mult_i_cos <= '0;
            mult_q_sin <= '0;
            mult_i_sin <= '0;
            mult_q_cos <= '0;
            
            sum_i <= '0;
            sum_q <= '0;

            mix_i <= '0;
            mix_q <= '0;
            mix_valid <= 1'b0;
        end else begin
            mult_i_cos <= buf_i * nco_cos;
            mult_q_sin <= buf_q * nco_sin;
            mult_i_sin <= buf_i * nco_sin;
            mult_q_cos <= buf_q * nco_cos;
            
            sum_i <= mult_i_cos - mult_q_sin;
            sum_q <= mult_i_sin + mult_q_cos;
            
            mix_i <= DATA_WIDTH'(sum_i >>> 8);
            mix_q <= DATA_WIDTH'(sum_q >>> 8);
            
            mix_valid <= buf_valid; 
        end
    end

endmodule
