module mixer_loop #(
    parameter WIDTH = 8
) (
    input logic                     clk,
    input logic                     rst,
    
    // Одиночные входы от NCO
    input logic signed [WIDTH-1:0]  sin_in,
    input logic signed [WIDTH-1:0]  cos_in,
    
    // 1-битный вход данных напрямую с АЦП
    input logic                     adc_data_in,

    // Потоковые выходы I и Q
    output logic signed [WIDTH-1:0] i_out,
    output logic signed [WIDTH-1:0] q_out
);

always_ff @(posedge clk) begin
    if(rst) begin
        i_out <= '0;
        q_out <= '0;
    end else begin 
        // BPSK: если пришла 0 — знак сохраняем, если 1 — инвертируем
        i_out <= adc_data_in ? -cos_in : cos_in;
        q_out <= adc_data_in ? -sin_in : sin_in;
    end 
end

endmodule
