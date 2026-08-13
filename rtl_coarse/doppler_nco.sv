module doppler_nco #(
    parameter int PHASE_WIDTH = 32,
    parameter int ROM_AWIDTH  = 8,
    parameter int ROM_DWIDTH  = 8
) (
    input  logic                            clk,
    input  logic                            rst,
    
    // Управление
    input  logic [PHASE_WIDTH-1:0]          doppler_step,
    input  logic                            stream_enable,
    
    // Выход сгенерированного NCO
    output logic signed [ROM_DWIDTH-1:0]    cos_out,
    output logic signed [ROM_DWIDTH-1:0]    sin_out,
    output logic                            valid_out
);

logic [PHASE_WIDTH-1:0] phase_acc;

always_ff @(posedge clk) begin 
    if(rst || !stream_enable) begin
        phase_acc <= '0;
    end
    else begin
        if(stream_enable) begin 
            phase_acc <= phase_acc + doppler_step;
        end
    end
end

logic signed [ROM_DWIDTH-1:0] sin_rom [0:(2**ROM_AWIDTH)-1];
initial begin
    for(int i = 0; i < 2**ROM_AWIDTH; i++) begin 
        sin_rom[i] = ROM_DWIDTH'($rtoi(127.0 * $sin(2.0 * 3.14159265 * i / (2.0**ROM_AWIDTH))));
    end 
end

logic [ROM_AWIDTH-1:0] addr;
assign addr = phase_acc[PHASE_WIDTH-1: PHASE_WIDTH - ROM_AWIDTH];

always_ff @(posedge clk) begin 
    if(rst) begin 
        cos_out <= '0;
        sin_out <= '0;
        valid_out <= 1'b0;
    end 
    else begin 
        sin_out <= sin_rom[addr];
        cos_out <= sin_rom[addr + 8'd64];

        valid_out <= stream_enable;
    end
end


endmodule