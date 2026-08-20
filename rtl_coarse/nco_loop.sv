module nco_loop #(
    parameter PHASE_WIDTH   = 32,                   
    parameter ROM_AWIDTH    = 8,                    
    parameter ROM_DWIDTH    = 8                    
) (
    input logic                         clk,
    input logic                         rst,
    
    input logic [PHASE_WIDTH-1:0]       phase_step,       

    output logic signed [ROM_DWIDTH-1:0] cos_out,
    output logic signed [ROM_DWIDTH-1:0] sin_out
);
    logic [PHASE_WIDTH-1:0] phase_acc;

    always_ff @(posedge clk) begin
        if(rst) begin
            phase_acc <= '0;
        end else begin
            phase_acc <= phase_acc + phase_step;
        end
    end

    logic signed [ROM_DWIDTH-1:0] sin_rom [0:(2**ROM_AWIDTH)-1];

    initial begin
        for(int i=0; i< 2 ** ROM_AWIDTH; i++) begin 
            sin_rom[i] = ROM_DWIDTH'($rtoi(127.0 * $sin(2.0 * 3.14159265 * i / (2.0**ROM_AWIDTH)))); 
        end 
    end

    logic [ROM_AWIDTH-1:0] addr;
    assign addr = phase_acc[PHASE_WIDTH-1 : PHASE_WIDTH - ROM_AWIDTH]; 

    always_ff @(posedge clk) begin
        if(rst) begin 
            cos_out <= '0;
            sin_out <= '0;
        end else begin 
            sin_out <= sin_rom[addr];
            cos_out <= sin_rom[addr + 8'd64]; // Сдвиг на 90 градусов для косинуса
        end 
    end 
endmodule
