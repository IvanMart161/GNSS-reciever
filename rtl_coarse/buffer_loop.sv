module buffer_loop #(
    parameter int INPUT_DATA = 24,
    parameter int TARGET_SAMPLES = 3819,
    parameter int FFT_SIZE = 4096
) (
    input  logic                          clk,
    input  logic                          rst,
    
    // Управление
    input  logic                          start_capture,
    output logic                          frame_ready, 

    // Потоковые данные от CIC
    input  logic signed [INPUT_DATA-1:0]  I_in,
    input  logic signed [INPUT_DATA-1:0]  Q_in,
    input  logic                          data_valid,  

    // Интерфейс чтения
    input  logic [$clog2(FFT_SIZE)-1:0]   read_addr,
    output logic signed [INPUT_DATA-1:0]  I_out,       
    output logic signed [INPUT_DATA-1:0]  Q_out 
);

//======================================
//          ВЫДЕЛЕНИЕ ПАМЯТИ 
//======================================

logic signed [INPUT_DATA-1:0] ram_I [0:FFT_SIZE-1];
logic signed [INPUT_DATA-1:0] ram_Q [0:FFT_SIZE-1];

assign I_out = ram_I[read_addr];
assign Q_out = ram_Q[read_addr];

//======================================
//                БУФФЕР
//======================================

typedef enum logic [1:0] { 
    IDLE,                           // Ждем команды на захват 
    COLLECT,                        // Копим 3819 точек
    ZERO_PAD,                       // Добиваем оставшиеся 277 точек нулями
    DONE                            // Держим флаг готовности 
} state_t;

state_t state;
logic [$clog2(FFT_SIZE)-1:0] write_addr;

always_ff @(posedge clk) begin
    if (rst) begin
        state       <= IDLE;
        write_addr  <= '0;
        frame_ready <= 1'b0;
    end else begin
        case (state)
            IDLE: begin
                frame_ready <= 1'b0;
                if (start_capture) begin
                    write_addr <= '0;
                    state      <= COLLECT;
                end
            end

            COLLECT: begin
                if (data_valid) begin
                    ram_I[write_addr] <= I_in;
                    ram_Q[write_addr] <= Q_in;
                    
                    if (write_addr == 12'(TARGET_SAMPLES - 1)) begin
                        write_addr <= write_addr + 1'b1;
                        state      <= ZERO_PAD;
                    end else begin
                        write_addr <= write_addr + 1'b1;
                    end
                end
            end

            ZERO_PAD: begin
                ram_I[write_addr] <= '0;
                ram_Q[write_addr] <= '0;
                
                if (write_addr == 12'(FFT_SIZE - 1)) begin
                    state <= DONE;
                end else begin
                    write_addr <= write_addr + 1'b1;
                end
            end

            DONE: begin
                frame_ready <= 1'b1;                 
                if (start_capture) begin
                    state       <= COLLECT;
                    write_addr  <= '0;
                    frame_ready <= 1'b0;
                end
            end
        endcase
    end
end

endmodule
