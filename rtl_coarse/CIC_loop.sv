module CIC_loop #(
    parameter INPUT_DATA = 8,
    parameter DEC = 10,
    parameter ACC_WIDTH = 16 
) (
    input logic                         clk,
    input logic                         rst,             
    input logic signed [INPUT_DATA-1:0] I_in,
    input logic signed [INPUT_DATA-1:0] Q_in,

    output logic signed [ACC_WIDTH-1:0] I_out,
    output logic signed [ACC_WIDTH-1:0] Q_out,
    output logic                        data_valid
);

// Интеграторы (примитивный, зато бесплатный ФНЧ)

logic signed [ACC_WIDTH-1:0] int_I, int_Q;

always_ff @(posedge clk) begin
    if(rst) begin
        // ИСПРАВЛЕНО: Сбрасываем внутренние регистры, а не входы
        int_I <= '0;
        int_Q <= '0;
    end else begin
        // ИСПРАВЛЕНО: Правильный каст к signed
        int_I <= int_I + 16'(I_in);
        int_Q <= int_Q + 16'(Q_in);
    end
end

// Прореживатель 
// Выпускает строб каждые 10 тактов, тем самым понижая частоту дискретизации 
// ИСПРАВЛЕНО: 4 бита для счета до 9 (DEC=10)
logic [3:0] cnt;
logic       cnt_ena;

always_ff @(posedge clk) begin
    if(rst) begin
        cnt     <= '0;
        cnt_ena <= 1'b0;
    end else begin
        if(cnt == (DEC - 1)) begin 
            cnt     <= '0;
            cnt_ena <= 1'b1; // ИСПРАВЛЕНО: неблокирующее присваивание
        end else begin 
            cnt     <= cnt + 1'b1;
            cnt_ena <= 1'b0;
        end
    end
end

// Гребенки

logic signed [ACC_WIDTH-1:0] int_I_delayed, int_Q_delayed;

always_ff @(posedge clk) begin
    if(rst) begin
        // ИСПРАВЛЕНО: Добавлен сброс всех регистров гребенки
        int_I_delayed <= '0;
        int_Q_delayed <= '0;
        I_out         <= '0;
        Q_out         <= '0;
        data_valid    <= 1'b0;
    end else begin
        if(cnt_ena) begin 
            I_out <= int_I - int_I_delayed;
            Q_out <= int_Q - int_Q_delayed; // ИСПРАВЛЕНО: минус вместо равно

            int_I_delayed <= int_I;
            int_Q_delayed <= int_Q;

            data_valid <= 1'b1;
        end else begin 
            data_valid <= 1'b0;
        end 
    end 
end 

endmodule