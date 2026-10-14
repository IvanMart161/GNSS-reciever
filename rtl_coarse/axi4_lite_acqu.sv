module axi4_lite_acqu #(
    parameter int INPUT_DATA = 8,
    parameter int PHASE_WIDTH = 32,
    parameter int ROM_AWIDTH = 8,
    parameter int ROM_DWIDTH = 8,
    parameter int DEC = 10,
    parameter int ACC_WIDTH = 16,
    parameter int TARGET_SAMPLES = 3800,
    parameter int FFT_SIZE = 4096,
    parameter int ADDR_WIDTH = 32
) (
    input  logic                  aclk,
    input  logic                  aresetn,

    input  logic                  adc_input,
    input  logic signed [23:0]    fft_gold_code_re,
    input  logic signed [23:0]    fft_gold_code_im,

    input  logic [ADDR_WIDTH-1:0] s_axi_awaddr,
    input  logic                  s_axi_awvalid,
    output logic                  s_axi_awready,
    input  logic [31:0]           s_axi_wdata,
    input  logic [3:0]            s_axi_wstrb,
    input  logic                  s_axi_wvalid,
    output logic                  s_axi_wready,
    output logic [1:0]            s_axi_bresp,
    output logic                  s_axi_bvalid,
    input  logic                  s_axi_bready,
    input  logic [ADDR_WIDTH-1:0] s_axi_araddr,
    input  logic                  s_axi_arvalid,
    output logic                  s_axi_arready,
    output logic [31:0]           s_axi_rdata,
    output logic [1:0]            s_axi_rresp,
    output logic                  s_axi_rvalid,
    input  logic                  s_axi_rready,

    output logic [11:0]           read_addr,
    output logic [36:0]           power_out,
    output logic                  peak_ready,
    output logic                  fft_ready,
    output logic signed [27:0]    fft_out_re,
    output logic signed [27:0]    fft_out_im,
    output logic signed [17:0]    mix_out_re,
    output logic signed [17:0]    mix_out_im,
    output logic                  mix_out_valid
);

    localparam logic [ADDR_WIDTH-1:0] REG_CTRL        = 'h00;
    localparam logic [ADDR_WIDTH-1:0] REG_STATUS      = 'h04;
    localparam logic [ADDR_WIDTH-1:0] REG_PHASE_STEP  = 'h08;
    localparam logic [ADDR_WIDTH-1:0] REG_DOPPLER_STEP = 'h0C;

    logic [31:0] phase_step_reg;
    logic signed [31:0] doppler_step_reg;
    logic start_capture;

    logic aw_pending;
    logic [ADDR_WIDTH-1:0] awaddr_reg;
    logic w_pending;
    logic [31:0] wdata_reg;
    logic [3:0] wstrb_reg;
    logic write_fire;
    logic [ADDR_WIDTH-1:0] write_addr;
    logic [31:0] write_data;
    logic [3:0] write_strb;
    logic aw_handshake;
    logic w_handshake;

    function automatic logic [31:0] merge_wstrb(
        input logic [31:0] old_value,
        input logic [31:0] new_value,
        input logic [3:0]  strobe
    );
        logic [31:0] merged;
        merged = old_value;
        for (int lane = 0; lane < 4; lane++) begin
            if (strobe[lane]) begin
                merged[lane*8 +: 8] = new_value[lane*8 +: 8];
            end
        end
        return merged;
    endfunction

    assign s_axi_awready = !aw_pending && !s_axi_bvalid;
    assign s_axi_wready  = !w_pending && !s_axi_bvalid;
    assign s_axi_arready = !s_axi_rvalid;

    assign aw_handshake = s_axi_awvalid && s_axi_awready;
    assign w_handshake  = s_axi_wvalid && s_axi_wready;
    assign write_fire = !s_axi_bvalid &&
                        (aw_pending || aw_handshake) &&
                        (w_pending || w_handshake);
    assign write_addr = aw_pending ? awaddr_reg : s_axi_awaddr;
    assign write_data = w_pending ? wdata_reg : s_axi_wdata;
    assign write_strb = w_pending ? wstrb_reg : s_axi_wstrb;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            aw_pending      <= 1'b0;
            awaddr_reg      <= '0;
            w_pending       <= 1'b0;
            wdata_reg       <= '0;
            wstrb_reg       <= '0;
            s_axi_bresp     <= 2'b00;
            s_axi_bvalid    <= 1'b0;
            s_axi_rdata     <= '0;
            s_axi_rresp     <= 2'b00;
            s_axi_rvalid    <= 1'b0;
            phase_step_reg  <= 32'h4000_0000;
            doppler_step_reg <= '0;
            start_capture   <= 1'b0;
        end else begin
            start_capture <= 1'b0;

            if (aw_handshake) begin
                aw_pending <= 1'b1;
                awaddr_reg <= s_axi_awaddr;
            end
            if (w_handshake) begin
                w_pending <= 1'b1;
                wdata_reg <= s_axi_wdata;
                wstrb_reg <= s_axi_wstrb;
            end

            if (write_fire) begin
                aw_pending <= 1'b0;
                w_pending  <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;

                if (write_addr[1:0] != 2'b00) begin
                    s_axi_bresp <= 2'b10;
                end else begin
                    case (write_addr)
                        REG_CTRL: begin
                            if (write_strb[0] && write_data[0]) begin
                                start_capture <= 1'b1;
                            end
                        end
                        REG_PHASE_STEP: begin
                            phase_step_reg <= merge_wstrb(
                                phase_step_reg, write_data, write_strb
                            );
                        end
                        REG_DOPPLER_STEP: begin
                            doppler_step_reg <= merge_wstrb(
                                doppler_step_reg, write_data, write_strb
                            );
                        end
                        default: s_axi_bresp <= 2'b10;
                    endcase
                end
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp <= 2'b00;
                if (s_axi_araddr[1:0] != 2'b00) begin
                    s_axi_rdata <= '0;
                    s_axi_rresp <= 2'b10;
                end else begin
                    case (s_axi_araddr)
                        REG_CTRL:         s_axi_rdata <= '0;
                        REG_STATUS:       s_axi_rdata <= {30'b0, peak_ready, fft_ready};
                        REG_PHASE_STEP:   s_axi_rdata <= phase_step_reg;
                        REG_DOPPLER_STEP: s_axi_rdata <= doppler_step_reg;
                        default: begin
                            s_axi_rdata <= '0;
                            s_axi_rresp <= 2'b10;
                        end
                    endcase
                end
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    top_acqu #(
        .INPUT_DATA(INPUT_DATA),
        .PHASE_WIDTH(PHASE_WIDTH),
        .ROM_AWIDTH(ROM_AWIDTH),
        .ROM_DWIDTH(ROM_DWIDTH),
        .DEC(DEC),
        .ACC_WIDTH(ACC_WIDTH),
        .TARGET_SAMPLES(TARGET_SAMPLES),
        .FFT_SIZE(FFT_SIZE)
    ) u_top_acqu (
        .clk(aclk),
        .rst(!aresetn),
        .adc_input(adc_input),
        .phase_step(phase_step_reg),
        .start_capture(start_capture),
        .doppler_step(doppler_step_reg),
        .fft_gold_code_re(fft_gold_code_re),
        .fft_gold_code_im(fft_gold_code_im),
        .read_addr(read_addr),
        .power_out(power_out),
        .peak_ready(peak_ready),
        .fft_ready(fft_ready),
        .fft_out_re(fft_out_re),
        .fft_out_im(fft_out_im),
        .mix_out_re(mix_out_re),
        .mix_out_im(mix_out_im),
        .mix_out_valid(mix_out_valid)
    );

endmodule
