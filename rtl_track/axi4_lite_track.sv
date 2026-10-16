`timescale 1ns / 1ps

module axi4_lite_track #(
    parameter int CARRIER_PHASE_WIDTH = 32,
    parameter int CODE_PHASE_WIDTH    = 32,
    parameter int NCO_ROM_AWIDTH      = 8,
    parameter int DATA_WIDTH          = 8,
    parameter int ACC_WIDTH           = 24,
    parameter int ADDR_WIDTH          = 32
) (
    input  logic                  aclk,
    input  logic                  aresetn,

    input  logic                  adc_data_in,
    input  logic [3:0]            s1,
    input  logic [3:0]            s2,
    input  logic signed [CARRIER_PHASE_WIDTH-1:0] carrier_phase_adjust,
    input  logic signed [CODE_PHASE_WIDTH-1:0]    code_phase_adjust,

    output logic signed [ACC_WIDTH-1:0] acc_i_e,
    output logic signed [ACC_WIDTH-1:0] acc_i_p,
    output logic signed [ACC_WIDTH-1:0] acc_i_l,
    output logic signed [ACC_WIDTH-1:0] acc_q_e,
    output logic signed [ACC_WIDTH-1:0] acc_q_p,
    output logic signed [ACC_WIDTH-1:0] acc_q_l,
    output logic                        dump_valid,

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
    input  logic                  s_axi_rready
);

    localparam logic [ADDR_WIDTH-1:0] REG_STATUS       = 'h04;
    localparam logic [ADDR_WIDTH-1:0] REG_CARRIER_STEP = 'h08;
    localparam logic [ADDR_WIDTH-1:0] REG_CODE_STEP    = 'h0C;

    logic [31:0] carrier_phase_step_reg;
    logic [31:0] code_phase_step_reg;

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
            aw_pending             <= 1'b0;
            awaddr_reg             <= '0;
            w_pending              <= 1'b0;
            wdata_reg              <= '0;
            wstrb_reg              <= '0;
            carrier_phase_step_reg <= '0;
            code_phase_step_reg    <= '0;
            s_axi_bresp            <= 2'b00;
            s_axi_bvalid           <= 1'b0;
            s_axi_rdata            <= '0;
            s_axi_rresp            <= 2'b00;
            s_axi_rvalid           <= 1'b0;
        end else begin
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
                        REG_CARRIER_STEP: begin
                            carrier_phase_step_reg <= merge_wstrb(
                                carrier_phase_step_reg, write_data, write_strb
                            );
                        end
                        REG_CODE_STEP: begin
                            code_phase_step_reg <= merge_wstrb(
                                code_phase_step_reg, write_data, write_strb
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
                        REG_STATUS:       s_axi_rdata <= {31'b0, dump_valid};
                        REG_CARRIER_STEP: s_axi_rdata <= carrier_phase_step_reg;
                        REG_CODE_STEP:    s_axi_rdata <= code_phase_step_reg;
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

    tracker_channel #(
        .CARRIER_PHASE_WIDTH(CARRIER_PHASE_WIDTH),
        .CODE_PHASE_WIDTH(CODE_PHASE_WIDTH),
        .NCO_ROM_AWIDTH(NCO_ROM_AWIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) u_tracker_channel (
        .clk                  (aclk),
        .rst                  (!aresetn),
        .adc_data_in          (adc_data_in),
        .carrier_phase_step   (CARRIER_PHASE_WIDTH'(carrier_phase_step_reg)),
        .carrier_phase_adjust (carrier_phase_adjust),
        .code_phase_step      (CODE_PHASE_WIDTH'(code_phase_step_reg)),
        .code_phase_adjust    (code_phase_adjust),
        .s1                   (s1),
        .s2                   (s2),
        .acc_i_e              (acc_i_e),
        .acc_i_p              (acc_i_p),
        .acc_i_l              (acc_i_l),
        .acc_q_e              (acc_q_e),
        .acc_q_p              (acc_q_p),
        .acc_q_l              (acc_q_l),
        .dump_valid           (dump_valid)
    );

endmodule
