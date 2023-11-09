module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    input   wire                     ss_tvalid,
    input   wire [(pDATA_WIDTH-1):0] ss_tdata,
    input   wire                     ss_tlast,
    output  wire                     ss_tready,
    input   wire                     sm_tready,
    output  wire                     sm_tvalid,
    output  wire [(pDATA_WIDTH-1):0] sm_tdata,
    output  wire                     sm_tlast,

    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
    localparam
    RAM_bit = log2(Tape_Num);
    reg  [9:0] count;

    wire ap_start;
    wire ap_done;
    reg  ap_done_reg;
    wire ap_idle;
    reg  ap_idle_reg;

    reg [(RAM_bit-1):0] tap_ptr;
    reg [(RAM_bit-1):0] tap_nptr;
    reg [(RAM_bit-1):0] data_ptr;
    reg [(RAM_bit-1):0] data_nptr;
    reg [(RAM_bit-1):0] start_ptr;

    wire [(pDATA_WIDTH-1):0] coef;
    wire [(pDATA_WIDTH-1):0] data;

    wire [(pDATA_WIDTH-1):0] data_length;

    wire [(pDATA_WIDTH-1):0] mul;
    reg  [(pDATA_WIDTH-1):0] sum;

    wire mul_start;
    reg  mul_start_reg;
    wire mul_done;

    wire r_hs;

    wire en;
    reg  en_reg;
    wire shift;
    wire wait_ram;

    reg [2:0] state;
    reg [2:0] next_state;
    localparam
    IDLE     = 3'b000,
    INIT     = 3'b001,
    RUN_INIT = 3'b010,
    RUN_TAP  = 3'b011,
    WAIT_MUL = 3'b100,
    SUM      = 3'b101,
    OUTPUT   = 3'b110,
    DONE     = 3'b111;

    axilite axilite_U(.awready(awready),.wready(wready),.awvalid(awvalid),.awaddr(awaddr),.wvalid(wvalid),.wdata(wdata),
        .arready(arready),.rready(rready),.arvalid(arvalid),.araddr(araddr),.rvalid(rvalid),.rdata(rdata),
        .tap_WE(tap_WE),.tap_EN(tap_EN),.tap_Di(tap_Di),.tap_A(tap_A),.tap_Do(tap_Do),
        .ap_start(ap_start),.ap_done(ap_done),.ap_idle(ap_idle),.data_length(data_length),
        .FIR_raddr(tap_ptr),.FIR_rdata(coef),.axis_clk(axis_clk),.axis_rst_n(axis_rst_n));

    axi_stream axi_stream_U(.ss_tvalid(ss_tvalid),.ss_tdata(ss_tdata),.ss_tlast(ss_tlast),.ss_tready(ss_tready),
        .data_WE(data_WE),.data_EN(data_EN),.data_Di(data_Di),.data_A(data_A),.data_Do(data_Do),
        .en(en),.shift(shift),.wait_ram(wait_ram),
        .FIR_addr(data_ptr),.FIR_data(data),.ap_start(ap_start),.ap_done(ap_done),
        .axis_clk(axis_clk),.axis_rst_n(axis_rst_n));

    booth booth_U(.axis_clk(axis_clk),.axis_rst_n(axis_rst_n),.din0({coef[31],coef[14:0]}),.din1({data[31],data[14:0]}),
        .dout(mul),.start(mul_start),.done(mul_done));


    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(~axis_rst_n) begin
            state <= IDLE;
            tap_ptr <= 0;
            data_ptr <= 0;
        end
        else begin
            state <= next_state;
            tap_ptr <= tap_nptr;
            data_ptr <= data_nptr;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(~axis_rst_n) begin
            sum <= 0;
            count <= 2;
            start_ptr <= 2;
        end
        else begin
            case(state)
                IDLE: begin
                    sum <= 0;
                    count <= 2;
                    start_ptr <= 2;
                end
                INIT: begin
                    sum <= 0;
                    count <= 2;
                    start_ptr <= 2;
                end
                RUN_INIT: begin
                    sum <= 0;
                    count <= count;
                    start_ptr <= start_ptr;
                end
                RUN_TAP: begin
                    sum <= sum;
                    count <= count;
                    start_ptr <= start_ptr;
                end
                WAIT_MUL: begin
                    if (mul_done) begin
                        sum <= sum + mul;
                        count <= count;
                        start_ptr <= start_ptr;
                    end
                    else begin
                        sum <= sum;
                        count <= count;
                        start_ptr <= start_ptr;
                    end
                end
                SUM: begin
                    sum <= sum;
                    count <= count;
                    start_ptr <= start_ptr;
                end
                OUTPUT: begin
                    if (r_hs) begin
                        if (start_ptr == (Tape_Num-1)) begin
                            start_ptr <= 0;
                        end
                        else begin
                            start_ptr <= start_ptr + 1;
                        end
                        count <= count + 1;
                        sum <= 0;
                    end
                    else begin
                        count <= count;
                        sum <= sum;
                        start_ptr <= start_ptr;
                    end
                end
                DONE: begin
                    sum <= 0;
                    count <= count;
                    start_ptr <= start_ptr;
                end
            endcase
        end
    end

    always@* begin
        case(state)
            IDLE: begin
                data_nptr = 0;
                tap_nptr  = 0;
            end
            INIT: begin
                data_nptr = 0;
                tap_nptr  = 0;
            end
            RUN_INIT: begin
                data_nptr = data_ptr;
                tap_nptr  = tap_ptr;
            end
            RUN_TAP: begin
                tap_nptr   = tap_ptr;
                data_nptr  = data_ptr;
            end
            WAIT_MUL: begin
                if (mul_done) begin
                    if (data_ptr == (Tape_Num-1)) begin
                        data_nptr = 0;
                    end
                    else begin
                        data_nptr = data_ptr + 1;
                    end
                    if (tap_ptr == 0) begin
                        tap_nptr = Tape_Num-1;
                    end
                    else begin
                        tap_nptr = tap_ptr-1;
                    end
                end
                else begin
                    tap_nptr   = tap_ptr;
                    data_nptr  = data_ptr;
                end
            end
            SUM: begin
                tap_nptr   = tap_ptr;
                data_nptr  = data_ptr;
            end
            OUTPUT: begin
                if (r_hs) begin
                    if (count < (Tape_Num+1)) begin
                        tap_nptr = (count - 1);
                    end
                    else begin
                        tap_nptr = (Tape_Num - 1);
                    end
                    if (count >= (Tape_Num+1)) begin
                        data_nptr = start_ptr;
                    end
                    else begin
                        data_nptr  = 0;
                    end
                end
                else begin
                    tap_nptr   = tap_ptr;
                    data_nptr  = data_ptr;
                end
            end
            DONE: begin
                tap_nptr   = 0;
                data_nptr  = 0;
            end
        endcase
    end

    always@* begin
        case(state)
            IDLE: begin
                en_reg = 0;
                mul_start_reg = 0;
            end
            INIT: begin
                if (wait_ram) begin
                    en_reg = 1;
                end
                else begin
                    en_reg = 0;
                end
                mul_start_reg = 0;
            end
            RUN_INIT: begin
                if (shift) begin
                    en_reg = 0;
                end else begin
                    en_reg = 1;
                end
                mul_start_reg = 0;
            end
            RUN_TAP: begin
                en_reg = 0;
                mul_start_reg = 1;
            end
            WAIT_MUL: begin
                en_reg = 0;
                mul_start_reg = 0;
            end
            SUM: begin
                en_reg = 0;
                mul_start_reg = 0;
            end
            OUTPUT: begin
                if (r_hs) begin
                    if (count == (data_length[0+:10]+1)) begin
                        en_reg = 0;
                    end
                    else begin
                        en_reg = 1;
                    end
                end
                else begin
                    en_reg = 0;
                end
                mul_start_reg = 0;
            end
            DONE: begin
                en_reg = 0;
                mul_start_reg = 0;
            end
        endcase
    end

    always@* begin
        case(state)
            IDLE: begin
                ap_done_reg = 0;
                ap_idle_reg = 1;
            end
            INIT: begin
                ap_done_reg = 0;
                ap_idle_reg = 0;
            end
            RUN_INIT: begin
                ap_done_reg = 0;
                ap_idle_reg = 0;
            end
            RUN_TAP: begin
                ap_done_reg = 0;
                ap_idle_reg = 0;
            end
            WAIT_MUL: begin
                ap_done_reg = 0;
                ap_idle_reg = 0;
            end
            SUM: begin
                ap_done_reg = 0;
                ap_idle_reg = 0;
            end
            OUTPUT: begin
                if (r_hs & count == (data_length[0+:10]+1)) begin
                    ap_done_reg = 1;
                    ap_idle_reg = 1;
                end
                else begin
                    ap_done_reg = 0;
                    ap_idle_reg = 0;
                end
            end
            DONE: begin
                if (ap_start) begin
                    ap_done_reg = 0;
                    ap_idle_reg = 0;
                end
                else begin
                    ap_done_reg = 1;
                    ap_idle_reg = 1;
                end
            end
        endcase
    end

    always@* begin
        case(state)
            IDLE: begin
                if (ap_start) begin
                    next_state = INIT;
                end
                else begin
                    next_state = IDLE;
                end
            end
            INIT: begin
                if (wait_ram) begin
                    next_state = RUN_INIT;
                end
                else begin
                    next_state = INIT;
                end
            end
            RUN_INIT: begin
                if (shift) begin
                    next_state = RUN_TAP;
                end
                else begin
                    next_state = RUN_INIT;
                end
            end
            RUN_TAP: begin
                next_state = WAIT_MUL;
            end
            WAIT_MUL: begin
                if (mul_done) begin
                    next_state = SUM;
                end
                else begin
                    next_state = WAIT_MUL;
                end
            end
            SUM: begin
                if (count <= Tape_Num + 1) begin
                    if (data_ptr == (Tape_Num-1)) begin
                        next_state = OUTPUT;
                    end
                    else begin
                        next_state = RUN_TAP;
                    end
                end
                else begin
                    if (tap_ptr == 0) begin
                        next_state = OUTPUT;
                    end
                    else begin
                        next_state = RUN_TAP;
                    end
                end
            end
            OUTPUT: begin
                if (r_hs) begin
                    if (count == (data_length[0+:10]+1)) begin
                        next_state = DONE;
                    end
                    else begin
                        next_state = RUN_INIT;
                    end
                end
                else begin
                    next_state = SUM;
                end
            end
            DONE: begin
                if (ap_start) begin
                    next_state = INIT;
                end
                else begin
                    next_state = DONE;
                end
            end
        endcase
    end

    assign ap_done = ap_done_reg;
    assign ap_idle = ap_idle_reg;

    assign en = en_reg;
    assign r_hs = sm_tvalid & sm_tready;
    assign sm_tvalid = (state == OUTPUT);
    assign sm_tdata  = sum;
    assign sm_tlast  = ss_tlast;
    assign mul_start = mul_start_reg;

    function integer log2;
        input integer x;
        integer n, m;
        begin
            n = 1;
            m = 2;
            while (m < x) begin
                n = n + 1;
                m = m * 2;
            end
            log2 = n;
        end
    endfunction
endmodule
