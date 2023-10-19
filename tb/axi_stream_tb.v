`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/09/2023 12:32:27 PM
// Design Name: 
// Module Name: axi_stream_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module axi_stream_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter RAM_bit     = log2(Tape_Num)
)();

    reg                         ss_tvalid;
    reg signed [(pDATA_WIDTH-1) : 0] ss_tdata;
    reg                         ss_tlast;
    wire                        ss_tready;

    wire [3:0]               data_WE;
    wire                     data_EN;
    wire [(pDATA_WIDTH-1):0] data_Di;
    wire [(pADDR_WIDTH-1):0] data_A;
    wire [(pDATA_WIDTH-1):0] data_Do;

    reg en;
    reg flush;
    wire shift;
    wire wait_ram;

    reg [RAM_bit-1:0] FIR_addr;
    wire signed [pDATA_WIDTH-1:0]FIR_data;

    reg ap_start;

    reg                         axis_clk;
    reg                         axis_rst_n;

    axi_stream axi_stream_U(.ss_tvalid(ss_tvalid),.ss_tdata(ss_tdata),.ss_tlast(ss_tlast),.ss_tready(ss_tready),
        .data_WE(data_WE),.data_EN(data_EN),.data_Di(data_Di),.data_A(data_A),.data_Do(data_Do),
        .en(en),.shift(shift),.wait_ram(wait_ram),
        .FIR_addr(FIR_addr),.FIR_data(FIR_data),.ap_start(ap_start),
        .axis_clk(axis_clk),.axis_rst_n(axis_rst_n));

    bram11 data_RAM(
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .A(data_A),
        .Do(data_Do)
    );

    initial begin
        $dumpfile("axi_stream.vcd");
        $dumpvars();
    end

    initial begin
        axis_clk = 0;
        forever begin
            #5 axis_clk = (~axis_clk);
        end
    end

    initial begin
        axis_rst_n = 0;
        @(posedge axis_clk); @(posedge axis_clk);
        axis_rst_n = 1;
        ap_start = 1;
    end

    integer i;
    initial begin
        en = 0;
        ss_tvalid = 0;
        FIR_addr = 0;
        wait(wait_ram);
        en = 1;
        for(i=0;i<20;i=i+1) begin
            ss_tlast = 0; ss(i);
        end
        ss_tlast = 1; ss(20);
        ss_tvalid <= 0;
        @(posedge axis_clk);
        for(i=0;i<Tape_Num;i=i+1) begin
            FIR_read(i);
        end
        @(posedge axis_clk);
        $finish;
    end


    task ss;
        input  signed [31:0] in;
        begin
            ss_tvalid <= 1;
            ss_tdata  <= in;
            @(posedge axis_clk);
            while (!ss_tready) begin
                @(posedge axis_clk);
            end
        end
    endtask

    task FIR_read;
        input [RAM_bit-1:0] raddr;
        begin
            FIR_addr <= raddr;
            @(posedge axis_clk);
        end
    endtask

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
