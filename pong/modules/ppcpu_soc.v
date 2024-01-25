// Connects ppcpu core with pcpu peripherals for use on original pcpu dev board


///////////////////////// config.v
`ifndef CONFIG_H
`define CONFIG_H

`define RW 16

`define REGNO 16
`define REGNO_LOG 4

`define ADDR_W 16
`define EXT_ADDR_W 24

`define I_SIZE 32

`define ADDR_BYTES 2

`define WB_ADDR_W 24

// -- ALU --
`define ALU_MODE_W 4
`define ALU_MODE_L_PASS `ALU_MODE_W'b0
`define ALU_MODE_R_PASS `ALU_MODE_W'b1
`define ALU_MODE_ADD `ALU_MODE_W'b10
`define ALU_MODE_SUB `ALU_MODE_W'b11
`define ALU_MODE_AND `ALU_MODE_W'b100
`define ALU_MODE_OR `ALU_MODE_W'b101
`define ALU_MODE_XOR `ALU_MODE_W'b110
`define ALU_MODE_SHL `ALU_MODE_W'b111
`define ALU_MODE_SHR `ALU_MODE_W'b1000
`define ALU_MODE_MUL `ALU_MODE_W'b1001
`define ALU_MODE_DIV `ALU_MODE_W'b1010
`define ALU_MODE_ASHR `ALU_MODE_W'b1011
`define ALU_MODE_SEXT `ALU_MODE_W'b1100
`define ALU_MODE_MOD `ALU_MODE_W'b1101
`define ALU_FLAG_W 3
`define ALU_FLAG_CNT 5
`define ALU_FLAG_Z `ALU_FLAG_W'b0
`define ALU_FLAG_C `ALU_FLAG_W'b1
`define ALU_FLAG_N `ALU_FLAG_W'b10
`define ALU_FLAG_O `ALU_FLAG_W'b11
`define ALU_FLAG_P `ALU_FLAG_W'b100

// -- SHARED DECODE --
`define JUMP_CODE_W 5 // 4+1
`define JUMP_CODE_BIT_EN 4

`endif

///////////////////////// soc.v

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module ppcpu_soc (
    input wire i_clk,
    input wire i_rst,

output wire hw_clk,
output wire hw_cyc,
output reg hw_stb,
output wire [23:0] hw_adr,
output wire hw_we,
input wire [15:0] hw_i_data,
output wire [15:0] hw_o_data,
    input wire hw_ack,
    input wire hw_err,
    /*
    input wire i_hydr_irq,
    output wire o_hydr_wb_cyc,
    output wire o_hydr_wb_stb,
    output wire o_hydr_wb_we,
    output wire [`WB_DATA_W-1:0] wb_o_dat,
    output wire [`WB_DATA_W-1:0] wb_i_dat,
    output wire [`WB_ADDR_W-1:0]  wb_adr,
    input wire wb_ack,
    input wire wb_err,
    input wire [`WB_SEL_BITS-1:0] wb_sel,
    */
    //input i_irq,

    
    output [17:0] pc_leds,
    output [15:0] dbg_r0,

    input uart_rx,
    output uart_tx
);

reg por_n = 1'b0;
reg [3:0] por_cnt = 'b0;
always @(posedge d_clk) begin
    por_cnt <= por_cnt + 'b1;
    if(&por_cnt)
        por_n <= 1'b1;
    if(~i_rst)
        por_n <= 1'b0;
end

wire [15:0] dbg_pc;

assign pc_leds = {wb_cyc, wb_ack, dbg_pc};

reg [4:0] clk_div;
wire d_clk = clk_div[1];
wire d_rst = ~(i_rst & por_n);
always @(posedge i_clk) begin
    clk_div <= clk_div + 5'b1;
end

`define MPRJ_IO_PADS 38
wire [`MPRJ_IO_PADS-1:0] m_io_in;
wire [`MPRJ_IO_PADS-1:0] m_io_out;
wire [`MPRJ_IO_PADS-1:0] m_io_oeb;

cpu_top cpu_top (
    .m_io_in(m_io_in),
    .m_io_out(m_io_out),
    .m_io_oeb(m_io_oeb),
    .mgt_wb_clk_i(d_clk),
    .mgt_wb_rst_i(d_rst),
    .mgt_wb_cyc_i(1'b0),
    .mgt_wb_stb_i(1'b0),
    .la_oenb({128{1'b1}}),
    .mgt_wb_we_i(1'b0),
    .mgt_wb_dat_i('b0),
    .mgt_wb_sel_i('b0),
    .mgt_wb_adr_i('b0),
    .la_data_in('b0),
    .mgt_wb_ack_o(ignored_wb_ack),
    .mgt_wb_dat_o(ignored_wb_dat_o),
    .la_data_out(ignored_data_out),
    .irq(ignored_irq),
    .dbg_r0(dbg_r0),
    .dbg_pc(dbg_pc)
);

wire [31:0] ignored_wb_dat_o;
wire ignored_wb_ack;
wire [127:0] ignored_data_out;
wire [2:0] ignored_irq;

// pins to cw bus
wire [`RW-1:0] cw_io_i;
wire [`RW-1:0] cw_io_o;
wire cw_req;
wire cw_dir;
wire cw_ack;
wire cw_err;
wire cw_clk;
wire cw_rst;

localparam CW_PIN_OFF=8;
assign cw_req = m_io_out[CW_PIN_OFF+0];
assign cw_dir = m_io_out[CW_PIN_OFF+1];
assign cw_io_o = m_io_out[CW_PIN_OFF+17:CW_PIN_OFF+2];
assign m_io_in[CW_PIN_OFF+17:CW_PIN_OFF+2] = cw_io_i;
assign m_io_in[CW_PIN_OFF+18] = cw_ack;
assign m_io_in[CW_PIN_OFF+19] = cw_err;
assign cw_clk = m_io_out[CW_PIN_OFF+20];
assign cw_rst = m_io_out[CW_PIN_OFF+21];
assign m_io_in[CW_PIN_OFF+22] = m_irq;
assign m_io_in[CW_PIN_OFF+23] = 1'b0; // split clock

wire wb_cyc;
wire wb_stb;
wire [`WB_DATA_W-1:0] wb_o_dat;
reg [`WB_DATA_W-1:0] wb_i_dat;
wire [`WB_ADDR_W-1:0]  wb_adr;
wire wb_we;
reg wb_ack;
reg wb_err;
wire [`WB_SEL_BITS-1:0] wb_sel;

wb_decomp wb_decomp (
    .i_clk(cw_clk),
    .i_rst(d_rst),

    .cw_io_i(cw_io_o),
    .cw_io_o(cw_io_i),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb),
    .wb_o_dat(wb_o_dat),
    .wb_i_dat(wb_i_dat),
    .wb_adr(wb_adr),
    .wb_we(wb_we),
    .wb_ack(wb_ack),
    .wb_err(wb_err),
    .wb_sel(wb_sel)
);

/*
 * Address map
 */

localparam UART_BASE =  24'h002000;
localparam UART_END =   24'h002003;

localparam TIMER_BASE = 24'h002008;
localparam TIMER_END =  24'h00200a;

localparam IRQC_BASE =  24'h00200c;
localparam IRQC_END =   24'h00200e;

localparam HYDRAULIK_BASE =  24'h004000;
localparam HYDRAULIK_END =   24'h008000;

localparam SDRAM_BASE = 24'h100000; 
localparam SDRAM_END =  24'hffdfff;

localparam SDRAM0_BASE = 24'h100000; 
localparam SDRAM0_END =  24'h110000;

localparam SDRAM1_BASE = 24'h800000; 
localparam SDRAM1_END =  24'h810000;

localparam ROM_BASE =   24'hffe000;
localparam ROM_END =    24'hffffff;


wire [`RW-1:0] sdram_data_out;
assign sdram_data_out = data_out[15:0];

wire [31:0] data_out;

reg sdram_req;
reg sdram_req_active;

//// Sdram interface to wishbone adapter
//always @(posedge cw_clk) begin
//    if(d_rst) begin
//        sdram_req <= 1'b0;
//        sdram_req_active <= 1'b0;
//    end else if (sdram_req & c_cack & wb_we) begin
//        sdram_req <= 1'b0;
//        sdram_req_active <= 1'b0;
//    end else if (sdram_req & c_cack & c_read_ready & ~wb_we) begin
//        sdram_req <= 1'b0;
//        sdram_req_active <= 1'b0;
//    end else if (sdram_req & c_cack & ~wb_we) begin
//        sdram_req <= 1'b0;
//    end else if (sdram_req_active & c_read_ready & ~wb_we) begin
//        sdram_req_active <= 1'b0;
//    end else if ((wb_adr >= SDRAM_BASE) & (wb_adr <= SDRAM_END) & wb_cyc & wb_stb & ~sdram_req_active) begin
//        sdram_req <= 1'b1;
//        sdram_req_active <= 1'b1;
//    end
//end

//wire sdram_ack = sdram_req_active & ((c_read_ready & ~wb_we) | (c_cack & wb_we));

reg prev_stb;
always @(posedge i_clk) begin
    prev_stb <= wb_stb;
end

/*wire c_busy, c_read_ready, c_cack;
sdram sdram (
    .clk(cw_clk),
    .srclk(cw_clk),
    .c_addr(wb_adr),
    .c_data_in(wb_o_dat),
    .c_data_out(data_out),
    .c_addr_sel(wb_sel),
    .c_read_req(sdram_req & ~wb_we),
    .c_write_req(sdram_req & wb_we),
    .c_busy(c_busy),
    .c_read_ready(c_read_ready),
    .c_cack(c_cack),

    .dr_dqml(dr_dqml), .dr_dqmh(dr_dqmh),
    .dr_cs_n(dr_cs_n), .dr_cas_n(dr_cas_n), .dr_ras_n(dr_ras_n), .dr_we_n(dr_we_n), .dr_cke(dr_cke),
    .dr_ba(dr_ba),
    .dr_a(dr_a),
    .dr_dq(dr_dq)
);*/

wire [`WB_DATA_W-1:0] cpu_ram_data;
reg cpu_ram0_active;
cpu_ram cpu_ram0 (
    .address(wb_adr),
    .clock(cw_clk),
    .data(wb_o_dat),
    .wren(wb_we & wb_cyc & wb_stb & cpu_ram0_active),
    .q(cpu_ram_data),
    .byteena(wb_sel)
);

wire cpu_req0_active = wb_cyc & wb_stb & cpu_ram0_active;
reg prev_cpu_reg0_active;
wire cpu_req0_ack = cpu_req0_active & prev_cpu_reg0_active;
always @(posedge cw_clk) begin
    prev_cpu_reg0_active <= cpu_req0_active;
end

wire [`WB_DATA_W-1:0] cpu1_ram_data;
reg cpu_ram1_active;
cpu_ram cpu_ram1 (
    .address(wb_adr),
    .clock(cw_clk),
    .data(wb_o_dat),
    .wren(wb_we & wb_cyc & wb_stb & cpu_ram1_active),
    .q(cpu1_ram_data),
    .byteena(wb_sel)
);

wire cpu_req1_active = wb_cyc & wb_stb & cpu_ram1_active;
reg prev_cpu_reg1_active;
wire cpu_req1_ack = cpu_req1_active & prev_cpu_reg1_active;
always @(posedge cw_clk) begin
    prev_cpu_reg1_active <= cpu_req1_active;
end
//assign dr_clk = ~cw_clk;//i_clk; // ram controller depends on setting edges half cycle before ram

wire [`RW-1:0] rom_data;
soc_rom soc_rom (
    .in_addr(wb_adr),
    .out_data(rom_data)
);

wire [`WB_DATA_W-1:0] uart_wb_i_dat;
wire uart_wb_ack;
uart uart (
    .i_clk(cw_clk),
    .i_full_clk(i_clk),
    .i_rst(d_rst),

    .tx(uart_tx),
    .rx(uart_rx),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & ((wb_adr >= UART_BASE) && (wb_adr <= UART_END))),
    .wb_adr(wb_adr - UART_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(uart_wb_i_dat),
    .wb_ack(uart_wb_ack)
);

wire timer_irq, timer_wb_ack;
wire [`WB_DATA_W-1:0] timer_wb_i_dat;
timer timer (
    .i_clk(cw_clk),
    .i_rst(d_rst),
    
    .irq(timer_irq),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= TIMER_BASE && wb_adr <= TIMER_END)),
    .wb_adr(wb_adr - TIMER_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(timer_wb_i_dat),
    .wb_ack(timer_wb_ack)
);

wire m_irq, irqc_wb_ack;
wire [`WB_DATA_W-1:0] irqc_wb_i_dat;
wire hydr_irq = 1'b0;
irq_ctrl irq_ctrl (
    .i_clk(cw_clk),
    .i_rst(d_rst),

    .o_irq(m_irq),
    .i_irq({13'b0, hydr_irq, 1'b0, timer_irq}),

    .wb_cyc(wb_cyc),
    .wb_stb(wb_stb & (wb_adr >= IRQC_BASE && wb_adr <= IRQC_END)),
    .wb_adr(wb_adr - IRQC_BASE),
    .wb_we(wb_we),
    .wb_i_dat(wb_o_dat),
    .wb_o_dat(irqc_wb_i_dat),
    .wb_ack(irqc_wb_ack)
);
assign hw_o_data = wb_o_dat;
assign hw_cyc = wb_cyc;
assign hw_clk = cw_clk;
assign hw_adr = wb_adr;
assign hw_we = wb_we;

always @(*) begin
    cpu_ram0_active = 1'b0;
    cpu_ram1_active = 1'b0;
    hw_stb = 1'b0;
    if (wb_adr >= SDRAM_BASE) begin
        if ((wb_adr >= SDRAM0_BASE) && (wb_adr <= SDRAM0_END)) begin
            wb_i_dat = cpu_ram_data;
            wb_ack = cpu_req0_ack;
            wb_err = 1'b0;
            cpu_ram0_active = 1'b1;
        end else if ((wb_adr >= SDRAM1_BASE) && (wb_adr <= SDRAM1_END)) begin
            wb_i_dat = cpu1_ram_data;
            wb_ack = cpu_req1_ack;
            wb_err = 1'b0;
            cpu_ram1_active = 1'b1;
        end else if ((wb_adr >= ROM_BASE) && (wb_adr <= ROM_END)) begin
            wb_i_dat = rom_data;
            wb_ack = wb_cyc & wb_stb;
            wb_err = 1'b0;
        end else begin
            wb_i_dat = 16'b0;
            wb_ack = 1'b0;
            wb_err = 1'b1;
        end
    end else begin
        if ((wb_adr >= UART_BASE) && (wb_adr <= UART_END)) begin
            wb_i_dat = uart_wb_i_dat;
            wb_ack = wb_cyc & wb_stb;
            wb_err = 1'b0;
        end else if ((wb_adr >= TIMER_BASE) && (wb_adr <= TIMER_END)) begin
            wb_i_dat = timer_wb_i_dat;
            wb_ack = timer_wb_ack;
            wb_err = 1'b0;
        end else if ((wb_adr >= IRQC_BASE) && (wb_adr <= IRQC_END)) begin
            wb_i_dat = irqc_wb_i_dat;
            wb_ack = irqc_wb_ack;
            wb_err = 1'b0;
        end else if ((wb_adr >= HYDRAULIK_BASE) && (wb_adr <= HYDRAULIK_END)) begin
            hw_stb = 1'b1;
            wb_i_dat = hw_i_data;
            wb_ack = hw_ack;
            wb_err = hw_err; 
        end else begin
            wb_i_dat = 16'b0;
            wb_ack = 1'b0;
            wb_err = 1'b1;
        end
    end
end


endmodule

/////////////////////////////////////////////////////////////////////////////
//// END OF SOC /////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

///////////////////////// uart.v

module uart (
    input i_clk,
    input i_full_clk,
    input i_rst,

    input rx,
    output reg tx,

    input wb_cyc, wb_stb, wb_we,
    output wb_ack,
    input [23:0] wb_adr,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat
);

localparam BAUD_RATE = 115200;
localparam OVERSAMPLE = 8;
localparam OVERSAMPLE_LOG = 3;
localparam CLOCK_FREQ = 50_000_000;
localparam UART_CLOCK_DIV = CLOCK_FREQ/(BAUD_RATE*2);
localparam OSMPL_CLOCK_DIV = CLOCK_FREQ/(BAUD_RATE*OVERSAMPLE*2);
localparam RX_BUFF_SIZE = 8;
localparam TX_BUFF_SIZE = 8;

reg uart_os_clk = 1'b0;
reg [5:0] uart_os_clk_cnt = 6'b0;
reg uart_clk = 1'b0;
reg [9:0] uart_clk_cnt = 10'b0;
always @(posedge i_full_clk) begin
    uart_os_clk_cnt <= uart_os_clk_cnt + 6'b1;
    uart_clk_cnt <= uart_clk_cnt + 6'b1;

    if (uart_os_clk_cnt == 27) begin
        uart_os_clk_cnt <= 6'b0;
        uart_os_clk <= ~uart_os_clk;
    end

    if (uart_clk_cnt == UART_CLOCK_DIV) begin
        uart_clk_cnt <= 10'b0;
        uart_clk <= ~uart_clk;
    end
end

// RECEIVE
// reg rx_active = 1'b0, rx_stop = 1'b0;
// reg [OVERSAMPLE_LOG:0] rx_os_cnt;
reg [7:0] rx_result;
reg [2:0] rx_res_bit;

//wire rx_submit = (rx_stop & (rx_os_cnt == 0));
wire rx_submit = (sub_clk_cnt == 4'b0111) && (state == 4'b1001);
//assign dbg_trig = rx_submit & ~rx;

// I DONT KNOW WHY but this code from old cpu works just fine,
// but preetier new code that seems to behave in the same way misses the stop bits. WHY???
reg [3:0] state = 4'b0;
reg [3:0] sub_clk_cnt = 4'b0;
//reg r1_trig = 1'b0;
always @(posedge uart_os_clk) begin
    case (state)
        4'b0: begin
            // if start bit
            if(rx == 1'b0) begin
                state <= 1'b1;
            end
        end
        4'b1001: begin
            if(sub_clk_cnt == 4'b0111) begin
                sub_clk_cnt <= 4'b0;
                state <= 4'b0;
                //r1_trig <= ~r1_trig;
            end else begin
                sub_clk_cnt <= sub_clk_cnt+4'b1;
            end
        end
        default: begin // default read bit
            if((state != 4'b1 && sub_clk_cnt == 4'b0111) || sub_clk_cnt == 4'b1010) begin //delay first clock by one and half
                rx_result[state-4'b1] <= rx;
                state <= state+4'b1;
                sub_clk_cnt <= 4'b0;
                //r1_trig <= ~r1_trig;
            end else begin
                sub_clk_cnt <= sub_clk_cnt+4'b1;
            end
        end
    endcase
end

// reg r2_trig = 1'b0;
// always @(posedge uart_os_clk) begin
//     if (i_rst) begin
//         rx_active <= 1'b0;
//         rx_os_cnt <= 'b0;
//         rx_stop <= 1'b0;
//     end else begin 
//         if (~rx_active & ~rx) begin
//             rx_active <= 1'b1;
//             rx_res_bit <= 'b0;
//             rx_os_cnt <= 4'b1010;
//         end else if (rx_stop & (rx_os_cnt == 0)) begin
//             rx_active <= 1'b0;
//             rx_stop <= 1'b0;
//             r2_trig <= ~r2_trig;
//             //dbg_trig <= ~dbg_trig;
//         end else if (rx_active & ~rx_stop & (rx_os_cnt == 0)) begin
//             rx_os_cnt <= 4'b0111;
//             r2_trig <= ~r2_trig;
//             //rx_result[rx_res_bit] <= rx;
//             rx_res_bit <= rx_res_bit + 1'b1;
//             rx_stop <= (rx_res_bit == 3'b111);
//             //dbg_trig <= ~dbg_trig;
//         end else if (rx_active) begin
//             rx_os_cnt <= rx_os_cnt - 1'b1;
//         end
//     end
// end


reg [7:0] rx_fifo [RX_BUFF_SIZE-1:0];
reg [2:0] rx_write_ptr, rx_read_ptr;

always @(posedge uart_os_clk) begin
    if (i_rst) begin
        rx_write_ptr <= 3'b0;
    end else if (rx_submit) begin
        rx_fifo[rx_write_ptr] <= rx_result;
        rx_write_ptr <= rx_write_ptr + 3'b1;
    end
end

wire rx_data_available = |(rx_read_ptr ^ rx_write_ptr);
reg [2:0] rx_prev_data;
wire rx_irq = (rx_prev_data == 3'b0 & ((rx_write_ptr-rx_read_ptr) != 3'b0)); 

always @(posedge i_clk) begin
    if (i_rst) begin
        rx_read_ptr <= 3'b0;
    end else begin
        rx_prev_data <= rx_write_ptr-rx_read_ptr;
        if (wb_cyc & wb_stb & ~wb_we & wb_adr == 24'h1 & rx_data_available) begin
            rx_read_ptr <= rx_read_ptr + 3'b1;
            rx_prev_data <= (rx_write_ptr-rx_read_ptr-3'b1);
        end
    end
end

// TRANSMIT

reg [1:0] tx_state;
wire tx_ready = (tx_state == 2'b0);
wire tx_start = tx_data_avail;
reg [7:0] tx_data;
reg [2:0] tx_data_cnt;

always @(posedge uart_clk) begin
    if (i_rst) begin
        tx <= 1'b1;        
        tx_state <= 2'b0;
    end else if ((tx_state == 2'b0) & tx_start) begin
        tx <= 1'b0; // start bit
        tx_data_cnt <= 3'b0;
        tx_state <= 2'b1; 
    end else if (tx_state == 2'b1) begin
        tx <= tx_data[tx_data_cnt];
        tx_data_cnt <= tx_data_cnt + 1'b1;
        if (&tx_data_cnt)
            tx_state <= 2'b10;
    end else if (tx_state == 2'b10) begin
        tx <= 1'b1; // stop bit
        tx_state <= 2'b0;
    end
end

reg [7:0] tx_fifo [RX_BUFF_SIZE-1:0];
reg [2:0] tx_write_ptr, tx_read_ptr;
wire tx_data_avail = |(tx_write_ptr^tx_read_ptr);
wire tx_full = (tx_write_ptr+3'b1) == tx_read_ptr;

always @(posedge uart_clk) begin
    if (i_rst) begin
        tx_read_ptr <= 3'b0;
    end else if (tx_ready & tx_data_avail) begin
        tx_read_ptr <= tx_read_ptr + 3'b1;
        tx_data <= tx_fifo[tx_read_ptr];
    end
end

reg [2:0] tx_prev_data;
wire tx_empty_irq = (tx_prev_data != 3'b0) && ((tx_write_ptr-tx_read_ptr) == 3'b0);

always @(posedge i_clk) begin
    if (i_rst) begin
        tx_write_ptr <= 3'b0;
    end else begin 
        tx_prev_data <= (tx_write_ptr-tx_read_ptr);
        if (wb_cyc & wb_stb & wb_we & wb_adr == 24'h2) begin
            tx_write_ptr <= tx_write_ptr + 3'b1;
            tx_fifo[tx_write_ptr] <= wb_i_dat[7:0]; 
        end
    end
end

assign wb_ack = wb_cyc & wb_stb;
always @* begin
    if (wb_adr == 24'h0) begin
        wb_o_dat = {14'b0, ~tx_full, rx_data_available};
    end else if (wb_adr == 24'h1) begin
        wb_o_dat = rx_fifo[rx_read_ptr];
    end else begin
        wb_o_dat = 16'b0;
    end
end

endmodule

///////////////////////// soc_rom.v


module soc_rom (
    input [`WB_ADDR_W-1:0] in_addr,
    output reg [`RW-1:0] out_data
);

reg [31:0] instr [127:0];

initial begin
    $readmemh("/home/piotro/pcpu2/soc/bootloader.hex", instr);
end

always @* begin
    if (in_addr[0])
        out_data = instr[in_addr[11:1]][31:16];
    else
        out_data = instr[in_addr[11:1]][15:0];
end

endmodule

///////////////////////// irq_ctl.v

// Interrrupt controller
// Takes multiple edge triggered interrupts as inputs (each able to clear and disable)
// and outputs one master level interrupt

module irq_ctrl (
    input i_clk,
    input i_rst,

    output o_irq,

    input [15:0] i_irq,

    input wb_cyc, wb_stb, wb_we,
    output wb_ack,
    input [23:0] wb_adr,
    input [15:0] wb_i_dat,
    output [15:0] wb_o_dat
);

assign o_irq = |(irq_active & irq_mask);

reg [15:0] irq_active;
reg [15:0] irq_mask;

always @(posedge i_clk) begin
    if (i_rst) begin
        irq_active = 16'b0;
        irq_mask = 16'h0000;
    end else begin
        if (wb_cyc & wb_stb & wb_we & wb_adr == 24'b1)
            irq_active = irq_active & ~wb_i_dat;
        if (wb_cyc & wb_stb & wb_we & wb_adr == 24'b10)
            irq_mask = wb_i_dat;
        
        irq_active = irq_active | ((prev_i_irq ^ i_irq) & i_irq);
    end
end

reg [15:0] prev_i_irq;
always @(posedge i_clk) begin
    if (i_rst)
        prev_i_irq <= 16'b0;
    else
        prev_i_irq <= i_irq;
end

assign wb_ack = wb_cyc & wb_stb;
assign wb_o_dat = (wb_adr == 24'b10 ? irq_mask : irq_active & irq_mask); 

endmodule

///////////////////////// END OF SOC/
///////////////////////// core.v



module alu_mul_div (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [`RW-1:0] i_a, i_b,
    output [`RW-1:0] o_d,

    input i_mul,
    input i_div,
    input i_mod,

    input i_submit,
    input i_flush,
    output o_busy
);

`define RWLOG 4

reg [`RWLOG-1:0] cbit;
reg comp;

always @(posedge i_clk) begin
    if (i_rst | i_flush) begin
        comp <= 1'b0;
    end else if (i_submit) begin
        cbit <= (i_mul ? `RWLOG'b1 : `RWLOG'b0);
        comp <= 1'b1;
    end else if (comp) begin
        cbit <= cbit+1'b1;
        if (cbit == `RWLOG'd15) begin
            comp <= 1'b0;
        end
    end
end

reg [`RW-1:0] mul_res;

always @(posedge i_clk) begin
    if (i_submit & i_mul) begin
        mul_res <= (i_b[0] ? i_a : `RW'b0);
    end else if (comp & i_b[cbit] & i_mul) begin
        mul_res <= mul_res + (i_a<<cbit);
    end
end

reg [`RW-1:0] div_res, div_cur;
wire [`RW-1:0] div_diff = div_cur-i_b;

always @(posedge i_clk) begin
    if (i_submit & (i_div | i_mod)) begin
        div_res <= `RW'b0;
        div_cur <= {{`RW-1{1'b0}}, i_a[`RW-1]};
    end else if (comp & (i_div | i_mod)) begin
        if (div_cur >= i_b) begin
            div_res[`RW-cbit-1] <= 1'b1;
            if (cbit != `RWLOG'd15)
                div_cur <= {div_diff[14:0], i_a[`RW-cbit-2]};
            else
                div_cur <= div_diff;
        end else begin
            if (cbit != `RWLOG'd15)
                div_cur <= {div_cur[14:0], i_a[`RW-cbit-2]};
        end
    end
end

assign o_d = (i_mod ? div_cur : (i_div ? div_res : mul_res));
assign o_busy = (i_submit | comp);

endmodule

`define IN_ADDR_W 16
`define OUT_ADDR_W 24

module dmmu (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [`IN_ADDR_W-1:0] i_addr,
    input [7:0] i_high_addr,
    output [`OUT_ADDR_W-1:0] o_addr,
    output o_cacheable,

    input [`RW-1:0] i_sr_addr,
    input [`RW-1:0] i_sr_data,
    input i_sr_we,

    input c_pag_en,
    input c_long
);

`define OFF_W 11
`define PAGE_IDX_W 4
`define PAGE_ENTRIES 16
`define PAGE_RES_SIZE 13
`define SR_ADDR_OFF `RW'h200

wire [`OFF_W-1:0] page_off = i_addr[`OFF_W-1:0];
wire [`PAGE_IDX_W-1:0] page_idx = i_addr[`OFF_W+`PAGE_IDX_W-1:`OFF_W];

reg [`PAGE_RES_SIZE-1:0] page_table [`PAGE_ENTRIES-1:0];

wire [`PAGE_RES_SIZE-1:0] page_res = {page_table[page_idx][`PAGE_RES_SIZE-1:0]};

wire [`RW-1:0] sr_addr_idx = i_sr_addr-`SR_ADDR_OFF;
wire addr_in_range = (i_sr_addr >= `SR_ADDR_OFF) && (i_sr_addr < `SR_ADDR_OFF+16);
reg [7:0] long_off_reg;

always @(posedge i_clk) begin
    if (i_rst) begin
        page_table[0] <= `PAGE_RES_SIZE'b0;
        page_table[1] <= `PAGE_RES_SIZE'b0;
        page_table[2] <= `PAGE_RES_SIZE'b0;
        page_table[3] <= `PAGE_RES_SIZE'b0;
        page_table[4] <= `PAGE_RES_SIZE'b0;
        page_table[5] <= `PAGE_RES_SIZE'b0;
        page_table[6] <= `PAGE_RES_SIZE'b0;
        page_table[7] <= `PAGE_RES_SIZE'b0;
        page_table[8] <= `PAGE_RES_SIZE'b0;
        page_table[9] <= `PAGE_RES_SIZE'b0;
        page_table[10] <= `PAGE_RES_SIZE'b0;
        page_table[11] <= `PAGE_RES_SIZE'b0;
        page_table[12] <= `PAGE_RES_SIZE'b0;
        page_table[13] <= `PAGE_RES_SIZE'b0;
        page_table[14] <= `PAGE_RES_SIZE'b0;
        page_table[15] <= `PAGE_RES_SIZE'b0;
    end else if (i_sr_we & addr_in_range)
        page_table[sr_addr_idx[`PAGE_IDX_W-1:0]] <= i_sr_data[`PAGE_RES_SIZE-1:0];
    else if (i_sr_we & i_sr_addr == `SR_ADDR_OFF+16)
        long_off_reg <= i_sr_data[7:0];
end

`define PAGE_DEFAULT_PREFIX 8'h10
wire [`OUT_ADDR_W-1:0] page_disable_address = {`PAGE_DEFAULT_PREFIX, i_addr[`IN_ADDR_W-1:0]};
wire [`OUT_ADDR_W-1:0] page_enable_address = {page_res, page_off};
wire [`OUT_ADDR_W-1:0] long_mode_address = (i_high_addr == 8'b0 && ~i_addr[15] ? page_enable_address : {i_high_addr[7:0]+long_off_reg, i_addr});

assign o_addr = c_long ? long_mode_address : (c_pag_en ? page_enable_address : page_disable_address);
assign o_cacheable = (o_addr >= `OUT_ADDR_W'h100000 && o_addr < `OUT_ADDR_W'h800000);

endmodule

module alu (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input [`RW-1:0] i_l, i_r,
    output [`RW-1:0] o_out,

    input [`ALU_MODE_W-1:0] i_mode,
    input wire i_carry,

    output reg [`ALU_FLAG_CNT-1:0] o_flags
);

reg [`RW:0] outc;
assign o_out = outc[`RW-1:0];

always @* begin
    case (i_mode)
        default:
            outc = {1'b0, i_l};
        `ALU_MODE_R_PASS:
            outc = {1'b0, i_r};
        `ALU_MODE_ADD:
            outc = i_l + i_r + {15'b0, i_carry};
        `ALU_MODE_SUB:
            outc = i_l - i_r - {15'b0, i_carry};
        `ALU_MODE_AND:
            outc = {1'b0, i_l & i_r};
        `ALU_MODE_OR:
            outc = {1'b0, i_l | i_r};
        `ALU_MODE_XOR:
            outc = {1'b0, i_l ^ i_r};
        `ALU_MODE_SHL:
            outc = {1'b0, i_l} << {1'b0, i_r};
        `ALU_MODE_SHR:
            outc = {1'b0, i_l >> i_r};
        `ALU_MODE_ASHR:
            outc = {1'b0, (i_l >> i_r) | ({`RW{i_l[`RW-1]}} << (`RW-i_r))};
        `ALU_MODE_SEXT:
            outc = {1'b0, {8{i_l[7]}}, i_l[7:0]};
    endcase

    o_flags[`ALU_FLAG_Z] = ~(|outc[`RW-1:0]);
    o_flags[`ALU_FLAG_C] = outc[`RW];
    o_flags[`ALU_FLAG_N] = outc[`RW-1];
    o_flags[`ALU_FLAG_O] = (i_l[`RW-1] ^ (i_r[`RW-1]^(i_mode == `ALU_MODE_SUB))) & ((i_l[`RW-1]^outc[`RW-1]));
    o_flags[`ALU_FLAG_P] = ^outc[`RW-1:0];
end

endmodule


`define PC_ADDR_W 16 // incremented by 2
`define OUT_ADDR_W 24

module immu (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [`PC_ADDR_W-1:0] i_addr,
    output [`OUT_ADDR_W-1:0] o_addr,

    input [`RW-1:0] i_sr_addr,
    input [`RW-1:0] i_sr_data,
    input i_sr_we,

    input [`OUT_ADDR_W-`PC_ADDR_W-1:0] i_long_high_addr,
    input c_long_mode,

    input c_pag_en
);

`define OFF_W 12
`define PAGE_IDX_W 4
`define PAGE_ENTRIES 16
`define PAGE_RES_SIZE 12
`define SR_ADDR_OFF `RW'h100

wire [`OFF_W-1:0] page_off = i_addr[`OFF_W-1:0];
wire [`PAGE_IDX_W-1:0] page_idx = i_addr[`OFF_W+`PAGE_IDX_W-1:`OFF_W];

reg [`PAGE_RES_SIZE-1:0] page_table [`PAGE_ENTRIES-1:0];

wire [`PAGE_RES_SIZE-1:0] page_res = {1'b1, page_table[page_idx][`PAGE_RES_SIZE-2:0]};

wire [`RW-1:0] sr_addr_idx = i_sr_addr-`SR_ADDR_OFF;
wire addr_in_range = (i_sr_addr >= `SR_ADDR_OFF) && (i_sr_addr < `SR_ADDR_OFF+16);
reg [7:0] high_addr_off;
always @(posedge i_clk) begin
    if (i_rst) begin
        page_table[0] <= `PAGE_RES_SIZE'h7fe;
        page_table[1] <= `PAGE_RES_SIZE'h7ff;
        page_table[2] <= `PAGE_RES_SIZE'b0;
        page_table[3] <= `PAGE_RES_SIZE'b0;
        page_table[4] <= `PAGE_RES_SIZE'b0;
        page_table[5] <= `PAGE_RES_SIZE'b0;
        page_table[6] <= `PAGE_RES_SIZE'b0;
        page_table[7] <= `PAGE_RES_SIZE'b0;
        page_table[8] <= `PAGE_RES_SIZE'b0;
        page_table[9] <= `PAGE_RES_SIZE'b0;
        page_table[10] <= `PAGE_RES_SIZE'b0;
        page_table[11] <= `PAGE_RES_SIZE'b0;
        page_table[12] <= `PAGE_RES_SIZE'b0;
        page_table[13] <= `PAGE_RES_SIZE'b0;
        page_table[14] <= `PAGE_RES_SIZE'b0;
        page_table[15] <= `PAGE_RES_SIZE'b0;
    end else if (i_sr_we & addr_in_range)
        page_table[sr_addr_idx[`PAGE_IDX_W-1:0]] <= i_sr_data[`PAGE_RES_SIZE-1:0];
    else if (i_sr_we & i_sr_addr == `SR_ADDR_OFF+16)
        high_addr_off <= i_sr_data[7:0];
end

`define PAGE_DEFAULT_PREFIX 8'h80
wire [`OUT_ADDR_W-1:0] page_disable_address = {`PAGE_DEFAULT_PREFIX, i_addr[`RW-1:0]};
wire [`OUT_ADDR_W-1:0] page_enable_address = {page_res, page_off};

wire [`OUT_ADDR_W-1:0] long_mode_addr = {i_long_high_addr+high_addr_off, i_addr};

assign o_addr = c_long_mode ? long_mode_addr : (c_pag_en ? page_enable_address : page_disable_address);

endmodule

`undef OFF_W
`undef PAGE_IDX_W
`undef PAGE_ENTRIES
`undef PAGE_RES_SIZE
`undef SR_ADDR_OFF
`undef PAGE_DEFAULT_PREFIX

module core #(parameter CORENO = 0, INT_VEC = 1) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,
    input i_disable,

    // fetch input singals
    output [`RW-1:0] o_req_addr,
    output o_req_active, o_req_ppl_submit,
    input [`I_SIZE-1:0] i_req_data,
    input i_req_data_valid,

    output [`RW-1:0] dbg_r0, dbg_pc,

    // data memory connections
    output [`RW-1:0] o_mem_addr, o_mem_data,
    input [`RW-1:0] i_mem_data,
    output o_mem_req, o_mem_we,
    input i_mem_ack,
    output [`ADDR_BYTES-1:0] o_mem_sel,
    output o_mem_long,
    output [7:0] o_mem_addr_high,

    input i_irq,
    output o_c_instr_page, o_c_data_page,
    output [`RW-1:0] sr_bus_addr, sr_bus_data_o,
    output sr_bus_we,
    output o_icache_flush,
    input i_mem_exception,
    input i_mc_core_int,
    input [`RW-1:0] i_core_int_sreg,
    output o_c_instr_long,
    output [7:0] o_instr_long_addr,

    output [35:0] dbg_out,
    input [3:0] dbg_in
);

// --- CPU INTERNAL CONNECTIONS ---

wire fetch_decode_next_ready;
wire fetch_decode_submit;
wire [`I_SIZE-1:0] fetch_decode_d_instr;
wire fetch_decode_jmp_pred;
wire execute_fetch_pc_update;
wire [`RW-1:0] execute_fetch_pc;
wire fde_pipeline_flush;

// Pipeline stage 0 - FETCH
fetch fetch (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .mem_addr(o_req_addr), .mem_submit(o_req_ppl_submit),
    .mem_data(i_req_data), .mem_ack(i_req_data_valid), .i_next_ready(fetch_decode_next_ready),
    .o_submit(fetch_decode_submit), .o_instr(fetch_decode_d_instr), .o_jmp_predict(fetch_decode_jmp_pred),
    .i_exec_pc(execute_fetch_pc), .i_flush(fde_pipeline_flush), .dbg_out(dbg_out[33]));
assign o_req_active = ~i_disable;

wire decode_execute_next_ready;
wire decode_execute_submit;
wire [`I_SIZE-17:0] de_imm_pass;
wire de_jmp_pred;

wire dec_pc_inc, dec_pc_ie;
wire dec_r_bus_imm;
wire [`ALU_MODE_W-1:0] dec_alu_mode;
wire dec_alu_carry_en, dec_alu_flags_ie;
wire [`REGNO_LOG-1:0] dec_l_reg_sel, dec_r_reg_sel; 
wire [`REGNO-1:0] dec_rf_ie;
wire [`JUMP_CODE_W-1:0] dec_jump_cond_code;
wire dec_mem_access, dec_mem_we, dec_mem_width;
wire [1:0] dec_used_operands;
wire dec_sreg_load, dec_sreg_store, dec_sreg_jal_over, dec_sreg_irt, dec_sys, dec_mem_long, dec_wfi;

// Pipeline stage 1 - DECODE
decode decode (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .o_ready(fetch_decode_next_ready), .o_submit(decode_execute_submit),
    .i_next_ready(decode_execute_next_ready), .i_instr_l(fetch_decode_d_instr[19:0]), .i_imm_pass(fetch_decode_d_instr[`I_SIZE-1:16]),
    .o_imm_pass(de_imm_pass), .oc_pc_inc(dec_pc_inc), .oc_pc_ie(dec_pc_ie), .oc_r_bus_imm(dec_r_bus_imm), .oc_alu_mode(dec_alu_mode),
    .oc_alu_flags_ie(dec_alu_flags_ie), .oc_alu_carry_en(dec_alu_carry_en), .oc_l_reg_sel(dec_l_reg_sel), .oc_r_reg_sel(dec_r_reg_sel),
    .oc_rf_ie(dec_rf_ie), .i_submit(fetch_decode_submit), .oc_jump_cond_code(dec_jump_cond_code), .i_jmp_pred_pass(fetch_decode_jmp_pred),
    .o_jmp_pred_pass(de_jmp_pred), .i_flush(fde_pipeline_flush), .oc_mem_access(dec_mem_access), .oc_mem_we(dec_mem_we),
    .oc_used_operands(dec_used_operands), .oc_sreg_load(dec_sreg_load), .oc_sreg_store(dec_sreg_store), .oc_sreg_jal_over(dec_sreg_jal_over),
    .oc_sreg_irt(dec_sreg_irt), .oc_sys(dec_sys), .oc_mem_width(dec_mem_width), .dbg_out(dbg_out[34]), .oc_mem_long(dec_mem_long), .oc_wfi(dec_wfi));

wire [`RW-1:0] ew_data;
wire [`RW-1:0] ew_addr;
wire [`REGNO-1:0] ew_reg_ie;
wire ew_mem_access, ew_mem_we, ew_mem_width;
wire [`REGNO-1:0] we_reg_ie;
wire [`RW-1:0] we_reg_data;
wire ew_submit;
wire ew_next_ready;
wire ew_long_mode;
wire [7:0] ew_addr_high;

// Pipeline stage 2 - EXECUTE
execute #(.CORENO(CORENO), .INT_VEC(INT_VEC)) execute(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .o_ready(decode_execute_next_ready), .i_imm(de_imm_pass), .c_pc_inc(dec_pc_inc),
    .c_pc_ie(dec_pc_ie), .c_r_bus_imm(dec_r_bus_imm), .c_alu_mode(dec_alu_mode), .c_alu_flags_ie(dec_alu_flags_ie), 
    .c_alu_carry_en(dec_alu_carry_en), .c_l_reg_sel(dec_l_reg_sel), .c_r_reg_sel(dec_r_reg_sel), .c_rf_ie(dec_rf_ie), 
    .dbg_pc(dbg_pc), .dbg_r0(dbg_r0), .i_submit(decode_execute_submit), .c_jump_cond_code(dec_jump_cond_code), .o_pc_update(execute_fetch_pc_update),
    .o_exec_pc(execute_fetch_pc), .i_jmp_predict(de_jmp_pred), .i_flush(fde_pipeline_flush), .o_flush(fde_pipeline_flush), .c_mem_access(dec_mem_access),
    .c_mem_we(dec_mem_we), .o_data(ew_data), .o_addr(ew_addr), .o_reg_ie(ew_reg_ie), .o_mem_access(ew_mem_access), .o_mem_we(ew_mem_we), .o_submit(ew_submit),
    .i_next_ready(ew_next_ready), .i_reg_ie(we_reg_ie), .i_reg_data(we_reg_data), .c_used_operands(dec_used_operands), .c_sreg_load(dec_sreg_load),
    .c_sreg_store(dec_sreg_store), .c_sreg_jal_over(dec_sreg_jal_over), .i_irq(i_irq), .c_sreg_irt(dec_sreg_irt), .o_c_instr_page(o_c_instr_page),
    .sr_bus_addr(sr_bus_addr), .sr_bus_data_o(sr_bus_data_o), .sr_bus_we(sr_bus_we), .o_icache_flush(o_icache_flush), .c_sys(dec_sys), .c_mem_width(dec_mem_width),
    .o_mem_width(ew_mem_width), .o_c_data_page(o_c_data_page), .i_mem_exception(i_mem_exception), .dbg_out(dbg_out[32:0]), .dbg_reg_sel(dbg_in[2:0]), .dbg_hold(dbg_in[3]),
    .i_core_int(i_mc_core_int), .i_core_int_sreg(i_core_int_sreg), .o_c_instr_long_mode(o_c_instr_long), .o_instr_addr_high(o_instr_long_addr), .c_mem_long(dec_mem_long),
    .o_mem_long_mode(ew_long_mode), .o_mem_addr_high(ew_addr_high), .c_wfi(dec_wfi));

// Pipeline stage 3 - MEM&WB
memwb memwb(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_data(ew_data), .i_addr(ew_addr), .i_reg_ie(ew_reg_ie), .i_mem_access(ew_mem_access), .i_mem_we(ew_mem_we),
    .o_reg_ie(we_reg_ie), .o_reg_data(we_reg_data), .i_submit(ew_submit), .o_ready(ew_next_ready), .o_mem_req(o_mem_req), .o_mem_addr(o_mem_addr),
    .o_mem_data(o_mem_data), .o_mem_we(o_mem_we), .i_mem_data(i_mem_data), .i_mem_ack(i_mem_ack), .i_mem_width(ew_mem_width), .o_mem_sel(o_mem_sel),
    .o_mem_exception(i_mem_exception), .dbg_out(dbg_out[35]), .i_addr_high(ew_addr_high), .i_mem_long(ew_long_mode), .o_mem_long(o_mem_long), .o_mem_addr_high(o_mem_addr_high));

endmodule


`define CORES 2

module intercore_sregs (
    input i_clk,
    input i_rst,

    input [`RW-1:0] c0_sr_bus_addr,
    input [`RW-1:0] c0_sr_bus_data_o,
    output [`RW-1:0] c0_sr_bus_data_i,
    input c0_sr_bus_we,

    input [`RW-1:0] c1_sr_bus_addr,
    input [`RW-1:0] c1_sr_bus_data_o,
    output [`RW-1:0] c1_sr_bus_data_i,
    input c1_sr_bus_we,

    output c0_disable,
    output c0_core_int,

    output c1_disable,
    output c1_core_int
);

`define SREG_ICINT_SET `RW'b1001
`define SREG_ICINT_RESET `RW'b1010
`define SREG_ICDISABLE `RW'b1011

wire [`CORES-1:0] ic_irq_set_read_w;
assign ic_irq_set_read_w[0] = (c0_sr_bus_addr == `SREG_ICINT_SET) && c0_sr_bus_we;
assign ic_irq_set_read_w[1] = (c1_sr_bus_addr == `SREG_ICINT_SET) && c1_sr_bus_we;

wire [`CORES-1:0] ic_irq_reset_w;
assign ic_irq_reset_w[0] = (c0_sr_bus_addr == `SREG_ICINT_RESET) && c0_sr_bus_we;
assign ic_irq_reset_w[1] = (c1_sr_bus_addr == `SREG_ICINT_RESET) && c1_sr_bus_we;

wire [`RW-1:0] sr_bus_dat [`CORES-1:0];
assign sr_bus_dat[0] = c0_sr_bus_data_o;
assign sr_bus_dat[1] = c1_sr_bus_data_o;

assign c0_sr_bus_data_i = {14'b0, ic_irq_state};
assign c1_sr_bus_data_i = {14'b0, ic_irq_state};

reg [`CORES-1:0] ic_irq_state;

always @(posedge i_clk) begin
    if (i_rst) begin
        ic_irq_state = 2'b0;
    end else begin
        if (ic_irq_reset_w[0])
            ic_irq_state = ic_irq_state & ~sr_bus_dat[0][`CORES-1:0];
        if (ic_irq_reset_w[1])
            ic_irq_state = ic_irq_state & ~sr_bus_dat[1][`CORES-1:0];
        
        if (ic_irq_set_read_w[0])
            ic_irq_state = ic_irq_state | sr_bus_dat[0][`CORES-1:0];
        if (ic_irq_set_read_w[1])
            ic_irq_state = ic_irq_state | sr_bus_dat[1][`CORES-1:0];
    end
end

wire c0_core_hold_we = (c0_sr_bus_addr == `SREG_ICDISABLE) && c0_sr_bus_we;

assign c0_disable = 1'b0;
assign c0_core_int = ic_irq_state[0];
assign c1_core_int = ic_irq_state[1];

reg core1_disable;
assign c1_disable = core1_disable;
always @(posedge i_clk) begin
    if (i_rst) begin
        core1_disable <= 1'b1;
    end else begin
        if (c0_core_hold_we)
            core1_disable <= sr_bus_dat[0][1];
    end
end

endmodule

module decode (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input wire i_clk,
    input wire i_rst,

    input [19:0] i_instr_l,
    input [`I_SIZE-17:0] i_imm_pass,
    output reg [`I_SIZE-17:0] o_imm_pass,
    input i_jmp_pred_pass,
    output reg o_jmp_pred_pass,

    // Pipeline control
    input i_next_ready,
    input i_submit,
    output o_ready,
    output reg o_submit,
    input i_flush,

    output reg oc_pc_inc, oc_pc_ie,
    output reg oc_r_bus_imm,
    output reg [`ALU_MODE_W-1:0] oc_alu_mode,
    output reg oc_alu_carry_en, oc_alu_flags_ie,
    output reg [`REGNO_LOG-1:0] oc_l_reg_sel, oc_r_reg_sel, 
    output reg [`REGNO-1:0] oc_rf_ie,
    output reg [`JUMP_CODE_W-1:0] oc_jump_cond_code,
    output reg oc_mem_access, oc_mem_we, oc_mem_width,
    output reg [1:0] oc_used_operands,
    output reg oc_sreg_load, oc_sreg_store, oc_sreg_jal_over, oc_sreg_irt, oc_sys,
    output reg oc_mem_long, oc_wfi,

    output dbg_out
);

// COMBINATIONAL INSTRUCTION DECODER

`define OPC_NOP 7'h00
`define OPC_MOV 7'h01
`define OPC_LDD 7'h02
`define OPC_LDO 7'h03
`define OPC_LDI 7'h04
`define OPC_STD 7'h05
`define OPC_STO 7'h06
`define OPC_ADD 7'h07
`define OPC_ADI 7'h08
`define OPC_ADC 7'h09
`define OPC_SUB 7'h0a
`define OPC_SUC 7'h0b
`define OPC_CMP 7'h0c
`define OPC_CMI 7'h0d
`define OPC_JMP 7'h0e
`define OPC_JAL 7'h0f
`define OPC_SRL 7'h10
`define OPC_SRS 7'h11
`define OPC_SYS 7'h12
`define OPC_AND 7'h13
`define OPC_ORR 7'h14
`define OPC_XOR 7'h15
`define OPC_ANI 7'h16
`define OPC_ORI 7'h17
`define OPC_XOI 7'h18
`define OPC_SHL 7'h19
`define OPC_SHR 7'h1a
`define OPC_CAI 7'h1b
`define OPC_MUL 7'h1c
`define OPC_DIV 7'h1d
`define OPC_IRT 7'h1e
`define OPC_LD8 7'h1f
`define OPC_LO8 7'h20
`define OPC_SD8 7'h21
`define OPC_SO8 7'h22
`define OPC_SLI 7'h23
`define OPC_SRI 7'h24
`define OPC_SAR 7'h25
`define OPC_SAI 7'h26
`define OPC_SEX 7'h27
`define OPC_LLO 7'h28
`define OPC_SLO 7'h29
`define OPC_LL8 7'h2a
`define OPC_SL8 7'h2b
`define OPC_MOD 7'h2c
`define OPC_WFI 7'h2d
// sreg imm??

wire [6:0] opcode = i_instr_l[6:0];
wire [3:0] reg_dst = i_instr_l[10:7];
wire [3:0] reg_st = i_instr_l[14:11];
wire [3:0] reg_nd = i_instr_l[19:16];

// Comb output signals
reg c_pc_inc, c_pc_ie;
reg c_r_bus_imm;
reg [`ALU_MODE_W-1:0] c_alu_mode;
reg c_alu_carry_en, c_alu_flags_ie;
reg [`REGNO_LOG-1:0] c_l_reg_sel, c_r_reg_sel; 
reg [`REGNO-1:0] c_rf_ie;
reg [`JUMP_CODE_W-1:0] c_jump_cond_code;
reg c_mem_access, c_mem_we, c_mem_width;
reg [1:0] c_used_operands;
reg c_sreg_load, c_sreg_store, c_sreg_jal_over, c_sreg_irt, c_sys, c_mem_long, c_wfi;

always @(*) begin
    // defaults
    c_pc_inc = 1'b1;
    {c_pc_ie, c_r_bus_imm, c_alu_carry_en, c_alu_flags_ie, c_mem_access,
        c_mem_we, c_sreg_load, c_sreg_store, c_sreg_jal_over, c_sreg_irt, c_sys, c_mem_width, c_mem_long, c_wfi} = 14'b0;
    c_rf_ie = `REGNO'b0;
    c_alu_mode = `ALU_MODE_W'b0;
    c_l_reg_sel = `REGNO_LOG'b0;
    c_r_reg_sel = `REGNO_LOG'b0;
    c_jump_cond_code = `JUMP_CODE_W'b0;
    c_used_operands = 2'b0;
    
    case (opcode)
        `OPC_NOP: begin
        end
        `OPC_MOV: begin
            c_alu_mode = `ALU_MODE_L_PASS;
            c_l_reg_sel = reg_st;
            c_rf_ie[reg_dst] = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_LDD: begin
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
        end
        `OPC_LDO: begin
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_LDI: begin
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
        end
        `OPC_STD: begin
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_used_operands = 2'b10;
        end
        `OPC_STO: begin
            c_l_reg_sel = reg_dst; // format exception to fit in new format (rd is register offset for address, previously rs2)
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_ADD: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_ADI: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_ADC: begin
            c_alu_mode = `ALU_MODE_ADD;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_alu_carry_en = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_SUB: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_SUC: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_r_bus_imm = 1'b1;
            c_alu_carry_en = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_CMP: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_CMI: begin
            c_alu_mode = `ALU_MODE_SUB;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_JMP: begin
            // NOTE: Conditional jumps are decoded in execute stage
            // to not unnecessarily stall the pipeline each time prediction
            // is taken
            c_pc_inc = 1'b0;
            c_pc_ie = 1'b0;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_bus_imm = 1'b1;
            c_jump_cond_code = {1'b1, i_instr_l[10:7]};
        end
        `OPC_JAL: begin
            c_pc_ie = 1'b0;
            c_pc_inc = 1'b0;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_bus_imm = 1'b1;
            c_sreg_jal_over = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            // set jump code to unconditional jump, pred is set by fetch
            c_jump_cond_code = {1'b1, 4'b0};
        end
        `OPC_SRL: begin
            c_sreg_load = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
        end
        `OPC_SRS: begin
            c_r_reg_sel = reg_st;
            c_sreg_store = 1'b1;
            c_used_operands = 2'b10;
        end
        `OPC_AND: begin
            c_alu_mode = `ALU_MODE_AND;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_ORR: begin
            c_alu_mode = `ALU_MODE_OR;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_XOR: begin
            c_alu_mode = `ALU_MODE_XOR;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_ANI: begin
            c_alu_mode = `ALU_MODE_AND;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_ORI: begin
            c_alu_mode = `ALU_MODE_OR;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_XOI: begin
            c_alu_mode = `ALU_MODE_XOR;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_SHL: begin
            c_alu_mode = `ALU_MODE_SHL;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_SHR: begin
            c_alu_mode = `ALU_MODE_SHR;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_CAI: begin
            c_alu_mode = `ALU_MODE_AND;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_MUL: begin
            c_alu_mode = `ALU_MODE_MUL;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_DIV: begin
            c_alu_mode = `ALU_MODE_DIV;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_SYS: begin
            c_sys = 1'b1;
        end
        `OPC_IRT: begin
            c_sreg_irt = 1'b1;
            c_pc_ie = 1'b0;
            c_pc_inc = 1'b0;
        end
        `OPC_LD8: begin
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
            c_mem_width = 1'b1;
        end
        `OPC_LO8: begin
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
            c_used_operands = 2'b01;
            c_mem_width = 1'b1;
        end
        `OPC_SD8: begin
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_R_PASS;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_used_operands = 2'b10;
            c_mem_width = 1'b1;
        end
        `OPC_SO8: begin
            c_l_reg_sel = reg_dst;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_used_operands = 2'b11;
            c_mem_width = 1'b1;
        end
        `OPC_SLI: begin
            c_alu_mode = `ALU_MODE_SHL;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_SRI: begin
            c_alu_mode = `ALU_MODE_SHR;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_SAR: begin
            c_alu_mode = `ALU_MODE_ASHR;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_SAI: begin
            c_alu_mode = `ALU_MODE_ASHR;
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_SEX: begin
            c_alu_mode = `ALU_MODE_SEXT;
            c_l_reg_sel = reg_st;
            c_rf_ie[reg_dst] = 1'b1;
            c_alu_flags_ie = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_LLO: begin
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
            c_mem_long = 1'b1;
            c_used_operands = 2'b01;
        end
        `OPC_SLO: begin
            c_l_reg_sel = reg_dst;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_mem_long = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_LL8: begin
            c_l_reg_sel = reg_st;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_rf_ie[reg_dst] = 1'b1;
            c_mem_access = 1'b1;
            c_used_operands = 2'b01;
            c_mem_long = 1'b1;
            c_mem_width = 1'b1;
        end
        `OPC_SL8: begin
            c_l_reg_sel = reg_dst;
            c_r_bus_imm = 1'b1;
            c_alu_mode = `ALU_MODE_ADD;
            c_r_reg_sel = reg_st;
            c_mem_access = 1'b1;
            c_mem_we = 1'b1;
            c_used_operands = 2'b11;
            c_mem_long = 1'b1;
            c_mem_width = 1'b1;
        end
        `OPC_MOD: begin
            c_alu_mode = `ALU_MODE_MOD;
            c_l_reg_sel = reg_st;
            c_r_reg_sel = reg_nd;
            c_rf_ie[reg_dst] = 1'b1;
            c_used_operands = 2'b11;
        end
        `OPC_WFI: begin
            c_pc_inc = 1'b0;
            c_wfi = 1'b1;
        end
        default: begin
        end
    endcase
end


// PIPELINE WRAPPER
reg input_valid;

// ready signal must be combinational to be registered on time by previous stage
assign o_ready = i_next_ready & ~input_valid;

always @(posedge i_clk) begin
    if(i_rst) begin
        o_submit <= 1'b0;
        input_valid <= 1'b0;
    end else begin
        // default bubble
        o_submit <= 1'b0;

        if (i_next_ready & (i_submit | input_valid) & ~i_flush) begin
            o_submit <= 1'b1;
            input_valid <= 1'b0;

            o_imm_pass <= i_imm_pass;
            o_jmp_pred_pass <= i_jmp_pred_pass;
            // Submit control signals
            oc_pc_inc <= c_pc_inc;
            oc_pc_ie <= c_pc_ie; 
            oc_r_bus_imm <= c_r_bus_imm;
            oc_alu_carry_en <= c_alu_carry_en;
            oc_alu_flags_ie <= c_alu_flags_ie;
            oc_rf_ie <= c_rf_ie;
            oc_alu_mode <= c_alu_mode;
            oc_l_reg_sel <= c_l_reg_sel;
            oc_r_reg_sel <= c_r_reg_sel;
            oc_jump_cond_code <= c_jump_cond_code;
            oc_mem_access <= c_mem_access;
            oc_mem_we <= c_mem_we;
            oc_mem_width <= c_mem_width;
            oc_used_operands <= c_used_operands;
            oc_sreg_load <= c_sreg_load;
            oc_sreg_store <= c_sreg_store;
            oc_sreg_jal_over <= c_sreg_jal_over;
            oc_sreg_irt <= c_sreg_irt;
            oc_sys <= c_sys;
            oc_mem_long <= c_mem_long;
            oc_wfi <= c_wfi;
        end

        if (i_submit & ~i_next_ready & ~i_flush) begin
            input_valid <= 1'b1; // don't overwrite buffer
        end

        if (i_flush) begin
            o_submit <= 1'b0;
            input_valid <= 1'b0;
        end 
    end
end

assign dbg_out = o_ready;

endmodule


module mem_dcache_arb (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk,
    input i_rst,

    output mem_req,
    output reg mem_we,
    input mem_ack,
    output reg [`WB_ADDR_W-1:0] mem_addr,
    output reg [`RW-1:0] mem_o_data,
    input [`RW-1:0] mem_i_data,
    output reg [1:0] mem_sel,
    output reg mem_cache_enable,
    input mem_exception,

    input mem0_req,
    input mem0_we,
    output reg mem0_ack,
    input [`WB_ADDR_W-1:0] mem0_addr,
    input [`RW-1:0] mem0_o_data,
    output reg [`RW-1:0] mem0_i_data,
    input [1:0] mem0_sel,
    input mem0_cache_enable,
    output reg mem0_exception,

    input mem1_req,
    input mem1_we,
    output reg mem1_ack,
    input [`WB_ADDR_W-1:0] mem1_addr,
    input [`RW-1:0] mem1_o_data,
    output reg [`RW-1:0] mem1_i_data,
    input [1:0] mem1_sel,
    input mem1_cache_enable,
    output reg mem1_exception
);
    
wire request_term = mem_ack | mem_exception;

reg select, transfer_active;
reg req0_pending, req1_pending;

wire req_w = mem0_req | mem1_req | req0_pending | req1_pending;

wire req_start = ~transfer_active & req_w;

assign mem_req = req_start;

always @(posedge i_clk) begin
    if (i_rst) begin
        select <= 1'b0;
        transfer_active <= 1'b0;
    end else if (req_start & ~request_term) begin
        select <= req_sel;
        transfer_active <= 1'b1;
    end else if (transfer_active & request_term) begin
        transfer_active <= 1'b0;
    end
end

always @(posedge i_clk) begin
    if (i_rst) begin
        req0_pending <= 1'b0;
        req1_pending <= 1'b0;
    end else if (req_start & ~req_sel) begin
        req0_pending <= 1'b0;
        req1_pending <= req1_pending | mem1_req;
    end else if (req_start & req_sel) begin
        req1_pending <= 1'b0;
        req0_pending <= req0_pending | mem0_req;
    end else if (transfer_active) begin
        req0_pending <= (req0_pending | mem0_req) & select;
        req1_pending <= (req1_pending | mem1_req) & ~select;
    end
end

reg req_sel;
always @* begin // round robin
    if (select == 1'b0 && (mem1_req | req1_pending))
        req_sel = 1'b1;
    else if (select == 1'b1 && (mem0_req | req0_pending))
        req_sel = 1'b0;
    else if (mem0_req | req0_pending)
        req_sel = 1'b0;
    else if (mem1_req | req1_pending)
        req_sel = 1'b1;
    else
        req_sel = 1'b0;
end

wire select_wire = (req_start ? req_sel : select);

always @(*) begin
    {mem0_ack, mem0_i_data, mem0_exception, mem1_ack, mem1_i_data, mem1_exception} = 'b0;
    if(~select_wire) begin
        mem_we = mem0_we;
        mem_addr = mem0_addr;
        mem_o_data = mem0_o_data;
        mem_sel = mem0_sel;
        mem_cache_enable = mem0_cache_enable;
        mem0_ack = mem_ack;
        mem0_i_data = mem_i_data;
        mem0_exception = mem_exception;
    end else begin
        mem_we = mem1_we;
        mem_addr = mem1_addr;
        mem_o_data = mem1_o_data;
        mem_sel = mem1_sel;
        mem_cache_enable = mem1_cache_enable;
        mem1_ack = mem_ack;
        mem1_i_data = mem_i_data;
        mem1_exception = mem_exception;
    end
end

endmodule


module execute #(parameter CORENO = 0, INT_VEC = 1) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    // Pipeline control singnals
    output o_ready,
    input i_submit,
    output reg o_flush,
    input i_flush,

    input [`RW-1:0] i_imm,
    input i_jmp_predict,

    // Execution control singals
    input c_pc_inc, c_pc_ie,
    input c_r_bus_imm,
    input [`ALU_MODE_W-1:0] c_alu_mode,
    input c_alu_carry_en, c_alu_flags_ie,
    input [`REGNO_LOG-1:0] c_l_reg_sel, c_r_reg_sel, 
    input [`REGNO-1:0] c_rf_ie,
    input [`JUMP_CODE_W-1:0] c_jump_cond_code,
    input c_mem_access, c_mem_we, c_mem_width,
    input [1:0] c_used_operands,
    input c_sreg_load, c_sreg_store, c_sreg_jal_over, c_sreg_irt, c_sys, c_mem_long, c_wfi,

    // Signals to fetch stage to handle mispredictions
    output o_pc_update,
    output [`RW-1:0] o_exec_pc,

    // Debug outputs
    output [`RW-1:0] dbg_r0, dbg_pc,

    // Pipeline next stage
    output reg [`RW-1:0] o_data,
    output reg [`RW-1:0] o_addr,
    output reg [`REGNO-1:0] o_reg_ie,
    output reg o_mem_access, o_mem_we, o_mem_width,
    output reg o_submit,
    input i_next_ready,
    input [`REGNO-1:0] i_reg_ie,
    input [`RW-1:0] i_reg_data,

    input i_irq,
    output o_c_instr_page,
    output reg o_c_data_page,
    output [`RW-1:0] sr_bus_addr, sr_bus_data_o,
    output sr_bus_we,
    output reg o_icache_flush,
    input i_mem_exception,
    input i_core_int,
    input [`RW-1:0] i_core_int_sreg,
    output o_c_instr_long_mode,
    output reg o_mem_long_mode,
    output [7:0] o_instr_addr_high,
    output reg [7:0] o_mem_addr_high,

    output [32:0] dbg_out,
    input dbg_hold,
    input [`REGNO_LOG-1:0] dbg_reg_sel
);

reg next_ready_delayed;
// detect RAW pipeline hazard
wire raw_hazard = (
        (c_used_operands[0] & o_reg_ie[c_l_reg_sel]) |
        (c_used_operands[1] & o_reg_ie[c_r_reg_sel]) |
        (c_mem_long & sreg_long_ptr_en & c_used_operands[0] & o_reg_ie[c_l_reg_sel+1])
    ) & (o_submit | ~next_ready_delayed);
// hazard happens also in the first cycle when next_ready becomes high, delayed signal is used 

wire i_invalidate = i_flush | irq | pc_high_updated;
// hazard doesn't invalidate instructions, only holds it
wire hold_req = raw_hazard | dbg_hold | alu_mul_busy;

wire i_valid = i_submit & ~i_invalidate;
reg hold_valid;

wire instr_valid = i_valid | (hold_valid & ~i_submit & ~i_invalidate);
wire exec_submit = i_next_ready & instr_valid & ~hold_req & ~c_wfi;

// don't update state when current instruction is not valid (flush or bubble)
assign o_ready = exec_submit | ~instr_valid;

// At IRQ, current instruction (and state update) is invalidated and pc is saved to sr, to
// continue execution from current instruction. Flush is requested on next cycle
wire irq = ((i_irq | i_core_int) & irq_en) | prev_sys | trap_exception | i_mem_exception;
// core_int is masked as externel event to not interrupt irq handler

wire multicycle_submit = (i_valid & ~raw_hazard) | (instr_valid & ~raw_hazard & ~multicycle_submited);

always @(posedge i_clk) begin
    if(i_rst) begin
        hold_valid <= 1'b0;
    end else if (i_invalidate | exec_submit) begin
        hold_valid <= 1'b0;
    end else if (i_valid) begin
        hold_valid <= 1'b1;
    end
end

always @(posedge i_clk) begin
    if(i_rst) begin
        next_ready_delayed <= 1'b0;
    end else begin
        next_ready_delayed <= i_next_ready;
    end
end

// Internal buses
wire [`RW-1:0] reg_l_con, reg_r_con;
wire [`RW-1:0] alu_l_bus, alu_r_bus;
wire [`RW-1:0] alu_bus;

// Muxes definitions
assign alu_l_bus = reg_l_con;
assign alu_r_bus = (c_r_bus_imm ? i_imm : reg_r_con);
assign alu_bus = (mul_div_op ? alu_mul_res : alu_res);

// Component connects
wire [`RW-1:0] pc_val;
wire [`ALU_FLAG_CNT-1:0] alu_flags_d, alu_flags_q;
assign dbg_pc = pc_val;
assign o_pc_update = exec_submit;
assign o_exec_pc = pc_val;
wire [`RW-1:0] sreg_in = reg_r_con;
reg [`RW-1:0] sreg_out;
wire [`RW-1:0] dbg_reg_out;
wire pc_overflow;
wire [`RW-1:0] reg_l_high_con;
wire [`RW-1:0] alu_mul_res, alu_res;
wire alu_mul_busy;
wire mul_div_op = (c_alu_mode == `ALU_MODE_MUL) || (c_alu_mode == `ALU_MODE_DIV) || (c_alu_mode == `ALU_MODE_MOD);

// Submodules
rf rf(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(i_reg_data), .o_lout(reg_l_con),
    .o_rout(reg_r_con), .i_lout_sel(c_l_reg_sel), .i_rout_sel(c_r_reg_sel),
    .i_ie(i_reg_ie), .i_gie(1'b1), .o_l_high_out(reg_l_high_con),
    .dbg_r0(dbg_r0), .dbg_sel(dbg_reg_sel), .dbg_reg(dbg_reg_out));

alu alu(
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_l(alu_l_bus), .i_r(alu_r_bus), .o_out(alu_res), .i_mode(c_alu_mode), 
    .o_flags(alu_flags_d), .i_carry(alu_flags_q[`ALU_FLAG_C] & c_alu_carry_en));

alu_mul_div alu_mul_div (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk),
    .i_rst(i_rst),

    .i_a(alu_l_bus), .i_b(alu_r_bus), .o_d(alu_mul_res),
    .i_submit(mul_div_op & multicycle_submit), .i_flush(i_invalidate),
    .o_busy(alu_mul_busy),
    .i_mul(c_alu_mode == `ALU_MODE_MUL),
    .i_div(c_alu_mode == `ALU_MODE_DIV),
    .i_mod(c_alu_mode == `ALU_MODE_MOD)
);

pc #(.INT_VEC(INT_VEC)) pc (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_bus(c_sreg_store | c_sreg_irt ? (c_sreg_irt ? sreg_out : sreg_in) : alu_bus),
    .i_c_pc_inc((c_pc_inc | (~jump_dec_en & jump_dec_valid)) & exec_submit), .i_c_pc_ie((c_pc_ie | (jump_dec_en & jump_dec_valid) | pc_write) & exec_submit),
    .o_pc(pc_val), .i_c_pc_irq(irq), .o_pc_ovf(pc_overflow));

// Cpu control registers
register  #(.N(`ALU_FLAG_CNT)) alu_flag_reg (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d((alu_flags_sreg_ie ? sreg_in[`ALU_FLAG_CNT-1:0] : alu_flags_d)),
    .o_d(alu_flags_q), .i_ie((c_alu_flags_ie | alu_flags_sreg_ie) & exec_submit));

// JUMP DECODE
reg jump_dec_en;
wire jump_dec_valid = c_jump_cond_code[`JUMP_CODE_BIT_EN];

wire jump_mispredict = jump_dec_valid & (jump_dec_en ^ i_jmp_predict);
wire pc_write = (pc_sreg_ie & c_sreg_store) | c_sreg_irt;

always @(posedge i_clk) begin
    o_flush <= ((jump_mispredict | pc_write | flush_instr_mmu) & exec_submit) | irq | pc_high_updated; // invalidate itself and all previous stages at next cycle
end

`define JUMP_CODE_UNCOND`JUMP_CODE_W'b10000
`define JUMP_CODE_CARRY `JUMP_CODE_W'b10001
`define JUMP_CODE_EQUAL `JUMP_CODE_W'b10010
`define JUMP_CODE_LT    `JUMP_CODE_W'b10011
`define JUMP_CODE_GT    `JUMP_CODE_W'b10100
`define JUMP_CODE_LE    `JUMP_CODE_W'b10101
`define JUMP_CODE_GE    `JUMP_CODE_W'b10110
`define JUMP_CODE_NE    `JUMP_CODE_W'b10111
`define JUMP_CODE_OVF   `JUMP_CODE_W'b11000
`define JUMP_CODE_PAR   `JUMP_CODE_W'b11001
`define JUMP_CODE_GTU   `JUMP_CODE_W'b11010
`define JUMP_CODE_GEU   `JUMP_CODE_W'b11011
`define JUMP_CODE_LEU   `JUMP_CODE_W'b11100
always @(*) begin
    case (c_jump_cond_code[`JUMP_CODE_W-1:0])
        `JUMP_CODE_UNCOND:
            jump_dec_en = 1'b1;
        `JUMP_CODE_CARRY:
            jump_dec_en = alu_flags_q[`ALU_FLAG_C];
        `JUMP_CODE_EQUAL:
            jump_dec_en = alu_flags_q[`ALU_FLAG_Z];
        `JUMP_CODE_LT:
            jump_dec_en = alu_flags_q[`ALU_FLAG_N];
        `JUMP_CODE_GT:
            jump_dec_en = ~(alu_flags_q[`ALU_FLAG_N] | alu_flags_q[`ALU_FLAG_Z]);
        `JUMP_CODE_LE:
            jump_dec_en = alu_flags_q[`ALU_FLAG_N] | alu_flags_q[`ALU_FLAG_Z];
        `JUMP_CODE_GE:
            jump_dec_en = ~alu_flags_q[`ALU_FLAG_N];
        `JUMP_CODE_NE:
            jump_dec_en = ~alu_flags_q[`ALU_FLAG_Z];
        `JUMP_CODE_OVF:
            jump_dec_en = alu_flags_q[`ALU_FLAG_O];
        `JUMP_CODE_PAR:
            jump_dec_en = alu_flags_q[`ALU_FLAG_P];
        `JUMP_CODE_GTU:
            jump_dec_en = ~(alu_flags_q[`ALU_FLAG_C] | alu_flags_q[`ALU_FLAG_Z]);
        `JUMP_CODE_GEU:
            jump_dec_en = ~alu_flags_q[`ALU_FLAG_C];
        `JUMP_CODE_LEU:
            jump_dec_en = alu_flags_q[`ALU_FLAG_C] | alu_flags_q[`ALU_FLAG_Z];
        default:
            jump_dec_en = 1'b0;
    endcase
end

// Forwarding to next pipeline stage
always @(posedge i_clk) begin
    if (i_rst) begin
        o_submit <= 1'b0;
    end else if (exec_submit) begin
        o_addr <= alu_bus;
        o_data <= (c_mem_access ? reg_r_con : 
            (c_sreg_load | c_sreg_jal_over ? sreg_out + (c_sreg_jal_over ? `RW'b1 : `RW'b0) 
                                            : alu_bus)); 
        o_reg_ie <= c_rf_ie;
        o_mem_access <= c_mem_access;
        o_mem_we <= c_mem_we;
        o_mem_width <= c_mem_width;
        o_mem_long_mode <= c_mem_long & sreg_long_ptr_en;
        o_mem_addr_high <= computed_mem_addr_high;
        o_submit <= 1'b1;
    end else begin
        o_submit <= 1'b0;
    end
end

reg prev_sys; // Execute sys instruction and trigger interrupt at next cycle to resume from next instruction
always @(posedge i_clk) begin
    if (i_rst) begin
        prev_sys <= 1'b0;
    end else if (c_sys & exec_submit) begin
        prev_sys <= 1'b1;
    end else begin
        prev_sys <= 1'b0;
    end
end

reg trap_exception;
always @(posedge i_clk) begin
    if (i_rst) begin
        trap_exception <= 1'b0;
    end else begin
        trap_exception <= trap_flag & exec_submit;
    end
end

reg [`RW-1:0] mem_stage_pc;
always @(posedge i_clk) begin
    if (i_rst)
        mem_stage_pc <= `RW'b0;
    else if (exec_submit)
        mem_stage_pc <= pc_val;
end

reg multicycle_submited;
always @(posedge i_clk) begin
    if (i_rst) begin
        multicycle_submited <= 1'b0;
    end else if (multicycle_submit) begin
        multicycle_submited <= 1'b1;
    end else if (i_submit) begin
        multicycle_submited <= 1'b0;
    end
end

// Special registers
`define SREG_PC `RW'b0
`define SREG_PRIV_CTRL `RW'b1
`define SREG_JTR `RW'b10
`define SREG_IRQ_PC `RW'b11
`define SREG_ALU_FLAGS `RW'b100
`define SREG_IRQ_FLAGS `RW'b101
`define SREG_SCRATCH `RW'b110
`define SREG_CPUID `RW'b111
`define SREG_COREID `RW'b1000
`define SREG_MT_IRQ `RW'b1001 //,1010, 1011
`define SREG_PC_HIGH `RW'b1100
`define SREG_PC_HIGH_BUFF `RW'b1101

reg pc_sreg_ie, sreg_priv_control_ie, sreg_irq_pc_ie, alu_flags_sreg_ie, sreg_jtr_ie, sreg_scratch_ie, sreg_pc_high_ie, sreg_pc_high_buff_ie;
wire [`RW-1:0] sreg_priv_control_out, sreg_irq_pc_out, sreg_scratch_out;
always @* begin
    {pc_sreg_ie, sreg_irq_pc_ie, sreg_priv_control_ie, alu_flags_sreg_ie, sreg_jtr_ie, sreg_scratch_ie, sreg_pc_high_ie, sreg_pc_high_buff_ie} = 8'b0;
    case (i_imm)
        `SREG_PC: begin
            sreg_out = pc_val;
            pc_sreg_ie = c_sreg_store;
        end
        `SREG_PRIV_CTRL: begin
            sreg_out = sreg_priv_control_out;
            sreg_priv_control_ie = c_sreg_store;
        end
        `SREG_IRQ_PC: begin
            sreg_out = sreg_irq_pc_out;
            sreg_irq_pc_ie = c_sreg_store;
        end
        `SREG_JTR: begin
            sreg_out = {13'b0, sreg_jtr_out};
            sreg_jtr_ie = c_sreg_store & sreg_priv_mode;
        end
        `SREG_ALU_FLAGS: begin
            sreg_out = {11'b0, alu_flags_q};
            alu_flags_sreg_ie = c_sreg_store;
        end
        `SREG_IRQ_FLAGS: begin
            sreg_out = {11'b0, sreg_irq_flags_out};
        end
        `SREG_SCRATCH: begin
            sreg_out = sreg_scratch_out;
            sreg_scratch_ie = c_sreg_store;
        end
        `SREG_CPUID: begin
            sreg_out = 16'b1111_0000_0011_0100;
        end
        `SREG_COREID: begin
            sreg_out = CORENO;
        end
        `SREG_MT_IRQ: begin // write is handled in upper_core
            sreg_out = i_core_int_sreg;
        end
        `SREG_PC_HIGH: begin
            sreg_pc_high_ie = c_sreg_store;
            sreg_out = {8'b0, pc_high_out};
        end
        `SREG_PC_HIGH_BUFF: begin
            sreg_pc_high_buff_ie = c_sreg_store;
            sreg_out = {8'b0, pc_high_buff_out};
        end 
        default:
            sreg_out = 16'b0;
    endcase

    if(c_sreg_jal_over)
        sreg_out = pc_val;
    if(c_sreg_irt)
        sreg_out = sreg_irq_pc_out;
end

// Special registers control


wire [`RW-1:0] priv_in = (irq ? (`RW'b0001) : (c_sreg_irt ? (sreg_priv_control_out | `RW'h0004) : sreg_in)); // disable irq and paging flag on interrupt and re-enable on return
register #(.RESET_VAL(`RW'b0001)) sreg_priv_control (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(priv_in), .o_d(sreg_priv_control_out),
    .i_ie((((sreg_priv_control_ie & sreg_priv_mode) | c_sreg_irt) & exec_submit) | irq));

wire sreg_priv_mode = sreg_priv_control_out[0];
wire sreg_data_page = sreg_priv_control_out[1];
wire irq_en = sreg_priv_control_out[2];
wire sreg_long_ptr_en = sreg_priv_control_out[3];

register sreg_irq_pc (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst),
    .i_d(i_mem_exception ? mem_stage_pc : (c_wfi ? pc_val+16'b1 : (irq ? pc_val : sreg_in))), .o_d(sreg_irq_pc_out), .i_ie(irq | (sreg_irq_pc_ie & exec_submit))
);

wire [2:0] sreg_jtr_buff_o, sreg_jtr_out;
wire jtr_jump_en = (pc_sreg_ie | jump_dec_valid | c_sreg_irt) & exec_submit;
wire jtr_irqh_write = irq;
wire [2:0] jtr_buff_in = (irq ? 3'b000 : sreg_in[2:0]);
wire [2:0] jtr_in = (irq ? 3'b000 : sreg_jtr_buff_o);
register  #(.RESET_VAL((CORENO == 0 ? 3'b001 : 3'b000)), .N(3)) sreg_jtr_buff (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(jtr_buff_in), .o_d(sreg_jtr_buff_o), .i_ie(sreg_jtr_ie | jtr_irqh_write));
register  #(.RESET_VAL((CORENO == 0 ? 3'b001 : 3'b000)), .N(3)) sreg_jtr (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(jtr_in), .o_d(sreg_jtr_out), .i_ie(jtr_jump_en | jtr_irqh_write));
assign o_c_instr_page = sreg_jtr_out[0];
wire trap_flag = sreg_jtr_out[1];
wire long_pc_mode = sreg_jtr_out[2];

register sreg_scratch (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(sreg_in), .o_d(sreg_scratch_out), .i_ie(sreg_scratch_ie & exec_submit));

wire [4:0] sreg_irq_flags_in = {(i_core_int & irq_en), i_mem_exception, trap_exception, prev_sys, (i_irq & irq_en)};
wire [4:0] sreg_irq_flags_out;
register #(.N(5)) sreg_irq_flags (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_d(sreg_irq_flags_in), .o_d(sreg_irq_flags_out), .i_ie(irq));

wire immu_write = c_sreg_store & exec_submit & (sr_bus_addr >= `RW'h100 && sr_bus_addr < `RW'h100 + 16); // flush after write to mmu is executed
wire flush_instr_mmu = (immu_write & o_c_instr_page) | ((jtr_in[0] ^ sreg_jtr_out[0]) & (jtr_jump_en | jtr_irqh_write)) | pc_high_updated;
always @(posedge i_clk)
    o_icache_flush <= flush_instr_mmu & ~i_rst;

// Delays disable of data paging in case of interrupt. MEMWB stage is still executing and changing
// address during request would break commited result. In other cases special handling is not needed,
// becaue sregs are updated only when next stage is ready
always @(posedge i_clk) begin
    if (i_rst)
        o_c_data_page <= 1'b0;
    else if (i_next_ready)
        o_c_data_page <= sreg_data_page;
end

// Higher part of PC in long pointer mode
// long pc mode is disabled on interrupt and reabled via jtr at irt, bot registers can be recovered
wire [7:0] pc_high = (long_pc_mode ? pc_high_out : 'b0);

wire pc_high_ovf = pc_overflow & long_pc_mode & exec_submit;
wire pc_high_jtr = (pc_sreg_ie | jump_dec_valid | c_sreg_irt) & long_pc_mode & exec_submit;
// Update from buffer at jump or at pc overflow
wire [7:0] pc_high_in = (sreg_pc_high_ie ? sreg_in[7:0] : (pc_high_jtr ? pc_high_buff_out : pc_high_out+7'b1));
wire [7:0] pc_high_out;
register #(.N(8), .RESET_VAL(8'h80)) pc_high_reg (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst),
    .i_d(pc_high_in),
    .o_d(pc_high_out),
    .i_ie((pc_high_ovf | pc_high_jtr | sreg_pc_high_ie) & exec_submit)
);

// Update buffer on function call (JAL) with current pointer
wire [7:0] pc_high_buff_in = (sreg_pc_high_buff_ie ? sreg_in[7:0] : pc_high);
wire [7:0] pc_high_buff_out;
register #(.N(8), .RESET_VAL(8'h80)) pc_high_buff_reg (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst),
    .i_d(pc_high_buff_in),
    .o_d(pc_high_buff_out),
    .i_ie((sreg_pc_high_buff_ie | (c_sreg_jal_over & long_pc_mode)) & exec_submit)
);

wire pc_high_updated = |(prev_pc_high ^ pc_high); // flush pipeline and cache on update. current instuction is incorrect
reg [7:0] prev_pc_high;
always @(posedge i_clk) begin
    if (i_rst) prev_pc_high <= 'b0;
    else prev_pc_high <= pc_high;
end

assign o_c_instr_long_mode = long_pc_mode;
assign o_instr_addr_high = pc_high;

wire [7:0] computed_mem_addr_high = (~alu_flags_d[`ALU_FLAG_C] & alu_r_bus[`RW-1] ? reg_l_high_con[7:0]-8'b1 : 
    (alu_flags_d[`ALU_FLAG_C] ? reg_l_high_con[7:0]+8'b1 : reg_l_high_con[7:0]));

assign sr_bus_addr = i_imm;
assign sr_bus_we = c_sreg_store & sreg_priv_mode & exec_submit;
assign sr_bus_data_o = sreg_in;

assign dbg_out = {o_ready, pc_val, dbg_reg_out};

endmodule


module wishbone_arbiter (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input i_wb0_cyc,
    input i_wb1_cyc,

    output o_wb_cyc,

    input wb0_stb,
    input [`WB_ADDR_W-1:0] wb0_adr,
    input [`RW-1:0] wb0_o_dat,
    input wb0_we,
    input [1:0] wb0_sel,
    input wb0_8_burst, wb0_4_burst,
    output reg wb0_ack,
    output reg wb0_err,

    input wb1_stb,
    input [`WB_ADDR_W-1:0] wb1_adr,
    input [`RW-1:0] wb1_o_dat,
    input wb1_we,
    input [1:0] wb1_sel,
    input wb1_8_burst, wb1_4_burst,
    output reg wb1_ack,
    output reg wb1_err,

    output reg owb_stb,
    output reg [`WB_ADDR_W-1:0] owb_adr,
    output reg [`RW-1:0] owb_o_dat,
    output reg owb_we,
    output reg [1:0] owb_sel,
    input owb_ack,
    input owb_err,
    output reg owb_8_burst, owb_4_burst
);

reg o_sel_sig;

wire bus_req = i_wb0_cyc | i_wb1_cyc;

assign o_wb_cyc = (o_sel_sig ? i_wb1_cyc : i_wb0_cyc) & ~i_rst;

always @(posedge i_clk) begin
    if(i_rst) begin
        o_sel_sig <= 1'b0;
    end else if(~o_wb_cyc & bus_req) begin
        o_sel_sig <= (i_wb0_cyc ? 1'b0 : 1'b1);
    end
end

always @(*) begin
    if(~o_sel_sig) begin
        owb_stb =  wb0_stb;
        owb_o_dat =  wb0_o_dat;
        owb_adr =  wb0_adr;
        owb_we =  wb0_we;
        owb_sel =  wb0_sel;
        wb0_ack = owb_ack;
        wb0_err = owb_err;
        wb1_ack = 1'b0;
        wb1_err = 1'b0;
        owb_4_burst =  wb0_4_burst;
        owb_8_burst = wb0_8_burst;
    end else begin
        owb_stb =  wb1_stb;
        owb_o_dat =  wb1_o_dat;
        owb_adr =  wb1_adr;
        owb_we =  wb1_we;
        owb_sel =  wb1_sel;
        wb1_ack = owb_ack;
        wb1_err = owb_err;
        wb0_ack = 1'b0;
        wb0_err = 1'b0;
        owb_4_burst =  wb1_4_burst;
        owb_8_burst =  wb1_8_burst;
    end
end

endmodule


// Instruction fetch stage v2

module fetch (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    output [`RW-1:0] mem_addr, // address must be valid only if submit is set (registered on other end)
    input [`I_SIZE-1:0] mem_data,
    input mem_ack,
    output mem_submit, // pipelined submit signal
    
    input i_next_ready,
    output reg o_submit,
    input i_flush,
 
    output reg [`I_SIZE-1:0] o_instr,
    output reg o_jmp_predict,

    input [`RW-1:0] i_exec_pc,

    output dbg_out
);

assign mem_addr = (pc_reset_override ? `RW'b0 : ((i_flush | pc_flush_override) ? i_exec_pc : pred_pc));
assign mem_submit = (((mem_ack & ~out_buffer_valid & i_next_ready & ~disable_prediction) | pc_reset_override | (pc_flush_override & ~instr_wait) | (out_buffer_valid & out_buff_read & ~disable_prediction))) & ~i_rst;

reg pc_reset_override;
always @(posedge i_clk) begin
    if (i_rst) begin
        pc_reset_override <= 1'b1;
    end else if (mem_submit) begin
        pc_reset_override <= 1'b0;
    end
end

reg pc_flush_override;
always @(posedge i_clk) begin
    if (i_rst | mem_submit)
        pc_flush_override <= 1'b0;
    else if (i_flush)
        pc_flush_override <= 1'b1;
end

reg flush_event_invalidate;
always @(posedge i_clk) begin
    if (i_rst | mem_ack)
        flush_event_invalidate <= 1'b0;
    else if (i_flush & instr_wait & ~mem_ack)
        flush_event_invalidate <= 1'b1;
end

reg instr_wait;
always @(posedge i_clk) begin
    if (i_rst)
        instr_wait <= 1'b0;
    else if (mem_submit)
        instr_wait <= 1'b1;
    else if (mem_ack)
        instr_wait <= 1'b0;
end

wire out_ready = out_buffer_valid | mem_ack;
wire [`I_SIZE-1:0] out_instr = (out_buffer_valid ? out_buffer_data_instr : mem_data);
wire out_jmp_predict = (out_buffer_valid ? out_buffer_data_pred : current_req_branch_pred);
wire submitable = out_ready & i_next_ready & ~(i_flush | flush_event_invalidate);
always @(posedge i_clk) begin
    o_submit <= submitable;
    if (submitable) begin
        o_instr <= out_instr;
        o_jmp_predict <= out_jmp_predict;
    end
end

reg [`RW-1:0] prev_request_pc;
always @(posedge i_clk) begin
    if(mem_submit)
        prev_request_pc <= mem_addr;
end

wire current_req_branch_pred = (mem_ack ? branch_pred_res : prev_req_branch_pred);
reg prev_req_branch_pred;
always @(posedge i_clk) begin
    if(mem_ack)
        prev_req_branch_pred <= branch_pred_res;
end

wire [`RW-1:0] pred_pc = (branch_pred_res ? branch_pred_imm : prev_request_pc + `RW'b1);

wire [`I_SIZE-1:0] branch_pred_instr = (out_buffer_valid ? out_buffer_data_instr : mem_data);
wire [`RW-1:0] branch_pred_imm = branch_pred_instr[`I_SIZE-1:`I_SIZE-`RW];

// disable predicting after sys, irt, srs0 instructions (always ends with flush)
wire disable_prediction = (branch_pred_instr[6:0] == 7'h12) || (branch_pred_instr[6:0] == 7'h1e) || (branch_pred_instr[6:0] == 7'h11 && ~(|branch_pred_imm));
reg branch_pred_res;

// BRANCH PREDICTION / PC DECODE
always @(*) begin
    if (pc_reset_override) begin
        branch_pred_res = 1'b0;
    end else if (branch_pred_instr[6:0] == 7'h0e) begin
        if (branch_pred_instr[10:7] == 4'h0) begin
            // unconditional jump
            branch_pred_res = 1'b1;
        end else begin
            // try to predict jump
            if (prev_request_pc > branch_pred_imm) begin
                // back jump (taken)
                branch_pred_res = 1'b1;
            end else begin
                // forward jump (not taken)
                branch_pred_res = 1'b0;
            end
        end
    end else if (branch_pred_instr[6:0] == 7'h0f) begin
        branch_pred_res = 1'b1;
    end else begin
        branch_pred_res = 1'b0;
    end
end

reg [`I_SIZE-1:0] out_buffer_data_instr;
reg out_buffer_data_pred;
reg out_buffer_valid;

wire out_buff_write = mem_ack & ~i_next_ready & ~(i_flush | flush_event_invalidate);
wire out_buff_read = out_buffer_valid & i_next_ready;
always @(posedge i_clk) begin
    if (i_rst) begin
        out_buffer_valid <= 1'b0;
    end else if (i_flush) begin
        out_buffer_valid <= 1'b0;
    end else if (out_buff_write) begin
        out_buffer_data_instr <= mem_data;
        out_buffer_data_pred <= current_req_branch_pred;
        out_buffer_valid <= 1'b1;
    end else if (out_buffer_valid & i_next_ready) begin
        out_buffer_valid <= 1'b0;
    end
end

assign dbg_out = instr_wait;

endmodule

// Memory and Writeback stage


module memwb (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [`RW-1:0] i_data,
    input [`RW-1:0] i_addr,
    input [7:0] i_addr_high,
    input [`REGNO-1:0] i_reg_ie,
    input i_mem_access,
    input i_mem_long,
    input i_mem_we,
    input i_mem_width,

    output [`REGNO-1:0] o_reg_ie,
    output [`RW-1:0] o_reg_data,

    input i_submit,
    output o_ready,

    output reg o_mem_req,
    output [`RW-1:0] o_mem_data,
    output [`RW-1:0] o_mem_addr,
    output [7:0] o_mem_addr_high,
    output o_mem_long,
    output o_mem_we,
    input i_mem_ack,
    input [`RW-1:0] i_mem_data,
    output [`ADDR_BYTES-1:0] o_mem_sel,
    input o_mem_exception,

    output dbg_out
);

wire [`RW-1:0] mem_result = (i_mem_width ? (i_addr[0] ? (i_mem_data>>`RW'h8) : (i_mem_data&`RW'hff)) : i_mem_data);
assign o_reg_data = (i_mem_access ? mem_result : i_data);

assign o_mem_data = ((i_mem_width & i_addr[0]) ? (i_data<<`RW'h8) : i_data);
assign o_mem_addr = {(i_addr_high[0]&i_mem_long), 15'b0} | (i_addr>>`RW'b1); // LSB is used for byte selection in STDMEM
assign o_mem_we = i_mem_we;
assign o_mem_addr_high = (i_addr_high>>`RW'b1);
assign o_mem_long = i_mem_long;

assign o_mem_sel = (i_mem_width ? (`ADDR_BYTES'b1 << i_addr[0]) : `ADDR_BYTES'b11);

wire reg_ie = ((i_mem_access & i_mem_ack) | (~i_mem_access & i_submit)) & (|i_reg_ie);
assign o_reg_ie = (reg_ie ? i_reg_ie : `REGNO'b0);

assign o_ready = (~o_mem_req | (o_mem_req & i_mem_ack)) & ~(i_submit & i_mem_access);

always @(posedge i_clk) begin
    if (i_rst) begin
        o_mem_req <= 1'b0;
    end else if (i_submit) begin
        o_mem_req <= i_mem_access;
    end else if (i_mem_ack | o_mem_exception) begin
        o_mem_req <= 1'b0;
    end
end

assign dbg_out = o_ready;
    
endmodule
/*
 * Everything connecting cores, caches together up to main wishbone bus
 */



`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module interconnect_inner (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input core_clock,
    input core_reset,

    // INTERCONNECT WB OUTPUT
    output inner_wb_cyc, inner_wb_stb,
    output inner_wb_we,
    output [`WB_ADDR_W-1:0] inner_wb_adr,
    output [`WB_DATA_W-1:0] inner_wb_o_dat,
    input [`WB_DATA_W-1:0] inner_wb_i_dat,
    input inner_wb_ack, inner_wb_err,
    output [`WB_SEL_BITS-1:0] inner_wb_sel,
    output inner_wb_4_burst, inner_wb_8_burst,
    input inner_ext_irq,
    input inner_embed_mode,
    input inner_disable,

    // CORE 0
    output c0_clk,
    output c0_rst,
    output c0_disable,
    input [`RW-1:0] c0_o_req_addr,
    input c0_o_req_active, c0_o_req_ppl_submit,
    output [`I_SIZE-1:0] c0_i_req_data,
    output c0_i_req_data_valid,
    input [`RW-1:0] c0_dbg_r0, c0_dbg_pc,
    input [`RW-1:0] c0_o_mem_addr, c0_o_mem_data,
    output [`RW-1:0] c0_i_mem_data,
    input c0_o_mem_req, c0_o_mem_we,
    output c0_i_mem_ack,
    input [`ADDR_BYTES-1:0] c0_o_mem_sel,
    output c0_i_irq,
    input c0_o_c_instr_page, c0_o_c_data_page,
    input [`RW-1:0] c0_sr_bus_addr, c0_sr_bus_data_o,
    input c0_sr_bus_we,
    input c0_o_icache_flush,
    output c0_i_mem_exception,
    output c0_i_mc_core_int,
    input c0_o_c_instr_long,
    input [7:0] c0_o_instr_long_addr,
    input c0_o_mem_long_mode,
    input [7:0] c0_o_mem_high_addr,
    output [`RW-1:0] c0_i_core_int_sreg,
    input [35:0] c0_dbg_out,
    output [3:0] c0_dbg_in,

    // CORE 1
    output c1_clk,
    output c1_rst,
    output c1_disable,
    input [`RW-1:0] c1_o_req_addr,
    input c1_o_req_active, c1_o_req_ppl_submit,
    output [`I_SIZE-1:0] c1_i_req_data,
    output c1_i_req_data_valid,
    input [`RW-1:0] c1_dbg_r0, c1_dbg_pc,
    input [`RW-1:0] c1_o_mem_addr, c1_o_mem_data,
    output [`RW-1:0] c1_i_mem_data,
    input c1_o_mem_req, c1_o_mem_we,
    output c1_i_mem_ack,
    input [`ADDR_BYTES-1:0] c1_o_mem_sel,
    output c1_i_irq,
    input c1_o_c_instr_page, c1_o_c_data_page,
    input [`RW-1:0] c1_sr_bus_addr, c1_sr_bus_data_o,
    input c1_sr_bus_we,
    input c1_o_icache_flush,
    output c1_i_mem_exception,
    output c1_i_mc_core_int,
    input c1_o_c_instr_long,
    input [7:0] c1_o_instr_long_addr,
    input c1_o_mem_long_mode,
    input [7:0] c1_o_mem_high_addr,
    output [`RW-1:0] c1_i_core_int_sreg,
    input [35:0] c1_dbg_out,
    output [3:0] c1_dbg_in,

    // ICACHE 0
    output ic0_clk,
    output ic0_rst,
    output ic0_mem_req,
    input ic0_mem_ack,
    output [`RW-1:0] ic0_mem_addr,
    input [`I_SIZE-1:0] ic0_mem_data,
    output ic0_mem_ppl_submit,
    output ic0_mem_cache_flush,
    input ic0_wb_cyc,
    input ic0_wb_stb,
    output [`RW-1:0] ic0_wb_i_dat,
    input [`RW-1:0]  ic0_wb_adr,
    input ic0_wb_we,
    output ic0_wb_ack,
    input [1:0] ic0_wb_sel,
    output ic0_wb_err,

    // ICACHE 1
    output ic1_clk,
    output ic1_rst,
    output ic1_mem_req,
    input ic1_mem_ack,
    output [`RW-1:0] ic1_mem_addr,
    input [`I_SIZE-1:0] ic1_mem_data,
    output ic1_mem_ppl_submit,
    output ic1_mem_cache_flush,
    input ic1_wb_cyc,
    input ic1_wb_stb,
    output [`RW-1:0] ic1_wb_i_dat,
    input [`RW-1:0]  ic1_wb_adr,
    input ic1_wb_we,
    output ic1_wb_ack,
    input [1:0] ic1_wb_sel,
    output ic1_wb_err,

    // DCACHE
    output dcache_clk,
    output dcache_rst,
    output dcache_mem_req,
    output dcache_mem_we,
    input dcache_mem_ack,
    output [`WB_ADDR_W-1:0] dcache_mem_addr,
    output [`RW-1:0] dcache_mem_i_data,
    input [`RW-1:0] dcache_mem_o_data,
    output [1:0] dcache_mem_sel,
    output dcache_mem_cache_enable,
    input dcache_mem_exception,
    input dcache_wb_cyc,
    input dcache_wb_stb,
    output [`RW-1:0] dcache_wb_i_dat,
    input [`RW-1:0] dcache_wb_o_dat,
    input [`WB_ADDR_W-1:0]  dcache_wb_adr,
    input dcache_wb_we,
    input [1:0] dcache_wb_sel,
    input dcache_wb_4_burst,
    output dcache_wb_ack,
    output dcache_wb_err

);

////////////////////////////////////////////////////////
// INNER SECTION CORE > MMU > CACHES > CACHE_WB_MUXES //
////////////////////////////////////////////////////////

assign c0_clk = core_clock;
assign c0_rst = core_reset;
assign c1_clk = core_clock;
assign c1_rst = core_reset;

// CORE <-> ICACHE <-> IMMU
assign ic0_clk = core_clock;
assign ic0_rst = core_reset;
assign ic0_mem_req = c0_o_req_active;
assign ic0_mem_addr = c0_o_req_addr;
assign c0_i_req_data = ic0_mem_data;
assign c0_i_req_data_valid = ic0_mem_ack;
assign ic0_mem_ppl_submit = c0_o_req_ppl_submit;
assign ic0_mem_cache_flush = c0_o_icache_flush;

wire [`WB_ADDR_W-1:0] ic0_wb_adr_paged;
immu immu_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset), 
    .i_addr(ic0_wb_adr), .o_addr(ic0_wb_adr_paged),
    .i_sr_addr(c0_sr_bus_addr), .i_sr_data(c0_sr_bus_data_o), .i_sr_we(c0_sr_bus_we),
    .c_pag_en(c0_o_c_instr_page & ~inner_embed_mode),
    .c_long_mode(c0_o_c_instr_long),
    .i_long_high_addr(c0_o_instr_long_addr)
);

assign ic1_clk = core_clock;
assign ic1_rst = core_reset;
assign ic1_mem_req = c1_o_req_active;
assign ic1_mem_addr = c1_o_req_addr;
assign c1_i_req_data = ic1_mem_data;
assign c1_i_req_data_valid = ic1_mem_ack;
assign ic1_mem_ppl_submit = c1_o_req_ppl_submit;
assign ic1_mem_cache_flush = c1_o_icache_flush;

wire [`WB_ADDR_W-1:0] ic1_wb_adr_paged;
immu immu_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset), 
    .i_addr(ic1_wb_adr), .o_addr(ic1_wb_adr_paged),
    .i_sr_addr(c1_sr_bus_addr), .i_sr_data(c1_sr_bus_data_o), .i_sr_we(c1_sr_bus_we),
    .c_pag_en(c1_o_c_instr_page & ~inner_embed_mode),
    .c_long_mode(c1_o_c_instr_long),
    .i_long_high_addr(c1_o_instr_long_addr)
);



// ICACHE{0,1} ARBITER
wire icache_wb_cyc, icache_wb_stb;
wire icache_wb_we;
wire icache_wb_ack, icache_wb_err;
wire [`WB_ADDR_W-1:0]  icache_wb_adr;
wire [`WB_DATA_W-1:0] icache_wb_o_dat, icache_wb_i_dat;
wire [`WB_SEL_BITS-1:0] icache_wb_sel;
wire icache_wb_4_burst, icache_wb_8_burst;

assign ic0_wb_i_dat = icache_wb_i_dat;
assign ic1_wb_i_dat = icache_wb_i_dat; 

wishbone_arbiter icache_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),

    .o_wb_cyc(icache_wb_cyc),
    .owb_stb(icache_wb_stb),
    .owb_we(icache_wb_we),
    .owb_ack(icache_wb_ack),
    .owb_adr(icache_wb_adr),
    .owb_sel(icache_wb_sel),
    .owb_err(icache_wb_err),
    .owb_o_dat(icache_wb_o_dat),
    .owb_4_burst(icache_wb_4_burst),
    .owb_8_burst(icache_wb_8_burst),

    .i_wb0_cyc(ic0_wb_cyc),
    .wb0_stb(ic0_wb_stb),
    .wb0_we(ic0_wb_we),
    .wb0_ack(ic0_wb_ack),
    .wb0_adr(ic0_wb_adr_paged),
    .wb0_sel(ic0_wb_sel),
    .wb0_err(ic0_wb_err),
    .wb0_o_dat(`RW'b0),
    .wb0_4_burst(1'b0),
    .wb0_8_burst(1'b1),

    .i_wb1_cyc(ic1_wb_cyc),
    .wb1_stb(ic1_wb_stb),
    .wb1_we(ic1_wb_we),
    .wb1_ack(ic1_wb_ack),
    .wb1_adr(ic1_wb_adr_paged),
    .wb1_sel(ic1_wb_sel),
    .wb1_err(ic1_wb_err),
    .wb1_o_dat(`RW'b0),
    .wb1_4_burst(1'b0),
    .wb1_8_burst(1'b1)
);

// CORE -> DMMU -> DCACHE BUS ARBITER

wire [`WB_ADDR_W-1:0] c0_mem_addr_paged;
wire c0_mmu_data_cacheable;
dmmu dmmu0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),
    .i_addr(c0_o_mem_addr), .o_addr(c0_mem_addr_paged),
    .i_sr_addr(c0_sr_bus_addr), .i_sr_data(c0_sr_bus_data_o), .i_sr_we(c0_sr_bus_we),
    .c_pag_en(c0_o_c_data_page), .o_cacheable(c0_mmu_data_cacheable), .c_long(c0_o_mem_long_mode), .i_high_addr(c0_o_mem_high_addr)
);

wire [`WB_ADDR_W-1:0] c1_mem_addr_paged;
wire c1_mmu_data_cacheable;
dmmu dmmu1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),
    .i_addr(c1_o_mem_addr), .o_addr(c1_mem_addr_paged),
    .i_sr_addr(c1_sr_bus_addr), .i_sr_data(c1_sr_bus_data_o), .i_sr_we(c1_sr_bus_we),
    .c_pag_en(c1_o_c_data_page), .o_cacheable(c1_mmu_data_cacheable), .c_long(c1_o_mem_long_mode), .i_high_addr(c1_o_mem_high_addr)
);

mem_dcache_arb mem_dcache_arb (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock),
    .i_rst(core_reset),

    .mem_req(dcache_mem_req), .mem_we(dcache_mem_we), .mem_ack(dcache_mem_ack), .mem_addr(dcache_mem_addr), .mem_i_data(dcache_mem_o_data), .mem_o_data(dcache_mem_i_data), .mem_sel(dcache_mem_sel), .mem_cache_enable(dcache_mem_cache_enable), .mem_exception(dcache_mem_exception),
    .mem0_req(c0_o_mem_req), .mem0_we(c0_o_mem_we), .mem0_ack(c0_i_mem_ack), .mem0_addr(c0_mem_addr_paged), .mem0_i_data(c0_i_mem_data), .mem0_o_data(c0_o_mem_data), .mem0_sel(c0_o_mem_sel), .mem0_cache_enable(c0_mmu_data_cacheable), .mem0_exception(c0_i_mem_exception),
    .mem1_req(c1_o_mem_req), .mem1_we(c1_o_mem_we), .mem1_ack(c1_i_mem_ack), .mem1_addr(c1_mem_addr_paged), .mem1_i_data(c1_i_mem_data), .mem1_o_data(c1_o_mem_data), .mem1_sel(c1_o_mem_sel), .mem1_cache_enable(c1_mmu_data_cacheable), .mem1_exception(c1_i_mem_exception)
);

assign dcache_clk = core_clock;
assign dcache_rst = core_reset; 

// {DCACHE, ICACHE} WB ARBITER

assign icache_wb_i_dat = inner_wb_i_dat;
assign dcache_wb_i_dat = inner_wb_i_dat;

wishbone_arbiter inner_wb_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),

    .o_wb_cyc(inner_wb_cyc),
    .owb_stb(inner_wb_stb),
    .owb_we(inner_wb_we),
    .owb_ack(inner_wb_ack),
    .owb_adr(inner_wb_adr),
    .owb_sel(inner_wb_sel),
    .owb_err(inner_wb_err),
    .owb_o_dat(inner_wb_o_dat),
    .owb_4_burst(inner_wb_4_burst),
    .owb_8_burst(inner_wb_8_burst),

    .i_wb0_cyc(icache_wb_cyc),
    .wb0_stb(icache_wb_stb),
    .wb0_we(icache_wb_we),
    .wb0_ack(icache_wb_ack),
    .wb0_adr(icache_wb_adr),
    .wb0_sel(icache_wb_sel),
    .wb0_err(icache_wb_err),
    .wb0_o_dat(icache_wb_o_dat),
    .wb0_4_burst(icache_wb_4_burst),
    .wb0_8_burst(icache_wb_8_burst),

    .i_wb1_cyc(dcache_wb_cyc),
    .wb1_stb(dcache_wb_stb),
    .wb1_we(dcache_wb_we),
    .wb1_ack(dcache_wb_ack),
    .wb1_adr(dcache_wb_adr),
    .wb1_sel(dcache_wb_sel),
    .wb1_err(dcache_wb_err),
    .wb1_o_dat(dcache_wb_o_dat),
    .wb1_4_burst(dcache_wb_4_burst),
    .wb1_8_burst(1'b0)
);

// INTERCORE CONTROL

assign c0_i_irq = inner_ext_irq;
assign c1_i_irq = 1'b0;
wire c0_sc_disable, c1_sc_disable;
assign c0_disable = c0_sc_disable | inner_disable;
assign c1_disable = c1_sc_disable | inner_disable;

intercore_sregs icore_sregs (
    .i_clk(core_clock),
    .i_rst(core_reset),

    .c0_sr_bus_addr(c0_sr_bus_addr),
    .c0_sr_bus_data_o(c0_sr_bus_data_o),
    .c0_sr_bus_data_i(c0_i_core_int_sreg),
    .c0_sr_bus_we(c0_sr_bus_we),

    .c1_sr_bus_addr(c1_sr_bus_addr),
    .c1_sr_bus_data_o(c1_sr_bus_data_o),
    .c1_sr_bus_data_i(c1_i_core_int_sreg),
    .c1_sr_bus_we(c1_sr_bus_we),

    .c0_disable(c0_sc_disable),
    .c0_core_int(c0_i_mc_core_int),
    .c1_disable(c1_sc_disable),
    .c1_core_int(c1_i_mc_core_int)
);
    
endmodule
module pc #(parameter INT_VEC = `RW'b1) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    output reg [`RW-1:0] o_pc,
    input [`RW-1:0] i_bus,

    input i_c_pc_inc,
    input i_c_pc_ie,
    input i_c_pc_irq,

    output o_pc_ovf
);

always @(posedge i_clk) begin
    if (i_rst) begin
        o_pc <= `RW'b0;
    end else if (i_c_pc_irq) begin
        o_pc <= INT_VEC;
    end else if (i_c_pc_ie) begin
        o_pc <= i_bus;
    end else if (i_c_pc_inc) begin
        o_pc <= o_pc + `RW'b1;
    end
end

assign o_pc_ovf = o_pc[14] & i_c_pc_inc & ~i_c_pc_irq & ~i_c_pc_ie; // due to address shifiting, pc overflows at 14th bit 

endmodule

`define MPRJ_IO_PADS 38

`define WB_DATA_W 16 
`define WB_SEL_BITS 2

module interconnect_outer (
    // IOs
    input  [`MPRJ_IO_PADS-1:0] m_io_in,
    output [`MPRJ_IO_PADS-1:0] m_io_out,
    output [`MPRJ_IO_PADS-1:0] m_io_oeb,

    // Wishbone from managment core
    input mgt_wb_clk_i,
    input mgt_wb_rst_i,
    input mgt_wb_stb_i,
    input mgt_wb_cyc_i,
    input mgt_wb_we_i,
    input [3:0] mgt_wb_sel_i,
    input [31:0] mgt_wb_dat_i,
    input [31:0] mgt_wb_adr_i,
    output mgt_wb_ack_o,
    output [31:0] mgt_wb_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IRQ
    output [2:0] irq,


    // Lower Interconnect
    output inner_clock, inner_reset,
    input inner_wb_cyc, inner_wb_stb,
    input inner_wb_we,
    input [`WB_ADDR_W-1:0] inner_wb_adr,
    input [`WB_DATA_W-1:0] inner_wb_o_dat,
    output [`WB_DATA_W-1:0] inner_wb_i_dat,
    output inner_wb_ack, inner_wb_err,
    input [`WB_SEL_BITS-1:0] inner_wb_sel,
    input inner_wb_4_burst, inner_wb_8_burst,
    output inner_ext_irq,
    output inner_embed_mode,
    output inner_disable,

    // Internal ram
    output iram_clk,
    output [8:0] iram_addr,
    output [`RW-1:0] iram_i_data,
    input  [`RW-1:0] iram_o_data,
    output iram_we
);

wire soc_clock = mgt_wb_clk_i;
wire soc_reset = mgt_wb_rst_i;

wire core_clock = soc_clock;
wire soft_reset = (~la_oenb[0]) & la_data_in[0]; // in future & embed mode
wire core_reset = soc_reset | soft_reset;

assign inner_clock = core_clock;
assign inner_reset = core_reset;

// MASTER CW BUS

wire [`RW-1:0] cw_io_i; // S>M
wire [`RW-1:0] cw_io_o; // M>S
wire cw_req;
wire cw_dir;
wire cw_ack;
wire cw_err;
wire cw_clk; // ouputs for bridge
wire cw_rst;

// PIN ASSIGNMENTS
// pins [4:0] are reserved to mgmt during boot

assign m_io_out[8] = cw_req;
assign m_io_out[9] = cw_dir;
assign m_io_out[25:10] = cw_io_o;
assign cw_io_i = m_io_in[25:10];
assign cw_ack = m_io_in[26];
assign cw_err = m_io_in[27];
assign m_io_out[28] = cw_clk;
assign m_io_out[29] = cw_rst; // reset out

wire ext_irq = m_io_in[30];

wire cs_split_clock = m_io_in[31];
wire core_disable = m_io_in[32];
wire embed_mode = m_io_in[33];

assign spi_clk = m_io_in[34];
assign spi_mosi = m_io_in[35];
assign m_io_out[36] = spi_miso;

assign gpio_in = m_io_in[7:0];
assign m_io_out[7:0] = gpio_out;
assign m_io_out[37] = 1'b1;

assign m_io_oeb[7:0] = gpio_dir;
assign m_io_oeb[9:8] = 2'b0;
assign m_io_oeb[25:10] = {16{cw_dir}};
assign m_io_oeb[27:26] = 2'b11;
assign m_io_oeb[29:28] = 2'b00;
assign m_io_oeb[33:30] = 4'b1111;
assign m_io_oeb[35:34] = 2'b11;
assign m_io_oeb[36] = 1'b0;
assign m_io_oeb[37] = 1'b0;

assign inner_embed_mode = embed_mode;
assign inner_disable = core_disable;

// CLOCKING
`define CLK_DIV_ADDR `WB_ADDR_W'h001001
wire clk_div_write = (m_wb_cyc & m_wb_ack & m_wb_we & wb_tsel_clk);
clk_div clk_div (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(soc_clock),
    .i_rst(core_reset),
    .o_clk(cw_clk),
    .div(m_wb_o_dat[3:0]),
    .div_we(clk_div_write),
    .clock_sel(cs_split_clock)
);

// WISHBONE CLOCK DOMAIN CROSSING

wire cwclk_wb_cyc;
wire cwclk_wb_stb;
wire [`WB_ADDR_W-1:0] cwclk_wb_adr;
wire cwclk_wb_we;
wire [`WB_SEL_BITS-1:0] cwclk_wb_sel;
wire [`WB_DATA_W-1:0] cwclk_wb_o_dat;
wire[`WB_DATA_W-1:0] cwclk_wb_i_dat = cw_mux_wb_i_dat;
wire cwclk_wb_ack = cw_mux_wb_ack;
wire cwclk_wb_err = cw_mux_wb_err;
wire cwclk_wb_8_burst, cwclk_wb_4_burst;

// CROSS_CLK OUPUT SIGNALS FROM M_WB SIDE FOR MUXING
wire [`WB_DATA_W-1:0] m_wb_i_dat_cross;
wire m_wb_ack_cross, m_wb_err_cross;

wb_cross_clk_outer wb_cross_clk (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .clk_m(core_clock),
    .clk_s(cw_clk),
    .m_rst(core_reset),
    .s_rst(cw_rst),

    .m_wb_cyc(m_wb_cyc),
    .m_wb_stb(m_wb_stb & wb_tsel_cw),
    .m_wb_o_dat(m_wb_o_dat),
    .m_wb_i_dat(m_wb_i_dat_cross),
    .m_wb_adr(m_wb_adr),
    .m_wb_we(m_wb_we),
    .m_wb_ack(m_wb_ack_cross),
    .m_wb_err(m_wb_err_cross),
    .m_wb_sel(m_wb_sel),
    .m_wb_4_burst(m_wb_4_burst),
    .m_wb_8_burst(m_wb_8_burst),

    .s_wb_cyc(cwclk_wb_cyc),
    .s_wb_stb(cwclk_wb_stb),
    .s_wb_o_dat(cwclk_wb_o_dat),
    .s_wb_i_dat(cwclk_wb_i_dat),
    .s_wb_adr(cwclk_wb_adr),
    .s_wb_we(cwclk_wb_we),
    .s_wb_ack(cwclk_wb_ack),
    .s_wb_err(cwclk_wb_err),
    .s_wb_sel(cwclk_wb_sel),
    .s_wb_4_burst(cwclk_wb_4_burst),
    .s_wb_8_burst(cwclk_wb_8_burst)
);

// clock split bypass

wire cw_mux_wb_cyc;
wire cw_mux_wb_stb;
wire [`WB_ADDR_W-1:0] cw_mux_wb_adr;
wire cw_mux_wb_we;
wire [`WB_SEL_BITS-1:0] cw_mux_wb_sel;
wire [`WB_DATA_W-1:0] cw_mux_wb_o_dat;
wire[`WB_DATA_W-1:0] cw_mux_wb_i_dat;
wire cw_mux_wb_ack;
wire cw_mux_wb_err;
wire cw_mux_wb_8_burst, cw_mux_wb_4_burst;
wire cw_out_wb_ack;
wire cw_out_wb_err;
wire [`RW-1:0] cw_out_wb_i_dat;

assign cw_mux_wb_cyc = (cs_split_clock ? cwclk_wb_cyc : m_wb_cyc);
assign cw_mux_wb_stb = (cs_split_clock ? cwclk_wb_stb : m_wb_stb);
assign cw_mux_wb_o_dat = (cs_split_clock ? cwclk_wb_o_dat : m_wb_o_dat);
assign cw_mux_wb_adr = (cs_split_clock ? cwclk_wb_adr : m_wb_adr);
assign cw_mux_wb_we = (cs_split_clock ? cwclk_wb_we : m_wb_we);
assign cw_mux_wb_sel = (cs_split_clock ? cwclk_wb_sel : m_wb_sel);
assign cw_mux_wb_8_burst = (cs_split_clock ? cwclk_wb_8_burst : m_wb_8_burst);
assign cw_mux_wb_4_burst = (cs_split_clock ? cwclk_wb_4_burst : m_wb_4_burst);
assign cw_out_wb_ack = (cs_split_clock ? m_wb_ack_cross : cw_mux_wb_ack);
assign cw_out_wb_err = (cs_split_clock ? m_wb_err_cross  : cw_mux_wb_err);
assign cw_out_wb_i_dat = (cs_split_clock ? m_wb_i_dat_cross  : cw_mux_wb_i_dat);


// CW BUS CONVERTER

wb_compressor wb_compressor (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(cw_clk),
    .i_rst(cw_rst),

    .cw_io_i(cw_io_i),
    .cw_io_o(cw_io_o),
    .cw_req(cw_req),
    .cw_dir(cw_dir),
    .cw_ack(cw_ack),
    .cw_err(cw_err),

    .wb_cyc(cw_mux_wb_cyc),
    .wb_stb(cw_mux_wb_stb & wb_tsel_cw),
    .wb_o_dat(cw_mux_wb_o_dat),
    .wb_i_dat(cw_mux_wb_i_dat),
    .wb_adr(cw_mux_wb_adr),
    .wb_we(cw_mux_wb_we),
    .wb_ack(cw_mux_wb_ack),
    .wb_err(cw_mux_wb_err),
    .wb_sel(cw_mux_wb_sel),
    .wb_4_burst(cw_mux_wb_4_burst),
    .wb_8_burst(cw_mux_wb_8_burst)
);

// EXT SPI WISHBONE MASTER
wire spi_wb_cyc;
wire spi_wb_stb;
wire [`WB_ADDR_W-1:0] spi_wb_adr;
wire spi_wb_we;
wire [`WB_SEL_BITS-1:0] spi_wb_sel;
wire [`WB_DATA_W-1:0] spi_wb_o_dat;
wire [`WB_DATA_W-1:0] spi_wb_i_dat;
wire spi_wb_ack;
wire spi_wb_err;
wire spi_wb_8_burst=1'b0, spi_wb_4_burst=1'b0;

wire spi_clk;
wire spi_mosi;
wire spi_miso;

sspi sspi (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock),
    .i_rst(core_reset),

    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),

    .wb_cyc(spi_wb_cyc),
    .wb_stb(spi_wb_stb),
    .wb_o_dat(spi_wb_o_dat),
    .wb_i_dat(spi_wb_i_dat),
    .wb_adr(spi_wb_adr),
    .wb_we(spi_wb_we),
    .wb_ack(spi_wb_ack),
    .wb_err(spi_wb_err),
    .wb_sel(spi_wb_sel)
);

wire gpio_wb_stb;
wire [`RW-1:0] gpio_wb_o_dat;
wire gpio_wb_ack;
wire [7:0] gpio_in, gpio_out, gpio_dir;

gpio #(.N(8)) gpio (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock),
    .i_rst(core_reset),

    .gpio_in(gpio_in),
    .gpio_out(gpio_out),
    .gpio_dir(gpio_dir),

    .wb_cyc(m_wb_cyc),
    .wb_stb(m_wb_stb),
    .wb_i_dat(m_wb_o_dat),
    .wb_o_dat(gpio_wb_o_dat),
    .wb_adr(m_wb_adr),
    .wb_we(m_wb_we),
    .wb_ack(gpio_wb_ack)
);

// WISHBONE ARBITTER INNER BUS, EXT SPI BUS
wire m_wb_cyc, m_wb_stb;
wire m_wb_we;
wire [`WB_ADDR_W-1:0]  m_wb_adr;
wire [`WB_DATA_W-1:0] m_wb_o_dat;
wire [`WB_SEL_BITS-1:0] m_wb_sel;
wire m_wb_4_burst, m_wb_8_burst;
reg m_wb_ack, m_wb_err;
reg [`WB_DATA_W-1:0] m_wb_i_dat;

assign spi_wb_i_dat = m_wb_i_dat;
assign inner_wb_i_dat = m_wb_i_dat; 

wishbone_arbiter m_arbiter (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(core_clock), .i_rst(core_reset),

    .o_wb_cyc(m_wb_cyc),
    .owb_stb(m_wb_stb),
    .owb_we(m_wb_we),
    .owb_ack(m_wb_ack),
    .owb_adr(m_wb_adr),
    .owb_sel(m_wb_sel),
    .owb_err(m_wb_err),
    .owb_o_dat(m_wb_o_dat),
    .owb_4_burst(m_wb_4_burst),
    .owb_8_burst(m_wb_8_burst),

    .i_wb0_cyc(spi_wb_cyc),
    .wb0_stb(spi_wb_stb),
    .wb0_we(spi_wb_we),
    .wb0_ack(spi_wb_ack),
    .wb0_adr(spi_wb_adr),
    .wb0_sel(spi_wb_sel),
    .wb0_err(spi_wb_err),
    .wb0_o_dat(spi_wb_o_dat),
    .wb0_4_burst(spi_wb_4_burst),
    .wb0_8_burst(spi_wb_8_burst),

    .i_wb1_cyc(inner_wb_cyc),
    .wb1_stb(inner_wb_stb),
    .wb1_we(inner_wb_we),
    .wb1_ack(inner_wb_ack),
    .wb1_adr(inner_wb_adr),
    .wb1_sel(inner_wb_sel),
    .wb1_err(inner_wb_err),
    .wb1_o_dat(inner_wb_o_dat),
    .wb1_4_burst(inner_wb_4_burst),
    .wb1_8_burst(inner_wb_8_burst)
);

// INTERNAL RAM (for easier tesing in embed mode)
wire [`RW-1:0] iram_wb_i_dat;

assign iram_clk = core_clock;
assign iram_addr = m_wb_adr[8:0];
assign iram_i_data = m_wb_o_dat;
assign iram_wb_i_dat = iram_o_data;
assign iram_we = m_wb_cyc & m_wb_stb & m_wb_we & wb_tsel_iram;

reg iram_wb_ack;
always @(posedge core_clock) begin
    if (core_reset) iram_wb_ack <= 1'b0;
    else iram_wb_ack <= m_wb_cyc & m_wb_stb & wb_tsel_iram & ~m_wb_ack; // 1 cyc delay
end

// WISHBONE TARGET SELECT
`define NE_INTMEM_BEGIN 24'h7ffe00
`define E_PROG_START    24'h800000
`define E_MEM_START     24'h100000
`define INTMEM_SIZE     24'h200
`define GPIO_START      24'h001010
`define GPIO_END        24'h001012

wire wb_tsel_cw = (~embed_mode && (m_wb_adr != `CLK_DIV_ADDR) && (~((m_wb_adr >= `GPIO_START) && (m_wb_adr <= `GPIO_END))) && ((m_wb_adr < `NE_INTMEM_BEGIN) || (m_wb_adr >= `NE_INTMEM_BEGIN+`INTMEM_SIZE)));
wire wb_tsel_iram = (~embed_mode && (m_wb_adr >= `NE_INTMEM_BEGIN) && (m_wb_adr < `NE_INTMEM_BEGIN+`INTMEM_SIZE))
                    || (embed_mode && (((m_wb_adr >= `E_PROG_START) && (m_wb_adr < `E_PROG_START+`INTMEM_SIZE)) 
                                    || (m_wb_adr >= `E_MEM_START) && (m_wb_adr < `E_MEM_START+`INTMEM_SIZE)));
wire wb_tsel_clk = (m_wb_adr == `CLK_DIV_ADDR);
wire wb_tsel_gpio = (m_wb_adr >= `GPIO_START) && (m_wb_adr <= `GPIO_END);

always @(*) begin
    if (wb_tsel_cw) begin
        m_wb_ack = cw_out_wb_ack;
        m_wb_err = cw_out_wb_err;
        m_wb_i_dat = cw_out_wb_i_dat;
    end else if (wb_tsel_iram) begin
        m_wb_ack = iram_wb_ack;
        m_wb_err = 1'b0;
        m_wb_i_dat = iram_wb_i_dat;
    end else if (wb_tsel_clk) begin
        m_wb_ack = 1'b1;
        m_wb_err = 1'b0;
        m_wb_i_dat = `RW'b0;
    end else if (wb_tsel_gpio) begin
        m_wb_ack = gpio_wb_ack;
        m_wb_err = 1'b0;
        m_wb_i_dat = gpio_wb_o_dat;
    end else begin
        m_wb_ack = 1'b0;
        m_wb_err = 1'b1;
        m_wb_i_dat = `RW'b0;
    end
end

// SYNCHRONIZERS

reset_sync rst_cw_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(cw_clk),
    .i_rst(soc_reset),
    .o_rst(cw_rst)
);

assign inner_ext_irq = irq_s_ff[1];
reg [1:0] irq_s_ff;
always @(posedge core_clock) begin
    irq_s_ff[0] <= ext_irq;
    irq_s_ff[1] <= irq_s_ff[0]; 
end


endmodule

module rf (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input i_gie,
    input [`REGNO-1:0] i_ie,
    input [`REGNO_LOG-1:0] i_lout_sel,
    input [`REGNO_LOG-1:0] i_rout_sel,
    
    input [`RW-1:0] i_d,
    output [`RW-1:0] o_lout,
    output [`RW-1:0] o_rout,
    output [`RW-1:0] o_l_high_out,

    output [`RW-1:0] dbg_r0,
    input [`REGNO_LOG-1:0] dbg_sel,
    output [`RW-1:0] dbg_reg
);

wire [`RW-1:0] reg_outputs [`REGNO-1:0];

assign o_lout = reg_outputs[i_lout_sel];
assign o_rout = reg_outputs[i_rout_sel];
assign o_l_high_out = reg_outputs[(i_lout_sel&3'b110) | 3'b1];

assign dbg_r0 = reg_outputs[0];
assign dbg_reg = reg_outputs[dbg_sel];

genvar i;
generate
    for (i=0; i<`REGNO; i=i+1) begin : rf_regs
        register rf_reg(
`ifdef USE_POWER_PINS
            .vccd1(vccd1), .vssd1(vssd1),
`endif
            .i_clk(i_clk),
            .i_rst(i_rst),
            .i_d(i_d),
            .i_ie(i_ie[i] & i_gie),
            .o_d(reg_outputs[i])
        );
    end
endgenerate

endmodule

module register #(parameter N = `RW, parameter RESET_VAL = 0) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk,
    input i_rst,

    input [N-1:0] i_d,
    output reg [N-1:0] o_d,
    input i_ie 
);

always @(posedge i_clk) begin
    if (i_rst)
        o_d <= RESET_VAL;
    else if (i_ie)
        o_d <= i_d;
end

    
endmodule

module dcache_ram (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input i_clk,
    input i_rst,

    input [5:0] i_addr,
    input [81:0] i_data,
    output reg [81:0] o_data,
    input i_we
);

reg [81:0] mem [63:0];
integer row;
always @(posedge i_clk) begin
    if (i_rst) begin

        for (row = 0; row < 64; row = row+1) begin
            mem[row][1:0] <= 2'b0;
        end
    end else begin
        if(i_we)
            mem[i_addr] <= i_data;
        o_data <= mem[i_addr];
    end
end

endmodule
`define MAX_DIV 16
`define MAX_DIV_LOG 4

module clk_div (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    input i_clk,
    input i_rst,

    output o_clk,

    input [`MAX_DIV_LOG-1:0] div,
    input div_we,
    input clock_sel
);

// ADD 130 buff

reg [`MAX_DIV-1:0] cnt;
reg [`MAX_DIV_LOG-1:0] curr_div, next_div_buff;
reg next_div_val;

reg res_clk;
assign o_clk = (clock_sel_r ? res_clk : i_clk);

always @(posedge i_clk) begin
    if (~cnt[curr_div]) begin
        cnt <= cnt + `MAX_DIV'b1;
    end else begin
        cnt <= `MAX_DIV'b0;
    end
end

always @(posedge i_clk) begin
    if (cnt[curr_div])
        res_clk <= ~res_clk;
end

always @(posedge i_clk) begin
    if(i_rst) begin
       // curr_div <= `MAX_DIV_LOG'b110;
                curr_div <= `MAX_DIV_LOG'b01;
        next_div_val <= 1'b0;
    end else begin
        if(cnt[curr_div] & next_div_val) begin
            curr_div <= next_div_buff;
            next_div_val <= 1'b0;
        end
        if (div_we) begin
            next_div_buff <= div;
            next_div_val <= 1'b1;
        end
    end
end

reg clock_sel_r;
always @(posedge i_clk)
    clock_sel_r <= clock_sel;

endmodule

module dcache (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input i_clk,
    input i_rst,

    input mem_req,
    input mem_we,
    output mem_ack,
    input [`WB_ADDR_W-1:0] mem_addr,
    input [`RW-1:0] mem_i_data,
    output reg [`RW-1:0] mem_o_data,
    input [1:0] mem_sel,
    input mem_cache_enable,
    output mem_exception,

    // output interface
    output reg wb_cyc,
    output reg wb_stb,
    input [`RW-1:0] wb_i_dat,
    output reg [`RW-1:0] wb_o_dat,
    output reg [`WB_ADDR_W-1:0] wb_adr,
    output reg wb_we,
    output [1:0] wb_sel,
    output wb_4_burst,
    input wb_ack,
    input wb_err

    // TODO: Multicore MSI cache protocol
);

// VIRTUALLY INDEXED, no need to flush on context switch, coherent with dma by cache protocols
`define TAG_SIZE 16
`define LINE_SIZE 64
// 16b tag + 64b line + 2b valid dirty
`define ENTRY_SIZE 82
`define CACHE_ASSOC 2
`define CACHE_ASSOC_W 1
`define CACHE_ENTR_N 64
`define CACHE_OFF_W 2

`define CACHE_IDX_WIDTH 6
`define CACHE_IDXES 64

`define VALID_BIT 0
`define DIRTY_BIT 1

`define SW 3
`define S_IDLE `SW'b0
`define S_CREAD `SW'b1
`define S_MISS_RD `SW'b10
`define S_MISS_WR `SW'b11
`define S_RQ_WR `SW'b100
`define S_RQ_WR_WAIT `SW'b101
`define S_NOCACHE `SW'b110
reg [`SW-1:0] state;

wire [`ENTRY_SIZE-1:0] cache_mem_in;
wire [`ENTRY_SIZE-1:0] cache_out [`CACHE_ASSOC-1:0];
reg [`CACHE_ASSOC-1:0] cache_we;
wire [`CACHE_ASSOC-1:0] cache_hit;

wire [`CACHE_IDX_WIDTH-1:0] cache_idx = mem_addr[7:2];
wire [`CACHE_OFF_W-1:0] cache_offset = mem_addr[1:0];
wire [`TAG_SIZE-1:0] cache_compare_tag = mem_addr[`WB_ADDR_W-1:8];

`ifndef USE_OC_RAM

dcache_ram mem_set_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(cache_idx), .i_data(cache_mem_in),
    .o_data(cache_out[0]), .i_we(cache_we[0]));
dcache_ram mem_set_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_rst(i_rst), .i_addr(cache_idx), .i_data(cache_mem_in),
    .o_data(cache_out[1]), .i_we(cache_we[1]));

`else

ocram_dcache mem_set_0 (
    .clock(i_clk),  .address(cache_idx), .data(cache_mem_in),
    .q(cache_out[0]), .wren(cache_we[0]));
ocram_dcache mem_set_1 (
    .clock(i_clk),  .address(cache_idx), .data(cache_mem_in),
    .q(cache_out[1]), .wren(cache_we[1]));

`endif

assign cache_hit[0] = (cache_out[0][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == cache_compare_tag) && cache_out[0][`VALID_BIT]; 
assign cache_hit[1] = (cache_out[1][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == cache_compare_tag) && cache_out[1][`VALID_BIT]; 

wire cache_ghit = (state == `S_CREAD) && (|cache_hit);
wire cache_gmiss = (state == `S_CREAD) && ~(|cache_hit);

wire mem_op_end = wb_cyc & wb_stb & (wb_ack | wb_err) & (&line_burst_cnt);
wire mem_fetch_end = (state == `S_MISS_RD) && mem_op_end;
wire write_to_cache = (state == `S_RQ_WR);

assign mem_ack = (state == `S_CREAD && ~mem_we && cache_ghit) | (mem_fetch_end && ~mem_we) | (state == `S_RQ_WR) | (state == `S_NOCACHE && wb_cyc && wb_stb && (wb_ack | wb_err));
// TODO: Emit exception on bus error and retry bus instruction 
//assign mem_bus_err_flush = (state == `S_NOCACHE && wb_cyc && wb_stb && wb_err) | (mem_fetch_end && ~mem_we && transfer_bus_err_w) | (state == `S_RQ_WR && transfer_bus_err_w);

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `S_IDLE;
    end else if (state == `S_IDLE && mem_req && ~mem_cache_enable && ~illegal_address) begin
        state <= `S_NOCACHE;
    end else if (state == `S_IDLE && mem_req && ~illegal_address) begin
        state <= `S_CREAD;
    end else if (state == `S_CREAD && cache_ghit && ~mem_we) begin
        state <= `S_IDLE;
    end else if (state == `S_CREAD && cache_ghit && mem_we) begin
        state <= `S_RQ_WR;
    end else if (state == `S_CREAD && cache_gmiss && ~all_entries_dirty) begin
        state <= `S_MISS_RD;
    end else if (state == `S_CREAD && cache_gmiss && all_entries_dirty) begin
        state <= `S_MISS_WR;
    end else if (state == `S_MISS_RD && mem_fetch_end && ~mem_we) begin
        state <= `S_IDLE;
    end else if (state == `S_MISS_RD && mem_fetch_end && mem_we) begin
        state <= `S_RQ_WR_WAIT;
    end else if (state == `S_RQ_WR_WAIT) begin
        state <= `S_RQ_WR;
    end else if (state == `S_MISS_WR && mem_op_end) begin
        state <= `S_MISS_RD;
    end else if (state == `S_RQ_WR) begin
        state <= `S_IDLE;
    end else if (state == `S_NOCACHE && wb_cyc && wb_stb && (wb_ack | wb_err)) begin
        state <= `S_IDLE;
    end
end

`define FIRST_PAGE `WB_ADDR_W'h000800
wire illegal_address = mem_addr < `FIRST_PAGE;

assign mem_exception = (state == `S_IDLE && mem_req && illegal_address) | (state == `S_NOCACHE && wb_cyc && wb_stb && wb_err);

wire wb_sel_adr_source = (state == `S_MISS_WR) | (state == `S_CREAD && cache_gmiss && all_entries_dirty);

wire [23:0] wb_adr_w = (wb_sel_adr_source ? {old_entry_addr[`WB_ADDR_W-1:2], line_burst_cnt} : {mem_addr[`WB_ADDR_W-1:2], line_burst_cnt});
assign wb_sel = (mem_cache_enable ? 2'b11 : mem_sel);
assign wb_4_burst = mem_cache_enable;

wire cache_we_en = (mem_fetch_end & ~transfer_bus_err_w) | write_to_cache;
assign cache_mem_in = (write_to_cache ? cache_update_entry : {cache_compare_tag, pre_assembled_line, 2'b01});

wire all_entries_dirty = (&cache_out[0][`DIRTY_BIT:`VALID_BIT]) & (&cache_out[1][`DIRTY_BIT:`VALID_BIT]);
wire [`WB_ADDR_W-1:0] old_entry_addr = {write_source_entry[`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE], mem_addr[`CACHE_OFF_W+`CACHE_IDX_WIDTH-1:0]};

reg [`LINE_SIZE-1:0] line_collect;
wire [`LINE_SIZE-1:0] pre_assembled_line = {wb_i_dat, line_collect[47:0]};

reg transfer_bus_err;
wire transfer_bus_err_w = transfer_bus_err | (wb_cyc & wb_stb & wb_err);

reg int_wb_cyc;

reg [`CACHE_OFF_W-1:0] line_burst_cnt;
always @(posedge i_clk) begin
    if (i_rst) begin
        int_wb_cyc <= 1'b0;
        transfer_bus_err <= 1'b0;
    end else if (state == `S_IDLE && mem_req && ~mem_cache_enable && ~illegal_address) begin
        int_wb_cyc <= 1'b1;
        wb_we <= mem_we;
        line_burst_cnt <= mem_addr[1:0];
        transfer_bus_err <= 1'b0;
    end else if (cache_gmiss || (state == `S_MISS_WR && mem_op_end)) begin
        line_burst_cnt <= 2'b0;
        int_wb_cyc <= 1'b1;
        wb_we <= all_entries_dirty & ~mem_op_end;
        transfer_bus_err <= transfer_bus_err & ~cache_gmiss; // clear error only on new request. Don't update cache if first write failed
    end else if (mem_op_end || (state == `S_NOCACHE && wb_cyc & wb_stb & (wb_ack | wb_err))) begin
        line_burst_cnt <= 2'b0;
        int_wb_cyc <= 1'b0;
    end else if (int_wb_cyc & wb_stb & (wb_ack | wb_err)) begin
        line_burst_cnt <= line_burst_cnt + 1'b1;
        transfer_bus_err <= transfer_bus_err | wb_err;
    end
end

// dealy wishbone start to cut combinational paths on addr
always @(posedge i_clk) begin
    wb_adr <= wb_adr_w;
    if (mem_op_end || (state == `S_NOCACHE && wb_cyc & wb_stb & (wb_ack | wb_err))) begin
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
    end else begin
        wb_cyc <= int_wb_cyc;
        wb_stb <= int_wb_cyc;
    end
end

always @(posedge i_clk) begin
    case (line_burst_cnt)
        default: line_collect[15:0] <= wb_i_dat;
        2'b01: line_collect[31:16] <= wb_i_dat;
        2'b10: line_collect[47:32] <= wb_i_dat;
        2'b11: line_collect[63:48] <= wb_i_dat;
    endcase
end

reg [`ENTRY_SIZE-1:0] cache_hit_entry;
always @* begin
    case (cache_hit)
        default: cache_hit_entry = cache_out[0];
        2'b10: cache_hit_entry = cache_out[1];
    endcase
end

wire [`ENTRY_SIZE-1:0] entry_out = (mem_fetch_end ? {`TAG_SIZE'b0, pre_assembled_line, 2'b00} : cache_hit_entry);

always @* begin
    if (mem_cache_enable) begin
        case (cache_offset)
            default: mem_o_data = entry_out[17:2];
            2'b01: mem_o_data = entry_out[33:18];
            2'b10: mem_o_data = entry_out[49:34];
            2'b11: mem_o_data = entry_out[65:50];
        endcase
    end else begin
        mem_o_data = wb_i_dat;
    end
end

wire [`ENTRY_SIZE-1:0] write_source_entry = cache_out[tx_cache_sel];
always @* begin
    if (mem_cache_enable) begin
        case (line_burst_cnt)
            default: wb_o_dat = write_source_entry[17:2];
            2'b01: wb_o_dat = write_source_entry[33:18];
            2'b10: wb_o_dat = write_source_entry[49:34];
            2'b11: wb_o_dat = write_source_entry[65:50];
        endcase
    end else begin
        wb_o_dat = mem_i_data;
    end
end

reg [`ENTRY_SIZE-1:0] cache_update_entry;
always @* begin
    case ({mem_addr[1:0], mem_sel})
        default: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:18], mem_i_data, 2'b11};
        4'b0001: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:10], mem_i_data[7:0], 2'b11};
        4'b0010: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:18], mem_i_data[15:8], write_source_entry[9:2], 2'b11};
        4'b0111: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:34], mem_i_data, write_source_entry[17:2], 2'b11};
        4'b0101: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:26], mem_i_data[7:0], write_source_entry[17:2], 2'b11};
        4'b0110: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:34], mem_i_data[15:8], write_source_entry[25:2], 2'b11};
        4'b1011: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:50], mem_i_data, write_source_entry[33:2], 2'b11};
        4'b1001: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:42], mem_i_data[7:0], write_source_entry[33:2], 2'b11};
        4'b1010: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:50], mem_i_data[15:8], write_source_entry[41:2], 2'b11};
        4'b1111: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:66], mem_i_data, write_source_entry[49:2], 2'b11};
        4'b1101: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:58], mem_i_data[7:0], write_source_entry[49:2], 2'b11};
        4'b1110: cache_update_entry = {write_source_entry[`ENTRY_SIZE-1:66], mem_i_data[15:8], write_source_entry[57:2], 2'b11};
    endcase
end

reg cache_sel;
reg tx_cache_sel, prev_cache_sel;
always @* begin
    if (~cache_out[0][`VALID_BIT]) cache_sel = 1'b0;
    else if (~cache_out[1][`VALID_BIT]) cache_sel = 1'b1;
    else if (~cache_out[0][`DIRTY_BIT]) cache_sel = 1'b0;
    else if (~cache_out[1][`DIRTY_BIT]) cache_sel = 1'b1;
    else cache_sel = prev_cache_sel + 1'b1;
end

always @(posedge i_clk) begin
    if (state == `S_CREAD && cache_gmiss && ~i_rst) begin
        tx_cache_sel <= cache_sel;
        prev_cache_sel <= tx_cache_sel;
    end else if (state == `S_CREAD && cache_ghit && mem_we && ~i_rst) begin
        tx_cache_sel <= (|(cache_hit&2'b1) ? 1'b0 : 1'b1);
        prev_cache_sel <= tx_cache_sel;
    end
end

always @* begin
    cache_we[1:0] = 2'b0;
    cache_we[tx_cache_sel] = cache_we_en;
end

endmodule

`undef SW
`undef TAG_SIZE
`undef LINE_SIZE
`undef ENTRY_SIZE
`undef CACHE_ASSOC
`undef CACHE_ENTR_N
`undef CACHE_IDX_WIDTH
`undef CACHE_IDXES

module ff_mb_sync #(parameter DATA_W = 1) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input src_clk,
    input dst_clk,

    input src_rst,
    input dst_rst,

    input [DATA_W-1:0] i_data,
    output [DATA_W-1:0] o_data,

    input i_xfer_req // transfer is possible only one time per 3 cycles
);

reg [DATA_W-1:0] s_data_ff;
always @(posedge src_clk) begin
    if (i_xfer_req)
        s_data_ff <= i_data;
end

reg s_xfer_xor_flag;
always @(posedge src_clk) begin
    if(src_rst)
        s_xfer_xor_flag <= 1'b0;
    else if (i_xfer_req)
        s_xfer_xor_flag <= ~s_xfer_xor_flag;
end

reg [2:0] d_xfer_xor_sync;
always @(posedge dst_clk) begin
    if(dst_rst) begin
        d_xfer_xor_sync[2:0] <= 3'b0;
    end else begin
        d_xfer_xor_sync[0] <= s_xfer_xor_flag;
        d_xfer_xor_sync[1] <= d_xfer_xor_sync[0];
        d_xfer_xor_sync[2] <= d_xfer_xor_sync[1];
    end
end

wire d_xfer_flag = d_xfer_xor_sync[1] ^ d_xfer_xor_sync[2];

reg [DATA_W-1:0] d_data;
always @(posedge dst_clk) begin
    if(dst_rst)
        d_data <= 'b0;
    else if(d_xfer_flag)
        d_data <= s_data_ff;
end

assign o_data = d_data;


endmodule

module gpio #(parameter N = 4) (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input [23:0] wb_adr,
    input wb_cyc,
    input wb_stb,
    output wb_ack,
    input wb_we,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat,

    output reg [N-1:0] gpio_out,
    input [N-1:0] gpio_in,
    output reg [N-1:0] gpio_dir
);

localparam GPIO_READ = `WB_ADDR_W'h001010;
localparam GPIO_WRITE = `WB_ADDR_W'h001011;
localparam GPIO_DIR = `WB_ADDR_W'h001012;

always @(posedge i_clk) begin
    if (i_rst) begin
        gpio_dir <= {N{1'b1}}; // default inputs
        gpio_out <= {N{1'b0}};
    end else if (wb_cyc & wb_stb & wb_we & (wb_adr == GPIO_WRITE)) begin
        gpio_out <= wb_i_dat[N-1:0];
    end else if (wb_cyc & wb_stb & wb_we & (wb_adr == GPIO_DIR)) begin
        gpio_dir <= wb_i_dat[N-1:0];
    end
end

always @* begin
    wb_o_dat = 16'b0;
    case (wb_adr)
        default: 
            wb_o_dat[N-1:0] = gpio_in;
        GPIO_WRITE:
            wb_o_dat[N-1:0] = gpio_out;
        GPIO_DIR:
            wb_o_dat[N-1:0] = gpio_dir; 
    endcase
end
assign wb_ack = wb_cyc & wb_stb;

endmodule
module reset_sync (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_rst,
    output o_rst,
    input i_clk
);

assign o_rst = reset_sync_ff[1];
reg [1:0] reset_sync_ff;
always @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        reset_sync_ff[0] <= 1'b1;
        reset_sync_ff[1] <= 1'b1;
    end else begin
        reset_sync_ff[0] <= 1'b0;
        reset_sync_ff[1] <= reset_sync_ff[0];
    end
end

endmodule

// Internal ram for use in embed mode (programmable from outside)
// It maps to address 0x800000 and 0x100000 noncacheable (the same segment, p/m split left to the user)
// In other case maps to 0x7ffe00 cacheable as internal fast memory

module int_ram (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,

    input [8:0] i_addr, // 1kB
    input [`RW-1:0] i_data,
    output reg [`RW-1:0] o_data,
    input i_we
);

reg [`RW-1:0] mem [1:0];

always @(posedge i_clk) begin
    if(i_we)
        mem[i_addr] <= i_data;
    o_data <= mem[i_addr];
end

endmodule

module wb_compressor (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input i_clk,
    input i_rst,

    input wb_cyc,
    input wb_stb,
    input [`WB_ADDR_W-1:0] wb_adr,
    input [`RW-1:0] wb_o_dat,
    output reg [`RW-1:0] wb_i_dat,
    input wb_we,
    input [1:0] wb_sel,
    input wb_8_burst, wb_4_burst,
    output reg wb_ack,
    output reg wb_err,

    output reg [`RW-1:0] cw_io_o,
    input [`RW-1:0] cw_io_i,
    output reg cw_req,
    output reg cw_dir,
    input cw_ack,
    input cw_err
);


`define SW 4
`define S_IDLE `SW'b0
`define S_HDR_1 `SW'b1
`define S_WACK `SW'b10
`define S_DATA_R `SW'b11
`define S_DATA_W `SW'b101
`define S_DATA_WT `SW'b111
`define S_WC `SW'b100
reg [`SW-1:0] state;

wire [3:0] cyc_type = (wb_8_burst ? 4'b001 : (wb_4_burst ? 4'b010 : 4'b000));
wire [`RW-1:0] header_0 = {wb_adr[`WB_ADDR_W-1:`RW], cyc_type, wb_we, wb_sel, 1'b1}; 

reg [`WB_ADDR_W-1:0] l_wb_adr;
wire [`WB_ADDR_W-1:0] wb_exp_adr = l_wb_adr + {20'b0, burst_cnt};
reg [`WB_ADDR_W-1:0] ack_exp_adr;

reg l_we;

`define MAX_BRST_LOG 3
reg [`MAX_BRST_LOG-1:0] burst_end, burst_cnt;

wire xfer_ack = ((cw_ack | cw_err) && wb_cyc && wb_stb);
wire ignored_addr = wb_adr < `WB_ADDR_W'h002000;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `S_IDLE;
        wb_ack <= 1'b0;
        cw_req <= 1'b0;
    end else begin
        case (state) 
            default: begin
                if (wb_cyc & wb_stb & ~ignored_addr) begin
                    state <= `S_HDR_1;
                    cw_io_o <= header_0;
                    cw_dir <= 1'b0;
                    cw_req <= 1'b1;
                    l_we <= wb_we;
                    burst_end <= (wb_8_burst ? 3'd7 : (wb_4_burst ? 3'd3 : 3'b0));

                    burst_cnt <= `MAX_BRST_LOG'b0;
                end
            end
            `S_HDR_1: begin
                state <= `S_WACK;
                cw_req <= 1'b0;
                cw_io_o <= wb_adr[`RW-1:0];
            end
            `S_WACK: begin
                if (cw_ack)
                    state <= (l_we ? `S_DATA_W : `S_DATA_R);
                cw_io_o <= wb_o_dat;
            end
            `S_DATA_R: begin
                cw_dir <= 1'b1;
                cw_io_o <= wb_o_dat;
                
                wb_ack <= 1'b0;
                wb_err <= 1'b0;

                if (xfer_ack && burst_cnt != burst_end) begin
                    burst_cnt <= burst_cnt + 1'b1;
                    wb_ack <= cw_ack; // It is good idea to break combinational path at end of IC
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    ack_exp_adr <= wb_exp_adr;
                end else if (xfer_ack && burst_cnt == burst_end) begin
                    burst_cnt <= `MAX_BRST_LOG'b0;
                    wb_ack <= cw_ack;
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    state <= `S_WC;
                    cw_dir <= ~cw_dir;
                    ack_exp_adr <= wb_exp_adr;
                end
            end
            `S_DATA_W: begin
                cw_dir <= 1'b0;
                wb_ack <= 1'b0;
                wb_err <= 1'b0;
                cw_req <= 1'b0;

                if (xfer_ack && burst_cnt != burst_end) begin
                    burst_cnt <= burst_cnt + 1'b1;
                    wb_ack <= cw_ack;
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    ack_exp_adr <= wb_exp_adr;
                    state <= `S_DATA_WT;
                end else if (xfer_ack && burst_cnt == burst_end) begin
                    burst_cnt <= `MAX_BRST_LOG'b0;
                    wb_ack <= cw_ack;
                    wb_err <= cw_err;
                    wb_i_dat <= cw_io_i;
                    state <= `S_WC;
                    cw_dir <= ~cw_dir;
                    ack_exp_adr <= wb_exp_adr;
                end
            end
            `S_DATA_WT: begin
                cw_req <= 1'b0;
                wb_ack <= 1'b0;
                wb_err <= 1'b0;
                // wait for new data and send sync signal
                // continous data delivery with clock sychronizers is not possible, due to ACK delay
                // it causes no problem with read burst, beacause wishbone communication is unidirectional then
                if (wb_cyc & wb_stb & ~wb_ack) begin // wait for stb reassert when new burst data is delivered
                    cw_req <= 1'b1;
                    cw_io_o <= wb_o_dat;
                    state <= `S_DATA_W;
                end
            end
            `S_WC: begin
                cw_dir <= 1'b0;
                cw_req <= 1'b0;
                wb_ack <= 1'b0;
                wb_err <= 1'b0;
                state <= `S_IDLE;
            end
        endcase
    end
end

`undef S_DATA_R
`undef S_DATA_W

endmodule
module sspi (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif
    
    input i_clk,
    input i_rst,

    input spi_clk,
    input spi_mosi,
    output reg spi_miso,

    output reg wb_cyc,
    output reg wb_stb,
    output reg [23:0] wb_adr,
    input [15:0] wb_i_dat,
    output reg [15:0] wb_o_dat,
    output reg wb_we,
    output [1:0] wb_sel,
    input wb_ack,
    input wb_err
);

reg [2:0] sy_clk;
wire sclk_edge = (sy_clk[2] ^ sy_clk[1]) & ~sy_clk[2];
always @(posedge i_clk) begin
    if (i_rst) sy_clk <= 3'b0;
    else sy_clk <= {sy_clk[1], sy_clk[0], spi_clk};
end

reg [23:0] req_addr;
reg [15:0] req_data;
reg [15:0] res_data;
reg resp_err;

reg [4:0] bit_cnt;
reg [3:0] state;
localparam STATE_IDLE = 4'b0;
localparam STATE_ADDR = 4'b1;
localparam STATE_RW = 4'b10;
localparam STATE_WRITE_DAT = 4'b11;
localparam STATE_WRITE_WB = 4'b100;
localparam STATE_WB_RESP = 4'b101;
localparam STATE_READ_WB = 4'b110;
localparam STATE_READ_DAT = 4'b111;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= STATE_IDLE;
        bit_cnt <= 'b0;
        spi_miso <= 1'b1;
    end else if (sclk_edge) begin
        if (state == STATE_IDLE) begin
            spi_miso <= 1'b1;
            if (~spi_mosi)
                state <= STATE_ADDR; 
        end else if (state == STATE_ADDR) begin
            req_addr[bit_cnt] <= spi_mosi; 
            bit_cnt <= bit_cnt + 5'b1;

            if (bit_cnt == 5'd23) begin
                state <= STATE_RW;
                bit_cnt <= 5'b0;
            end
        end else if (state == STATE_RW) begin
            state <= (spi_mosi ? STATE_WRITE_DAT : STATE_READ_WB);
        end else if (state == STATE_WRITE_DAT) begin
            req_data[bit_cnt[3:0]] <= spi_mosi; 
            bit_cnt <= bit_cnt + 5'b1;

            if (bit_cnt == 5'd15) begin
                state <= STATE_WRITE_WB;
                bit_cnt <= 5'b0;
            end
        end else if (state == STATE_WRITE_WB) begin
            wb_cyc <= 1'b1;
            wb_stb <= 1'b1;
            wb_we <= 1'b1;

            wb_adr <= req_addr;
            wb_o_dat <= req_data;
            
            if ((wb_ack | wb_err) & wb_cyc) begin
                state <= STATE_WB_RESP;
                bit_cnt <= 5'b0;

                resp_err <= wb_err;
                wb_cyc <= 1'b0;
                wb_stb <= 1'b0;
                
                spi_miso <= 1'b0; // end of wait
            end
        end else if (state == STATE_WB_RESP) begin
            spi_miso <= resp_err;
            state <= STATE_IDLE;
        end else if (state == STATE_READ_WB) begin
            wb_cyc <= 1'b1;
            wb_stb <= 1'b1;
            wb_we <= 1'b0;
            wb_adr <= req_addr;

            if ((wb_ack | wb_err) & wb_cyc) begin
                state <= STATE_READ_DAT;
                bit_cnt <= 5'b0;

                res_data <= wb_i_dat;
                resp_err <= wb_err;
                wb_cyc <= 1'b0;
                wb_stb <= 1'b0;
                
                spi_miso <= 1'b0; // end of wait
            end
        end else if (state == STATE_READ_DAT) begin
            spi_miso <= res_data[bit_cnt[3:0]]; 
            bit_cnt <= bit_cnt + 5'b1;

            if (bit_cnt == 5'd15) begin
                state <= STATE_WB_RESP;
                bit_cnt <= 5'b0;
            end
        end
    end
end

assign wb_sel = 2'b11;

endmodule

module wb_cross_clk_outer (
`ifdef USE_POWER_PINS
    inout vccd1,
    inout vssd1,
`endif

    input clk_m,
    input clk_s,
    input m_rst,
    input s_rst,

    input m_wb_cyc,
    input m_wb_stb,
    input [`WB_ADDR_W-1:0] m_wb_adr,
    input [`RW-1:0] m_wb_o_dat,
    output [`RW-1:0] m_wb_i_dat,
    input m_wb_we,
    input [1:0] m_wb_sel,
    input m_wb_8_burst, m_wb_4_burst,
    output m_wb_ack,
    output m_wb_err,

    output s_wb_cyc,
    output s_wb_stb,
    output [`WB_ADDR_W-1:0] s_wb_adr,
    output [`RW-1:0] s_wb_o_dat,
    input [`RW-1:0] s_wb_i_dat,
    output s_wb_we,
    output [1:0] s_wb_sel,
    output s_wb_8_burst, s_wb_4_burst,
    input s_wb_ack,
    input s_wb_err
);

reg prev_stb, prev_ack;
always @(posedge clk_m) begin
    if (m_rst) begin
        prev_stb <= 1'b0;
        prev_ack <= 1'b0;
    end else begin
        prev_stb <= m_wb_stb;
        prev_ack <= (msy_flag_ack |  msy_flag_err);
    end
end
wire m_new_req = m_wb_stb & (~prev_stb | prev_ack) & ~ignored_addr;
wire ignored_addr = m_wb_adr < `WB_ADDR_W'h002000;

`define MAX_BURST_LOG 4
reg [`MAX_BURST_LOG-1:0] m_burst_cnt;

reg m_new_req_flag;
always @(posedge clk_m) begin
    if (m_rst) begin
        m_new_req_flag <= 1'b0;
        m_burst_cnt <= `MAX_BURST_LOG'b0;
    end else if (m_new_req & ~(|m_burst_cnt)) begin
        m_new_req_flag <= ~m_new_req_flag;
        if (m_wb_we)
            m_burst_cnt <= `MAX_BURST_LOG'd1; // burst write is not possible, as new data must be transmitted after each ack, which causes delay
                                              // although burst signal is passed for later optimization of compressed bus write
        else
            m_burst_cnt <= (m_wb_8_burst ? `MAX_BURST_LOG'd8 : (m_wb_4_burst ? `MAX_BURST_LOG'd4 : `MAX_BURST_LOG'd1));
    end else if (msy_flag_ack | msy_flag_err) begin
        m_burst_cnt <= m_burst_cnt - `MAX_BURST_LOG'b1;
    end
end

reg ack_next_hold;
reg [`MAX_BURST_LOG-1:0] s_burst_cnt;
wire burst_in_progress = (|s_burst_cnt);

always @(posedge clk_s) begin
    if (s_rst) begin
        ack_next_hold <= 1'b0;
        s_burst_cnt <= `MAX_BURST_LOG'b0;
    end else if ((s_wb_ack | s_wb_err) & burst_in_progress) begin
        s_burst_cnt <= s_burst_cnt - `MAX_BURST_LOG'b1;
        ack_next_hold <= 1'b0;
    end else if ((s_wb_ack | s_wb_err) & ~burst_in_progress) begin
        ack_next_hold <= 1'b1;
    end else if (ssy_newreq) begin
        ack_next_hold <= 1'b0;
        if (m_wb_we)
            s_burst_cnt <= 'b0;
        else
            s_burst_cnt <= (s_wb_8_burst ? `MAX_BURST_LOG'd7 : (s_wb_4_burst ? `MAX_BURST_LOG'd3 : `MAX_BURST_LOG'd0));
    end
end

assign s_wb_stb = s_wb_cyc & ~ack_next_hold;

// XOR strobe setup
reg ack_xor_flag;
reg err_xor_flag;
always @(posedge clk_s) begin
    if(s_rst) begin
        ack_xor_flag <= 1'b0;
        err_xor_flag <= 1'b0;
    end else begin
        if(s_wb_ack)
            ack_xor_flag <= ~ack_xor_flag;
        if(s_wb_err)
            err_xor_flag <= ~err_xor_flag;
    end
end

reg prev_xor_ack, prev_xor_err;
always @(posedge clk_m) begin
    if(m_rst) begin
        prev_xor_ack <= 1'b0;
        prev_xor_err <= 1'b0;
    end else begin
        prev_xor_ack <= msy_xor_ack;
        prev_xor_err <= msy_xor_err;
    end
end
wire msy_flag_ack = prev_xor_ack ^ msy_xor_ack;
wire msy_flag_err = prev_xor_err ^ msy_xor_err;

reg prev_xor_newreq;
always @(posedge clk_s) begin
    if(s_rst)
        prev_xor_newreq <= 1'b0;
    else
        prev_xor_newreq <= ssy_flag_newreq; 
end
wire ssy_newreq = prev_xor_newreq ^ ssy_flag_newreq;

// S->M ff sync
`define SM_SYNC_W 2+16
wire [`SM_SYNC_W-1:0] smsync1;
ff_mb_sync #(.DATA_W(`SM_SYNC_W)) s_m_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .src_clk(clk_s),
    .dst_clk(clk_m),
    .src_rst(s_rst),
    .dst_rst(m_rst),
    .i_data({s_wb_i_dat, err_xor_flag^s_wb_err, ack_xor_flag^s_wb_ack}),
    .o_data(smsync1),
    .i_xfer_req(s_wb_ack | s_wb_err) // NOTE: CLOCK MUST BE DIVIDED BY >4 TO NOT VIOLATE 3 CYCLE DELAY (to dst clock) 
);

assign m_wb_i_dat = smsync1[17:2];
assign m_wb_ack = msy_flag_ack; 
assign m_wb_err = msy_flag_err; 

wire msy_xor_ack = smsync1[0];
wire msy_xor_err = smsync1[1];

// M->S ff sync
`define MS_SYNC_W 1+24+16+1+2+2+1
wire [`MS_SYNC_W-1:0] mssync1;

ff_mb_sync #(.DATA_W(`MS_SYNC_W)) m_s_sync (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .src_clk(clk_m),
    .dst_clk(clk_s),
    .src_rst(m_rst),
    .dst_rst(s_rst),
    .i_data({m_wb_cyc, m_wb_adr, m_wb_o_dat, m_wb_we, m_wb_sel, m_wb_4_burst, m_wb_8_burst, ~m_new_req_flag}),
    .o_data(mssync1),
    .i_xfer_req(m_new_req & ~(|m_burst_cnt))
);

wire ssy_flag_newreq = mssync1[0]; 
assign s_wb_8_burst = mssync1[1];
assign s_wb_4_burst = mssync1[2];
assign s_wb_sel = mssync1[4:3];
assign s_wb_we = mssync1[5];
assign s_wb_o_dat = mssync1[21:6];
assign s_wb_adr = mssync1[45:22];
assign s_wb_cyc = mssync1[46];

endmodule
module icache_ram (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input i_clk,

    input [4:0] i_addr,
    input [137:0] i_data,
    output reg [137:0] o_data,
    input i_we
);

reg [137:0] mem [31:0];

always @(posedge i_clk) begin
    if(i_we)
        mem[i_addr] <= i_data;
    o_data <= mem[i_addr];
end

endmodule


module wb_decomp (
    input i_clk,
    input i_rst,

    output reg wb_cyc,
    output reg wb_stb,
    output [`WB_ADDR_W-1:0] wb_adr,
    output reg [`RW-1:0] wb_o_dat,
    input [`RW-1:0] wb_i_dat,
    output reg wb_we,
    output reg [1:0] wb_sel,
    input wb_ack,
    input wb_err,

    input [`RW-1:0] cw_io_i,
    output reg [`RW-1:0] cw_io_o,
    input cw_req,
    input cw_dir,
    output reg cw_ack,
    output reg cw_err
);

`define SW 4
`define S_IDLE `SW'b0
`define S_HDR_1 `SW'b1
`define S_DATA_R `SW'b10
`define S_DATA_W `SW'b11
`define S_DATA_W_PRE `SW'b100
reg [`SW-1:0] state;

reg [`WB_ADDR_W-1:0] l_wb_adr;
assign wb_adr = l_wb_adr + {20'b0, burst_cnt};

`define MAX_BRST_LOG 3
reg [`MAX_BRST_LOG-1:0] burst_end, burst_cnt;

`define DATA_WRITE_DELAY 3
`define DATA_WDEL_LOG 2
reg [`DATA_WDEL_LOG-1:0] wdel_cnt;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= `S_IDLE;
        cw_ack <= 1'b0;
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        wdel_cnt <= `DATA_WDEL_LOG'b0;
    end else begin
        case (state) 
            default: begin
                cw_ack <= 1'b0;
                cw_err <= 1'b0;
                wb_cyc <= 1'b0;
                wb_stb <= 1'b0;
                if (cw_req & cw_io_i[0]) begin
                    state <= `S_HDR_1;
                    l_wb_adr[`WB_ADDR_W-1:`RW] <= cw_io_i[`RW-1:8];
                    wb_we <= cw_io_i[3];
                    wb_sel <= cw_io_i[2:1];
                    burst_end <= (~(|cw_io_i[7:4]) ? 3'd0 : (cw_io_i[4] ? 3'd7 : 3'd3));
                    burst_cnt <= `MAX_BRST_LOG'b0;
                    cw_ack <= 1'b1;
                end
            end
            `S_HDR_1: begin
                state <= (wb_we ? `S_DATA_W_PRE : `S_DATA_R);
                l_wb_adr[`RW-1:0] <= cw_io_i;
                cw_ack <= 1'b0;
                if (~wb_we) begin
                    wb_cyc <= 1'b1;
                    wb_stb <= 1'b1;
                end
            end
            `S_DATA_R: begin
                cw_ack <= 1'b0;
                cw_err <= 1'b0;
                if ((wb_ack | wb_err) && burst_cnt != burst_end) begin
                    cw_ack <= wb_ack;
                    cw_err <= wb_err;
                    burst_cnt <= burst_cnt + `MAX_BRST_LOG'b1;
                    cw_io_o <= wb_i_dat;
                end else if ((wb_ack | wb_err) && burst_cnt == burst_end) begin
                    cw_ack <= wb_ack;
                    cw_err <= wb_err;
                    wb_stb <= 1'b0;
                    state <= `S_IDLE;
                    cw_io_o <= wb_i_dat;
                end
            end
            `S_DATA_W_PRE: begin
                wb_cyc <= 1'b1;
                wb_stb <= 1'b1;
                wb_o_dat <= cw_io_i;
                state <= `S_DATA_W;
            end
            `S_DATA_W: begin
                if (~wb_stb & cw_req) begin // wait for write burst new data
                    wb_stb <= 1'b1;
                    wb_o_dat <= cw_io_i;
                end

                cw_ack <= 1'b0;
                cw_err <= 1'b0;
                if ((wb_ack | wb_err) && burst_cnt != burst_end) begin
                    cw_ack <= wb_ack;
                    cw_err <= wb_err;
                    wb_stb <= 1'b0;
                    burst_cnt <= burst_cnt + `MAX_BRST_LOG'b1;
                end else if ((wb_ack | wb_err) && burst_cnt == burst_end) begin
                    cw_ack <= wb_ack;
                    cw_err <= wb_err;
                    wb_cyc <= 1'b0;
                    wb_stb <= 1'b0;
                    state <= `S_IDLE;
                end
            end
        endcase
    end
end

endmodule

`undef S_DATA_R
`undef S_DATA_W
`undef SW

module icache (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input i_clk,
    input i_rst,

    input mem_req,
    output mem_ack,
    input [`RW-1:0] mem_addr,
    output reg [`I_SIZE-1:0] mem_data,
    input mem_ppl_submit,
    input mem_cache_flush,

    // output interface
    output reg wb_cyc,
    output reg wb_stb,
    input [`RW-1:0] wb_i_dat,
    output reg [`RW-1:0]  wb_adr,
    output wb_we,
    input wb_ack,
    output [1:0] wb_sel,
    input wb_err
);

assign wb_sel = 2'b11;
assign wb_we = 1'b0;

`define TAG_SIZE 9
`define LINE_SIZE 128
// 9b tag + 128b line + 1b valid
`define ENTRY_SIZE 138
`define CACHE_ASSOC 1
`define CACHE_ENTR_N 32
`define CACHE_SETS_N 8
`define CACHE_OFF_W 2

`define CACHE_IDX_WIDTH 5
`define CACHE_IDXES 32


wire [`TAG_SIZE-1:0] compare_tag = cache_read_addr[15:7];
wire [`TAG_SIZE-1:0] write_tag = cache_write_addr[15:7];
wire [`CACHE_IDX_WIDTH-1:0] mem_index = (submit_pending ?  submit_pending_addr[6:2] : mem_addr[6:2]);
wire [`CACHE_IDX_WIDTH-1:0] compare_index = cache_read_addr[6:2];
wire [`CACHE_IDX_WIDTH-1:0] wire_index = cache_write_addr[6:2];
wire [`CACHE_OFF_W-1:0] compare_off = cache_read_addr[1:0];
wire [`CACHE_OFF_W-1:0] write_off = cache_write_addr[1:0];

wire [`ENTRY_SIZE-1:0] cache_mem_in = cache_write_entry;
wire [`ENTRY_SIZE-1:0] cache_out [`CACHE_ASSOC-1:0];
reg [`CACHE_ASSOC-1:0] cache_we;
wire [`CACHE_ASSOC-1:0] cache_hit;

reg [`RW-1:0] cache_read_addr, cache_write_addr;
reg cache_read_valid;
wire cache_write_valid = wb_cyc & wb_stb;
reg prev_write_compl;

wire [`CACHE_IDX_WIDTH-1:0] cache_addr = ((|cache_we) ? wire_index : mem_index);

`ifndef USE_OC_RAM

icache_ram mem (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(i_clk), .i_addr(cache_addr), .i_data(cache_mem_in),
    .o_data(cache_out[0]), .i_we(cache_we[0])
);

`else

ocram_icache mem (
    .clock(i_clk), .address(cache_addr), .data(cache_mem_in),
    .q(cache_out[0]), .wren(cache_we[0])
);

`endif

// we have to use valid bits from delayed index for data to arrive at the same time as memory
assign cache_hit[0] = (cache_out[0][`ENTRY_SIZE-1:`ENTRY_SIZE-`TAG_SIZE] == compare_tag) && valid_bits[compare_index]; 

reg [`CACHE_ENTR_N-1:0] valid_bits;
always @(posedge i_clk) begin
    if (i_rst | mem_cache_flush) begin
        valid_bits <= `CACHE_ENTR_N'b0;
    end else if (cache_we[0]) begin
        valid_bits[cache_addr] <= cache_mem_in[0];
    end
end

assign mem_ack = cache_ghit | prev_mem_fetch_end;

wire cache_miss = cache_read_valid & (~(|cache_hit) | flush_prev_cycle);
wire cache_ghit = cache_read_valid & ((|cache_hit) & ~flush_prev_cycle);

always @(posedge i_clk)
    cache_read_addr <= (submit_pending ? submit_pending_addr : mem_addr);

always @(posedge i_clk) begin
    if(cache_read_valid)
        cache_write_addr <= cache_read_addr;
end

always @(posedge i_clk)
    prev_write_compl <= |cache_we;

reg submit_pending;
reg [`RW-1:0] submit_pending_addr;
always @(posedge i_clk) begin
    if(i_rst)
        submit_pending <= 1'b0;
    else if (mem_ppl_submit & ~accept_ok) begin
        submit_pending <= 1'b1;
        submit_pending_addr <= mem_addr;
    end else if (accept_ok)
        submit_pending <= 1'b0;
end

wire accept_ok = mem_req & ~cache_write_valid & ~cache_miss;

reg flush_prev_cycle;
always @(posedge i_clk)
    flush_prev_cycle <= mem_cache_flush; // feedback after invalidating memortm takes 2 cycles; use this to force miss signal after 1 cycle

always @(posedge i_clk) begin
    if(i_rst)
        cache_read_valid <= 1'b0;
    else
        cache_read_valid <= accept_ok & (submit_pending | mem_ppl_submit);
end

wire mem_fetch_end = wb_cyc & wb_stb & (wb_ack | wb_err) & (&line_burst_cnt);

reg prev_mem_fetch_end;
always @(posedge i_clk) begin
    if (i_rst)
        prev_mem_fetch_end <= 1'b0;
    else
        prev_mem_fetch_end <= mem_fetch_end;
end

reg [`LINE_SIZE-1:0] line_collect;
reg [`CACHE_OFF_W:0] line_burst_cnt;
reg invalidate_bus_err;
always @(posedge i_clk) begin
    if (i_rst) begin
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        wb_adr <= 16'b0;
        invalidate_bus_err <= 1'b0;
    end else if (mem_fetch_end) begin
        line_burst_cnt <= 3'b0;
        wb_adr <= 16'b0;
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        invalidate_bus_err <= invalidate_bus_err | wb_err;
    end else if (wb_cyc & wb_stb & (wb_ack | wb_err)) begin
        wb_adr <= {cache_write_addr[14:2], line_burst_cnt+1'b1};
        line_burst_cnt <= line_burst_cnt + 1'b1;
        invalidate_bus_err <= invalidate_bus_err | wb_err;
    end else if(cache_miss) begin
        line_burst_cnt <= 3'b0;    
        if(cache_read_valid)
            wb_adr <= {cache_read_addr[14:2], 3'b0};
        else
            wb_adr <= {cache_write_addr[14:2], 3'b0};
        wb_cyc <= 1'b1;
        wb_stb <= 1'b1;
        invalidate_bus_err <= 1'b0;
    end
end

reg invalidate_cache_update; // don't write pending memory request to cache after flush (fetch unit invalidates this request)
always @(posedge i_clk) begin
    if (i_rst) begin
        invalidate_cache_update <= 1'b0;
    end else if (mem_fetch_end) begin
        invalidate_cache_update <= 1'b0;
    end else if (mem_cache_flush & (cache_write_valid | cache_miss)) begin
        invalidate_cache_update <= 1'b1;
    end
end

wire invalidate_bus_err_w = (mem_fetch_end & wb_err) | invalidate_bus_err;

wire [`LINE_SIZE-1:0] pre_assembled_line = {wb_i_dat, line_collect[111:0]};
wire [`ENTRY_SIZE-1:0] cache_write_entry = {write_tag, pre_assembled_line, 1'b1};
wire cache_we_en = mem_fetch_end & ~invalidate_cache_update & ~invalidate_bus_err_w;
always @* begin
    cache_we[0] = cache_we_en;
end


always @(posedge i_clk) begin
    case (line_burst_cnt)
        default: line_collect[15:0] <= wb_i_dat;
        3'b001: line_collect[31:16] <= wb_i_dat;
        3'b010: line_collect[47:32] <= wb_i_dat;
        3'b011: line_collect[63:48] <= wb_i_dat;
        3'b100: line_collect[79:64] <= wb_i_dat;
        3'b101: line_collect[95:80] <= wb_i_dat;
        3'b110: line_collect[111:96] <= wb_i_dat;
        3'b111: line_collect[127:112] <= wb_i_dat;
    endcase
end

reg [`ENTRY_SIZE-1:0] cache_hit_entry;

wire [`ENTRY_SIZE-1:0] entry_out = (prev_mem_fetch_end ? {`TAG_SIZE'b0, line_collect[127:0], 1'b0} : cache_hit_entry);

always @* begin
    cache_hit_entry = cache_out[0];
end

wire [`CACHE_OFF_W-1:0] offset_out = (prev_mem_fetch_end ? write_off : compare_off);
always @* begin
    case (offset_out)
        default: mem_data = entry_out[32:1];
        2'b01: mem_data = entry_out[64:33];
        2'b10: mem_data = entry_out[96:65];
        2'b11: mem_data = entry_out[128:97];
    endcase
end

endmodule

`undef TAG_SIZE
`undef LINE_SIZE
`undef ENTRY_SIZE
`undef CACHE_ASSOC
`undef CACHE_ENTR_N
`undef CACHE_SETS_N
`undef CACHE_OFF_W
`undef CACHE_IDX_WIDTH
`undef CACHE_IDXES
`undef SW

///////////////////////////////////// top.v
// Connects all moules to interconnects
// No logic or buffers allowed here
// See both interconnect codes to see how it is connected

`define MPRJ_IO_PADS 38
`define WB_DATA_W 16 
`define WB_SEL_BITS 2

`define TOP_DEBUG

module cpu_top (
    // IOs
    input  [`MPRJ_IO_PADS-1:0] m_io_in,
    output [`MPRJ_IO_PADS-1:0] m_io_out,
    output [`MPRJ_IO_PADS-1:0] m_io_oeb,

    // Wishbone from managment core
    input mgt_wb_clk_i,
    input mgt_wb_rst_i,
    input mgt_wb_stb_i,
    input mgt_wb_cyc_i,
    input mgt_wb_we_i,
    input [3:0] mgt_wb_sel_i,
    input [31:0] mgt_wb_dat_i,
    input [31:0] mgt_wb_adr_i,
    output mgt_wb_ack_o,
    output [31:0] mgt_wb_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IRQ
    output [2:0] irq

`ifdef TOP_DEBUG
    ,output [15:0] dbg_r0,
    output [15:0] dbg_pc
`endif
);

/*
 * UPPER INTERCONNECT
 * External bus, clocking, outside signals
 */
wire inner_clock, inner_reset;
interconnect_outer interconnect_outer (
    .m_io_in(m_io_in),
    .m_io_out(m_io_out),
    .m_io_oeb(m_io_oeb),
    .mgt_wb_clk_i(mgt_wb_clk_i),
    .mgt_wb_rst_i(mgt_wb_rst_i),
    .mgt_wb_stb_i(mgt_wb_stb_i),
    .mgt_wb_cyc_i(mgt_wb_cyc_i),
    .mgt_wb_we_i(mgt_wb_we_i),
    .mgt_wb_sel_i(mgt_wb_sel_i),
    .mgt_wb_dat_i(mgt_wb_dat_i),
    .mgt_wb_adr_i(mgt_wb_adr_i),
    .mgt_wb_ack_o(mgt_wb_ack_o),
    .mgt_wb_dat_o(mgt_wb_dat_o),
    .la_data_in(la_data_in),
    .la_data_out(la_data_out),
    .la_oenb(la_oenb),
    .irq(irq),
    .inner_clock(inner_clock),
    .inner_reset(inner_reset),
    .inner_wb_cyc(inner_wb_cyc),
    .inner_wb_stb(inner_wb_stb),
    .inner_wb_we(inner_wb_we),
    .inner_wb_adr(inner_wb_adr),
    .inner_wb_o_dat(inner_wb_o_dat),
    .inner_wb_i_dat(inner_wb_i_dat),
    .inner_wb_ack(inner_wb_ack),
    .inner_wb_err(inner_wb_err),
    .inner_wb_sel(inner_wb_sel),
    .inner_wb_4_burst(inner_wb_4_burst),
    .inner_wb_8_burst(inner_wb_8_burst),
    .inner_ext_irq(inner_ext_irq),
    .inner_embed_mode(inner_embed_mode),
    .inner_disable(inner_disable),
    .iram_clk(iram_clk),
    .iram_addr(iram_addr),
    .iram_i_data(iram_i_data),
    .iram_o_data(iram_o_data),
    .iram_we(iram_we)
);

wire inner_wb_cyc, inner_wb_stb;
wire inner_wb_we;
wire [`WB_ADDR_W-1:0] inner_wb_adr;
wire [`WB_DATA_W-1:0] inner_wb_o_dat;
wire [`WB_DATA_W-1:0] inner_wb_i_dat;
wire inner_wb_ack, inner_wb_err;
wire [`WB_SEL_BITS-1:0] inner_wb_sel;
wire inner_wb_4_burst, inner_wb_8_burst;
wire inner_ext_irq;
wire inner_embed_mode;
wire inner_disable;

/*
 * LOWER INTERCONNECT
 * All cpu functions: core and caches
 */

interconnect_inner interconnect_inner (
    .core_clock(inner_clock),
    .core_reset(inner_reset),
    .inner_wb_cyc(inner_wb_cyc), // why some compilers don't support .* ???
    .inner_wb_stb(inner_wb_stb),
    .inner_wb_we(inner_wb_we),
    .inner_wb_adr(inner_wb_adr),
    .inner_wb_o_dat(inner_wb_o_dat),
    .inner_wb_i_dat(inner_wb_i_dat),
    .inner_wb_ack(inner_wb_ack),
    .inner_wb_err(inner_wb_err),
    .inner_wb_sel(inner_wb_sel),
    .inner_wb_4_burst(inner_wb_4_burst),
    .inner_wb_8_burst(inner_wb_8_burst),
    .inner_ext_irq(inner_ext_irq),
    .inner_embed_mode(inner_embed_mode),
    .c0_clk(c0_clk),
    .c0_rst(c0_rst),
    .c0_disable(c0_disable),
    .c0_o_req_addr(c0_o_req_addr),
    .c0_o_req_active(c0_o_req_active), 
    .c0_o_req_ppl_submit(c0_o_req_ppl_submit),
    .c0_i_req_data(c0_i_req_data),
    .c0_i_req_data_valid(c0_i_req_data_valid),
    .c0_dbg_r0(c0_dbg_r0), 
    .c0_dbg_pc(c0_dbg_pc),
    .c0_o_mem_addr(c0_o_mem_addr),
    .c0_o_mem_data(c0_o_mem_data),
    .c0_i_mem_data(c0_i_mem_data),
    .c0_o_mem_req(c0_o_mem_req),
    .c0_o_mem_we(c0_o_mem_we),
    .c0_i_mem_ack(c0_i_mem_ack),
    .c0_o_mem_sel(c0_o_mem_sel),
    .c0_i_irq(c0_i_irq),
    .c0_o_c_instr_page(c0_o_c_instr_page),
    .c0_o_c_data_page(c0_o_c_data_page),
    .c0_sr_bus_addr(c0_sr_bus_addr),
    .c0_sr_bus_data_o(c0_sr_bus_data_o),
    .c0_sr_bus_we(c0_sr_bus_we),
    .c0_o_icache_flush(c0_o_icache_flush),
    .c0_i_mem_exception(c0_i_mem_exception),
    .c0_i_mc_core_int(c0_i_mc_core_int),
    .c0_i_core_int_sreg(c0_i_core_int_sreg),
    .c0_o_c_instr_long(c0_o_c_instr_long),
    .c0_o_instr_long_addr(c0_o_instr_long_addr),
    .c0_o_mem_long_mode(c0_o_mem_long),
    .c0_o_mem_high_addr(c0_o_mem_addr_high),
    .c0_dbg_out(c0_dbg_out),
    .c0_dbg_in(c0_dbg_in),
    .c1_clk(c1_clk),
    .c1_rst(c1_rst),
    .c1_disable(c1_disable),
    .c1_o_req_addr(c1_o_req_addr),
    .c1_o_req_active(c1_o_req_active),
    .c1_o_req_ppl_submit(c1_o_req_ppl_submit),
    .c1_i_req_data(c1_i_req_data),
    .c1_i_req_data_valid(c1_i_req_data_valid),
    .c1_dbg_r0(c1_dbg_r0),
    .c1_dbg_pc(c1_dbg_pc),
    .c1_o_mem_addr(c1_o_mem_addr),
    .c1_o_mem_data(c1_o_mem_data),
    .c1_i_mem_data(c1_i_mem_data),
    .c1_o_mem_req(c1_o_mem_req),
    .c1_o_mem_we(c1_o_mem_we),
    .c1_i_mem_ack(c1_i_mem_ack),
    .c1_o_mem_sel(c1_o_mem_sel),
    .c1_i_irq(c1_i_irq),
    .c1_o_c_instr_page(c1_o_c_instr_page),
    .c1_o_c_data_page(c1_o_c_data_page),
    .c1_sr_bus_addr(c1_sr_bus_addr),
    .c1_sr_bus_data_o(c1_sr_bus_data_o),
    .c1_sr_bus_we(c1_sr_bus_we),
    .c1_o_icache_flush(c1_o_icache_flush),
    .c1_i_mem_exception(c1_i_mem_exception),
    .c1_i_mc_core_int(c1_i_mc_core_int),
    .c1_i_core_int_sreg(c1_i_core_int_sreg),
    .c1_o_c_instr_long(c1_o_c_instr_long),
    .c1_o_instr_long_addr(c1_o_instr_long_addr),
    .c1_o_mem_long_mode(c1_o_mem_long),
    .c1_o_mem_high_addr(c1_o_mem_addr_high),
    .c1_dbg_out(c1_dbg_out),
    .c1_dbg_in(c1_dbg_in),
    .ic0_clk(ic0_clk),
    .ic0_rst(ic0_rst),
    .ic0_mem_req(ic0_mem_req),
    .ic0_mem_ack(ic0_mem_ack),
    .ic0_mem_addr(ic0_mem_addr),
    .ic0_mem_data(ic0_mem_data),
    .ic0_mem_ppl_submit(ic0_mem_ppl_submit),
    .ic0_mem_cache_flush(ic0_mem_cache_flush),
    .ic0_wb_cyc(ic0_wb_cyc),
    .ic0_wb_stb(ic0_wb_stb),
    .ic0_wb_i_dat(ic0_wb_i_dat),
    .ic0_wb_adr(ic0_wb_adr),
    .ic0_wb_we(ic0_wb_we),
    .ic0_wb_ack(ic0_wb_ack),
    .ic0_wb_sel(ic0_wb_sel),
    .ic0_wb_err(ic0_wb_err),
    .ic1_clk(ic1_clk),
    .ic1_rst(ic1_rst),
    .ic1_mem_req(ic1_mem_req),
    .ic1_mem_ack(ic1_mem_ack),
    .ic1_mem_addr(ic1_mem_addr),
    .ic1_mem_data(ic1_mem_data),
    .ic1_mem_ppl_submit(ic1_mem_ppl_submit),
    .ic1_mem_cache_flush(ic1_mem_cache_flush),
    .ic1_wb_cyc(ic1_wb_cyc),
    .ic1_wb_stb(ic1_wb_stb),
    .ic1_wb_i_dat(ic1_wb_i_dat),
    .ic1_wb_adr(ic1_wb_adr),
    .ic1_wb_we(ic1_wb_we),
    .ic1_wb_ack(ic1_wb_ack),
    .ic1_wb_sel(ic1_wb_sel),
    .ic1_wb_err(ic1_wb_err),
    .dcache_clk(dcache_clk),
    .dcache_rst(dcache_rst),
    .dcache_mem_req(dcache_mem_req),
    .dcache_mem_we(dcache_mem_we),
    .dcache_mem_ack(dcache_mem_ack),
    .dcache_mem_addr(dcache_mem_addr),
    .dcache_mem_i_data(dcache_mem_i_data),
    .dcache_mem_o_data(dcache_mem_o_data),
    .dcache_mem_sel(dcache_mem_sel),
    .dcache_mem_cache_enable(dcache_mem_cache_enable),
    .dcache_mem_exception(dcache_mem_exception),
    .dcache_wb_cyc(dcache_wb_cyc),
    .dcache_wb_stb(dcache_wb_stb),
    .dcache_wb_i_dat(dcache_wb_i_dat),
    .dcache_wb_o_dat(dcache_wb_o_dat),
    .dcache_wb_adr(dcache_wb_adr),
    .dcache_wb_we(dcache_wb_we),
    .dcache_wb_sel(dcache_wb_sel),
    .dcache_wb_4_burst(dcache_wb_4_burst),
    .dcache_wb_ack(dcache_wb_ack),
    .dcache_wb_err(dcache_wb_err),
    .inner_disable(inner_disable)
);

// CORES

wire c0_clk, c0_rst;
wire [`RW-1:0] c0_o_req_addr;
wire c0_o_req_active;
wire [`I_SIZE-1:0] c0_i_req_data;
wire c0_i_req_data_valid;
wire c0_o_req_ppl_submit;
wire [`RW-1:0] c0_o_mem_addr, c0_o_mem_data, c0_i_mem_data;
wire c0_o_mem_req, c0_o_mem_we, c0_i_mem_ack;
wire [`ADDR_BYTES-1:0] c0_o_mem_sel;
wire [`RW-1:0] c0_sr_bus_addr, c0_sr_bus_data_o, c0_i_core_int_sreg;
wire c0_sr_bus_we;
wire c0_o_c_instr_page, c0_o_c_data_page, c0_o_icache_flush, c0_i_mem_exception;
wire [`RW-1:0] c0_dbg_r0, c0_dbg_pc;
wire [35:0] c0_dbg_out;
wire [3:0] c0_dbg_in;
wire c0_i_mc_core_int, c0_disable, c0_i_irq;
wire c0_o_c_instr_long;
wire [7:0] c0_o_instr_long_addr;
wire c0_o_mem_long;
wire [7:0] c0_o_mem_addr_high;

core #(.CORENO(0), .INT_VEC(1)) core_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(c0_clk), .i_rst(c0_rst), .o_req_addr(c0_o_req_addr), .o_req_active(c0_o_req_active), .i_req_data(c0_i_req_data), .i_req_data_valid(c0_i_req_data_valid), .o_req_ppl_submit(c0_o_req_ppl_submit),
    .o_mem_addr(c0_o_mem_addr), .o_mem_data(c0_o_mem_data), .i_mem_data(c0_i_mem_data), .o_mem_req(c0_o_mem_req), .o_mem_we(c0_o_mem_we), .i_mem_ack(c0_i_mem_ack),
    .dbg_r0(c0_dbg_r0), .dbg_pc(c0_dbg_pc), .i_irq(c0_i_irq), .o_c_instr_page(c0_o_c_instr_page), .sr_bus_addr(c0_sr_bus_addr),
    .sr_bus_data_o(c0_sr_bus_data_o), .sr_bus_we(c0_sr_bus_we), .o_icache_flush(c0_o_icache_flush), .o_mem_sel(c0_o_mem_sel), .o_c_data_page(c0_o_c_data_page),
    .i_mem_exception(c0_i_mem_exception), .dbg_out(c0_dbg_out), .dbg_in(c0_dbg_in), .i_disable(c0_disable), .i_mc_core_int(c0_i_mc_core_int), .i_core_int_sreg(c0_i_core_int_sreg),
    .o_c_instr_long(c0_o_c_instr_long), .o_instr_long_addr(c0_o_instr_long_addr), .o_mem_long(c0_o_mem_long), .o_mem_addr_high(c0_o_mem_addr_high)
);

wire c1_clk, c1_rst;
wire [`RW-1:0] c1_o_req_addr;
wire c1_o_req_active;
wire [`I_SIZE-1:0] c1_i_req_data;
wire c1_i_req_data_valid;
wire c1_o_req_ppl_submit;
wire [`RW-1:0] c1_o_mem_addr, c1_o_mem_data, c1_i_mem_data;
wire c1_o_mem_req, c1_o_mem_we, c1_i_mem_ack;
wire [`ADDR_BYTES-1:0] c1_o_mem_sel;
wire [`RW-1:0] c1_sr_bus_addr, c1_sr_bus_data_o, c1_i_core_int_sreg;
wire c1_sr_bus_we;
wire c1_o_c_instr_page, c1_o_c_data_page, c1_o_icache_flush, c1_i_mem_exception;
wire [`RW-1:0] c1_dbg_r0, c1_dbg_pc;
wire [35:0] c1_dbg_out;
wire [3:0] c1_dbg_in;
wire c1_i_mc_core_int, c1_disable, c1_i_irq;
wire c1_o_c_instr_long;
wire [7:0] c1_o_instr_long_addr;
wire c1_o_mem_long;
wire [7:0] c1_o_mem_addr_high;

core #(.CORENO(1), .INT_VEC(2)) core_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(c1_clk), .i_rst(c1_rst), .o_req_addr(c1_o_req_addr), .o_req_active(c1_o_req_active), .i_req_data(c1_i_req_data), .i_req_data_valid(c1_i_req_data_valid), .o_req_ppl_submit(c1_o_req_ppl_submit),
    .o_mem_addr(c1_o_mem_addr), .o_mem_data(c1_o_mem_data), .i_mem_data(c1_i_mem_data), .o_mem_req(c1_o_mem_req), .o_mem_we(c1_o_mem_we), .i_mem_ack(c1_i_mem_ack),
    .dbg_r0(c1_dbg_r0), .dbg_pc(c1_dbg_pc), .i_irq(c1_i_irq), .o_c_instr_page(c1_o_c_instr_page), .sr_bus_addr(c1_sr_bus_addr),
    .sr_bus_data_o(c1_sr_bus_data_o), .sr_bus_we(c1_sr_bus_we), .o_icache_flush(c1_o_icache_flush), .o_mem_sel(c1_o_mem_sel), .o_c_data_page(c1_o_c_data_page),
    .i_mem_exception(c1_i_mem_exception), .dbg_out(c1_dbg_out), .dbg_in(c1_dbg_in), .i_disable(c1_disable), .i_mc_core_int(c1_i_mc_core_int), .i_core_int_sreg(c1_i_core_int_sreg),
    .o_c_instr_long(c1_o_c_instr_long), .o_instr_long_addr(c1_o_instr_long_addr), .o_mem_long(c1_o_mem_long), .o_mem_addr_high(c1_o_mem_addr_high)
);

// DCACHE

wire dcache_clk;
wire dcache_rst;
wire dcache_mem_req;
wire dcache_mem_we;
wire dcache_mem_ack;
wire [`WB_ADDR_W-1:0] dcache_mem_addr;
wire [`RW-1:0] dcache_mem_i_data;
wire [`RW-1:0] dcache_mem_o_data;
wire [1:0] dcache_mem_sel;
wire dcache_mem_cache_enable;
wire dcache_mem_exception;
wire dcache_wb_cyc;
wire dcache_wb_stb;
wire [`RW-1:0] dcache_wb_i_dat;
wire [`RW-1:0] dcache_wb_o_dat;
wire [`WB_ADDR_W-1:0]  dcache_wb_adr;
wire dcache_wb_we;
wire [1:0] dcache_wb_sel;
wire dcache_wb_4_burst;
wire dcache_wb_ack;
wire dcache_wb_err;

dcache dcache (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(dcache_clk), .i_rst(dcache_rst), .mem_req(dcache_mem_req), .mem_addr(dcache_mem_addr), .mem_i_data(dcache_mem_i_data), .mem_o_data(dcache_mem_o_data), .mem_we(dcache_mem_we), .mem_ack(dcache_mem_ack), .mem_sel(dcache_mem_sel),
    .wb_cyc(dcache_wb_cyc), .wb_stb(dcache_wb_stb), .wb_we(dcache_wb_we), .wb_ack(dcache_wb_ack), .wb_i_dat(dcache_wb_i_dat), .wb_o_dat(dcache_wb_o_dat), .wb_adr(dcache_wb_adr), .wb_sel(dcache_wb_sel), .mem_cache_enable(dcache_mem_cache_enable), .wb_4_burst(dcache_wb_4_burst), .wb_err(dcache_wb_err), .mem_exception(dcache_mem_exception)
);

// ICACHES

wire ic0_clk, ic0_rst;
wire ic0_mem_req;
wire ic0_mem_ack;
wire [`RW-1:0] ic0_mem_addr;
wire [`I_SIZE-1:0] ic0_mem_data;
wire ic0_mem_ppl_submit;
wire ic0_mem_cache_flush;
wire ic0_wb_cyc;
wire ic0_wb_stb;
wire [`RW-1:0] ic0_wb_i_dat;
wire [`RW-1:0]  ic0_wb_adr;
wire ic0_wb_we;
wire ic0_wb_ack;
wire [1:0] ic0_wb_sel;
wire ic0_wb_err;

icache icache_0 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(ic0_clk), .i_rst(ic0_rst), .mem_req(ic0_mem_req), .mem_addr(ic0_mem_addr), .mem_data(ic0_mem_data), .mem_ack(ic0_mem_ack), .mem_ppl_submit(ic0_mem_ppl_submit),
    .wb_cyc(ic0_wb_cyc), .wb_stb(ic0_wb_stb), .wb_we(ic0_wb_we), .wb_ack(ic0_wb_ack), .wb_i_dat(ic0_wb_i_dat), .wb_adr(ic0_wb_adr), .wb_sel(ic0_wb_sel), .mem_cache_flush(ic0_mem_cache_flush), .wb_err(ic0_wb_err)
);

wire ic1_clk, ic1_rst;
wire ic1_mem_req;
wire ic1_mem_ack;
wire [`RW-1:0] ic1_mem_addr;
wire [`I_SIZE-1:0] ic1_mem_data;
wire ic1_mem_ppl_submit;
wire ic1_mem_cache_flush;
wire ic1_wb_cyc;
wire ic1_wb_stb;
wire [`RW-1:0] ic1_wb_i_dat;
wire [`RW-1:0]  ic1_wb_adr;
wire ic1_wb_we;
wire ic1_wb_ack;
wire [1:0] ic1_wb_sel;
wire ic1_wb_err;

icache icache_1 (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(ic1_clk), .i_rst(ic1_rst), .mem_req(ic1_mem_req), .mem_addr(ic1_mem_addr), .mem_data(ic1_mem_data), .mem_ack(ic1_mem_ack), .mem_ppl_submit(ic1_mem_ppl_submit),
    .wb_cyc(ic1_wb_cyc), .wb_stb(ic1_wb_stb), .wb_we(ic1_wb_we), .wb_ack(ic1_wb_ack), .wb_i_dat(ic1_wb_i_dat), .wb_adr(ic1_wb_adr), .wb_sel(ic1_wb_sel), .mem_cache_flush(ic1_mem_cache_flush), .wb_err(ic1_wb_err)
);

// Internal ram
wire iram_clk;
wire [8:0] iram_addr;
wire [`RW-1:0] iram_i_data, iram_o_data;
wire iram_we;
int_ram int_ram (
`ifdef USE_POWER_PINS
    .vccd1(vccd1), .vssd1(vssd1),
`endif
    .i_clk(iram_clk),
    .i_addr(iram_addr),
    .i_data(iram_i_data),
    .o_data(iram_o_data),
    .i_we(iram_we)
);

`ifdef TOP_DEBUG
    assign dbg_r0 = c0_dbg_r0;
    assign dbg_pc = c0_dbg_pc;
`endif

endmodule
/////////////////////////////////////////////////////////
module cpu_ram (
	address,
	byteena,
	clock,
	data,
	wren,
	q);

	input	[15:0]  address;
	input	[1:0]  byteena;
	input	  clock;
	input	[15:0]  data;
	input	  wren;
	output	[15:0]  q;

reg [7:0] mem0 [65535:0];
reg [7:0] mem1 [65535:0];

reg [7:0] o_data0;
reg [7:0] o_data1;

assign q = {o_data1, o_data0};

always @(posedge clock) begin
    if(wren & byteena[0])
        mem0[address] <= data[7:0];
    o_data0 <= mem0[address];
end

always @(posedge clock) begin
    if(wren & byteena[1])
        mem1[address] <= data[15:8];
    o_data1 <= mem1[address];
end

endmodule


module timer (
    input i_clk,
    input i_rst,
    output reg irq,

    input [23:0] wb_adr,
    input wb_cyc,
    input wb_stb,
    output wb_ack,
    input wb_we,
    input [15:0] wb_i_dat,
    output [15:0] wb_o_dat
);

reg [15:0] timer_cnt;

reg [15:0] pre_div_cnt;
reg [3:0] clk_div;

reg [15:0] reset_val;

always @(posedge i_clk) begin
    if(i_rst) begin
        timer_cnt <= 16'b0;
        pre_div_cnt <= 16'b0;
    end else if(wb_cyc & wb_stb & wb_we & (wb_adr == 2'b0)) begin
        timer_cnt <= wb_i_dat;
    end else begin
        pre_div_cnt <= pre_div_cnt + 16'b1;
        
        if(pre_div_cnt >= (1<<clk_div) - 16'b1) begin
            timer_cnt <= timer_cnt + 16'b1;
            pre_div_cnt <= 16'b0;

            if(timer_cnt == 16'hffff)
                timer_cnt <= reset_val;
        end
    end
end

always @(posedge i_clk) begin
    irq <= (timer_cnt == 16'hffff);
end

always @(posedge i_clk) begin
    if(i_rst) begin
        reset_val <= 16'b0;
        clk_div <= 4'b0;
    end else if(wb_cyc & wb_stb & wb_we & (wb_adr == 2'b1)) begin
        clk_div <= wb_i_dat[3:0];
    end else if(wb_cyc & wb_stb & wb_we & (wb_adr == 2'b10)) begin
        reset_val <= wb_i_dat;
    end
end

assign wb_o_dat = timer_cnt;
assign wb_ack = (wb_cyc & wb_stb);

endmodule
