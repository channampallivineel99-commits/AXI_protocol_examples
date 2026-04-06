// Simple AXI-Stream Station with Memory Buffer
// Accepts AXI-Stream input, buffers in FIFO, outputs via AXI-Stream

module axis_station_fifo #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH = 16,  // Memory buffer depth
    parameter ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                  ACLK,
    input  wire                  ARESETN,

    // AXI-Stream Input (write into FIFO)
    input  wire [DATA_WIDTH-1:0] S_AXIS_TDATA,
    input  wire                  S_AXIS_TVALID,
    output reg                   S_AXIS_TREADY,
    input  wire                  S_AXIS_TLAST,

    // AXI-Stream Output (read from FIFO)
    output reg  [DATA_WIDTH-1:0] M_AXIS_TDATA,
    output reg                   M_AXIS_TVALID,
    input  wire                  M_AXIS_TREADY,
    output reg                   M_AXIS_TLAST,

    // Status signals
    output reg                   fifo_full,
    output reg                   fifo_empty,
    output reg [ADDR_WIDTH:0]    fifo_count  // Count of items in FIFO
);

    // Memory buffer
    reg [DATA_WIDTH-1:0] mem       [0:DEPTH-1];
    reg                  mem_last  [0:DEPTH-1];  // tracks TLAST per entry
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   count;

    // Handshake helpers
    wire write_en = S_AXIS_TVALID && S_AXIS_TREADY;
    wire read_en  = M_AXIS_TVALID && M_AXIS_TREADY;

    // Combinational status
    always @(*) begin
        fifo_full     = (count == DEPTH);
        fifo_empty    = (count == 0);
        fifo_count    = count;
        S_AXIS_TREADY = !fifo_full;
    end

    // Sequential: write side, read side, pointer management
    always @(posedge ACLK or negedge ARESETN) begin
        if (!ARESETN) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
        end else begin
            case ({write_en, read_en})
                2'b10: begin  // write only
                    mem[wr_ptr]      <= S_AXIS_TDATA;
                    mem_last[wr_ptr] <= S_AXIS_TLAST;
                    wr_ptr           <= wr_ptr + 1;
                    count            <= count + 1;
                end
                2'b01: begin  // read only
                    rd_ptr <= rd_ptr + 1;
                    count  <= count - 1;
                end
                2'b11: begin  // simultaneous read and write
                    mem[wr_ptr]      <= S_AXIS_TDATA;
                    mem_last[wr_ptr] <= S_AXIS_TLAST;
                    wr_ptr           <= wr_ptr + 1;
                    rd_ptr           <= rd_ptr + 1;
                    // count stays the same
                end
                default: ;
            endcase
        end
    end

    // M_AXIS output: drive data from read pointer
    always @(posedge ACLK or negedge ARESETN) begin
        if (!ARESETN) begin
            M_AXIS_TDATA  <= 0;
            M_AXIS_TVALID <= 0;
            M_AXIS_TLAST  <= 0;
        end else begin
            if (!fifo_empty) begin
                M_AXIS_TDATA  <= mem[rd_ptr];
                M_AXIS_TLAST  <= mem_last[rd_ptr];
                M_AXIS_TVALID <= 1;
            end else begin
                M_AXIS_TVALID <= 0;
            end
        end
    end

endmodule
