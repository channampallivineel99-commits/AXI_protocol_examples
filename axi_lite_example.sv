// AXI4-Lite Subordinate: Up/Down Counter Controller
//
// Register Map (word-aligned):
//   0x00  INITIAL_VALUE  (R/W) - Counter starting value
//   0x04  FINAL_VALUE    (R/W) - Counter ending value
//   0x08  CONTROL        (R/W) - bit[0]: start (write 1 to launch, self-clearing)
//                                 bit[1]: direction (0 = count up, 1 = count down)
//   0x0C  STATUS         (R)   - bit[0]: done (counter reached final value)
//   0x10  INCREMENT      (R/W) - Step size per clock cycle

module axi4lite_counter #(
    parameter ADDR_WIDTH = 5,
    parameter DATA_WIDTH = 32
)(
    input  wire                      ACLK,
    input  wire                      ARESETN,

    // Write Address Channel
    input  wire [ADDR_WIDTH-1:0]     AWADDR,
    input  wire                      AWVALID,
    output reg                       AWREADY,

    // Write Data Channel
    input  wire [DATA_WIDTH-1:0]     WDATA,
    input  wire                      WVALID,
    output reg                       WREADY,

    // Write Response Channel
    output reg  [1:0]                BRESP,
    output reg                       BVALID,
    input  wire                      BREADY,

    // Read Address Channel
    input  wire [ADDR_WIDTH-1:0]     ARADDR,
    input  wire                      ARVALID,
    output reg                       ARREADY,

    // Read Data Channel
    output reg  [DATA_WIDTH-1:0]     RDATA,
    output reg  [1:0]                RRESP,
    output reg                       RVALID,
    input  wire                      RREADY,

    // Counter output / interrupt
    output wire [DATA_WIDTH-1:0]     count_out,
    output wire                      done_irq
);

    // ----------------------------------------------------------------
    // Register file
    // ----------------------------------------------------------------
    reg [DATA_WIDTH-1:0] initial_value_reg;  // 0x00
    reg [DATA_WIDTH-1:0] final_value_reg;    // 0x04
    reg [DATA_WIDTH-1:0] control_reg;        // 0x08
    reg [DATA_WIDTH-1:0] status_reg;         // 0x0C (read-only)
    reg [DATA_WIDTH-1:0] increment_reg;      // 0x10
    reg [DATA_WIDTH-1:0] counter;            // internal counter (not memory-mapped)

    // Control-register bit fields
    wire start     = control_reg[0];
    wire direction = control_reg[1];  // 0 = up, 1 = down

    // Status-register bit fields
    wire done = status_reg[0];

    assign count_out = counter;
    assign done_irq  = done;

    // ----------------------------------------------------------------
    // Latched write address (needed because AW and W may arrive in
    // different cycles)
    // ----------------------------------------------------------------
    reg [ADDR_WIDTH-1:0] awaddr_latched;
    reg                  aw_received;
    reg                  w_received;

    // ----------------------------------------------------------------
    // Write Channel Logic
    // ----------------------------------------------------------------
    always @(posedge ACLK) begin
        if (!ARESETN) begin
            AWREADY        <= 0;
            WREADY         <= 0;
            BVALID         <= 0;
            BRESP          <= 2'b00;
            aw_received    <= 0;
            w_received     <= 0;
            awaddr_latched <= 0;
        end else begin
            // Default deassert
            AWREADY <= 0;
            WREADY  <= 0;

            // Capture write address
            if (AWVALID && !aw_received) begin
                AWREADY        <= 1;
                awaddr_latched <= AWADDR;
                aw_received    <= 1;
            end

            // Capture write data
            if (WVALID && !w_received) begin
                WREADY     <= 1;
                w_received <= 1;
            end

            // Both address and data received – issue response
            if (aw_received && w_received && !BVALID) begin
                BVALID <= 1;
                BRESP  <= 2'b00; // OKAY
                aw_received <= 0;
                w_received  <= 0;
            end else if (BVALID && BREADY) begin
                BVALID <= 0;
            end
        end
    end

    // ----------------------------------------------------------------
    // Register Writes
    // ----------------------------------------------------------------
    always @(posedge ACLK) begin
        if (!ARESETN) begin
            initial_value_reg <= 0;
            final_value_reg   <= 0;
            control_reg       <= 0;
            increment_reg     <= 1;  // default step size = 1
        end else begin
            // Self-clear the start bit once the counter begins running
            if (start && running)
                control_reg[0] <= 0;

            // Register writes happen when both AW and W are captured
            if (aw_received && w_received && !BVALID) begin
                case (awaddr_latched)
                    5'h00: initial_value_reg <= WDATA;
                    5'h04: final_value_reg   <= WDATA;
                    5'h08: control_reg       <= WDATA;
                    5'h10: increment_reg     <= WDATA;
                    default: ; // writes to read-only / invalid addresses ignored
                endcase
            end
        end
    end

    // ----------------------------------------------------------------
    // Read Channel Logic
    // ----------------------------------------------------------------
    always @(posedge ACLK) begin
        if (!ARESETN) begin
            ARREADY <= 0;
            RVALID  <= 0;
            RRESP   <= 2'b00;
            RDATA   <= 0;
        end else begin
            ARREADY <= 0;

            if (ARVALID && !RVALID) begin
                ARREADY <= 1;
                RVALID  <= 1;
                RRESP   <= 2'b00; // OKAY
                case (ARADDR)
                    5'h00:   RDATA <= initial_value_reg;
                    5'h04:   RDATA <= final_value_reg;
                    5'h08:   RDATA <= control_reg;
                    5'h0C:   RDATA <= status_reg;
                    5'h10:   RDATA <= increment_reg;
                    default: RDATA <= 32'hDEAD_BEEF;
                endcase
            end else if (RVALID && RREADY) begin
                RVALID <= 0;
            end
        end
    end

    // ----------------------------------------------------------------
    // Up / Down Counter FSM
    // ----------------------------------------------------------------
    reg running;

    always @(posedge ACLK) begin
        if (!ARESETN) begin
            counter    <= 0;
            status_reg <= 0;
            running    <= 0;
        end else begin
            if (start && !running) begin
                // Latch initial value and begin counting
                counter       <= initial_value_reg;
                status_reg[0] <= 0;   // clear done
                running       <= 1;
            end else if (running) begin
                if (counter == final_value_reg) begin
                    // Reached target – stop and flag done
                    status_reg[0] <= 1;
                    running       <= 0;
                end else begin
                    if (!direction)
                        counter <= counter + increment_reg;
                    else
                        counter <= counter - increment_reg;
                end
            end
        end
    end

endmodule