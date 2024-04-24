module sqrt_iterative(
    input [15:0] num,   // Input number
    output reg [7:0] root,  // Output square root
    input clk,
    input rst_n
);
    reg [15:0] x_n;    // Current estimate
    reg [15:0] x_next; // Next estimate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_n <= num;
            root <= 0;
        end else begin
            x_next <= (x_n + num / x_n) >> 1; // Update estimate
            if (x_next == x_n)               // Convergence check
                root <= x_n[7:0];            // Output result
            else
                x_n <= x_next;
        end
    end
endmodule
