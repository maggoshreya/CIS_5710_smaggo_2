`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1 (
    input  wire a,
    b,
    output wire g,
    p
);
  assign g = a & b;
  assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4 (
    input wire [3:0] gin, pin,
    input wire cin,
    output wire gout, pout,
    output wire [2:0] cout
);

  /**************************************************
  * Local wire/variable instantiation
  *
  * NOTE: This is only for readability, not 
  * performance or optimization 
  ***************************************************/
  logic g3, g2, g1, g0;
  logic p3, p2, p1, p0;

  assign g3 = gin[3];
  assign g2 = gin[2];
  assign g1 = gin[1];
  assign g0 = gin[0];

  assign p3 = pin[3];
  assign p2 = pin[2];
  assign p1 = pin[1];
  assign p0 = pin[0];

  /* C1 */
  assign cout[0] = g0 | (p0 & cin);

  /* C2 */
  assign cout[1] = g1 | (p1 & g0) | (p1 & p0 & cin);

  /* C3 */
  assign cout[2] = g2 | (p2 & g1) | (p2 & p1 & g0) | (p2 & p1 & p0 & cin);

  /* gout */
  assign gout = (g3 | g2) | (p3 & p2) & (g1 | g0);

  /* pout */ 
  assign pout = p0 & p1 & p2 & p3;

endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8 (
    input wire [7:0] gin, pin,
    input wire cin,
    output wire gout, pout,
    output wire [6:0] cout,
    output wire next_carry
);

  // TODO: your code here

  /**************************************************
  * Local wire/variable instantiation
  *
  * NOTE: This is only for readability, not 
  * performance or optimization 
  ***************************************************/
  logic g7, g6, g5, g4, g3, g2, g1, g0;
  logic p7, p6, p5, p4, p3, p2, p1, p0;

  assign g7 = gin[7];
  assign g6 = gin[6];
  assign g5 = gin[5];
  assign g4 = gin[4];
  assign g3 = gin[3];
  assign g2 = gin[2];
  assign g1 = gin[1];
  assign g0 = gin[0];

  assign p7 = pin[7];
  assign p6 = pin[6];
  assign p5 = pin[5];
  assign p4 = pin[4];
  assign p3 = pin[3];
  assign p2 = pin[2];
  assign p1 = pin[1];
  assign p0 = pin[0];

  /* C1 */
  assign cout[0] = g0 | (p0 & cin);

  /* C2 */
  assign cout[1] = g1 | (p1 & g0) | (p1 & p0 & cin);

  /* C3 */
  assign cout[2] = g2 | (p2 & g1) | (p2 & p1 & g0) | (p2 & p1 & p0 & cin);

  /* C4 */
  assign cout[3] = g3 | (p3 & g2) | (p3 & p2 & g1) | (p3 & p2 & p1 & g0) | (p3 & p2 & p1 & p0 & cin);

  /* C5 */
  assign cout[4] = g4 | (p4 & g3) | (p4 & p3 & g2) | (p4 & p3 & p2 & g1) | (p4 & p3 & p2 & p1 & g0) | (p4 & p3 & p2 & p1 & p0 & cin);

  /* C6 */
  assign cout[5] = g5 | (p5 & g4) | (p5 & p4 & g3) | (p5 & p4 & p3 & g2) | (p5 & p4 & p3 & p2 & g1) | (p5 & p4 & p3 & p2 & p1 & g0) | (p5 & p4 & p3 & p2 & p1 & p0 & cin);

  /* C7 */
  assign cout[6] = g6 | (p6 & g5) | (p6 & p5 & g4) | (p6 & p5 & p4 & g3) | (p6 & p5 & p4 & p3 & g2) | (p6 & p5 & p4 & p3 & p2 & g1) | (p6 & p5 & p4 & p3 & p2 & p1 & g0) | (p6 & p5 & p4 & p3 & p2 & p1 & p0 & cin);

  /* gout */
  assign gout = (g3 | g2) | (p3 & p2) & (g1 | g0);

  /* pout */ 
  assign pout = p0 & p1 & p2 & p3;

  /* Next carry C8x */
  assign next_carry = g7 | (p7 & cout[6]);

endmodule

module cla (
    input  wire [31:0] a,
    b,
    input  wire        cin,
    output wire [31:0] sum
);

  // TODO: your code here
  logic [31:0] l1_gen_store;
  logic [31:0] l1_prop_store;
  logic [32:0] l2_gen_store;
  logic [32:0] l2_prop_store;
  logic [31:0] carry_outs;

  logic [31:0] gp4_carry_outs;
  logic [31:0] gp4_gout; 
  logic [31:0] gp4_pout;

  /* Individual carry out wires */
  logic [31:0] carry_ins;
  logic c8;
  logic c16;
  logic c24;
  logic c32;
  /* Loop variables */
  genvar i;
  // genvar j;

  for(i = 0; i<32; i++) begin
    gp1 gp1_inst_n(
      .a(a[i]),
      .b(b[i]),
      .g(l1_gen_store[i]),
      .p(l1_prop_store[i])
    );
  end

  gp8 gp8_inst_0(
    .gin(l1_gen_store[7:0]),
    .pin(l1_prop_store[7:0]),
    .cin(cin),
    .gout(l2_gen_store[8]),
    .pout(l2_prop_store[8]),
    .cout(carry_outs[7:1]),
    .next_carry(c8)
  );

  gp8 gp8_inst_1(
    .gin(l1_gen_store[15:8]),
    .pin(l1_prop_store[15:8]),
    .cin(c8),
    .gout(l2_gen_store[16]),
    .pout(l2_prop_store[16]),
    .cout(carry_outs[15:9]),
    .next_carry(c16)
  );

  gp8 gp8_inst_2(
    .gin(l1_gen_store[23:16]),
    .pin(l1_prop_store[23:16]),
    .cin(c16),
    .gout(l2_gen_store[24]),
    .pout(l2_prop_store[24]),
    .cout(carry_outs[23:17]),
    .next_carry(c24)
  );

  gp8 gp8_inst_3(
    .gin(l1_gen_store[31:24]),
    .pin(l1_prop_store[31:24]),
    .cin(c24),
    .gout(l2_gen_store[32]),
    .pout(l2_prop_store[32]),
    .cout(carry_outs[31:25]),
    .next_carry(c32)
  );

  assign carry_outs[8] = c8;
  assign carry_outs[16] = c16;
  assign carry_outs[24] = c24;
  assign sum = (carry_outs | {31'b0, cin}) ^ a ^ b;

  // always_comb begin
  //   $display("g7-0: 0b%b | p7-0: 0b%b", l1_gen_store[7:0], l1_prop_store[7:0]);
  //   $display("sum_value: 0b%b | carry_outs: 0b%b | c8: %d | c16: %d | c24: %d", sum, carry_outs, c8, c16, c24);
  // end 


endmodule


