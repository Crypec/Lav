use ux::{u11, u4};

// NOTE(Simon): https://www.win.tue.nl/~aeb/comp/8051/set8051.html#51dec

pub type Programm = Vec<Segment>;

// NOTE(Simon): this may very well be a u32 I don't know!
pub type Offset = u16;

pub type Const = u8;
pub type Addr = u16;
pub type Addr16 = u16;
pub type Reg = u4;
pub type Addr11 = u11;

#[derive(Debug)]
pub struct Segment {
    pub name: String,
    pub prog: Vec<Instr>,
}

#[derive(Debug)]
pub struct Instr {
    pub op_code: u16,
    pub kind: InstrKind,
}

#[derive(Debug, Copy, Clone)]
pub enum InstrKind {
    /// The ACALL instruction calls a subroutine located at the specified address.
    /// The PC is incremented twice to obtain the address of the following instruction.
    /// The 16-bit PC is then stored on the stack (low-order byte first) and the stack pointer
    /// is incremented twice. No flags are affected.
    ///
    /// The address of the subroutine is calculated by combining the 5 high-order bits of
    /// the incremented PC (for A15-A11), the 3 high-order bits of the ACALL instruction opcode (for A10-A8),
    /// and the second byte of the instruction (for A7-A0). The subroutine that is called must
    /// be located in the same 2KByte block of program memory as the opcode following the ACALL instruction.
    ACALL(Addr11),

    /// The ADD instruction adds a byte value to the accumulator and stores the results back in the accumulator.
    /// Several of the flag registers are affected.
    ADD(AddKind),

    /// The ADDC instruction adds a byte value and the value of the carry flag to the accumulator.
    /// The results of the addition are stored back in the accumulator. Several of the flag registers are affected.
    ADDC(AddcKind),

    /// The AJMP instruction transfers program execution to the specified address.
    /// The address is formed by combining the 5 high-order bits of the address of the following instruction (for A15-A11),
    /// the 3 high-order bits of the opcode (for A10-A8), and the second byte of the instruction (for A7-A0).
    /// The destination address must be located in the same 2KByte block of program memory as the opcode following the AJMP instruction.
    /// No flags are affected.
    ///
    /// Operation:
    ///           PC = PC + 2
    ///           PC_10-0 = A_10-0
    ///Example: AJMP LABEL
    AJMP(Addr11),

    /// The ANL instruction performs a bitwise logical AND operation between the specified byte or bit operands and
    /// stores the result in the destination operand.
    /// Note: When this instruction is used to modify an output port, the value used as the port data will be read from the output data latch,
    /// not the input pins of the port.
    ANL(AnlKind),

    /// The CJNE instruction compares the first two operands and branches to the specified destination if their values are not equal.
    /// If the values are the same, execution continues with the next instruction.
    CJNE(CjneKind),

    /// The CLR instruction sets the specified destination operand to a value of 0.
    CLR(ClrKind),

    /// The CPL instruction logically complements the value of the specified destination operand and stores the result back in the destination operand.
    /// Bits that previously contained a 1 will be changed to a 0 and bits that previously contained a 0 will be changed to a 1.
    CPL(CplKind),

    /// The DA instruction adjusts the eight-bit value in the Accumulator resulting from the earlier addition of two variables (each in packed-BCD format), producing two four-bit digits.
    /// Any ADD or ADDC instruction may have been used to perform the addition.
    /// If Accumulator bits 3-0 are greater than nine (xxx1010-xxx1111), or if the AC flag is one, six is added to the Accumulator,
    /// producing the proper BCD digit in the low-order nibble.
    //// This internal addition would set the carry flag if a carry-out of the low-order four-bit field propagated through
    /// all high-order bits, but it would not clear the carry flag otherwise.
    /// If the carry flag is now set, or if the four high-order bits now exceed nine (1010xxx-111xxxx), these high-order bits are incremented by six, producing the proper BCD digit in the high-order nibble.
    /// Again, this would set the carry flag if there was a carry-out of the high-order bits, but would not clear the carry.
    /// The carry flag thus indicates if the sum of the original two BCD variables is greater than 100, allowing multiple precision decimal addition. OV is not affected.
    /// All of this occurs during the one instruction cycle.
    /// Essentially, this instruction performs the decimal conversion by adding 00H, 06H, 60H, or 66H to the Accumulator, depending on initial Accumulator and PSW conditions.
    ///
    /// Operation:
    ///           IF (A3-0 > 9) OR (AC = 1)
    ///               A = A + 6
    ///           IF (A7-4 > 9) OR (C = 1)
    ///               A = A + 60h
    /// Example: DA A
    DA,

    /// The DEC instruction decrements the specified operand by 1.
    /// An original value of 00h underflows to 0FFh. No flags are affected by this instruction.
    /// Note: When this instruction is used to modify an output port, the value used as the port data is read from the output data latch, not the pins of the port.
    DEC(DecKind),

    /// The DIV instuction divides the unsigned 8-bit integer in the accumulator by the unsigned 8-bit integer in register B.
    /// After the division, the quotient is stored in the accumulator and the remainder is stored in the B register.
    /// The carry and OV flags are cleared.
    /// If the B register begins with a value of 00h the division operation is undefined, the values of the accumulator and B register are undefined after the division, and the OV flag will be set indicating a division-by-zero error.
    ///
    /// Operation: AB = A / B
    /// Example: DIV AB
    DIV,

    /// The DJNZ instruction decrements the byte indicated by the first operand and, if the resulting value is not zero, branches to the address specified in the second operand.
    /// Note: When this instruction is used to modify an output port, the value used as the port data is read from the output data latch, not the input pins of the port.
    DJNZ(DjnzKind),

    /// The INC instruction increments the specified operand by 1.
    /// An original value of 0FFh or 0FFFFh overflows to 00h or 0000h. No flags are affected by this instruction.
    /// Note: When this instruction is used to modify an output port, the value used as the port data is read from the output data latch, not the input pins of the port.
    INC(IncKind),

    /// The JB instruction branches to the address specified in the second operand if the value of the bit specified in the first operand is 1.
    /// The bit that is tested is not modified. No flags are affected by this instruction.
    ///
    /// Operation:
    /// PC = PC + 3
    /// IF (bit) = 1
    ///     PC = PC + offset
    /// Example: JB P1.2 LABEL
    JB,

    /// The JBC instruction branches to the address specified in the second operand if the value of the bit specified in the first operand is 1.Otherwise, execution continues with the next instruction.
    /// If the bit specified in the first operand is set, it is cleared. No flags are affected by this instruction.
    /// Note: When this instruction is used to modify an output port, the value used as the port data is read from the output data latch, not the input pins of the port.
    ///
    /// Operation:
    /// PC = PC + 3
    /// IF (bit) = 1
    ///   (bit) = 0
    ///   PC = PC + offset
    /// Example: JBC 44h
    JBC,

    /// The JC instruction branches to the specified address if the carry flag is set.
    /// Otherwise, execution continues with the next instruction. No flags are affected by this instruction.
    ///
    /// Operation:
    /// PC = PC + 2
    /// IF C = 1
    ///   PC = PC + offset
    /// Example: JC LABEL
    JC,

    /// The JMP instruction transfers execution to the address generated by adding the 8-bit value in the accumulator to the 16-bit value in the DPTR register.
    /// Neither the accumulator nor the DPTR register are altered. No flags are affected by this instruction.
    ///
    /// Operation: PC = A + DPTR
    /// Example: JMP @A+DPTR
    JMP,

    /// The JNB instruction branches to the specified address if the specified bit operand has a value of 0.
    /// Otherwise, execution continues with the next instruction. No flags are affected by this instruction.
    ///
    /// Operation:
    /// PC = PC + 3
    /// IF (bit) = 0
    ///   PC = PC + offset
    /// Example: JNB P1.3, LABEL
    JNB,

    /// The JNC instruction transfers program control to the specified address if the carry flag is 0.
    /// Otherwise, execution continues with the next instruction. No flags are affected by this instruction.
    ///
    /// Operation:
    /// PC = PC + 2
    /// IF C = 0
    ///   PC = PC + offset
    /// Example: JNC LABEL
    JNC,

    /// The JNZ instruction transfers control to the specified address if the value in the accumulator is not 0.
    /// If the accumulator has a value of 0, the next instruction is executed. Neither the accumulator nor any flags are modified by this instruction.
    ///
    /// Operation:
    /// PC = PC + 2
    /// IF A <> 0
    ///   PC = PC + offset
    /// Example: JNZ LABEL
    JNZ,

    /// The JZ instruction transfers control to the specified address if the value in the accumulator is 0.
    /// Otherwise, the next instruction is executed. Neither the accumulator nor any flags are modified by this instruction.
    ///
    /// Operation:
    /// PC = PC + 2
    /// IF A = 0
    ///   PC = PC + offset
    /// Example: JZ LABEL
    JZ,

    /// The LCALL instruction calls a subroutine located at the specified address.
    /// This instruction first adds 3 to the PC to generate the address of the next instruction.
    /// This result is pushed onto the stack low-byte first and the stack pointer is incremented by 2. The high-order and low-order bytes of the PC are loaded from the second and third bytes of the instruction respectively.
    /// Program execution is transferred to the subroutine at this address. No flags are affected by this instruction.
    ///
    /// Operation:
    /// PC = PC + 3
    /// SP = SP + 1
    /// (SP) = PC[7-0]
    /// SP = SP + 1
    /// (SP) = PC[15-8]
    /// PC = addr16
    /// Example: LCALL SUB1
    LCALL,

    /// The LJMP instruction transfers program execution to the specified 16-bit address.
    /// The PC is loaded with the high-order and low-order bytes of the address from the second and third bytes of this instruction respectively.
    /// No flags are affected by this instruction.
    ///
    /// Operation: PC = addr16
    /// Example: LJMP LABEL
    LJMP(Addr16),

    /// The MOV instruction moves data bytes between the two specified operands.
    /// The byte specified by the second operand is copied to the location specified by the first operand. The source data byte is not affected.
    MOV(MovKind),

    /// The MOVC instruction moves a byte from the code or program memory to the accumulator
    MOVC(MovcKind),

    /// The MOVX instruction transfers data between the accumulator and external data memory.
    /// External memory may be addressed via 16-bits in the DPTR register or via 8-bits in the R0 or R1 registers.
    /// When using 8-bit addressing, Port 2 must contain the high-order byte of the address.
    MOVX(MovxKind),

    /// The MUL instruction multiplies the unsigned 8-bit integer in the accumulator and the unsigned 8-bit integer in the B register producing a 16-bit product.
    /// The low-order byte of the product is returned in the accumulator.
    /// The high-order byte of the product is returned in the B register.
    /// The OV flag is set if the product is greater than 255 (0FFh), otherwise it is cleared.
    /// The carry flag is always cleared.
    ///
    /// Operation: BA = A * B
    /// Example: MUL AB
    MUL,
    /// The NOP instruction does nothing. Execution continues with the next instruction.
    /// No registers or flags are affected by this instruction.
    /// NOP is typically used to generate a delay in execution or to reserve space in code memory.
    ///
    /// Operation: PC = PC + 1
    /// Example: NOP
    NOP,

    /// The ORL instruction performs a bitwise logical OR operation on the specified operands, the result of which is stored in the destination operand.
    /// Note: When this instruction is used to modify an output port, the value used as the port data will be read from the output data latch, not the input pins of the port.
    ORL(OrlKind),
    /// The POP instruction reads a byte from the address indirectly referenced by the SP register.
    /// The value read is stored at the specified address and the stack pointer is decremented.
    /// No flags are affected by this instruction.
    ///
    /// Operation:
    /// (direct) = (SP)
    /// SP = SP - 1
    /// Example: POP 34h
    POP,

    /// The PUSH instruction increments the stack pointer and stores the value of the specified byte operand at the internal RAM address indirectly referenced by the stack pointer.
    /// No flags are affected by this instruction.
    ///
    /// Operation:
    /// SP = SP + 1
    /// (SP) = (direct)
    /// Example: PUSH A
    PUSH,

    /// The RET instruction pops the high-order and low-order bytes of the PC from the stack (and decrements the stack pointer by 2).
    /// Program execution resumes from the resulting address which is typically the instruction following an ACALL or LCALL instruction.
    /// No flags are affected by this instruction.
    ///
    /// Operation:
    /// PC15-8 = (SP)
    /// SP = SP - 1
    /// PC7-0 = (SP)
    /// SP = SP - 1
    /// Example: RET
    RET,
    /// The RETI instruction is used to end an interrupt service routine.
    /// This instruction pops the high-order and low-order bytes of the PC (and decrements the stack pointer by 2) and restores the interrput logic to accept additional interrupts. No other registers are affected by this instruction.

    /// The RETI instruction does not restore the PSW to its value before the interrupt.
    /// The interrupt service routine must save and restore the PSW.

    /// Execution returns to the instruction immediately after the point at which the interrupt was detected.
    /// If another interrupt was pending when the RETI instruction is executed, one instruction at the return address is executed before the pending interrupt is processed.
    ///
    /// Operation:
    /// PC15-8 = (SP)
    /// SP = SP - 1
    /// PC7-0 = (SP)
    /// SP = SP - 1
    /// Example: RETI
    RETI,

    /// The RL instruction rotates the eight bits in the accumulator left one bit position.
    /// Bit 7 of the accumulator is rotated into bit 0, bit 0 into bit 1, bit 1 into bit 2, and so on.
    /// No flags are affected by this instruction.
    ///
    /// Operation:
    /// An+1 = An WHERE n = 0 TO 6
    /// A0 = A7
    /// Example: RL A
    RL,

    /// The RLC instruction rotates the eight bits in the accumulator and the one bit in the carry flag left one bit position.
    /// Bit 7 of the accumulator is rotated into the carry flag while the original value of the carry flag is rotated into bit 0 of the accumulator.
    /// Bit 0 of the accumulator is rotated into bit 1, bit 1 into bit 2, and so on.
    /// No other flags are affected by this operation.
    ///
    /// Operation:
    /// An+1 = AN WHERE N = 0 TO 6
    /// A0 = C
    /// C = A7
    /// Example: RLC A
    RLC,

    /// The RR instruction rotates the eight bits in the accumulator right one bit position.
    /// Bit 0 of the accumulator is rotated into bit 7, bit 7 into bit 6, and so on.
    /// No flags are affected by this instruction.
    ///
    /// Operation:
    /// An = An+1 where n = 0 to 6
    /// A7 = A0
    /// Example: PR A
    PR,

    /// The RRC instruction rotates the eight bits in the accumulator and the one bit in the carry flag right one bit position.
    /// Bit 0 of the accumulator is rotated into the carry flag while the original value of the carry flag is rotated in to bit 7 of the accumulator.
    /// Bit 7 of the accumulator is rotated into bit 6, bit 6 into bit 5, and so on.
    /// No other flags are affected by this instruction.
    ///
    /// Operation:
    /// An = An+1 where n = 0 to 6
    /// A7 = C
    /// C = A0
    /// Example: RRC A
    RRC,

    /// The SETB instruction sets the bit operand to a value of 1.
    /// This instruction can operate on the carry flag or any other directly addressable bit.
    /// No flags are affected by this instruction.
    SETB(SetbKind),

    /// The SJMP instruction transfers execution to the specified address.
    /// The address is calculated by adding the signed relative offset in the second byte of the instruction to the address of the following instruction.
    /// The range of destination addresses is from 128 before the next instruction to 127 bytes after the next instruction.
    ///
    /// Operation:
    /// PC = PC + 2
    /// PC = PC + offset
    /// Example: SJMP LABEL
    SJMP,

    /// The SUBB instruction subtracts the specified byte variable and the carry flag from the accumulator.
    /// The result is stored in the accumulator.
    /// This instruction sets the carry flag if a borrow is required for bit 7 of the result.
    /// If no borrow is required, the carry flag is cleared.
    SUBB(SubbKind),

    /// The SWAP instruction exchanges the low-order and high-order nibbles within the accumulator.
    /// No flags are affected by this instruction.
    ///
    /// Operation: A3-0 swap A7-4
    /// Example: SWAP A
    SWAP,

    /// The XCH instruction loads the accumulator with the byte value of the specified operand while simultaneously storing the previous contents of the accumulator in the specified operand.
    XCH(XchKind),

    /// The XCHD instruction exchanges the low-order nibble of the accumulator with the low-order nibble of the specified internal RAM location.
    /// The internal RAM is accessed indirectly through R0 or R1.
    /// The high-order nibbles of each operand are not affected.
    ///
    /// Operation: A3-0 swap (Ri)3-0
    /// Example: XCHD A, @R1
    XCHD,

    /// The XRL instruction performs a logical exclusive OR operation between the specified operands.
    /// The result is stored in the destination operand.
    XRL(XrlKind),
}

#[derive(Debug, Copy, Clone)]
pub enum AddKind {
    /// Operation: A = A + immediate
    /// Example: ADD A, #03h
    Const(Const),

    /// Operation: A = A + (Ri)
    /// Example: ADD A, @R1
    Addr(Addr),

    /// Operation: A = A + (direct)
    /// Example: ADD A, 20h
    Direct(Const),

    /// Operation: A = A + Rn
    /// Example: ADD A, R0
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum AddcKind {
    /// Operation: A = A + C + immediate
    /// Example: ADDC A, #23h
    Const(Const),

    /// Operation: A = A + C + (Ri)
    /// Example: ADDC A, @R0
    Addr(Addr),

    /// Operation: A = A + C + (direct)
    /// Example: ADDC A, 30h
    Direct(Const),

    /// Operation: A = A + C + Rn
    /// Example: ADDC A, R5
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum AnlKind {
    Accu(AnlAccu),
    Carry(AnlCarry),
    Direct(AnlDirect),
}

#[derive(Debug, Copy, Clone)]
pub enum AnlAccu {
    /// Operation: A = A AND immediate
    /// Example: ANL A, #3Fh
    Const(Const),

    /// Operation: A = A AND (Ri)
    /// Example ANL A, @R0
    Addr(Addr),

    /// Operation: A = A AND (direct)
    /// Example: ANL A, 40h
    Direct(Const),

    /// Operation: A = A AND Rn
    /// Example: ANL A, R4
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum AnlCarry {
    /// Operation: C = C AND NOT (bit)
    /// Example: ANL C, /22h
    NotConst(Const),

    /// Operation: C = C AND (bit)
    /// Example: ANL C, 22h
    Const(Const),
}

#[derive(Debug, Copy, Clone)]
pub enum AnlDirect {
    /// Operation: (direct) = (direct) AND A
    /// Example: ANL 40h, A
    Acc(Const),

    /// Operation: (direct) = (direct) AND immediate
    /// Example: ANL 30h, #77h
    Const(Const),
}

#[derive(Debug, Copy, Clone)]
pub enum CjneKind {
    /// Operation:
    /// PC = PC + 3
    /// IF (Rn) <> immedate
    ///   PC = PC + offset
    /// IF (Rn) < immediate
    ///   C = 1
    /// ELSE
    ///   C = 0
    /// Example: CJNE @R1, #24H, LABEL
    Addr(Addr, Const, Offset),

    /// Operation:
    /// PC = PC + 3
    /// IF A <> immediate
    ///   PC = PC + offset
    /// IF A < immediate
    ///   C = 1
    /// ELSE
    ///   C = 0
    /// Example: CJNE A, #01H, LABEL
    Const(Const, Offset),

    /// Operation:
    /// PC = PC + 3
    /// IF A <> (direct)
    ///   PC = PC + offset
    /// IF A < (direct)
    ///   C = 1
    /// ELSE
    ///   C = 0
    /// Example: CJNE A, 60h, LABEL
    Direct(Const, Offset),

    /// Operation:
    /// PC = PC + 3
    /// IF Rn <> immedate
    ///   PC = PC + offset
    /// IF Rn < immediate
    ///   C = 1
    /// ELSE
    ///   C = 0
    /// Example: CJNE R6, #12H, LABEL
    Reg(Reg, Const, Offset),
}

#[derive(Debug, Copy, Clone)]
pub enum ClrKind {
    /// Operation: A = 0
    /// Example: CLR A
    Acc,

    /// Operation: (bit)  = 0
    /// Example: CLR 01h
    Bit,

    /// Operation: C = 0
    /// Example: CLR C
    Carry,
}

#[derive(Debug, Copy, Clone)]
pub enum CplKind {
    /// Operation: A = NOT A
    /// Example: CPL A
    Acc,

    /// Operation: (bit) = NOT (bit)
    /// Example: CPL 55h
    Bit,

    /// Operation: C = NOT C
    /// Example: CPL C
    Carry,
}

#[derive(Debug, Copy, Clone)]
pub enum DecKind {
    /// Operation: (Ri) = (Ri) - 1
    /// Example: DEC @R1
    Addr(Addr),

    /// Operation: A = A - 1
    /// Example: DEC A
    Acc,

    /// Operation: (direct) = (direct) - 1
    /// Example: DEC 35h
    Direct(Const),

    /// Operation: Rn = Rn - 1
    /// Example: DEC R7
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum DjnzKind {
    /// Operation:
    /// PC = PC + 2
    /// (direct) = (direct) - 1
    /// IF (direct) <> 0
    ///   PC = PC + offset
    /// Example: DJNZ 40h, LABEL
    Direct(Const, Offset),

    /// Operation:
    /// PC = PC + 2
    /// Rn = Rn - 1
    /// IF Rn <> 0
    ///   PC = PC + offset
    /// Example: DJNZ R6, LABEL
    Reg(Reg, Offset),
}

#[derive(Debug, Copy, Clone)]
pub enum IncKind {
    /// Operation: (Ri) = (Ri) + 1
    /// Example: INC @R0
    Addr(Addr),

    /// Operation: A = A + 1
    /// Example: INC A
    Acc,

    /// Operation: (direct) = (direct) + 1
    /// Example: INC 34h
    Direct(Const),

    /// Operation: DPTR = DPTR + 1
    /// Example: INC DPTR
    DPTR,

    /// Operation: Rn = Rn + 1
    /// Example: INC Rn
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum MovKind {
    Addr(MovAddr),

    Acc(MovAcc),

    /// Operation: (bit) = C
    /// Example: MOV 22h, C
    Bit(Addr),

    /// Operation: C = (bit)
    /// Example: MOV C, 22h
    Carry(Addr),

    Direct(MovDirect),

    /// Operation: DPTR = immediate
    /// Example: MOV DPTR, #1234h
    DPTR(Const),

    Reg(MovReg),
}

#[derive(Debug, Copy, Clone)]
pub enum MovAddr {
    /// Operation: (Rn) = immediate
    /// Example: MOV @R0, #0
    Const(Addr, Const),

    /// Operation: (Ri) = A
    /// Example: MOV @R0, A
    Acc(Addr),

    // TODO(Simon): I'm not sure if this instrustion really takes a const as it's 2. operand?
    /// Operation: (Ri) = (direct)
    /// Example: MOV @R1, P2
    Direct(Addr, Const),
}

#[derive(Debug, Copy, Clone)]
pub enum MovAcc {
    /// Operation: A = immediate
    /// Example: MOV A, #0FFh
    Const(Const),

    /// Operation: A = (Ri)
    /// Example: MOV A, @R1
    Addr(Addr),

    /// Operation: A = (direct)
    /// Example: MOV A, P0
    Direct(Addr),

    /// Operation: A = Rn
    /// Example: MOV A, R6
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum MovDirect {
    /// Operation: (dest_direct) = (src_direct)
    /// Example: MOV P1, P0
    SrcDest(Addr, Addr),

    /// Operation: (direct) = immediate
    /// Example: MOV P2, #0FFh
    Const(Const),

    /// Operation: (direct) = A
    /// Example: MOV P0, A
    Acc(Addr),

    /// Operation: (direct) = Rn
    /// Example: MOV P2, R5
    Reg(Addr, Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum MovReg {
    /// Operation: Rn = immediate
    /// Example: MOV R4, #0h
    Const(Reg, Const),

    /// Operation: Rn = A
    /// Example: MOV R5, A
    Acc(Reg),

    /// Operation: Rn = (direct)
    /// Example: MOV R4, P1
    Direct(Addr),
}

#[derive(Debug, Copy, Clone)]
pub enum MovcKind {
    /// Operation: A = (A + DPTR)
    /// Example: MOVC A, @A+DPTR
    Acc,

    /// Operation:
    /// PC = PC + 1
    /// A = (A+PC)
    ///
    /// Example: MOV A, @A+PC
    Pc,
}

#[derive(Debug, Copy, Clone)]
pub enum MovxKind {
    /// Operation: (Ri) = A
    /// Example: MOVX @R0, A
    Direct(Addr),

    /// Operation: A = (DPTR)
    /// Example: MOVX A, @DPTR
    DPTR,

    /// Operation: A = (Ri)
    /// Example: MOVX A, @R1
    Acc,
}

#[derive(Debug, Copy, Clone)]
pub enum OrlKind {
    Acc(OrlAcc),
}

#[derive(Debug, Copy, Clone)]
pub enum OrlAcc {
    /// Operation: A = A OR immediate
    /// Example: ORL A, #01h
    Const(Const),

    /// Operation: A = A OR (Ri)
    /// Example: ORL A, @R0
    Addr(Addr),

    /// Operation: A = A OR (direct)
    /// ORL A, P0
    Direct(Addr),
}

#[derive(Debug, Copy, Clone)]
pub enum OrlCarry {
    /// Operation: AC = C OR NOT (bit)
    /// Example: ORL C, /22h
    NotBit(Addr),

    /// Operation: C = C OR (bit)
    /// Example: ORL C, 22h
    Bit(Addr),
}

#[derive(Debug, Copy, Clone)]
pub enum OrlDirect {
    /// Operation: (direct) = (direct) OR immediate
    /// Example: ORL P0, #01h
    Const(Const),

    /// Operation: (direct) = (direct) OR A
    /// Operation: ORL P0, A
    Direct(Addr),
}

#[derive(Debug, Copy, Clone)]
pub enum SetbKind {
    /// Operation: (bit) = 1
    /// Example: SETB 63h
    Bit(Addr),

    /// Operation: C = 1
    /// Example: SETB C
    Carry,
}

#[derive(Debug, Copy, Clone)]
pub enum SubbKind {
    /// Operation: A = A - C - immediate
    /// Example: SUBB A, #01h
    Const(Const),

    /// Operation: A = A - C - (Ri)
    /// Example: SUBB A, @R1
    Addr(Addr),

    /// Operation: A = A - C - (direct)
    /// Example: SUBB A, 44h
    Direct(Addr),

    /// Operation: A = A - C - Rn
    /// Example: SUBB A, R5
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum XchKind {
    /// Operation: A swap (Ri)
    /// Example: XCH A, @R0
    Addr(Addr),

    /// Operation: A swap (direct)
    /// Example: XCH A, 45h
    Direct(Addr),

    /// Operation: A swap Rn
    /// Example: XCH A, R6
    Reg(Reg),
}
#[derive(Debug, Copy, Clone)]
pub enum XrlKind {
    Acc(XrlAcc),

    Direct(XrlDirect),
}

#[derive(Debug, Copy, Clone)]
pub enum XrlAcc {
    /// Operation: A = A XOR immediate
    /// Example: XRL A, #0FFh
    Const(Const),

    /// Operation: A = A XOR (Ri)
    /// Example: XRL A, @R0
    Addr(Addr),

    /// Operation: A = A XOR (direct)
    /// Example: XRL A, 34h
    Direct(Addr),

    /// Operation: A = A XOR Rn
    /// Example: XRL A, R7
    Reg(Reg),
}

#[derive(Debug, Copy, Clone)]
pub enum XrlDirect {
    /// Operation: (direct) = (direct) XOR immediate
    /// Example: XRL 34H, #0FFh
    Const(Const),

    /// Operation: (direct) = (direct) XOR A
    /// Example: XRL 34h, A
    Acc(Addr),
}
