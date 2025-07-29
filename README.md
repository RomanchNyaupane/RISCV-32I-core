# RISC-V RV32I Processor Implementation

## Overview

This repository contains an implementation of a RISC-V RV32I processor in Verilog HDL. The processor implements the base integer instruction set architecture (ISA) with a two-cycle datapath and finite state machine-based control unit.

## Architecture Overview

### Processor Components

The processor consists of the following major components:

- **Arithmetic Logic Unit (ALU)**: Performs arithmetic and logical operations
- **Register Bank**: 32 general-purpose registers (x0-x31)
- **Control Unit**: FSM-based instruction decode and control signal generation
- **Memory Subsystem**: Separate instruction and data memories
- **Instruction Counter**: Program counter implementation
- **Immediate Generator**: Sign-extends immediate values for different instruction types
- **Branch Comparator**: Handles branch condition evaluation

### Datapath Architecture

The processor uses datapath with the following key characteristics:

- 32-bit data width throughout the datapath
- Multiplexers for datapath routing
- Demultiplexer for ALU output distribution
- Register-based intermediate storage for multi-cycle operations

## Memory Architecture

### Memory Organization

The memory subsystem is organized into separate instruction and data memories, each implementing a banked architecture:

#### Data Memory (data_memory.v)
- **Size**: 4096 words (16'h0FFF)
- **Architecture**: 4-bank organization
- **Banks**: 
  - `data_mem_bank_3`: Bits [31:24]
  - `data_mem_bank_2`: Bits [23:16] 
  - `data_mem_bank_1`: Bits [15:8]
  - `data_mem_bank_0`: Bits [7:0]

#### Instruction Memory (instruction_memory.v)
- Read Only Memory(ROM) based
- Implemented using case statement
- Not all RV32I instruction is implemented - non-word memory read/write, J type, U type, environment call and break instruction is planned to be implemented in future.

### Address Translation

The memory subsystem implements a specific address translation scheme:

#### Input Address Width
- **Nominal Address**: 32-bit address input
- **Effective Address**: 30-bit address used for memory access
- **Translation**: `data_mem_addr[29:0]` from 32-bit input

#### Bank Selection and Word Alignment

The 4-bank architecture requires specific address handling:

1. **Word Alignment**: All memory accesses must be word-aligned (4-byte boundaries)
2. **Address Increment**: Memory addresses increment by 4 for consecutive word access
3. **Bank Distribution**: Each 32-bit word is distributed across 4 banks:
   - Bank 3: Most significant byte
   - Bank 2: Second byte
   - Bank 1: Third byte  
   - Bank 0: Least significant byte

#### Address Calculation Example
```
32-bit Address: 0x00000000 → Bank Address: 0x00000000 (Word 0)
32-bit Address: 0x00000004 → Bank Address: 0x00000001 (Word 1)  
32-bit Address: 0x00000008 → Bank Address: 0x00000002 (Word 2)
```

The lower 2 bits of the 32-bit address are effectively ignored since all accesses are word-aligned, allowing the 30-bit address to access the full memory range.

### Memory Operations

#### Synchronous Memory Design
Both instruction and data memories operate synchronously with the system clock:

- **Write Operations**: Data is written on the positive edge of the clock
- **Read Operations**: Data is available on the positive edge of the clock
- **Access Latency**: Single clock cycle for both read and write operations

#### Read Operation
```verilog
always @(posedge data_mem_clk) begin
    data_mem_out <= {data_mem_bank_3[data_mem_addr], 
                     data_mem_bank_2[data_mem_addr], 
                     data_mem_bank_1[data_mem_addr], 
                     data_mem_bank_0[data_mem_addr]};
end
```

#### Write Operation
```verilog
always @(posedge data_mem_clk) begin
    if (data_mem_wr_en) begin
        data_mem_bank_3[data_mem_addr] <= data_mem_in[31:24];
        data_mem_bank_2[data_mem_addr] <= data_mem_in[23:16];
        data_mem_bank_1[data_mem_addr] <= data_mem_in[15:8];
        data_mem_bank_0[data_mem_addr] <= data_mem_in[7:0];
    end
end
```

#### Memory Access Limitations
The memory subsystem supports only word (32-bit) aligned access:

- **Supported**: Word load (LW) and word store (SW) operations
- **Not Supported**: Halfword (16-bit) and byte (8-bit) operations
- **Alignment Requirement**: All memory addresses must be 4-byte aligned

## Control Unit Architecture

### Control Unit Design Philosophy

The control unit implements a finite state machine (FSM) approach combined with combinational logic rather than microcode-based control.
- Instruction Type Decoding: Combinational logic decodes instructions into types based on opcode fields.
- ALU Opcode Generation: Combinational always block generates ALU control signals.
- Multiplexer Control: Combinational assignment of datapath control signals.
This provides:

- Deterministic timing for each instruction type
- Simplified control logic
- Reduced complexity compared to microcoded implementations

### State Machine States

The control unit operates with 5 primary states:

1. **state_1**: Initial fetch preparation state
2. **state_2**: Instruction decode and execution initiation
3. **state_3**: Load instruction memory read completion
4. **state_4**: Store instruction memory write completion  
5. **state_5**: Halt state for undefined instructions

### Control Signal Generation

#### Instruction Type Decoding
The control unit decodes instructions into the following types:

- **R_type**: Register-register operations
- **I_type_1**: Immediate arithmetic operations
- **I_type_2**: Load operations
- **I_type_3**: Jump and link register operations
- **I_type_4**: System operations
- **S_type**: Store operations
- **B_type**: Branch operations
- **U_type**: Upper immediate operations
- **J_type**: Jump operations

#### ALU Operation Control
ALU operations are controlled through a 4-bit opcode:

```verilog
4'b0001: Addition
4'b0010: Subtraction  
4'b0011: XOR
4'b0100: OR
4'b0101: AND
4'b0110: Shift Left Logical
4'b0111: Shift Right Logical
4'b1000: Shift Right Arithmetic
4'b1001: Set Less Than (signed)
4'b1010: Set Less Than Unsigned
```

#### Control Signal Outputs

The control unit generates the following control signals sequentially:

- **ir_wr_en**: Instruction register write enable
- **ic_count**: Instruction counter increment
- **reg_wr_en**: Register bank write enable
- **mem_wr_en**: Data memory write enable
- **mar_wr_en**: Memory address register write enable
- **mdr_rd_en**: Memory data register read enable
- **imm_gen_instr_wr_en**: Immediate generator instruction write enable

The following control signals are generated combinationally:

- **alu_opcode[3:0]**: ALU operation selection
- **reg_rs_1_addr_wr_en**: Register source 1 address write enable
- **reg_rs_2_addr_wr_en**: Register source 2 address write enable
- **reg_rd_addr_wr_en**: Register destination address write enable
- **bc_en**: Branch comparator enable
- **mux_1_sel, mux_2_sel**: Datapath multiplexer selection
- **demux_1_sel**: ALU output demultiplexer selection
- **mux_3_sel[1:0]**: Register writeback multiplexer selection

  
### Multi-Cycle Operation Timing

#### R-Type Instructions (1 cycle)
1. **Cycle 1**: Instruction fetch, decode, execute, writeback, and program counter increment.

#### I-Type Arithmetic Instructions (1 cycle)  
1. **Cycle 1**: Instruction fetch and Decode, execute, writeback, and program counter increment.

#### I-Type Load Instructions (2 cycles)  
1. **Cycle 1**: Instruction fetch Address calculation, memory read initiation, and memory address register update
2. **Cycle 3**: Memory read completion and register writeback

#### S-Type Store Instructions (2 cycles)
1. **Cycle 1**: Instruction fetch, Address calculation, data preparation, and memory address register update
3. **Cycle 2**: Memory write completion

#### B-Type Branch Instructions (1 cycles)
1. **Cycle 1**: Instruction fetch and Condition evaluation and program counter update



## Register Bank

### Register Organization
- **Registers**: 32 × 32-bit general-purpose registers
- **Register x0**: Hardwired to zero (read-only)
- **Access**: Dual-port read, single-port write

### Register Access
- **Read Ports**: Two simultaneous read operations (rs1, rs2)
- **Write Port**: Single write operation (rd)
- **Address Width**: 5 bits (supports 32 registers)

## ALU Implementation

### Supported Operations
The ALU supports all RV32I arithmetic and logical operations:

- Arithmetic: ADD, SUB
- Logical: AND, OR, XOR  
- Shift: SLL, SRL, SRA
- Comparison: SLT, SLTU

### Flag Generation
- **Zero Flag**: Set when result equals zero
- **Carry Flag**: Set for arithmetic overflow (currently implemented as zero flag due to design error)
- **Branch Flag**: Set to indicate whether branching condition is true or false

## Instruction Counter (Program Counter)

### Functionality
- **Width**: 5-bit counter (supports 32 instruction addresses)
- **Operations**: Increment, decrement, direct load
- **Direction Control**: Bidirectional counting capability
- **Reset**: Synchronous reset to address 0

### Address Management
- **Default Operation**: Increment by 1 for sequential execution
- **Branch Operations**: Direct load for branch targets

## Immediate Generator

### Supported Immediate Types
The immediate generator handles sign extension for:

- **I-Type**: 12-bit signed immediate
- **S-Type**: 12-bit signed immediate (store offset)
- **B-Type**: 13-bit signed immediate (branch offset)


### Sign Extension Logic
All immediate values are sign-extended to 32 bits based on the most significant bit of the immediate field.

## Multiplexer Network

### Datapath Multiplexers
- **mux_1**: Selects between register output and instruction counter
- **mux_2**: Selects between register output and immediate value
- **mux_3**: Selects ALU result, memory data, or instruction counter for register writeback

### Control Logic
Multiplexer selection is controlled by the control unit based on instruction type and execution state.

## Branch Comparator

### Comparison Operations
- **BEQ**: Branch if equal
- **BNE**: Branch if not equal  
- **BLT**: Branch if less than (signed)
- **BGE**: Branch if greater than or equal (signed)
- **BLTU**: Branch if less than (unsigned)
- **BGEU**: Branch if greater than or equal (unsigned)

### Implementation
The branch comparator operates in parallel with ALU operations to minimize branch latency.

## Module Interconnection

### Top-Level Module (module_connect.v)
The top-level module instantiates and interconnects all processor components:

- Component instantiation with parameter passing
- Signal routing between modules
- Clock and reset distribution
- Control signal fan-out

### Signal Naming Convention
- **dp_**: Data path signals
- **ctrl_**: Control signals
- Signals named after the output port of the driving module

## Supported Instruction Set

### Implemented Instructions
The processor implements a subset of the RV32I base instruction set:

#### Arithmetic Instructions
- ADD, SUB, ADDI
- SLT, SLTU, SLTI, SLTIU

#### Logical Instructions  
- AND, OR, XOR, ANDI, ORI, XORI

#### Shift Instructions
- SLL, SRL, SRA, SLLI, SRLI, SRAI

#### Memory Instructions
- LW (Load Word)
- SW (Store Word)

#### Branch Instructions
- BEQ, BNE, BLT, BGE, BLTU, BGEU

### Unimplemented Instructions
The following RV32I instructions are not currently implemented:

#### Jump Instructions
- JAL, JALR

#### Upper Immediate Instructions
- LUI, AUIPC

#### Memory Access Limitations
- Load/Store Halfword (LH, LHU, SH)
- Load/Store Byte (LB, LBU, SB)

## Design Constraints

### Current Limitations
- **Memory Size**: Limited to 4K words per memory
- **Address Space**: 5-bit instruction counter limits program size to 32 instructions
- **Data Width**: Fixed 32-bit data path
- **Memory Access**: Word-aligned access only (no halfword or byte operations)
- **Instruction Set**: Subset implementation missing JAL, JALR, LUI, and AUIPC instructions
- **Memory Design**: Synchronous operation requires clock cycle for all memory access

### Architecture Decisions
- **Multi-cycle Design**: Chosen over single-cycle for reduced hardware complexity
- **FSM Control**: State machine approach provides deterministic timing but limits performance optimization
- **Synchronous Memory**: Simplifies timing analysis but adds latency to memory operations

## Usage

### Simulation
1. Include all Verilog files in simulation environment
2. Instantiate `module_connect` as top-level module
3. Apply clock and reset signals
4. Load program into instruction memory
5. Monitor register and memory states

### Synthesis
The design is synthesizable for FPGA implementation with appropriate clock constraints.

## File Structure

```
RISC_V_RV32I/
├── ALU.v                           # Arithmetic Logic Unit
├── control_unit.v                  # FSM-based control unit
├── data_memory.v                   # Data memory with banking
├── instruction_memory.v            # Instruction memory with banking
├── register_bank.v                 # 32-register file
├── instruction_counter.v           # Program counter
├── instruction_register.v          # Instruction register
├── memory_address_register.v       # Memory address register
├── memory_data_register.v          # Memory data register
├── imm_gen.v                      # Immediate generator
├── branch_comparator.v            # Branch condition evaluation
├── mux_1.v, mux_2.v, mux_3.v     # Datapath multiplexers
├── demux_1.v                      # ALU output demultiplexer
├── reg_bank_address_register.v    # Register address storage
└── module_connect.v               # Top-level interconnection
```

This implementation provides a complete, functional RISC-V RV32I processor suitable for educational purposes and further development.
