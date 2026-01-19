# Shrike-V & SERV-lite: RISC-V Architecture Exploration

This project features the implementation and testing of two distinct RISC-V processor architectures on an FPGA: **Shrike-V** (a 32-bit parallel RV32I core) and **SERV-lite** (a bit-serial RV32I core). The project includes a full hardware-software interface allowing for program injection via an ESP32 master using a custom SPI protocol.

---

## 🚀 Project Overview

The goal of this project was to understand the trade-offs between **performance (parallel design)** and **area optimization (serial design)** within the RISC-V ecosystem.

* **Shrike-V:** A standard 32-bit datapath where instructions execute in a single-cycle/parallel fashion.
* **SERV-lite:** A minimal-area bit-serial implementation that processes data 1 bit at a time over 32 clock cycles.
* **Firmware Bridge:** A MicroPython-based test suite running on an ESP32 to bootstrap the FPGA, inject instructions into memory, and verify CPU liveness.

---

## 🏗️ Hardware Architecture (RV32I)

Both cores implement the **RV32I Base Integer Instruction Set**, which consists of 47 instructions covering:
* **Arithmetic & Logical:** ADD, SUB, AND, OR, XOR, SLL, SRL, SRA.
* **Memory Access:** LW (Load Word) and SW (Store Word).
* **Control Flow:** BEQ, BNE, BLT, JAL (Jump and Link).

### System Components:
1.  **ALU (Arithmetic Logic Unit):** Handles 32-bit operations (Parallel in Shrike, 1-bit Serial in SERV).
2.  **Register File:** Houses 32 general-purpose registers (x0-x31), with x0 hardwired to zero.
3.  **Control Unit:** Decodes 32-bit machine code into control signals (ALUOp, MemWrite, RegWrite).
4.  **ImmGen (Immediate Generator):** Extracts and sign-extends constant values from instructions.
5.  **Program Counter (PC):** Manages instruction sequencing and branching logic.



---

## 🔌 Hardware-Software Integration

Since FPGAs lose memory contents on power-off, I developed a **Bootloader Bridge** to "brain surgery" the CPU.

### SPI Bootstrap Protocol
The ESP32 acts as the SPI Master, using a **5-byte write protocol** to program the FPGA's Instruction Memory:
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 |
| :--- | :--- | :--- | :--- | :--- |
| **8-bit Address** | **Data [7:0]** | **Data [15:8]** | **Data [23:16]** | **Data [31:24]** |



### Firmware Features (`tester.py`):
* **`load_program()`**: Converts hex lists into SPI packets to inject machine code.
* **`debug_spi_read()`**: Frequency-sweeping diagnostic to verify FPGA MISO timing (detects `0xA5` handshake).
* **`test_cpu_bootstrap()`**: Verifies execution by monitoring memory-mapped I/O changes.

---

## 📊 Parallel vs. Serial Comparison

| Feature | Shrike-V | SERV-lite |
| :--- | :--- | :--- |
| **ALU Width** | 32-bit | 1-bit |
| **Execution Speed** | 1 Cycle / Instruction | 32+ Cycles / Instruction |
| **FPGA Resource Use** | ~1200 LUTs | ~200 LUTs |
| **Primary Advantage** | High Throughput | Extreme Area Efficiency |

---

## 🛠️ How to Use

1.  **Synthesize Hardware:** Use the Yosys/Nextpnr flow (or your preferred FPGA toolchain) to flash the `src/` Verilog files onto your FPGA.
2.  **Connect ESP32:** Wire the SPI pins (MOSI, MISO, SCK, CS) between the ESP32 and FPGA.
3.  **Run Firmware:**
    ```python
    from tester import SERVTester
    tester = SERVTester()
    tester.full_test() # Runs all diagnostics and loads a sample loop
    ```

---

## 📜 Acknowledgments
* **SERV Architecture:** Inspired by the award-winning bit-serial RISC-V core by Olof Kindgren.
* **RISC-V International:** For the open-source ISA specifications.
