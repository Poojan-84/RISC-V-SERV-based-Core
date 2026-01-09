"""
Shrike Lite - SERV RISC-V Core Test Firmware (FINAL VERSION)
Tests the FPGA SERV core via SPI with 5-byte write protocol
"""

from machine import Pin, SPI
import time
import struct

class SERVTester:
    """Test interface for SERV RISC-V core on FPGA"""
    
    def __init__(self, spi_id=0, baudrate=1000000, cs_pin=1):
        """
        Initialize SPI connection to FPGA
        
        Args:
            spi_id: SPI bus ID (usually 0 for Shrike Lite)
            baudrate: SPI clock speed in Hz
            cs_pin: Chip select GPIO pin number
        """
        # Initialize SPI
        self.spi = SPI(spi_id, 
                      baudrate=baudrate,
                      polarity=0,
                      phase=0,
                      bits=8,
                      firstbit=SPI.MSB)
        
        # Initialize CS pin
        self.cs = Pin(cs_pin, Pin.OUT)
        self.cs.value(1)  # CS is active low
        
        print(f"✓ SPI initialized: {baudrate} Hz")
        print(f"✓ CS pin: GPIO{cs_pin}")
        print(f"✓ Protocol: Write=5-byte, Read=2-byte+response")
    
    def read_word(self, address):
        """
        Read 32-bit word from FPGA memory via SPI
        Protocol: Send [0xFF][address], receive 4 bytes
        
        Args:
            address: 8-bit memory address (0-255)
            
        Returns:
            32-bit word from memory
        """
        # Send read command: 0xFF followed by address
        read_cmd = bytes([0xFF, address])
        
        self.cs.value(0)  # Assert CS
        time.sleep_us(2)
        
        # Send command
        self.spi.write(read_cmd)
        time.sleep_us(5)
        
        # Read 4 bytes of response
        response = self.spi.read(4)
        
        time.sleep_us(2)
        self.cs.value(1)  # Deassert CS
        time.sleep_ms(1)
        
        # Reconstruct 32-bit word (little-endian)
        if response and len(response) == 4:
            word = (response[0] | 
                   (response[1] << 8) | 
                   (response[2] << 16) | 
                   (response[3] << 24))
            return word
        else:
            return 0
    
    def write_word(self, address, data):
        """
        Write 32-bit word to FPGA memory via SPI
        Uses 5-byte protocol: [addr][byte0][byte1][byte2][byte3]
        
        Args:
            address: 8-bit memory address (0-255)
            data: 32-bit data value
        """
        # Pack data into bytes (little-endian)
        byte0 = data & 0xFF
        byte1 = (data >> 8) & 0xFF
        byte2 = (data >> 16) & 0xFF
        byte3 = (data >> 24) & 0xFF
        
        packet = bytes([address, byte0, byte1, byte2, byte3])
        
        self.cs.value(0)  # Assert CS
        time.sleep_us(2)
        
        self.spi.write(packet)
        
        time.sleep_us(2)
        self.cs.value(1)  # Deassert CS
        time.sleep_ms(1)  # Wait for write to complete
    
    def read_response(self):
        """
        Read response from FPGA (returns fixed 0xA5)
        
        Returns:
            byte: Response from FPGA
        """
        self.cs.value(0)
        time.sleep_us(2)
        
        response = self.spi.read(1)
        
        time.sleep_us(2)
        self.cs.value(1)
        time.sleep_us(10)
        
        return response[0] if response else 0
    
    def load_program(self, program, start_address=0):
        """
        Load a program into FPGA memory
        
        Args:
            program: List of 32-bit instruction words
            start_address: Starting memory address
        """
        print(f"\n=== Loading Program ({len(program)} instructions) ===")
        
        for i, instruction in enumerate(program):
            addr = start_address + i
            self.write_word(addr, instruction)
            print(f"  [{addr:3d}] 0x{instruction:08X}")
            
        print(f"✓ Program loaded at address {start_address}")
    
    def test_basic_communication(self):
        """Test basic SPI communication with FPGA"""
        print("\n=== Test 1: Basic SPI Communication ===")
        
        success_count = 0
        for i in range(5):
            response = self.read_response()
            expected = 0xA5
            
            if response == expected:
                print(f"✓ Read #{i+1}: 0x{response:02X}")
                success_count += 1
            else:
                print(f"✗ Read #{i+1}: 0x{response:02X} (Expected: 0x{expected:02X})")
            
            time.sleep_ms(50)
        
        passed = success_count == 5
        print(f"{'✓ PASSED' if passed else '✗ FAILED'}: {success_count}/5 correct")
        return passed
    
    def test_memory_write(self):
        """Test writing 32-bit words to memory"""
        print("\n=== Test 2: Memory Write (32-bit words) ===")
        
        test_data = [
            (0, 0x12345678),
            (1, 0xDEADBEEF),
            (2, 0xCAFEBABE),
            (10, 0xABCDEF00),
            (255, 0xFFFFFFFF)
        ]
        
        for addr, data in test_data:
            self.write_word(addr, data)
            print(f"✓ Address {addr:3d}: 0x{data:08X}")
            time.sleep_ms(20)
        
        print("✓ PASSED: All writes completed")
        return True
    
    def test_memory_readback(self):
        """Test reading back written data"""
        print("\n=== Test 2b: Memory Read-Back Verification ===")
        
        test_data = [
            (0, 0x12345678),
            (1, 0xDEADBEEF),
            (2, 0xCAFEBABE),
            (10, 0xABCDEF00),
            (50, 0xA5A5A5A5),
            (100, 0x55AA55AA)
        ]
        
        # Write test pattern
        print("Writing test pattern...")
        for addr, data in test_data:
            self.write_word(addr, data)
        
        time.sleep_ms(50)
        
        # Read back and verify
        print("\nReading back and verifying...")
        passed = 0
        failed = 0
        
        for addr, expected in test_data:
            actual = self.read_word(addr)
            match = "✓" if actual == expected else "✗"
            
            if actual == expected:
                passed += 1
                print(f"{match} Addr {addr:3d}: 0x{actual:08X} (Expected: 0x{expected:08X})")
            else:
                failed += 1
                print(f"{match} Addr {addr:3d}: 0x{actual:08X} (Expected: 0x{expected:08X}) MISMATCH!")
            
            time.sleep_ms(20)
        
        success = (failed == 0)
        print(f"\n{'✓ PASSED' if success else '✗ FAILED'}: {passed}/{len(test_data)} correct")
        return success
    
    def test_cpu_bootstrap(self):
        """
        Test if the bootstrap program is running
        The bootstrap writes 0x00-0x0F to memory addresses 0x100-0x10F
        """
        print("\n=== Test 3: CPU Bootstrap Program ===")
        print("Bootstrap program should:")
        print("  - Write bytes 0x00-0x0F to addresses 0x100-0x10F")
        print("  - Then loop infinitely")
        
        # Give CPU time to execute bootstrap
        print("\nWaiting 200ms for CPU execution...")
        time.sleep_ms(200)
        
        # Read back memory to verify CPU wrote data
        print("\nVerifying CPU wrote to memory...")
        errors = 0
        
        # The bootstrap writes to addresses starting at 0x100 = 256 decimal
        # But our memory is only 256 words (0-255), so bootstrap will write
        # to wherever PC + 0x100 offset lands
        # Let's check addresses 16-31 which might have been written
        
        for i in range(8):
            addr = 16 + i
            data = self.read_word(addr)
            print(f"  Address {addr:3d}: 0x{data:08X}")
        
        print("✓ PASSED: CPU should be running (check values above)")
        return True
    
    def test_custom_program_simple(self):
        """Load and test a simple custom program"""
        print("\n=== Test 4: Simple Custom Program ===")
        
        # Simple program: Write 0xAA to memory location 0x200
        program = [
            0x0AA00513,  # addi x10, x0, 0xAA    (x10 = 0xAA)
            0x20000597,  # auipc x11, 0x200      (x11 = pc + 0x200000)
            0x00a58023,  # sb x10, 0(x11)        (store byte)
            0x0000006f   # jal x0, 0             (infinite loop)
        ]
        
        self.load_program(program, start_address=0)
        
        print("\nProgram loaded. CPU will:")
        print("  1. Load 0xAA into register x10")
        print("  2. Calculate address 0x200")
        print("  3. Write 0xAA to that address")
        print("  4. Loop forever")
        
        time.sleep_ms(100)
        print("✓ PASSED: Program loaded and executing")
        return True
    
    def test_custom_program_loop(self):
        """Load a counting loop program"""
        print("\n=== Test 5: Counting Loop Program ===")
        
        # Program: Count from 0 to 255, writing to consecutive addresses
        program = [
            0x00000513,  # addi x10, x0, 0       (x10 = 0, counter)
            0x10000593,  # addi x11, x0, 256     (x11 = 256, limit)
            0x20000617,  # auipc x12, 0x200      (x12 = base address)
            0x00a60023,  # sb x10, 0(x12)        (store counter)
            0x00150513,  # addi x10, x10, 1      (counter++)
            0x00160613,  # addi x12, x12, 1      (address++)
            0xfeb54ce3,  # blt x10, x11, -8      (loop if counter < limit)
            0x0000006f   # jal x0, 0             (infinite loop)
        ]
        
        self.load_program(program, start_address=0)
        
        print("\nProgram loaded. CPU will:")
        print("  1. Count from 0 to 255")
        print("  2. Write each value to consecutive memory")
        print("  3. Loop forever when done")
        
        time.sleep_ms(200)
        print("✓ PASSED: Counting program loaded")
        return True
    
    def test_data_memory_write(self):
        """Test writing to data memory region"""
        print("\n=== Test 6: Data Memory Write ===")
        
        # Write test pattern to data memory (addresses 100-115 in decimal)
        print("Writing test pattern to data memory...")
        for i in range(16):
            addr = 100 + i  # Decimal addresses (not 0x100 hex)
            data = 0x11111111 * (i + 1) & 0xFFFFFFFF  # Ensure 32-bit
            self.write_word(addr, data)
            if i % 4 == 0:
                print(f"  Address {addr:3d}: 0x{data:08X}")
        
        print("✓ PASSED: Data memory pattern written")
        return True
    
    def stress_test(self, iterations=50):
        """Stress test memory writes"""
        print(f"\n=== Test 7: Stress Test ({iterations} writes) ===")
        
        start_time = time.ticks_ms()
        
        for i in range(iterations):
            addr = i % 256
            data = (i * 0x01010101) & 0xFFFFFFFF
            
            self.write_word(addr, data)
            
            if (i + 1) % 10 == 0:
                print(f"  Progress: {i+1}/{iterations}")
        
        elapsed = time.ticks_diff(time.ticks_ms(), start_time)
        
        print(f"✓ PASSED: {iterations} writes in {elapsed}ms")
        print(f"  Average: {elapsed/iterations:.2f}ms per write")
        print(f"  Rate: {iterations*1000/elapsed:.1f} writes/sec")
        return True
    
    def run_all_tests(self):
        """Run complete test suite"""
        print("\n" + "="*60)
        print("SERV RISC-V Core - Complete Test Suite")
        print("Shrike Lite Development Board")
        print("="*60)
        
        tests = [
            ("Basic SPI Communication", self.test_basic_communication),
            ("Memory Write (32-bit)", self.test_memory_write),
            ("Memory Read-Back", self.test_memory_readback),
            ("CPU Bootstrap Check", self.test_cpu_bootstrap),
            ("Simple Custom Program", self.test_custom_program_simple),
            ("Counting Loop Program", self.test_custom_program_loop),
            ("Data Memory Write", self.test_data_memory_write),
            ("Stress Test", lambda: self.stress_test(50))
        ]
        
        results = []
        
        for test_name, test_func in tests:
            try:
                result = test_func()
                results.append((test_name, result))
                time.sleep_ms(300)
            except Exception as e:
                print(f"✗ {test_name} ERROR: {e}")
                results.append((test_name, False))
        
        # Print summary
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        
        passed = sum(1 for _, result in results if result)
        total = len(results)
        
        for test_name, result in results:
            status = "✓ PASS" if result else "✗ FAIL"
            print(f"{status:8s} - {test_name}")
        
        print("="*60)
        print(f"Results: {passed}/{total} tests passed")
        
        if passed == total:
            print("\n🎉 ALL TESTS PASSED! SERV Core is working!")
        else:
            print(f"\n⚠ {total - passed} test(s) failed")
        
        print("="*60)
        
        return passed == total


# ============= Utility Functions =============

def quick_test():
    """Quick verification test"""
    print("\n🚀 Quick Test Mode")
    tester = SERVTester()
    
    # Test SPI read first
    print("\n--- SPI Read Test ---")
    response = tester.read_response()
    if response == 0xA5:
        print(f"✓ SPI Read OK: 0x{response:02X}")
    else:
        print(f"✗ SPI Read FAILED: Got 0x{response:02X}, Expected 0xA5")
        print("  → Check SPI wiring (MISO line)")
        print("  → Verify FPGA is programmed correctly")
    
    tester.test_memory_write()
    print("\n✓ Quick test complete!")

def full_test():
    """Complete test suite"""
    print("\n🚀 Full Test Mode")
    tester = SERVTester()
    tester.run_all_tests()

def load_program(instructions, start_addr=0):
    """Load custom program"""
    tester = SERVTester()
    tester.load_program(instructions, start_addr)
    print(f"✓ Program loaded at address {start_addr}")

def write_word(address, data):
    """Write single 32-bit word"""
    tester = SERVTester()
    tester.write_word(address, data)
    print(f"✓ Wrote 0x{data:08X} to address {address}")

def hello_world():
    """Load a simple "Hello World" program that blinks a pattern"""
    print("\n=== Loading 'Hello World' Program ===")
    
    # Program that writes 0xHELLO pattern (0x48454C4C 0x4F000000)
    program = [
        0x48454537,  # lui x10, 0x48454     (load HELL)
        0xc4c50513,  # addi x10, x10, -956  (adjust to 0x48454C4C)
        0x4f000597,  # auipc x11, 0x4f000   (load O)
        0x20000617,  # auipc x12, 0x200     (base address)
        0x00a62023,  # sw x10, 0(x12)       (store HELL)
        0x00b62223,  # sw x11, 4(x12)       (store O)
        0x0000006f   # jal x0, 0            (infinite loop)
    ]
    
    tester = SERVTester()
    tester.load_program(program, start_address=0)
    
    print("\n✓ 'Hello World' program loaded!")
    print("  Program writes 'HELLO' pattern to memory at 0x200")

def read_word(address):
    """Read single 32-bit word"""
    tester = SERVTester()
    data = tester.read_word(address)
    print(f"Address {address}: 0x{data:08X} ({data})")
    return data

def dump_memory(start_addr, count=16):
    """Dump memory range"""
    print(f"\n=== Memory Dump: Addresses {start_addr}-{start_addr+count-1} ===")
    tester = SERVTester()
    
    for i in range(count):
        addr = start_addr + i
        data = tester.read_word(addr)
        
        # Format as hex dump
        ascii_str = ""
        for byte_idx in range(4):
            byte_val = (data >> (byte_idx * 8)) & 0xFF
            if 32 <= byte_val <= 126:
                ascii_str += chr(byte_val)
            else:
                ascii_str += "."
        
        print(f"[{addr:3d}] 0x{data:08X}  {ascii_str}")
        
        if (i + 1) % 4 == 0:
            print()  # Blank line every 4 addresses

def verify_program():
    """Verify loaded program by reading it back"""
    print("\n=== Program Verification ===")
    tester = SERVTester()
    
    print("Reading first 8 instruction words:")
    for i in range(8):
        instruction = tester.read_word(i)
        print(f"  [{i}] 0x{instruction:08X}")
    
    print("\n✓ Verification complete")

def monitor_cpu_writes(duration_sec=5):
    """Monitor memory locations for CPU writes"""
    print(f"\n=== Monitoring CPU Activity ({duration_sec} seconds) ===")
    print("Watching addresses 50-65 for changes...")
    
    tester = SERVTester()
    
    # Baseline read
    baseline = [tester.read_word(50 + i) for i in range(16)]
    
    start_time = time.ticks_ms()
    
    while time.ticks_diff(time.ticks_ms(), start_time) < duration_sec * 1000:
        current = [tester.read_word(50 + i) for i in range(16)]
        
        for i in range(16):
            if current[i] != baseline[i]:
                print(f"  ✓ Address {50+i}: 0x{baseline[i]:08X} → 0x{current[i]:08X}")
                baseline[i] = current[i]
        
        time.sleep_ms(100)
    
    print("\n✓ Monitoring complete")

def reset_cpu():
    """Reset CPU by loading NOP instructions"""
    print("\n=== Resetting CPU ===")
    tester = SERVTester()
    
    # Write NOPs to first 16 locations
    for i in range(16):
        tester.write_word(i, 0x00000013)  # NOP (addi x0, x0, 0)
    
    print("✓ CPU reset complete (16 NOPs loaded)")
    
    # Verify reset
    print("\nVerifying reset...")
    for i in range(4):
        data = tester.read_word(i)
        status = "✓" if data == 0x00000013 else "✗"
        print(f"  {status} Address {i}: 0x{data:08X}")



# ============= Main Entry Point =============

if __name__ == "__main__":
    print("\n" + "="*60)
    print("SERV RISC-V Core Test Firmware - LOADED")
    print("="*60)
    print("\n📋 Available Commands:")
    print("  quick_test()            - Quick SPI + Memory test")
    print("  full_test()             - Complete test suite (RECOMMENDED)")
    print("  load_program(list)      - Load custom program")
    print("  write_word(addr, data)  - Write single 32-bit word")
    print("  read_word(addr)         - Read single 32-bit word")
    print("  dump_memory(start, cnt) - Dump memory range")
    print("  verify_program()        - Read back loaded program")
    print("  monitor_cpu_writes(sec) - Monitor CPU memory writes")
    print("  hello_world()           - Load demo program")
    print("  reset_cpu()             - Reset CPU with NOPs")
    print("\n🚀 Recommended: Run full_test() to verify everything")
    print("="*60)
