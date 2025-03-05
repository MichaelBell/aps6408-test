import time
import machine
import gc
import random
import rp2
from machine import SPI, Pin

@rp2.asm_pio(autopush=True, push_thresh=8, in_shiftdir=rp2.PIO.SHIFT_LEFT,
             autopull=True, pull_thresh=8, out_shiftdir=rp2.PIO.SHIFT_RIGHT,
             out_init=(rp2.PIO.OUT_HIGH,)*8,
             sideset_init=(rp2.PIO.OUT_LOW))
def ospi_cmd():
    label("top")
    out(x, 8).side(0)
    out(y, 32).side(0)
    out(pindirs, 8).side(0)
    
    label("cmd_loop")
    out(pins, 8).side(0)
    nop().side(1)
    out(pins, 8).side(1)
    jmp(x_dec, "cmd_loop").side(0)
    
    jmp(not_y, "top").side(0)
    
    out(pindirs, 8).side(0)
    label("data_loop")
    in_(pins, 8).side(1).delay(1)
    in_(pins, 8).side(0)
    jmp(y_dec, "data_loop").side(0)
    
@rp2.asm_pio(autopush=True, push_thresh=24, in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def pio_capture():
    in_(pins, 12)
    
class OSPI:
    def __init__(self):
        self.sm = rp2.StateMachine(0, ospi_cmd, 8_000_000, in_base=Pin(4), out_base=Pin(4), sideset_base=Pin(12))
        self.sm.active(1)
        
        self.ce = Pin(13, Pin.OUT, value=1)
        self.dqs = Pin(14, Pin.IN)
        self.rst = Pin(15, Pin.OUT, value=0)
        time.sleep_us(100)
        self.rst.on()
    
    def read_cregs(self):    
        for creg in range(5):
            data = bytearray(2)
            num_bytes = len(data)
            
            self.ce.off()
        
            self.sm.put(1+2-1)     # Command + Address - 1
            self.sm.put(num_bytes//2 + 5 - 1) # Data + Dummy - 1
            self.sm.put(0xff)  # Directions
            
            self.sm.put(0x40)  # Command
            self.sm.put(0x40)
            self.sm.put(0)
            self.sm.put(0)
            self.sm.put(0)
            self.sm.put(creg)
            self.sm.put(0)     # Directions
            
            for i in range(10):
                self.sm.get()
            for i in range(num_bytes):
                data[i] = self.sm.get()
                
            self.ce.on()
            
            print(f"MA {creg} Received: {data[0]:02x} {data[1]:02x}")

    def write(self, data, addr, wlat=2, cmd=0xa0):
        self.ce.off()
    
        self.sm.put(1+2 + wlat + len(data)//2 -1) # Command + Address + Latency + data - 1
        self.sm.put(0)     # No read
        self.sm.put(0xff)  # Directions
        
        self.sm.put(cmd)  # Command
        self.sm.put(cmd)
        self.sm.put(addr >> 24)
        self.sm.put(addr >> 16)
        self.sm.put(addr >> 8)
        self.sm.put(addr)
        
        for i in range(wlat):
            self.sm.put(0)
            self.sm.put(0)
        
        for i in range(len(data)):
            self.sm.put(data[i])

        time.sleep_us(50)
        self.ce.on()
        
    def read(self, data, addr, rlat=9):
        num_bytes = len(data)
        
        self.ce.off()
    
        self.sm.put(1+2-1)     # Command + Address - 1
        self.sm.put(num_bytes//2 + rlat - 1) # Data + Dummy - 1
        self.sm.put(0xff)  # Directions
        
        self.sm.put(0x20)  # Command
        self.sm.put(0x20)
        self.sm.put(addr >> 24)
        self.sm.put(addr >> 16)
        self.sm.put(addr >> 8)
        self.sm.put(addr)
        self.sm.put(0)     # Directions
        
        for i in range(rlat):
            self.sm.get()
            self.sm.get()
        for i in range(num_bytes):
            data[i] = self.sm.get()
            
        self.ce.on()

sram = OSPI()
sram.read_cregs()

# Set up fixed read latency, write latency 3
sram.write(b"\x29\x00", 0, 0, cmd=0xc0)
sram.write(b"\x00\x00", 4, 0, cmd=0xc0)

sram.read_cregs()

data = b"Hello world!"
sram.write(data, 0x80, 2)

read_data = bytearray(len(data))
sram.read(read_data, 0x80, 9)

print("Read back: ", read_data)

t = time.ticks_ms()
data = bytearray(32)
val = 0
for addr in range(0, 1024 * 1024, len(data)):
    for i in range(len(data)):
        data[i] = (val + i) & 0xff
    val += 1
    sram.write(data, addr)

print(f"Writes done in {time.ticks_ms() - t}ms")
    
t = time.ticks_ms()
val = 0
for addr in range(0, 1024 * 1024, len(data)):
    sram.read(data, addr)
    for i in range(len(data)):
        if data[i] != (val + i) & 0xff:
            print(f"Fail at addr {addr+i}: {data[i]} != {val + i}")
            break
    val += 1

print(f"Reads checked in {time.ticks_ms() - t}ms")
