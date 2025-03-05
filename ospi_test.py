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
def ospi_read():
    out(x, 8).side(0)
    out(y, 32).side(0)
    out(pindirs, 8).side(0)
    
    label("cmd_loop")
    out(pins, 8).side(0)
    nop().side(1)
    out(pins, 8).side(1)
    jmp(x_dec, "cmd_loop").side(0)
    
    out(pindirs, 8).side(0)
    label("data_loop")
    in_(pins, 8).side(1).delay(1)
    in_(pins, 8).side(0)
    jmp(y_dec, "data_loop").side(0)
    
@rp2.asm_pio(autopush=True, push_thresh=24, in_shiftdir=rp2.PIO.SHIFT_RIGHT)
def pio_capture():
    in_(pins, 12)
    
def test_ospi():
    sm = rp2.StateMachine(0, ospi_read, 8_000_000, in_base=Pin(4), out_base=Pin(4), sideset_base=Pin(12))
    sm.active(1)
    
    ce = Pin(13, Pin.OUT, value=1)
    dqs = Pin(14, Pin.IN)
    rst = Pin(15, Pin.OUT, value=1)
    
    cap_sm = rp2.StateMachine(1, pio_capture, 8_000_000, in_base=Pin(4))

    capture_len=1024
    buf = bytearray(capture_len)

    rx_dma = rp2.DMA()
    c = rx_dma.pack_ctrl(inc_read=False, treq_sel=5) # Read using the SM1 RX DREQ
    cap_sm.restart()
    cap_sm.exec("wait(%d, gpio, %d)" % (1, 12))
    rx_dma.config(
        read=0x5020_0024,        # Read from the SM1 RX FIFO
        write=buf,
        ctrl=c,
        count=capture_len//4,
        trigger=True
    )
    cap_sm.active(1)    
    
    for creg in range(5):
        data = bytearray(2)
        num_bytes = len(data)
        
        ce.off()
    
        sm.put(1+2-1)     # Command + Address - 1
        sm.put(num_bytes//2 + 5 - 1) # Data + Dummy - 1
        sm.put(0xff)  # Directions
        
        sm.put(0x40)  # Command
        sm.put(0x40)
        sm.put(0)
        sm.put(0)
        sm.put(0)
        sm.put(creg)
        sm.put(0)     # Directions
        
        for i in range(10):
            sm.get()
        for i in range(num_bytes):
            data[i] = sm.get()
            
        ce.on()
        
        print(f"MA {creg} Received: {data[0]:02x} {data[1]:02x}")

    # Wait for DMA to complete
    while rx_dma.active():
        time.sleep_ms(1)
        
    cap_sm.active(0)
    del cap_sm

    for j in range(12):
        print("%02d: " % (j+4,), end="")
        for i in range(0, capture_len, 4):
            d = (buf[i+1] << 8) | (buf[i+2] << 16) | (buf[i+3] << 24)
            d >>= 8
            print("-" if (d & (1 << j)) != 0 else "_", end = "")
            print("-" if (d & (1 << (j+12))) != 0 else "_", end = "")
        print()
