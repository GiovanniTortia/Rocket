from machine import I2C
import math

class QMC5883L:
    
    def __init__(self, i2c):
        self.address = 0x0D
        self.i2c = i2c
        self.res = 1/0xFFFF
        self.config()
        
    def write_reg(self, reg_addr, value):
        self.i2c.writeto_mem(self.address, reg_addr, value)
    
    def read_reg(self, reg_addr, nbytes):
        return self.i2c.readfrom_mem(self.address, reg_addr, nbytes)
        
    def config(self):
        self.write_reg(0x0A, b'\x80')		#soft reset
        self.write_reg(0x0B, b'\x01')		#set/reset period (?)
        self.write_reg(0x09, b'\x1D')		#full-scale, osr, odr, continuous/standby mode
        self.write_reg(0x0A, b'\x01')		#disable interrupt pin
    
    def isReady(self):
        drdy = self.read_reg(0x06, 1)
        mask = b'\x01'
        return drdy[0]&mask[0] == 1
        
        
    def read_mag(self):
        reg = 0x00
        out = []
        
        for i in range(0, 3):
            raw = self.read_reg(reg, 2)
            raw = int.from_bytes(raw, "little")
            if raw >= 32768:
                raw -= 65536
            out.append(raw*self.res*16)
            reg = reg + 2
            
        return out
    
    #must update to consider declination
    def getHdn(self):
        read = self.read_mag()
        hdn = math.atan2(read[1], read[0])*180/math.pi
        if hdn < 0:
            hdn = hdn + 360
        return hdn
        
    
   
            
    
        