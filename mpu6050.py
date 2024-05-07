from machine import I2C
import time
import math

#I2C ADDRESS FOR THE SENSOR
MPU_ADDR = 0x68

#ACCELEROMETERS OUTPUT REGISTERS, BOTH THESE AND THE GYRO'S ARE COMPOSED OF 2 8b REGISTERS
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

#GYRO OUTPUT REGISTERS
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

#CONFIGURATION REGISTERS
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
PWR_MGMT_1 = 0x6B


#INTERRUPT REGISTERS
INT_STATUS = 0X3A
INT_ENABLE = 0X38



class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        self.acc_res = 1/16384.0				#based on full scale, see datasheet
        self.gyro_res = 1/65.5
        self.g_offset = [0, 0, 0]
        self.angle = [0, 0, 0]
        self.last = 0
        
    def update_angle(self):
        if self.isReady():
            rpm = self.get_gyro()
            now = time.ticks_ms()
            
            self.angle[0] += rpm[0]*time.ticks_diff(now, self.last)/1000
            self.angle[1] += rpm[1]*time.ticks_diff(now, self.last)/1000
            self.angle[2] += rpm[2]*time.ticks_diff(now, self.last)/1000
            
            self.last = now
        
    def write_reg(self, reg, val):
        self.i2c.writeto_mem(MPU_ADDR, reg, val)
        
    def read_reg(self, reg, nbytes):
        return self.i2c.readfrom_mem(MPU_ADDR, reg, nbytes)
    
    def config(self):
        self.write_reg(PWR_MGMT_1, b'\x80')		#reset
        time.sleep(1)
        self.write_reg(SMPLRT_DIV, b'\x00')		#no sample rate divider
        self.write_reg(CONFIG, b'\x00')			#no low pass filter, no FSYNC
        self.write_reg(GYRO_CONFIG, b'\x08')	#full scale = +/-500°/s
        self.write_reg(ACCEL_CONFIG, b'\x00')	#full scale = 2g
        self.write_reg(PWR_MGMT_1, b'\x01')		#clock source: PLL with X axis gyroscope reference
        self.write_reg(INT_ENABLE, b'\x01')		#enable data_ready interrupt
        print("accel configured")
        self.calibrate()
        
    def calibrate(self):
        count = 0
        g_avg = [0, 0, 0]
        a_avg = [0, 0, 0]
        print("calibrating, don't move the sensor")
        while count < 5000:
            if self.isReady():
                read = self.get_gyro()
                g_avg[0] += read[0]
                g_avg[1] += read[1]
                g_avg[2] += read[2]
                
                read = self.get_accel()
                a_avg[0] += read[0]
                a_avg[1] += read[1]
                a_avg[2] += read[2]
                
                count += 1
                
        g_avg[0] /= 5000
        g_avg[1] /= 5000
        g_avg[2] /= 5000
        
        a_avg[0] /= 5000
        a_avg[1] /= 5000
        a_avg[2] /= 5000
        
        phi = -math.atan2(a_avg[0], math.sqrt(pow(a_avg[2], 2) + pow(a_avg[1], 2)))*180/math.pi
        theta = math.atan2(a_avg[1], math.sqrt(pow(a_avg[2], 2) + pow(a_avg[0], 2)))*180/math.pi
        
        self.angle = [phi, theta, 0]
        self.last = time.ticks_ms()
        self.g_offset = g_avg
        print("calibration complete")
        
    def isReady(self):
        drdy = self.read_reg(INT_STATUS, 1)
        mask = b'\x01'
        return drdy[0]&mask[0] == 1
    
    def get_accel(self):
        out = [0, 0, 0]
        reg = ACCEL_XOUT_H
        
        for i in range(0, 3):
            raw = self.read_reg(reg, 2)
            raw = int.from_bytes(raw, "big")
            if raw >= 32768:					#convert to 2's complement
                raw -= 65536
            out[i] = round(raw*self.acc_res, 2)		#convert to g
            reg = reg + 2
            
        return out
    
    def get_gyro(self):
        out = [0, 0, 0]
        reg = GYRO_XOUT_H
        
        for i in range(0, 3):
            raw = self.read_reg(reg, 2)
            raw = int.from_bytes(raw, "big")
            if raw >= 32768:					#convert to 2's complement
                raw -= 65536
            out[i] = round(raw*self.gyro_res - self.g_offset[i], 2) 		#convert to °/s
            reg = reg + 2
            
        return out
    
    """
    def get_tilt(self):
        read = self.read_accel()
        if read[2] > 1:
            read[2] = 1
        tilt = math.acos(read[2])*180/math.pi
        return tilt
    """
    
    def get_roll(self):
        read = self.get_accel()
        return math.atan2(read[1], math.sqrt(pow(read[2], 2) + pow(read[0], 2)))*180/math.pi
            
    def get_pitch(self):
        read = self.get_accel()
        return -math.atan2(read[0], math.sqrt(pow(read[2], 2) + pow(read[1], 2)))*180/math.pi
                
    def get_angle(self):
        self.update_angle()
        return self.angle