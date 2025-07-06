#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Sensor_lib/sensors.h"
#include <math.h>
#include <chrono>

int main(){
	stdio_init_all();
	
	
	
	i2c_init(i2c0, 400*1000);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
    
    sleep_ms(3000);
    printf("\n---------SONO SVEGLIO---------\n");
    sleep_ms(2000);	
	
	MPU6050 gyracc = MPU6050(i2c0);
	while(!gyracc.config()){
		printf("\nCouldn't properly configure the MPU6050, check connections\n");
		printf("Retrying in 10 seconds");
		for(int i=5;i>0;i--){
			sleep_ms(2000);
			printf(".");
		}
		printf("\n");
	}
	
	sleep_ms(1000);
	
	printf("Starting calibration...");
	
	gyracc.calibrate();
	
	printf("MPU6050 initialized!\n");
	
	while(true){
		gyracc.update();
		printf("A: [%.2f %.2f %.2f]g, G:[%.2f %.2f %.2f]°/s\n", gyracc.accel[0], gyracc.accel[1], gyracc.accel[2], gyracc.gyro[0], gyracc.gyro[1], gyracc.gyro[2]);
		sleep_ms(100);
	}
	
	/*BMP280 baro = BMP280(i2c0);
	
	while(!baro.config()){
		printf("\nCouldn't properly configure the BMP280, check connections\n");
		printf("Retrying in 10 seconds");
		for(int i=5;i>0;i--){
			sleep_ms(2000);
			printf(".");
		}
	}
	
	printf("- BMP280 configured correctly\n");
	
	QMC5883L mag = QMC5883L(i2c0);
	
	while(!mag.config()){
		printf("\nCouldn't properly configure the QMC5883L, check connections\n");
		printf("Retrying in 10 seconds");
		for(int i=5;i>0;i--){
			sleep_ms(2000);
			printf(".");
		}
	}
	
	printf("- QMC5883L configured correctly\n");
	
	double *mr;
	auto start = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::high_resolution_clock::now() - start;
	long long microseconds;
	
	while(true){
		//baro.update();
		//gyracc.update();
		//mag.update();0
		mag.update();
		mr = mag.get_field();
		elapsed = std::chrono::high_resolution_clock::now() - start;
		std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		printf("[%.2f %.2f %.2f]G\t|G|: %.2f\tHDN: %.2f\tt: %lld\n", mr[0], mr[1], mr[2], sqrt(mr[0]*mr[0]+mr[1]*mr[1]+mr[2]*mr[2]), mag.get_hdn(), std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
		//printf("pressure: %.2fPa, temp: %.2f°C, alt: %.2fm\n", baro.get_pressure(), baro.get_temp(), baro.get_rel_alt());
	}
	*/
	return 0;
}
