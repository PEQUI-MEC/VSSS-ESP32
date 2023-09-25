#ifndef IMU_H
#define IMU_H

#include "imu_regs.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <array>

struct ImuData2D {
	float gyro_z_rate;
	float mag_z_theta;
	float mag_x;
	float mag_y;
};

struct ImuData {
	std::array<float, 3> gyro;
	std::array<float, 3> acc;
	std::array<float, 3> mag;
};

struct mag_components {
	float x;
	float y;
};

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

class IMU {
		int addr_gyro_acc;
		int addr_comp;

	public:
		IMU(i2c_port_t i2c_num, int sda_pin, int scl_pin);
		ImuData2D read_imu_data(bool use_mag);
		ImuData get_data();
		float read_mag();
		mag_components read_mag_components();
		float read_gyro();
		void read_acc_all(int16_t *data);
		void read_gyro_all(int16_t *data);
		void read_comp_all(int16_t *data);
		void read_reg(int addr, uint8_t reg, uint8_t *data, int num_bytes);
		void write_reg(int addr, uint8_t reg, uint8_t data);
		float gyro_scale = 1.16;
};

#endif

