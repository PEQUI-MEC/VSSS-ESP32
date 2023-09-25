#include "freertos/FreeRTOS.h"

#include "imu.h"
#include <math.h> 

#define PI 3.1415926f

#define MAX_GYRO (2000.0 * (PI)/180) // 2000 dps para rad/s
#define MAX_MAG 4.0f
#define MAX_ACC (4 * 9.80665f) // 4g para m/s^2
#define MAG_X_OFF 0.571852f
#define MAG_Y_OFF 0.132321f
#define MAG_X_MAX 0.174501f
#define MAG_Y_MAX 0.182615f

#define MAG_X_OFF_a 0.4261779f
#define MAG_Y_OFF_a 0.1183533f
#define MAG_X_MAX_a 0.1886219f
#define MAG_Y_MAX_a 0.1955043f

#define MAG_X_OFF_e 0.4261779f
#define MAG_Y_OFF_e 0.1183533f
#define MAG_X_MAX_e 0.1886219f
#define MAG_Y_MAX_e 0.1955043f

#define MAG_X_OFF_b 1.0294917f
#define MAG_Y_OFF_b (-0.24693936f)
#define MAG_X_MAX_b 0.1941387f
#define MAG_Y_MAX_b 0.2118146f

IMU::IMU(i2c_port_t i2c_num, int sda_pin, int scl_pin) {
	addr_gyro_acc = 0b1101011;
	addr_comp = 0b0011110;

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sda_pin;
	conf.scl_io_num = scl_pin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = 0;

    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

	// Habilita acelerometro nos 3 eixos
	write_reg(addr_gyro_acc, CTRL9_XL, 0x38);

	// Acc no modo de alta performance, 1.66kHz, 4g max
	write_reg(addr_gyro_acc, CTRL1_XL, 0x88);

	// Habilita giroscopio nos 3 eixos
	write_reg(addr_gyro_acc, CTRL10_C, 0x38);

	// Habilita giroscopio no eixo z
	// write_reg(addr_gyro_acc, CTRL10_C, 0x20);

	// Gyro no modo de alta performance, 1.66KHz, 2000dps max
	write_reg(addr_gyro_acc, CTRL2_G, 0x7C);

//	Habilita BDU - Atualiza LSB e MSB juntos
	write_reg(addr_gyro_acc, CTRL3_C, 0x44);

	// Habilita bussola
	write_reg(addr_comp, LIS3MDL_CTRL_REG2, 0x00); // 0x00 = 4 gauss
	write_reg(addr_comp, LIS3MDL_CTRL_REG1, 0xFC); // 0xFC = 80Hz, high performance
	write_reg(addr_comp, LIS3MDL_CTRL_REG4, 0x0C); // 0x0C = Z axis high performance
	write_reg(addr_comp, LIS3MDL_CTRL_REG3, 0x00); // 0x00 = continuous conversion mode

	// Esp wait 100ms
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

#define offset_gyro 0.035373f
#define new_offset 0.0435021f
#define gyro_off_a 0.0266207f
ImuData2D IMU::read_imu_data(bool use_mag) {
	ImuData2D data{};
	int16_t gyro_data, mag_data[2];

	read_reg(addr_gyro_acc, OUTZ_L_G, (uint8_t *) &gyro_data, 2);
//	data.gyro_z_rate = ((gyro_data * MAX_GYRO/INT16_MAX) + gyro_off_a)*(237611.9f/205838.1f) - 0.016999742f;
	data.gyro_z_rate = (gyro_data * MAX_GYRO/INT16_MAX)*(237611.9f/205838.1f) + 0.02063384f;

	if(use_mag) {
		read_reg(addr_comp, LIS3MDL_OUT_X_L, (uint8_t *) &mag_data, 4);
//		data.mag_x = (mag_data[0] * MAX_MAG / INT16_MAX + MAG_X_OFF) / MAG_X_MAX;
//		data.mag_y = (mag_data[1] * MAX_MAG / INT16_MAX + MAG_Y_OFF) / MAG_Y_MAX;
		data.mag_x = (mag_data[0] * MAX_MAG / INT16_MAX);
		data.mag_y = (mag_data[1] * MAX_MAG / INT16_MAX);
//		data.mag_z_theta = std::atan2(-data.mag_y, data.mag_x);
	}

	return data;
}

// magnetometer is disabled
ImuData IMU::get_data() {
	ImuData data{};
	int16_t gyro_acc_data[6], mag_data[3];
	read_reg(addr_gyro_acc, OUTX_L_G, (uint8_t *) &gyro_acc_data, 12);
	// read_reg(addr_comp, LIS3MDL_OUT_X_L, (uint8_t *) &mag_data, 6);
	for (int i = 0; i < 3; ++i) {
		data.gyro[i] = gyro_acc_data[i] * (MAX_GYRO/INT16_MAX);
		data.acc[i] = gyro_acc_data[i+3] * (MAX_ACC/INT16_MAX);
		// data.mag[i] = mag_data[i] * (MAX_MAG/INT16_MAX);
	}
	return data;
}

// Preenche o vetor data com os valores lidos do registrador, num_bytes indica quantos bytes devem ser lidos
void IMU::read_reg(int addr, uint8_t reg, uint8_t *data, int num_bytes) {
    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, data, num_bytes, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
}

// Escreve em um registrador
void IMU::write_reg(int addr, uint8_t reg, uint8_t data) {
	uint8_t cmd[2];
	cmd[0] = reg;
	cmd[1] = data;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
}

