/*
 * Copyright (c) 2023 SAKURA internet Inc.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <nrf_modem.h>
#include <nrf_modem_at.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/net/socket.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/watchdog.h>
#include <modem/lte_lc.h>

#define UDP_IP_HEADER_SIZE 28

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0)); //UART

static const struct gpio_dt_spec ST_A_LED = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec ST_B_LED = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec WA_START = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec WS_POWER = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static const struct gpio_dt_spec ADV_EN   = GPIO_DT_SPEC_GET(DT_ALIAS(led4), gpios);
static const struct gpio_dt_spec SW0      = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec SW1      = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec SW2      = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec SW3      = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc)); //ADC
static const struct adc_channel_cfg battsence_channel_cfg = 
                     ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_0)); //ADC

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2)); //I2C

static int wdt_main_channel;
static const struct device *const wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt)); //WDT

static int client_fd;
static struct sockaddr_storage host_addr;
static struct k_work_delayable server_transmission_work;

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(cereg_sem, 0, 1);

//GPIO������
static int gpio_init(void)
{
    int ret;

	if (!device_is_ready(SW0.port) ||
	    !device_is_ready(SW1.port) ||
	    !device_is_ready(SW2.port) ||
	    !device_is_ready(SW3.port) ||
	    !device_is_ready(ST_A_LED.port) ||
	    !device_is_ready(ST_B_LED.port) ||
	    !device_is_ready(WA_START.port) ||
	    !device_is_ready(WS_POWER.port) ||
	    !device_is_ready(ADV_EN.port)) {
		printk("\n **** GPIO initialize error ****\n");
	}

	//���̓s��
	ret = gpio_pin_configure_dt(&SW0, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		printk("\n **** Configure SW0 pin failed (%d) ****",ret);
	}
	ret = gpio_pin_configure_dt(&SW1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		printk("\n **** Configure SW1 pin failed (%d) ****",ret);
	}
	ret = gpio_pin_configure_dt(&SW2, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		printk("\n **** Configure SW2 pin failed (%d) ****",ret);
	}
	ret = gpio_pin_configure_dt(&SW3, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		printk("\n **** Configure SW3 pin failed (%d) ****",ret);
	}

	//�o�̓s��
	ret = gpio_pin_configure_dt(&ST_A_LED, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		printk("\n **** Configure ST_A_LED pin failed (%d) ****",ret);
	}
	ret = gpio_pin_configure_dt(&ST_B_LED, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		printk("\n **** Configure ST_B_LED pin failed (%d) ****",ret);
	}
	ret = gpio_pin_configure_dt(&WA_START, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		printk("\n **** Configure WA_START pin failed (%d) ****",ret);
	}
	ret = gpio_pin_configure_dt(&WS_POWER, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		printk("\n **** Configure WS_POWER pin failed (%d) ****",ret);
	}
	ret = gpio_pin_configure_dt(&ADV_EN, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		printk("\n **** Configure ADV_EN pin failed (%d) ****",ret);
	}

    ret = gpio_pin_set_dt(&ST_A_LED, 0); //ST_A_LED
	if (ret) {
		printk("\n **** Set LED 0 pin failed (%d) ****",ret);
	}
    ret = gpio_pin_set_dt(&ST_B_LED, 0); //ST_B_LED
	if (ret) {
		printk("\n **** Set LED 1 pin failed (%d) ****",ret);
	}
    ret = gpio_pin_set_dt(&WA_START, 0);//WA_START
	if (ret) {
		printk("\n **** Set LED 2 pin failed (%d) ****",ret);
	}
    ret = gpio_pin_set_dt(&WS_POWER, 0);//WS_POWER
	if (ret) {
		printk("\n **** Set LED 3 pin failed (%d) ****",ret);
	}
    ret = gpio_pin_set_dt(&ADV_EN, 0); //ADV_EN
	if (ret) {
		printk("\n **** Set LED 4 pin failed (%d) ****",ret);
	}

    return 0;
}

//I2C�̃��C�g�֐�(�f�o�C�X�h���C�o, �f�[�^, ����, �X���[�u�A�h���X)
static int write_bytes(const struct device *i2c_dev, uint8_t *data, uint32_t num_bytes, uint8_t slave_addr)
{
	struct i2c_msg msgs;

	//���C�g�f�[�^�Z�b�g
	msgs.buf = data;
	msgs.len = num_bytes;
	msgs.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	//I2C���C�g���s
	return i2c_transfer(i2c_dev, &msgs, 1, slave_addr);
}

//I2C�̓ǂݍ��݊֐�(�f�o�C�X�h���C�o, �擪�A�h���X(8bit), �f�[�^, ����, �X���[�u�A�h���X)
static int read_bytes(const struct device *i2c_dev, uint8_t addr, uint8_t *data, uint32_t num_bytes, uint8_t slave_addr)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	//���[�h�A�h���X�Z�b�g
	wr_addr[0] = addr;
	wr_addr[1] = addr;
	msgs[0].buf = wr_addr;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_RESTART;

	//���[�h�f�[�^�Z�b�g
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	//I2C���[�h���s
	return i2c_transfer(i2c_dev, &msgs[0], 2, slave_addr);
}

//I2C������
int i2c_init(void) {

	if (!i2c_dev) {
	  printk("device_get_binding i2c failed\n");
	  return -1;
	}
	
	return 0;
}

//���x�v��
float measure_temp() {
	uint8_t I2C_BUFF[3]; //I2C�f�[�^�o�b�t�@
	int err;

	I2C_BUFF[0] = 0x01; //�|�C���^���W�X�^(�R���t�B�O���[�V�������W�X�^�Z�b�g)
	I2C_BUFF[1] = 0x81; //�R���t�B�O���[�V�������W�X�^(�����V���b�g�L�����V���b�g�_�E�����[�h�L��)
	err = write_bytes(i2c_dev, I2C_BUFF, 2, 0x48); //I2C��������2�o�C�g
	if (err != 0) {
		return 99;
	}

	//���x�ϊ������҂�
	err = read_bytes(i2c_dev, 0x01, I2C_BUFF, 1, 0x48);
	if (err != 0) {
		return 99;
	}

	while (I2C_BUFF[0] != 0x01)
	{
		err = read_bytes(i2c_dev, 0x01, I2C_BUFF, 1, 0x48);
	}
	if (err != 0) {
		return 99;
	}

	//���x�v���l�ǂݏo��
	err = read_bytes(i2c_dev, 0x00, I2C_BUFF, 2, 0x48);
	if (err != 0) {
		return 99;
	}

	//�r�b�g���Z
	uint16_t uv = ((uint16_t)I2C_BUFF[0] << 8) | I2C_BUFF[1]; //�v���l��16bit���Ɍ���
	int16_t v = (int16_t)uv; //��������̕ϐ��ɕϊ�
	v >>= 4; //��������̏�ԂŌv���l�𐳂������ɃV�t�g
	return (float)v/16.0; //1bit�𑜓x0.0625���̂���16�Ŋ����ď����_����Ŗ߂�
}

//ADC������
int adc_init(void) {
	int err;
	if (!adc_dev) {
		printk("device_get_binding adc failed\n");
		return -1;
	}
	err = adc_channel_setup(adc_dev, &battsence_channel_cfg);
	if (err) {
		printk("Error in adc setup: %d\n", err);
	return -1;
	}
	return 0;
}

//�o�b�e���[�d���v��
int16_t measure_batt_mv() {
	int err;
	int16_t a_sample_buffer[10] = {0};
	int16_t s_sample_buffer = 0;
	int a;

	pm_device_action_run(adc_dev, PM_DEVICE_ACTION_RESUME);

	int16_t m_sample_buffer;
	const struct adc_sequence sequence = {
		.channels = BIT(7),
		.buffer = &m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = 12,
		.oversampling = 8,
		.calibrate = 1,
	};

	if (!adc_dev) {
		return -1;
	}

	gpio_pin_set_dt(&ADV_EN, 1); //ADV_enable
	for (a = 0; a < 10; a++) {
		//k_msleep(1);
		err = adc_read(adc_dev, &sequence);
		a_sample_buffer[a] = m_sample_buffer;
	}
	gpio_pin_set_dt(&ADV_EN, 0); //ADV desable
	if (err) {
		printk("ADC read err: %d\n", err);
	return -1;
	}

	for (a = 0; a < 10; a++) {
		//printk("a_sample_buffer[%d]: %d\n", a, a_sample_buffer[a]);
		s_sample_buffer = s_sample_buffer + a_sample_buffer[a];
	}
	s_sample_buffer = s_sample_buffer / 10;
	//printk("s_sample_buffer: %d\n", s_sample_buffer);

	double adc_value = (double)s_sample_buffer;
	adc_value = adc_value * 3600.0d; // MAX3.6V
	adc_value = adc_value / 4095.0d; // 12bit
	adc_value = adc_value * 1.529411765; // ��R����

	pm_device_action_run(adc_dev, PM_DEVICE_ACTION_SUSPEND);

	return (int16_t)adc_value;
}

//UART�L�������؂�ւ�(����d�͍팸)
static void uart0_set_enable(bool enable)
{
	const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

	if (!device_is_ready(uart_dev)) {
		return;
	}

	pm_device_action_run(uart_dev, enable ? PM_DEVICE_ACTION_RESUME : PM_DEVICE_ACTION_SUSPEND);
}

//�E�H�b�`�h�b�N�^�C�}�[�J�E���^(�������ΏۊO�ϐ��̒�`)
volatile uint8_t WDT_call_count __attribute__((section(".noinit.test_wdt")));

//�E�H�b�`�h�b�N�^�C�}�[�R�[���o�b�N
static void wdt_cb(const struct device *wdt_dev, int channel_id)
{
	ARG_UNUSED(wdt_dev);
	ARG_UNUSED(channel_id);
	WDT_call_count++; //WDT������+1
}

//�E�H�b�`�h�b�N�^�C�}�[������
static int wdt_init(void)
{
	int err;

	//���M�Ԋu +30�b�Őݒ�
	static struct wdt_timeout_cfg wdt_cfg = {
	    .window.max = (CONFIG_UDP_DATA_UPLOAD_FREQUENCY_SECONDS + 30) * 1000,
	    .callback = wdt_cb,
	    .flags = WDT_FLAG_RESET_SOC,
	};

	if (!device_is_ready(wdt_dev)) {
		printk("\n*** WDT device is not ready\n");
		return -1;
	}

	wdt_main_channel = wdt_install_timeout(wdt_dev, &wdt_cfg);
	if (wdt_main_channel < 0) {
		printk("\n*** Could not install wdt timeout. (error: %d)\n", wdt_main_channel);
		return wdt_main_channel;
	}

	err = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (err < 0) {
		printk("\n*** Could not setup WDT\n");
		return err;
	}
	printk("\nWatchdog set up with a %d sec interval\n", wdt_cfg.window.max / 1000);

	return 0;
}

//UDP�f�[�^���M�t�@���N�V����
uint32_t countUDPsend = 1;
uint8_t UDP_error_count = 0;
static void server_transmission_work_fn(struct k_work *work)
{
	int err;
	char ResponseBuffer[128] = {0};
	char buffer[256] = {"\0"};
	char request_iccid[32] = {0};
	char request_cclk[21] = {0};
	char request_cops[15] = {0};
	int count = 0;
	char rx_byte;
	char data_wls[13][20] = {{0},{0}};
	int countRetry = 0;
	int a,i = 0;
	char ResponseData[17][12];
	char *ResponsePt;

	char request_plmn[8] = {0};
	char request_tac[7] = {0};
	char request_band[3] = {0};
	char request_cell_id[11] = {0};
	char request_es[2] = {0};
	char request_rsrp[4] = {0};
	char request_rsrq[4] = {0};
	char request_snr[4] = {0};
	char setMB7388 = 0;
	char setMB7051 = 0;
	char setSensor10Meter = 0;

	uart0_set_enable(true); //UART�L��

	//printk("VIN VOLTAGE: %dmV\n",measure_batt_mv());
	printk("SW0=%d\n",gpio_pin_get_dt(&SW0));
	printk("SW1=%d\n",gpio_pin_get_dt(&SW1));
	printk("SW2=%d\n",gpio_pin_get_dt(&SW2));
	printk("SW3=%d\n",gpio_pin_get_dt(&SW3));

	//DIP�X�C�b�` 2�� �Z���T�[�^�C�v
	// ON�F�V���[�g�^�C�vMB7388(10m)
	//OFF�F�V���[�g�^�C�vMB7389(5m)
	if(gpio_pin_get_dt(&SW2) == 1) {
		setMB7388 = 1; //MB7388(10m)
	} else {
		setMB7388 = 0; //MB7389(5m)
	}

	//DIP�X�C�b�` 3�� �Z���T�[�^�C�v
	// ON�F�����O�^�C�vMB7051(10m)
	//OFF�F�V���[�g�^�C�vMB7389(5m)
	if(gpio_pin_get_dt(&SW3) == 1) {
		setMB7051 = 1; //MB7051(10m)
	} else {
		setMB7051 = 0; //MB7389(5m)
	}

	//�Z���T�[�������Z�b�g
	if(gpio_pin_get_dt(&SW2) == 0 && gpio_pin_get_dt(&SW3) == 0) {
		setSensor10Meter = 0; //5m�Z���T�[
	} else {
		setSensor10Meter = 1; //10m�Z���T�[
	}

	//�����g�Z���T�[�d��ON
	gpio_pin_set_dt(&WS_POWER, 1); //WS_POWER
	gpio_pin_set_dt(&WA_START, 1); //WS_STAR
	k_msleep(170); //�N�����b�Z�[�W����҂�

	countRetry = 0;
	do {
		printk("Ultrasonic Range Finder Sensing Try.%d\n", countRetry + 1);
		for(a = 0; a < 13; a++) {
			for (i = 0; i < 20; i++) {
				data_wls[a][i] = 0;
			}
		}
		//�����g�Z���T�[�f�[�^�擾
		for(a = 0; a < 13; a++) {
			i = 0;
			rx_byte = 0;
			count = 0;
			//�ŏI��M���������s�R�[�h�������ꍇ�͎��̔z��(�s)�Ɉڂ�
			while (rx_byte != 13) {
				err = uart_poll_in(uart_dev, &rx_byte); //UART��M�f�[�^1�����ǂݍ��݁B�󂾂����ꍇ�͑҂��B
				if (err != -1) {
					count = 0;
					if (rx_byte != 'R' && rx_byte != 13) {
						data_wls[a][i] = rx_byte; //UART��M�f�[�^��'R'�������͉��s�R�[�h�łȂ����1�����i�[
						i++;
					}
				} else {
					count++;     //UART�̃f�[�^���󂾂����ꍇ�̓J�E���g�A�b�v
					k_msleep(1); //1ms�X���[�v
				}
				//�^�C���A�E�g���� 5�b�ȓ���UART��M�ł��Ȃ������ꍇ�̓^�C���A�E�g
				if (count > 5000) {
					printk("*** Range Finder ERROR\n");
					break;
				}
			}
			//�^�C���A�E�g���͔z���-999�������ău���C�N
			if (count > 5000) {
				for (a = 0; a < 13; a++) {
					sprintf(data_wls[a], "-999");
				}
				break;
			}
		}

		//�f�o�b�O�p
		for (a = 0; a < 13; a++) {
			printk("UART [%.2d] %s\n", a, data_wls[a]);
		}

		//�Z���T�[�������O�^�C�vMB7051(10m)�̏ꍇ�͒l��10�{����cm����mm�ɂ���
		if (setMB7051 == 1 && atoi(data_wls[0]) != -999) {
			for (a = 8; a < 13; a++) {
				if (atoi(data_wls[a]) > 999) {
					sprintf(data_wls[a],"9999"); //999cm(9990mm�ȏ�̂Ƃ��̓G���[�l�ɒu������
				} else {
					sprintf(data_wls[a],"%d", atoi(data_wls[a]) * 10);
				}
				//printk("MB7051 String %s\n", data_wls[a]);
			}
		}

		//�v���l�G���[����
		//8,9,10,11,12���v���l�Ƃ��č̗p
		//5m�Z���T�[�̃����W 300�`4999mm
		//10m�Z���T�[�̃����W 500�`9998mm
		if (atoi(data_wls[0]) != -999) {
			for (a = 8; a < 13; a++) {
				//5m�Z���T�[
				if (setSensor10Meter == 0 && (atoi(data_wls[a]) < 300 || atoi(data_wls[a]) > 4999)) {
					printk("Sensing error No[%d] = %s\n", a-7, data_wls[a]);
					sprintf(data_wls[a], "-1"); //5m�Z���T�[��300mm�ȉ���4999mm�ȏ�̓G���[(���o���s����5000mm)
				}
				//10m�Z���T�[
				if (setSensor10Meter == 1 && (atoi(data_wls[a]) < 500 || atoi(data_wls[a]) > 9998)) {
					printk("Sensing error No[%d] = %s\n", a-7, data_wls[a]);
					sprintf(data_wls[a], "-1"); //10m�Z���T�[��500mm�ȉ���9998mm�ȏ�̓G���[(���o���s����9999mm)
				}
			}
		}

		countRetry++; //���g���C�J�E���^�[ +1

		//10��ȏ㎸�s�����炠����߂�
		if (countRetry > 9) {
			countRetry = 11;
			break;
		}

		//�v���l�G���[���J�E���g
		err = 0;
		for (a = 8; a < 13; a++) {
			//printk("Sensing Data No[%d] = %s\n", a-7, data_wls[a]);
			if (atoi(data_wls[a]) < 0) {
				err++;
			}
		}

		printk("error count %d\n", err);
		if (err >= 3) {
			printk("Sensing ERROR\n");
		} else {
			printk("Sensing OK\n");
		}

	} while (err >= 3); //3�ȏ�̃G���[�Ń��g���C�B�Œ�3�̌v���l�𓾂�B

	//�����g�Z���T�[�d��OFF
	gpio_pin_set_dt(&WA_START, 0); //WS_STAR
	gpio_pin_set_dt(&WS_POWER, 0); //WS_POWER

	//XMONITOR���擾
	nrf_modem_at_scanf("AT%XMONITOR","%%XMONITOR: %120[ ,-\"a-zA-Z0-9]", ResponseBuffer);
	printk("AT%%XMONITOR=%s\n",ResponseBuffer);
	ResponsePt = strtok(ResponseBuffer,",");
	sprintf(ResponseData[0], "%s", ResponsePt);
	a = 1;
	while (ResponsePt != NULL)
	{
		ResponsePt = strtok(NULL,",");
		if(ResponsePt == NULL) {break;}
		sprintf(ResponseData[a], "%s", ResponsePt);
		a++;
		if(a >= 17) {break;}
	}
	sprintf(request_plmn   , "%.7s" , ResponseData[3]); // PLMN "44020"
	sprintf(request_tac    , "%.6s" , ResponseData[4]); // TAC�R�[�h "1010"
	sprintf(request_band   , "%.2s" , ResponseData[6]); // �o���h�ԍ� 1
	sprintf(request_cell_id, "%.10s", ResponseData[7]); // CELL ID "00E1C13B"
	printk("plmn   : %s\n", request_plmn   );
	printk("tac    : %s\n", request_tac    );
	printk("band   : %s\n", request_band   );
	printk("cell_id: %s\n", request_cell_id);

	//CONEVAL���擾
	nrf_modem_at_scanf("AT%CONEVAL","%%CONEVAL: %120[ ,-\"a-zA-Z0-9]", ResponseBuffer);
	printk("AT%%CONEVAL=%s\n",ResponseBuffer);
	ResponsePt = strtok(ResponseBuffer,",");
	sprintf(ResponseData[0], "%s", ResponsePt);
	a = 1;
	while (ResponsePt != NULL)
	{
		ResponsePt = strtok(NULL,",");
		if(ResponsePt == NULL) {break;}
		sprintf(ResponseData[a], "%s", ResponsePt);
		a++;
		if(a >= 17) {break;}
	}
	//CONEVAL���擾��������
	if (strcmp(ResponseData[0],"0") == 0) {
		sprintf(request_es  , "%.1s", ResponseData[2]); // �d�͌��� 6
		sprintf(request_rsrp, "%.3s", ResponseData[3]); // �M����M�d�� -17
		sprintf(request_rsrq, "%.3s", ResponseData[4]); // �M����M�i�� -30
		sprintf(request_snr , "%.3s", ResponseData[5]); // �M���m�C�Y�� 49
	} else {
		printk("AT%%CONEVAL ERROR\n");
		sprintf(request_es  , "0");   // �d�͌��� 
		sprintf(request_rsrp, "255"); // �M����M�d��
		sprintf(request_rsrq, "255"); // �M����M�i��
		sprintf(request_snr , "127"); // �M���m�C�Y��
	}
	printk("es   : %s\n", request_es  );
	printk("rsrp : %s\n", request_rsrp);
	printk("rsrq : %s\n", request_rsrq);
	printk("snr  : %s\n", request_snr );

	//�������擾 ������� [+CCLK: "18/12/06,22:10:00+08"]
	err = nrf_modem_at_scanf("AT+CCLK?","+CCLK: \"%20[,:+/0-9]\"", request_cclk);
	printk("AT+CCLK=%s\n",request_cclk);
	if (err != 1){
		sprintf(request_cclk, "-1,-1");
	}

	//ICCID�擾
	err = nrf_modem_at_scanf("AT%XICCID","%%XICCID: ""%20[0-9]", request_iccid);
	printk("AT%%XICCID=%s\n",request_iccid);
	if (err != 1) {
		sprintf(request_iccid, "-1");
	}

	//���M�����񐶐�
	sprintf(buffer, "%.20s,%.19s,%04d,%+06.2f,%.4s,%.4s,%.4s,%.4s,%.4s,%010d,%.2s,%.7s,%.6s,%.10s,%.1s,%.3s,%.3s,%.3s,%1d,%02d",
	                request_cclk,     //���� (20��������)
	                request_iccid,    //ICCID (19��������)
	                measure_batt_mv(),//�d���d��
	                measure_temp(),   //���x
	                data_wls[ 8],     //�����g��������1��� (4��������)
	                data_wls[ 9],     //�����g��������2��� (4��������)
	                data_wls[10],     //�����g��������3��� (4��������)
	                data_wls[11],     //�����g��������4��� (4��������)
	                data_wls[12],     //�����g��������5��� (4��������)
	                countUDPsend,     //���M��
	                request_band   ,  //�o���h�ԍ� (2��������)
	                request_plmn   ,  //PLMN�ԍ� (7��������)
	                request_tac    ,  //TAC�R�[�h (6��������)
	                request_cell_id,  //�Z��ID (10��������)
	                request_es     ,  //�G�l���M�[���� (1��������)
	                request_rsrp   ,  //RSRP ��M�d�� (3��������)
	                request_rsrq   ,  //RSRQ ��M�\�d�� (3��������)
	                request_snr    ,  //SNR  �M���m�C�Y�� (3��������)
	                setSensor10Meter, //�����g�Z���T�[���(0:5m/1:10m)
	                countRetry - 1    //�������胊�g���C��
	                );
	printk("UDP send data [%s]\n", buffer);
	printk("Transmitting UDP/IP payload of %d bytes to the ", strlen(buffer) + UDP_IP_HEADER_SIZE);
	printk("IP address %s, port number %d\n", CONFIG_UDP_SERVER_ADDRESS_STATIC, CONFIG_UDP_SERVER_PORT);
	printk("WDT call count %d\n", WDT_call_count);
	err = send(client_fd, buffer, strlen(buffer), 0); //UDP���M���s
	if (err < 0) {
		UDP_error_count++; //UDP�G���[�J�E���g�A�b�v
		printk("Failed to transmit UDP packet, %d\n", errno);
		printk("UDP error count %d\n", UDP_error_count);
		if (UDP_error_count >= 3) {
			wdt_feed(wdt_dev, wdt_main_channel); //WDT���Z�b�g
			NVIC_SystemReset(); //UDP���M�G���[3��ȏ�ŃV�X�e�����Z�b�g
		}
	} else {
		UDP_error_count = 0; //UDP�G���[�J�E���g���Z�b�g
	}

	//COPS���擾 �������[+COPS: 0,2,"44020",7]
	//��n�ǂւ̐ڑ����m�F�ł��Ȃ������ꍇ�̓V�X�e�����Z�b�g����
	nrf_modem_at_scanf("AT+COPS?","+COPS: %14[,\"0-9]", request_cops);
	printk("AT+COPS=%s\n",request_cops);
	if (strcmp(request_cops,"1") == 0 )
	{
		printk("CONNECTION ERROR\n");
		NVIC_SystemReset(); //�V�X�e�����Z�b�g
	} else {
		wdt_feed(wdt_dev, wdt_main_channel); //WDT���Z�b�g
		WDT_call_count = 0;
	}

	countUDPsend++; //�A�����M�񐔃J�E���g
	uart0_set_enable(false); //UART��~
	k_work_schedule(&server_transmission_work, K_SECONDS(CONFIG_UDP_DATA_UPLOAD_FREQUENCY_SECONDS)); //����UDP���M���X�P�W���[���ɒǉ�
}

//������M�X���b�h������
static void work_init(void)
{
	k_work_init_delayable(&server_transmission_work, server_transmission_work_fn);
	printk("work_init done.\n");
}

//LTE�n���h��
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		printk("Network registration status: %s\n",evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming\n");
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		printk("PSM parameter update: TAU: %d, Active time: %d\n",evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f\n",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			printk("%s\n", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		printk("RRC mode: %s\n",evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle\n");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d\n",evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}

//LTE link controller ������
static void modem_init(void)
{
	int err;

	//Initializes the module and configures the modem. 
	err = lte_lc_init();
	if (err) {
		printk("Modem initialization failed, error: %d\n", err);
		return;
	}

	//Power Saving Mode Enable
	err = lte_lc_psm_req(true);
	if (err) {
		printk("lte_lc_psm_req, error: %d\n", err);
	}

	//Enhanced Discontinuous Reception Enable
	/* eDRX�g�p���ɃR�����g�A�E�g����
	err = lte_lc_edrx_req(true);
	if (err) {
		printk("lte_lc_edrx_req, error: %d\n", err);
	}

	//Release Assistance Indication Enable
	err = lte_lc_rai_req(true);
	if (err) {
		printk("lte_lc_rai_req, error: %d\n", err);
	}
	*/

}

//UDP�ؒf
static void server_disconnect(void)
{
	(void)close(client_fd);
}

//UDP�T�[�o������
static int server_init(void)
{
	struct sockaddr_in *server4 = ((struct sockaddr_in *)&host_addr);
	printk("server_init start\n");
	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_UDP_SERVER_PORT);

	inet_pton(AF_INET, CONFIG_UDP_SERVER_ADDRESS_STATIC, &server4->sin_addr);

	return 0;
}

//UDP�ڑ�
static int server_connect(void)
{
	int err;

	printk("server_connect start\n");
	client_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); //�\�P�b�g
	if (client_fd < 0) {
		printk("Failed to create UDP socket: %d\n", errno);
		err = -errno;
		server_disconnect();
		return err;
	}

	err = connect(client_fd, (struct sockaddr *)&host_addr, sizeof(struct sockaddr_in)); //�w��IP�ɐڑ�
	if (err < 0) {
		printk("Connect failed : %d\n", errno);
		server_disconnect();
		return err;
	}

	return 0;
}

//LTE�ڑ��葱��AT�R�}���h�Q
static int modem_connect(void)
{
    int err = 0;
    char setPLMN[32] = {0};

	//AT�R�}���h�m�F
	printk("AT\n");
	err = nrf_modem_at_printf("AT");
	printk("responce %d\n", err);
	if (err) {
		printk(" *** AT failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//APN�ݒ�
	printk("\nAPN setting\n");
	printk("AT+CGDCONT=1,%s,%s\n","\"IP\"","\"sakura\"");
	err = nrf_modem_at_printf("AT+CGDCONT=1,%s,%s\n","\"IP\"","\"sakura\"");
	if (err) {
		printk(" *** AT+CGDCONT failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//LTE cat.M1 �Œ�
	printk("\nLTE-M setting\n");
	printk("AT%%XSYSTEMMODE=1,0,0,1\n");
	err = nrf_modem_at_printf("AT%%XSYSTEMMODE=1,0,0,1");
	if (err) {
		printk(" *** AT%%XSYSTEMMODE failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//�l�b�g���[�N�ؒf
	printk("\nDis connecting to network\n");
	printk("AT+CFUN=0\n");
	err = nrf_modem_at_printf("AT+CFUN=0");
	if (err) {
		printk(" *** AT+CFUN failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//�ڑ���L�����A�ݒ� PLMN�Z�b�g
	printk("\nPLMN setting\n");
	
	if(gpio_pin_get_dt(&SW0) == 0 && gpio_pin_get_dt(&SW1) == 0) {
		sprintf(setPLMN, "AT+COPS=1,2,%s\n","\"44020\""); //�\�t�g�o���N
	}
	else if (gpio_pin_get_dt(&SW0) == 1 && gpio_pin_get_dt(&SW1) == 0) {
		sprintf(setPLMN, "AT+COPS=1,2,%s\n","\"44010\""); //�h�R��
	}
	else if (gpio_pin_get_dt(&SW0) == 0 && gpio_pin_get_dt(&SW1) == 1) {
		sprintf(setPLMN, "AT+COPS=1,2,%s\n","\"44051\""); //KDDI
	}
	else {
		sprintf(setPLMN, "AT+COPS=1,2,%s\n","\"44020\"");
	}

	printk("%s", setPLMN);
	err = nrf_modem_at_printf("%s", setPLMN);
	if (err) {
		printk(" *** AT+COPS failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//UICC�T�X�y���h�ݒ� SIM�̃T�X�y���h�ǃ��W���[����L����
	printk("\nUICC suspend and deactivate control\n");
	printk("AT+SSRDA=1,1,0\n");
	err = nrf_modem_at_printf("AT+SSRDA=1,1,0");
	if (err) {
		printk(" *** AT+SSRDA failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//LTE�ڑ��J�n
	printk("\nConnecting to network\n");
	printk("AT+CFUN=1\n");
	err = nrf_modem_at_printf("AT+CFUN=1");
	if (err) {
		printk(" *** AT+CFUN failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//CPU�p���[���x���ݒ�@�ŏ��d�͂ɃZ�b�g
	printk("\nModem power level setting (Ultra-low power)\n");
	printk("AT%%XDATAPRFL=0\n");
	err = nrf_modem_at_printf("AT%%XDATAPRFL=0");
	if (err) {
		printk(" *** AT%%XDATAPRFL failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//PSM�ݒ�
	printk("\nPower saving mode setting\n");
	printk("AT+CPSMS=1,\"\",\"\",\"00100110\",\"00000000\"\n");
	err = nrf_modem_at_printf("AT+CPSMS=1,\"\",\"\",\"00100110\",\"00000000\"");
	if (err) {
		printk(" *** AT+CPSMS failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//PTW�ݒ�
	printk("\nPaging Time Window (PTW) setting\n");
	printk("AT%%XPTW=4,\"1111\"\n");
	err = nrf_modem_at_printf("AT%%XPTW=4,\"1111\"");
	if (err) {
		printk(" *** AT%%XPTW failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//eDRX�ݒ�
	printk("\neDRX setting\n");
	printk("AT+CEDRXS=2,4,\"1010\"\n");
	err = nrf_modem_at_printf("AT+CEDRXS=2,4,\"1010\"");
	if (err) {
		printk(" *** AT+CEDRXS failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	err = lte_lc_connect_async(lte_handler);
	printk("\nlte_lc_connect_async status %d\n",err);
	if (err) {
		printk("Connecting to LTE network failed, error: %d\n",err);
	}

    return err;
}

//**** main ****//
void main(void)
{
	int err;
	int countSleepMin = 0;
	int wdtSleepMin = 0;

	if (!device_is_ready(uart_dev)) {
		printk("\n**** UART device not ready ****\n");
	}

	printk("\n\n------ LTE Water Level Gauge v1.0.0 ------\n");
	printk(    "--- Development is SAKURA internet Inc.---\n");

	wdt_init(); //WDT������

	//�E�H�b�`�h�b�N�������񐔂Őڑ����s�ɃE�F�C�g��������
	printk("WDT count %d\n", WDT_call_count);
	if (WDT_call_count > 4) {
		WDT_call_count = 0;
	}
	switch(WDT_call_count){
		case 0: wdtSleepMin = 0; printk("none sleep\n");break;
		case 1: wdtSleepMin = 1; printk("sleep %dmin\n",wdtSleepMin); break; //1��
		case 2: wdtSleepMin = 2; printk("sleep %dmin\n",wdtSleepMin); break; //2��
		case 3: wdtSleepMin = 4; printk("sleep %dmin\n",wdtSleepMin); break; //4��
		case 4: wdtSleepMin = 8; printk("sleep %dmin\n",wdtSleepMin); break; //8��
		default:wdtSleepMin = 0; break;
	}
	for (countSleepMin = wdtSleepMin; countSleepMin > 0; countSleepMin--) {
		if (countSleepMin > 10) {
			break; //10���𒴂���l���Z�b�g����Ă����ꍇ�̓��[�v�𔲂���
		}
		wdt_feed(wdt_dev, wdt_main_channel);//WDT�J�E���^���Z�b�g
		printk("%d minute sleep remaining\n", countSleepMin);
		k_sleep(K_SECONDS(60)); //1���X���[�v
	}

	gpio_init();     //GPIO������
	adc_init();      //ADC������
	i2c_init();      //I2C������
	work_init();     //UDP���M�X���b�h������
	modem_init();    //LTE���f��������
	modem_connect(); //LTE�ڑ��pAT�R�}���h���s

	err = k_sem_take(&lte_connected, K_SECONDS(30)); //�w�莞�Ԑڑ������҂�
	if (err == -EAGAIN) 
	{
		printk("\n*** CONNECTION TIMEOUT!!\n");
		WDT_call_count++; //WDT�J�E���^�����Z
		NVIC_SystemReset(); //�ڑ��^�C���A�E�g�ŃV�X�e�����Z�b�g
	}

	wdt_feed(wdt_dev, wdt_main_channel);//WDT�J�E���^���Z�b�g
	WDT_call_count = 0;

	err = server_init(); //UDP�T�[�o������
	printk("server_init status %d\n",err);
	if (err) {
		printk("Not able to initialize UDP server connection\n");
		return;
	}

	err = server_connect(); //UDP�R�l�N�g
	printk("server_connect status %d\n",err);
	if (err) {
		printk("Not able to connect to UDP server\n");
		return;
	}

	//UDP������M�X���b�h���s
	k_work_schedule(&server_transmission_work, K_NO_WAIT);
}
