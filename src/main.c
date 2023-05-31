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

//GPIO初期化
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

	//入力ピン
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

	//出力ピン
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

//I2Cのライト関数(デバイスドライバ, データ, 長さ, スレーブアドレス)
static int write_bytes(const struct device *i2c_dev, uint8_t *data, uint32_t num_bytes, uint8_t slave_addr)
{
	struct i2c_msg msgs;

	//ライトデータセット
	msgs.buf = data;
	msgs.len = num_bytes;
	msgs.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	//I2Cライト実行
	return i2c_transfer(i2c_dev, &msgs, 1, slave_addr);
}

//I2Cの読み込み関数(デバイスドライバ, 先頭アドレス(8bit), データ, 長さ, スレーブアドレス)
static int read_bytes(const struct device *i2c_dev, uint8_t addr, uint8_t *data, uint32_t num_bytes, uint8_t slave_addr)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	//リードアドレスセット
	wr_addr[0] = addr;
	wr_addr[1] = addr;
	msgs[0].buf = wr_addr;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_RESTART;

	//リードデータセット
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	//I2Cリード実行
	return i2c_transfer(i2c_dev, &msgs[0], 2, slave_addr);
}

//I2C初期化
int i2c_init(void) {

	if (!i2c_dev) {
	  printk("device_get_binding i2c failed\n");
	  return -1;
	}
	
	return 0;
}

//温度計測
float measure_temp() {
	uint8_t I2C_BUFF[3]; //I2Cデータバッファ
	int err;

	I2C_BUFF[0] = 0x01; //ポインタレジスタ(コンフィグレーションレジスタセット)
	I2C_BUFF[1] = 0x81; //コンフィグレーションレジスタ(ワンショット有効＆シャットダウンモード有効)
	err = write_bytes(i2c_dev, I2C_BUFF, 2, 0x48); //I2C書き込み2バイト
	if (err != 0) {
		return 99;
	}

	//温度変換完了待ち
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

	//温度計測値読み出し
	err = read_bytes(i2c_dev, 0x00, I2C_BUFF, 2, 0x48);
	if (err != 0) {
		return 99;
	}

	//ビット演算
	uint16_t uv = ((uint16_t)I2C_BUFF[0] << 8) | I2C_BUFF[1]; //計測値を16bit幅に結合
	int16_t v = (int16_t)uv; //符号ありの変数に変換
	v >>= 4; //符号ありの状態で計測値を正しい桁にシフト
	return (float)v/16.0; //1bit解像度0.0625℃のため16で割って小数点ありで戻す
}

//ADC初期化
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

//バッテリー電圧計測
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
	adc_value = adc_value * 1.529411765; // 抵抗分圧

	pm_device_action_run(adc_dev, PM_DEVICE_ACTION_SUSPEND);

	return (int16_t)adc_value;
}

//UART有効無効切り替え(消費電力削減)
static void uart0_set_enable(bool enable)
{
	const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

	if (!device_is_ready(uart_dev)) {
		return;
	}

	pm_device_action_run(uart_dev, enable ? PM_DEVICE_ACTION_RESUME : PM_DEVICE_ACTION_SUSPEND);
}

//ウォッチドックタイマーカウンタ(初期化対象外変数の定義)
volatile uint8_t WDT_call_count __attribute__((section(".noinit.test_wdt")));

//ウォッチドックタイマーコールバック
static void wdt_cb(const struct device *wdt_dev, int channel_id)
{
	ARG_UNUSED(wdt_dev);
	ARG_UNUSED(channel_id);
	WDT_call_count++; //WDT鳴いたら+1
}

//ウォッチドックタイマー初期化
static int wdt_init(void)
{
	int err;

	//送信間隔 +30秒で設定
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

//UDPデータ送信ファンクション
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

	uart0_set_enable(true); //UART有効

	//printk("VIN VOLTAGE: %dmV\n",measure_batt_mv());
	printk("SW0=%d\n",gpio_pin_get_dt(&SW0));
	printk("SW1=%d\n",gpio_pin_get_dt(&SW1));
	printk("SW2=%d\n",gpio_pin_get_dt(&SW2));
	printk("SW3=%d\n",gpio_pin_get_dt(&SW3));

	//DIPスイッチ 2番 センサータイプ
	// ON：ショートタイプMB7388(10m)
	//OFF：ショートタイプMB7389(5m)
	if(gpio_pin_get_dt(&SW2) == 1) {
		setMB7388 = 1; //MB7388(10m)
	} else {
		setMB7388 = 0; //MB7389(5m)
	}

	//DIPスイッチ 3番 センサータイプ
	// ON：ロングタイプMB7051(10m)
	//OFF：ショートタイプMB7389(5m)
	if(gpio_pin_get_dt(&SW3) == 1) {
		setMB7051 = 1; //MB7051(10m)
	} else {
		setMB7051 = 0; //MB7389(5m)
	}

	//センサー距離情報セット
	if(gpio_pin_get_dt(&SW2) == 0 && gpio_pin_get_dt(&SW3) == 0) {
		setSensor10Meter = 0; //5mセンサー
	} else {
		setSensor10Meter = 1; //10mセンサー
	}

	//超音波センサー電源ON
	gpio_pin_set_dt(&WS_POWER, 1); //WS_POWER
	gpio_pin_set_dt(&WA_START, 1); //WS_STAR
	k_msleep(170); //起動メッセージ流れ待ち

	countRetry = 0;
	do {
		printk("Ultrasonic Range Finder Sensing Try.%d\n", countRetry + 1);
		for(a = 0; a < 13; a++) {
			for (i = 0; i < 20; i++) {
				data_wls[a][i] = 0;
			}
		}
		//超音波センサーデータ取得
		for(a = 0; a < 13; a++) {
			i = 0;
			rx_byte = 0;
			count = 0;
			//最終受信文字が改行コードだった場合は次の配列(行)に移る
			while (rx_byte != 13) {
				err = uart_poll_in(uart_dev, &rx_byte); //UART受信データ1文字読み込み。空だった場合は待ち。
				if (err != -1) {
					count = 0;
					if (rx_byte != 'R' && rx_byte != 13) {
						data_wls[a][i] = rx_byte; //UART受信データが'R'もしくは改行コードでなければ1文字格納
						i++;
					}
				} else {
					count++;     //UARTのデータが空だった場合はカウントアップ
					k_msleep(1); //1msスリープ
				}
				//タイムアウト判定 5秒以内にUART受信できなかった場合はタイムアウト
				if (count > 5000) {
					printk("*** Range Finder ERROR\n");
					break;
				}
			}
			//タイムアウト時は配列に-999を書いてブレイク
			if (count > 5000) {
				for (a = 0; a < 13; a++) {
					sprintf(data_wls[a], "-999");
				}
				break;
			}
		}

		//デバッグ用
		for (a = 0; a < 13; a++) {
			printk("UART [%.2d] %s\n", a, data_wls[a]);
		}

		//センサーがロングタイプMB7051(10m)の場合は値を10倍してcmからmmにする
		if (setMB7051 == 1 && atoi(data_wls[0]) != -999) {
			for (a = 8; a < 13; a++) {
				if (atoi(data_wls[a]) > 999) {
					sprintf(data_wls[a],"9999"); //999cm(9990mm以上のときはエラー値に置き換え
				} else {
					sprintf(data_wls[a],"%d", atoi(data_wls[a]) * 10);
				}
				//printk("MB7051 String %s\n", data_wls[a]);
			}
		}

		//計測値エラー判定
		//8,9,10,11,12を計測値として採用
		//5mセンサーのレンジ 300～4999mm
		//10mセンサーのレンジ 500～9998mm
		if (atoi(data_wls[0]) != -999) {
			for (a = 8; a < 13; a++) {
				//5mセンサー
				if (setSensor10Meter == 0 && (atoi(data_wls[a]) < 300 || atoi(data_wls[a]) > 4999)) {
					printk("Sensing error No[%d] = %s\n", a-7, data_wls[a]);
					sprintf(data_wls[a], "-1"); //5mセンサーで300mm以下と4999mm以上はエラー(検出失敗時は5000mm)
				}
				//10mセンサー
				if (setSensor10Meter == 1 && (atoi(data_wls[a]) < 500 || atoi(data_wls[a]) > 9998)) {
					printk("Sensing error No[%d] = %s\n", a-7, data_wls[a]);
					sprintf(data_wls[a], "-1"); //10mセンサーで500mm以下と9998mm以上はエラー(検出失敗時は9999mm)
				}
			}
		}

		countRetry++; //リトライカウンター +1

		//10回以上失敗したらあきらめる
		if (countRetry > 9) {
			countRetry = 11;
			break;
		}

		//計測値エラー数カウント
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

	} while (err >= 3); //3個以上のエラーでリトライ。最低3個の計測値を得る。

	//超音波センサー電源OFF
	gpio_pin_set_dt(&WA_START, 0); //WS_STAR
	gpio_pin_set_dt(&WS_POWER, 0); //WS_POWER

	//XMONITOR情報取得
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
	sprintf(request_tac    , "%.6s" , ResponseData[4]); // TACコード "1010"
	sprintf(request_band   , "%.2s" , ResponseData[6]); // バンド番号 1
	sprintf(request_cell_id, "%.10s", ResponseData[7]); // CELL ID "00E1C13B"
	printk("plmn   : %s\n", request_plmn   );
	printk("tac    : %s\n", request_tac    );
	printk("band   : %s\n", request_band   );
	printk("cell_id: %s\n", request_cell_id);

	//CONEVAL情報取得
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
	//CONEVAL情報取得成功判定
	if (strcmp(ResponseData[0],"0") == 0) {
		sprintf(request_es  , "%.1s", ResponseData[2]); // 電力効率 6
		sprintf(request_rsrp, "%.3s", ResponseData[3]); // 信号受信電力 -17
		sprintf(request_rsrq, "%.3s", ResponseData[4]); // 信号受信品質 -30
		sprintf(request_snr , "%.3s", ResponseData[5]); // 信号ノイズ比 49
	} else {
		printk("AT%%CONEVAL ERROR\n");
		sprintf(request_es  , "0");   // 電力効率 
		sprintf(request_rsrp, "255"); // 信号受信電力
		sprintf(request_rsrq, "255"); // 信号受信品質
		sprintf(request_snr , "127"); // 信号ノイズ比
	}
	printk("es   : %s\n", request_es  );
	printk("rsrp : %s\n", request_rsrp);
	printk("rsrq : %s\n", request_rsrq);
	printk("snr  : %s\n", request_snr );

	//時刻情報取得 文字列例 [+CCLK: "18/12/06,22:10:00+08"]
	err = nrf_modem_at_scanf("AT+CCLK?","+CCLK: \"%20[,:+/0-9]\"", request_cclk);
	printk("AT+CCLK=%s\n",request_cclk);
	if (err != 1){
		sprintf(request_cclk, "-1,-1");
	}

	//ICCID取得
	err = nrf_modem_at_scanf("AT%XICCID","%%XICCID: ""%20[0-9]", request_iccid);
	printk("AT%%XICCID=%s\n",request_iccid);
	if (err != 1) {
		sprintf(request_iccid, "-1");
	}

	//送信文字列生成
	sprintf(buffer, "%.20s,%.19s,%04d,%+06.2f,%.4s,%.4s,%.4s,%.4s,%.4s,%010d,%.2s,%.7s,%.6s,%.10s,%.1s,%.3s,%.3s,%.3s,%1d,%02d",
	                request_cclk,     //時刻 (20文字制限)
	                request_iccid,    //ICCID (19文字制限)
	                measure_batt_mv(),//電源電圧
	                measure_temp(),   //温度
	                data_wls[ 8],     //超音波距離測定1回目 (4文字制限)
	                data_wls[ 9],     //超音波距離測定2回目 (4文字制限)
	                data_wls[10],     //超音波距離測定3回目 (4文字制限)
	                data_wls[11],     //超音波距離測定4回目 (4文字制限)
	                data_wls[12],     //超音波距離測定5回目 (4文字制限)
	                countUDPsend,     //送信回数
	                request_band   ,  //バンド番号 (2文字制限)
	                request_plmn   ,  //PLMN番号 (7文字制限)
	                request_tac    ,  //TACコード (6文字制限)
	                request_cell_id,  //セルID (10文字制限)
	                request_es     ,  //エネルギー効率 (1文字制限)
	                request_rsrp   ,  //RSRP 受信電力 (3文字制限)
	                request_rsrq   ,  //RSRQ 受信可能電力 (3文字制限)
	                request_snr    ,  //SNR  信号ノイズ比 (3文字制限)
	                setSensor10Meter, //超音波センサー種別(0:5m/1:10m)
	                countRetry - 1    //距離測定リトライ回数
	                );
	printk("UDP send data [%s]\n", buffer);
	printk("Transmitting UDP/IP payload of %d bytes to the ", strlen(buffer) + UDP_IP_HEADER_SIZE);
	printk("IP address %s, port number %d\n", CONFIG_UDP_SERVER_ADDRESS_STATIC, CONFIG_UDP_SERVER_PORT);
	printk("WDT call count %d\n", WDT_call_count);
	err = send(client_fd, buffer, strlen(buffer), 0); //UDP送信実行
	if (err < 0) {
		UDP_error_count++; //UDPエラーカウントアップ
		printk("Failed to transmit UDP packet, %d\n", errno);
		printk("UDP error count %d\n", UDP_error_count);
		if (UDP_error_count >= 3) {
			wdt_feed(wdt_dev, wdt_main_channel); //WDTリセット
			NVIC_SystemReset(); //UDP送信エラー3回以上でシステムリセット
		}
	} else {
		UDP_error_count = 0; //UDPエラーカウントリセット
	}

	//COPS情報取得 文字列例[+COPS: 0,2,"44020",7]
	//基地局への接続が確認できなかった場合はシステムリセットする
	nrf_modem_at_scanf("AT+COPS?","+COPS: %14[,\"0-9]", request_cops);
	printk("AT+COPS=%s\n",request_cops);
	if (strcmp(request_cops,"1") == 0 )
	{
		printk("CONNECTION ERROR\n");
		NVIC_SystemReset(); //システムリセット
	} else {
		wdt_feed(wdt_dev, wdt_main_channel); //WDTリセット
		WDT_call_count = 0;
	}

	countUDPsend++; //連続送信回数カウント
	uart0_set_enable(false); //UART停止
	k_work_schedule(&server_transmission_work, K_SECONDS(CONFIG_UDP_DATA_UPLOAD_FREQUENCY_SECONDS)); //次のUDP送信をスケジュールに追加
}

//定期送信スレッド初期化
static void work_init(void)
{
	k_work_init_delayable(&server_transmission_work, server_transmission_work_fn);
	printk("work_init done.\n");
}

//LTEハンドラ
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

//LTE link controller 初期化
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
	/* eDRX使用時にコメントアウト解除
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

//UDP切断
static void server_disconnect(void)
{
	(void)close(client_fd);
}

//UDPサーバ初期化
static int server_init(void)
{
	struct sockaddr_in *server4 = ((struct sockaddr_in *)&host_addr);
	printk("server_init start\n");
	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_UDP_SERVER_PORT);

	inet_pton(AF_INET, CONFIG_UDP_SERVER_ADDRESS_STATIC, &server4->sin_addr);

	return 0;
}

//UDP接続
static int server_connect(void)
{
	int err;

	printk("server_connect start\n");
	client_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); //ソケット
	if (client_fd < 0) {
		printk("Failed to create UDP socket: %d\n", errno);
		err = -errno;
		server_disconnect();
		return err;
	}

	err = connect(client_fd, (struct sockaddr *)&host_addr, sizeof(struct sockaddr_in)); //指定IPに接続
	if (err < 0) {
		printk("Connect failed : %d\n", errno);
		server_disconnect();
		return err;
	}

	return 0;
}

//LTE接続手続きATコマンド群
static int modem_connect(void)
{
    int err = 0;
    char setPLMN[32] = {0};

	//ATコマンド確認
	printk("AT\n");
	err = nrf_modem_at_printf("AT");
	printk("responce %d\n", err);
	if (err) {
		printk(" *** AT failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//APN設定
	printk("\nAPN setting\n");
	printk("AT+CGDCONT=1,%s,%s\n","\"IP\"","\"sakura\"");
	err = nrf_modem_at_printf("AT+CGDCONT=1,%s,%s\n","\"IP\"","\"sakura\"");
	if (err) {
		printk(" *** AT+CGDCONT failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//LTE cat.M1 固定
	printk("\nLTE-M setting\n");
	printk("AT%%XSYSTEMMODE=1,0,0,1\n");
	err = nrf_modem_at_printf("AT%%XSYSTEMMODE=1,0,0,1");
	if (err) {
		printk(" *** AT%%XSYSTEMMODE failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//ネットワーク切断
	printk("\nDis connecting to network\n");
	printk("AT+CFUN=0\n");
	err = nrf_modem_at_printf("AT+CFUN=0");
	if (err) {
		printk(" *** AT+CFUN failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//接続先キャリア設定 PLMNセット
	printk("\nPLMN setting\n");
	
	if(gpio_pin_get_dt(&SW0) == 0 && gpio_pin_get_dt(&SW1) == 0) {
		sprintf(setPLMN, "AT+COPS=1,2,%s\n","\"44020\""); //ソフトバンク
	}
	else if (gpio_pin_get_dt(&SW0) == 1 && gpio_pin_get_dt(&SW1) == 0) {
		sprintf(setPLMN, "AT+COPS=1,2,%s\n","\"44010\""); //ドコモ
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

	//UICCサスペンド設定 SIMのサスペンドどリジュームを有効化
	printk("\nUICC suspend and deactivate control\n");
	printk("AT+SSRDA=1,1,0\n");
	err = nrf_modem_at_printf("AT+SSRDA=1,1,0");
	if (err) {
		printk(" *** AT+SSRDA failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//LTE接続開始
	printk("\nConnecting to network\n");
	printk("AT+CFUN=1\n");
	err = nrf_modem_at_printf("AT+CFUN=1");
	if (err) {
		printk(" *** AT+CFUN failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//CPUパワーレベル設定　最小電力にセット
	printk("\nModem power level setting (Ultra-low power)\n");
	printk("AT%%XDATAPRFL=0\n");
	err = nrf_modem_at_printf("AT%%XDATAPRFL=0");
	if (err) {
		printk(" *** AT%%XDATAPRFL failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//PSM設定
	printk("\nPower saving mode setting\n");
	printk("AT+CPSMS=1,\"\",\"\",\"00100110\",\"00000000\"\n");
	err = nrf_modem_at_printf("AT+CPSMS=1,\"\",\"\",\"00100110\",\"00000000\"");
	if (err) {
		printk(" *** AT+CPSMS failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//PTW設定
	printk("\nPaging Time Window (PTW) setting\n");
	printk("AT%%XPTW=4,\"1111\"\n");
	err = nrf_modem_at_printf("AT%%XPTW=4,\"1111\"");
	if (err) {
		printk(" *** AT%%XPTW failed\n");
		printk("AT command error, type: %d\n\n", nrf_modem_at_err_type(err));
	}

	//eDRX設定
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

	wdt_init(); //WDT初期化

	//ウォッチドックが鳴いた回数で接続試行にウェイトをかける
	printk("WDT count %d\n", WDT_call_count);
	if (WDT_call_count > 4) {
		WDT_call_count = 0;
	}
	switch(WDT_call_count){
		case 0: wdtSleepMin = 0; printk("none sleep\n");break;
		case 1: wdtSleepMin = 1; printk("sleep %dmin\n",wdtSleepMin); break; //1分
		case 2: wdtSleepMin = 2; printk("sleep %dmin\n",wdtSleepMin); break; //2分
		case 3: wdtSleepMin = 4; printk("sleep %dmin\n",wdtSleepMin); break; //4分
		case 4: wdtSleepMin = 8; printk("sleep %dmin\n",wdtSleepMin); break; //8分
		default:wdtSleepMin = 0; break;
	}
	for (countSleepMin = wdtSleepMin; countSleepMin > 0; countSleepMin--) {
		if (countSleepMin > 10) {
			break; //10分を超える値がセットされていた場合はループを抜ける
		}
		wdt_feed(wdt_dev, wdt_main_channel);//WDTカウンタリセット
		printk("%d minute sleep remaining\n", countSleepMin);
		k_sleep(K_SECONDS(60)); //1分スリープ
	}

	gpio_init();     //GPIO初期化
	adc_init();      //ADC初期化
	i2c_init();      //I2C初期化
	work_init();     //UDP送信スレッド初期化
	modem_init();    //LTEモデム初期化
	modem_connect(); //LTE接続用ATコマンド発行

	err = k_sem_take(&lte_connected, K_SECONDS(30)); //指定時間接続完了待ち
	if (err == -EAGAIN) 
	{
		printk("\n*** CONNECTION TIMEOUT!!\n");
		WDT_call_count++; //WDTカウンタを加算
		NVIC_SystemReset(); //接続タイムアウトでシステムリセット
	}

	wdt_feed(wdt_dev, wdt_main_channel);//WDTカウンタリセット
	WDT_call_count = 0;

	err = server_init(); //UDPサーバ初期化
	printk("server_init status %d\n",err);
	if (err) {
		printk("Not able to initialize UDP server connection\n");
		return;
	}

	err = server_connect(); //UDPコネクト
	printk("server_connect status %d\n",err);
	if (err) {
		printk("Not able to connect to UDP server\n");
		return;
	}

	//UDP定期送信スレッド実行
	k_work_schedule(&server_transmission_work, K_NO_WAIT);
}
