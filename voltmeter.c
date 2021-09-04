/*
=== WiFi-enabled voltmeter managed by ESP8266 (esp-12e/f) (RTOS SDK based) ===

Install toolchain for crosscompiling (may not be easy, old esp-open-sdk toolchain works fine despite of warnings):
 - supported toolchain version is 1.22.0-92-g8facf4c
 - supported compiler version 5.2.0

 export PATH=~/esp-open-sdk/xtensa-lx106-elf/bin:$PATH

cd ~
wget https://github.com/espressif/ESP8266_RTOS_SDK/releases/download/v3.3/ESP8266_RTOS_SDK-v3.3.zip
unzip ESP8266_RTOS_SDK-v3.3.zip
mkdir -p ESP8266_RTOS_SDK/voltmeter/main

Review and update defines in voltmeter.c

cp voltmeter.c ESP8266_RTOS_SDK/voltmeter/main
cp ESP8266_RTOS_SDK/examples/get-started/project-template/Makefile ESP8266_RTOS_SDK/voltmeter
Edit ESP8266_RTOS_SDK/voltmeter/Makefile
touch ESP8266_RTOS_SDK/voltmeter/main/component.mk

export IDF_PATH=~/ESP8266_RTOS_SDK

make menuconfig # to produce sdkconfig

Set menuconfig->Component config->PHY->vdd33_const value according to 
https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-reference/peripherals/adc.html
Default is 33 meaning that the actual VCC will be used as a reference voltage.

make

Wiring:
VCC to 3.3V
GND to ground
EN, GPIO2 to 1 via 10K pull up resister.
GPIO15 to 0 via 10K pull down resister.
RX to TX (3.3v levels)
TX to RX (3.3v levels)
RST to GND to reset if needed without a pull-down resistor
ADC to a voltage divider in kOhm range; for example, to measure 12V
I use 200 Ohm from ADC to ground and 3.3kOhm from 12V to ADC, which
gives nominal ratio 16.5 (12V / 16.5 = 0.72V on ADC pin (1.0V is max!). 
To account for the actual ratio, the voltmeter needs a calibration 
using a digital voltmeter to calculate and adjust the nominal ratio - 
see VOLTAGE_DIVIDER_RATIO define below

Power off, ground GPIO0 via 10K pull down resister before flashing, power on

make flash

Power off, unground GPIO0 for normal operation, power on

Run terminal emulator to see the ESP_LOGx(TAG,) messages:
miniterm --raw /dev/ttyAMA1 74880

I (47) boot: ESP-IDF v3.3-dirty 2nd stage bootloader
I (47) boot: compile time 23:45:07
I (48) qio_mode: Enabling default flash chip QIO
I (54) boot: SPI Speed      : 40MHz
I (61) boot: SPI Mode       : QIO
I (67) boot: SPI Flash Size : 4MB
I (73) boot: Partition Table:
I (79) boot: ## Label            Usage          Type ST Offset   Length
I (90) boot:  0 nvs              WiFi data        01 02 00009000 00004000
I (101) boot:  1 otadata          OTA data         01 00 0000d000 00002000
I (113) boot:  2 phy_init         RF data          01 01 0000f000 00001000
I (125) boot:  3 ota_0            OTA app          00 10 00010000 000f0000
I (136) boot:  4 ota_1            OTA app          00 11 00110000 000f0000
I (148) boot: End of partition table
I (154) boot: No factory image, trying OTA 0
I (162) esp_image: segment 0: paddr=0x00010010 vaddr=0x40210010 size=0x40020 (262176) map
I (176) esp_image: segment 1: paddr=0x00050038 vaddr=0x40250030 size=0x080c0 ( 32960) map
I (189) esp_image: segment 2: paddr=0x00058100 vaddr=0x3ffe8000 size=0x005a4 (  1444) load
I (203) esp_image: segment 3: paddr=0x000586ac vaddr=0x40100000 size=0x00a30 (  2608) load
I (217) esp_image: segment 4: paddr=0x000590e4 vaddr=0x40100a30 size=0x057f8 ( 22520) load
I (230) boot: Loaded app from partition at offset 0x10000
I (275) system_api: Base MAC address is not set, read default base MAC address from EFUSE
I (283) system_api: Base MAC address is not set, read default base MAC address from EFUSE
W (289) phy_init: failed to load RF calibration data (0x1102), falling back to full calibration
phy_version: 1159.0, 85b471e, Apr 21 2020, 17:03:08, RTOS new
I (450) phy_init: phy ver: 1159_0
W (453) phy_init: saving new calibration data because of checksum failure, mode(2)
I (466) reset_reason: RTC reset 1 wakeup 0 store 0, reason is 1
I (514) voltmeter: wifi mode 3 means 802.11bg (<54Mbps, 35-140m)
I (2817) wifi: state: 0 -> 2 (b0)
I (2819) wifi: state: 2 -> 3 (0)
I (2822) wifi: state: 3 -> 5 (10)
I (2823) wifi: pm start, type: 1
I (2854) voltmeter: event_handler: connected to ssid beruta-rp3, channel 5, authmode wpa2_psk
I (2855) voltmeter: system IDF version v3.3-dirty
I (2856) voltmeter: free heap size 67948
*/

//#define NDEBUG //to get rid of ESP_ERROR_CHECK asserts and subsequent aborts() which will exit the task loop
			   // and eventually (in 15sec) causes the task to be killed by watchdog timer because the timer is not reset
			   // as the task loop is gone. Perhaps, ESP_ERROR_CHECK is not the correct macro for tasks
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <ctype.h>
#include <netdb.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_system.h"
#include "esp_wifi_types.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "lwip/netif.h"
#include "lwip/netdb.h"
#include "lwip/dhcp.h"
#include "lwip/apps/sntp.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "rom/ets_sys.h"


#define SSID "ssid"
#define PASSPHRASE "password"
//iw wlan0 station dump to check the signal >= -50dBm is good 
// WIFI_POWER values, range is from -128 to 127, level0 - is the maximum power
// from esp_wifi.h:
// [78, 127]: level0
// [76, 77]: level1
// [74, 75] : level2
// [68, 73] : level3
// [60, 67] : level4
// [52, 59] : level5
// [44, 51] : level5 -2dBm
// [34, 43] : level5 -4.5dBm
// [28, 33] : level5 -6dBm
// [20, 27] : level5 -8dBm
// [8, 19] : level5 -11dBm
// [-128, 7] : level5 -14dBm
// https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-reference/wifi/esp_wifi.html gives different numbers
// [82, 127]: level0
// [78, 81] : level1
// [74, 77] : level2
// // [68, 73] : level3
// [64, 67] : level4
// [56, 63] : level5
// [49, 55] : level5 - 2dBm
// [33, 48] : level5 - 6dBm
// [25, 32] : level5 - 8dBm
// [13, 24] : level5 - 11dBm
// [1, 12] : level5 - 14dBm
// [-128, 0] : level5 - 17.5dBm
#define WIFI_POWER 7
//wifi modes: //WIFI_PROTOCOL_11B, WIFI_PROTOCOL_11G, WIFI_PROTOCOL_11N, can be ORed (|)
//Currently only 802.11b or 802.11bg or 802.11bgn modes are supported
#define WIFI_MODE WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G
#define SSID_CHANNEL 5 //optional channel for AP scan
#define HOSTNAME "voltmeter"
//Station static IP config
#define USE_STATIC_IP true
#define STATIC_IP "192.168.1.100"
#define NETMASK "255.255.255.0"
#define GATEWAY_IP "192.168.1.1"
#define NTP0 "0.pool.ntp.org"
#define NTP1 "1.pool.ntp.org"
#define NTP2 "2.pool.ntp.org"
#define DNS1 "8.8.8.8"
#define DNS2 "8.8.4.4"
#define TIMEZONE "NZST-12NZDT-13,M10.1.0,M3.3.0" //for TZ format see man tzset()

//OTA update of esp8266 flash with the new image
#define FW_HTTP_UPDATE true
#define FW_UPDATE_TASK_DELAY_MS 900000
#define FW_UPDATE_TASK_PRIORITY 4
#define FW_UPDATE_TASK_SS 4000
#define WEB_SERVER "192.168.1.1"
#define WEB_SERVER_PORT "80"
#define HTTP_METHOD "GET"
#define HTTP_VERSION "HTTP/1.0" //HTTP/1.1 may need to deal with Transfer-Encoding
#define USER_AGENT "esp8266-voltmeter"
#define HTTP_TIMEOUT 3 //sec
//increase timer version every time you modify this file to avoid re-downloading the same image over and over;
//for example, make it /voltmeter-2.bin but upload the binary to the web server with the previously flashed version, i.e. /timer-1.bin
#define FW_PATH "/voltmeter-2.bin"

#define VOLTMETER_TASK_DELAY_MS 60000 //ms
#define VOLTMETER_TASK_PRIORITY 5
#define VOLTMETER_TASK_SS 2400
// ESP-12e/f range on ADC input is 0.0 - 1.0V
// The ADC channel is 10 bit, so the voltage increment is 1/1023 V
#define MAX_VOLTAGE_MV 1023.0
// The calibrated ratio of resistences of a voltage divider (uncalibrated 16.5)
// Calibration reflects the fact that the actual VCC is not exactly 3.3V
// and that the actual resistence ratio is not exactly 16.5
#define VOLTAGE_DIVIDER_RATIO 17.1949878
// see https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-reference/peripherals/adc.html adc_config_t.clk_div range [8..32]
// ADC sample collection rate=80MHz/clk_div with the max rate of 10MHZ if CLOCK_DIVIER = 8
#define CLOCK_DIVIDER 32
#define REMOTE_LOGGING true
#define REMOTE_LOGGING_IP "192.168.1.1"
#define REMOTE_LOGGING_UDP_PORT 6666
#define MAX_LOG_BUFFER_SIZE 1460 //network packet max payload size
#define LOGGING_TASK_PRIORITY 3
#define LOGGING_TASK_DELAY_MS 1000 //ms, waiting for new log char
#define REMOTE_LOGGING_TASK_SS 1000
#define ESP_CONFIG_TASK_PRIORITY 5
#define ESP_CONFIG_TASK_DELAY_MS 1000
#define ESP_CONFIG_TASK_SS 2000
#define AP_CHECK true 
#define AP_CHECK_TASK_PRIORITY 3
#define AP_CHECK_TASK_DELAY_MS 60000
#define AP_CHECK_TASK_SS 1000

//log high water mark warnings or errors - watch them and adjust task's stack size if needed
#define HIGH_WATER_MARK_WARNING 50
#define HIGH_WATER_MARK_CRITICAL 10

#define RRD_HOST "192.168.1.1"
#define RRD_PORT 13900
#define RRD_DB "voltage.rrd"

static const char *TAG = HOSTNAME;
static uint8_t log_buffer[MAX_LOG_BUFFER_SIZE];
static uint16_t log_buffer_pointer;
static TaskHandle_t esp_config_task_handle = NULL, logging_task_handle = NULL;

int rrd_sock, rrd_port = RRD_PORT;
const char * rrd_host = RRD_HOST;
const char * rrd_db_file = RRD_DB;

int rrd_connect() {
	struct sockaddr_in sock_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(rrd_port) //converts port from host to network byte order
	};
	//converts IP address in dotted notation to struct in_addr, which has s_addr member in network byte order
	int res = inet_aton(rrd_host, &sock_addr.sin_addr);
	if (res == 0) {
		ESP_LOGE(TAG, "rrd_connect: inet_aton() - bad IP address of RRD_HOST");
		return 1;
	}
	//rrdtool listens on rrd_host:13900/tcp
	if (rrd_sock > 0) close(rrd_sock);
	rrd_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (rrd_sock == -1) {
		ESP_LOGE(TAG, "rrd_connect: error creating rrdtool socket");
		return 2;
	}
	res = connect(rrd_sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr));
	if (res == -1) {
		ESP_LOGE(TAG, "rrd_connect: connect() failed");
		if (rrd_sock > 0) close(rrd_sock);
		return 3;
	}
	return 0;
}

int save_data(float voltage) {
	time_t t = time(NULL);

	ESP_LOGI(TAG, "House battery voltage %.3f V\n", voltage);

	char buf[255];
	memset(buf, 0, sizeof(buf));
	snprintf(buf, sizeof(buf), "update %s N:%.3f\r\n", rrd_db_file, voltage);

	int n = write(rrd_sock, buf, strlen(buf));
	if (n == -1) {
		ESP_LOGE(TAG, "%serror writing to rrd: %s\n", asctime(localtime(&t)), strerror(errno));
		return 1;
	}
	return 0;
}

static char * authmode(wifi_auth_mode_t mode) {
	switch (mode) {
	case WIFI_AUTH_OPEN:
		return "open";
	case WIFI_AUTH_WEP:
		return "wep";
	case WIFI_AUTH_WPA_PSK:
		return "wpa_psk";
	case WIFI_AUTH_WPA2_PSK:
		return "wpa2_psk";
	case WIFI_AUTH_WPA_WPA2_PSK:
		return "wpa_wpa2_psk";
	case WIFI_AUTH_WPA2_ENTERPRISE:
		return "wpa2_enterprise";
	case WIFI_AUTH_MAX:
		return "max";
	default:
		return "unknown";
	}
}
static void sysinfo(void) {
	ESP_LOGI(TAG, "system IDF version %s", esp_get_idf_version());
	ESP_LOGI(TAG, "free heap size %u", esp_get_free_heap_size());
}

static void voltmeter_task(void *arg) {
	char datetime[64];
	time_t now;
	struct tm timeinfo;
	UBaseType_t hwm;
	uint16_t voltage;

	while (1) {
		now = 0;
		memset(&timeinfo, 0, sizeof(timeinfo));
		time(&now);
		localtime_r(&now, &timeinfo);
		strftime(datetime, sizeof(datetime), "%a %b %d %H:%M:%S %Y", &timeinfo);
		ESP_LOGI(TAG, "voltmeter_task: datetime %s", datetime);

		esp_err_t status = adc_read(&voltage);
		if (status != ESP_OK)
			ESP_LOGE(TAG, "%s adc_read(&voltage) returned %d\n", asctime(&timeinfo), status);
                if (rrd_connect()) continue;
		save_data(voltage * VOLTAGE_DIVIDER_RATIO / MAX_VOLTAGE_MV);
		if (rrd_sock > 0) close(rrd_sock);

		hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "voltmeter_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "voltmeter_task: uxTaskGetStackHighWaterMark %lu", hwm);

		vTaskDelay(pdMS_TO_TICKS(VOLTMETER_TASK_DELAY_MS));
	}
}

static void fw_update_task(void *arg) {
	int http_status, s, r;
	esp_err_t e;
	uint16_t rand;
	char buf[1460];
	char * body;
	const esp_partition_t * upgrade_partition;
	esp_ota_handle_t ota_handle = NULL;
	const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
		.ai_protocol = IPPROTO_TCP,
	};
	struct addrinfo * res;

	while (1) {
		ESP_LOGI(TAG, "fw_update_task: checking for a new %s image %s...", HOSTNAME, FW_PATH);
		s = -1;
		res = NULL;
		rand = esp_random(); //return uint32_t from 0..UINT32_MAX range
		//normalizing rand to be within 0..1000 range
		rand /= (UINT32_MAX - rand);
		rand = rand * 1000 / (1 + rand);

		int err = getaddrinfo(WEB_SERVER, WEB_SERVER_PORT, &hints, &res);
		if (err != 0) {
			ESP_LOGE(TAG, "fw_update_task: DNS lookup failed - %d", err);
			vTaskDelay(pdMS_TO_TICKS(rand));
			continue;
		}

		s = socket(res->ai_family, res->ai_socktype, 0);
		if (s < 0) {
			ESP_LOGE(TAG, "fw_update_task: socket failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: allocated socket");

		if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
			ESP_LOGE(TAG, "fw_update_task: connect failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: connected");

		struct timeval receiving_timeout;
		receiving_timeout.tv_sec = HTTP_TIMEOUT;
		receiving_timeout.tv_usec = 0;
		if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout, sizeof(receiving_timeout)) < 0) {
			ESP_LOGE(TAG, "fw_update_task: failed to set socket receiving timeout");
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: set socket receiving timeout success");

		char buff[1024];
		int l = sprintf(buff, "%s %s %s\r\nHost: %s:%s\r\nConnection: close\r\nUser-Agent: %s\r\nAccept: %s\r\n\r\n", HTTP_METHOD, FW_PATH, HTTP_VERSION, WEB_SERVER, WEB_SERVER_PORT, USER_AGENT, "application/octet-stream");
		//ESP_LOGI(TAG, "request: sending GET request:\n%s", buff);
		if (write(s, buff, l) < 0) {
			ESP_LOGE(TAG, "fw_update_task: write failed - %s", strerror(errno));
			goto sleep;
		}
		//ESP_LOGI(TAG, "request: socket send success");

		memset(buf, 0, sizeof(buf));
		r = read(s, buf, sizeof(buf));

		if (strncmp(buf, "HTTP/1.", 7) == 0) {
			http_status = atoi(buf + strlen(HTTP_VERSION) + 1);
			if (http_status == 200) {
				upgrade_partition = esp_ota_get_next_update_partition(NULL);
				if (upgrade_partition) {
					e = esp_ota_begin(upgrade_partition, OTA_SIZE_UNKNOWN, &ota_handle);
					if (e != ESP_OK) {
						ESP_LOGE(TAG, "fw_update_task: esp_ota_begin failed with error %u", e);
						goto sleep;
					}
					body = strstr(buf, "\r\n\r\n");
					if (body) {
						body += 4;
						r -= (body - buf);
						if (r > 0) {
							e = esp_ota_write(ota_handle, body, r);
							if (e != ESP_OK) {
								ESP_LOGE(TAG, "fw_update_task: esp_ota_write failed with error %u", e);
								goto sleep;
							}
						}
					}
				}
				else {
					ESP_LOGE(TAG, "fw_update_task: unable to get next update partition");
					goto sleep;
				}
			}
			else if (http_status == 404) {
				ESP_LOGI(TAG, "fw_update_task: no new %s image available. Next check is in %u ms", HOSTNAME, FW_UPDATE_TASK_DELAY_MS);
				goto sleep;
			}
			else {
				ESP_LOGE(TAG, "fw_update_task: http_status %d", http_status);
				goto sleep;
			}
		}
		else {
			ESP_LOGE(TAG, "fw_update_task: not an HTTP packet");
			goto sleep;
		}

		do {
			memset(buf, 0, sizeof(buf));
			r = read(s, buf, sizeof(buf));
			e = esp_ota_write(ota_handle, buf, r);
			if (e != ESP_OK) {
				ESP_LOGE(TAG, "fw_update_task: esp_ota_write failed with error %u", e);
				goto sleep;
			}
		} while (r > 0);
		//ESP_LOGI(TAG, "fw_update_task: done reading from socket. Last read return=%d errno=%d\n", r, errno);		

		e = esp_ota_end(ota_handle);
		if (e != ESP_OK) {
			ESP_LOGE(TAG, "fw_update_task: esp_ota_end failed with error %u", e);
			goto sleep;
		}
		ESP_LOGI(TAG, "fw_update_task: done uploading the new image. Rebooting...\n");
		e = esp_ota_set_boot_partition(upgrade_partition);
		if (e != ESP_OK) {
			ESP_LOGE(TAG, "fw_update_task: esp_ota_set_boot_partition failed with error %u", e);
			goto sleep;
		}
		esp_restart();

	sleep:
		if (res) freeaddrinfo(res);
		if (s >= 0) {
			shutdown(s, SHUT_RDWR);
			close(s);
		}
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "fw_update_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "fw_update_task: uxTaskGetStackHighWaterMark %lu", hwm);

		//1 tick is 10ms as defined by CONFIG_FREERTOS_HZ=100
		//so delaying for 1s translates to 100 ticks, for example
		vTaskDelay(pdMS_TO_TICKS(FW_UPDATE_TASK_DELAY_MS));
	}
}

static void esp_config_task(void *arg)
{
	esp_err_t err;
	uint32_t notificationValue;

	while (1) {
		notificationValue = ulTaskNotifyTake(1, pdMS_TO_TICKS(ESP_CONFIG_TASK_DELAY_MS));
		if (notificationValue) {
			sysinfo();
			ESP_LOGI(TAG, "esp_config_task: setting hostname %s...", HOSTNAME);
			err = tcpip_adapter_set_hostname(WIFI_IF_STA, HOSTNAME);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_hostname failed - %s", esp_err_to_name(err));

			if (USE_STATIC_IP) {
				tcpip_adapter_dhcp_status_t status;
				tcpip_adapter_dhcps_get_status(TCPIP_ADAPTER_IF_STA, &status);
				//ESP_LOGI(TAG, "esp_config_task: tcpip_adapter_dhcps_get_status %u", status);
				if (status != TCPIP_ADAPTER_DHCP_STOPPED) { //tcpip_adapter_dhcps_get_status does not return TCPIP_ADAPTER_DHCP_STARTED but TCPIP_ADAPTER_DHCP_INIT
					ESP_LOGI(TAG, "esp_config_task: USE_STATIC_IP is true, stopping dhcp client...");
					err = tcpip_adapter_dhcpc_stop(WIFI_IF_STA);
					if (err != ESP_OK)
						ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_dhcpc_stop failed - %s", esp_err_to_name(err));
				}
				tcpip_adapter_ip_info_t ip;
				ipaddr_aton(STATIC_IP, &ip.ip);
				ipaddr_aton(GATEWAY_IP, &ip.gw);
				ipaddr_aton(NETMASK, &ip.netmask);

				ESP_LOGI(TAG, "esp_config_task: setting static ip %s...", ipaddr_ntoa(&ip.ip));
				err = tcpip_adapter_set_ip_info(WIFI_IF_STA, &ip);
				if (err != ESP_OK)
					ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_ip_info failed - %s", esp_err_to_name(err));
			}

			tcpip_adapter_dns_info_t prim_dns_ip;
			tcpip_adapter_dns_info_t sec_dns_ip;

			ipaddr_aton(DNS1, &prim_dns_ip.ip);
			ipaddr_aton(DNS2, &sec_dns_ip.ip);
			ESP_LOGI(TAG, "esp_config_task: setting primary dns %s...", DNS1);
			err = tcpip_adapter_set_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &prim_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_dns_info failed - %s", esp_err_to_name(err));
			ESP_LOGI(TAG, "esp_config_task: setting secondary dns %s...", DNS2);
			err = tcpip_adapter_set_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_BACKUP, &sec_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_set_dns_info failed - %s", esp_err_to_name(err));
			err = tcpip_adapter_get_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &prim_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_get_dns_info failed - %s", esp_err_to_name(err));
			err = tcpip_adapter_get_dns_info(WIFI_IF_STA, TCPIP_ADAPTER_DNS_BACKUP, &sec_dns_ip);
			if (err != ESP_OK)
				ESP_LOGE(TAG, "esp_config_task: tcpip_adapter_get_dns_info failed - %s", esp_err_to_name(err));
			//ESP_LOGI(TAG, "dns 1: %s", ipaddr_ntoa(&prim_dns_ip.ip));
			//ESP_LOGI(TAG, "dns 2: %s", ipaddr_ntoa(&sec_dns_ip.ip));

			ESP_LOGI(TAG, "esp_config_task: initializing SNTP...");
			sntp_setoperatingmode(SNTP_OPMODE_POLL);
			sntp_setservername(0, NTP0);
			sntp_setservername(1, NTP1);
			sntp_setservername(2, NTP2);
			sntp_init();

			ESP_LOGI(TAG, "sntp 0: %s", sntp_getservername(0));
			ESP_LOGI(TAG, "sntp 1: %s", sntp_getservername(1));
			ESP_LOGI(TAG, "sntp 2: %s", sntp_getservername(2));

			ESP_LOGI(TAG, "esp_config_task: setting timezone %s...", TIMEZONE);
			setenv("TZ", TIMEZONE, 1);
			tzset();

			rrd_connect();
			
			// init ADC
			adc_config_t adc_config;
			// mode values: ADC_READ_TOUT_MODE = 0, ADC_READ_VDD_MODE, ADC_READ_MAX_MODE
			adc_config.mode = ADC_READ_TOUT_MODE;
			//ADC sample collection clock=80M/clk_div, range[8, 32]
			adc_config.clk_div = CLOCK_DIVIDER;
			if (ESP_OK != adc_init(&adc_config))
				ESP_LOGE(TAG, "esp_config_task: failed to initialize ADC");

			if (pdPASS != xTaskCreate(&voltmeter_task, "voltmeter_task", VOLTMETER_TASK_SS, NULL, VOLTMETER_TASK_PRIORITY, NULL))
				ESP_LOGE(TAG, "esp_config_task: failed to create voltmeter task");
			if (FW_HTTP_UPDATE)
				if (pdPASS != xTaskCreate(&fw_update_task, "fw_update_task", FW_UPDATE_TASK_SS, NULL, FW_UPDATE_TASK_PRIORITY, NULL))
					ESP_LOGE(TAG, "esp_config_task: failed to create fw update task");
		}
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "esp_config_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "esp_config_task: uxTaskGetStackHighWaterMark %lu", hwm);
	}
}

static esp_err_t event_handler(void *ctx, system_event_t *evt)
{
	//ESP_LOGI(TAG,"event %x", evt->event_id);
	char * reason = "";
	esp_err_t err;

	switch (evt->event_id) {
	case SYSTEM_EVENT_WIFI_READY: //should be triggered after esp-wifi_init() but it is not!
		//ESP_LOGI(TAG, "event_handler: wifi is ready");
		break;
	case SYSTEM_EVENT_SCAN_DONE:
		//ESP_LOGI(TAG, "event_handler: scan is done");
		break;
	case SYSTEM_EVENT_STA_START:
		//ESP_LOGI(TAG, "wifi station started");
		err = esp_wifi_set_max_tx_power(WIFI_POWER); //should be called after esp_wifi_start()
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_set_max_tx_power failed - %s", esp_err_to_name(err));

		uint8_t wifi_mode = WIFI_MODE;
		err = esp_wifi_set_protocol(ESP_IF_WIFI_STA, wifi_mode);
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_set_protocol failed - %s", esp_err_to_name(err));
		switch (wifi_mode) {
		case WIFI_PROTOCOL_11B:
			ESP_LOGI(TAG, "wifi mode %u means 802.11b (<11Mbps, 35-140m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11G:
			ESP_LOGI(TAG, "wifi mode %u means 802.11g (<54Mbps, 38-140m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11N:
			ESP_LOGI(TAG, "wifi mode %u means 802.11n (<289/600Mbps, 70-250m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G:
			ESP_LOGI(TAG, "wifi mode %u means 802.11bg (<54Mbps, 35-140m)", WIFI_MODE);
			break;
		case WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N:
			ESP_LOGI(TAG, "wifi mode %u means 802.11bgn (<289/600Mbps, 35-250m)", WIFI_MODE);
			break;
		default:
			ESP_LOGI(TAG, "wifi mode is unknown %u", wifi_mode);
		}

		wifi_config_t wifi_config = {
			.sta = {
				.ssid = SSID,
				.password = PASSPHRASE,
				.bssid_set = 0,
#ifdef SSID_CHANNEL
				.scan_method = WIFI_FAST_SCAN,
				.channel = SSID_CHANNEL
#else
				.scan_method = WIFI_ALL_CHANNEL_SCAN,
				.channel = 0
#endif // SSID_CHANNEL
			},
		};
		//ESP_LOGI(TAG, "Configuring esp for scan method %u, channel %u...", wifi_config.sta.scan_method, wifi_config.sta.channel);
		err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_set_config failed - %s", esp_err_to_name(err));
		//ESP_LOGI(TAG, "connecting to ap %s...", SSID);
		err = esp_wifi_connect();
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_connect failed - %s", esp_err_to_name(err));
		break;
	case SYSTEM_EVENT_STA_STOP:
		ESP_LOGW(TAG, "event_handler: wifi stopped, restarting...");
		esp_restart();
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
		ESP_LOGI(TAG, "event_handler: connected to ssid %s, channel %d, authmode %s",
			evt->event_info.connected.ssid,
			evt->event_info.connected.channel, authmode(evt->event_info.connected.authmode));
		if (USE_STATIC_IP) {
			if (esp_config_task_handle) {
				//ESP_LOGI(TAG, "event_handler: notifying esp_config_task...");
				xTaskNotifyGive(esp_config_task_handle);
			}
			else ESP_LOGE(TAG, "event_handler: esp_config_task_handle is NULL");
		}
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		switch (evt->event_info.disconnected.reason) {
		case WIFI_REASON_UNSPECIFIED: reason = "unknown"; break;
		case WIFI_REASON_AUTH_EXPIRE: reason = "auth expired"; break;
		case WIFI_REASON_AUTH_LEAVE: reason = "left authentication"; break;
		case WIFI_REASON_ASSOC_EXPIRE: reason = "assoc expired"; break;
		case WIFI_REASON_ASSOC_TOOMANY: reason = "too many assoc"; break;
		case WIFI_REASON_NOT_AUTHED: reason = "not authenticated"; break;
		case WIFI_REASON_NOT_ASSOCED: reason = "not associated"; break;
		case WIFI_REASON_ASSOC_LEAVE: reason = "left association"; break;
		case WIFI_REASON_ASSOC_NOT_AUTHED: reason = "association not authenticated"; break;
		case WIFI_REASON_DISASSOC_PWRCAP_BAD: reason = "disassociated - bad pwr cap"; break;
		case WIFI_REASON_DISASSOC_SUPCHAN_BAD: reason = "disassociated - bad sup channel"; break;
		case WIFI_REASON_IE_INVALID: reason = "invalid IE"; break;
		case WIFI_REASON_MIC_FAILURE: reason = "misc failure"; break;
		case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: reason = "4-way handshake timeout"; break;
		case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT: reason = "group key update timeout"; break;
		case WIFI_REASON_IE_IN_4WAY_DIFFERS: reason = "IE in 4-way differs"; break;
		case WIFI_REASON_GROUP_CIPHER_INVALID: reason = "invalid group cipher"; break;
		case WIFI_REASON_PAIRWISE_CIPHER_INVALID: reason = "invalid pairwise cipher"; break;
		case WIFI_REASON_AKMP_INVALID: reason = "invalid AKMP"; break;
		case WIFI_REASON_UNSUPP_RSN_IE_VERSION: reason = "unsupported RSN IE version"; break;
		case WIFI_REASON_INVALID_RSN_IE_CAP: reason = "invalid RSN IE cap"; break;
		case WIFI_REASON_802_1X_AUTH_FAILED: reason = "802.1x auth failed"; break;
		case WIFI_REASON_CIPHER_SUITE_REJECTED: reason = "cipher suite rejected"; break;
		case WIFI_REASON_BEACON_TIMEOUT: reason = "beacon timeout"; break;
		case WIFI_REASON_NO_AP_FOUND: reason = "no AP found"; break;
		case WIFI_REASON_AUTH_FAIL: reason = "authentication failed"; break;
		case WIFI_REASON_ASSOC_FAIL: reason = "association failed"; break;
		case WIFI_REASON_HANDSHAKE_TIMEOUT: reason = "handshake timeout"; break;
		}
		ESP_LOGW(TAG, "event_handler: disconnected from ssid %s, reason %s, re-connecting...",
			evt->event_info.disconnected.ssid, reason);
		//ESP_LOGI(TAG, "connecting to ap %s...", SSID);
		err = esp_wifi_connect();
		if (err != ESP_OK)
			ESP_LOGE(TAG, "event_handler: esp_wifi_connect failed - %s", esp_err_to_name(err));
		break;
	case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
		ESP_LOGW(TAG, "event_handler: station authmode changed: %d -> %d, restarting...",
			evt->event_info.auth_change.old_mode,
			evt->event_info.auth_change.new_mode);
		esp_restart();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		if (!USE_STATIC_IP) {
			ESP_LOGI(TAG, "event_handler: station got ip:" IPSTR ", mask:" IPSTR ", gw:" IPSTR,
				IP2STR(&evt->event_info.got_ip.ip_info.ip),
				IP2STR(&evt->event_info.got_ip.ip_info.netmask),
				IP2STR(&evt->event_info.got_ip.ip_info.gw));
			if (esp_config_task_handle) {
				//ESP_LOGI(TAG, "event_handler: notifying esp_config_task...");
				xTaskNotifyGive(esp_config_task_handle);
			}
			else ESP_LOGE(TAG, "event_handler: esp_config_task_handle is NULL");
		}
		break;
	case SYSTEM_EVENT_STA_LOST_IP:
		ESP_LOGE(TAG, "event_handler: station lost ip, restarting...");
		esp_restart();
		break;
	default:
		ESP_LOGI(TAG, "event_handler: unknown event with id %x", evt->event_id);
		break;
	}
	return ESP_OK;
}

static void sendlog() {
	int sock, err;
	struct sockaddr_in remote_addr = {
		.sin_family = AF_INET,
		.sin_addr.s_addr = inet_addr(REMOTE_LOGGING_IP),
		.sin_port = htons(REMOTE_LOGGING_UDP_PORT)
	};
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock >= 0) {
		err = sendto(sock, log_buffer, log_buffer_pointer, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
		if (err < 0) {
			printf("sendlog: sendto failed - %s\n", strerror(errno));
		}
		else {
			memset(log_buffer, 0, log_buffer_pointer);
			log_buffer_pointer = 0;
		}
		close(sock);
	}
	else printf("sendlog: socket failed - %s, remote logging will not work\n", strerror(errno));
}

static void remote_logging_task(void *arg) {
	uint32_t notificationValue;

	while (1) {
		notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LOGGING_TASK_DELAY_MS));
		if (notificationValue)
			sendlog();
		else if (log_buffer_pointer > 0)
			sendlog();

		UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "remote_logging_task: uxTaskGetStackHighWaterMark %lu", hwm);
	}
}

static int remote_logging(int ch) {
	if (log_buffer_pointer == MAX_LOG_BUFFER_SIZE) {
		xTaskNotifyGive(logging_task_handle);
		putchar(ch);
	}
	else {
		log_buffer[log_buffer_pointer++] = ch;
		if (log_buffer_pointer == MAX_LOG_BUFFER_SIZE)
			xTaskNotifyGive(logging_task_handle);
	}
	return ch;
}

static void ap_check_task(void *arg) {
	wifi_ap_record_t ap_record;
	esp_err_t err;
	UBaseType_t hwm;
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(AP_CHECK_TASK_DELAY_MS));

		err = esp_wifi_sta_get_ap_info(&ap_record);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "ap_check_task: esp_wifi_sta_get_ap_info failed - %s, restarting...", esp_err_to_name(err));
			esp_restart();
		}
		hwm = uxTaskGetStackHighWaterMark(NULL);
		if (hwm <= HIGH_WATER_MARK_CRITICAL)
			ESP_LOGE(TAG, "ap_check_task: uxTaskGetStackHighWaterMark %lu", hwm);
		else if (hwm <= HIGH_WATER_MARK_WARNING)
			ESP_LOGW(TAG, "ap_check_task: uxTaskGetStackHighWaterMark %lu", hwm);
	}
}

void app_main(void) {
	esp_err_t err;
	if (REMOTE_LOGGING) {
		memset(log_buffer, 0, MAX_LOG_BUFFER_SIZE);
		log_buffer_pointer = 0;
		if (pdPASS != xTaskCreate(&remote_logging_task, "logging_task", REMOTE_LOGGING_TASK_SS, NULL, LOGGING_TASK_PRIORITY, &logging_task_handle))
			printf("app_main: failed to create a logging task, remote logging will not work\n");
		else esp_log_set_putchar(remote_logging);
	}

	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
		// OTA app partition table has a smaller NVS partition size than the non-OTA
		// partition table. This size mismatch may cause NVS initialization to fail.
		// If this happens, we erase NVS partition and initialize NVS again.
		err = nvs_flash_erase();
		if (err != ESP_OK)
			ESP_LOGE(TAG, "app_main: nvs_flash_erase failed - %s", esp_err_to_name(err));
		else {
			err = nvs_flash_init();
			if (err != ESP_OK)
				ESP_LOGE(TAG, "app_main: nvs_flash_init2 failed - %s", esp_err_to_name(err));
		}
	}
	else if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: nvs_flash_init1 failed - %s", esp_err_to_name(err));

	tcpip_adapter_init(); //enables tcpip stack, see tcpip_adapter.h

	err = esp_event_loop_init(event_handler, NULL); //see esp_event_loop.h
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_event_loop_init failed - %s", esp_err_to_name(err));

	//ESP_LOGI(TAG, "Calling esp_wifi_init()...");
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	err = esp_wifi_init(&cfg); //init event queue according to esp_wifi.h
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_init failed - %s", esp_err_to_name(err));

	//ESP_LOGI(TAG, "setting esp to station mode...");
	err = esp_wifi_set_mode(WIFI_MODE_STA);
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_set_mode failed - %s", esp_err_to_name(err));

	if (pdPASS != xTaskCreate(&esp_config_task, "esp_config_task", ESP_CONFIG_TASK_SS, NULL, ESP_CONFIG_TASK_PRIORITY, &esp_config_task_handle))
		ESP_LOGE(TAG, "app_main: failed to create esp config task");

	if (AP_CHECK)
		if (pdPASS != xTaskCreate(&ap_check_task, "ap_check_task", AP_CHECK_TASK_SS, NULL, AP_CHECK_TASK_PRIORITY, NULL))
			ESP_LOGE(TAG, "app_main: failed to create AP check task");

	err = esp_wifi_start();
	if (err != ESP_OK)
		ESP_LOGE(TAG, "app_main: esp_wifi_start failed - %s", esp_err_to_name(err));
}
