/**
 * @file main.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Example for WisBlock Tool Box to setup LoRa/LoRaWAN
 * @version 0.1
 * @date 2025-04-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "main.h"

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "RAK-TEST";

/** Set firmware version */
uint16_t g_sw_ver_1 = 1; // major version increase on API change / not backwards compatible
uint16_t g_sw_ver_2 = 0; // minor version increase on API change / backward compatible
uint16_t g_sw_ver_3 = 0; // patch version increase on bugfix, no affect on API

/** Timer for frequent packet sending */
time_t last_send;

/**
 * @brief Arduino setup, called once
 *
 */
void setup(void)
{
	// Initialize the built in LED
	pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, LOW);

	// Initialize the BLE status LED
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, LOW);

	// Initialize Serial
	Serial.begin(115200);

	time_t serial_timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - serial_timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
		}
		else
		{
			break;
		}
	}
	digitalWrite(LED_GREEN, HIGH);

	// Get LoRa parameter
	init_flash();

	// Enable BLE
	APP_LOG("SETUP", "Init BLE");

	// Init BLE
	init_ble();

	// If P2P mode, override auto join setting
	if (!g_lorawan_settings.lorawan_enable)
	{
		g_lorawan_settings.auto_join = true;
	}

	// LoRaWAN initialization
	// Check if auto join is enabled
	if (g_lorawan_settings.auto_join)
	{
		// Initialize LoRa and start join request
		int8_t lora_init_result = 0;
		if (g_lorawan_settings.lorawan_enable)
		{
			APP_LOG("SETUP", "Auto join is enabled, start LoRaWAN and join");
			lora_init_result = init_lorawan();
		}
		else
		{
			APP_LOG("API", "Auto join is enabled, start LoRa P2P listen");
			lora_init_result = init_lora();
		}

		if (lora_init_result != 0)
		{
			APP_LOG("SETUP", "Init LoRa failed");
			APP_LOG("SETUP", "Get your LoRa stuff in order");
		}
		else
		{
			APP_LOG("SETUP", "LoRa init success");
		}
	}
	else
	{
		// Put radio into sleep mode
		lora_rak4630_init();
		Radio.Sleep();

		APP_LOG("SETUP", "Auto join is disabled, waiting for connect command");
		delay(100);
	}

	// Keep BLE advertising forever
	restart_advertising(0);

	// Set time for sending a packet
	last_send = millis();
}

/**
 * @brief Arduino loop
 *
 */
void loop(void)
{
	// BLE UART event (AT commands)
	if ((g_task_event_type & BLE_DATA) == BLE_DATA)
	{
		g_task_event_type &= N_BLE_DATA;
		// Send it to AT command parser
		while (g_ble_uart.available() > 0)
		{
			at_serial_input(uint8_t(g_ble_uart.read()));
			delay(5);
		}
		at_serial_input(uint8_t('\n'));
	}

	// BLE config characteristic received
	if ((g_task_event_type & BLE_CONFIG) == BLE_CONFIG)
	{
		g_task_event_type &= N_BLE_CONFIG;
		APP_LOG("LOOP", "Config received over BLE");
		delay(100);

		// Inform connected device about new settings
		g_lora_data.write((void *)&g_lorawan_settings, sizeof(s_lorawan_settings));
		g_lora_data.notify((void *)&g_lorawan_settings, sizeof(s_lorawan_settings));
	}

	// Serial input event (AT commands)
	if ((g_task_event_type & AT_CMD) == AT_CMD)
	{
		g_task_event_type &= N_AT_CMD;
		while (Serial.available() > 0)
		{
			at_serial_input(uint8_t(Serial.read()));
			delay(5);
		}
	}

	// Time to send a packet?
	if (g_lorawan_settings.send_repeat_time != 0)
	{
		if ((millis() - last_send) > g_lorawan_settings.send_repeat_time)
		{
			g_task_event_type &= N_STATUS;

			last_send = millis();
			// Send example data
			// Prepare payload
			uint8_t payload[4] = {0x01, 0x74, 0x01, 0x9f};

			if (g_lorawan_settings.lorawan_enable)
			{
				// LoRaWAN
				if (g_lpwan_has_joined)
				{
					// Device has joined
					lmh_error_status result = send_lora_packet(payload, 4, 2);
					switch (result)
					{
					case LMH_SUCCESS:
						APP_LOG("APP", "Packet enqueued");
						digitalWrite(LED_GREEN, HIGH);
						break;
					case LMH_BUSY:
						APP_LOG("APP", "LoRa transceiver is busy");
						break;
					case LMH_ERROR:
						APP_LOG("APP", "Packet error, too big to send with current DR");
						break;
					}
				}
				else
				{
					APP_LOG("APP", "Network not joined, skip sending");
				}
			}
			else
			{
				if (!send_p2p_packet(payload, 4)) 
				{
					APP_LOG("APP", "P2P send failed, check packet size");
				}
			}
		}
	}
	yield();
}
