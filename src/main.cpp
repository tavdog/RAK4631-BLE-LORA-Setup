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
// Function declarations for WS85 setup and loop
void ws85_setup();
void ws85_loop();

// Variables for WS85-related data
static double dir_sum_sin = 0;
static double dir_sum_cos = 0;
static float velSum = 0;
static float gust = 0;
static float lull = -1;
static int velCount = 0;
static int dirCount = 0;

static float batVoltageF = 0;
static float capVoltageF = 0;
static float temperatureF = 0;
static float rain = 0;
static int rainSum = 0;
static bool initialSendDone = false;

// Define constants
//   _____ _   _ _______ ______ _______      __     _
//  |_   _| \ | |__   __|  ____|  __ \ \    / /\   | |
//    | | |  \| |  | |  | |__  | |__) \ \  / /  \  | |
//    | | | . ` |  | |  |  __| |  _  / \ \/ / /\ \ | |
//   _| |_| |\  |  | |  | |____| | \ \  \  / ____ \| |____
//  |_____|_| \_|  |_|  |______|_|  \_\  \/_/    \_\______|

#define SEND_INTERVAL 1 // minutes

// Timer for sending data
unsigned long lastSendTime = 0;
unsigned long send_interval_ms = SEND_INTERVAL * 60000;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "WS8X";

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
	// flash_reset();
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
	if (g_lorawan_settings.auto_join || 1)
	{
		// Initialize LoRa and start join request
		int8_t lora_init_result = 0;
		if (g_lorawan_settings.lorawan_enable or 1)
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

	ws85_setup();
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
	
	ws85_loop();
	yield();
}

// Function to set up the WS85 serial communication
void ws85_setup()
{
	Serial1.begin(115200); // Initialize Serial1 for WS85 communication
	Serial.println("WS85 setup complete.");
}

// Function to handle WS85-related logic in the loop
void ws85_loop()
{
	const int maxIterations = 100; // Maximum number of lines to read per loop
	int iterationCount = 0;

	// Check if data is available on the serial port
	while (Serial1.available() > 0 && iterationCount < maxIterations)
	{
		iterationCount++;
		String line = Serial1.readStringUntil('\n');
		line.trim();
#ifdef PRINT_WX_SERIAL
		Serial.println(line);
#else
		// Serial.print('.');
#endif
		if (line.length() > 0)
		{
			// Find the '=' character
			int index = line.indexOf('=');
			if (index != -1)
			{
				// Split into key and value, removing whitespace
				String key = line.substring(0, index);
				String value = line.substring(index + 1);
				key.trim();
				value.trim();

				// Remove 'V' suffix from voltage readings if present
				if (value.endsWith("V"))
				{
					value = value.substring(0, value.length() - 1);
				}

				// Parse based on the key
				if (key == "WindDir")
				{
					float windDir = value.toFloat();
					double radians = windDir * M_PI / 180.0;
					dir_sum_sin += sin(radians);
					dir_sum_cos += cos(radians);
					dirCount++;
				}
				else if (key == "WindSpeed")
				{
					float windSpeed = value.toFloat();
					velSum += windSpeed;
					velCount++;
					if (lull == -1 || windSpeed < lull)
						lull = windSpeed;
				}
				else if (key == "WindGust")
				{
					float windGust = value.toFloat();
					if (windGust > gust)
						gust = windGust;
				}
				else if (key == "BatVoltage")
				{
					batVoltageF = value.toFloat();
				}
				else if (key == "CapVoltage")
				{
					capVoltageF = value.toFloat();
				}
				else if (key == "GXTS04Temp" || key == "Temperature")
				{ // Handle both sensor types
					if (value != "--")
					{ // Check for valid temperature
						temperatureF = value.toFloat();
					}
				}
				// Add more parsing logic here if needed
			}
		}
	}

	if (iterationCount >= maxIterations)
	{
		Serial.println("Maximum serial reading iterations reached");
	}

	// Check if it's time to send data
	if (lmh_join_status_get() != LMH_SET) {
		APP_LOG("WS85LOOP", "not joined, not sending");
		return;
	}
		if (millis() - lastSendTime >= send_interval_ms)
		{
			lastSendTime = millis();

			// After first send, switch to normal interval
			if (!initialSendDone)
			{
				send_interval_ms = SEND_INTERVAL * 60000;
				initialSendDone = true;
				Serial.printf("Switching to normal send interval: %lu minutes\n", send_interval_ms / 60000);
			}

			// Calculate averages
			float velAvg = (velCount > 0) ? velSum / velCount : 0;
			double avgSin = (dirCount > 0) ? dir_sum_sin / dirCount : 0;
			double avgCos = (dirCount > 0) ? dir_sum_cos / dirCount : 0;
			double avgRadians = atan2(avgSin, avgCos);
			float dirAvg = avgRadians * 180.0 / M_PI; // Convert to degrees
			if (dirAvg < 0)
				dirAvg += 360.0;

			// Print data
			Serial.printf("Wind Speed Avg: %.1f m/s, Wind Dir Avg: %d°, Gust: %.1f m/s, Lull: %.1f m/s\n",
						  velAvg, (int)dirAvg, gust, lull);
			Serial.printf("Battery Voltage: %.1f V, Capacitor Voltage: %.1f V, Temperature: %.1f °C\n",
						  batVoltageF, capVoltageF, temperatureF);
			Serial.printf("Rain: %.1f mm, Rain Sum: %d\n", rain, rainSum);

			// Reset counters
			dir_sum_sin = dir_sum_cos = 0; // Reset wind direction sums
			velSum = 0;					   // Reset wind speed sum
			velCount = dirCount = 0;	   // Reset wind direction and speed counts
			gust = 0;					   // Reset gust
			lull = -1;					   // Reset lull

			// Reset other metrics
			batVoltageF = 0;  // Reset battery voltage
			capVoltageF = 0;  // Reset capacitor voltage
			temperatureF = 0; // Reset temperature
			rain = 0;		  // Reset rain
			rainSum = 0;	  // Reset rain sum
		}
}

uint8_t boardGetBatteryLevel(void)
{
	float voltage = (analogRead(BATTERY_PIN) * REAL_VBAT_MV_PER_LSB) / 1000.0; // Convert to volts

	// Define voltage range for battery level
	const float MAX_VOLTAGE = 4.2; // Maximum LiPo voltage
	const float MIN_VOLTAGE = 3.0; // Minimum LiPo voltage

	if (voltage > MAX_VOLTAGE)
	{
		return 254; // Max level (254 as per LoRaWAN spec)
	}
	else if (voltage < MIN_VOLTAGE)
	{
		return 1; // Min level (1 as per LoRaWAN spec)
	}

	// Calculate percentage (1-254 range)
	uint8_t level = 1 + ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 253;
	return level;
	// float raw;

	// // Get the raw 12-bit, 0..3000mV ADC value
	// raw = analogRead(BATTERY_PIN);

	// // Convert the raw value to compensated mv, taking the resistor-
	// // divider into account (providing the actual LIPO voltage)
	// // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
	// return (uint8_t)(raw * REAL_VBAT_MV_PER_LSB *2.55);
}