# nanoMODBUS - A compact MODBUS RTU/TCP C library for embedded/microcontrollers
STM32 modbus example with [nanoModbus Library](https://github.com/debevv/nanoMODBUS)

## Installation

1. Configure the USART settings (Baud Rate, 8, N, 1) and enable the global interrupt for USART.
2. Copy `nanomodbus.c` and `nanomodbus.h` inside your application codebase.
3. #include "nanomodbus.h"
4. Create the Modbus client.
```c
    nmbs_platform_conf platform_conf;
    nmbs_platform_conf_create(&platform_conf);
    platform_conf.transport = NMBS_TRANSPORT_RTU;
    platform_conf.read = uart_read;
    platform_conf.write = uart_write;
    nmbs_t nmbs;
    nmbs_error err = nmbs_client_create(&nmbs, &platform_conf);
    if (err != NMBS_ERROR_NONE) {
        Error_Handler();
    }
//Set the timeout values.
	nmbs_set_byte_timeout(&nmbs, 100);
	nmbs_set_read_timeout(&nmbs, 1000);
//Set the target slave address (e.g., 1).
	nmbs_set_destination_rtu_address(&nmbs, 1);
```
## Transport read/write
-> Define the pins for the receiver/transmitter modes of the RS485 module.
```C
void RS485_SetTransmitMode(void) {
	HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port, RS485_CTRL_Pin, GPIO_PIN_SET); // DE ve RE pin set.
}

void RS485_SetReceiveMode(void) {
	HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port, RS485_CTRL_Pin, GPIO_PIN_RESET); // DE ve RE pin reset.
}
int32_t uart_read(uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg) {
	RS485_SetReceiveMode(); // Switch to receiver mode.
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, buf, count,
			timeout_ms);
	if (status == HAL_OK)
		return count;
	return -1;
}

int32_t uart_write(const uint8_t *buf, uint16_t count, int32_t timeout_ms,
		void *arg) {
	RS485_SetTransmitMode(); // Switch to transmit mode.
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*) buf, count,
			timeout_ms);
	if (status == HAL_OK) {
		// Switch back to receiver mode once the transmission is complete.
		RS485_SetReceiveMode();
		return count;
	}
	return -1;

}



