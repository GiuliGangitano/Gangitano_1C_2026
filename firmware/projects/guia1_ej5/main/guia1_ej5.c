/*! @mainpage Template
 *
 * @section genDesc General Description
 *
 * This section describes how the program works.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN_X	 	| 	GPIO_X		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 25/03/2026 | Document creation		                         |
 *
 * @author Giuliana Gangitano (giuligangitano95@gmail.com)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "switch.h"
#include "gpio_mcu.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data definition]===============================*/
typedef struct {
	gpio_t pin; // GPIO pin number
	io_t dir; // GPIO direction -- 0 "IN"; 1 "OUT"
} gpioConf_t;
/*==================[internal functions declaration]=========================*/
void conv_bcd_to_GPIO (uint8_t digito, gpioConf_t *data_GPIO) {
	uint8_t i = 0;
	while (i<4) {
		if (digito & (1<<i)) { // 1<<i significa el número 1 corrido i veces hacia la izquierda
			GPIOOn (data_GPIO[i].pin); // prendo el pin
		}
		else {
			GPIOOff (data_GPIO[i].pin); // apago el pin
		}
		i++;
	}
}
/*==================[external functions definition]==========================*/
void app_main(void) {
	// gpioConf_t vector_GPIO [4] = { {GPIO_20,GPIO_OUTPUT}, {GPIO_21,GPIO_OUTPUT}, {GPIO_22,GPIO_OUTPUT}, {GPIO_23,GPIO_OUTPUT} };
	gpioConf_t vector_GPIO [4];
	vector_GPIO [0].pin = GPIO_20;
	vector_GPIO [1].pin = GPIO_21;
	vector_GPIO [2].pin = GPIO_22;
	vector_GPIO [3].pin = GPIO_23;
	uint8_t j = 0;
	while (j<4) {
		vector_GPIO[j].dir = GPIO_OUTPUT;
		j++;
	}
	uint8_t k = 0;
	while (k<4) {
		GPIOInit (vector_GPIO[k].pin, vector_GPIO[k].dir);
		k++;
	}
	uint8_t numero = 4;
	conv_bcd_to_GPIO (numero, vector_GPIO);
}
/*==================[end of file]============================================*/