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
 * | 18/03/2026 | Document creation		                         |
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
/*==================[macros and definitions]=================================*/

/*==================[internal data definition]===============================*/

/*==================[internal functions declaration]=========================*/
uint8_t conv_a_BCD (uint32_t dato, uint8_t digitos, uint8_t *numero_BCD) {
	int8_t i = digitos - 1; // no puede ser uint porque me genera problemas en -1
	while (i >= 0) {
	numero_BCD [i] = dato % 10;
	dato = dato / 10;
	i--;
	}
	return 0; // no necesito devolver nada porque queda guardado en numero_BCD
}
/*==================[external functions definition]==========================*/
void app_main(void) {
	uint32_t numero = 78;
	uint8_t cant_digitos = 2;
	uint8_t nro_BCD [10];
	conv_a_BCD (numero, cant_digitos, nro_BCD);
	for(uint8_t i = 0; i < cant_digitos; i++){
		printf ("%d", nro_BCD [i]);
	}
	printf("\n");
}
/*==================[end of file]============================================*/