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
enum {ON, OFF, TOGGLE};	// defino constantes enteras con un nombre más descriptivo 
/*==================[internal data definition]===============================*/
struct leds {
	uint8_t modo;	// ON, OFF, TOGGLE
	uint8_t n_led;	// cuál led quiero encender
	uint8_t n_ciclos;	// cantidad de ciclos
	uint16_t periodo;	// tiempo de delay de cada ciclo
} my_leds;	// crea la variable my_leds del tipo struct leds (es lo mismo que decir struct leds my_leds)
/*==================[internal functions declaration]=========================*/
void manejo_led (struct leds *datos_leds) { // el nombre en la funcion no conviene que sea el mismo que el de la variable
	uint8_t i = 0;
	switch (datos_leds->modo) {
	case ON:
		LedOn (datos_leds->n_led);
		break;
	case OFF:
		LedOff (datos_leds->n_led);
		break;
	case TOGGLE:
		while (i<datos_leds->n_ciclos) {
			LedOn(datos_leds->n_led);
			vTaskDelay(datos_leds->periodo);
			LedOff(datos_leds->n_led);
			vTaskDelay(datos_leds->periodo);
			i++; 
		}
		break;
	}
}
/*==================[external functions definition]==========================*/
void app_main(void)	{
	LedsInit(); // recordar declarar la variable tipo struct leds si es que no lo hice antes
	my_leds.modo = TOGGLE;
	my_leds.n_ciclos = 10;
	my_leds.n_led = LED_3;
	my_leds.periodo = 100;
	manejo_led (&my_leds);	// el & es para mandarle la dirección (puntero)
}
/*==================[end of file]============================================*/