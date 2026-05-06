/*! @mainpage Medidor de distancia por ultrasonido con interrupciones
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
 * | 	ECO 	 	| 	GPIO_3		|
 * | 	TRIGGER	 	| 	GPIO_2		|
 * | 	+5V 	    | 	+5V		    |
 * | 	GND 	 	| 	GND 		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 06/05/2026 | Document creation		                         |
 *
 * @author Giuliana Gangitano (giuligangitano95@gmail.com)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "switch.h"
#include "gpio_mcu.h"
#include "hc_sr04.h"
#include "lcditse0803.h"
#include "timer_mcu.h"
#include "uart_mcu.h"
/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD_LED 1000000
/*==================[internal data definition]===============================*/
TaskHandle_t led_task_handle = NULL;
uint16_t distancia = 0;
bool encendido = true;
bool mantener_lectura = false;
/*==================[internal functions declaration]=========================*/
void TEC1_encendido(void *ptr)
{
	encendido = !encendido;
}

void TEC2_mantener_medicion(void *ptr)
{
	if (encendido == true)
	{
		mantener_lectura = !mantener_lectura;
	}
}

void Atender_timer(void *param)
{
	vTaskNotifyGiveFromISR(led_task_handle, pdFALSE);
}

static void DistanciaTask(void *pvParameter)
{
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (encendido == true)
		{
			distancia = HcSr04ReadDistanceInCentimeters();
			if (distancia < 10)
			{
				LedsOffAll();
			}
			if ((distancia >= 10) & (distancia < 20))
			{
				LedOn(LED_1);
				LedOff(LED_2);
				LedOff(LED_3);
			}
			if ((distancia >= 20) & (distancia < 30))
			{
				LedOn(LED_1);
				LedOn(LED_2);
				LedOff(LED_3);
			}
			if (distancia >= 30)
			{
				LedOn(LED_1);
				LedOn(LED_2);
				LedOn(LED_3);
			}
			if (mantener_lectura == false)
			{
				LcdItsE0803Write(distancia);
			}
		}
		if (encendido == false)
		{
			LedsOffAll();
		}
		UartSendString(UART_PC, (char*)UartItoa(distancia, 10));
		UartSendString(UART_PC, " cm\r\n");
	}
}

void RecibirCaracter(void *ptr){
	uint8_t caracter;
	UartReadByte(UART_PC, &caracter);
	switch(caracter){
		case 'O':
			encendido = !encendido;
			break;
		case 'H':
			if (encendido == true)
			{
				mantener_lectura = !mantener_lectura;
			}
			break;
	}
}
/*==================[external functions definition]==========================*/
void app_main(void)
{
	SwitchesInit();
	HcSr04Init(GPIO_3, GPIO_2);
	LedsInit();
	LcdItsE0803Init();

	xTaskCreate(&DistanciaTask, "Leer_distancia", 512, NULL, 5, &led_task_handle);

	SwitchActivInt(SWITCH_1, *TEC1_encendido, NULL);
	SwitchActivInt(SWITCH_2, *TEC2_mantener_medicion, NULL);

	timer_config_t timer_lectura = {
		.timer = TIMER_A,
		.period = CONFIG_BLINK_PERIOD_LED, // tiene que estar en microsegundos
		.func_p = Atender_timer,
		.param_p = NULL};
	TimerInit(&timer_lectura);
	TimerStart(timer_lectura.timer);

	serial_config_t puerto_uart_pc = {
		.port = UART_PC,
		.baud_rate = 9600, // funciona bien, averiguar por qué me sirve este valor
		.func_p = RecibirCaracter,
		.param_p = NULL};
	UartInit(&puerto_uart_pc);
}
/*==================[end of file]============================================*/