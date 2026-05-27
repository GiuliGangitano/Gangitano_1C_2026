/*! @mainpage Medidor de distancia por ultrasonido con interrupciones y puerto serie
 *
 * @section genDesc Descripción General
 *
 * Se modifica el firmware de 'guia2_ej2.c' de manera que los datos que se visualizan en el display LCD, sean 
 * enviados por puerto serie y puedan observarse en un terminal de la PC. Además, se utilizan dos letras del teclado
 * de la PC para replicar la funcionalidad de las teclas 1 y 2 de la EDU_ESP.
 *
 *
 * @section hardConn Conexión de hardware
 *
 * |    Periférico  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	ECO 	 	| 	GPIO_3		|
 * | 	TRIGGER	 	| 	GPIO_2		|
 * | 	+5V 	    | 	+5V		    |
 * | 	GND 	 	| 	GND 		|
 *
 *
 * @section changelog Registro de cambios
 *
 * |   Fecha    | Descripción                                    |
 * |:----------:|:-----------------------------------------------|
 * | 06/05/2026 | Creación del documento                         |
 * | 06/05/2026 | Código funcionando y verificado                |
 * | 20/05/2026 | Documentación finalizada                       |
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
/**
 * @brief Período del timer de lectura.
 * 
 * Define el tiempo en microsegundos del timer que controla la activación de la medición de distancia.
 */
#define CONFIG_BLINK_PERIOD_LED 1000000
/*==================[internal data definition]===============================*/
/**
 * @brief Handle de la tarea encargada de medir distancia y controlar los LEDs.
 */
TaskHandle_t led_task_handle = NULL;

/**
 * @brief Variable global que almacena la distancia medida.
 */
uint16_t distancia = 0;

/**
 * @brief Estado del sistema de medición.
 * 
 * True: el sensor realiza la medición y los LEDs operan normalmente.
 * False: el sistema entra en reposo y apaga los LEDs.
 */
bool encendido = true;

/**
 * @brief Estado de retención de lectura en el display LCD.
 * 
 * True: en pantalla se congela el último valor medido.
 * False: el display se actualiza con cada medición.
 */
bool mantener_lectura = false;
/*==================[internal functions declaration]=========================*/
/**
 * @brief Función ejecutada ante la interrupción de la tecla 1.
 * 
 * @details Invierte el estado de la variable 'encendido' para pausar o reanudar el sistema.
 * 
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 * 
 * @param[in] ptr Puntero a parámetros genéricos (no utilizado).
 */
void TEC1_encendido(void *ptr)
{
	encendido = !encendido;
}

/**
 * @brief Función ejecutada ante la interrupción de la tecla 2.
 * 
 * @details Si el sistema está activo, invierte el estado de la variable 'mantener_lectura' para congelar o
 * descongelar la medición mostrada en el display.
 * 
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 * 
 * @param[in] ptr Puntero a parámetros genéricos (no utilizado).
 */
void TEC2_mantener_medicion(void *ptr)
{
	if (encendido == true)
	{
		mantener_lectura = !mantener_lectura;
	}
}

/**
 * @brief Servicio de interrupción del timer.
 * 
 * @details Esta función se ejecuta cada vez que el timer llega a su cuenta máxima. Envía una notificación a la
 * tarea 'DistanciaTask' para desbloquearla.
 * 
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 * 
 * @param[in] param Puntero a parámetros genéricos (no utilizado).
 */
void Atender_timer(void *param)
{
	vTaskNotifyGiveFromISR(led_task_handle, pdFALSE);
}

/**
 * @brief Tarea encargada de la medición de distancia y control de LEDs.
 * 
 * @details Esta tarea entra en un bucle y permanece bloqueada mediante 'ulTaskNotifyTake' hasta recibir una 
 * notificación de 'Atender_timer'. Una vez desbloqueada, si el sistema está encendido (control TECLA 1), realiza 
 * la medición de distancia, actualiza el display y los LEDs, y envía los datos de la medición por UART.
 * 
 * @param[in] pvParameter Puntero a los parámetros de la tarea (no utilizado).
 */
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

/**
 * @brief Función para replicar funcionalidad de las teclas 1 y 2 de la EDU-ESP.
 * 
 * @details Esta función lee la tecla presionada en el teclado y la almacena en 'caracter'. Si se presiona la 'O'
 * se replica la función de la tecla 1, si se presiona la tecla 'H' se replica la función de la tecla 2.
 * 
 * @note Esta función se ejecuta en contexto de interrupción por recepción de la UART.
 * 
 * @param[in] ptr Puntero a parámetros genéricos (no utilizado).
 */
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
/**
 * @brief Función principal de la aplicación.
 * 
 * @details Inicializa los periféricos (switches, display LCD, LEDs y sensor de ultrasonido), configura e 
 * inicializa la tarea 'DistanciaTask', vincula las rutinas ISR a las interrupciones de las teclas, se configura
 * el timer para que se dispare de forma periódica y se configura el puerto UART.
 */
void app_main(void)
{
	// Inicialización de periféricos
	SwitchesInit();
	HcSr04Init(GPIO_3, GPIO_2);
	LedsInit();
	LcdItsE0803Init();

	// Creación de la tarea de medición
	xTaskCreate(&DistanciaTask, "Leer_distancia", 512, NULL, 5, &led_task_handle);

	// Asociación de interrupciones para las teclas
	SwitchActivInt(SWITCH_1, *TEC1_encendido, NULL);
	SwitchActivInt(SWITCH_2, *TEC2_mantener_medicion, NULL);

	// Configuración, inicialización y arranque del timer
	timer_config_t timer_lectura = {
		.timer = TIMER_A,
		.period = CONFIG_BLINK_PERIOD_LED, 
		.func_p = Atender_timer,
		.param_p = NULL};
	TimerInit(&timer_lectura);
	TimerStart(timer_lectura.timer);

	// Configuración e inicialización del puerto UART
	serial_config_t puerto_uart_pc = {
		.port = UART_PC,
		.baud_rate = 9600,
		.func_p = RecibirCaracter,
		.param_p = NULL};
	UartInit(&puerto_uart_pc);
}
/*==================[end of file]============================================*/