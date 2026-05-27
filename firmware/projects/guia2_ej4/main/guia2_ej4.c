/*! @mainpage Proyecto Osciloscopio
 *
 * @section genDesc Descripción General
 *
 * Diseño e implementación de una aplicación que digitalice una señal analógica y la transmita a un graficador
 * de puerto serie de la PC.
 * 
 *
 * @section ardConn Conexión de hardware
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	CH1	 	    | 	GPIO_01		|
 * | 	+3.3V 	    | 	+3.3V	    |
 * | 	GND 	 	| 	GND 		|
 *
 *
 * @section changelog Registro de cambios
 *
 * |   Fecha    | Descripción                                    |
 * |:----------:|:-----------------------------------------------|
 * | 13/05/2026 | Creación del documento                         |
 * | 13/05/2026 | Código funcionando y verificado                |
 * | 27/05/2026 | Documentación finalizada                       |
 *
 * @author Giuliana Gangitano (giuligangitano95@gmail.com)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_mcu.h"
#include "analog_io_mcu.h"
#include "timer_mcu.h"
/*==================[macros and definitions]=================================*/
/**
 * @brief Período del timer de lectura.
 * 
 * Define el tiempo en microsegundos del timer que controla la lectura del CAD.
 */
#define CONFIG_BLINK_PERIOD_CAD 20000
/*==================[internal data definition]===============================*/
/**
 * @brief Handle de la tarea encargada de la lectura del CAD.
 */
TaskHandle_t CAD_task_handle = NULL;

/**
 * @brief Variable global que almacena la lectura del CAD.
 */
uint16_t lectura_CAD = 0;
/*==================[internal functions declaration]=========================*/
/**
 * @brief Servicio de interrupción del timer.
 * 
 * @details Esta función se ejecuta cada vez que el timer llega a su cuenta máxima. Envía una notificación a la
 * tarea 'Lectura_CADTask' para desbloquearla.
 * 
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 * 
 * @param[in] param Puntero a parámetros genéricos (no utilizado).
 */
void Atender_timer(void *param)
{
	vTaskNotifyGiveFromISR(CAD_task_handle, pdFALSE);
}

/**
 * @brief Tarea encargada de lectura del canal analógico y envío por puerto serie.
 * 
 * @details Esta tarea entra en un bucle y permanece bloqueada mediante 'ulTaskNotifyTake' hasta recibir una 
 * notificación de 'Atender_timer'. Una vez desbloqueada, realiza una lectura del canal CH1, convierte 
 * el valor y lo transmite para el graficador de la PC.
 * 
 * @param[in] pvParameter Puntero a los parámetros de la tarea (no utilizado).
 */
static void Lectura_CADTask(void *pvParameter) {
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		AnalogInputReadSingle(CH1, &lectura_CAD);
		UartSendString(UART_PC, ">pote:");
		UartSendString(UART_PC, (char*)UartItoa(lectura_CAD, 10));
		UartSendString(UART_PC, "\r\n");
	}
}
/*==================[external functions definition]==========================*/
/**
 * @brief Función principal de la aplicación.
 * 
 * @details Inicializa el conversor analógico/digital, configura e inicializa la tarea 'Lectura_CADTask', 
 * configura el puerto UART, configura el timer para que se dispare de forma periódica.
 */
void app_main(void){
	
	// Creación de la tarea de lectura del CAD
	xTaskCreate(&Lectura_CADTask, "Leer_pote", 512, NULL, 5, &CAD_task_handle);

	// Configuración e inicialización del canal analógico
	analog_input_config_t pote = {
		.input = CH1,
		.mode = ADC_SINGLE,
		.func_p = NULL,
		.param_p = NULL,
		.sample_frec = 0};
	AnalogInputInit(&pote);

	// Configuración e inicialización del puerto UART
	serial_config_t puerto_uart_pc = {
		.port = UART_PC,
		.baud_rate = 115200,
		.func_p = NULL,
		.param_p = NULL};
	UartInit(&puerto_uart_pc);

	// Configuración, inicialización y arranque del timer
	timer_config_t timer_pote = {
		.timer = TIMER_A,
		.period = CONFIG_BLINK_PERIOD_CAD,
		.func_p = Atender_timer,
		.param_p = NULL};
	TimerInit(&timer_pote);
	TimerStart(timer_pote.timer);
}
/*==================[end of file]============================================*/