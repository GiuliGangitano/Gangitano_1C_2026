/*! @mainpage Proyecto Osciloscopio - ECG
 *
 * @section genDesc Descripción General
 *
 * Diseño e implementación de una aplicación que convierta una señal digital de ECG (provista por la cátedra)
 * en una señal analógica. La misma se debe transmitir por puerto serie para ser visualizada en osciloscopio.
 *
 *
 * @section ardConn Conexión de hardware
 *
 * |    Periférico  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	CH0 	    | 	GPIO_03		|
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
 * | 20/05/2026 | Código funcionando y verificado		         |
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
 * @brief Período del timer del CAD.
 * 
 * Define el tiempo en microsegundos del timer que controla la lectura del CAD.
 */
#define CONFIG_BLINK_PERIOD_CAD 4000

/**
 * @brief Período del timer del ECG.
 * 
 * Define el tiempo en microsegundos del timer que controla la actualización del DAC.
 */
#define CONFIG_BLINK_PERIOD_ECG 4000

/**
 * @brief Define el tamaño del vector de ECG.
 */
#define ECG_LENGTH (sizeof(ECG) / sizeof(ECG[0]))
/*==================[internal data definition]===============================*/
/**
 * @brief Handle de la tarea encargada de la lectura del CAD.
 */
TaskHandle_t CAD_task_handle = NULL;

/**
 * @brief Handle de la tarea encargada de la escritura del ECG.
 */
TaskHandle_t ECG_task_handle = NULL;

/**
 * @brief Variable global que almacena el índice del vector de ECG.
 */
uint16_t indice_ECG = 0;

/**
 * @brief Vector que contiene los datos de una señal de ECG.
 */
const uint8_t ECG[] = {
17,17,17,17,17,17,17,17,17,17,17,18,18,18,17,17,17,17,17,17,17,18,18,18,18,18,18,18,17,17,16,16,16,16,17,17,18,18,18,17,17,17,17,
18,18,19,21,22,24,25,26,27,28,29,31,32,33,34,34,35,37,38,37,34,29,24,19,15,14,15,16,17,17,17,16,15,14,13,13,13,13,13,13,13,12,12,
10,6,2,3,15,43,88,145,199,237,252,242,211,167,117,70,35,16,14,22,32,38,37,32,27,24,24,26,27,28,28,27,28,28,30,31,31,31,32,33,34,36,
38,39,40,41,42,43,45,47,49,51,53,55,57,60,62,65,68,71,75,79,83,87,92,97,101,106,111,116,121,125,129,133,136,138,139,140,140,139,137,
133,129,123,117,109,101,92,84,77,70,64,58,52,47,42,39,36,34,31,30,28,27,26,25,25,25,25,25,25,25,25,24,24,24,24,25,25,25,25,25,25,25,
24,24,24,24,24,24,24,24,23,23,22,22,21,21,21,20,20,20,20,20,19,19,18,18,18,19,19,19,19,18,17,17,18,18,18,18,18,18,18,18,17,17,17,17,
17,17,17};

/*==================[internal functions declaration]=========================*/
/**
 * @brief Servicio de interrupción del timer del CAD.
 * 
 * @details Esta función se ejecuta cada vez que el timer llega a su cuenta máxima. Envía una notificación a la
 * tarea 'Lectura_CADTask' para desbloquearla.
 * 
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 * 
 * @param[in] param Puntero a parámetros genéricos (no utilizado).
 */
void Atender_timer_pote(void *param)
{
	vTaskNotifyGiveFromISR(CAD_task_handle, pdFALSE);
}

/**
 * @brief Servicio de interrupción del timer del ECG.
 * 
 * @details Esta función se ejecuta cada vez que el timer llega a su cuenta máxima. Envía una notificación a la
 * tarea 'Lectura_ECGTask' para desbloquearla.
 * 
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 * 
 * @param[in] param Puntero a parámetros genéricos (no utilizado).
 */
void Atender_timer_ECG(void *param)
{
	vTaskNotifyGiveFromISR(ECG_task_handle, pdFALSE);
}

/**
 * @brief Tarea encargada de lectura del canal analógico y envío por puerto serie.
 * 
 * @details Esta tarea entra en un bucle y permanece bloqueada mediante 'ulTaskNotifyTake' hasta recibir una 
 * notificación de 'Atender_timer_pote'. Una vez desbloqueada, realiza una lectura del canal CH1, convierte 
 * el valor y lo transmite para el graficador de la PC.
 * 
 * @param[in] pvParameter Puntero a los parámetros de la tarea (no utilizado).
 */
static void Lectura_CADTask(void *pvParameter) {
	uint16_t lectura_CAD = 0;
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		AnalogInputReadSingle(CH1, &lectura_CAD);
		UartSendString(UART_PC, ">pote:");
		UartSendString(UART_PC, (char*)UartItoa(lectura_CAD, 10));
		UartSendString(UART_PC, "\r\n");
	}
}

/**
 * @brief Tarea encargada de la salida analógica de la señal de ECG.
 * 
 * @details Esta tarea entra en un bucle y permanece bloqueada mediante 'ulTaskNotifyTake' hasta recibir una 
 * notificación de 'Atender_timer_ECG'. Una vez desbloqueada, escribe secuencialmente los valores del vector
 * ECG usando la salida analógica CH0 e incrementa el índice. Al alcanzar el final del vector, reinicia el
 * índice a cero para volver a iniciar el ciclo de la señal.
 * 
 * @param[in] pvParameter Puntero a los parámetros de la tarea (no utilizado).
 */
static void Lectura_ECGTask(void *pvParameter) {
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		AnalogOutputWrite(ECG[indice_ECG]);
		indice_ECG++;
		if (indice_ECG >= ECG_LENGTH){
			indice_ECG = 0;
		}
	}
}
/*==================[external functions definition]==========================*/
/**
 * @brief Función principal de la aplicación.
 * 
 * @details Inicializa los periféricos de conversión analógico-digital y digital-analógico, configura e
 * inicializa las tareas 'Lectura_CADTask' y 'Lectura_ECGTask', configura el puerto UART y configura los
 * timers.
 */
void app_main(void){

	// Creación de la tarea de lectura del CAD
	xTaskCreate(&Lectura_CADTask, "Leer_pote", 2048, NULL, 5, &CAD_task_handle);

	// Creación de la tarea de lectura del ECG
	xTaskCreate(&Lectura_ECGTask, "Mostrar_ECG", 2048, NULL, 5, &ECG_task_handle);

	// Configuración del CAD
	analog_input_config_t pote = {
		.input = CH1,
		.mode = ADC_SINGLE,
		.func_p = NULL,
		.param_p = NULL,
		.sample_frec = 0};
	AnalogInputInit(&pote);

	// Inicialización canal de salida del ECG (CH0)
	AnalogOutputInit();

	// Configuración e inicialización del puerto UART
	serial_config_t puerto_uart_pc = {
		.port = UART_PC,
		.baud_rate = 115200,
		.func_p = NULL,
		.param_p = NULL};
	UartInit(&puerto_uart_pc);

	// Configuración, inicialización y arranque del timer para el potenciómetro
	timer_config_t timer_pote = {
		.timer = TIMER_A,
		.period = CONFIG_BLINK_PERIOD_CAD, 
		.func_p = Atender_timer_pote,
		.param_p = NULL};
	TimerInit(&timer_pote);
	TimerStart(timer_pote.timer);

	// Configuración, inicialización y arranque del timer para el ECG
	timer_config_t timer_ECG = {
		.timer = TIMER_B,
		.period = CONFIG_BLINK_PERIOD_ECG, 
		.func_p = Atender_timer_ECG,
		.param_p = NULL};
	TimerInit(&timer_ECG);
	TimerStart(timer_ECG.timer);
}
/*==================[end of file]============================================*/