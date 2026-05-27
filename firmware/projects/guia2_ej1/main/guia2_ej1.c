/*! @mainpage Medidor de distancia por ultrasonido
 *
 * @section genDesc Descripción General
 *
 * Diseño de firmware, modelado con un diagrama de flujo, para medir distancia con un sensor de ultrasonido
 * HC-SR04. Se muestra la distancia medida en un display LCD y se señaliza mediante LEDs de la siguiente manera:
 * 	- **Distancia menor a 10cm:** todos los LEDs apagados.
 * 	- **Distancia entre 10cm y 20cm:** encender LED 1.
 * 	- **Distancia entre 20cm y 30cm:** encender LED 1 y LED 2.
 * 	- **Distancia mayor a 30cm:** todos los LEDs encendidos.
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
 * | 22/04/2026 | Creación del documento                         |
 * | 22/04/2026 | Código funcionando y verificado                |
 * | 19/05/2026 | Documentación finalizada                       |
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
/*==================[macros and definitions]=================================*/
/**
 * @brief Período de refresco para los LEDs de señalización.
 * 
 * Define el tiempo en milisegundos que transcurre entre cada actualización del estado de los LEDs.
 */
#define CONFIG_BLINK_PERIOD_LED 1000

/**
 * @brief Período de lectura para las teclas.
 * 
 * Define el intervalo de tiempo en milisegundos en el que se consulta el estado de las teclas de entrada.
 */
#define CONFIG_BLINK_PERIOD_TECLA 10
/*==================[internal data definition]===============================*/
/**
 * @brief Handle de la tarea encargada de medir distancia y controlar los LEDs.
 */
TaskHandle_t led_task_handle = NULL;

/**
 * @brief Handle de la tarea encargada de lectura y control de switches.
 */
TaskHandle_t teclas_task_handle = NULL;

/**
 * @brief Variable global que almacena la distancia medida.
 */
uint16_t distancia = 0;

/**
 * @brief Variable global que almacena el código de la/s tecla/s presionada/s.
 */
uint8_t teclas;

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
 * @brief Tarea encargada de la medición de distancia y control de LEDs.
 * 
 * @details Esta tarea inicializa el sensor HC-SR04, los LEDs y el display LCD. Luego, entra en un bucle donde, si 
 * el sistema está encendido (control TECLA 1), realiza la medición de distancia y actualiza el display y los LEDs.
 * 
 * Si la variable 'mantener_lectura' está en 'false', el valor medido se actualiza en el LCD.
 * Si el sistema se apaga, se apagan todos los LEDs.
 * 
 * @param[in] pvParameter Puntero a parámetros de la tarea (no utilizado)
 */
static void DistanciaTask(void *pvParameter){
	HcSr04Init(GPIO_3, GPIO_2);
	LedsInit();
	LcdItsE0803Init();
	while (1)
	{
		if (encendido == true){
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
			if (mantener_lectura == false){
				LcdItsE0803Write(distancia);
			}
		}
		if (encendido == false){
			LedsOffAll();
		}
		vTaskDelay(CONFIG_BLINK_PERIOD_LED / portTICK_PERIOD_MS);
	}
}

/**
 * @brief Tarea encargada de la lectura de las teclas.
 * 
 * @details Esta tarea inicializa los switches y luego, entra en un bucle donde lee periódicamente el estado
 * de las teclas. 
 * Dependiendo de la tecla presionada, conmuta el estado de las variables:
 * 	- **SWITCH_1:** alterna el estado del sistema entre encendido y apagado.
 * 	- **SWITCH_2:** alterna el estado de retención de la lectura.
 * 
 * @param pvParameter
 */
static void TeclasTask(void *pvParameter){
	SwitchesInit();
	while (1){
		teclas = SwitchesRead();
		switch(teclas){
    		case SWITCH_1:
    			encendido = !encendido;
    		break;
    		case SWITCH_2:
    			mantener_lectura = !mantener_lectura;
    		break;
    	}
		vTaskDelay(CONFIG_BLINK_PERIOD_TECLA / portTICK_PERIOD_MS);
	}
}
/*==================[external functions definition]==========================*/
/**
 * @brief Función principal de la aplicación.
 * 
 * @details Configura e inicializa las tareas 'DistanciaTask' y 'TeclasTask'.
 */
void app_main(void)
{
	// Creación de las tareas
	xTaskCreate(&DistanciaTask, "Leer_distancia", 512, NULL, 5, &led_task_handle);
	xTaskCreate(&TeclasTask, "Leer_teclas", 512, NULL, 5, &teclas_task_handle);
}
/*==================[end of file]============================================*/