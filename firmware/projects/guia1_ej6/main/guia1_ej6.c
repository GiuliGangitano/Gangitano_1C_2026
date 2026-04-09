/*! @mainpage Convertir número a BCD y mostrar en display
 *
 * @section Descripción general
 *
 * Se define una función que recibe un número entero, la cantidad de dígitos del mismo, lo convierte a BCD y
 * lo muestra por pantalla.
 *
 * 
 * @section Conexión Hardware 
 *
 * |    Periférico  |   ESP32   	|	Descripción				|
 * |:--------------:|:--------------|:--------------------------|
 * | 	D1		 	| 	GPIO_20		| 	LSB 					|
 * | 	D2		 	| 	GPIO_21		| 	-    					|
 * | 	D3		 	| 	GPIO_22		| 	-       				|
 * | 	D4		 	| 	GPIO_23		| 	MSB 					|
 * | 	SEL_1	 	| 	GPIO_19		| 	Display centenas		|
 * | 	SEL_2	 	| 	GPIO_18		| 	Display decenas			|
 * | 	SEL_3	 	| 	GPIO_9		| 	Display unidades		|
 *
 *
 * @section Changelog
 *
 * |   Fecha    | Descripción                                    |
 * |:----------:|:-----------------------------------------------|
 * | 08/04/2026 | Creación del documento                         |
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
/*! @brief Estructura para configurar pin GPIO
*
* Estructura que define configuración GPIO, incluyendo número de pin (gpio_t pin) y dirección (io_t dir).
* Valores posibles de dirección: 0 (IN) - 1 (OUT)
*/
typedef struct
{
	gpio_t pin; 
	io_t dir;	
} gpioConf_t;

/*==================[internal functions declaration]=========================*/
/*! @brief Convierte un número entero a representación BCD
*
* La función toma un número entero, lo descompone en sus dígitos decimales y los almacena en un arreglo
* en formato BCD (un dígito por posición).
*
* @param dato número a convertir en BCD.
* @param digitos cantidad de dígitos que se desean obtener.
* @param numero_BCD puntero al arreglo donde almaceno los dígitos BCD.
*/
uint8_t conv_a_BCD(uint32_t dato, uint8_t digitos, uint8_t *numero_BCD)
{
	int8_t i = digitos - 1; // no puede ser uint porque me genera problemas en -1
	while (i >= 0)
	{
		numero_BCD[i] = dato % 10;
		dato = dato / 10;
		i--;
	}
	return 0; // no necesito devolver nada porque queda guardado en numero_BCD
}

/*! @brief Convierte un dígito BCD a señales GPIO.
*
* La función toma un dígito en formato BCD y configura el conjunto de pines GPIO en función de su
* representación binaria.
*
* @param digito valor BCD a representar.
* @param data_GPIO puntero al arreglo de estructuras con la configuración de los pines GPIO.
*
*/
void conv_bcd_to_GPIO(uint8_t digito, gpioConf_t *data_GPIO)
{
	uint8_t i = 0;
	while (i < 4)
	{
		if (digito & (1 << i))
		{							  // 1<<i significa el número 1 corrido i veces hacia la izquierda
			GPIOOn(data_GPIO[i].pin); // prendo el pin
		}
		else
		{
			GPIOOff(data_GPIO[i].pin); // apago el pin
		}
		i++;
	}
}

/*! @brief Convierte un dígito BCD y lo muestra en displays.
*
* La función recibe un número entero y lo convierte a su representación en código BCD. Luego recorre cada dígito
* y lo envía a los pines GPIO del decodificador para ser visualizado en los displays.
*
* @param dato_num número a convertir y mostrar.
* @param digitos cantidad de dígitos a mostrar en el display.
* @param vector_pulsos vector de estructuras GPIO que controlan la habilitación de los display.
* @param vector_GPIO  vector de estructuras que representa el dígito en BCD.
*
* @note los displays comparten las mismas entradas, por lo cual es necesario encender y apagar cada display
* secuencialmente
*
*/
void bcd_to_display(uint32_t dato_num, uint8_t digitos, gpioConf_t *vector_pulsos, gpioConf_t *vector_GPIO)
{
	uint8_t numero_BCD[digitos];
	conv_a_BCD(dato_num, digitos, numero_BCD);

	for (uint8_t i = 0; i < digitos; i++)
	{
		conv_bcd_to_GPIO(numero_BCD[i], vector_GPIO);
		GPIOOn(vector_pulsos[i].pin);
		GPIOOff(vector_pulsos[i].pin);
	}
}
/*==================[external functions definition]==========================*/
void app_main(void)
{
	uint32_t numero = 872;
	uint8_t cant_digitos = 3;
	uint8_t pin_BCD = 4;

	/*! @brief Inicializa pines GPIO para habilitación de display.
	*
	* Se define un vector de estructuras del tipo gpioConf_t que contiene los GPIO utilizados para habilitar
	* los displays e indica si son de entrada o salida. Luego se inicializan los GPIO.
	*
	*/
	gpioConf_t pulsos[3] = {{GPIO_19, GPIO_OUTPUT}, // no le gusta que le de tamaño al vector con una variable
							{GPIO_18, GPIO_OUTPUT}, // del main, o los defino con #define o bien los dejo como
							{GPIO_9, GPIO_OUTPUT}}; // está ahora
	uint8_t k = 0;
	while (k < cant_digitos)
	{
		GPIOInit(pulsos[k].pin, pulsos[k].dir); // Es para decirle al micro que quiero GPIO y no CH
		k++;
	}

	/*! @brief Inicializa pines GPIO que representan el número en BCD.
	*
	* Se define un vector de estructuras del tipo gpioConf_t que contiene los GPIO que representan el valor del  
	* dígito en BCD e indica si son de entrada o salida. Luego se inicializan los GPIO.
	*
	*/
	gpioConf_t datos_GPIO[4] = {{GPIO_20, GPIO_OUTPUT}, // no le gusta que le de tamaño al vector con una variable
								{GPIO_21, GPIO_OUTPUT}, // del main, o los defino con #define o bien los dejo como 
								{GPIO_22, GPIO_OUTPUT}, // está ahora
								{GPIO_23, GPIO_OUTPUT}};

	uint8_t n = 0;
	while (n < pin_BCD)
	{
		GPIOInit(datos_GPIO[n].pin, datos_GPIO[n].dir);
		n++;
	}

	bcd_to_display(numero, cant_digitos, pulsos, datos_GPIO);
}
/*==================[end of file]============================================*/