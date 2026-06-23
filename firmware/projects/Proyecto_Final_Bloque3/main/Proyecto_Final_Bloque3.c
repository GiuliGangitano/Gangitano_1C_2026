/*! @mainpage Sistema de análisis de estabilidad postural para tiro con arco.
 *
 * @section genDesc Descripción General
 *
 * Este código implementa un sistema basado en ESP32 y el sensor MPU6050 para monitorear la estabilidad
 * postural de arqueros. El sistema calcula en tiempo real los ángulos de inclinación (pitch y roll) del arco
 * y proporciona retroalimentación auditiva y visual cuando el usuario se desvía de una postura de referencia
 * establecida. Además, el sistema integra una etapa de selección de rutina para los entrenamientos.
 *
 *
 * @section hardConn Conexión de hardware
 *
 * :--------------------------------:
 * |    MPU6050     |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	VCC		 	| 	VCC			|
 * | 	GND		 	| 	GND			|
 * | 	SDA		 	| 	GPIO_6		|
 * | 	SCL		 	| 	GPIO_7		|
 * :--------------------------------:
 * |    BUZZER      |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	B +		 	| 	GPIO_20		|
 * | 	B -		 	| 	GND			|
 * :--------------------------------:
 *
 * @section changelog Registro de cambios
 *
 * |   Fecha    | Descripción                                                                  |
 * |:----------:|:-----------------------------------------------------------------------------|
 * | 27/05/2026 | Creación del documento. Diagrama en bloques. Revisión del driver del MPU6050.|
 * | 03/06/2026 | Adquisición de datos, cálculo de pitch/roll, timer y tarea MPU6050.   	   |
 * | 06/06/2026 | Instalación de Wokwi para simulación y pruebas.                              |
 * | 09/06/2026 | Incorporación de teclas de referencia y encendido (sin funcionar aún).       |
 * | 10/06/2026 | Teclas por interrupción, comparación con referencia y alarma. Bloque 1 OK.   |   
 * | 11/06/2026 | Comienzo Bloque 2. Configuración de tecla 2 para encendido/apagado además de |
 * |            | inicio/fin de medición (sin funcionar aún). Prueba en Wokwi.                 |
 * | 16/06/2026 | Configuración de tecla 2 para encendido/apagado, tecla 'S' para inicio/fin   |
 * |            | de medición. Mensaje de inicio por consola (sin funcionar aún).              |
 * | 17/06/2026 | Cambio a máquina de estados. Configuración de teclas OK. Se deja Bloque 2 y  | 
 * |            | se arma otro archivo de prueba con todas las nuevas modificaciones.          | 
 * | 21/06/2026 | Configuración flujo de información por UART.                                 | 
 * | 22/06/2026 | Creación archivo Bloque 3 y final. Creación tarea de rutina. Verificación de |
 * |            | todos los pasos y feedback. Bloque 3 OK.                                     |
 * | 23/06/2026 | Documentación completa en Doxygen.                                           |
 * x
 * @author Giuliana Gangitano (giuligangitano95@gmail.com)
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>	 // para poder usar memset
#include <stdlib.h>	 // para poder usar atoi
#include <stdbool.h>  // para poder usar bool, true, false (si no está ya incluido)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>  // para poder usar el atan2, sqrt y M_PI en el cálculo de pitch y roll
#include "i2c_mcu.h"
#include "mpu6050.h"
#include "led.h"
#include "switch.h"
#include "uart_mcu.h"
#include "buzzer.h"
#include "timer_mcu.h"
#include "pwm_mcu.h"
/*==================[macros and definitions]=================================*/
/**
 * @brief Período del timer del sensor MPU6050.
 *
 * Define el tiempo en microsegundos del timer que controla la lectura de datos del sensor MPU6050.
 */
#define CONFIG_PERIOD_MPU6050 500000 // 0.5s

/**
 * @brief Frecuencia del maestro del I2C.
 *
 * Define la frecuencia en Hz del clock del maestro para el protocolo I2C.
 */
#define I2C_MASTER_FREQ 100000

/**
 * @brief Frecuencia del PWM.
 *
 * Define la frecuencia en Hz del PWM utilizado en el buzzer.
 */
#define PWM_WAVE_FREQ 2000

/**
 * @brief Ciclo de trabajo del PWM (porcentaje).
 *
 * Define el ciclo de trabajo del PWM utilizado en el buzzer.
 */
#define PWM_CT 50

/**
 * @brief Frecuencia de tono de alarma.
 *
 * Define la frecuencia en Hz del tono del buzzer cuando se detecta desvío postural.
 */
#define BUZZER_TONE_FREQ 3000

/**
 * @brief Máxima cantidad de digitos permitidos para la configuración de la rutina.
 * 
 * Define el límite de caracteres para el buffer de entrada vía UART.
 */
#define MAX_DIGITOS 3

/**
 * @brief Cantidad de series por defecto.
 *
 * Define la cantidad de series para la elección de rutina DEFAULT.
 */
#define DEFAULT_SERIES 1

/**
 * @brief Duración del ejercicio por defecto.
 *
 * Define la cantidad en segundos que dura la etapa de ejercicio para la elección de rutina DEFAULT.
 */
#define DEFAULT_EJERCICIO 20

/**
 * @brief Duración del descanso por defecto.
 *
 * Define la cantidad en segundos que dura la etapa de descanso para la elección de rutina DEFAULT.
 */
#define DEFAULT_DESCANSO 0
/*=========================[typedef]=========================================*/
/**
 * @brief Estados posibles del sistema.
 *
 * @details Enumeración que representa los cuatro estados del sistema:
 * 	- ESTADO_APAGADO: el sistema está inactivo (LED rojo encendido).
 * 	- ESTADO_STANDBY: el sistema está encendido pero no midiendo (LED amarillo encendido).
 * 	- ESTADO_EJERCICIO: el sistema está en modo de medición activa (LED verde encendido).
 * 	- ESTADO_DESCANSO: el sistema está en pausa entre series (LED amarillo y verde encendidos).
 */
typedef enum
{
	ESTADO_APAGADO = 0,
	ESTADO_STANDBY,
	ESTADO_EJERCICIO,
	ESTADO_DESCANSO,
} estado_sistema_t;

/**
 * @brief Estados del flujo de configuración de rutina vía UART.
 *
 * @details Enumeración que representa las etapas del proceso de configuración de rutina.
 * Cada valor indica qué dato se está esperando recibir del usuario en ese momento:
 * 	- CONFIG_NINGUNO: no hay configuración en curso.
 * 	- CONFIG_PREGUNTA_DP: esperando elección entre rutina por defecto ('D') o personalizada ('P').
 * 	- CONFIG_SERIES: esperando número de series (rango: 1 a 20).
 * 	- CONFIG_EJERCICIO: esperando duración del ejercicio en segundos (rango: 5 a 600).
 * 	- CONFIG_DESCANSO: esperando duración del descanso en segundos (rango: 0 a 600).
 */
typedef enum
{
	CONFIG_NINGUNO = 0,
	CONFIG_PREGUNTA_DP, 
	CONFIG_SERIES,		
	CONFIG_EJERCICIO,	
	CONFIG_DESCANSO,	
} config_rutina_t;
/*==================[internal data definition]===============================*/
/**
 * @brief Handle de la tarea encargada de la lectura del sensor y los cálculos principales.
 */
TaskHandle_t MPU6050_task_handle = NULL;

/**
 * @brief Handle de la tarea encargada del diseño de rutina.
 */
TaskHandle_t Rutina_task_handle = NULL;

/**
 * @brief Variables globales de aceleración crudas (enteros).
 */
int16_t acc_x;
int16_t acc_y;
int16_t acc_z;

/**
 * @brief Variables globales de aceleración convertidas a float.
 */
float_t accf_x;
float_t accf_y;
float_t accf_z;

/**
 * @brief Variables globales de pitch y roll actuales calculados.
 */
float_t pitch;
float_t roll;

/**
 * @brief Variables globales de pitch y roll de referencia.
 */
float_t pitch_ref = 0;
float_t roll_ref = 0;

/**
 * @brief Variable global de tolerancia angular en grados para detección de desvío postural.
 */
float_t tolerancia = 10;

/**
 * @brief Estado actual del sistema.
 * 
 * @details Variable global que refñeja en qué estado se encuentra el sistema en todo momento. Es modificada
 * tanto desde las interrupciones (TECLA 2) como desde la tarea de rutina y los comandos de UART.
 */
volatile estado_sistema_t estado_sist = ESTADO_APAGADO;

/**
 * @brief Estado actual del flujo de configuración de rutina.
 * 
 * @details Variable global que indica en qué etapa del proceso de configuración se encuentra el sistema 
 * cuando el usuario está configurando una rutina vía UART.
 */
volatile config_rutina_t config_rut = CONFIG_NINGUNO;

/**
 * @brief Flag para registrar postura de referencia (seteado por TECLA 1).
 *
 * True: se toma la referencia correctamente.
 * False: no hay solicitud de referencia.
 */
volatile bool flag_set_referencia = false;

/**
 * @brief Flag que indica si ya se registró al menos una vez la referencia.
 *
 * Se usa solo para decidir qué mensaje mostrar en consola (no condiciona la lógica
 * de medición).
 */
volatile bool referencia_registrada = false;

/**
 * @brief Flag que indica si la rutina queda configurada.
 *
 * True: rutina registrada correctamente.
 * False: rutina no registrada.
 */
volatile bool rutina_configurada = false;

/**
 * @brief Flag que indica si la rutina fue interrumpida.
 *
 * True: rutina interrumpida.
 * False: rutina no interrumpida.
 */
volatile bool rutina_interrumpida = false;

/**
 * @brief Número de series configuradas para la rutina actual.
 *
 * @details Cantiad de repeticiones de ciclos de ejercicio/descanso que se ejecutarán al iniciar la rutina.
 * Toma el valor de DEFAULT_SERIES al inicio y se actualiza con la condigutación del usuario vía UART.
 */
volatile uint8_t series = DEFAULT_SERIES;

/**
 * @brief Duración en segundos de la etapa de ejercicio por serie.
 *
 * @details Tiempo activo de medición por serie. Toma el valor de DEFAULT_EJERCICIO al inicio y se 
 * actualiza con la configuración del usuario vía UART.
 */
volatile uint16_t ejercicio = DEFAULT_EJERCICIO;

/**
 * @brief Duración en segundos de la etapa de descanso entre series.
 *
 * @details Tiempo de pausa entre series consecutivas. Toma el valor de DEFAULT_DESCANSO al inicio y se 
 * actualiza con la configuración del usuario vía UART. Si es 0, no hay descanso entre series.
 */
volatile uint16_t descanso = DEFAULT_DESCANSO;

/**
 * @brief Buffer de entrada para recibir datos númericos por UART.
 *
 * @details Almacena temporalmente los caracteres recibidos por puerto serie durante la condiguración
 * de la rutina.
 */
uint8_t buffer_entrada[MAX_DIGITOS + 1] = {0};
/*==================[internal functions declaration]=========================*/
/**
 * @brief Función con mensaje de inicio de sistema.
 *
 * @details Muestra por consola un mensaje de inicio de sistema que se muestra una sola vez, el cual
 * contiene información de cómo funciona la aplicación.
 */
void Mensaje_inicio(void)
{
	printf("\n");
	printf("==================================================================================\n");
	printf("         Bienvenido a tu entrenador de analisis postural de tiro con arco         \n");
	printf("==================================================================================\n");
	printf("\n");
	printf("IMPORTANTE: Este mensaje se mostrara solo una vez. Presta atencion.\n");
	printf("\n");
	printf("--- INSTRUCCIONES DE USO ---\n");
	printf("	[TECLA 2]: encender y apagar el sistema.\n");
	printf("	[TECLA 1]: establecer referencia de trabajo.\n");
	printf("	[TECLA S]: iniciar y detener medición de parámetros.\n");
	printf("	[TECLA D]: seleccionar rutina por default.\n");
	printf("	[TECLA P]: seleccionar rutina personalizada.\n");
	printf("\n");
	printf("--- GUIA VISUAL (LEDS) ---\n");
	printf("	- LUZ ROJA: el sistema se encuentra apagado.\n");
	printf("	- LUZ AMARILLA: el sistema se encuentra encendido (Standby).\n");
	printf("	- LUZ AMARILLA Y VERDE: el sistema esta midiendo (tiempo descanso).\n");
	printf("	- LUZ VERDE: el sistema esta midiendo activamente (tiempo ejercicio).\n");
	printf("==================================================================================\n");
}

/**
 * @brief Función para calcular pitch y roll.
 *
 * @details Adquiere los datos de aceleración del sensor MPU6050 en sus tres ejes, hace los cálculos
 * de pitch y roll.
 *
 * @param[in] p Puntero a dirección donde se almacena el valor de pitch calculado (en grados).
 * @param[in] r Puntero a dirección donde se almacena el valor de roll calculado (en grados).
 */
void Calculo_Pitch_Roll(float *p, float *r)
{
	MPU6050_getAcceleration(&acc_x, &acc_y, &acc_z);
	accf_x = (float_t)acc_x;
	accf_y = (float_t)acc_y;
	accf_z = (float_t)acc_z;
	*p = atan2f(-accf_x, sqrtf((accf_y * accf_y) + (accf_z * accf_z))) * 180 / (float_t)M_PI;
	*r = atan2f(accf_y, accf_z) * 180 / (float_t)M_PI;
}

/**
 * @brief Función que valida un valor numérico ingresado por UART dentro de un rango permitido.
 *
 * @details Interpreta el buffer de entrada como un número enterp y verifica que no esté vacío y que se
 * encuentre dentro del rango establecido [valor_min, valor_max]. En caso de error, imprime un mensaje por 
 * consola para corregir la entrada de datos.
 *
 * @param[in] valor_min Valor mínimo aceptado (inclusive).
 * @param[in] valor_max Valor máximo aceptado (inclusive).
 * @param[in] buffer_valor Puntero al buffer de caracteres con el valor ingresado.
 * 
 * @return true si el valor es válido y se encuentra dentro del rango.
 * @return false si la entrada está vacía o el valor está fuera del rango permitido.
 */
bool Procesar_Entrada_Numerica(uint16_t valor_min, uint16_t valor_max, char *buffer_valor)
{
	char carac = buffer_valor[0];
	uint16_t valor = (uint16_t)atoi(buffer_valor);

	if (carac == ' ' || carac == '\n' || carac == '\r' || carac == '\0')
	{
		printf("\nNo ingresó ningún valor. Intente nuevamente:\n");
		return false;
	}
	else
	{
		if (valor < valor_min || valor > valor_max)
		{
			printf("\nValor fuera de rango (%d - %d). Intente nuevamente:\n", valor_min, valor_max);
			return false;
		}
		else
		{
			return true;
		}
	}
}

/**
 * @brief Función que inicia el flujo de configuración de rutina via UART.
 * 
 * @details Establece la variable global 'config_rut' en CONFIG_PREGUNTA_DP e imprime por consola el 
 * mensaje inicial para que el usuario elija entre rutina por defecto ('D') o personalizada ('P'). Esta
 * función debe llamarse desde ESTADO_STANDBY, luego de que el usuario haya registrado la postura de 
 * referencia.
 */
void Iniciar_Configuracion_Rutina(void)
{
	config_rut = CONFIG_PREGUNTA_DP;
	printf("\n¿Usar rutina por defecto o personalizada?\n");
	printf("  Envíe 'D' para DEFAULT (%d series x %ds ejercicio / %ds descanso)\n",
		   DEFAULT_SERIES, DEFAULT_EJERCICIO, DEFAULT_DESCANSO);
	printf("  Envíe 'P' para PERSONALIZADA\n");
}

/**
 * @brief Máquina de estados para la confiuración de rutina vía UART.
 * 
 * @details Procesa los caracteres recibidos por UART según el estado actual de configuración (config_rut).
 * Avanza por las etapas de configuración en el siguiente orden:
 *   1. CONFIG_PREGUNTA_DP: selección entre rutina por defecto o personalizada.
 *   2. CONFIG_SERIES: ingreso de cantidad de series (1-20).
 *   3. CONFIG_EJERCICIO: ingreso de duración del ejercicio en segundos (5-600).
 *   4. CONFIG_DESCANSO: ingreso de duración del descanso en segundos (0-600).
 * 
 * Al completarse la configuración, establece rutina_configurada en true y config_rut en CONFIG_NINGUNO.
 * 
 * @param[in] eleccion_rut Primer caracter recibido por UART en el ciclo actual.
 * @param[in] buffer_2 Puntero al buffer completo con los caracteres recibidos.
 */
void Configuracion_Rutina_UART(uint8_t eleccion_rut, char *buffer_2)
{
	switch (config_rut)
	{
	case CONFIG_PREGUNTA_DP:
		if (eleccion_rut == 'd' || eleccion_rut == 'D')
		{
			series = DEFAULT_SERIES;
			ejercicio = DEFAULT_EJERCICIO;
			descanso = DEFAULT_DESCANSO;
			rutina_configurada = true;
			config_rut = CONFIG_NINGUNO;
			printf("\nRutina DEFAULT seleccionada: %d series x %ds ejercicio / %ds descanso\n",
				   series, ejercicio, descanso);
			printf("Presione 'S' en el teclado para iniciar la medición\n");
		}
		else if (eleccion_rut == 'p' || eleccion_rut == 'P')
		{
			config_rut = CONFIG_SERIES;
			printf("\nIngrese la cantidad de series (1-20) y presione Enter:\n");
		}
		else
		{
			printf("\nValor no válido. Por favor, envíe 'D' (default) o 'P' (personalizada).\n");
		}
		break;
	case CONFIG_SERIES:
		if (Procesar_Entrada_Numerica(1, 20, &buffer_2[0]))
		{
			series = (uint8_t)atoi(buffer_2);
			config_rut = CONFIG_EJERCICIO;
			printf("\nSeries = %d\n", series);
			printf("Ingrese la duración del ejercicio en segundos (5-600) y presione Enter:\n");
		}
		break;
	case CONFIG_EJERCICIO:
		if (Procesar_Entrada_Numerica(5, 600, &buffer_2[0]))
		{
			ejercicio = (uint8_t)atoi(buffer_2);
			config_rut = CONFIG_DESCANSO;
			printf("\nDuración de ejercicio = %ds\n", ejercicio);
			printf("Ingrese la duración del descanso en segundos (0-600) y presione Enter:\n");
		}
		break;
	case CONFIG_DESCANSO:
		if (Procesar_Entrada_Numerica(0, 600, &buffer_2[0]))
		{
			descanso = (uint8_t)atoi(buffer_2);
			rutina_configurada = true;
			config_rut = CONFIG_NINGUNO;
			printf("\nDuración de descanso = %ds\n", descanso);
			printf("\nRutina PERSONALIZADA configurada: %d series x %ds ejercicio / %ds descanso\n",
				   series, ejercicio, descanso);
			printf("Presione 'S' en el teclado para iniciar la medición\n");
		}
		break;
	case CONFIG_NINGUNO:
		break;
	}
}

/**
 * @brief Función de recepción de datos por UART.
 * 
 * @details Esta función se invoca automáticamente cada vez que se reciben datos por el puerto UART_PC.
 * Lee hasta MAX_DIGITOS + 1 caracteres en el buffer global 'buffer_entrada' y actúa según el contexto:
 * 	- Si hay una configuración de rutina en curso, delega el procesamiento a 'Configuracion_Rutina_UART()'.
 * 	- Si se recibe 'S' o 's' en ESTADO_STANDBY, verifica que haya referencia registrada y rutina 
 * 	  configurada antes de iniciar la medición (notifica a 'Rutina_task_handle').
 * 	- Si se recibe 'S' o 's' en ESTADO_EJERCICIO o ESTADO_DESCANSO, detiene la medición.
 * 	- Cualquier otro carácter fuera de contexto genera un mensaje de error por consola.
 * 
 * @param[in] param Puntero a parámetros genéricos (no utilizado).
 */
void Recibir_Comando_UART(void *param)
{
	uint8_t caracter;
	memset(buffer_entrada, 0, sizeof(buffer_entrada));

	UartReadBuffer(UART_PC, &buffer_entrada[0], MAX_DIGITOS + 1);
	caracter = buffer_entrada[0];

	if (config_rut != CONFIG_NINGUNO)
	{
		Configuracion_Rutina_UART(caracter, (char *)&buffer_entrada[0]);
		return;
	}
	if (caracter == 's' || caracter == 'S')
	{
		if (estado_sist == ESTADO_STANDBY)
		{
			if (!referencia_registrada)
			{
				printf("\nPresione TECLA 1 para setear referencia\n");
			}
			else if (!rutina_configurada)
			{
				printf("\nConfigure la rutina primero.\n");
				Iniciar_Configuracion_Rutina();
			}
			else
			{
				estado_sist = ESTADO_EJERCICIO;
				printf("\nMedición INICIADA.\n");
				xTaskNotifyGive(Rutina_task_handle);
			}
		}
		else if (estado_sist == ESTADO_EJERCICIO || estado_sist == ESTADO_DESCANSO)
		{
			estado_sist = ESTADO_STANDBY;
			rutina_configurada = false;
			printf("\nMedición DETENIDA.\n");
		}
		else
		{
			printf("\nComando no disponible en el estado actual del sistema.\n");
		}
	}
	else
	{
		printf("\nTecla '%c' no reconocida. Presione 'S' para iniciar/detener la medición.\n", caracter);
	}
}

/**
 * @brief Función de buzzer para indicar inicio o fin de rutina.
 *
 * @details Reproduce dos notas consecutivas para señalizar al usuario que la rutina comenzó o finalizó. 
 * Se inserta un retardo de 50 ms entre ambas.
 */
void Buzzer_Inicio_Fin(void)
{
    BuzzerPlayTone(NOTE_C5, 150);
    vTaskDelay(50 / portTICK_PERIOD_MS); 
    BuzzerPlayTone(NOTE_E5, 200);
}

/**
 * @brief Función de buzzer para indicar transición entre etapas de rutina.
 *
 * @details Reproduce uan nota durante 400 ms para indicar al usuario el cambio de etapa dentro de la 
 * rutina: inicio de descanso o comienzo de nueva serie.
 */
void Buzzer_Transicion(void)
{
    BuzzerPlayTone(NOTE_A4, 400);
}

/**
 * @brief Función que aplica Feedabakk visual y auditivo según el estado del sistema.
 *
 * @details Actualiza el estado de los tres LEDs y del buzzer de acuerdo con el valor actual de la variable
 * global 'estado_sist':
 * 
 * 	- ESTADO_APAGADO: LED rojo encendido, buzzer apagado.
 * 	- ESTADO_STANDBY: LED amarillo encendido, buzzer apagado.
 * 	- ESTADO_EJERCICIO: LED verde encendido. El buzzer se activa si el pitch o el roll actuales superan 
 * 	  la tolerancia respecto a los valores de referencia; de lo contrario, el buzzer permanece apagado.
 * 	- ESTADO_DESCANSO: LED verde y LED amarillo encendidos, buzzer apagado.
 * 
 * @note Esta función se llamada periódicamente desde la tarea de detección para mantener el feedback
 * sincronizado con el estado del sistema.
 */
void Aplicar_Feedback(void)
{
	switch (estado_sist)
	{
	case ESTADO_APAGADO:
		LedOff(LED_1);
		LedOff(LED_2);
		LedOn(LED_3);
		BuzzerOff();
		break;
	case ESTADO_STANDBY:
		LedOff(LED_1);
		LedOn(LED_2);
		LedOff(LED_3);
		BuzzerOff();
		break;
	case ESTADO_EJERCICIO:
		LedOn(LED_1);
		LedOff(LED_2);
		LedOff(LED_3);
		if ((pitch > pitch_ref + tolerancia) || (pitch < pitch_ref - tolerancia) ||
			(roll > roll_ref + tolerancia) || (roll < roll_ref - tolerancia))
		{
			BuzzerSetFrec(NOTE_C6);
			BuzzerOn();
		}
		else
		{
			BuzzerOff();
		}
		break;
	case ESTADO_DESCANSO:
		LedOn(LED_1);
		LedOn(LED_2);
		LedOff(LED_3);
		BuzzerOff();
		break;
	}
}

/**
 * @brief Función ejecutada ante la interrupción de la TECLA 1.
 *
 * @details Setea el estado de la variable 'flag_set_referencia' en true.
 *
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 *
 * @param[in] ptr Puntero a parámetros genéricos (no utilizado).
 */
void TEC1_set_referencia(void *ptr)
{
	flag_set_referencia = true;
}

/**
 * @brief Función ejecutada ante la interrupción de la TECLA 2.
 *
 * @details  Si la variable 'estado_sist' es ESTADO_APAGADO pasa a ESTADO_STANDBY, en cualquier otro caso pasa a
 * apagado ESTADO_APAGADO. Como medida de seguridad, se resetean ciertas flags y el valor de 'config_rut'.
 *
 * @note Esta función se ejecuta en contexto de interrupción (ISR). No inicia la medición: eso es 
 * responsabilidad exclusiva de la tecla 'S' por UART.
 *
 * @param[in] ptr Puntero a parámetros genéricos (no utilizado).
 */
void TEC2_encendido(void *ptr)
{
	switch (estado_sist)
	{
	case ESTADO_APAGADO:
		estado_sist = ESTADO_STANDBY;
		break;
	default:
		estado_sist = ESTADO_APAGADO;
		referencia_registrada = false;
		rutina_configurada = false;
		config_rut = CONFIG_NINGUNO;
		break;
	}
}

/**
 * @brief Servicio de interrupción del timer del sensor MPU6050.
 *
 * @details Esta función se ejecuta cada vez que el timer llega a su cuenta máxima. Envía una notificación a la
 * tarea 'Deteccion_Pitch_Roll' para desbloquearla.
 *
 * @note Esta función se ejecuta en contexto de interrupción (ISR).
 *
 * @param[in] param Puntero a parámetros genéricos (no utilizado).
 */
void Atender_timer_MPU6050(void *param)
{
	vTaskNotifyGiveFromISR(MPU6050_task_handle, pdFALSE);
}

/**
 * @brief Tarea encargada de la adquisición y procesamiento de datos del sensor.
 *
 * @details Esta tarea permanece en estado bloqueado hasta recibir una notificación del timer de muestreo.
 * Al despertar, lee los valores brutos del MPU6050, realiza el cálculo trigonométrico para obtener pitch
 * y roll, y actualiza el feedback del sistema según el estado actual.
 * 
 * 	- ESTADO_APAGADO: solo actualiza el feedback (LED rojo, buzzer apagado).
 * 	- ESTADO_STANDBY: muestra el mensaje de bienvenida la primera vez, actualiza el feedback y, si
 * 	  'flag_set_referencia' está activo, registra los valores de pitch y roll actuales como referencia e 
 * 	  inicia el flujo de configuración de rutina.
 * 	- ESTADO_EJERCICIO: calcula pitch y roll actuales, los imprime por consola junto con los valores de 
 * 	  referencia y las aceleraciones crudas, y actualiza el feedback.
 * 	- ESTADO_DESCANSO: solo actualiza el feedback (LEDs verde y amarillo, buzzer apagado).
 *
 * @param[in] pvParameter Puntero a los parámetros de la tarea (no utilizado).
 */
static void vTask_Deteccion_Pitch_Roll(void *pvParameter)
{
	bool mensaje_inicio_mostrado = false;
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		switch (estado_sist)
		{
		case ESTADO_APAGADO:
			Aplicar_Feedback();
			break;
		case ESTADO_STANDBY:
			Aplicar_Feedback();
			if(!mensaje_inicio_mostrado){
				Mensaje_inicio();
				mensaje_inicio_mostrado = true;
			}
			if (flag_set_referencia)
			{
				Calculo_Pitch_Roll(&pitch_ref, &roll_ref);
				printf("\nSe registra correctamente valores de referencia  | Pitch=%.2f  Roll=%.2f\n",
					   pitch_ref, roll_ref);
				flag_set_referencia = false;
				referencia_registrada = true;
				Iniciar_Configuracion_Rutina();
			}
			break;
		case ESTADO_EJERCICIO:
			Calculo_Pitch_Roll(&pitch, &roll);
			printf("ax=%d  ay=%d  az=%d  | Pitch_ref=%.2f  Roll_ref=%.2f  Pitch=%.2f  Roll=%.2f\n",
				   acc_x, acc_y, acc_z, pitch_ref, roll_ref, pitch, roll);
			Aplicar_Feedback();
			break;
		case ESTADO_DESCANSO:
			Aplicar_Feedback();
			break;
		}
	}
}

/**
 * @brief Tarea encargada de gestionar el ciclo de series de la rutina de entrenamiento.
 *
 * @details Esta tarea permanece bloqueada hasta recibir una notificación desde 'Recibir_Comando_UART()'
 * al detectar el comando 'S' con las condiciones de 'referencia registrada' y 'rutina configurada'
 * satisfechas.
 *
 * Al desbloquearse, ejecuta el siguiente flujo:
 * 	1. Emite el tono de inicio de rutina (Buzzer_Inicio_Fin()).
 * 	2. Para cada serie (1 a 'series'):
 *    - Cambia el sistema a ESTADO_EJERCICIO y espera 'ejercicio' segundos. Si el sistema abandona ese 
 * 	    estado (interrupción por 'S'), se activa 'rutina_interrumpida' y se aborta el ciclo.
 *    - Si no es la última serie y 'descanso' > 0, cambia a ESTADO_DESCANSO, emite el tono de transición y 
 * 	    espera 'descanso' segundos. Verifica nuevamente posible interrupción.
 *    - A partir de la segunda serie, emite el tono de transición al comenzar.
 * 3. Al finalizar todas las series (o ante una interrupción), imprime el resultado por consola, emite el 
 * 	  tono de fin de rutina y vuelve a ESTADO_STANDBY, reseteando 'rutina_configurada'.
 *
 * @note En cada segundo se verifica si el estado del sistema fue modificado externamente (por UART o por 
 * TECLA 2) para permitir la interrupción inmediata de la rutina.
 *
 * @param[in] pvParameter Puntero a los parámetros de la tarea (no utilizado).
 */
static void vTask_Rutina(void *pvParameter)
{
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		rutina_interrumpida = false;
		Buzzer_Inicio_Fin(); // Tono inicio de rutina
		for (uint8_t serie_actual = 1; serie_actual <= series && !rutina_interrumpida; serie_actual++){
			if(serie_actual > 1){
				Buzzer_Transicion(); // Tono a partir de finalizar el primer descanso y comenzar la segunda serie
			}
			estado_sist = ESTADO_EJERCICIO;
			printf("\n=== Serie %d/%d | EJERCICIO: %ds ===\n", serie_actual, series, ejercicio);
			for (uint16_t t = ejercicio; t > 0; t--){
				if (estado_sist != ESTADO_EJERCICIO)
                {
                    rutina_interrumpida = true;
                    break;
                }
				vTaskDelay(1000/portTICK_PERIOD_MS);
			}
			if (rutina_interrumpida){
				break;
			}
			if (serie_actual < series && descanso > 0){
				Buzzer_Transicion(); // Tono inicio de descanso
				estado_sist = ESTADO_DESCANSO;
                printf("\n=== Serie %d/%d | DESCANSO: %ds ===\n", serie_actual, series, descanso);
				for (uint16_t t = descanso; t > 0; t--)
                {
                    if (estado_sist != ESTADO_DESCANSO)
                    {
                        rutina_interrumpida = true;
                        break;
                    }
					vTaskDelay(1000/portTICK_PERIOD_MS);
				}
			}
			if (rutina_interrumpida){
				break;
			}
		}
		if (!rutina_interrumpida)
        {
            printf("\n¡Rutina completada! %d series realizadas.\n", series);
        }
        else
        {
            printf("\nRutina interrumpida por el usuario.\n");
        }
		Buzzer_Inicio_Fin(); // Tono fin de rutina
        estado_sist = ESTADO_STANDBY;
        rutina_configurada = false;
	}
}
/*==================[external functions definition]==========================*/
/**
 * @brief Función principal de la aplicación.
 *
 * @details Inicializa todos los periféricos y recursos del sistema.
 */
void app_main(void)
{
	// Inicialización de periféricos
	I2C_initialize(I2C_MASTER_FREQ);
	MPU6050_initialize();
	LedsInit();
	SwitchesInit();
	PWMInit(PWM_3, GPIO_20, PWM_WAVE_FREQ);
	PWMOn(PWM_3);
	PWMSetDutyCycle(PWM_3, PWM_CT);
	BuzzerInit(GPIO_20);
	BuzzerSetFrec(BUZZER_TONE_FREQ);

	// Configuración e inicialización del puerto UART
	serial_config_t puerto_uart_pc = {
		.port = UART_PC,
		.baud_rate = 115200,
		.func_p = Recibir_Comando_UART,
		.param_p = NULL};
	UartInit(&puerto_uart_pc);

	// Asociación de interrupciones para las teclas
	SwitchActivInt(SWITCH_1, TEC1_set_referencia, NULL);
	SwitchActivInt(SWITCH_2, TEC2_encendido, NULL);

	// Creación tareas
	xTaskCreate(&vTask_Deteccion_Pitch_Roll, "Leer_sensor_MPU6050", 2048, NULL, 5, &MPU6050_task_handle);
	xTaskCreate(&vTask_Rutina, "Rutina_entrenamiento", 2048, NULL, 5, &Rutina_task_handle);

	// Configuración, inicialización y arranque del timer para adquisión de datos del MPU6050
	timer_config_t timer_MPU6050 = {
		.timer = TIMER_A,
		.period = CONFIG_PERIOD_MPU6050,
		.func_p = Atender_timer_MPU6050,
		.param_p = NULL};
	TimerInit(&timer_MPU6050);
	TimerStart(timer_MPU6050.timer);
}
/*==================[end of file]============================================*/