/*! @mainpage Sistema de análisis de estabilidad postural para tiro con arco
 *
 * @section genDesc Descripción General
 *
 * This section describes how the program works.
 *
 *
 * @section hardConn Conexión de hardware
 *
 * |    Periférico  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	VCC		 	| 	VCC			|
 * | 	GND		 	| 	GND			|
 * | 	SDA		 	| 	GPIO_6		|
 * | 	SCL		 	| 	GPIO_7		|
 *
 *
 * @section changelog Registro de cambios
 *
 * |   Fecha    | Descripción                                    							   |
 * |:----------:|:-----------------------------------------------------------------------------|
 * | 27/05/2026 | Creación del documento. Diagrama en bloques. Veo funciones disponibles en    |
 * | 		    | el driver del MPU-6050  													   |
 * | 03/06/2026 | Adquisición de datos, cálculo de pitch y roll, definición de los ejes, 	   |
 * | 		    | creación de timer y tarea para la adquisición del datos del sensor MPU-6050. |
 * | 		    | Documentación de lo hecho hasta el momento     							   |
 * | 06/06/2026 | Instalación de Wokwi para simular la placa y hacer pruebas con el código.	   |
 * | 09/06/2026 | Uso de teclas para tomar referencia y encender/apagar el sistema, prueba en  |
 * | 		    | Wokwi pero no funciona. Falta documentar las modificaciones hechas.		   |
 * | 10/06/2026 | Modificación del uso de teclas por interrupción, funciona la primer parte del|
 * | 		    | código. Implementación de comparación con valores de referencia y encendido  |
 * | 		    | de alarma. Funciona bloque 1 completo. Se termina la documentación de todo   |
 * | 		    | el bloque 1   |
 *
 * @author Giuliana Gangitano (giuligangitano95@gmail.com)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <math.h> // para poder usar el atan2, sqrt y M_PI en el cálculo de pitch y roll
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
 * @brief Período del timer del densor MPU-6050.
 *
 * Define el tiempo en microsegundos del timer que controla la lectura de datos del sensor MPU-6050.
 */
#define CONFIG_PERIOD_MPU6050 100000 // 0.1s

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
 * Define la frecuencia en Hz del tono del buzzer.
 */
#define BUZZER_TONE_FREQ 3000 // este valor es el que se escucha mejor
/*==================[internal data definition]===============================*/
/**
 * @brief Handle de la tarea encargada de la lectura del sensor y los cálculos principales.
 */
TaskHandle_t MPU6050_task_handle = NULL;

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
 * @brief Estado del sistema (encendido/apagado).
 *
 * True: el sistema está encendido
 * False: el sistema está apagado.
 */
volatile bool encendido = false;

/**
 * @brief Estado de medición (activo/reposo).
 *
 * True: el sistema está midiendo activamente (calcula pitch/roll y compara con referencia).
 * False: el sistema está en reposo, no mide.
 *
 * @note Sólo puede ser true si 'encendido' también es true. Se activa/desactiva exclusivamente mediante
 * la tecla 'S' recibida por UART.
 */
volatile bool midiendo = false;

/**
 * @brief Flag para registrar postura de referencia (seteado por tecla 1).
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
/*==================[internal functions declaration]=========================*/
/**
 * @brief Función para calcular pitch y roll.
 *
 * @details Adquiere los datos de aceleración del sensor MPU-6050 en sus tres ejes, hace los cálculos
 * de pitch y roll.
 *
 * @param[in] p Puntero a dirección donde se almacena el valor de pitch.
 * @param[in] r Puntero a dirección donde se almacena el valor de roll.
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
 * @brief Función ejecutada ante la interrupción de la tecla 1.
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
 * @brief Función ejecutada ante la interrupción de la tecla 2.
 *
 * @details  Invierte el estado de la variable 'encendido' para encender o apagar el sistema.
 * Al apagar, detiene también la medición en curso ('midiendo' pasa a false), por seguridad
 * y para evitar que quede midiendo "fantasma" si se vuelve a encender.
 *
 * @note Esta función se ejecuta en contexto de interrupción (ISR). No inicia la medición:
 * eso es responsabilidad exclusiva de la tecla 'S' por UART.
 *
 * @param[in] ptr Puntero a parámetros genéricos (no utilizado).
 */
void TEC2_encendido(void *ptr)
{
	encendido = !encendido;
	if (!encendido)
	{
		midiendo = false;
		referencia_registrada = false;
	}
	else
	{
		printf("\nSistema ENCENDIDO.\n");
		printf("Presione la tecla 1 para setear la referencia\n");
	}
}

/**
 * @brief Callback de recepción UART: procesa la tecla 'S' para iniciar/detener la medición.
 *
 * @details Si se recibe el carácter 's' o 'S' y el sistema está encendido, invierte el
 * estado de 'midiendo'. Si el sistema está apagado, informa por consola que no se puede
 * iniciar la medición y descarta el comando.
 *
 * @param param Puntero a parámetros genéricos (no utilizado).
 */
void Recibir_Comando_UART(void *param)
{
	uint8_t caracter;
	UartReadByte(UART_PC, &caracter);
	if (caracter == 's' || caracter == 'S')
	{
		if (encendido)
		{
			midiendo = !midiendo;
			if (midiendo)
			{
				printf("\nMedición INICIADA.\n");
			}
			else
			{
				printf("\nMedición DETENIDA.\n");
			}
		}
		else
		{
			printf("\nSistema apagado, no se puede iniciar medición. Presione TEC2 primero.\n");
		}
	}
}

/**
 * @brief Servicio de interrupción del timer del sensor MPU_6050.
 *
 * @details Esta función se ejecuta cada vez que el timer llega a su cuenta máxima. Envía una notificación a la
 * tarea 'Adquirir_datos_MPU6050' para desbloquearla.
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
 * @brief Tarea encargada de la adquisición de datos para análisis postural.
 *
 * @details Esta tarea entra en un bucle. Si 'flag_set_referencia' es true, setea el pitch y roll que se
 * usará de referecia en el entrenamiento, imprime un mensaje en pantalla y cambia 'flag_set_referencia' a
 * false. Luego, siempre y cuando el sistema esté encendido, el bucle permanece bloqueado mediante
 * 'ulTaskNotifyTake' hasta recibir una notificación de 'Atender_timer_MPU6050'. Una vez desbloqueado, llama
 * a la función 'Calculo_Pitch_Roll', comunica los resultados por pantalla, compara los valores de pitch y roll
 * con los de referencia y activa el LED y alarma en caso de que se salgan de rango. Si el sistema se encuentra
 * apagado, detiene la medición, apaga el LED y apaga el buzzer.
 *
 * @param[in] pvParameter Puntero a los parámetros de la tarea (no utilizado).
 */
static void vTask_Deteccion_Pitch_Roll(void *pvParameter)
{
	bool mensaje_apagado_mostrado = false;
	bool mensaje_medicion_mostrado = false;
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (!encendido)
		{
			LedOff(LED_1);
			LedOff(LED_2);
			LedOn(LED_3);
			BuzzerOff();
			if (!mensaje_apagado_mostrado)
			{
				printf("Presione la tecla 2 para encender el sistema\n");
				mensaje_apagado_mostrado = true;
			}
			mensaje_medicion_mostrado = false;
		}
		else
		{
			mensaje_apagado_mostrado = false;
			if (flag_set_referencia)
			{
				Calculo_Pitch_Roll(&pitch_ref, &roll_ref);
				printf("Se registra correctamente valores de referencia  | Pitch=%.2f  Roll=%.2f\n",
					   pitch_ref, roll_ref);
				flag_set_referencia = false;
				referencia_registrada = true;
			}
			if (midiendo)
			{
				LedOn(LED_1);
				LedOff(LED_2);
				LedOff(LED_3);

				Calculo_Pitch_Roll(&pitch, &roll);
				printf("ax=%d  ay=%d  az=%d  | Pitch_ref=%.2f  Roll_ref=%.2f  Pitch=%.2f  Roll=%.2f\n",
					   acc_x, acc_y, acc_z, pitch_ref, roll_ref, pitch, roll);
				if ((pitch_ref + tolerancia < pitch) || (pitch < pitch_ref - tolerancia) ||
					(roll_ref + tolerancia < roll) || (roll < roll_ref - tolerancia))
				{
					BuzzerOn();
				}
				else
				{
					BuzzerOff();
				}
				mensaje_medicion_mostrado = false;
			}
			else
			{
				LedOff(LED_1);
				LedOn(LED_2);
				LedOff(LED_3);
				BuzzerOff();
				if (!referencia_registrada)
				{
					// El mensaje de "presione TEC1" ya se mostró al encender (en TEC2_encendido).
					// No se repite acá para no spamear la consola.
				}
				else if (!mensaje_medicion_mostrado)
				{
					printf("Presione 'S' en el teclado para iniciar la medición\n");
					mensaje_medicion_mostrado = true;
				}
			}
		}
	}
}
/*==================[external functions definition]==========================*/
/**
 * @brief Función principal de la aplicación.
 *
 * @details Inicializa periféricos, timers, tareas, puerto UART.
 */
void app_main(void)
{

	// Inicialización de periféricos
	MPU6050_initialize();
	LedsInit();
	SwitchesInit();
	I2C_initialize(I2C_MASTER_FREQ);
	PWMInit(PWM_3, GPIO_20, PWM_WAVE_FREQ);
	PWMOn(PWM_3);
	PWMSetDutyCycle(PWM_3, PWM_CT);
	BuzzerInit(GPIO_20);
	BuzzerSetFrec(BUZZER_TONE_FREQ);

	// Estado inicial: LED_3 encendido, esperando que se presione TEC2
	LedOff(LED_1);
	LedOff(LED_2);
	LedOn(LED_3);

	// Creación tareas
	xTaskCreate(&vTask_Deteccion_Pitch_Roll, "Leer_sensor_MPU6050", 2048, NULL, 5, &MPU6050_task_handle);

	// Asociación de interrupciones para las teclas
	SwitchActivInt(SWITCH_1, TEC1_set_referencia, NULL);
	SwitchActivInt(SWITCH_2, TEC2_encendido, NULL);

	// Configuración e inicialización del puerto UART
	serial_config_t puerto_uart_pc = {
		.port = UART_PC,
		.baud_rate = 115200,
		.func_p = Recibir_Comando_UART,
		.param_p = NULL};
	UartInit(&puerto_uart_pc);

	// Configuración, inicialización y arranque del timer para adquisión de datos del MPU-6050
	timer_config_t timer_MPU6050 = {
		.timer = TIMER_A,
		.period = CONFIG_PERIOD_MPU6050,
		.func_p = Atender_timer_MPU6050,
		.param_p = NULL};
	TimerInit(&timer_MPU6050);
	TimerStart(timer_MPU6050.timer);
}
/*==================[end of file]============================================*/