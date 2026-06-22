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
 * | 12/09/2023 | Document creation		                         |
 *
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>	 // memset
#include <stdlib.h>	 // atoi
#include <stdbool.h> // bool, true, false (si no está ya incluido)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h> // para poder usar el atan2, sqrt y M_PI en el cálculo de pitch y roll
#include "i2c_mcu.h"
#include "mpu6050.h"
#include "led.h"
#include "switch.h"
#include "uart_mcu.h"
#include "buzzer.h"
#include "timer_mcu.h"
#include "pwm_mcu.h"
/*==================[macros and definitions]=================================*/
#define CONFIG_PERIOD_MPU6050 500000 // 0.5s
#define I2C_MASTER_FREQ 100000

#define PWM_WAVE_FREQ 2000
#define PWM_CT 50
#define BUZZER_TONE_FREQ 3000

#define MAX_DIGITOS 3
#define DEFAULT_SERIES 1
#define DEFAULT_EJERCICIO 10
#define DEFAULT_DESCANSO 0
/*=========================[typedef]=========================================*/
typedef enum
{
	ESTADO_APAGADO = 0,
	ESTADO_STANDBY,
	ESTADO_EJERCICIO,
	ESTADO_DESCANSO,
} estado_sistema_t;

typedef enum
{
	CONFIG_NINGUNO = 0,
	CONFIG_PREGUNTA_DP, /**< Esperando 'D' (default) o 'P' (personalizada). */
	CONFIG_SERIES,		/**< Esperando número de series. */
	CONFIG_EJERCICIO,	/**< Esperando duración de ejercicio (segundos). */
	CONFIG_DESCANSO,	/**< Esperando duración de descanso (segundos). */
} config_rutina_t;
/*==================[internal data definition]===============================*/
TaskHandle_t MPU6050_task_handle = NULL;

TaskHandle_t Rutina_task_handle = NULL;

int16_t acc_x;
int16_t acc_y;
int16_t acc_z;

float_t accf_x;
float_t accf_y;
float_t accf_z;

float_t pitch;
float_t roll;

float_t pitch_ref = 0;
float_t roll_ref = 0;

float_t tolerancia = 20;

volatile estado_sistema_t estado_sist = ESTADO_APAGADO;

volatile config_rutina_t config_rut = CONFIG_NINGUNO;

volatile bool flag_set_referencia = false;

volatile bool referencia_registrada = false;

volatile bool rutina_configurada = false;

volatile bool rutina_interrumpida = false;

volatile uint8_t series = DEFAULT_SERIES;
volatile uint16_t ejercicio = DEFAULT_EJERCICIO;
volatile uint16_t descanso = DEFAULT_DESCANSO;

uint8_t buffer_entrada[MAX_DIGITOS + 1] = {0};
uint8_t idx_buffer = 0;
/*==================[internal functions declaration]=========================*/
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

void Calculo_Pitch_Roll(float *p, float *r)
{
	MPU6050_getAcceleration(&acc_x, &acc_y, &acc_z);
	accf_x = (float_t)acc_x;
	accf_y = (float_t)acc_y;
	accf_z = (float_t)acc_z;
	*p = atan2f(-accf_x, sqrtf((accf_y * accf_y) + (accf_z * accf_z))) * 180 / (float_t)M_PI;
	*r = atan2f(accf_y, accf_z) * 180 / (float_t)M_PI;
}

bool Procesar_Entrada_Numerica(uint8_t c, uint16_t valor_min, uint16_t valor_max, char *buffer_valor)
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

void Iniciar_Configuracion_Rutina(void)
{
	config_rut = CONFIG_PREGUNTA_DP;
	printf("\n¿Usar rutina por defecto o personalizada?\n");
	printf("  Envíe 'D' para DEFAULT (%d series x %ds ejercicio / %ds descanso)\n",
		   DEFAULT_SERIES, DEFAULT_EJERCICIO, DEFAULT_DESCANSO);
	printf("  Envíe 'P' para PERSONALIZADA\n");
}

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
		if (Procesar_Entrada_Numerica(eleccion_rut, 1, 20, &buffer_2[0]))
		{
			series = (uint8_t)atoi(buffer_2);
			config_rut = CONFIG_EJERCICIO;
			printf("\nSeries = %d\n", series);
			printf("Ingrese la duración del ejercicio en segundos (5-600) y presione Enter:\n");
		}
		break;
	case CONFIG_EJERCICIO:
		if (Procesar_Entrada_Numerica(eleccion_rut, 5, 600, &buffer_2[0]))
		{
			ejercicio = (uint8_t)atoi(buffer_2);
			config_rut = CONFIG_DESCANSO;
			printf("\nDuración de ejercicio = %ds\n", ejercicio);
			printf("Ingrese la duración del descanso en segundos (0-600) y presione Enter:\n");
		}
		break;
	case CONFIG_DESCANSO:
		if (Procesar_Entrada_Numerica(eleccion_rut, 0, 600, &buffer_2[0]))
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

void Buzzer_Inicio_Fin(void)
{
    BuzzerPlayTone(NOTE_C5, 150);
    vTaskDelay(50 / portTICK_PERIOD_MS); 
    BuzzerPlayTone(NOTE_E5, 200);
}

void Buzzer_Transicion(void)
{
    BuzzerPlayTone(NOTE_A4, 400);
}

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

void TEC1_set_referencia(void *ptr)
{
	flag_set_referencia = true;
}

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

void Atender_timer_MPU6050(void *param)
{
	vTaskNotifyGiveFromISR(MPU6050_task_handle, pdFALSE);
}

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