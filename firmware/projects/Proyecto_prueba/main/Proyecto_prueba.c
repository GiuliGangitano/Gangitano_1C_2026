/*! @mainpage Template
 *
 * @section genDesc General Description
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
#define CONFIG_PERIOD_MPU6050 500000 // 0.1s
#define I2C_MASTER_FREQ 100000
#define PWM_WAVE_FREQ 2000
#define PWM_CT 50
#define BUZZER_TONE_FREQ 3000
/*==================[internal data definition]===============================*/
TaskHandle_t MPU6050_task_handle = NULL;

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

float_t tolerancia = 10;

volatile bool encendido = false;
volatile bool midiendo = false;
volatile bool flag_set_referencia = false;
volatile bool referencia_registrada = false;

/*==================[internal functions declaration]=========================*/
void Calculo_Pitch_Roll(float *p, float *r)
{
	MPU6050_getAcceleration(&acc_x, &acc_y, &acc_z);
	accf_x = (float_t)acc_x;
	accf_y = (float_t)acc_y;
	accf_z = (float_t)acc_z;
	*p = atan2f(-accf_x, sqrtf((accf_y * accf_y) + (accf_z * accf_z))) * 180 / (float_t)M_PI;
	*r = atan2f(accf_y, accf_z) * 180 / (float_t)M_PI;
}

void Recibir_Comando_UART(void *param)
{
	uint8_t caracter;
	UartReadByte(UART_PC, &caracter);
	if (caracter == 's' || caracter == 'S')
	{
		if (encendido)
		{
			if (!referencia_registrada)
			{
				printf("\nPresione TECLA 1 para setear referencia\n");
			}
			else
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
		}
	}
}

void TEC1_set_referencia(void *ptr)
{
	flag_set_referencia = true;
	referencia_registrada = true;
}

void TEC2_encendido(void *ptr)
{
	encendido = !encendido;
	if (!encendido)
	{
		midiendo = false;
		referencia_registrada = false;
	}
}

void Atender_timer_MPU6050(void *param)
{
	vTaskNotifyGiveFromISR(MPU6050_task_handle, pdFALSE);
}

static void vTask_Deteccion_Pitch_Roll(void *pvParameter)
{
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (!encendido)
		{
			LedOff(LED_1);
			LedOff(LED_2);
			LedOn(LED_3);
		}
		else
		{
			if (flag_set_referencia)
			{
				Calculo_Pitch_Roll(&pitch_ref, &roll_ref);
				printf("Se registra correctamente valores de referencia  | Pitch=%.2f  Roll=%.2f\n",
					   pitch_ref, roll_ref);
				flag_set_referencia = false;
			}
			if (!midiendo)
			{
				LedOff(LED_1);
				LedOn(LED_2);
				LedOff(LED_3);
			}
			else
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
			}
		}
	}
}
/*==================[external functions definition]==========================*/
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