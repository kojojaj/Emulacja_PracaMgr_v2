/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define button_portType GPIO_TypeDef*
#define button_pinType uint16_t
typedef struct {
	button_portType button_port;
	button_pinType button_pin;
} button_HandleTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BLINK_TIME 125
#define CLICK_TIME 10
#define MASS_MAX 1000 //max. mass value is 10 kg
#define MASS_MIN 10   //min. mass value is 0.1 kg
#define BEAM_OFF 80
#define ADC_TO_VOLTS 0.00080566406
#define DX_MAX 0.0075 //[m]
#define DX_MIN 0.000153075 //[m]
#define X_FACTOR 6532.74538625f // X_FACTOR = 1/DX_MIN [mm] (how many single steps is needed to reach calc. displacement)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile static uint16_t ADC_value[2];
volatile static float mass = 0.0f;
volatile float F_right = 0.0f;
volatile float F_left = 0.0f;
volatile float F = 0.0f; //Net force value

/* RK4 Method variables */
volatile static float h = 0.02f; 		  //single step value
volatile float dq[2] = {0.0f, 0.0f}; //initial conditions
volatile float Q[2] = {0, 0}; 		  //Initial values Q[0] = displacement, Q[1] = velocity;
volatile float k1[2] = {0, 0};
volatile float k2[2] = {0, 0};
volatile float k3[2] = {0, 0};
volatile float k4[2] = {0, 0};
volatile float a = 0.0f;

volatile float dx = 0.0f; 		 //displacement in [m]
volatile uint16_t no_steps = 0; //number of stepper motor micro-steps

/* Forces measurement variables */
  uint16_t beam_right_stable_ADCval = 0;
  uint16_t beam_left_stable_ADCval = 0;
  uint16_t beam_right_ADCval = 0;
  uint16_t beam_left_ADCval = 0;
  int16_t right_val = 0;
  int16_t left_val = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* Function used for interfacing mass definition and beams voltage regulation */
void displayf(int* p_m, int menu, Lcd_HandleTypeDef lcd, uint16_t adc[2]);
/* Function used for displaying countdown after pressing enter button.
 * Initial positions of cars have to be set during countdown. */
void countdownf(Lcd_HandleTypeDef lcd);
/* Function used along with RK4 method to calculate k1, k2, k3 and k4 values. */
void f(float Q1,float a, float dq[2], float h);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* LCD variables and TypeDefs */
  Lcd_PortType ports[] = { LCD_D4_GPIO_Port, LCD_D5_GPIO_Port, LCD_D6_GPIO_Port, LCD_D7_GPIO_Port };
  Lcd_PinType pins[] = {LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin};
  Lcd_HandleTypeDef lcd;
  lcd = Lcd_create(ports, pins, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_E_GPIO_Port, LCD_E_Pin, LCD_4_BIT_MODE);

  /*Initial stepper motor driver setup*/
  HAL_GPIO_WritePin(DRV_MS1_GPIO_Port, DRV_MS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_MS2_GPIO_Port, DRV_MS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_I1_GPIO_Port, DRV_I1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);

  /* ADC calibration and measurement through DMA start */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Menu variables */
  bool isEnterPressed = 0;
  int menu = 0;
  int m = 200; // m = 100 * mass -> easier LCD programming
  int* p_m = &m; //pointer to 'm' useful in 'menu' function

  /* Menu and beams calibration loop */
  while (isEnterPressed == false)
  {
	  /*Assuming that there is no strain on springs yet, actual ADC values are 'stable' values.*/
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  ADC_value[0] = HAL_ADC_GetValue(&hadc1);
	  beam_left_stable_ADCval = ADC_value[0];
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  ADC_value[1] = HAL_ADC_GetValue(&hadc1);
	  beam_right_stable_ADCval = ADC_value[1];

	  displayf(p_m, menu, lcd, (uint16_t*)ADC_value);

	  //Check if and which button was pressed
	  if (HAL_GPIO_ReadPin(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin) == GPIO_PIN_SET){
		  while (HAL_GPIO_ReadPin(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin) == GPIO_PIN_SET)
		  {}
		  menu = !(menu);
		  Lcd_clear(&lcd);
	  } else if (HAL_GPIO_ReadPin(BUTTON_ENTER_GPIO_Port, BUTTON_ENTER_Pin) == GPIO_PIN_SET) {
		  while (HAL_GPIO_ReadPin(BUTTON_ENTER_GPIO_Port, BUTTON_ENTER_Pin) == GPIO_PIN_SET)
		  {}
		  isEnterPressed = true;
		  mass = (float)m / 100.0f;
		  Lcd_clear(&lcd);
		  countdownf(lcd);
		  //Enable H-bridges in the stepper-motor driver:
		  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
	  }
  }

  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim8);
  while (1)
  {
	  if (dx >= DX_MIN){
		  HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin, GPIO_PIN_SET); //set CW rotation (movement) direction.
		  no_steps = dx * X_FACTOR; 	//calc. number of steps needed to reach calculated displacement
		  TIM1 -> RCR = no_steps - 1; 	//write number of steps to RepetitionCounterRegister
		  TIM1->CR1 |= TIM_CR1_CEN;   	/*Run TIM1 which will generate fixed number of PWM pulses. *Single PWM pulse results in single stepper motor micro-step.*/
		  Q[0] = 0;
		  dx = 0;//zero-out previous 'dx' value
	  } else if (dx <= - DX_MIN){
		  HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin, GPIO_PIN_RESET); //set CCW rotation (movement) direction.
		  no_steps = -1 * dx * X_FACTOR; //calc. number of steps needed to reach calculated displacement:
		  TIM1 -> RCR = no_steps - 1; 	 //write number of steps to RepetitionCounterRegister:
		  TIM1->CR1 |= TIM_CR1_CEN;   	 /*Run TIM1 which will generate fixed number of PWM pulses.
		   *Single PWM pulse results in single stepper motor micro-step.*/
		  Q[0] = 0;
		  dx = 0;//zero-out previous 'dx' value:
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM8_UP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
}

/* USER CODE BEGIN 4 */
void displayf(int* p_m, int menu, Lcd_HandleTypeDef lcd, uint16_t adc[2])
{
	enum col {UNITIES = 4, TENTHS = 6, HUNDREDTHS = 7, UNITS = 9};
	enum row {ROW_UP = 0, ROW_BOTTOM = 1};
	enum menu {MASS = 0, CALIBRATION = 1};
	int cursor_pos = 0;
	char lcd_infoText[] = "m = ";
	char lcd_unitText[] = "[kg]";
	int mass = *p_m;
	uint16_t adc_left_beam = adc[0];
	uint16_t adc_right_beam = adc[1];
	switch (menu)
	{
		case (MASS):
				Lcd_cursor(&lcd, ROW_UP, 0);
			  	Lcd_string(&lcd, "DEFINE MASS:");
			  	Lcd_cursor(&lcd, ROW_BOTTOM, 0);
			  	Lcd_string(&lcd, lcd_infoText);
			  	cursor_pos = strlen(lcd_infoText);
			  	Lcd_cursor(&lcd, ROW_BOTTOM, cursor_pos);
			  	Lcd_cursor(&lcd, ROW_BOTTOM, UNITIES);
			  	Lcd_int(&lcd, mass/100);
			  	Lcd_string(&lcd, ".");
			  	if (mass%100 < 10){
			  		Lcd_cursor(&lcd, ROW_BOTTOM, TENTHS);
			  		Lcd_int(&lcd, 0);
			  		Lcd_cursor(&lcd, ROW_BOTTOM, HUNDREDTHS);
			  	}
			  	Lcd_int(&lcd, mass%100);
			  	Lcd_cursor(&lcd, ROW_BOTTOM, UNITS);
			  	Lcd_string(&lcd, lcd_unitText);
			  	if(HAL_GPIO_ReadPin(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin) == GPIO_PIN_SET && mass < MASS_MAX){
			  		mass++;
			  	} else if (HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin) == GPIO_PIN_SET && mass > MASS_MIN) {
			  		mass--;
			  	}
			  	*p_m = mass;
			  	break;
		case (CALIBRATION): ;
				float volts_left_beam = 3.3f * (float) adc_left_beam / 4096.0f;
				float volts_right_beam = 3.3f * adc_right_beam / 4096.0f;
				float display_left_beam = volts_left_beam * 100;
				float display_right_beam = volts_right_beam * 100;
				int v_l = (int) display_left_beam;
				int v_r = (int) display_right_beam;
				Lcd_cursor(&lcd, ROW_UP, 0);
				Lcd_string(&lcd, "BEAMS CALIB.:");

				Lcd_cursor(&lcd, ROW_BOTTOM, 0);
				Lcd_string(&lcd, "Vl=");
				Lcd_cursor(&lcd, ROW_BOTTOM, UNITIES-1);
				Lcd_int(&lcd, v_l/100);
				Lcd_string(&lcd, ".");
				if (v_l%100 < 10){
					Lcd_cursor(&lcd, ROW_BOTTOM, TENTHS-1);
					Lcd_int(&lcd, 0);
					Lcd_cursor(&lcd, ROW_BOTTOM, HUNDREDTHS-1);
				}
				Lcd_int(&lcd, v_l%100);

				Lcd_cursor(&lcd, ROW_BOTTOM, strlen("Vl=")+6);
				Lcd_string(&lcd, "Vr=");
				Lcd_int(&lcd, v_r/100);
				Lcd_string(&lcd, ".");
				if (v_r%100 < 10){
					Lcd_cursor(&lcd, ROW_BOTTOM, 15);
					Lcd_int(&lcd, 0);
					Lcd_cursor(&lcd, ROW_BOTTOM, 16);
				}
				Lcd_int(&lcd, v_r%100);
				break;
		default:
			break;
	}
}
void countdownf(Lcd_HandleTypeDef lcd)
{
	enum row {ROW_UP = 0, ROW_BOTTOM = 1};
	Lcd_cursor(&lcd, ROW_UP, 0);
	Lcd_string(&lcd, "EMULATION IN: ");
	for(int i = 1; i >= 0; i--){
		if (i != 10){
			Lcd_cursor(&lcd, ROW_UP, strlen("EMULATION IN: "));
			Lcd_string(&lcd, " ");
		}
		Lcd_int(&lcd, i);
		HAL_Delay(999);
	}
	Lcd_clear(&lcd);
	Lcd_cursor(&lcd, ROW_UP, 3);
	Lcd_string(&lcd, "EMULATION!");
}

/*'f' function used for RK4 Method
 * 'Q1' is 'velocity' calculated in previous step
 * 'a' is 'acceleration' calculated in each step when force is measured and mass is user defined
 * 'dq' is an array that stores temporary acceleration and velocity values*/
void f(float Q1, float a, float dq[2], float dt)
{
	/*dq[0] = a*dt; //'velocity' from which displacement 'dx' will be calculated
	dq[1] = a;  //'acceleration' from which velocity will be calculated*/
	dq[0] = Q1;
	dq[1] = a;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_ADC_Start(&hadc1);
		/*Get actual ADC values for each tensometric beam*/
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		/*beam_left_ADCval = ADC_value[0];*/
		beam_left_ADCval = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		/*beam_right_ADCval = ADC_value[1];*/
		beam_right_ADCval = HAL_ADC_GetValue(&hadc1);

		/*Calculate how much actual ADC values differ from 'stable' ones*/
		left_val = beam_left_ADCval - beam_left_stable_ADCval;
		right_val = beam_right_ADCval - beam_right_stable_ADCval;
		/*if actual ADC values are smaller than BEAM_OFF(SET) the they are considered as '0'*/
		F_right = 0.0f;
		F_left = 0.0f;

		//Left beam -> measure and calculate force in [N]
		if (beam_left_ADCval > beam_left_stable_ADCval + BEAM_OFF){
			F_left = 27.742f * (ADC_TO_VOLTS * left_val);
		} else if (beam_left_ADCval < beam_left_stable_ADCval - BEAM_OFF){
			F_left = 28.297f * (ADC_TO_VOLTS * left_val);
		}
		//Right beam -> measure and calculate force in [N]
		if (beam_right_ADCval > beam_right_stable_ADCval + BEAM_OFF){
			F_right = (29.322f * (ADC_TO_VOLTS * right_val));
		} else if (beam_right_ADCval < beam_right_stable_ADCval - BEAM_OFF){
			F_right = (29.920f * (ADC_TO_VOLTS * right_val));
		}

		/*Calculate Net Force 'F'*/
		F = F_left - F_right;

		/*Calculate acceleration 'a'*/
		a = F / mass;
		/*RK4 Method -> Calculate displacement 'dx'*/
		/*(Intermediate step 'k1'*/
		f(Q[1], a, (float*)dq, h);
		k1[0] = h * dq[0];
		k1[1] = h * dq[1];
		/*Intermediate step 'k2'*/
		f((Q[1] + k1[1]/2), a, (float*)dq, h/2);
		k2[0] = h * dq[0];
		k2[1] = h * dq[1];
		/*Intermediate step 'k3'*/
		f((Q[1] + k2[1]/2), a, (float*)dq, h/2);
		k3[0] = h * dq[0];
		k3[1] = h * dq[1];
		/*Intermediate step 'k4'*/
		f((Q[1] + k3[1]), a, (float*)dq, h);
		k4[0] = h * dq[0];
		k4[1] = h * dq[1];

		/*Final value - calculated velocity*/
		Q[1] = Q[1] + h/6 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
		/*Final value - calculated displacement*/
		Q[0] = Q[0] + h/6 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
		dx = Q[0];
		float xxx = 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
