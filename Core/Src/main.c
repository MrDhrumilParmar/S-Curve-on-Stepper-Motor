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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */

#define ZIG_DIAMETER 4.5
#define STROCKLENGTH 16 //in cm
#define STROCK_PER_MINUTE 120 // strick per minute we want to set in
// Define the number of steps per revolution of the stepper motor
#define STEPS_PER_REV 48
// Define the number of microsteps per step
#define MICROSTEPS 4

//phase of acceleration
#define acce_Acceleration_Phase1 0
#define const_Posi_Acceleration_Phase 1
#define deacce_Acceleration_Phase1 2
#define zero_Acceleration_Phase 3
#define deacce_Acceleration_Phase2 4
#define const_Neg_Acceleration_Phase 5
#define acce_Acceleration_Phase2 6

//Basic parameter START

const uint32_t HIGH_Pulse_Width = 7; // in µs
uint32_t one_Rotation_Time; //in ms
float one_Rotation_Length;
uint32_t one_Rotation_MicroStep;

float one_MicroStep_Length;
uint32_t one_Step_Length;
float one_MicroStep_Time;
uint32_t one_Step_Time;

uint32_t Strock_Time; //in ms
uint32_t one_Strock_No_Of_Angle;
uint32_t one_Strock_No_Of_Steps;

uint32_t one_Strock_No_Of_MicroSteps;

//  uint32_t step_In_Incresing_Velocity_Phase;
//  uint32_t step_In_Const_Velocity_Phase;

//Basic parameter END
//Time paramter START

uint32_t percentage_Accelerate_Acceleration = 5; //i %
uint32_t percentage_Const_postive_Acceleration = 10; //i %
uint32_t percentage_Zero_Acceleration = 60; //i %

float time_On_Acce_Acce1_Phase;
float time_On_Const_Pos_Acce_Phase;
float time_On_Deacce_Acce1_Phase;
float time_On_Zero_Acce_Phase;
float time_On_Deacce_Acce2_Phase;
float time_On_Const_Neg_Acce_Phase;
float time_On_Acce_Acce2_Phase;

float distance_On_Acce_Acce1_Phase;
float distance_On_Const_Pos_Acce_Phase;
float distance_On_Deacce_Acce1_Phase;
float distance_On_Zero_Acce_Phase;
float distance_On_Deacce_Acce2_Phase;
float distance_On_Const_Neg_Acce_Phase;
float distance_On_Acce_Acce2_Phase;

//Time paramter END
//velocity parameter START

//const variable

uint32_t step_In_Const_Pos_Neg_Acceleration_Phase;
uint32_t step_In_Zero_Acceleration_Phase;
uint32_t step_In_Accelerate_Acceleration_Phase;
uint32_t step_In_Deaccelerate_Acceleration_Phase;

uint32_t step_Acce_Acce1_Phase;
uint32_t step_Const_Pos_Acce_Phase;
uint32_t step_Deacce_Acce1_Phase;
uint32_t step_Zero_Acce_Phase;
uint32_t step_Deacce_Acce2_Phase;
uint32_t step_Const_Neg_Acce_Phase;
uint32_t step_Acce_Acce2_Phase;

uint32_t no_Of_step_Deacce_velocity_phase;
uint32_t no_Of_step_Const_velocity_phase;
uint32_t no_Of_step_Acce_velocity_phase;
uint32_t temp = 0;
uint32_t temp2 = 0;
uint32_t temp3 = 0;
uint32_t temp4 = 0;
uint32_t temp5 = 0;
uint32_t tempA = 0;
uint32_t tempB = 0;
uint32_t tempC = 0;
uint32_t tempD = 0;
uint32_t tempE = 0;
uint32_t tempF = 0;
uint32_t tempG = 0;

//velocity parameter END

//Acceleration Varible START
float acce_Acceleration_Rate;
float deacce_Acceleration_Rate;
float const_Posi_Acceleration_Rate;
float const_Neg_Acceleration_Rate;

//float
//float

//#define  20*one_MicroStep_Length
//#define  20*one_MicroStep_Length

//uint32_t
//uint32_t
//uint32_t
//uint32_t
//uint32_t

//Acceleration parameter END

//changing variables

//uint8_t veloci_Phase;
volatile uint16_t acceleration_Phase = 0;
volatile uint32_t ceilDelay;
volatile float delay = 0;
volatile float prevPos = 0;
volatile float currPos = 0;
volatile float currStep = 0;
volatile float currVelocity = 0;
volatile float currAcceleration = 0;
volatile float timer = 0;
volatile uint8_t goOnFlag = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(uint32_t us) {
	__HAL_TIM_SET_COUNTER(&htim11, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim11) < us)
		;  // wait for the counter to reach the us input in the parameter
}

void calcdelay(uint32_t prevPos, uint32_t currPos) {
	//	Delay = 100 / ceiling[(prePos - currPos) / (OneUStep_distance)] - HIGH_pulse_width
	volatile float tempii = (currPos - prevPos) / (one_MicroStep_Length);
	delay = HIGH_Pulse_Width - (100 / tempii);
}
void calBasicForRotation() {
	temp2 = 101;
	one_Rotation_Length = ZIG_DIAMETER * 3.14159;
	one_Rotation_MicroStep = STEPS_PER_REV * MICROSTEPS;
	one_Rotation_Time = (one_Rotation_Length * 60000)
			/ (STROCK_PER_MINUTE * STROCKLENGTH); // in milisecond

	//	1 Strock calculation
	one_Strock_No_Of_Angle = STROCKLENGTH * 360 / one_Rotation_Length;
	one_Strock_No_Of_Steps = STROCKLENGTH * STEPS_PER_REV / one_Rotation_Length;
	one_Strock_No_Of_MicroSteps = one_Strock_No_Of_Steps * MICROSTEPS + 1;
	Strock_Time = 60000 / STROCK_PER_MINUTE; // in ms

	// 1 step
//	one_Step_Length = one_Rotation_Length / STEPS_PER_REV;
//	one_Step_Time = 1.0 * Strock_Time / STEPS_PER_REV; // in ms

	// 1µstep
	one_MicroStep_Length = one_Rotation_Length / one_Rotation_MicroStep; // in cm
	one_MicroStep_Time = 1.0 * Strock_Time / one_Strock_No_Of_MicroSteps; // in ms

}
void calStepInVelocityPhase() {
	temp3 = 66;
	step_In_Accelerate_Acceleration_Phase = 1
			+ (percentage_Accelerate_Acceleration * one_Strock_No_Of_MicroSteps
					/ 100);
	step_In_Deaccelerate_Acceleration_Phase =
			step_In_Accelerate_Acceleration_Phase;

	step_In_Zero_Acceleration_Phase =
			1
					+ (percentage_Zero_Acceleration
							* one_Strock_No_Of_MicroSteps / 100);
	step_In_Const_Pos_Neg_Acceleration_Phase =
			(percentage_Const_postive_Acceleration * one_Strock_No_Of_MicroSteps
					/ 100);

	no_Of_step_Acce_velocity_phase = 2 * step_In_Accelerate_Acceleration_Phase
			+ step_In_Const_Pos_Neg_Acceleration_Phase;
	no_Of_step_Const_velocity_phase = step_In_Zero_Acceleration_Phase;
	no_Of_step_Deacce_velocity_phase = 2 * step_In_Accelerate_Acceleration_Phase
			+ step_In_Const_Pos_Neg_Acceleration_Phase;

//	void calStepCounters() {

	step_Acce_Acce1_Phase = step_In_Accelerate_Acceleration_Phase + 1;
	step_Const_Pos_Acce_Phase = step_In_Const_Pos_Neg_Acceleration_Phase
			+ step_Acce_Acce1_Phase + 1;
	step_Deacce_Acce1_Phase = step_In_Accelerate_Acceleration_Phase
			+ step_Const_Pos_Acce_Phase + 1;
	step_Zero_Acce_Phase = step_In_Zero_Acceleration_Phase
			+ step_Deacce_Acce1_Phase;
	step_Deacce_Acce2_Phase = step_In_Accelerate_Acceleration_Phase
			+ step_Zero_Acce_Phase + 1;
	step_Const_Neg_Acce_Phase = step_In_Const_Pos_Neg_Acceleration_Phase
			+ step_Deacce_Acce2_Phase + 1;
	step_Acce_Acce2_Phase = step_In_Accelerate_Acceleration_Phase
			+ step_Const_Neg_Acce_Phase + 1;
//	}

//	void calTimePerPhase() {
	time_On_Acce_Acce1_Phase = 1.0
			* (percentage_Accelerate_Acceleration * Strock_Time / 100);
	time_On_Const_Pos_Acce_Phase = time_On_Acce_Acce1_Phase
			+ 1.0 * (percentage_Const_postive_Acceleration * Strock_Time / 100);
	time_On_Deacce_Acce1_Phase = time_On_Const_Pos_Acce_Phase
			+ 1.0 * (percentage_Accelerate_Acceleration * Strock_Time / 100);
	time_On_Zero_Acce_Phase = time_On_Deacce_Acce1_Phase
			+ 1.0 * (percentage_Zero_Acceleration * Strock_Time / 100);
	time_On_Deacce_Acce2_Phase = time_On_Zero_Acce_Phase
			+ 1.0 * (percentage_Accelerate_Acceleration * Strock_Time / 100);
	time_On_Const_Neg_Acce_Phase = time_On_Deacce_Acce2_Phase
			+ 1.0 * (percentage_Const_postive_Acceleration * Strock_Time / 100);
	time_On_Acce_Acce2_Phase = time_On_Const_Neg_Acce_Phase
			+ 1.0 * (percentage_Accelerate_Acceleration * Strock_Time / 100);

//	}
}

void calDistancePerPhase() {

	distance_On_Acce_Acce1_Phase = 1.0 * step_In_Accelerate_Acceleration_Phase
			* one_MicroStep_Length;
	distance_On_Const_Pos_Acce_Phase = distance_On_Acce_Acce1_Phase
			+ 1.0
					* (step_In_Const_Pos_Neg_Acceleration_Phase
							* one_MicroStep_Length);
	distance_On_Deacce_Acce1_Phase = distance_On_Const_Pos_Acce_Phase
			+ 1.0
					* (step_In_Deaccelerate_Acceleration_Phase
							* one_MicroStep_Length);
	distance_On_Zero_Acce_Phase = distance_On_Deacce_Acce1_Phase
			+ 1.0 * (step_In_Zero_Acceleration_Phase * one_MicroStep_Length);
	distance_On_Deacce_Acce2_Phase = distance_On_Zero_Acce_Phase
			+ 1.0
					* (step_In_Deaccelerate_Acceleration_Phase
							* one_MicroStep_Length);
	distance_On_Const_Neg_Acce_Phase = distance_On_Deacce_Acce2_Phase
			+ 1.0
					* (step_In_Const_Pos_Neg_Acceleration_Phase
							* one_MicroStep_Length);
	distance_On_Acce_Acce2_Phase = distance_On_Const_Neg_Acce_Phase
			+ 1.0
					* (step_In_Accelerate_Acceleration_Phase
							* one_MicroStep_Length);

//	void calAccelerationParameter() {

	acce_Acceleration_Rate = 2 * distance_On_Acce_Acce1_Phase
			/ (time_On_Acce_Acce1_Phase);
	deacce_Acceleration_Rate = acce_Acceleration_Rate;

	const_Posi_Acceleration_Rate = 2 * distance_On_Const_Pos_Acce_Phase
			/ time_On_Const_Pos_Acce_Phase;
	const_Neg_Acceleration_Rate = const_Posi_Acceleration_Rate;
//	}
}

void doAllCalculation() {
	calBasicForRotation();
	calStepInVelocityPhase();
//	calStepCounters();
//	calTimePerPhase();
	calDistancePerPhase();
//	calAccelerationParameter();
}
void pulseGenerate() {
	temp = 007;
	goOnFlag = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	delay_us(HIGH_Pulse_Width);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	delay_us(1);

}

void triversalControl() {
	if (goOnFlag == 1) {
		goOnFlag = 0;

		for (int i = 0; i < one_Strock_No_Of_MicroSteps; i++) {
			if (currStep >= one_Strock_No_Of_MicroSteps
					&& currPos >= one_Step_Length)
				break;

			if (timer <= time_On_Acce_Acce1_Phase)
				acceleration_Phase = acce_Acceleration_Phase1;
			else if (timer <= time_On_Const_Pos_Acce_Phase)
				acceleration_Phase = const_Posi_Acceleration_Phase;
			else if (timer <= time_On_Deacce_Acce1_Phase)
				acceleration_Phase = deacce_Acceleration_Phase1;
			else if (timer <= time_On_Zero_Acce_Phase)
				acceleration_Phase = zero_Acceleration_Phase;
			else if (timer <= time_On_Deacce_Acce2_Phase)
				acceleration_Phase = deacce_Acceleration_Phase2;
			else if (timer <= time_On_Const_Neg_Acce_Phase)
				acceleration_Phase = const_Neg_Acceleration_Phase;
			else if (timer <= time_On_Acce_Acce2_Phase) {
				acceleration_Phase = acce_Acceleration_Phase2;
			} else {
			}
			switch (acceleration_Phase) {
			case (acce_Acceleration_Phase1): {
				tempA = 86;

				if (currStep == step_Acce_Acce1_Phase) {

					currAcceleration = const_Posi_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

					acceleration_Phase = const_Posi_Acceleration_Phase;

				} else if (currStep < step_Acce_Acce1_Phase) {

					currAcceleration += acce_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

				}
				//			delay_us(1);
				break;
			}
			case (const_Posi_Acceleration_Phase): {
				tempB = 86;
				if (currStep == step_Const_Pos_Acce_Phase) {

					currAcceleration -= deacce_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

					acceleration_Phase = deacce_Acceleration_Phase1;

				} else if (currStep < step_Const_Pos_Acce_Phase) {

					currAcceleration = const_Posi_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;

					currStep++;
				}
				delay_us(1);
				break;
			}
			case (deacce_Acceleration_Phase1): {
				tempC = 86;
				if (currStep == step_Deacce_Acce1_Phase) {

					currAcceleration = 0;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

					acceleration_Phase = zero_Acceleration_Phase;

				} else if (currStep < step_Deacce_Acce1_Phase) {

					currAcceleration -= deacce_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;
				}
				break;
			}
			case (zero_Acceleration_Phase): {
				tempD = 86;
				if (currStep == step_Zero_Acce_Phase) {

					currAcceleration -= deacce_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

					acceleration_Phase = deacce_Acceleration_Phase2;

				} else if (currStep < step_Zero_Acce_Phase) {

					currAcceleration = 0;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

				}
				break;
			}
			case (deacce_Acceleration_Phase2): {
				tempE = 86;
				if (currStep == step_Deacce_Acce2_Phase) {

					currAcceleration = const_Neg_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;
					acceleration_Phase = const_Neg_Acceleration_Phase;

				} else if (currStep < step_Deacce_Acce2_Phase) {

					currAcceleration -= deacce_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

				}
				break;
			}
			case (const_Neg_Acceleration_Phase): {
				tempF = 86;
				if (currStep == step_Const_Neg_Acce_Phase) {

					currAcceleration -= deacce_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;
					acceleration_Phase = acce_Acceleration_Phase2;

				} else if (currStep < step_Const_Neg_Acce_Phase) {

					currAcceleration = const_Posi_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;
				}
				break;
			}
			case (acce_Acceleration_Phase2): {
				tempG = 86;
				if (currStep == step_Acce_Acce2_Phase) {

					currAcceleration = const_Posi_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;

					acceleration_Phase = const_Posi_Acceleration_Phase;

				} else if (currStep < step_Acce_Acce2_Phase) {
					currAcceleration += acce_Acceleration_Rate;
					currVelocity += currAcceleration;
					prevPos = currPos;
					currPos += currVelocity;
					currStep++;
				}
				break;
			}

			} // switch end

//			delay = (uint32_t) (calcdelay(prevPos, currPos)-1);
//			pulseGenerate();

//			volatile float tempii = 1.0 * (currPos - prevPos) / (one_MicroStep_Length);
//			delay = HIGH_Pulse_Width - (100 / tempii);

			goOnFlag = 1;

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			delay_us(HIGH_Pulse_Width);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			calcdelay(prevPos, currPos);
			delay_us((uint32_t) delay);

//			HAL_Delay((uint32_t) delay);
			HAL_Delay(500);

		} // forloop end

	} //goonflag end

} //fxn end

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI3_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM11_Init();
	MX_TIM12_Init();
	/* USER CODE BEGIN 2 */

	// Disable the stepper motor driver
	//	GPIO_SetBits(GPIOA, ENABLE_PIN);
	// Set the current step delay to the maximum delay
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim11);

	doAllCalculation();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

//		triversalControl();

		HAL_Delay(50);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 42 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 168 - 1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 100;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void) {

	/* USER CODE BEGIN TIM12_Init 0 */

	/* USER CODE END TIM12_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };

	/* USER CODE BEGIN TIM12_Init 1 */

	/* USER CODE END TIM12_Init 1 */
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 42000 - 1;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = 100 - 1;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM12_Init 2 */

	/* USER CODE END TIM12_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC2 PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1 PB2 PB12 PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB14 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
