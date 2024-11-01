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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"
#include "timer.h"
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
Semaforo semaforo;
Timer timer;  // Timer utilizado para manejar tiempos de transición
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/***********************************************
*        PASO 4: FUNCIONES DE TRANSICIÓN       *
***********************************************/

/**
 * @brief Condición para pasar de verde a verde parpadeante.
 */
static int condicion_verde_a_verde_parpadeo(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    return (semaforo->fsm.currentState == ESTADO_VERDE) &&
           (HAL_GPIO_ReadPin(semaforo->PortSwitch, semaforo->PinSwitch) == GPIO_PIN_SET);
}

/**
 * @brief Condición para pasar de verde parpadeante a rojo.
 */
static int condicion_verde_parpadeo_a_rojo(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    return timer_has_expired(semaforo->timer);
}

/**
 * @brief Condición para pasar de rojo a rojo parpadeante.
 */
static int condicion_rojo_a_rojo_parpadeo(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    return timer_has_expired(semaforo->timer);
}

/**
 * @brief Condición para pasar de rojo parpadeante a verde.
 */
static int condicion_rojo_parpadeo_a_verde(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    return timer_has_expired(semaforo->timer);
}

/***********************************************
*          PASO 5: FUNCIONES DE ACCIÓN         *
***********************************************/

/**
 * @brief Acción al entrar en el estado verde.
 */
void on_state_verde(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    HAL_GPIO_WritePin(semaforo->PortVerde, semaforo->PinVerde, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(semaforo->PortRojo, semaforo->PinRojo, GPIO_PIN_SET);
}

/**
 * @brief Acción al entrar en el estado verde parpadeante.
 */
void on_state_verde_parpadeo(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    timer_start(semaforo->timer, 1000); // Parpadeo durante 1 segundo
}

/**
 * @brief Acción al entrar en el estado rojo.
 */
void on_state_rojo(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    HAL_GPIO_WritePin(semaforo->PortVerde, semaforo->PinVerde, GPIO_PIN_SET);
    HAL_GPIO_WritePin(semaforo->PortRojo, semaforo->PinRojo, GPIO_PIN_RESET);
    timer_start(semaforo->timer, 3000); // Estado rojo durante 3 segundos
}

/**
 * @brief Acción al entrar en el estado rojo parpadeante.
 */
void on_state_rojo_parpadeo(void *context) {
    Semaforo *semaforo = (Semaforo *)context;
    timer_start(semaforo->timer, 1000); // Parpadeo rojo durante 1 segundo
}

/***********************************************
*          PASO 6: DECLARAR TRANSICIONES       *
***********************************************/

static Transition VerdeTransitions[] = {
    {condicion_verde_a_verde_parpadeo, ESTADO_VERDE_PARPADEO}
};

static Transition VerdeParpadeoTransitions[] = {
    {condicion_verde_parpadeo_a_rojo, ESTADO_ROJO}
};

static Transition RojoTransitions[] = {
    {condicion_rojo_a_rojo_parpadeo, ESTADO_ROJO_PARPADEO}
};

static Transition RojoParpadeoTransitions[] = {
    {condicion_rojo_parpadeo_a_verde, ESTADO_VERDE}
};

/***********************************************
*           PASO 7: ENSAMBLAR EL MÓDULO        *
***********************************************/

static FSMState SemaforoEstados[] = {
    {VerdeTransitions, 1, on_state_verde},
    {VerdeParpadeoTransitions, 1, on_state_verde_parpadeo},
    {RojoTransitions, 1, on_state_rojo},
    {RojoParpadeoTransitions, 1, on_state_rojo_parpadeo}
};

/***********************************************
*         PASO 8: INICIALIZAR EL MÓDULO        *
***********************************************/

/**
 * @brief Inicializa el módulo de semáforo.
 */
void semaforo_init(Semaforo *semaforo, GPIO_TypeDef *PortVerde, uint16_t PinVerde,
                   GPIO_TypeDef *PortRojo, uint16_t PinRojo,
                   GPIO_TypeDef *PortSwitch, uint16_t PinSwitch) {
    fsm_init(&semaforo->fsm, SemaforoEstados, ESTADO_VERDE, semaforo);
    semaforo->PortVerde = PortVerde;
    semaforo->PinVerde = PinVerde;
    semaforo->PortRojo = PortRojo;
    semaforo->PinRojo = PinRojo;
    semaforo->PortSwitch = PortSwitch;
    semaforo->PinSwitch = PinSwitch;
    on_state_verde(semaforo); // Establece el estado inicial en verde
}

/***********************************************
*         PASO 9: ACTUALIZAR EL MÓDULO         *
***********************************************/

/**
 * @brief Actualiza el módulo de semáforo.
 */
void semaforo_update(Semaforo *semaforo) {
    fsm_update(&semaforo->fsm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  semaforo.timer = &timer;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  semaforo_init(&semaforo, LED_VERDE_GPIO_Port, LED_VERDE_Pin, LED_ROJO_GPIO_Port, LED_ROJO_Pin, SW_1_GPIO_Port, SW_1_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    semaforo_update(&semaforo);
    /* USER CODE END WHILE */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_VERDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_VERDE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_ROJO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ROJO_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SW_1_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
