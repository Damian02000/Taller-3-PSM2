/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"
#include "timer.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/***********************************************
*        Definición de Estados FSM             *
***********************************************/
typedef enum {
    ESTADO_VERDE,           // Luz verde encendida
    ESTADO_VERDE_PARPADEO,  // Luz verde parpadeante
    ESTADO_ROJO,            // Luz roja encendida
    ESTADO_ROJO_PARPADEO    // Luz roja parpadeante
} SemaforoEstado;

/***********************************************
*       Estructura del Módulo de Semáforo      *
***********************************************/
typedef struct {
    FSM fsm;                    // Máquina de estados finitos
    GPIO_TypeDef *PortVerde;    // Puerto para LED verde
    uint16_t PinVerde;          // Pin para LED verde
    GPIO_TypeDef *PortRojo;     // Puerto para LED rojo
    uint16_t PinRojo;           // Pin para LED rojo
    GPIO_TypeDef *PortSwitch;   // Puerto para el botón de solicitud
    uint16_t PinSwitch;         // Pin para el botón de solicitud
    Timer *timer;               // Timer para control de transiciones
} Semaforo;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void semaforo_init(Semaforo *semaforo, GPIO_TypeDef *PortVerde, uint16_t PinVerde,
                   GPIO_TypeDef *PortRojo, uint16_t PinRojo,
                   GPIO_TypeDef *PortSwitch, uint16_t PinSwitch);
void semaforo_update(Semaforo *semaforo);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_VERDE_Pin GPIO_PIN_13
#define LED_VERDE_GPIO_Port GPIOC
#define LED_ROJO_Pin GPIO_PIN_12
#define LED_ROJO_GPIO_Port GPIOB
#define SW_1_Pin GPIO_PIN_5
#define SW_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
