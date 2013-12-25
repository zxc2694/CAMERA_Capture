/*
 * ov7670.h
 *
 *  Created on: Dec 21, 2012
 *      Author: Анатолий
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OV7670_H_
#define OV7670_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*----------------------------------------------------------------------------*/

/*-----------------------------------
                    Hardware Configuration defines parameters
                                     -----------------------------------------*/
/* Audio Reset Pin definition */
#define AUDIO_RESET_GPIO_CLK           RCC_AHB1Periph_GPIOD
#define AUDIO_RESET_PIN                GPIO_Pin_4
#define AUDIO_RESET_GPIO               GPIOD

/*------------------------------------
                    OPTIONAL Configuration defines parameters
                                      ----------------------------------------*/
/* I2C clock speed configuration (in Hz)
  WARNING:
   Make sure that this define is not already declared in other files (ie.
  stm322xg_eval.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED                        400000
#endif /* I2C_SPEED */


/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define CAMERA_FLAG_TIMEOUT             ((uint32_t)0xF000)
#define CAMERA_LONG_TIMEOUT             ((uint32_t)(300 * CAMERA_FLAG_TIMEOUT))

/* I2C peripheral configuration defines (control interface of the audio CAMERA) */
#define CAMERA_I2C                      I2C1
#define CAMERA_I2C_CLK                  RCC_APB1Periph_I2C1
#define CAMERA_I2C_GPIO_CLOCK           RCC_AHB1Periph_GPIOB
#define CAMERA_I2C_GPIO_AF              GPIO_AF_I2C1
#define CAMERA_I2C_GPIO                 GPIOB
#define CAMERA_I2C_SCL_PIN              GPIO_Pin_6
#define CAMERA_I2C_SDA_PIN              GPIO_Pin_9
#define CAMERA_I2S_SCL_PINSRC           GPIO_PinSource6
#define CAMERA_I2S_SDA_PINSRC           GPIO_PinSource9

/** @defgroup STM32F4_DISCOVERY_AUDIO_CAMERA_Exported_Functions
  * @{
  */
//void EVAL_AUDIO_SetAudioInterface(uint32_t Interface);
uint32_t CAMERA_Init(); //uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq
uint32_t CAMERA_DeInit(void);
//uint32_t EVAL_AUDIO_Play(uint16_t* pBuffer, uint32_t Size);
//uint32_t EVAL_AUDIO_PauseResume(uint32_t Cmd);
//uint32_t EVAL_AUDIO_Stop(uint32_t CAMERAPowerDown_Mode);
//uint32_t EVAL_AUDIO_VolumeCtl(uint8_t Volume);
//uint32_t EVAL_AUDIO_Mute(uint32_t Command);
//void Audio_MAL_Play(uint32_t Addr, uint32_t Size);


/* User Callbacks: user has to implement these functions in his code if
  they are needed. -----------------------------------------------------------*/

//uint16_t EVAL_AUDIO_GetSampleCallBack(void);

/* This function is called when the requested data has been completely transferred.
   In Normal mode (when  the define AUDIO_MAL_MODE_NORMAL is enabled) this function
   is called at the end of the whole audio file.
   In circular mode (when  the define AUDIO_MAL_MODE_CIRCULAR is enabled) this
   function is called at the end of the current buffer transmission. */
//void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size);

/* This function is called when half of the requested buffer has been transferred
   This callback is useful in Circular mode only (when AUDIO_MAL_MODE_CIRCULAR
   define is enabled)*/
//void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void CAMERA_Error_CallBack(void* pData);

/* Camera_TIMEOUT_UserCallback() function is called whenever a timeout condition
   occurs during communication (waiting on an event that doesn't occur, bus
   errors, busy devices ...) on the CAMERA control interface (I2C).
   You can use the default timeout callback implementation by uncommenting the
   define USE_DEFAULT_TIMEOUT_CALLBACK in stm32f4_discovery_audio_CAMERA.h file.
   Typically the user implementation of this callback should reset I2C peripheral
   and re-initialize communication or in worst case reset all the application. */
uint32_t Camera_TIMEOUT_UserCallback(void);


#endif /* OV7670_H_ */
