/*
 * ov7670.c
 *
 *  Created on: Dec 21, 2012
 *      Author: Анатолий
 */

/* Includes ------------------------------------------------------------------*/
#include "ov7670.h"

/** @defgroup OV7670_Private_Defines
  * @{
  */

/* Delay for the CAMERA to be correctly reset */
#define CAMERA_RESET_DELAY               0x4FFF

/* The 7 bits CAMERA address (sent through I2C interface) */
#define CAMERA_READ_ADDRESS           0x43
#define CAMERA_WRITE_ADDRESS          0x42

__IO uint32_t  CAMERATimeout = CAMERA_LONG_TIMEOUT;

/*-----------------------------------
                           Audio CAMERA functions
                                    ------------------------------------------*/
/* High Layer CAMERA functions */
static uint32_t Camera_Init();
static uint32_t Camera_DeInit(void);
//static uint32_t CAMERA_Play(void);
/* Low layer CAMERA functions */
static void     Camera_CtrlInterface_Init(void);
static void     Camera_CtrlInterface_DeInit(void);
static void     Camera_Reset(void);
static void     Camera_GPIO_Init(void);
static void     Camera_GPIO_DeInit(void);
static uint32_t Camera_WriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue);
static uint32_t Camera_ReadRegister(uint8_t RegisterAddr);
static void     Delay(__IO uint32_t nCount);

/*-----------------------------------
                   MAL (Media Access Layer) functions
                                    ------------------------------------------*/
/* Peripherals configuration functions */
static void     Video_MAL_Init(void);
static void     Video_MAL_DeInit(void);
//static void     Video_MAL_PauseResume(uint32_t Cmd, uint32_t Addr);
//static void     Video_MAL_Stop(void);
/*----------------------------------------------------------------------------*/


/**
  * @}
  */

uint32_t CAMERA_Init()
{
  /* Perform low layer CAMERA initialization */
  if (Camera_Init())
  {
    return 1;
  }
  else
  {
    /* I2S data transfer preparation:
    Prepare the Media to be used for the audio transfer from memory to I2S peripheral */
    //Video_MAL_Init();

    /* Return 0 when all operations are OK */
    return 0;
  }
}

/**
  * @brief  Deinitializes all the resources used by the CAMERA (those initialized
  *         by EVAL_AUDIO_Init() function).
  * @param  None
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t CAMERA_DeInit(void)
{
  /* DeInitialize the Media layer */
  Video_MAL_DeInit();

  /* DeInitialize CAMERA */
  Camera_DeInit();

  return 0;
}


static void Camera_Reset(void)
{
  /* Power Down the CAMERA */
  //GPIO_WriteBit(CAMERA_RESET_GPIO, CAMERA_RESET_PIN, Bit_RESET);

  /* wait for a delay to insure registers erasing */
  Delay(CAMERA_RESET_DELAY);

  /* Power on the CAMERA */
  //GPIO_WriteBit(CAMERA_RESET_GPIO, CAMERA_RESET_PIN, Bit_SET);
}

/*========================

                OV7670 Video Camera Control Functions
                                                ==============================*/
/**
  * @brief  Initializes the audio CAMERA and all related interfaces (control
  *         interface: I2C and audio interface: I2S)
  * @param  OutputDevice: can be OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO .
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @retval 0 if correct communication, else wrong communication
  */
static uint32_t Camera_Init()
{
  uint32_t counter = 0;

  /* Configure the CAMERA related IOs */
  Camera_GPIO_Init();

  /* Reset the CAMERA Registers */
  Camera_Reset();

  /* Initialize the Control interface of the Audio CAMERA */
  Camera_CtrlInterface_Init();

  counter += Camera_ReadRegister(0x01);
  //

  /* Keep CAMERA powered OFF */
  //counter += Camera_WriteRegister(0x02, 0x01);

  //counter += CAMERA_WriteRegister(0x04, 0xAF); /* SPK always OFF & HP always ON */
  //OutputDev = 0xAF;

  /* Clock configuration: Auto detection */
  //counter += CAMERA_WriteRegister(0x05, 0x81);

  /* Set the Slave Mode and the audio Standard */
  //counter += CAMERA_WriteRegister(0x06, CAMERA_STANDARD);

  /* Set the Master volume */
  //CAMERA_VolumeCtrl(Volume);

//  if (CurrAudioInterface == AUDIO_INTERFACE_DAC)
//  {
//    /* Enable the PassThrough on AIN1A and AIN1B */
//    counter += CAMERA_WriteRegister(0x08, 0x01);
//    counter += CAMERA_WriteRegister(0x09, 0x01);
//
//    /* Route the analog input to the HP line */
//    counter += CAMERA_WriteRegister(0x0E, 0xC0);
//
//    /* Set the Passthough volume */
//    counter += CAMERA_WriteRegister(0x14, 0x00);
//    counter += CAMERA_WriteRegister(0x15, 0x00);
//  }
//
//  /* Power on the CAMERA */
//  counter += CAMERA_WriteRegister(0x02, 0x9E);

  /* Additional configuration for the CAMERA. These configurations are done to reduce
      the time needed for the CAMERA to power off. If these configurations are removed,
      then a long delay should be added between powering off the CAMERA and switching
      off the I2S peripheral MCLK clock (which is the operating clock for CAMERA).
      If this delay is not inserted, then the CAMERA will not shut down properly and
      it results in high noise after shut down. */

  /* Disable the analog soft ramp */
//  counter += CAMERA_WriteRegister(0x0A, 0x00);
//  if (CurrAudioInterface != AUDIO_INTERFACE_DAC)
//  {
//    /* Disable the digital soft ramp */
//    counter += CAMERA_WriteRegister(0x0E, 0x04);
//  }
//  /* Disable the limiter attack level */
//  counter += CAMERA_WriteRegister(0x27, 0x00);
//  /* Adjust Bass and Treble levels */
//  counter += CAMERA_WriteRegister(0x1F, 0x0F);
//  /* Adjust PCM volume level */
//  counter += CAMERA_WriteRegister(0x1A, 0x0A);
//  counter += CAMERA_WriteRegister(0x1B, 0x0A);
//
//  /* Configure the I2S peripheral */
//  CAMERA_AudioInterface_Init(AudioFreq);

  /* Return communication control value */
  return counter;
}

/**
  * @brief  Restore the audio CAMERA state to default state and free all used
  *         resources.
  * @param  None
  * @retval 0 if correct communication, else wrong communication
  */
static uint32_t Camera_DeInit(void)
{
  uint32_t counter = 0;

  /* Reset the CAMERA Registers */
  Camera_Reset();

  /* Keep CAMERA powered OFF */
  //counter += CAMERA_WriteRegister(0x02, 0x01);

  /* Deinitialize all use GPIOs */
  Camera_GPIO_DeInit();

  /* Disable the CAMERA control interface */
  Camera_CtrlInterface_DeInit();

  /* Deinitialize the CAMERA audio interface (I2S) */
  //CAMERA_AudioInterface_DeInit();

  /* Return communication control value */
  return counter;
}

/**
  * @brief  Initializes the Cideo Camera control interface (I2C).
  * @param  None
  * @retval None
  */
static void Camera_CtrlInterface_Init(void)
{
  I2C_InitTypeDef I2C_InitStructure;

  /* Enable the CODEC_I2C peripheral clock */
  RCC_APB1PeriphClockCmd(CAMERA_I2C_CLK, ENABLE);

  /* CODEC_I2C peripheral configuration */
  I2C_DeInit(CAMERA_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x33;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  /* Enable the I2C peripheral */
  I2C_Cmd(CAMERA_I2C, ENABLE);
  I2C_Init(CAMERA_I2C, &I2C_InitStructure);
}

/**
  * @brief  Restore the Audio Codec control interface to its default state.
  *         This function doesn't de-initialize the I2C because the I2C peripheral
  *         may be used by other modules.
  * @param  None
  * @retval None
  */
static void Camera_CtrlInterface_DeInit(void)
{
  /* Disable the I2C peripheral */ /* This step is not done here because
     the I2C interface can be used by other modules */
  /* I2C_DeInit(CODEC_I2C); */
}


/**
  * @brief Initializes IOs used by the Audio CAMERA (on the control and audio
  *        interfaces).
  * @param  None
  * @retval None
  */
static void Camera_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Reset GPIO Clock */
  RCC_AHB1PeriphClockCmd(AUDIO_RESET_GPIO_CLK,ENABLE);

  /* Audio reset pin configuration -------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = AUDIO_RESET_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(AUDIO_RESET_GPIO, &GPIO_InitStructure);

  /* Enable I2S and I2C GPIO clocks */
  RCC_AHB1PeriphClockCmd(CAMERA_I2C_GPIO_CLOCK , ENABLE);

  /* CAMERA_I2C SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = CAMERA_I2C_SCL_PIN | CAMERA_I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(CAMERA_I2C_GPIO, &GPIO_InitStructure);
  /* Connect pins to I2C peripheral */
  GPIO_PinAFConfig(CAMERA_I2C_GPIO, CAMERA_I2S_SCL_PINSRC, CAMERA_I2C_GPIO_AF);
  GPIO_PinAFConfig(CAMERA_I2C_GPIO, CAMERA_I2S_SDA_PINSRC, CAMERA_I2C_GPIO_AF);

  /* CAMERA_I2S pins configuration: WS, SCK and SD pins -----------------------------*/
//  GPIO_InitStructure.GPIO_Pin = CAMERA_I2S_SCK_PIN | CAMERA_I2S_SD_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(CAMERA_I2S_GPIO, &GPIO_InitStructure);
//
//  /* Connect pins to I2S peripheral  */
//  GPIO_PinAFConfig(CAMERA_I2S_WS_GPIO, CAMERA_I2S_WS_PINSRC, CAMERA_I2S_GPIO_AF);
//  GPIO_PinAFConfig(CAMERA_I2S_GPIO, CAMERA_I2S_SCK_PINSRC, CAMERA_I2S_GPIO_AF);
//
//  if (CurrAudioInterface != AUDIO_INTERFACE_DAC)
//  {
//    GPIO_InitStructure.GPIO_Pin = CAMERA_I2S_WS_PIN ;
//    GPIO_Init(CAMERA_I2S_WS_GPIO, &GPIO_InitStructure);
//    GPIO_PinAFConfig(CAMERA_I2S_GPIO, CAMERA_I2S_SD_PINSRC, CAMERA_I2S_GPIO_AF);
//  }
//  else
//  {
//    /* GPIOA clock enable (to be used with DAC) */
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//
//    /* DAC channel 1 & 2 (DAC_OUT1 = PA.4) configuration */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//  }
//
//#ifdef CAMERA_MCLK_ENABLED
//  /* CAMERA_I2S pins configuration: MCK pin */
//  GPIO_InitStructure.GPIO_Pin = CAMERA_I2S_MCK_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(CAMERA_I2S_MCK_GPIO, &GPIO_InitStructure);
//  /* Connect pins to I2S peripheral  */
//  GPIO_PinAFConfig(CAMERA_I2S_MCK_GPIO, CAMERA_I2S_MCK_PINSRC, CAMERA_I2S_GPIO_AF);
//#endif /* CAMERA_MCLK_ENABLED */
}

/**
  * @brief  Restores the IOs used by the Audio CAMERA interface to their default state.
  * @param  None
  * @retval None
  */
static void Camera_GPIO_DeInit(void)
{
//  GPIO_InitTypeDef GPIO_InitStructure;
//
//  /* Deinitialize all the GPIOs used by the driver */
//  GPIO_InitStructure.GPIO_Pin =  CAMERA_I2S_SCK_PIN | CAMERA_I2S_SD_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//  GPIO_Init(CAMERA_I2S_GPIO, &GPIO_InitStructure);
//
//  GPIO_InitStructure.GPIO_Pin = CAMERA_I2S_WS_PIN ;
//  GPIO_Init(CAMERA_I2S_WS_GPIO, &GPIO_InitStructure);
//
//  /* Disconnect pins from I2S peripheral  */
//  GPIO_PinAFConfig(CAMERA_I2S_WS_GPIO, CAMERA_I2S_WS_PINSRC, 0x00);
//  GPIO_PinAFConfig(CAMERA_I2S_GPIO, CAMERA_I2S_SCK_PINSRC, 0x00);
//  GPIO_PinAFConfig(CAMERA_I2S_GPIO, CAMERA_I2S_SD_PINSRC, 0x00);
//
//#ifdef CAMERA_MCLK_ENABLED
//  /* CAMERA_I2S pins deinitialization: MCK pin */
//  GPIO_InitStructure.GPIO_Pin = CAMERA_I2S_MCK_PIN;
//  GPIO_Init(CAMERA_I2S_MCK_GPIO, &GPIO_InitStructure);
//  /* Disconnect pins from I2S peripheral  */
//  GPIO_PinAFConfig(CAMERA_I2S_MCK_GPIO, CAMERA_I2S_MCK_PINSRC, CAMERA_I2S_GPIO_AF);
//#endif /* CAMERA_MCLK_ENABLED */
}


/**
  * @brief  Writes a Byte to a given register into the video camera through the
            control interface (I2C)
  * @param  RegisterAddr: The address (location) of the register to be written.
  * @param  RegisterValue: the Byte value to be written into destination register.
  * @retval 0 if correct communication, else wrong communication
  */
static uint32_t Camera_WriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue)
{
  uint32_t result = 0;

  /*!< While the bus is busy */
  CAMERATimeout = CAMERA_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(CAMERA_I2C, I2C_FLAG_BUSY))
  {
    if((CAMERATimeout--) == 0) return Camera_TIMEOUT_UserCallback();
  }

  /* Start the config sequence */
  I2C_GenerateSTART(CAMERA_I2C, ENABLE);

  /* Test on EV5 and clear it */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CAMERA_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((CAMERATimeout--) == 0) return Camera_TIMEOUT_UserCallback();
  }

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(CAMERA_I2C, CAMERA_WRITE_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CAMERA_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if((CAMERATimeout--) == 0) return Camera_TIMEOUT_UserCallback();
  }

  /* Transmit the first address for write operation */
  I2C_SendData(CAMERA_I2C, RegisterAddr);

  /* Test on EV8 and clear it */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CAMERA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
  {
    if((CAMERATimeout--) == 0) return Camera_TIMEOUT_UserCallback();
  }

  /* Prepare the register value to be sent */
  I2C_SendData(CAMERA_I2C, RegisterValue);

  /*!< Wait till all data have been physically transferred on the bus */
  CAMERATimeout = CAMERA_LONG_TIMEOUT;
  while(!I2C_GetFlagStatus(CAMERA_I2C, I2C_FLAG_BTF))
  {
    if((CAMERATimeout--) == 0) Camera_TIMEOUT_UserCallback();
  }

  /* End the configuration sequence */
  I2C_GenerateSTOP(CAMERA_I2C, ENABLE);

#ifdef VERIFY_WRITTENDATA
  /* Verify that the data has been correctly written */
  result = (Camera_ReadRegister(RegisterAddr) == RegisterValue)? 0:1;
#endif /* VERIFY_WRITTENDATA */

  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;
}

/**
  * @brief  Reads and returns the value of an audio CAMERA register through the
  *         control interface (I2C).
  * @param  RegisterAddr: Address of the register to be read.
  * @retval Value of the register to be read or dummy value if the communication
  *         fails.
  */
static uint32_t Camera_ReadRegister(uint8_t RegisterAddr)
{
  uint32_t result = 0;

  /*!< While the bus is busy */
  CAMERATimeout = CAMERA_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(CAMERA_I2C, I2C_FLAG_BUSY))
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /* Start the config sequence */
  I2C_GenerateSTART(CAMERA_I2C, ENABLE);

  /* Test on EV5 and clear it */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CAMERA_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(CAMERA_I2C, CAMERA_READ_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(CAMERA_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /* Transmit the register address to be read */
  I2C_SendData(CAMERA_I2C, RegisterAddr);

  /* Test on EV8 and clear it */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while (I2C_GetFlagStatus(CAMERA_I2C, I2C_FLAG_BTF) == RESET)
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /*!< Send START condition a second time */
  I2C_GenerateSTART(CAMERA_I2C, ENABLE);

  /*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while(!I2C_CheckEvent(CAMERA_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /*!< Send CAMERA address for read */
  I2C_Send7bitAddress(CAMERA_I2C, CAMERA_READ_ADDRESS, I2C_Direction_Receiver);

  /* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while(I2C_GetFlagStatus(CAMERA_I2C, I2C_FLAG_ADDR) == RESET)
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /*!< Disable Acknowledgment */
  I2C_AcknowledgeConfig(CAMERA_I2C, DISABLE);

  /* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
  (void)CAMERA_I2C->SR2;

  /*!< Send STOP Condition */
  I2C_GenerateSTOP(CAMERA_I2C, ENABLE);

  /* Wait for the byte to be received */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while(I2C_GetFlagStatus(CAMERA_I2C, I2C_FLAG_RXNE) == RESET)
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /*!< Read the byte received from the CAMERA */
  result = I2C_ReceiveData(CAMERA_I2C);

  /* Wait to make sure that STOP flag has been cleared */
  CAMERATimeout = CAMERA_FLAG_TIMEOUT;
  while(CAMERA_I2C->CR1 & I2C_CR1_STOP)
  {
    if((CAMERATimeout--) == 0) return CAMERA_TIMEOUT_UserCallback();
  }

  /*!< Re-Enable Acknowledgment to be ready for another reception */
  I2C_AcknowledgeConfig(CAMERA_I2C, ENABLE);

  /* Clear AF flag for next communication */
  I2C_ClearFlag(CAMERA_I2C, I2C_FLAG_AF);

  /* Return the byte read from CAMERA */
  return result;
}


/**
  * @brief  Inserts a delay time (not accurate timing).
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void Delay( __IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}
