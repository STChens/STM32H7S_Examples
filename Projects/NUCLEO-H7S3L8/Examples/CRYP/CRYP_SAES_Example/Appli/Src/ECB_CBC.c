/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ecb_cbc.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
static CRYP_HandleTypeDef hcryp;
static uint32_t pKeySAES[4] = {0x2B7E1516,0x28AED2A6,0xABF71588,0x09CF4F3C};

/* USER CODE BEGIN PV */
/* The size of the plaintext is in words */
#define PLAINTEXT_SIZE    16

static uint32_t AESIV[4] =   {0x00010203 , 0x04050607 , 0x08090A0B , 0x0C0D0E0F};

static uint32_t AESKey256[8] = {0x603DEB10 ,0x15CA71BE ,0x2B73AEF0 ,0x857D7781 ,0x1F352C07 ,0x3B6108D7 ,0x2D9810A3 ,0x0914DFF4};

static uint32_t Plaintext[16] = { 0x6BC1BEE2 ,0x2E409F96 ,0xE93D7E11 ,0x7393172A ,

                          0xAE2D8A57 ,0x1E03AC9C ,0x9EB76FAC ,0x45AF8E51 ,
                          0x30C81C46 ,0xA35CE411 ,0xE5FBC119 ,0x1A0A52EF ,
                          0xF69F2445 ,0xDF4F9B17 ,0xAD2B417B ,0xE66C3710};

static uint32_t CiphertextAESECB128[16] = {0x3AD77BB4 ,0x0D7A3660 ,0xA89ECAF3 ,0x2466EF97 ,
                           0xF5D3D585 ,0x03B9699D ,0xE785895A ,0x96FDBAAF ,
                           0x43B1CD7F ,0x598ECE23 ,0x881B00E3 ,0xED030688 ,
                           0x7B0C785E ,0x27E8AD3F ,0x82232071 ,0x04725DD4};

static uint32_t CiphertextAESCBC256[16] = {0xF58C4C04 ,0xD6E5F1BA ,0x779EABFB ,0x5F7BFBD6 ,
                          0x9CFC4E96 ,0x7EDB808D ,0x679F777B ,0xC6702C7D ,
                          0x39F23369 ,0xA9D9BACF ,0xA530E263 ,0x04231461 ,
                          0xB2EB05E2 ,0xC39BE9FC ,0xDA6C1907 ,0x8C6A9D1B};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

static void MX_SAES_CRYP_Init_ECB(int keyselect);
static void MX_SAES_CRYP_Init_CBC(int keyselect);
static void PrintMenu_ECB(void);
static void PrintMenu_CBC(void);
static void write_AHK_ECB(void);
static void write_AHK_CBC(void);
static void Read_BHK_ECB(void);
static void Lock_BHK_ECB(void);
static void Write_BHK_ECB(void);
static void Read_BHK_CBC(void);
static void Lock_BHK_CBC(void);
static void Write_BHK_CBC(void);

/* USER CODE BEGIN PFP */

/* Used for storing the encrypted text */
static uint32_t EncryptedText[PLAINTEXT_SIZE];

/* Used for storing the decrypted text */
static uint32_t DecryptedText[PLAINTEXT_SIZE];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PrintMenu_ECB(void)
{
  printf("\r\n============== SAES ECB Test Menu =================\r\n\n");
  printf("  Write AHK to OBK index 0   ------------------- a\r\n\n");
  printf("  Write key to BHK           ------------------- b\r\n\n");
  printf("  Lock BHK to avoid SW read  ------------------- l\r\n\n");
  printf("  Read BHK from BKP register ------------------- r\r\n\n");
  printf("  ================================================\r\n\n");  
  printf("  Key select normal (SW key) ------------------- 1\r\n\n");
  printf("  Key select DHUK            ------------------- 2\r\n\n");
  printf("  Key select BHK             ------------------- 3\r\n\n");
  printf("  Key select DHUK xor BHK    ------------------- 4\r\n\n");  
  printf("  Key select AHK             ------------------- 5\r\n\n");
  printf("  Key select AHK xor DHUK    ------------------- 6\r\n\n"); 
  printf("  Exit                       ------------------- x\r\n\n");
}

static void PrintMenu_CBC(void)
{
  printf("\r\n============== SAES CBC Test Menu =================\r\n\n");
  printf("  Write AHK to OBK index 0   ------------------- a\r\n\n");
  printf("  Write key to BHK           ------------------- b\r\n\n");
  printf("  Lock BHK to avoid SW read  ------------------- l\r\n\n");
  printf("  Read BHK from BKP register ------------------- r\r\n\n");
  printf("  ================================================\r\n\n");
  printf("  Key select normal (SW key) ------------------- 1\r\n\n");
  printf("  Key select DHUK            ------------------- 2\r\n\n");
  printf("  Key select BHK             ------------------- 3\r\n\n");
  printf("  Key select DHUK xor BHK    ------------------- 4\r\n\n");  
  printf("  Key select AHK             ------------------- 5\r\n\n");
  printf("  Key select AHK xor DHUK    ------------------- 6\r\n\n"); 
  printf("  Exit                       ------------------- x\r\n\n");
}

static void write_AHK_ECB(void)
{
  uint32_t key[8];
  uint32_t destination = (uint32_t)&key;
  FLASH_KeyConfigTypeDef keyConfig = { 0U };
  
  key[0] = pKeySAES[3];
  key[1] = pKeySAES[2];
  key[2] = pKeySAES[1];
  key[3] = pKeySAES[0];
  key[4] = pKeySAES[3];
  key[5] = pKeySAES[2];
  key[6] = pKeySAES[1];
  key[7] = pKeySAES[0];
  
  /* Unlock Flash area */
  (void) HAL_FLASH_Unlock();
  (void) HAL_FLASH_OB_Unlock();

  /* Program OBKeys */
  keyConfig.HDPLLevel = FLASH_KEY_LEVEL_CURRENT;
  keyConfig.Size = FLASH_KEY_128_BITS;
  keyConfig.Index = 0;
  if (HAL_FLASHEx_KeyConfig(&keyConfig, (uint32_t *) destination) != HAL_OK)
  {
    Error_Handler();
  }

  __ISB();
  __DSB();

  /* Lock the User Flash area */
  (void) HAL_FLASH_OB_Lock();
  (void) HAL_FLASH_Lock();
}

static void write_AHK_CBC(void)
{
  uint32_t key[8];
  uint32_t destination = (uint32_t)&key;
  FLASH_KeyConfigTypeDef keyConfig = { 0U };
  
  key[0] = AESKey256[7];
  key[1] = AESKey256[6];
  key[2] = AESKey256[5];
  key[3] = AESKey256[4];
  key[4] = AESKey256[3];
  key[5] = AESKey256[2];
  key[6] = AESKey256[1];
  key[7] = AESKey256[0];
  
  /* Unlock Flash area */
  (void) HAL_FLASH_Unlock();
  (void) HAL_FLASH_OB_Unlock();

  /* Program OBKeys */
  keyConfig.HDPLLevel = FLASH_KEY_LEVEL_CURRENT;
  keyConfig.Size = FLASH_KEY_256_BITS;
  keyConfig.Index = 0;
  if (HAL_FLASHEx_KeyConfig(&keyConfig, (uint32_t *) destination) != HAL_OK)
  {
    Error_Handler();
  }

  __ISB();
  __DSB();

  /* Lock the User Flash area */
  (void) HAL_FLASH_OB_Lock();
  (void) HAL_FLASH_Lock();
}

static void Write_BHK_ECB(void)
{
  TAMP->BKP0R = pKeySAES[3];
  TAMP->BKP1R = pKeySAES[2];
  TAMP->BKP2R = pKeySAES[1];
  TAMP->BKP3R = pKeySAES[0];
  TAMP->BKP4R = pKeySAES[3];
  TAMP->BKP5R = pKeySAES[2];
  TAMP->BKP6R = pKeySAES[1];
  TAMP->BKP7R = pKeySAES[0];
}

static void Read_BHK_ECB(void)
{
  printf("TAMP->BKP0R: %08x\r\n", TAMP->BKP0R);
  printf("TAMP->BKP1R: %08x\r\n", TAMP->BKP1R);
  printf("TAMP->BKP2R: %08x\r\n", TAMP->BKP2R);
  printf("TAMP->BKP3R: %08x\r\n", TAMP->BKP3R); 
}

static void Lock_BHK_ECB(void)
{
  TAMP->CFGR |= TAMP_CFGR_BHKLOCK;  
}

static void Write_BHK_CBC(void)
{
  TAMP->BKP0R = AESKey256[7];
  TAMP->BKP1R = AESKey256[6];
  TAMP->BKP2R = AESKey256[5];
  TAMP->BKP3R = AESKey256[4];
  TAMP->BKP4R = AESKey256[3];
  TAMP->BKP5R = AESKey256[2];
  TAMP->BKP6R = AESKey256[1];
  TAMP->BKP7R = AESKey256[0];
}

static void Read_BHK_CBC(void)
{
  printf("TAMP->BKP0R: %08x\r\n", TAMP->BKP0R);
  printf("TAMP->BKP1R: %08x\r\n", TAMP->BKP1R);
  printf("TAMP->BKP2R: %08x\r\n", TAMP->BKP2R);
  printf("TAMP->BKP3R: %08x\r\n", TAMP->BKP3R);
  printf("TAMP->BKP4R: %08x\r\n", TAMP->BKP4R);
  printf("TAMP->BKP5R: %08x\r\n", TAMP->BKP5R);
  printf("TAMP->BKP6R: %08x\r\n", TAMP->BKP6R);
  printf("TAMP->BKP7R: %08x\r\n", TAMP->BKP7R);
}

static void Lock_BHK_CBC(void)
{
  TAMP->CFGR |= TAMP_CFGR_BHKLOCK;  
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void ECB_TEST(void)
{
  printf("Start AES ECB Test with different keys\r\n");
  
  /* Reset RTC clock */
  __HAL_RCC_RTC_DISABLE();
  __HAL_RCC_RTCAPB_CLK_DISABLE();
  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  
  PrintMenu_ECB();
  while (1)
  {
    uint8_t keyselect = 0U;
    

    /* Clean the input path */
    __HAL_UART_FLUSH_DRREGISTER(&hcom_uart[COM1]);
    /* Receive key */
    if (HAL_UART_Receive(&hcom_uart[COM1], &keyselect, 1, 0x2000) == HAL_OK)
    {
      HAL_CRYP_DeInit(&hcryp);
      
      printf("\r\n ====> Your input is %c\r\n", keyselect);
      if ( keyselect >= '1' && keyselect <= '6')
      {
        printf("\r\n ====> Your input is %c\r\n", keyselect);
        if ( keyselect >= '1' && keyselect <= '6')
        {
          keyselect = keyselect - '1';
          
          printf("\r\n ************ Initialize SAES ************\r\n");
          MX_SAES_CRYP_Init_ECB(keyselect);
          printf("Test for key select [%d]\r\n", keyselect);
          
          print_buf("Plain text: ", (uint8_t*)Plaintext, sizeof(Plaintext));
          printf("Encrypt data\r\n");
          memset(EncryptedText, 0, sizeof(EncryptedText));
          
          /* AES ECB enc/dec */
          if (HAL_CRYP_Encrypt(&hcryp, Plaintext, PLAINTEXT_SIZE, EncryptedText, TIMEOUT_VALUE) != HAL_OK)
          {
            printf(RED"Call HAL_CRYP_Encrypt failed!!\r\n"RESET);
            /* Processing Error */
            Error_Handler();
          }
          
          print_buf("Cipher text: ", (uint8_t*)EncryptedText, sizeof(EncryptedText));

          /* compare the result only if SW key is used */
          if((keyselect == 0)|| (keyselect == 2) || (keyselect == 4))
          {
            printf("Compare result\r\n");
            /*Compare results with expected buffer*/
            if(memcmp(EncryptedText, CiphertextAESECB128, PLAINTEXT_SIZE) != 0)
            {
              printf(RED"Cipher text result is not as expected!!\r\n"RESET);
              /* Processing Error */
              Error_Handler();
            }
            else
            {
              printf(GREEN"Cipher text result is as expected!!\r\n"RESET);
            }
          }
          
          printf("Decrypt\r\n");
          /*##-3- Decryption Phase #################################################*/
          //if (HAL_CRYP_Decrypt(&hcryp,CiphertextAESECB128 , PLAINTEXT_SIZE, DecryptedText, TIMEOUT_VALUE) != HAL_OK)
          if (HAL_CRYP_Decrypt(&hcryp,EncryptedText , PLAINTEXT_SIZE, DecryptedText, TIMEOUT_VALUE) != HAL_OK)
          {
            printf(RED"Call HAL_CRYP_Decrypt failed!!\r\n"RESET);
            /* Processing Error */
            Error_Handler();
          }
          print_buf("Decrypted Plain text: ", (uint8_t*)DecryptedText, sizeof(DecryptedText));
          printf("Compare result\r\n");
          /*Compare results with expected buffer*/
          if(memcmp(DecryptedText, Plaintext, PLAINTEXT_SIZE) != 0)
          {
            printf(RED"Plain text result is not as expected!!\r\n"RESET);
            /* Processing Error */
            Error_Handler();
          }
          else
          {
            printf(GREEN"Plain text result is as expected!!\r\n"RESET);
          }
        }
      }
      else if (keyselect == 'a')
      {
        write_AHK_ECB();
      }
      else if (keyselect == 'b')
      {
        Write_BHK_ECB();
      }
      else if (keyselect == 'r')
      {
        Read_BHK_ECB();
      }
      else if (keyselect == 'l')
      {
        Lock_BHK_ECB();
      }   
      else if (keyselect == 'x')
      {
        break;
      }
      else
      {
        printf("Invalid Number !\r");
      }      
      /* Print Main Menu message */
      PrintMenu_ECB();
    }    
  }  
}

void CBC_TEST(void)
{
  printf("Start AES CBC Test with different keys\r\n");
  
  /* Reset RTC clock */
  __HAL_RCC_RTC_DISABLE();
  __HAL_RCC_RTCAPB_CLK_DISABLE();
  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  
  PrintMenu_CBC();
  while (1)
  {
    uint8_t keyselect = 0U;
    HAL_CRYP_DeInit(&hcryp);

    /* Clean the input path */
    __HAL_UART_FLUSH_DRREGISTER(&hcom_uart[COM1]);
    /* Receive key */
    if (HAL_UART_Receive(&hcom_uart[COM1], &keyselect, 1, 0x2000) == HAL_OK)
    {
      printf("\r\n ====> Your input is %c\r\n", keyselect);
      if ( keyselect >= '1' && keyselect <= '6')
      {
        printf("\r\n ====> Your input is %c\r\n", keyselect);
        if ( keyselect >= '1' && keyselect <= '6')
        {
          keyselect = keyselect - '1';
          
          printf("\r\n ************ Initialize SAES ************\r\n");
          MX_SAES_CRYP_Init_CBC(keyselect);
          printf("Test for key select [%d]\r\n", keyselect);
          
          print_buf("Plain text: ", (uint8_t*)Plaintext, sizeof(Plaintext));
          printf("Encrypt data\r\n");
          memset(EncryptedText, 0, sizeof(EncryptedText));
          /*##-3- Change Secure AES configuration #################################################*/
          
          /* AES CBC enc/dec */
          if (HAL_CRYP_Encrypt(&hcryp, Plaintext, PLAINTEXT_SIZE, EncryptedText, TIMEOUT_VALUE) != HAL_OK)
          {
            printf(RED"Call HAL_CRYP_Encrypt failed!!\r\n"RESET);
            /* Processing Error */
            Error_Handler();
          }
          
          print_buf("Cipher text: ", (uint8_t*)EncryptedText, sizeof(EncryptedText));

          /* compare the result only if SW key is used */
          if((keyselect == 0)|| (keyselect == 2) || (keyselect == 4))
          {
            printf("Compare result\r\n");
            /*Compare results with expected buffer*/
            if(memcmp(EncryptedText, CiphertextAESCBC256, PLAINTEXT_SIZE) != 0)
            {
              printf(RED"Cipher text result is not as expected!!\r\n"RESET);
              /* Processing Error */
              Error_Handler();
            }
            else
            {
              printf(GREEN"Cipher text result is as expected!!\r\n"RESET);
            }
          }
          
          printf("Decrypt\r\n");
          /*##-3- Decryption Phase #################################################*/
          //if (HAL_CRYP_Decrypt(&hcryp,CiphertextAESCBC256 , PLAINTEXT_SIZE, DecryptedText, TIMEOUT_VALUE) != HAL_OK)
          if (HAL_CRYP_Decrypt(&hcryp,EncryptedText , PLAINTEXT_SIZE, DecryptedText, TIMEOUT_VALUE) != HAL_OK)
          {
            printf(RED"Call HAL_CRYP_Decrypt failed!!\r\n"RESET);
            /* Processing Error */
            Error_Handler();
          }
          print_buf("Decrypted Plain text: ", (uint8_t*)DecryptedText, sizeof(DecryptedText));
          printf("Compare result\r\n");
          /*Compare results with expected buffer*/          
          if(memcmp(DecryptedText, Plaintext, PLAINTEXT_SIZE) != 0)
          {
            printf(RED"Plain text result is not as expected!!\r\n"RESET);
            /* Processing Error */
            Error_Handler();
          }
          else
          {
            printf(GREEN"Plain text result is as expected!!\r\n"RESET);
          }
        }
      }
      else if (keyselect == 'a')
      {
        write_AHK_CBC();
      }
      else if (keyselect == 'b')
      {
        Write_BHK_CBC();
      }
      else if (keyselect == 'r')
      {
        Read_BHK_CBC();
      }
      else if (keyselect == 'l')
      {
        Lock_BHK_CBC();
      }
      else if (keyselect == 'x')
      {
        break;
      }
      else
      {
        printf("Invalid Number !\r");
      }      
      /* Print Main Menu message */
      PrintMenu_CBC();
    }    
  } 
  /* USER CODE END 2 */
}

/**
  * @brief SAES Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAES_CRYP_Init_ECB(int keyselect)
{

  /* USER CODE BEGIN SAES_Init 0 */

  /* USER CODE END SAES_Init 0 */

  /* USER CODE BEGIN SAES_Init 1 */

  /* USER CODE END SAES_Init 1 */
  hcryp.Instance = SAES;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.HeaderSize = 7;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
  hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
  hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;    
  hcryp.Init.KeySelect = CRYP_KEYSEL_NORMAL;
  hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
  hcryp.Init.pKey = NULL;
  
  /* USER CODE BEGIN SAES_Init 2 */
  if ( keyselect == 0)
  {
    hcryp.Init.KeySelect = CRYP_KEYSEL_NORMAL;
    hcryp.Init.pKey = (uint32_t *)pKeySAES;    
  }
  else if (keyselect == 1)
  {    
    hcryp.Init.KeySelect = CRYP_KEYSEL_HW;
  }
  else if (keyselect == 2)
  {  
    hcryp.Init.KeySelect = CRYP_KEYSEL_SW;
  }
  else if (keyselect == 3)
  {
    hcryp.Init.KeySelect = CRYP_KEYSEL_HSW;
  }
  else if (keyselect == 4)
  {    
    hcryp.Init.KeySelect = CRYP_KEYSEL_AHK;
  }
  else if (keyselect == 5)
  {
    hcryp.Init.KeySelect = CRYP_KEYSEL_DUK_AHK;
  }  

  /* USER CODE BEGIN SAES_Init 2 */

  if ((keyselect == 4) || (keyselect == 5))
  {
      uint32_t timeout = 0U;
      HAL_StatusTypeDef status = HAL_ERROR;
      FLASH_KeyConfigTypeDef keyCfg = { 0U };
   
      /* enable SBS clock */
      __HAL_RCC_SBS_CLK_ENABLE();
      /* enable HSI 48 MHz */
      __HAL_RCC_HSI48_ENABLE();
      /* enable RNG clock */
      __HAL_RCC_RNG_CLK_ENABLE();
      /* enable SAES clock */
      __HAL_RCC_SAES_CLK_ENABLE();
      
      /* USER CODE BEGIN SAES_Init 0 */
      /* force the SAES Peripheral Clock Reset */
      __HAL_RCC_SAES_FORCE_RESET();
      /* release the SAES Peripheral Clock Reset */
      __HAL_RCC_SAES_RELEASE_RESET();
      
      if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_CRYP_Init(&hcryp) != HAL_OK)
      {
        Error_Handler();
      }
      __DSB();

      (void) HAL_FLASH_Unlock();
      (void) HAL_FLASH_OB_Unlock();

      keyCfg.HDPLLevel = FLASH_KEY_LEVEL_CURRENT;
      keyCfg.Index = 0;

      status = HAL_FLASHEx_GetKey(&keyCfg, NULL);

      if (status != HAL_OK)
      {
        Error_Handler();
      }

      /* Wait the key transfer is completed*/
      timeout = HAL_GetTick() + timeout;
      while (((SAES->SR & SAES_SR_BUSY) != 0U) && ((SAES->SR & SAES_SR_KEYVALID) == 0U))
      {
        if (timeout != HAL_MAX_DELAY)
        {
          if (HAL_GetTick() >= 100)
          {
            Error_Handler();
          }
        }
      }

      (void) HAL_FLASH_OB_Lock();
      (void) HAL_FLASH_Lock();
  }
  else if((keyselect == 2) || (keyselect == 3))
  {
    uint32_t timeout = 0U;
    
    /* enable HSI 48 MHz */
    __HAL_RCC_HSI48_ENABLE();
    /* enable RNG clock */
    __HAL_RCC_RNG_CLK_ENABLE();
    /* enable SAES clock */
    __HAL_RCC_SAES_CLK_ENABLE();
    
    /* USER CODE BEGIN SAES_Init 0 */
    /* force the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_FORCE_RESET();
    /* release the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_RELEASE_RESET();   
    
    if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_CRYP_Init(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    __DSB();
      
    READ_REG(TAMP->BKP0R);
    READ_REG(TAMP->BKP1R);
    READ_REG(TAMP->BKP2R);
    READ_REG(TAMP->BKP3R);    
    READ_REG(TAMP->BKP4R);
    READ_REG(TAMP->BKP5R);
    READ_REG(TAMP->BKP6R);
    READ_REG(TAMP->BKP7R);      
    
    /* Wait the key transfer is completed*/
    timeout = HAL_GetTick() + timeout;
    while (((SAES->SR & SAES_SR_BUSY) != 0U) && ((SAES->SR & SAES_SR_KEYVALID) == 0U))
    {
      if (timeout != HAL_MAX_DELAY)
      {
        if (HAL_GetTick() >= 100)
        {
          Error_Handler();
        }
      }
    }
  }
  else
  {
    /* enable HSI 48 MHz */
    __HAL_RCC_HSI48_ENABLE();
    /* enable RNG clock */
    __HAL_RCC_RNG_CLK_ENABLE();
    /* enable SAES clock */
    __HAL_RCC_SAES_CLK_ENABLE();
    
    /* USER CODE BEGIN SAES_Init 0 */
    /* force the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_FORCE_RESET();
    /* release the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_RELEASE_RESET();   
    
    if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_CRYP_Init(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
  }
  /* USER CODE END SAES_Init 2 */

}

/**
  * @brief SAES Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAES_CRYP_Init_CBC(int keyselect)
{

  /* USER CODE BEGIN SAES_Init 0 */

  /* USER CODE END SAES_Init 0 */

  /* USER CODE BEGIN SAES_Init 1 */

  /* USER CODE END SAES_Init 1 */
  /*
  hcryp.Instance = SAES;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.pKey = (uint32_t *)pKeySAES;
  hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
  hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
  hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
  hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;
  hcryp.Init.KeySelect = CRYP_KEYSEL_NORMAL;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  */
  
  /* USER CODE BEGIN SAES_Init 2 */
  if ( keyselect == 0)
  {
    hcryp.Instance = SAES;
    hcryp.Init.DataType = CRYP_DATATYPE_32B;
    hcryp.Init.KeySize = CRYP_KEYSIZE_256B;
    hcryp.Init.Algorithm = CRYP_AES_CBC;
    hcryp.Init.HeaderSize = 7;
    hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
    hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
    hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;    
    hcryp.Init.KeySelect = CRYP_KEYSEL_NORMAL;
    hcryp.Init.pKey = (uint32_t *)AESKey256;   
    hcryp.Init.pInitVect = (uint32_t *)AESIV;
    hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
  }
  else if (keyselect == 1)
  {    
    hcryp.Init.KeySelect = CRYP_KEYSEL_HW;
  }
  else if (keyselect == 2)
  {  
    hcryp.Init.KeySelect = CRYP_KEYSEL_SW;
  }
  else if (keyselect == 3)
  {
    hcryp.Init.KeySelect = CRYP_KEYSEL_HSW;
  }
  else if (keyselect == 4)
  {    
    hcryp.Init.KeySelect = CRYP_KEYSEL_AHK;
  }
  else if (keyselect == 5)
  {
    hcryp.Init.KeySelect = CRYP_KEYSEL_DUK_AHK;
  }  

  /* USER CODE BEGIN SAES_Init 2 */

  if ((keyselect == 4) || (keyselect == 5))
  {
      uint32_t timeout = 0U;
      HAL_StatusTypeDef status = HAL_ERROR;
      FLASH_KeyConfigTypeDef keyCfg = { 0U };
   
      /* enable SBS clock */
      __HAL_RCC_SBS_CLK_ENABLE();
      /* enable HSI 48 MHz */
      __HAL_RCC_HSI48_ENABLE();
      /* enable RNG clock */
      __HAL_RCC_RNG_CLK_ENABLE();
      /* enable SAES clock */
      __HAL_RCC_SAES_CLK_ENABLE();
      
      /* USER CODE BEGIN SAES_Init 0 */
      /* force the SAES Peripheral Clock Reset */
      __HAL_RCC_SAES_FORCE_RESET();
      /* release the SAES Peripheral Clock Reset */
      __HAL_RCC_SAES_RELEASE_RESET();
      
      if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_CRYP_Init(&hcryp) != HAL_OK)
      {
        Error_Handler();
      }
      __DSB();

      (void) HAL_FLASH_Unlock();
      (void) HAL_FLASH_OB_Unlock();

      keyCfg.HDPLLevel = FLASH_KEY_LEVEL_CURRENT;
      keyCfg.Index = 0;

      status = HAL_FLASHEx_GetKey(&keyCfg, NULL);

      if (status != HAL_OK)
      {
        Error_Handler();
      }

      /* Wait the key transfer is completed*/
      timeout = HAL_GetTick() + timeout;
      while (((SAES->SR & SAES_SR_BUSY) != 0U) && ((SAES->SR & SAES_SR_KEYVALID) == 0U))
      {
        if (timeout != HAL_MAX_DELAY)
        {
          if (HAL_GetTick() >= 100)
          {
            Error_Handler();
          }
        }
      }

      (void) HAL_FLASH_OB_Lock();
      (void) HAL_FLASH_Lock();
  }
  else if((keyselect == 2) || (keyselect == 3))
  {
    uint32_t timeout = 0U;    
    
    /* enable HSI 48 MHz */
    __HAL_RCC_HSI48_ENABLE();
    /* enable RNG clock */
    __HAL_RCC_RNG_CLK_ENABLE();
    /* enable SAES clock */
    __HAL_RCC_SAES_CLK_ENABLE();
    
    /* USER CODE BEGIN SAES_Init 0 */
    /* force the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_FORCE_RESET();
    /* release the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_RELEASE_RESET(); 
    
    if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_CRYP_Init(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    __DSB();
      
    READ_REG(TAMP->BKP0R);
    READ_REG(TAMP->BKP1R);
    READ_REG(TAMP->BKP2R);
    READ_REG(TAMP->BKP3R);
    READ_REG(TAMP->BKP4R);
    READ_REG(TAMP->BKP5R);
    READ_REG(TAMP->BKP6R);
    READ_REG(TAMP->BKP7R); 
    
    /* Wait the key transfer is completed*/
    timeout = HAL_GetTick() + timeout;
    while (((SAES->SR & SAES_SR_BUSY) != 0U) && ((SAES->SR & SAES_SR_KEYVALID) == 0U))
    {
      if (timeout != HAL_MAX_DELAY)
      {
        if (HAL_GetTick() >= 100)
        {
          Error_Handler();
        }
      }
    }
  }
  else
  {
    /* enable HSI 48 MHz */
    __HAL_RCC_HSI48_ENABLE();
    /* enable RNG clock */
    __HAL_RCC_RNG_CLK_ENABLE();
    /* enable SAES clock */
    __HAL_RCC_SAES_CLK_ENABLE();
    
    /* USER CODE BEGIN SAES_Init 0 */
    /* force the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_FORCE_RESET();
    /* release the SAES Peripheral Clock Reset */
    __HAL_RCC_SAES_RELEASE_RESET();   
    
    
    if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_CRYP_Init(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
  }
  /* USER CODE END SAES_Init 2 */

}


/* USER CODE BEGIN 4 */


