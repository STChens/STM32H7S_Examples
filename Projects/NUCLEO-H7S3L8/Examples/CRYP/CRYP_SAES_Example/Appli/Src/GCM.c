/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gcm.c
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
/* The size of the plaintext is in words */
#define PLAINTEXT_SIZE    12 /* Plaintext size in Words */

/* The key size must be 128 bits (16 bytes), 192 bits (24 bytes) or 256 bits (32 bytes) */
#define KEY_SIZE          8 /* Key size in word */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static CRYP_HandleTypeDef hcryp;
static uint32_t pKeySAES[8] = {0xE3C08A8F,0x06C6E3AD,0x95A70557,0xB23F7548,0x3CE33021,0xA9C72B70,0x25666204,0xC69C0B72};
static uint32_t pInitVectSAES[4] = {0x12153524,0xC0895E81,0xB2C28465,0x00000002};
__ALIGN_BEGIN static const uint32_t HeaderSAES[7] __ALIGN_END = {
                            0xD609B1F0,0x56637A0D,0x46DF998D,0x88E52E00,0xB2C28465,0x12153524,0xC0895E81};

/* USER CODE BEGIN PV */

/* The size of the plaintext is in words */
#define PLAINTEXT_SIZE    12

static uint32_t Plaintext[12] = {0x08000F10,0x11121314,0x15161718,0x191A1B1C
                         ,0x1D1E1F20,0x21222324,0x25262728,0x292A2B2C
                         ,0x2D2E2F30,0x31323334,0x35363738,0x393A0002};

static uint32_t Ciphertext[12] = {0xE2006EB4,0x2F527702,0x2D9B1992,0x5BC419D7
                          ,0xA592666C,0x925FE2EF,0x718EB4E3,0x08EFEAA7
                          ,0xC5273B39,0x4118860A,0x5BE2A97F,0x56AB7836};
static uint32_t ExpectedTAG[4]= {0x5CA597CD,0xBB3EDB8D,0x1A1151EA,0x0AF7B436};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

static void MX_SAES_CRYP_Init(int keyselect);
static void PrintMenu(void);
static void Read_BHK(void);
static void Lock_BHK(void);
static void Write_BHK(void);

/* USER CODE BEGIN PFP */

/* Used for storing the encrypted text */
static uint32_t EncryptedText[12]={0};

/* Used for storing the decrypted text */
static uint32_t DecryptedText[12]={0};

/* Used for storing the computed MAC (aTAG) */
static uint32_t TAG[4]={0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PrintMenu(void)
{
  printf("\r\n============== SAES GCM Test Menu =================\r\n\n");
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

static void write_AHK(void)
{
  uint32_t key[8];
  uint32_t destination = (uint32_t)&key;
  FLASH_KeyConfigTypeDef keyConfig = { 0U };
  
  key[0] = pKeySAES[7];
  key[1] = pKeySAES[6];
  key[2] = pKeySAES[5];
  key[3] = pKeySAES[4];
  key[4] = pKeySAES[3];
  key[5] = pKeySAES[2];
  key[6] = pKeySAES[1];
  key[7] = pKeySAES[0];
  
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

static void Lock_BHK(void)
{
  TAMP->CFGR |= TAMP_CFGR_BHKLOCK;
}

static void Write_BHK(void)
{
  TAMP->BKP0R = pKeySAES[7];
  TAMP->BKP1R = pKeySAES[6];
  TAMP->BKP2R = pKeySAES[5];
  TAMP->BKP3R = pKeySAES[4];
  TAMP->BKP4R = pKeySAES[3];
  TAMP->BKP5R = pKeySAES[2];
  TAMP->BKP6R = pKeySAES[1];
  TAMP->BKP7R = pKeySAES[0];
  
}
static void Read_BHK(void)
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void GCM_TEST(void)
{
  printf("Start AES GCM Test with different keys\r\n");
  
  /* Reset RTC clock */
  __HAL_RCC_RTC_DISABLE();
  __HAL_RCC_RTCAPB_CLK_DISABLE();
  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  
  PrintMenu();
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
        keyselect = keyselect - '1';
        
          printf("\r\n ************ Initialize SAES ************\r\n");
          MX_SAES_CRYP_Init(keyselect);
          printf("Test for key select [%d]\r\n", keyselect);
          
          print_buf("Plain text: ", (uint8_t*)Plaintext, sizeof(Plaintext));
          /* USER CODE BEGIN 2 */
          printf("Encrypt data\r\n");
          memset(EncryptedText, 0, sizeof(EncryptedText));
          
          /*##-2- Encryption Phase #################################################*/
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
            if(memcmp(EncryptedText, Ciphertext, PLAINTEXT_SIZE) != 0)
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
          
          printf("Compute TAG\r\n");
          /* Compute the authentication TAG */
          if (HAL_CRYPEx_AESGCM_GenerateAuthTAG(&hcryp,TAG, TIMEOUT_VALUE) != HAL_OK)
          {
            printf(RED"Call HAL_CRYPEx_AESGCM_GenerateAuthTAG failed!!\r\n"RESET);
            /* Processing Error */
            Error_Handler();
          }
          
          print_buf("TAG: ", (uint8_t*)TAG, sizeof(TAG));
          /* compare the result only if SW key is used */
          if((keyselect == 0)|| (keyselect == 2) || (keyselect == 4))
          {          
            /*Compare results with expected buffer*/          
            printf("Compare result\r\n");
            if(memcmp(TAG, ExpectedTAG, PLAINTEXT_SIZE) != 0)
            {
              printf(RED"GCM TAG result is not as expected!!\r\n"RESET);
              /* Processing Error */
              Error_Handler();
            }
            else
            {
              printf(GREEN"GCM TAG result is as expected!!\r\n"RESET);
            }
          }
          printf("Decrypt\r\n");
          /*##-3- Decryption Phase #################################################*/
          //if (HAL_CRYP_Decrypt(&hcryp,Ciphertext , PLAINTEXT_SIZE, DecryptedText, TIMEOUT_VALUE) != HAL_OK)
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
      else if (keyselect == 'a')
      {
        write_AHK();
      }
      else if (keyselect == 'b')
      {
        Write_BHK();
      }
      else if (keyselect == 'r')
      {
        Read_BHK();
      }
      else if (keyselect == 'l')
      {
        Lock_BHK();
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
      PrintMenu();
    }    
  }    
  
  /* USER CODE END 2 */
}

/**
  * @brief SAES Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAES_CRYP_Init(int keyselect)
{
  /* USER CODE END SAES_Init 0 */

  /* USER CODE BEGIN SAES_Init 1 */

  /* USER CODE END SAES_Init 1 */
  
  hcryp.Instance = SAES;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_256B;
  hcryp.Init.Algorithm = CRYP_AES_GCM;
  hcryp.Init.Header = (uint32_t *)HeaderSAES;
  hcryp.Init.HeaderSize = 7;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
  hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
  hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;
  
  hcryp.Init.pInitVect = (uint32_t *)pInitVectSAES;
  hcryp.Init.pKey = NULL;
  
  if ( keyselect == 0)
  {
    hcryp.Init.KeySelect = CRYP_KEYSEL_NORMAL;
    hcryp.Init.pKey = (uint32_t *)pKeySAES;    
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
