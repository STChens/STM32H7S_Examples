/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NB_KEYS 32
#define KEY_256BIT_INDEX 0U
#define KEY_128BIT_INDEX (KEY_256BIT_INDEX+0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RNG_HandleTypeDef hrng;

/* USER CODE BEGIN PV */
FLASH_KeyConfigTypeDef KeyConfig;
uint32_t ProgKey128[4]; // buffer for 128 bit key current HDPL
uint32_t ProgKey128_N[4]; // buffer for 128 bit key next HDPL
uint32_t ProgKey256[8]; // buffer for 256 bit key current HDPL
uint32_t ProgKey256_N[8]; // buffer for 256 bit key next HDPL

uint32_t ReadKey128[4];
uint32_t ReadKey256[8];

CRYP_HandleTypeDef hcryp;

#define  TIMEOUT_VALUE 0xFFF

const uint32_t pKeySAES[4] = {0x2B7E1516,0x28AED2A6,0xABF71588,0x09CF4F3C};

/* USER CODE BEGIN PV */

static CRYP_ConfigTypeDef Conf;

/* The size of the plaintext is in words */
#define PLAINTEXT_SIZE    16

const uint32_t AESIV[4] =   {0x00010203 , 0x04050607 , 0x08090A0B , 0x0C0D0E0F};

const uint32_t AESKey256[8] = {0x603DEB10 ,0x15CA71BE ,0x2B73AEF0 ,0x857D7781 ,0x1F352C07 ,0x3B6108D7 ,0x2D9810A3 ,0x0914DFF4};
const uint32_t AESKey256_REV[8] = {
  0x0914DFF4,
  0x2D9810A3,
  0x3B6108D7,
  0x1F352C07,
  0x857D7781,
  0x2B73AEF0,
  0x15CA71BE,
  0x603DEB10, 
};

uint32_t Plaintext[16] = { 0x6BC1BEE2 ,0x2E409F96 ,0xE93D7E11 ,0x7393172A ,

                          0xAE2D8A57 ,0x1E03AC9C ,0x9EB76FAC ,0x45AF8E51 ,
                          0x30C81C46 ,0xA35CE411 ,0xE5FBC119 ,0x1A0A52EF ,
                          0xF69F2445 ,0xDF4F9B17 ,0xAD2B417B ,0xE66C3710};

const uint32_t CiphertextAESECB128[16] = {0x3AD77BB4 ,0x0D7A3660 ,0xA89ECAF3 ,0x2466EF97 ,
                           0xF5D3D585 ,0x03B9699D ,0xE785895A ,0x96FDBAAF ,
                           0x43B1CD7F ,0x598ECE23 ,0x881B00E3 ,0xED030688 ,
                           0x7B0C785E ,0x27E8AD3F ,0x82232071 ,0x04725DD4};

const uint32_t CiphertextAESCBC256[16] = {0xF58C4C04 ,0xD6E5F1BA ,0x779EABFB ,0x5F7BFBD6 ,
                          0x9CFC4E96 ,0x7EDB808D ,0x679F777B ,0xC6702C7D ,
                          0x39F23369 ,0xA9D9BACF ,0xA530E263 ,0x04231461 ,
                          0xB2EB05E2 ,0xC39BE9FC ,0xDA6C1907 ,0x8C6A9D1B};
uint32_t EncryptedText[PLAINTEXT_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t get_hdpl(void);
static void inc_hdpl(void);
static void print_buf(char *str, uint8_t *buf, size_t size);
static void prog_obk_128(uint32_t curr_next);
static void read_obk_128(uint32_t curr_next);
static void prog_obk_256(uint32_t curr_next);
static void read_obk_256(uint32_t curr_next);
static void MX_SAES_CRYP_Init(void);
static void test_ahk(int hdpl);
static void LL_SECU_ConfigureSAES(CRYP_HandleTypeDef *hCryp, uint32_t HDPLLevel, uint32_t SaesTimeout,
                           uint32_t AHKIndex);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void read_SBS_reg(void)
{
  __HAL_RCC_SBS_CLK_ENABLE();
  printf("READ SBS HDPL and DBG registers:\r\n");
  printf("SBS DBGCR [0x%08X] \r\n", SBS->DBGCR);
  printf("SBS DBGLOCKR [0x%08X] \r\n", SBS->DBGLOCKR);
  printf("SBS HDPLCR [0x%08X] \r\n", SBS->HDPLCR);
  printf("SBS HDPLSR [0x%08X] \r\n", SBS->HDPLSR);
  printf("DBGMCU CR [0x%08X] \r\n", DBGMCU->CR);
  printf("DBGMCU SR [0x%08X] \r\n", DBGMCU->SR);
}

static void open_debug(void)
{
  __HAL_RCC_SBS_CLK_ENABLE();

  printf("Test SBS DBG \r\n");
  printf("==> Read SBS DBG CR register [%08X]\r\n", SBS->DBGCR);
  printf("==> Enable AP and DBG from SBS\r\n");
  uint32_t dbgcr = SBS->DBGCR;
  dbgcr = dbgcr & 0xFF000000 | 0x51b4b4;
  SBS->DBGCR = dbgcr;
  SBS->DBGLOCKR = 0x0000006a;
  DBGMCU->CR |= 0x00010000; /* set bit16 as 1 to reset SBS under power reset instead of system reset*/
  read_SBS_reg();
  while(1){}
}

static void regression(void)
{
  FLASH_OBProgramInitTypeDef flash_option_bytes = {0};
  HAL_StatusTypeDef ret = HAL_ERROR;
  
  flash_option_bytes.OptionType = OPTIONBYTE_NV;  
  HAL_FLASHEx_OBGetConfig(&flash_option_bytes);
  
  flash_option_bytes.NVState = OB_NVSTATE_OPEN;
  printf("Setting NVSTATE to 0x%X ...\r\n", flash_option_bytes.NVState);

   /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

  /* Unlock the Options Bytes */
  HAL_FLASH_OB_Unlock();


  printf("Program state ...\r\n");

  ret = HAL_FLASHEx_OBProgram(&flash_option_bytes);
  if (ret != HAL_OK)
  {
    printf("Error while setting OB Bank1 config state!\r\n");
    Error_Handler();
  }

  printf("After OB programming: system reset !\r\n");  
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();

}

static void prog_obk_128(uint32_t curr_next)
{
  int i;
  uint32_t hdpl = get_hdpl();
  uint32_t *pDataProg = ((hdpl == 0x51) && (curr_next == FLASH_KEY_LEVEL_CURRENT))? ProgKey128 : ProgKey128_N;
  
  printf("Program %d set of 128 bit keys in %s\r\n",NB_KEYS, (curr_next == FLASH_KEY_LEVEL_CURRENT)? "CURRENT HDPL" : "NEXT HDPL");
  
  /* copy AES 128 key to buffer for programming */
  memcpy(pDataProg, pKeySAES, sizeof(pKeySAES));
      
  for(i = 0 ; i < NB_KEYS; i++)
  {
    /* Program a 128-bit key */
    KeyConfig.Size = FLASH_KEY_128_BITS;
    KeyConfig.HDPLLevel = curr_next;
    KeyConfig.Index = KEY_128BIT_INDEX + i;
    /* Change the content of the first byte for the next index */    
    pDataProg[0]++;
    /* increase the value in second word if programming for next level */
    pDataProg[1] += curr_next;
        
    if (HAL_FLASHEx_KeyConfig(&KeyConfig, pDataProg) != HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      printf("Index %d programmed\r\n", KeyConfig.Index);
      
      /* Read back the key to check */
      memset(ReadKey128, 0, sizeof(ReadKey128));
      if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey128) != HAL_OK)
      {
        Error_Handler();
      }
      else
      {
        printf("Index [%d]  ", KEY_128BIT_INDEX + i);
        print_buf("Prog data", (uint8_t*)pDataProg, sizeof(pKeySAES));        
        print_buf("Read data", (uint8_t*)&ReadKey128[0], sizeof(ReadKey128));
      }
    }
  }
}

static void prog_obk_256(uint32_t curr_next)
{
  int i;
  uint32_t hdpl = get_hdpl();
  uint32_t *pDataProg = ((hdpl == 0x51) && (curr_next == FLASH_KEY_LEVEL_CURRENT))? ProgKey256 : ProgKey256_N;
  
  printf("Program %d set of 256 bit keys in %s\r\n",NB_KEYS, (curr_next == FLASH_KEY_LEVEL_CURRENT)? "CURRENT HDPL" : "NEXT HDPL");
  memcpy(pDataProg, AESKey256_REV, sizeof(AESKey256_REV));
  
  for(i = 0 ; i < NB_KEYS; i++)
  {
    /* Program a 256-bit key */
    KeyConfig.Size = FLASH_KEY_256_BITS;
    KeyConfig.HDPLLevel = curr_next;  
    KeyConfig.Index = KEY_256BIT_INDEX + i;
    /* Change the content of the first byte for the next index */
    pDataProg[0]++;
    /* increase the value in second word if programming for next level */
    pDataProg[1] += curr_next;

    if (HAL_FLASHEx_KeyConfig(&KeyConfig, pDataProg) != HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      printf("Index %d programmed\r\n", KeyConfig.Index);
            
      /* Read back the key to check */
      memset(ReadKey256, 0, sizeof(ReadKey256));
      if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey256) != HAL_OK)
      {
        Error_Handler();
      }
      else
      {
        printf("Index [%d]  ", KEY_256BIT_INDEX + i);
        print_buf("Prog data", (uint8_t*)pDataProg, sizeof(AESKey256_REV));
        print_buf("Read data", (uint8_t*)&ReadKey256[0], sizeof(ReadKey256));
      }
    }
  }
}

static void read_obk_128(uint32_t curr_next)
{
  int i;
  printf("Read %d set of 128 bit keys in %s\r\n",NB_KEYS, (curr_next == FLASH_KEY_LEVEL_CURRENT)? "CURRENT HDPL" : "NEXT HDPL");
  memset(ReadKey128, 0, sizeof(ReadKey128));
  
  for(i = 0 ; i < NB_KEYS; i++)
  {
    KeyConfig.HDPLLevel = curr_next;

    /* Read back 128-bit key value and check consistency */
    KeyConfig.Size = FLASH_KEY_128_BITS;
    KeyConfig.Index = KEY_128BIT_INDEX+i;
    if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey128) != HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      printf("Index [%d]  ", KEY_128BIT_INDEX+i);
      print_buf("read data", (uint8_t*)&ReadKey128[0], sizeof(ReadKey128));
    }
  }
}

static void read_obk_256(uint32_t curr_next)
{
  int i;
  printf("Read %d set of 256 bit keys in %s\r\n", NB_KEYS, (curr_next == FLASH_KEY_LEVEL_CURRENT)? "CURRENT HDPL" : "NEXT HDPL");
  memset(ReadKey256, 0, sizeof(ReadKey256));
  
  for(i = 0 ; i < NB_KEYS; i++)
  {
    KeyConfig.HDPLLevel = curr_next;

    /* Read back 256-bit key value and check consistency => no key expected  */
    KeyConfig.Size = FLASH_KEY_256_BITS;
    KeyConfig.Index = KEY_256BIT_INDEX+i;
    if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey256) != HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      printf("Index [%d]  ", KEY_256BIT_INDEX+i);
      print_buf("read data", (uint8_t*)&ReadKey256[0], sizeof(ReadKey256));
    }
  }
}

static uint32_t get_hdpl(void)
{
  uint32_t hdpl;
  __HAL_RCC_SBS_CLK_ENABLE();
  hdpl = SBS->HDPLSR;
  switch(hdpl)
  {
    case 0x51: printf("HDP level 1\r\n"); 
      break;
    case 0xB4: printf("HDP level 0\r\n"); 
      break;
    case 0x8A: printf("HDP level 2\r\n"); 
      break;
    case 0x6F:
    default: printf("HDP level 3!!\r\n");
      break;
  }
  return hdpl;
}

static void inc_hdpl(void)
{
  __HAL_RCC_SBS_CLK_ENABLE();
  printf("Before HDPl increase : ");
  get_hdpl();
  SBS->HDPLCR = 0x6A;
  printf("After HDPl increase : ");
  get_hdpl();  
}

static void print_buf(char *str, uint8_t *buf, size_t size)
{
  int i;
  
  if(str!=NULL)
  {
    printf(">>> %s\r\n", str);
  }
  else
  {
    printf(">>> Data:\r\n");
  }
  printf("=============================================\r\n");
  
  for (i = 0;i<size;i++)
  {
    printf("%02x ", buf[i]);
    if( (i+1)%16 == 0 )
    {
      printf("\r\n");
    }
  }
  printf("\r\n=============================================\r\n");
    
}


/**
  * @brief  Configure SAES peripheral with AHK
  * @param  hCryp SAES handle reference
  * @param  HDPLLevel HDP level key to encrypt/decrypt
  * @param  SaesTimeout timeout of SAES
  * @retval None
  */
static void LL_SECU_ConfigureSAES(CRYP_HandleTypeDef *hCryp, uint32_t HDPLLevel, uint32_t SaesTimeout,
                           uint32_t AHKIndex)
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

  /* force the SAES Peripheral Clock Reset */
  __HAL_RCC_SAES_FORCE_RESET();
  /* release the SAES Peripheral Clock Reset */
  __HAL_RCC_SAES_RELEASE_RESET();

  if (HAL_CRYP_DeInit(hCryp) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure SAES to use 256-Byte AHK */
  if (HAL_CRYP_Init(hCryp) != HAL_OK)
  {
    Error_Handler();
  }

  __DSB();

  (void) HAL_FLASH_Unlock();
  (void) HAL_FLASH_OB_Unlock();

  keyCfg.HDPLLevel = HDPLLevel;
  keyCfg.Index = AHKIndex;

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
      if (HAL_GetTick() >= SaesTimeout)
      {
        Error_Handler();
      }
    }
  }

  (void) HAL_FLASH_OB_Lock();
  (void) HAL_FLASH_Lock();
}


static void test_ahk(int hdpl)
{
  int i;  

  
  if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
  {
   printf("Error %s:%d!!\r\n", __FUNCTION__, __LINE__);
   Error_Handler();
  }

  hcryp.Instance = SAES;
  hcryp.Init.DataType = CRYP_NO_SWAP;
  //hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  //hcryp.Init.pKey = (uint32_t *)pKeySAES;
  //hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
  hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
  hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
  hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;
  hcryp.Init.KeySelect = CRYP_KEYSEL_NORMAL;
  
  hcryp.Init.Algorithm = CRYP_AES_CBC;
  
  hcryp.Init.KeySize = CRYP_KEYSIZE_256B;
  hcryp.Init.pKey = (uint32_t *)AESKey256;
  hcryp.Init.pInitVect = (uint32_t *)AESIV;
  
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
#if 0
  printf("Test AES-128 CBC\r\n");
  memset(EncryptedText, 0, sizeof(EncryptedText));
  /*##-2- Secure AES ECB Encryption/Decryption #################################################*/
  if (HAL_CRYP_Encrypt(&hcryp, Plaintext, PLAINTEXT_SIZE, EncryptedText, TIMEOUT_VALUE) != HAL_OK)
  {
    /* Processing Error */
    Error_Handler();
  }
  /*Compare results with expected buffer*/
  if(memcmp(EncryptedText, CiphertextAESECB128, PLAINTEXT_SIZE) != 0)
  {
    /* Processing Error */
    Error_Handler();
  }
  
  print_buf("Plain text", (uint8_t*)Plaintext, sizeof(Plaintext));
  print_buf("Cypher text", (uint8_t*)EncryptedText, sizeof(EncryptedText));

  /*##-3- Change Secure AES configuration #################################################*/
#endif
  printf("Test AES-256 CBC\r\n");
#if 0  
  /* Get the Secure AES parameters */
  HAL_CRYP_GetConfig(&hcryp, &Conf);

  /* Change CRYP parameters */
  Conf.KeySize   = CRYP_KEYSIZE_256B;
  Conf.pKey    = AESKey256;
  Conf.Algorithm = CRYP_AES_CBC;
  Conf.pInitVect = AESIV;

  /* Set the CRYP Configuration */
  HAL_CRYP_SetConfig(&hcryp, &Conf);
#endif
  /*##-4- Secure AES CBC Encryption/Decryption #################################################*/
  if (HAL_CRYP_Encrypt(&hcryp, Plaintext, PLAINTEXT_SIZE, EncryptedText, TIMEOUT_VALUE) != HAL_OK)
  {
    /* Processing Error */
    Error_Handler();
  }
  /*Compare results with expected buffer*/
  if(memcmp(EncryptedText, CiphertextAESCBC256, PLAINTEXT_SIZE) != 0)
  {
    /* Processing Error */
    Error_Handler();
  }
  
  print_buf("Plain text", (uint8_t*)Plaintext, sizeof(Plaintext));
  print_buf("Cypher text", (uint8_t*)EncryptedText, sizeof(EncryptedText));
    
  for ( i = 0 ; i< 8; i++)
  {
#if 0
    if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    
    hcryp.Instance = SAES;
    hcryp.Init.DataType = CRYP_NO_SWAP;
    hcryp.Init.KeySize = CRYP_KEYSIZE_256B;
    hcryp.Init.Algorithm = CRYP_AES_ECB;
    hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
    hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
    hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
    hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;
    hcryp.Init.KeySelect = CRYP_KEYSEL_AHK;
    if (HAL_CRYP_Init(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    
    KeyConfig.Size = FLASH_KEY_256_BITS;
    KeyConfig.Index = i;
    KeyConfig.HDPLLevel = hdpl;
    if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey256) != HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      printf("Do encrypt with AHK index %d\r\n", i);
      if (HAL_CRYP_Encrypt(&hcryp, Plaintext, 16, EncryptedText, TIMEOUT_VALUE) != HAL_OK)
      {
        /* Processing Error */
        Error_Handler();
      }
      else
      {
        print_buf("Plain text", (uint8_t*)Plaintext, sizeof(Plaintext));
        print_buf("Cypher text", (uint8_t*)EncryptedText, sizeof(EncryptedText));
      }
    }
#else
    if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
    
    printf("Do encrypt with AHK index %d for %s\r\n", i, (hdpl == FLASH_KEY_LEVEL_CURRENT)?"HDPL1": "HDPL2");
    
    hcryp.Instance            = SAES;
    hcryp.Init.DataType       = CRYP_NO_SWAP;
    hcryp.Init.Algorithm      = CRYP_AES_CBC;
    hcryp.Init.KeySelect      = CRYP_KEYSEL_AHK;
    hcryp.Init.KeyMode        = CRYP_KEYMODE_NORMAL;
    hcryp.Init.KeySize        = CRYP_KEYSIZE_256B;       /* 256 bits AES Key*/
  
    hcryp.Init.pInitVect = (uint32_t *)AESIV;
    
    hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
    hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
    hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
    hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;
    
    LL_SECU_ConfigureSAES(&hcryp, (hdpl == FLASH_KEY_LEVEL_CURRENT)? FLASH_KEY_LEVEL_CURRENT : FLASH_KEY_LEVEL_NEXT, TIMEOUT_VALUE, i);

    if (HAL_CRYP_Encrypt(&hcryp, (uint32_t *)Plaintext, sizeof(Plaintext)/4, EncryptedText,
                         TIMEOUT_VALUE) != HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      print_buf("Plain text", (uint8_t*)Plaintext, sizeof(Plaintext));
      print_buf("Cypher text", (uint8_t*)EncryptedText, sizeof(EncryptedText));
    }
  
    if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
    {
      Error_Handler();
    }
#endif
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t index;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if (BSP_LED_Init(LED_GREEN) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  if (BSP_LED_Init(LED_RED) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */   
  MX_SAES_CRYP_Init();
  
  COM_InitTypeDef COM_Init;
  COM_Init.BaudRate = 115200;
  COM_Init.HwFlowCtl = COM_HWCONTROL_NONE;
  COM_Init.Parity = COM_PARITY_NONE;
  COM_Init.StopBits = COM_STOPBITS_1;
  COM_Init.WordLength = COM_WORDLENGTH_8B;  
  BSP_COM_Init(COM1, &COM_Init);
  printf("\r\nCOM Init done.\r\n");
  
  /* Initialize key table with random values */
  for (index = 0U; index < 4U; index++)
  {
    if (HAL_RNG_GenerateRandomNumber(&hrng, &(ProgKey128[index])) != HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      ProgKey128_N[index] = ~ProgKey128[index];
    }
  }
  print_buf("ProgKey128", (uint8_t*)&ProgKey128[0], sizeof(ProgKey128));
  print_buf("ProgKey128_N", (uint8_t*)&ProgKey128_N[0], sizeof(ProgKey128_N));

  for (index = 0U; index < 8U; index++)
  {
    if (HAL_RNG_GenerateRandomNumber(&hrng, &(ProgKey256[index])) != HAL_OK)
    {
      Error_Handler();
    }    
    else
    {
      ProgKey256_N[index] = ~ProgKey256[index];
    }
  }
  print_buf("ProgKey256", (uint8_t*)&ProgKey256[0], sizeof(ProgKey256));
  print_buf("ProgKey256_N", (uint8_t*)&ProgKey256_N[0], sizeof(ProgKey256_N));

  /* Unlock Flash interface */
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    Error_Handler();
  }

  /* Unlock Option Bytes access */
  if (HAL_FLASH_OB_Unlock() != HAL_OK)
  {
    Error_Handler();
  }
  
  get_hdpl();
  {
    extern UART_HandleTypeDef hcom_uart[COMn];
    int loop=1;

    // Check UART
    while(loop == 1)
    {
      uint8_t response;
      printf("\r\n ============= Test menu ============== \r\n");
      printf("Open debug                 ---------- d\n");
      printf("Show sbs register value    ---------- s\n");
      printf("Regression                 ---------- r\n");
      
      printf("Show current HDPL          ---------- h\n");
      printf("Increase HDP level         ---------- i\n");
      printf("Program OBK 128 Current (idx %d)    ---------- 1\n", KEY_128BIT_INDEX);
      printf("Program OBK 128 Next (idx %d)       ---------- 2\n", KEY_128BIT_INDEX);
      printf("Program OBK 256 Current (idx %d)    ---------- 3\n", KEY_256BIT_INDEX);
      printf("Program OBK 256 Next (idx %d)       ---------- 4\n", KEY_256BIT_INDEX);
      printf("Read OBK 128 Current (idx %d)    ---------- 5\n", KEY_128BIT_INDEX);
      printf("Read OBK 128 Next (idx %d)       ---------- 6\n", KEY_128BIT_INDEX);
      printf("Read OBK 256 Current (idx %d)    ---------- 7\n", KEY_256BIT_INDEX);
      printf("Read OBK 256 Next (idx %d)       ---------- 8\n", KEY_256BIT_INDEX);
      printf("AHK Test                        ---------- a\n");
      printf("Exit                       ---------- x\n");
      while (HAL_UART_Receive(&hcom_uart[COM1], &response, 1, 1000) == HAL_TIMEOUT);
      printf("Your input is : %c\r\n", response);
      switch (response)
      {
        
      case 'a': 
        printf("===> Test AHK + SAES for current HDPL (HDPL1)!\r\n");
        test_ahk(FLASH_KEY_LEVEL_CURRENT);
        printf("\r\n===> Test AHK + SAES for next HDPL (HDPL2)!\r\n");
        test_ahk(FLASH_KEY_LEVEL_NEXT);
        break;
      case 'd':
        printf("===> Recovery => open debug!\r\n");
        open_debug();
      case 's':
        printf("===> Show => sbs register state!\r\n");
        read_SBS_reg();
        break;
      case 'r':
        printf("===> Launch regression NOT SUPPORTED!\r\n");        
        regression();        
      case '1':
        printf("===> Program 128 bit obk in current HDPL\r\n");
        prog_obk_128(FLASH_KEY_LEVEL_CURRENT);
        break;
      case '2':
        printf("===> Program 128 bit obk in next HDPL\r\n");
        prog_obk_128(FLASH_KEY_LEVEL_NEXT);
        break;
      case '3':
        printf("===> Program 256 bit obk in current HDPL\r\n");
        prog_obk_256(FLASH_KEY_LEVEL_CURRENT);
        break;
      case '4':
        printf("===> Program 256 bit obk in next HDPL\r\n");
        prog_obk_256(FLASH_KEY_LEVEL_NEXT);
        break;
      case '5':
        printf("===> Read 128 bit obk in current HDPL\r\n");
        read_obk_128(FLASH_KEY_LEVEL_CURRENT);
        break;
      case '6':
        printf("===> Read 128 bit obk in next HDPL\r\n");
        read_obk_128(FLASH_KEY_LEVEL_NEXT);
        break;
      case '7':
        printf("===> Read 256 bit obk in current HDPL\r\n");
        read_obk_256(FLASH_KEY_LEVEL_CURRENT);
        break;
      case '8':
        printf("===> Read 256 bit obk in next HDPL\r\n");
        read_obk_256(FLASH_KEY_LEVEL_NEXT);
        break;
      case 'i':
        printf("===> Increase HDPL\r\n");
        inc_hdpl();
        break;
      case 'h':
        printf("===> Get current HDPL\r\n");
        get_hdpl();
        break;
      case 'x':
        loop = 0;
        break;
      default :
        printf("===> Bad input!\r\n");
        break;
      }
    }
  }
  printf(" Exit from menu, start OBK test...\r\n");

  /* ----------- Programming keys with several size and HDP level ----------- */
  /* Program a 128-bit key */
  KeyConfig.Size = FLASH_KEY_128_BITS;
  KeyConfig.Index = KEY_128BIT_INDEX;
  KeyConfig.HDPLLevel = FLASH_KEY_LEVEL_CURRENT;
  if (HAL_FLASHEx_KeyConfig(&KeyConfig, ProgKey128) != HAL_OK)
  {
    Error_Handler();
  }

  /* Program a 256-bit key */
  KeyConfig.Size = FLASH_KEY_256_BITS;
  KeyConfig.Index = KEY_256BIT_INDEX;
  KeyConfig.HDPLLevel = FLASH_KEY_LEVEL_NEXT;
  if (HAL_FLASHEx_KeyConfig(&KeyConfig, ProgKey256) != HAL_OK)
  {
    Error_Handler();
  }
  /* ----------------------- End of keys programming ------------------------ */
    

  /* --------------- Check available keys according HDP level --------------- */
  /* --> Current HDL level */
  KeyConfig.HDPLLevel = FLASH_KEY_LEVEL_CURRENT;

  /* Read back 128-bit key value and check consistency */
  KeyConfig.Size = FLASH_KEY_128_BITS;
  KeyConfig.Index = KEY_128BIT_INDEX;
  if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey128) != HAL_OK)
  {
    Error_Handler();
  }

  /* 128-bit key has been programmed for current level => no error expected */
  for (index = 0U; index < 4U; index++)
  {
    if (ProgKey128[index] != ReadKey128[index])
    {
      Error_Handler();
    }
  }

  /* Read back 256-bit key value and check consistency => no key expected  */
  KeyConfig.Size = FLASH_KEY_256_BITS;
  KeyConfig.Index = KEY_256BIT_INDEX;
  if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey256) == HAL_OK)
  {
    Error_Handler();
  }

  /* 256-bit key has been programmed for next level => no key expected */
  if ((HAL_FLASH_GetError() & HAL_FLASH_ERROR_KV) == 0U)
  {
    Error_Handler();
  }

  /* --> Next HDP level */
  KeyConfig.HDPLLevel = FLASH_KEY_LEVEL_NEXT;

  /* Read back 128-bit key value and check consistency => no key expected  */
  KeyConfig.Size = FLASH_KEY_128_BITS;
  KeyConfig.Index = KEY_128BIT_INDEX;
  if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey128) == HAL_OK)
  {
    Error_Handler();
  }

  /* 128-bit key has been programmed for current level => no key expected */
  if ((HAL_FLASH_GetError() & HAL_FLASH_ERROR_KV) == 0U)
  {
    Error_Handler();
  }

  /* Read back 256-bit key value and check consistency */
  KeyConfig.Size = FLASH_KEY_256_BITS;
  KeyConfig.Index = KEY_256BIT_INDEX;
  if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey256) != HAL_OK)
  {
    Error_Handler();
  }

  /* 256-bit key has been programmed for next level => no error expected */
  for (index = 0U; index < 8U; index++)
  {
    if (ProgKey256[index] != ReadKey256[index])
    {
      Error_Handler();
    }
  }
  /* ------------------ End of check for current HDP level ------------------ */

  /* ---------------------------- Increment HDPL ---------------------------- */
  HAL_SBS_IncrementHDPLValue();

  /* --------------- Check available keys according HDP level --------------- */
  /* --> Current HDL level */
  KeyConfig.HDPLLevel = FLASH_KEY_LEVEL_CURRENT;

  /* Read back 128-bit key value and check consistency */
  KeyConfig.Size = FLASH_KEY_128_BITS;
  KeyConfig.Index = KEY_128BIT_INDEX;
  if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey128) == HAL_OK)
  {
    Error_Handler();
  }

  /* 128-bit key has been programmed at previous level => no key expected */
  if ((HAL_FLASH_GetError() & HAL_FLASH_ERROR_KV) == 0U)
  {
    Error_Handler();
  }

  /* Read back 256-bit key value and check consistency */
  KeyConfig.Size = FLASH_KEY_256_BITS;
  KeyConfig.Index = KEY_256BIT_INDEX;
  if (HAL_FLASHEx_GetKey(&KeyConfig, ReadKey256) != HAL_OK)
  {
    Error_Handler();
  }

  /* 256-bit key has been programmed at this level => no error expected */
  for (index = 0U; index < 8U; index++)
  {
    if (ProgKey256[index] != ReadKey256[index])
    {
      Error_Handler();
    }
  }
  /* ------------------ End of check for next HDP level ------------------ */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    (void)BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(200U);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 32;
  RCC_OscInitStruct.PLL1.PLLN = 300;
  RCC_OscInitStruct.PLL1.PLLP = 1;
  RCC_OscInitStruct.PLL1.PLLQ = 2;
  RCC_OscInitStruct.PLL1.PLLR = 2;
  RCC_OscInitStruct.PLL1.PLLS = 2;
  RCC_OscInitStruct.PLL1.PLLT = 2;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK4|RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_DISABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/* USER CODE BEGIN 4 */
/**
  * @brief SAES Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAES_CRYP_Init(void)
{

  /* USER CODE BEGIN SAES_Init 0 */

  /* USER CODE END SAES_Init 0 */

  /* USER CODE BEGIN SAES_Init 1 */

  /* USER CODE END SAES_Init 1 */
  hcryp.Instance = SAES;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
  hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
  hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
  hcryp.Init.KeyMode = CRYP_KEYMODE_NORMAL;
  hcryp.Init.KeySelect = CRYP_KEYSEL_AHK;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAES_Init 2 */

  /* USER CODE END SAES_Init 2 */

}
/* USER CODE END 4 */

 /* MPU Configuration */

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  printf("Error!!\r\n");
  return;
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
    (void)BSP_LED_Toggle(LED_RED);
    HAL_Delay(200U);
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
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
