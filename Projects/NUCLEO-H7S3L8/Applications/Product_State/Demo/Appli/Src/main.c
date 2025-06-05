/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32H7RSxx HAL API.
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
#define PRODUCT_STATE_PROVISIONING 0x17
#define PRODUCT_STATE_CLOSED 0x72
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void MPU_AdjustRegionAddressSize(uint32_t Address, uint32_t Size, MPU_Region_InitTypeDef* pInit);
static void MPU_Config(void);

static void open_debug(void);
static void read_SBS_reg(void);
static void GetProductState(void);
static void read_obk(void);
static void regression(void);
static void SetProductState(uint32_t state);
static void GetRSSStatus(void);

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

static void GetRSSStatus(void)
{
  uint32_t result = RSSLIB_PFUNC->GetRssStatus();
  switch ( result )
  {
    case 0xEAEAEAEA:
      printf("...Success\r\n");
      break;
    case 0xF5F5F5F5:
      printf("...Error\r\n");
      break;
    case 0xF5F5E0E0:
      printf("...Bad input address range\r\n");
      break;
    case 0xF5F50E0E:
      printf("...Bad size\r\n");
      break;
    case 0xF5F5E00E:
      printf("...Bad encryption value in DoEncryption\r\n");
      break;
    case 0xF5F50880:
      printf("...Hardware cryptography not available (H7R)\r\n");
      break;
    case 0xF5F50EE0:
      printf("...Encryption error\r\n");
      break;    
    case 0xF5F50808:
      printf("...Programming error\r\n");
      break;  
    case 0xF5F50008:
      printf("...Bad value in Destination\r\n");
      break;  
    default:
      printf("...Unkown status\r\n");
      break;
  }  
}

static void SetProductState(uint32_t state)
{
  if ( state == PRODUCT_STATE_PROVISIONING )
  {
    printf("Set product state to PROVISIONING\r\n");  
  }
  else if(state == PRODUCT_STATE_CLOSED )
  {
    printf("Set product state to CLOSED\r\n");
  }
  else
  {
    printf("Invalid product state, can only be 0x17(provisioning) or 0x72 (closed)!\r\n");
    return;
  }
  
  if( RSSLIB_PFUNC->SetProductState(state) == 0xF5F5F5F5)
  {
    printf("Call RSSLIB_PFUNC->SetProductState() failed!\r\n");
  }  
}

static void GetProductState(void)
{
#if 1
  /*
  * 0x39 : Open
  * 0x17 : Provisioning
  * 0x72 : Closed
  * 0x5C : Locked
  */
  uint32_t ps = RSSLIB_PFUNC->GetProductState();
  switch (ps)
  {
    case 0x39: printf("\t : << OPEN >> \r\n"); break;
    case 0x17: printf("\t : << PROVISIONING >> \r\n"); break;
    case 0x72: printf("\t : << CLOSED >> \r\n"); break;
    case 0x5C: printf("\t : << LOCKED >> \r\n"); break;
    default: printf("\t UNKNOWN bad value!!\r\n"); break;
  }
  
//#else
  FLASH_OBProgramInitTypeDef flash_option_bytes = {0};
  uint8_t nvstate = 0;  
  uint32_t rotselect = 0;
  uint32_t provd = 0;
  uint32_t dbgauth = 0;
  
  flash_option_bytes.OptionType = OPTIONBYTE_NV;
  HAL_FLASHEx_OBGetConfig(&flash_option_bytes);
  
  nvstate = flash_option_bytes.NVState & FLASH_NVSRP_NVSTATE;
  printf("NVSTATE is %02x ",nvstate);
  
  switch (nvstate)
  {
    case OB_NVSTATE_OPEN: printf("\t : << OPEN >> \r\n"); break;
    case OB_NVSTATE_CLOSE: printf("\t : << CLOSED >> \r\n"); break;
    default: printf("\t UNKNOWN bad value!!\r\n"); break;
  }
  
  printf("ROT Config is %08x \r\n",flash_option_bytes.ROTConfig);
  rotselect = flash_option_bytes.ROTConfig & FLASH_ROTSR_IROT_SELECT;
  dbgauth = flash_option_bytes.ROTConfig & FLASH_ROTSR_DBG_AUTH;
  provd = flash_option_bytes.ROTConfig & FLASH_ROTSRP_OEM_PROVD;
  
  printf("ROT select is %08x ",rotselect);
  switch (rotselect)
  {
    case OB_IROT_SELECTION_ST: printf("\t : << ST >> \r\n"); break;
    case OB_IROT_SELECTION_OEM: printf("\t : << OEM >> \r\n"); break;
    default: printf("\t UNKNOWN bad value!!\r\n"); break;
  }
  printf("DBG AUTH is %08x ",dbgauth);
  switch (dbgauth)
  {
    case OB_DBG_AUTH_LOCKED: printf("\t : << LOCKED >> \r\n"); break;
    case OB_DBG_AUTH_DEFAULT: printf("\t : << NOT SET >> \r\n"); break;
    case OB_DBG_AUTH_ECDSA_SIGN: printf("\t : << ECDSA SIGN >> \r\n"); break;
    case OB_DBG_AUTH_PASSWORD: printf("\t : << DEFAULT >> \r\n"); break;
    default: printf("\t UNKNOWN bad value!!\r\n"); break;
  }
  printf("PROVISION state is %08x ",provd);
  switch (provd)
  {
    case OB_OEM_PROVD_ENABLE: printf("\t : << PROVISIONED >> \r\n"); break;
    case OB_OEM_PROVD_DEFAULT: printf("\t : << NOT PROVISIONED >> \r\n"); break;
    default: printf("\t UNKNOWN bad value!!\r\n"); break;
  }
#endif
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  MPU_Config();
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Update SystemCoreClock variable according to RCC registers values. */
  SystemCoreClockUpdate();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  COM_InitTypeDef COM_Init;
  COM_Init.BaudRate = 115200;
  COM_Init.HwFlowCtl = COM_HWCONTROL_NONE;
  COM_Init.Parity = COM_PARITY_NONE;
  COM_Init.StopBits = COM_STOPBITS_1;
  COM_Init.WordLength = COM_WORDLENGTH_8B;  
  BSP_COM_Init(COM1, &COM_Init);
  printf("\r\nCOM Init done.\r\n");
  
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
      printf("Prov default password obk  ---------- p\n");
      printf("Get RSS status             ---------- a\n");
      printf("Regression                 ---------- r\n");
      printf("Get current product state  ---------- 0\n");      
      printf("Set PS to Provisioning     ---------- 1\n");      
      printf("Set PS to Closed           ---------- 2\n");
      printf("Get current HDP level      ---------- 3\n");
      printf("Set HDPL to level 2        ---------- 4\n");
      printf("Set HDPL to level 3        ---------- 5\n");
      printf("Exit                       ---------- x\n");
      while (HAL_UART_Receive(&hcom_uart[COM1], &response, 1, 1000) == HAL_TIMEOUT);
      printf("Your input is : %c\r\n", response);
      switch (response)
      {
      case 'a':
        printf("===> Call RSS API to Get RSS status\r\n");
        GetRSSStatus();
        break;
      case '0':
        printf("===> Call RSS API to get current product state \r\n");
        GetProductState();
        break;
      case '1':
        printf("===> Call RSS API to set product state to provisioning\r\n");
        SetProductState(PRODUCT_STATE_PROVISIONING);
        break;
      case '2':
        printf("===> Call RSS API to set product state to closed\r\n");
        SetProductState(PRODUCT_STATE_CLOSED);
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
        break;
//      case 'b':
//        printf("===> Read OBK HDPL1 128bit !\r\n"); 
//        read_obk();
//        break;  
      case 'i':
        //back_to_privisioning();
        break;
      case 'p':
        //provision_default_password_obk();
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
  printf(" Exit from menu, start GPIO toggle \r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LD1_GPIO_PORT, LD1_Pin);
    /* Insert delay 100 ms */
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_PORT, LD2_PIN);
    /* Insert delay 100 ms */
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD3_GPIO_PORT, LD3_PIN);
    /* Insert delay 100 ms */
    HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOM_CLK_ENABLE();
  __HAL_RCC_GPIOO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOM, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOO, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOM, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOO, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function configures the MPU context of the application.
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  uint32_t index = MPU_REGION_NUMBER0;
  uint32_t address;
  uint32_t size;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Initialize the background region */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = index;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  index++;

  /* Initialize the non cacheable region */
#if defined ( __ICCARM__ )
  /* get the region attribute form the icf file */
  extern uint32_t NONCACHEABLEBUFFER_start;
  extern uint32_t NONCACHEABLEBUFFER_size;

  address = (uint32_t)&NONCACHEABLEBUFFER_start;
  size = (uint32_t)&NONCACHEABLEBUFFER_size;

#elif defined (__CC_ARM) || defined(__ARMCC_VERSION)
  extern uint32_t Image$$RW_NONCACHEABLEBUFFER$$Base;
  extern uint32_t Image$$RW_NONCACHEABLEBUFFER$$Length;
  extern uint32_t Image$$RW_NONCACHEABLEBUFFER$$ZI$$Length;

  address = (uint32_t)&Image$$RW_NONCACHEABLEBUFFER$$Base;
  size  = (uint32_t)&Image$$RW_NONCACHEABLEBUFFER$$Length + (uint32_t)&Image$$RW_NONCACHEABLEBUFFER$$ZI$$Length;
#elif defined ( __GNUC__ )
  extern int __NONCACHEABLEBUFFER_BEGIN;
  extern int __NONCACHEABLEBUFFER_END;

  address = (uint32_t)&__NONCACHEABLEBUFFER_BEGIN;
  size  = (uint32_t)&__NONCACHEABLEBUFFER_END - (uint32_t)&__NONCACHEABLEBUFFER_BEGIN;
#else
#error "Compiler toolchain is unsupported"
#endif

  if (size != 0)
  {
    /* Configure the MPU attributes as Normal Non Cacheable */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = index;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_AdjustRegionAddressSize(address, size, &MPU_InitStruct);
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    index++;
  }

  /* Initialize the region corresponding to the execution area
     (external or internal flash or external or internal RAM
     depending on scatter file definition) */
#if defined ( __ICCARM__ )
  extern uint32_t __ICFEDIT_region_ROM_start__;
  extern uint32_t __ICFEDIT_region_ROM_end__;
  address = (uint32_t)&__ICFEDIT_region_ROM_start__;
  size = (uint32_t)&__ICFEDIT_region_ROM_end__ - (uint32_t)&__ICFEDIT_region_ROM_start__ + 1;
#elif defined (__CC_ARM) || defined(__ARMCC_VERSION)
  extern uint32_t Image$$ER_ROM$$Base;
  extern uint32_t Image$$ER_ROM$$Limit;
  address = (uint32_t)&Image$$ER_ROM$$Base;
  size    = (uint32_t)&Image$$ER_ROM$$Limit-(uint32_t)&Image$$ER_ROM$$Base;
#elif defined ( __GNUC__ )
  extern uint32_t __FLASH_BEGIN;
  extern uint32_t __FLASH_SIZE;
  address = (uint32_t)&__FLASH_BEGIN;
  size  = (uint32_t)&__FLASH_SIZE;
#else
#error "Compiler toolchain is unsupported"
#endif

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = index;
  MPU_InitStruct.SubRegionDisable = 0u;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_AdjustRegionAddressSize(address, size, &MPU_InitStruct);
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  index++;

  /* Reset unused MPU regions */
  for(; index < __MPU_REGIONCOUNT ; index++)
  {
    /* All unused regions disabled */
    MPU_InitStruct.Enable = MPU_REGION_DISABLE;
    MPU_InitStruct.Number = index;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
  }

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief This function adjusts the MPU region Address and Size within an MPU configuration.
  * @param Address memory address
  * @param Size memory size
  * @param pInit pointer to an MPU initialization structure
  * @retval None
  */
static void MPU_AdjustRegionAddressSize(uint32_t Address, uint32_t Size, MPU_Region_InitTypeDef* pInit)
{
  /* Compute the MPU region size */
  pInit->Size = ((31 - __CLZ(Size)) - 1);
  if (Size > (1 << (pInit->Size + 1)))
  {
    pInit->Size++;
  }
  uint32_t Modulo = Address % (1 << (pInit->Size - 1));
  if (0 != Modulo)
  {
    /* Align address with MPU region size considering there is no need to increase the size */
    pInit->BaseAddress = Address - Modulo;
  }
  else
  {
    pInit->BaseAddress = Address;
  }
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
  while(1) 
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
  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
