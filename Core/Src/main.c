/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#ifdef DOWNLINK_FLAG
static uint8_t downlink_request = 1;
#else
static uint8_t downlink_request = 0;
#endif
void SystemClock_Config(void);
void ST_Init(void);


extern uint8_t user_button;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();

	mcuConfig();

	/********** OPEN AND CONFIFIGURES SIGFOX LIBRARY IN RCZ2 *********************/
	/********** IN ORDER TO OPEN OTHER RCZS, SEE SIGFOX_API.h **/
	/********** BASICALLY CHANGES TO OTHER RC VALUE LIKE RCZ3 **/
	HT_API_ConfigRegion(RCZ2);
	while (1)
	{

	}
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void HT_API_setSmpsVoltageAction(sfx_u8 mode) {
	ST_RF_API_smps(mode);
	printf("Set_smps_voltage %d\n", mode);
}

void HT_API_switchPa(uint8_t state) {

	ST_RF_API_set_pa(state);

	printf("Switch PA: %d\n", state);
}

void HT_API_ConfigRegion(rc_mask RCZ) {
	ST_SFX_ERR open_err = ST_SFX_ERR_NONE;

	switch(RCZ){
	case RCZ1:
		ST_RF_API_reduce_output_power(RCZ1_OUTPUT_POWER);
		open_err = St_Sigfox_Open_RCZ(RCZ1);
		HT_API_switchPa(0);
		HT_API_setSmpsVoltageAction(7);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ2:
		ST_RF_API_reduce_output_power(RCZ2_OUTPUT_POWER);
		open_err = St_Sigfox_Open_RCZ(RCZ2);
		HT_API_switchPa(1);
		HT_API_setSmpsVoltageAction(2);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ3:
		open_err = St_Sigfox_Open_RCZ(RCZ3);
		ST_RF_API_reduce_output_power(RCZ3_OUTPUT_POWER);
		HT_API_switchPa(0);
		HT_API_setSmpsVoltageAction(7);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ4:
		open_err = St_Sigfox_Open_RCZ(RCZ4);
		ST_RF_API_reduce_output_power(RCZ4_OUTPUT_POWER);
		HT_API_switchPa(1);
		HT_API_setSmpsVoltageAction(2);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ5:
		open_err = St_Sigfox_Open_RCZ(RCZ5);
		ST_RF_API_reduce_output_power(RCZ5_OUTPUT_POWER);
		HT_API_switchPa(0);
		HT_API_setSmpsVoltageAction(7);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ6:
		open_err = St_Sigfox_Open_RCZ(RCZ6);
		ST_RF_API_reduce_output_power(RCZ6_OUTPUT_POWER);
		HT_API_switchPa(0);
		HT_API_setSmpsVoltageAction(7);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ7:
		open_err = St_Sigfox_Open_RCZ(RCZ7);
		ST_RF_API_reduce_output_power(RCZ7_OUTPUT_POWER);
		HT_API_switchPa(0);
		HT_API_setSmpsVoltageAction(7);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	default:
		break;
	}

}

sfx_error_t HT_API_SendFrame(void) {

	/********** SEND MESSAGE TO RCZ2 ****************************/

	uint8_t customer_data[12]={0xAA, 0xAA, 0xAA, 0xAA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA};
	uint8_t customer_resp[8];
	sfx_error_t err;

#ifdef DOWNLINK_FLAG
	HAL_TIM_Base_Start_IT(&htim21);
#endif

	/********** FUNCTION PARAMETERS  ****************************/
	/********** THE LAST ONE IS TO REQUEST DOWNLINK ************/
	/********** 1 - YES, 0 - NO	 ******************************/

	err=SIGFOX_API_send_frame(customer_data,sizeof(customer_data),customer_resp, 3, downlink_request);

	if(downlink_request && !err) {
		printf("Customer resp: {");

		for(uint16_t i = 0; i < 7; i++) 
			printf("0x%x,", customer_resp[i]);

		printf("0x%x}\n\r", customer_resp[7]);
	}

	printf("\nError Send Frame: %X\n", err);

#ifdef DOWNLINK_FLAG
	HAL_TIM_Base_Stop_IT(&htim21);
#endif

	return err;
}

void mcuConfig(void) {
	ST_SFX_ERR stSfxRetErr;

	ST_Init();

	NVM_BoardDataType sfxConfiguration;
	stSfxRetErr = ST_Sigfox_Init(&sfxConfiguration, 0);

	if(stSfxRetErr != ST_SFX_ERR_NONE) {
		if(stSfxRetErr == ST_SFX_ERR_CREDENTIALS) {
			sfxConfiguration.id = 0;
			memset(sfxConfiguration.pac, 0x00, 8);
			sfxConfiguration.rcz = 0;

		}
	}

	/* Calibrate RTC in case of STM32*/
	ST_MCU_API_TimerCalibration(500);

	printf("Sigfox iMCP HT32SX\n");

	printf("ID: %.8X - PAC: ", (unsigned int)sfxConfiguration.id);

	for(uint16_t i = 0; i < sizeof(sfxConfiguration.pac); i++)
	{
		printf("%.2X", sfxConfiguration.pac[i]);
	}

	printf("\n");

	/*			SET S2LP XTAL FREQUENCY														*/
	/*			DO NOT CHANGE IT																	*/

	ST_RF_API_set_xtal_freq(50000000); 

	/*			SET A PROPER FREQUENCY OFFSET											*/
	/*			THIS VALUE CAN BE FOUND IN CREDENTIALS 						*/

	ST_RF_API_set_freq_offset(sfxConfiguration.freqOffset);
	printf("Freq Offset %ld \n", (int32_t)sfxConfiguration.freqOffset);

	/*			SET LBT OFFSET																		*/
	/*			THIS VALUE CAN BE FOUND IN CREDENTIALS 						*/

	ST_RF_API_set_lbt_thr_offset(sfxConfiguration.lbtOffset);
	printf("LBT %ld \n", (int32_t)sfxConfiguration.lbtOffset);

	/*			SET RSSI OFFSET																		*/
	/*			THIS VALUE CAN BE FOUND IN CREDENTIALS 						*/

	ST_RF_API_set_rssi_offset(sfxConfiguration.rssiOffset);
	printf("RSSI %ld \n", sfxConfiguration.rssiOffset);
}

void ST_Init(void) {
	/* Put the radio off */
	S2LPShutdownInit();
	HAL_Delay(10);
	S2LPShutdownExit();

	/* FEM Initialization */
	FEM_Init();
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
