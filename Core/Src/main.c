/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define P10_WIDTH 32
#define P10_HEIGHT 16
#define P10_SCAN_RATE 8 // Umumnya 1/8 scan untuk panel 16 baris
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

PCD_HandleTypeDef hpcd_USB_FS;

/* Definitions for p10Task */
osThreadId_t p10TaskHandle;
const osThreadAttr_t p10Task_attributes = {
    .name = "p10Task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
    .name = "sensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for tempQueue */
osMessageQueueId_t tempQueueHandle;
const osMessageQueueAttr_t tempQueue_attributes = {
    .name = "tempQueue"};
/* Definitions for framebufferMutex */
osMutexId_t framebufferMutexHandle;
const osMutexAttr_t framebufferMutex_attributes = {
    .name = "framebufferMutex"};
/* USER CODE BEGIN PV */
// Framebuffer untuk menyimpan data piksel yang akan ditampilkan
uint8_t framebuffer[P10_HEIGHT][P10_WIDTH / 8];
// Variabel untuk melacak baris yang sedang di-scan
volatile uint8_t scan_row = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_RTC_Init(void);
void Startp10Task(void *argument);
void StartsensorTask(void *argument);

/* USER CODE BEGIN PFP */
// Fungsi helper untuk kontrol P10
void p10_latch();
void p10_setOutputEnable(uint8_t enable);
void p10_selectRow(uint8_t row);
// Fungsi untuk menggambar di framebuffer
void drawPixel(int x, int y, uint8_t color);
void drawString(int x, int y, const char *text, const uint8_t *font, uint8_t font_width, uint8_t font_height, uint8_t color);
void clearFramebuffer();
// Fungsi untuk membaca sensor MAX31855
float read_max31855(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USB_PCD_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //  MX_USB_DEVICE_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of framebufferMutex */
  framebufferMutexHandle = osMutexNew(&framebufferMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of tempQueue */
  tempQueueHandle = osMessageQueueNew(4, sizeof(float), &tempQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of p10Task */
  p10TaskHandle = osThreadNew(Startp10Task, NULL, &p10Task_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartsensorTask, NULL, &sensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
   */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, P10_A_Pin | P10_B_Pin | P10_OE_Pin | P10_LAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_LED_Pin | MAX_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : P10_A_Pin P10_B_Pin P10_OE_Pin P10_LAT_Pin */
  GPIO_InitStruct.Pin = P10_A_Pin | P10_B_Pin | P10_OE_Pin | P10_LAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_Pin MAX_CS_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin | MAX_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Fungsi callback yang dipanggil oleh interrupt TIM2.
 * Fungsi ini menangani refresh rate panel P10.
 * @note  Jaga fungsi ini secepat mungkin. Hindari operasi yang lama.
 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//   if (htim->Instance == TIM2)
//   {
//     p10_setOutputEnable(0); // Matikan display untuk mencegah ghosting saat ganti baris
//
//     // Ambil mutex (sebentar) untuk membaca framebuffer dengan aman
//     if (osMutexAcquire(framebufferMutexHandle, 0) == osOK)
//     {
//       // Kirim data piksel untuk baris saat ini dan baris pasangannya (scan rate 1/8)
//       HAL_SPI_Transmit(&hspi1, framebuffer[scan_row], P10_WIDTH / 8, 5);
//       HAL_SPI_Transmit(&hspi1, framebuffer[scan_row + P10_SCAN_RATE], P10_WIDTH / 8, 5);
//       osMutexRelease(framebufferMutexHandle);
//     }
//
//     p10_latch();             // Latch data ke output driver panel
//     p10_selectRow(scan_row); // Pilih baris untuk siklus berikutnya
//     p10_setOutputEnable(1);  // Nyalakan display lagi
//
//     scan_row++;
//     if (scan_row >= P10_SCAN_RATE)
//     {
//       scan_row = 0;
//     }
//   }
// }

void p10_latch()
{
  HAL_GPIO_WritePin(P10_LAT_GPIO_Port, P10_LAT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(P10_LAT_GPIO_Port, P10_LAT_Pin, GPIO_PIN_RESET);
}

void p10_setOutputEnable(uint8_t enable)
{
  HAL_GPIO_WritePin(P10_OE_GPIO_Port, P10_OE_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET); // Aktif LOW
}

void p10_selectRow(uint8_t row)
{
  HAL_GPIO_WritePin(P10_A_GPIO_Port, P10_A_Pin, (row & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(P10_B_GPIO_Port, P10_B_Pin, (row & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  // Tambahkan untuk C dan D jika panel Anda 1/16 scan
}

void clearFramebuffer()
{
  memset(framebuffer, 0, sizeof(framebuffer));
}

void drawPixel(int x, int y, uint8_t color)
{
  if (x < 0 || x >= P10_WIDTH || y < 0 || y >= P10_HEIGHT)
    return;

  if (color)
  {
    framebuffer[y][x / 8] |= (1 << (7 - (x % 8)));
  }
  else
  {
    framebuffer[y][x / 8] &= ~(1 << (7 - (x % 8)));
  }
}

/**
 * @brief Menggambar string ke framebuffer.
 * @note  Fungsi ini SANGAT DISEDERHANAKAN. Ia menggambar karakter "built-in"
 * Untuk font kustom, Anda perlu mengimplementasikan logika pembacaan array font.
 */
void drawString(int x, int y, const char *text, const uint8_t *font, uint8_t font_width, uint8_t font_height, uint8_t color)
{
  if (!font || !text)
    return; // Pastikan font dan teks valid

  int current_x = x;
  char ch;

  while ((ch = *text++))
  {
    // Asumsikan font dimulai dari karakter spasi (ASCII 32)
    // dan mencakup hingga '~' (ASCII 126)
    if (ch < ' ' || ch > '~')
    {
      // Karakter di luar jangkauan, bisa diganti spasi atau karakter default
      ch = ' '; // Ganti dengan spasi
    }

    // Hitung offset ke data karakter dalam array font
    // Setiap karakter memiliki 'font_width' byte data (1 byte per kolom)
    const uint8_t *char_font_data = font + (ch - ' ') * font_width;

    for (uint8_t col = 0; col < font_width; col++)
    {
      uint8_t column_pixels = char_font_data[col]; // Ambil data piksel untuk kolom saat ini
      for (uint8_t row = 0; row < font_height; row++)
      {
        // Cek apakah bit untuk baris saat ini menyala (bit 0 = baris atas)
        if ((column_pixels >> row) & 0x01)
        {
          drawPixel(current_x + col, y + row, color);
        }
      }
    }
    current_x += font_width + 1; // Maju ke posisi karakter berikutnya (lebar font + 1 piksel spasi)
  }
}

/**
 * @brief Membaca data suhu dari sensor MAX31855 via SPI2.
 * @retval Nilai suhu dalam Celcius, atau nilai negatif jika ada error.
 */
float read_max31855(void)
{
  uint8_t rx_data[4];
  int16_t temp_data;
  float temperature = -999.0; // Nilai error default

  // Aktifkan sensor dengan menarik CS ke LOW
  HAL_GPIO_WritePin(MAX_CS_GPIO_Port, MAX_CS_Pin, GPIO_PIN_RESET);
  osDelay(1); // Delay singkat

  // Terima 4 byte data dari sensor
  if (HAL_SPI_Receive(&hspi2, rx_data, 4, 100) == HAL_OK)
  {
    // Tarik CS ke HIGH untuk menonaktifkan sensor
    HAL_GPIO_WritePin(MAX_CS_GPIO_Port, MAX_CS_Pin, GPIO_PIN_SET);

    // Cek bit 16 untuk fault (Open Circuit, Short to GND, Short to VCC)
    if (rx_data[1] & 0x01)
    {
      // Ada fault pada thermocouple
      return -100.0; // Kode error untuk open circuit
    }

    // Ambil data suhu (14 bit, MSB ada di byte 0)
    temp_data = (rx_data[0] << 6) | (rx_data[1] >> 2);

    // Konversi ke Celcius (resolusi 0.25 C per LSB)
    temperature = temp_data * 0.25;
  }
  else
  {
    // Gagal komunikasi SPI, tarik CS ke HIGH
    HAL_GPIO_WritePin(MAX_CS_GPIO_Port, MAX_CS_Pin, GPIO_PIN_SET);
  }

  return temperature;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Startp10Task */
/**
 * @brief  Function implementing the p10Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Startp10Task */
void Startp10Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  float temperature = 0.0;
  char display_buffer[20];

  // Pesan awal
  osMutexAcquire(framebufferMutexHandle, osWaitForever);
  clearFramebuffer();
  drawString(1, 1, "Membaca", NULL, 0, 0, 1);
  drawString(1, 9, "Sensor..", NULL, 0, 0, 1);
  osMutexRelease(framebufferMutexHandle);
  /* Infinite loop */
  for (;;)
  {
    // Tunggu data suhu baru dari queue
    osStatus_t status = osMessageQueueGet(tempQueueHandle, &temperature, NULL, osWaitForever);

    if (status == osOK)
    {
      // Format string suhu
      sprintf(display_buffer, "%d'C", (int)temperature);

      // Ambil mutex sebelum mengakses framebuffer
      osMutexAcquire(framebufferMutexHandle, osWaitForever);

      // Bersihkan dan gambar ulang framebuffer
      clearFramebuffer();
      drawString(1, 5, display_buffer, NULL, 0, 0, 1); // Ganti NULL dengan data font jika sudah ada

      // Lepaskan mutex
      osMutexRelease(framebufferMutexHandle);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartsensorTask */
/**
 * @brief Function implementing the sensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartsensorTask */
void StartsensorTask(void *argument)
{
  /* USER CODE BEGIN StartsensorTask */
  /* Infinite loop */
  for (;;)
  {
    float current_temp = read_max31855();

    // Kirim suhu ke queue
    osMessageQueuePut(tempQueueHandle, &current_temp, 0U, 0U);

    // Tunggu 1 detik sebelum pembacaan berikutnya
    osDelay(1000);
  }
  /* USER CODE END StartsensorTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2)
  {
    p10_setOutputEnable(0); // Matikan display untuk mencegah ghosting saat ganti baris

    // Ambil mutex (sebentar) untuk membaca framebuffer dengan aman
    if (osMutexAcquire(framebufferMutexHandle, 0) == osOK)
    {
      // Kirim data piksel untuk baris saat ini dan baris pasangannya (scan rate 1/8)
      HAL_SPI_Transmit(&hspi1, framebuffer[scan_row], P10_WIDTH / 8, 5);
      HAL_SPI_Transmit(&hspi1, framebuffer[scan_row + P10_SCAN_RATE], P10_WIDTH / 8, 5);
      osMutexRelease(framebufferMutexHandle);
    }

    p10_latch();             // Latch data ke output driver panel
    p10_selectRow(scan_row); // Pilih baris untuk siklus berikutnya
    p10_setOutputEnable(1);  // Nyalakan display lagi

    scan_row++;
    if (scan_row >= P10_SCAN_RATE)
    {
      scan_row = 0;
    }
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
