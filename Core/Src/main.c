/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Fixed ADC DMA + Timer Trigger)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
// MPU6500 SETTING
#define MPU6050_ADDR          (0x68 << 1) // Địa chỉ I2C 8-bit (0xD0) nếu AD0 nối GND
#define REG_SMPLRT_DIV        0x19        // Sample Rate Divider
#define REG_CONFIG            0x1A        // Cấu hình DLPF & FSYNC
#define REG_GYRO_CONFIG       0x1B        // Dải đo Gyro
#define REG_ACCEL_CONFIG      0x1C        // Dải đo Accel
#define REG_PWR_MGMT_1        0x6B        // Quản lý nguồn & Clock Source
// gyro và gia tốc accel
uint8_t imu_rx_buffer[14];
int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;
  int16_t Gyro_X_RAW = 0,  Gyro_Y_RAW = 0,  Gyro_Z_RAW = 0;
  float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float gyro_x_offset = 0.0f;
    float gyro_y_offset = 0.0f;
    float gyro_z_offset = 0.0f;
    float pitch_angle = 0.0f; // Góc ngóc đầu/chúi mũi
    float roll_angle = 0.0f;  // Góc lật nghiêng xe
    float yaw_angle = 0.0f;
// Thêm từ khóa volatile để báo cho CPU biết mảng này bị thay đổi ngầm bởi DMA
volatile uint16_t adc_value[5];
// Biến Gửi mẫu USB Device
char msg[200];
//
// proces Data CNT encoder
uint16_t prev_cnt_left = 0;
uint16_t prev_cnt_right = 0;
// Vận Tốc Bánh
float speed_left_ms = 0.0f;
float speed_right_ms = 0.0f;
int32_t speed_left_int_uMs = 0;
 int32_t speed_right_int_uMs = 0;
//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
// Hàm gửi dữ liệu qua CDC
void CDC_Print(const char *str) {
    uint8_t len = (uint8_t)strlen(str);
    while (CDC_Transmit_FS((uint8_t*)str, len) == USBD_BUSY) {
        HAL_Delay(1);
    }
}
// MPU6500
// Tìm Địa Chỉ

void I2C_ScanAndPrint_CDC(void) {
    HAL_StatusTypeDef result;
    uint8_t count = 0;

    CDC_Print("\r\n===============================\r\n");
    CDC_Print("   Bat dau quet bus I2C...\r\n");
    CDC_Print("===============================\r\n");

    // Dải địa chỉ I2C 7-bit tiêu chuẩn từ 0x01 đến 0x77 (1 đến 119)
    for (uint8_t addr_7bit = 1; addr_7bit < 128; addr_7bit++) {
        // Địa chỉ đưa vào HAL cần dịch trái 1 bit (addr_7bit << 1)
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr_7bit << 1), 2, 5);

        if (result == HAL_OK) {
            // Định dạng chuỗi in ra địa chỉ Hex
            snprintf(msg, sizeof(msg),
                     "[+] Tim thay thiet bi tai dia chi: 0x%02X\r\n", addr_7bit);
            CDC_Print(msg);

            // Kiểm tra xem có đúng là MPU-6500 hay không
            if (addr_7bit == 0x68 || addr_7bit == 0x69) {
                snprintf(msg, sizeof(msg),
                         "    -> Co the la MPU-6050 (AD0=%s)\r\n",
                         (addr_7bit == 0x68) ? "GND" : "VCC");
                CDC_Print(msg);
            }

            count++;
        }
    }

    if (count == 0) {
        CDC_Print("[-] Khong tim thay thiet bi nao tren bus I2C!\r\n");
    } else {
        snprintf(msg, sizeof(msg),
                 "[*] Hoan thanh! Tong so thiet bi tim thay: %d\r\n", count);
        CDC_Print(msg);
    }
    CDC_Print("===============================\r\n\r\n");
}
// khởi tạo MPU6500
HAL_StatusTypeDef MPU6500_Init(void) {
    uint8_t data[2];
    HAL_StatusTypeDef status;

    // 1. Thoát Sleep Mode, dùng PLL trục Gyro X làm clock reference
    data[0] = 0x6B; // REG_PWR_MGMT_1
    data[1] = 0x01;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // 2. Cấu hình DLPF = 3 (Băng thông 41Hz)
    // Giảm delay xuống ~5.9ms, đủ nhạy để PID bám tường mà vẫn lọc được nhiễu rung cơ khí
    data[0] = 0x1A; // REG_CONFIG
    data[1] = 0x03;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 3. Tốc độ lấy mẫu (Sample Rate) = 1kHz / (1 + 4) = 200 Hz
    // Tốc độ 200Hz là dư sức cho vòng lặp PID 100Hz (10ms) của xe
    data[0] = 0x19; // REG_SMPLRT_DIV
    data[1] = 0x04;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 4. Accel Full Scale: ±2g -> Độ nhạy 16384 LSB/g (Xe chạy mặt phẳng không cần lớn)
    data[0] = 0x1C; // REG_ACCEL_CONFIG
    data[1] = 0x00;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 5. Gyro Full Scale: ±2000 deg/s -> Độ nhạy 16.4 LSB/(deg/s)
    // BẮT BUỘC dùng thang đo lớn nhất vì Micromouse xoay rất gắt
    data[0] = 0x1B; // REG_GYRO_CONFIG
    data[1] = 0x18;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    return HAL_OK;
}
void MPU6500_Read_All(void) {
    uint8_t buffer[14];
    HAL_StatusTypeDef status;

    // Đọc liên tiếp 14 byte từ địa chỉ 0x3B (ACCEL_XOUT_H)
    status = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, I2C_MEMADD_SIZE_8BIT, buffer, 14, 100);

    if (status == HAL_OK) {
        Accel_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
        Accel_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
        Accel_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
        // buffer[6] và buffer[7] là nhiệt độ (TEMP_OUT)
        Gyro_X_RAW  = (int16_t)((buffer[8]  << 8) | buffer[9]);
        Gyro_Y_RAW  = (int16_t)((buffer[10] << 8) | buffer[11]);
        Gyro_Z_RAW  = (int16_t)((buffer[12] << 8) | buffer[13]);
    } else {
        CDC_Print("Loi doc I2C tu MPU6050!\r\n");
    }
}
void MPU6500_Calibrate(void) {
    uint8_t raw_data[6];
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int samples = 500;

    for (int i = 0; i < samples; i++) {
        // Đọc trực tiếp 6 byte thanh ghi Gyro (0x43 đến 0x48)
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, raw_data, 6, 100);

        sum_gx += (int16_t)((raw_data[0] << 8) | raw_data[1]);
        sum_gy += (int16_t)((raw_data[2] << 8) | raw_data[3]);
        sum_gz += (int16_t)((raw_data[4] << 8) | raw_data[5]);

        HAL_Delay(2); // Dãn cách mỗi lần đọc
    }

    // Chia trung bình để lấy sai số tĩnh
    gyro_x_offset = (float)sum_gx / samples;
    gyro_y_offset = (float)sum_gy / samples;
    gyro_z_offset = (float)sum_gz / samples;
}
void MPU6500_Process_Data(float dt) {
    // 1. Lắp ghép 14 byte từ mảng DMA thành số nguyên 16-bit
    int16_t raw_ax = (int16_t)((imu_rx_buffer[0] << 8) | imu_rx_buffer[1]);
    int16_t raw_ay = (int16_t)((imu_rx_buffer[2] << 8) | imu_rx_buffer[3]);
    int16_t raw_az = (int16_t)((imu_rx_buffer[4] << 8) | imu_rx_buffer[5]);
    // byte [6][7] là nhiệt độ, bỏ qua.
    int16_t raw_gx = (int16_t)((imu_rx_buffer[8] << 8) | imu_rx_buffer[9]);
    int16_t raw_gy = (int16_t)((imu_rx_buffer[10] << 8) | imu_rx_buffer[11]);
    int16_t raw_gz = (int16_t)((imu_rx_buffer[12] << 8) | imu_rx_buffer[13]);

    // 2. Trừ đi nhiễu tĩnh và chia hệ số nhạy (16.4 cho thang đo ±2000 dps)
    float rate_gx = (raw_gx - gyro_x_offset) / 16.4f;
    float rate_gy = (raw_gy - gyro_y_offset) / 16.4f;
    float rate_gz = (raw_gz - gyro_z_offset) / 16.4f;

    // 3. Tính góc tuyệt đối từ Gia tốc kế
    // Lưu ý: Tùy chiều gắn chip trên xe của bạn mà trục X/Y có thể đảo cho nhau
    float pitch_acc = atan2f(-raw_ax, sqrtf(raw_ay * raw_ay + raw_az * raw_az)) * 180.0f / 3.141592f;
    float roll_acc  = atan2f(raw_ay, raw_az) * 180.0f / 3.141592f;

    // 4. BỘ LỌC BÙ (COMPLEMENTARY FILTER)
    // Hệ số alpha (0.96) có nghĩa là: 96% tin tưởng vào độ nhạy của Gyro, 4% dùng Accel để neo giữ không bị trôi
    float alpha = 0.96f;
    pitch_angle = alpha * (pitch_angle + rate_gy * dt) + (1.0f - alpha) * pitch_acc;
    roll_angle  = alpha * (roll_angle  + rate_gx * dt) + (1.0f - alpha) * roll_acc;

    // 5. Tính góc Yaw (Heading) với Deadband
    // Góc Yaw không có giá trị từ gia tốc kế để bù, nên chỉ dùng tích phân Gyro.
    // Lọc bỏ các xung nhiễu nhỏ hơn 0.8 độ/giây để xe đứng yên không bị trôi góc
    if (fabs(rate_gz) > 0.8f) {
        yaw_angle += rate_gz * dt;
    }
}
void My_MPU6500_Callback(I2C_HandleTypeDef *hi2c){
	MPU6500_Process_Data(0.005);
	 print_mpu6500();
}
void print_mpu6500(){

		 	          // 2. In giá trị 3 trục Ox, Oy, Oz của Gia tốc kế (g) và Gyroscope (deg/s)
		 	  snprintf(msg, sizeof(msg), "DATA,%f,%f,%f\r\n",
		 			 pitch_angle, // Góc ngóc đầu/chúi mũi
		 			      roll_angle,  // Góc lật nghiêng xe
		 			     yaw_angle);

		 	      CDC_Print(msg);
		 	          // Chu kỳ cập nhật (ví dụ 100ms một lần)
		 	          HAL_Delay(20);

}
//Hàm test LED
void print_LED(){
	      // Sử dụng hàm CDC_Print của bạn để tránh mất gói tin khi USB Busy
	      sprintf(msg, "IRFL:%u | IRL:%u | IRR:%u | IRFR:%u\r\n",
	              adc_value[0], adc_value[1], adc_value[2], adc_value[3]);

	      CDC_Print(msg);
		  HAL_Delay(20);
}
//Test ENCODER and interup Veloc
void print_Velocity(){


		  uint16_t len = sprintf(msg, "Speed L: %ld | R: %ld\r\n", speed_left_int_uMs, speed_right_int_uMs);
		  CDC_Transmit_FS((uint8_t*)msg, len);
		  HAL_Delay(50);
		   len = sprintf(msg, "Speed L: %ld | R: %ld\r\n", TIM3->CNT, TIM2->CNT);
			  CDC_Transmit_FS((uint8_t*)msg, len);
	      // Delay một chút để tránh làm treo phần mềm Serial Monitor trên máy tính
	      HAL_Delay(10);
}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // 1. Kích hoạt ADC ở chế độ DMA trước để nó nằm vùng chờ tín hiệu
  // Kích Hoạt Tim4 nguồn trig cho DMA ADC 5khz
  // TIM 4 CH4 no output generator PWWM
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  // Kích Hoạt TIM9 IT ngắt Tính Vận tốc tức thời 500Hz
  HAL_TIM_Base_Start_IT(&htim9);
  //
  // Kích Hoạt DMA cho ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, 4);
  // 2. Kích hoạt Timer 1 (Base đếm) 20kz cho PWM TIM1 CH2N
  // 3. Kích hoạt kênh PWM 1 CHO bên trái
  HAL_TIM_Base_Start(&htim1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  // 2. Kích hoạt Timer 5 (Base đếm) 20kz cho PWM TIM5 CH4N
   // 3. Kích hoạt kênh PWM 5
  //CHO bên phải
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  // Kích Hoạt TIMER cho ENCODER
  //TIM 2 CHO bên phải
  //TIM3 CHO bên trái
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // INIT CÁC GPIO
  // Bật tất cả LED phát (Chỉ để test, nếu làm mạch thật cần băm xung theo Timer)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
  // Chờ cho led bật hoàn toàn.
  HAL_Delay(300);
  //
  //Tắt Động Cơ
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
  TIM1->CCR3=2900;
  TIM5->CCR3=2999;
  //
// KHỞI ĐÔNG MPU LỖI NẾU LED KO SÁNG
  if (MPU6500_Init() == HAL_OK) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
    	      } else {
    	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
    	      }
  MPU6500_Calibrate();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 300000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 12000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 60000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    // Cú pháp đúng: TIM9 (không có &htim)
    if (htim->Instance == TIM9)
    {
        uint16_t curr_cnt_left = TIM3->CNT;
        uint16_t curr_cnt_right = TIM2->CNT;

        int16_t delta_pulse_left = (int16_t)(curr_cnt_left - prev_cnt_left);
        int16_t delta_pulse_right = (int16_t)(curr_cnt_right - prev_cnt_right);

        prev_cnt_left = curr_cnt_left;
        prev_cnt_right = curr_cnt_right;

        // 2. TÍNH TỐC ĐỘ (m/s)
        speed_left_ms = ((float)delta_pulse_left / 700) * 141.3716694f;
        speed_right_ms = ((float)delta_pulse_right / 700) * 141.3716694f;

        speed_left_int_uMs = (int32_t)(speed_left_ms * 1000000.0f);
        speed_right_int_uMs = (int32_t)(speed_right_ms * 1000000.0f);

        // CHỈ ra lệnh khởi động DMA, không xử lý MPU ở đây
        HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, imu_rx_buffer, 14);
    }
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        // 1. Tính toán MPU6500 (Góc Pitch, Roll, Yaw)
        MPU6500_Process_Data(0.005f);

        // 2. Chạy thuật toán PID ngay tại đây
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
