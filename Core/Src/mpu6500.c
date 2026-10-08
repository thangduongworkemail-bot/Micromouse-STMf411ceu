/**
  ******************************************************************************
  * @file    mpu6500.c
  * @brief   IMU MPU6500 qua I2C1 (DMA1 Stream0 cho chiều nhận)
  *
  * Luồng chạy:
  *   main: MPU6500_Setup() = Init + Calibrate (đọc kiểu chờ) -> imu_ready = 1
  *   ngắt TIM9 (2ms): MPU6500_Tick() -> HAL_I2C_Mem_Read_DMA 14 byte từ 0x3B
  *   DMA xong: HAL_I2C_MemRxCpltCallback() -> MPU6500_Process_Data(dt)
  ******************************************************************************
  */
#include "mpu6500.h"
#include "pid.h"            // điều khiển động cơ trong MPU6500_Test
#include <math.h>           // atan2f, sqrtf, fabsf
#include <stdio.h>          // snprintf

extern I2C_HandleTypeDef hi2c1;           // main.c (CubeMX)

// gyro và gia tốc accel
static uint8_t imu_rx_buffer[14];         // DMA ghi vào đây, phải là biến toàn cục/static
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
volatile float yaw_rate_dps = 0.0f;       // Tốc độ quay Z mới nhất (độ/s), dương = quay trái
volatile int16_t imu_raw[7];              // Mẫu thô mới nhất (thứ tự như 14 byte từ 0x3B)
volatile uint32_t imu_sample_count = 0;
// Ngắt TIM9 chỉ bắt đầu đọc IMU bằng DMA sau khi MPU6500_Init + MPU6500_Calibrate xong
// (trước đó hiệu chỉnh dùng lệnh đọc I2C kiểu chờ, đọc DMA chen vào sẽ tranh bus -> offset gyro sai)
volatile uint8_t imu_ready = 0;
// Số chu kỳ TIM9 (2ms) kể từ lần xử lý IMU trước -> dt thật cho tích phân gyro,
// kể cả khi có lần đọc I2C bị lỗi/bỏ qua
static volatile uint16_t imu_ticks = 0;
volatile uint8_t imu_still = 0;           // 1: xe đứng yên -> giữ yaw, offset Gz tự chỉnh
static float still_time = 0.0f;           // Thời gian (s) điều kiện đứng yên đã thỏa liên tục
static float gyro_z_cal = 0.0f;           // Offset Gz lúc hiệu chỉnh khởi động (gyro_z_offset thì tự chỉnh dần)
uint8_t mpu_whoami = 0;                   // Thanh ghi WHO_AM_I: 0x70 MPU6500, 0x71 lõi MPU9250
static volatile uint32_t imu_i2c_errors = 0;   // Lỗi bus I2C khi đọc DMA (HAL_I2C_ErrorCallback)
static volatile uint32_t imu_read_fail  = 0;   // Lệnh đọc DMA bị HAL từ chối (bus đang bận/kẹt)

// ===== THỐNG KÊ CHO MPU6500_Test =====
// Callback I2C cộng dồn từng mẫu (trong ngắt), luồng chính tính trung bình / độ nhiễu khi đo xong.
// Dùng số nguyên 64 bit: cộng bình phương az (~16384^2) x 1000 mẫu vẫn chính xác tuyệt đối.
typedef struct {
    int64_t sum;
    int64_t sumsq;
    int16_t min, max;
} IMU_ChStat;
// 7 kênh, đúng thứ tự 14 byte đọc từ thanh ghi 0x3B
enum { CH_AX, CH_AY, CH_AZ, CH_TEMP, CH_GX, CH_GY, CH_GZ, IMU_CH_COUNT };
static volatile IMU_ChStat imu_stat[IMU_CH_COUNT];
static volatile uint32_t   imu_stat_n = 0;
static volatile uint32_t   imu_stat_still = 0;  // Số mẫu lúc imu_still = 1
static volatile uint8_t    imu_stat_active = 0;

static char msg[256];                     // Bộ đệm in USB của module này

// ===== KHỞI ĐỘNG =====
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
                         "    -> Co the la MPU6500 (AD0=%s)\r\n",
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
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Đọc WHO_AM_I (0x75): 0x70 MPU6500, 0x71 lõi MPU9250 (cùng bản đồ thanh ghi); MPU6500_Test in ra
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &mpu_whoami, 1, 100);
    if (status != HAL_OK) return status;

    // 2. DLPF GYRO = GYRO_DLPF_CFG (3: băng thông 41Hz, trễ ~5.9ms) - lý do chọn xem mpu6500.h
    // MPU6500: thanh ghi này CHỈ lọc gyro (khác MPU6050 lọc cả accel) -> accel lọc riêng ở bước 4b
    data[0] = 0x1A; // REG_CONFIG
    data[1] = GYRO_DLPF_CFG;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 3. Tốc độ lấy mẫu (Sample Rate) = 1kHz / (1 + 1) = 500 Hz
    // Khớp với ngắt TIM9 đọc IMU mỗi 2ms -> mỗi lần đọc là 1 mẫu mới, không đọc lặp mẫu cũ
    data[0] = 0x19; // REG_SMPLRT_DIV
    data[1] = 0x01;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 4. Accel Full Scale: ±2g -> Độ nhạy 16384 LSB/g (Xe chạy mặt phẳng không cần lớn)
    data[0] = 0x1C; // REG_ACCEL_CONFIG
    data[1] = 0x00;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 4b. Bộ lọc riêng của Accel (chỉ MPU6500 có): A_DLPF_CFG = ACCEL_DLPF_CFG (4: ~20Hz)
    // Mặc định (0) gần như không lọc (~460Hz) trong khi đọc 500Hz -> rung động cơ lọt vào pitch/roll
    data[0] = REG_ACCEL_CONFIG2;
    data[1] = ACCEL_DLPF_CFG;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 5. Gyro Full Scale: ±2000 deg/s -> Độ nhạy 16.4 LSB/(deg/s)
    // BẮT BUỘC dùng thang đo lớn nhất vì Micromouse xoay rất gắt
    data[0] = 0x1B; // REG_GYRO_CONFIG
    data[1] = 0x18;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    return HAL_OK;
}
// Hiệu chỉnh offset gyro lúc khởi động, xe phải đứng yên. Bỏ GYRO_CAL_DISCARD mẫu đầu (gyro vừa
// khởi động / bộ lọc vừa đổi cấu hình), lấy trung bình GYRO_CAL_SAMPLES mẫu. Nhiễu lớn (xe bị chạm,
// rung) thì đo lại. Về sau offset Z còn tự chỉnh mỗi khi xe đứng yên (MPU6500_Process_Data).
HAL_StatusTypeDef MPU6500_Calibrate(void) {
    uint8_t raw_data[6];

    HAL_Delay(50);                        // Gyro khởi động từ sleep mất ~35ms (datasheet)
    for (int attempt = 0; attempt < GYRO_CAL_RETRY; attempt++) {
        int64_t sum[3] = { 0, 0, 0 }, sumsq[3] = { 0, 0, 0 };
        int n = 0;

        for (int i = 0; i < GYRO_CAL_DISCARD + GYRO_CAL_SAMPLES; i++) {
            HAL_Delay(2);                 // 1 mẫu mới mỗi 2ms (500Hz)
            // Đọc trực tiếp 6 byte thanh ghi Gyro (0x43 đến 0x48), lần đọc lỗi thì bỏ mẫu đó
            if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100) != HAL_OK) continue;
            if (i < GYRO_CAL_DISCARD) continue;
            for (int a = 0; a < 3; a++) {
                int16_t v = (int16_t)((raw_data[2 * a] << 8) | raw_data[2 * a + 1]);
                sum[a]   += v;
                sumsq[a] += (int32_t)v * v;
            }
            n++;
        }
        if (n < GYRO_CAL_SAMPLES / 2) continue;   // Bus I2C lỗi quá nhiều: đo lại

        // Trung bình = sai số tĩnh. Độ lệch chuẩn lớn nhất của 3 trục kiểm tra xe có đứng yên không
        float mean[3], worst_std = 0.0f;
        for (int a = 0; a < 3; a++) {
            double m   = (double)sum[a] / n;
            double var = (double)sumsq[a] / n - m * m;
            float std  = (var > 0.0) ? (float)sqrt(var) / 16.4f : 0.0f;
            mean[a] = (float)m;
            if (std > worst_std) worst_std = std;
        }
        gyro_x_offset = mean[0];
        gyro_y_offset = mean[1];
        gyro_z_offset = mean[2];
        gyro_z_cal    = mean[2];
        if (worst_std < GYRO_CAL_MAX_STD_DPS) return HAL_OK;
    }
    return HAL_BUSY;                      // Giữ kết quả lần đo cuối, offset Z sẽ tự sửa khi xe đứng yên
}
// Khởi tạo + hiệu chỉnh gyro (~1.2s, xe phải đứng yên), xong mới cho ngắt TIM9 đọc IMU bằng DMA.
// Lúc hiệu chỉnh ngắt TIM9 chưa đọc IMU nên không tranh bus I2C. MPU lỗi thì không bật đọc.
HAL_StatusTypeDef MPU6500_Setup(void)
{
    HAL_StatusTypeDef status = MPU6500_Init();
    if (status != HAL_OK) return status;

    status = MPU6500_Calibrate();
    yaw_angle  = 0.0f;
    still_time = 0.0f;
    imu_still  = 0;
    imu_ticks  = 0;
    imu_ready  = 1;
    return status;
}
void MPU6500_Read_All(void) {
    uint8_t buffer[14];
    HAL_StatusTypeDef status;

    // Đọc liên tiếp 14 byte từ địa chỉ 0x3B (ACCEL_XOUT_H)
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, buffer, 14, 100);

    if (status == HAL_OK) {
        Accel_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
        Accel_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
        Accel_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
        // buffer[6] và buffer[7] là nhiệt độ (TEMP_OUT)
        Gyro_X_RAW  = (int16_t)((buffer[8]  << 8) | buffer[9]);
        Gyro_Y_RAW  = (int16_t)((buffer[10] << 8) | buffer[11]);
        Gyro_Z_RAW  = (int16_t)((buffer[12] << 8) | buffer[13]);
    } else {
        CDC_Print("Loi doc I2C tu MPU6500!\r\n");
    }
}

// ===== GỌI TRONG NGẮT =====
// Ngắt TIM9 mỗi 2ms: chỉ RA LỆNH đọc bằng DMA, không xử lý MPU ở đây
// Chỉ đọc khi IMU đã khởi tạo + hiệu chỉnh xong (tránh tranh bus I2C với lệnh đọc kiểu chờ)
void MPU6500_Tick(void)
{
    imu_ticks++;
    if (imu_ready) {
        if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6500_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, imu_rx_buffer, 14) != HAL_OK) {
            imu_read_fail++;              // Lần đọc trước chưa xong / bus I2C kẹt
        }
    }
}
// Cộng 1 mẫu vừa đọc vào thống kê (gọi trong callback I2C khi MPU6500_Test đang đo)
static void IMU_Stat_Add(void)
{
    for (int c = 0; c < IMU_CH_COUNT; c++) {
        int16_t v = (int16_t)((imu_rx_buffer[2 * c] << 8) | imu_rx_buffer[2 * c + 1]);
        imu_stat[c].sum   += v;
        imu_stat[c].sumsq += (int32_t)v * v;
        if (v < imu_stat[c].min) imu_stat[c].min = v;
        if (v > imu_stat[c].max) imu_stat[c].max = v;
    }
    if (imu_still) imu_stat_still++;
    imu_stat_n++;
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
    imu_raw[0] = raw_ax; imu_raw[1] = raw_ay; imu_raw[2] = raw_az;
    imu_raw[3] = (int16_t)((imu_rx_buffer[6] << 8) | imu_rx_buffer[7]);
    imu_raw[4] = raw_gx; imu_raw[5] = raw_gy; imu_raw[6] = raw_gz;

    // 2. Trừ đi nhiễu tĩnh và chia hệ số nhạy (16.4 cho thang đo ±2000 dps)
    float rate_gx = (raw_gx - gyro_x_offset) / 16.4f;
    float rate_gy = (raw_gy - gyro_y_offset) / 16.4f;
    float rate_gz = (raw_gz - gyro_z_offset) / 16.4f;

    // 3. Tính góc tuyệt đối từ Gia tốc kế
    // Lưu ý: Tùy chiều gắn chip trên xe của bạn mà trục X/Y có thể đảo cho nhau
    float ax = raw_ax, ay = raw_ay, az = raw_az;      // float: ay^2 + az^2 kiểu int có thể tràn khi va đập
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.141592f;
    float roll_acc  = atan2f(ay, az) * 180.0f / 3.141592f;

    // 4. BỘ LỌC BÙ (COMPLEMENTARY FILTER), hằng số thời gian ANGLE_COMP_TAU_S
    // Ngắn hơn tau: tin gyro; dài hơn tau: accel kéo góc về (gyro không trôi mãi). Tính alpha theo dt thật
    float alpha = ANGLE_COMP_TAU_S / (ANGLE_COMP_TAU_S + dt);
    pitch_angle = alpha * (pitch_angle + rate_gy * dt) + (1.0f - alpha) * pitch_acc;
    roll_angle  = alpha * (roll_angle  + rate_gx * dt) + (1.0f - alpha) * roll_acc;

    // 5. Nhận biết đứng yên: 2 bánh không quay (encoder, pid.c) và |Gz| nhỏ, liên tục STILL_HOLD_S giây
    //    (xe bị nhấc lên xoay tay: bánh không quay nhưng Gz lớn -> không tính là đứng yên)
    if (fabsf(speed_left_mms) < STILL_SPEED_MMS && fabsf(speed_right_mms) < STILL_SPEED_MMS
        && fabsf(rate_gz) < STILL_GZ_DPS) {
        if (still_time < STILL_HOLD_S) still_time += dt;
        else imu_still = 1;
    } else {
        still_time = 0.0f;
        imu_still  = 0;
    }

    // 6. Góc Yaw (Heading): không có accel để bù, chỉ tích phân gyro
    yaw_rate_dps = YAW_SIGN * rate_gz;    // Cho khâu D giữ hướng (pid.c) và so sánh với encoder
    if (imu_still) {
        // Đứng yên: Gz đo được lúc này chính là bias -> offset bám theo (lọc thông thấp GYRO_BIAS_TAU_S),
        // sửa sai số hiệu chỉnh lúc khởi động và bias trôi theo nhiệt độ. Yaw giữ nguyên.
        float k = dt / GYRO_BIAS_TAU_S;
        if (k > 1.0f) k = 1.0f;
        gyro_z_offset += k * (raw_gz - gyro_z_offset);
    } else {
        // Đang chạy / đang quay: cộng toàn bộ, không ngưỡng chết (offset đã chính xác, nhiễu trung bình 0)
        // YAW_SIGN: yaw tăng khi xe quay trái
        yaw_angle += YAW_SIGN * rate_gz * dt;
    }
}
// DMA I2C1 đọc xong 14 byte (gọi từ ngắt DMA1_Stream0 / I2C1_EV qua HAL)
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        // 1. Tính toán MPU6500 (Góc Pitch, Roll, Yaw)
        //    dt = thời gian thật kể từ lần xử lý trước: 1 chu kỳ TIM9 = 2ms
        //    (trước đây cố định 0.005f trong khi gọi mỗi 2ms -> góc yaw bị cộng gấp 2.5 lần)
        float dt = imu_ticks * CONTROL_DT;
        imu_ticks = 0;
        MPU6500_Process_Data(dt);
        imu_sample_count++;
        if (imu_stat_active) IMU_Stat_Add();   // MPU6500_Test đang đo

        // 2. Chạy thuật toán PID ngay tại đây
    }
}
// Lỗi bus I2C (NACK, nhiễu làm hỏng xung SCL/SDA...): HAL đã hủy lần đọc DMA và trả I2C về READY,
// chu kỳ TIM9 sau đọc lại. Chỉ đếm để MPU6500_Test báo cáo.
uint32_t MPU6500_ErrorCount(void)
{
    return imu_i2c_errors + imu_read_fail;
}
float gyro_z_rezero_dps = 0.0f;
HAL_StatusTypeDef MPU6500_ZeroGyroZ(void)
{
    float sum = 0.0f, sumsq = 0.0f;
    uint32_t n = 0, last = imu_sample_count, t0 = HAL_GetTick();
    if (!imu_ready) return HAL_ERROR;
    while (n < GYRO_REZERO_SAMPLES && HAL_GetTick() - t0 < GYRO_REZERO_SAMPLES * 2U + 500U) {
        if (imu_sample_count == last) continue;           // Chờ mẫu DMA mới (2ms/mẫu)
        last = imu_sample_count;
        if (fabsf(speed_left_mms) >= STILL_SPEED_MMS || fabsf(speed_right_mms) >= STILL_SPEED_MMS) return HAL_BUSY;
        float v = imu_raw[6];
        sum   += v;
        sumsq += v * v;
        n++;
    }
    if (n < GYRO_REZERO_SAMPLES / 2) return HAL_TIMEOUT;
    float mean = sum / n, var = sumsq / n - mean * mean;
    if (var > 0.0f && sqrtf(var) / 16.4f > GYRO_CAL_MAX_STD_DPS) return HAL_BUSY;   // Bị chạm / rung
    gyro_z_rezero_dps = (mean - gyro_z_offset) / 16.4f;
    gyro_z_offset = mean;
    return HAL_OK;
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) imu_i2c_errors++;
}

// ===== DEBUG =====
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

// ===== TEST NHIỄU / NHIỆT ĐỘ (MPU6500_Test) =====
// Kết quả 1 lần đo, giá trị thô của chip (đổi đơn vị lúc in)
typedef struct {
    uint32_t n;                    // Số mẫu nhận được
    uint32_t expected;             // Số mẫu lẽ ra phải có (1 mẫu / 2ms)
    uint32_t errors;               // Lỗi I2C + lệnh đọc DMA bị từ chối trong lúc đo
    float mean[IMU_CH_COUNT];
    float std[IMU_CH_COUNT];       // Độ lệch chuẩn = độ nhiễu
    float p2p[IMU_CH_COUNT];       // Đỉnh - đỉnh
    float yaw_drift;               // Yaw thay đổi trong lúc đo (độ)
    float seconds;
    float speed_l, speed_r;        // Vận tốc bánh lúc đo (mm/s): kiểm tra bánh có quay hay không
    float still_pct;               // % mẫu nhận ra đứng yên (yaw giữ nguyên, offset Gz tự chỉnh)
    float gz_off;                  // Offset Gz (số thô) lúc đo xong, offset tự chỉnh nên thay đổi dần
} IMU_Result;
// 1 điểm theo dõi nhiệt độ (trung bình 1 giây)
typedef struct {
    float temp;                    // °C
    float gx, gy, gz;              // Gyro trung bình trừ offset LÚC KHỞI ĐỘNG (độ/s) = bias đã trôi
    float gz_off;                  // Offset Gz tự chỉnh - offset lúc khởi động (độ/s): phải bám theo gz
    float yaw;                     // Góc yaw lúc đó (độ)
} IMU_TempPoint;
static IMU_TempPoint imu_temp_log[MPU_TEMP_LOG_MAX];

// Đổi số thô nhiệt độ sang °C (datasheet MPU6500: 333.87 LSB/°C, 0 = 21°C)
static float IMU_TempC(float raw)
{
    return raw / 333.87f + 21.0f;
}
// Đo 'samples' mẫu (2ms/mẫu) ở điều kiện hiện tại của xe. Gọi ở luồng chính.
// IMU ngừng trả dữ liệu (bus I2C kẹt) thì thoát sau thời gian dự kiến + 1s, không treo.
static void IMU_Measure(uint32_t samples, IMU_Result *r)
{
    imu_stat_active = 0;
    for (int c = 0; c < IMU_CH_COUNT; c++) {
        imu_stat[c].sum   = 0;
        imu_stat[c].sumsq = 0;
        imu_stat[c].min   = INT16_MAX;
        imu_stat[c].max   = INT16_MIN;
    }
    imu_stat_n = 0;
    imu_stat_still = 0;
    uint32_t err0 = imu_i2c_errors + imu_read_fail;
    float yaw0 = yaw_angle;
    uint32_t t0 = HAL_GetTick();
    imu_stat_active = 1;
    while (imu_stat_n < samples && HAL_GetTick() - t0 < samples * 2 + 1000) {}
    imu_stat_active = 0;              // Callback I2C chạy xong trọn vẹn trước khi luồng chính chạy tiếp
    uint32_t ms = HAL_GetTick() - t0;

    r->n         = imu_stat_n;
    r->expected  = ms / 2;
    r->errors    = imu_i2c_errors + imu_read_fail - err0;
    r->yaw_drift = yaw_angle - yaw0;
    r->seconds   = ms / 1000.0f;
    r->speed_l   = speed_left_mms;
    r->speed_r   = speed_right_mms;
    r->still_pct = (r->n > 0) ? 100.0f * imu_stat_still / r->n : 0.0f;
    r->gz_off    = gyro_z_offset;
    for (int c = 0; c < IMU_CH_COUNT; c++) {
        if (r->n == 0) {
            r->mean[c] = r->std[c] = r->p2p[c] = 0.0f;
            continue;
        }
        double mean = (double)imu_stat[c].sum / r->n;
        double var  = (double)imu_stat[c].sumsq / r->n - mean * mean;
        r->mean[c] = (float)mean;
        r->std[c]  = (var > 0.0) ? (float)sqrt(var) : 0.0f;
        r->p2p[c]  = (float)(imu_stat[c].max - imu_stat[c].min);
    }
}
static uint32_t IMU_Lost(const IMU_Result *r)
{
    return (r->expected > r->n) ? r->expected - r->n : 0;
}
// Bias Gz còn lại (độ/s) = Gz trung bình trừ offset đang dùng: phần yaw thật sự cộng nhầm khi xe chạy
static float IMU_GzResidual(const IMU_Result *r)
{
    return (r->mean[CH_GZ] - r->gz_off) / 16.4f;
}
// 1 dòng bảng kết quả. Gyro: độ/s (chia 16.4), accel: mg (chia 16384 x 1000)
static void IMU_PrintRow(const char *name, const IMU_Result *r)
{
    snprintf(msg, sizeof(msg),
             "%-22s %6.2f %+7.3f %7.3f %8.2f %7.3f %7.3f %7.2f %7.2f %7.2f %8.2f %6.0f %5lu/%-5lu %3lu %4.0f/%-4.0f\r\n",
             name, IMU_TempC(r->mean[CH_TEMP]),
             IMU_GzResidual(r), r->std[CH_GZ] / 16.4f, r->p2p[CH_GZ] / 16.4f,
             r->std[CH_GX] / 16.4f, r->std[CH_GY] / 16.4f,
             r->std[CH_AX] * 1000.0f / 16384.0f, r->std[CH_AY] * 1000.0f / 16384.0f, r->std[CH_AZ] * 1000.0f / 16384.0f,
             (r->seconds > 0.0f) ? r->yaw_drift * 60.0f / r->seconds : 0.0f, r->still_pct,
             (unsigned long)r->n, (unsigned long)r->expected, (unsigned long)r->errors, r->speed_l, r->speed_r);
    CDC_Print(msg);
}
// Tên chip theo WHO_AM_I
static const char *IMU_ChipName(void)
{
    if (mpu_whoami == MPU6500_WHO_AM_I) return "MPU6500";
    if (mpu_whoami == MPU9250_WHO_AM_I) return "loi MPU9250, gyro/accel/nhiet giong MPU6500 -> dung duoc";
    return "LA - khong phai MPU6500/9250, kiem tra chip";
}
// ĐO NHIỄU IMU VÀ ẢNH HƯỞNG CỦA NHIỆT ĐỘ
//   1. Tắt PWM (phanh: 2 chân IN giữ mức cao, không có xung đóng cắt)
//   2. PWM băm xung MPU_TEST_PWM_LOW, bánh chưa quay   -> chỉ có nhiễu điện từ dòng đóng cắt 20kHz
//   3. Bánh quay đều MPU_TEST_SPEED bằng PID           -> nhiễu điện + rung cơ khí
//   4. Theo dõi bias gyro theo nhiệt độ temp_seconds giây, xe đứng yên tắt PWM (0 = bỏ qua)
// KÊ BÁNH LÊN (bước 3 bánh quay ~2.5s). Xe phải đứng yên tuyệt đối, kể cả lúc khởi động (hiệu chỉnh gyro).
// Chạy xong đứng phanh, CHỜ máy tính mở cổng COM rồi mới in (chạy không cắm dây vẫn lấy được kết quả).
void MPU6500_Test(uint16_t temp_seconds)
{
    static const char *names[3] = { "1.Tat PWM (phanh)", "2.PWM bam xung, dung", "3.Banh quay" };
    IMU_Result res[3];
    uint8_t n_res = 0;
    uint16_t n_temp = (temp_seconds > MPU_TEMP_LOG_MAX) ? MPU_TEMP_LOG_MAX : temp_seconds;

    if (!imu_ready) {
        CDC_WaitHost();
        CDC_Print("\r\nLOI: IMU chua san sang (MPU6500_Setup loi hoac chua goi) - khong test duoc\r\n");
        return;
    }

    // 1. Tắt PWM
    Motor_Stop();
    HAL_Delay(300);
    IMU_Measure(MPU_TEST_SAMPLES, &res[n_res++]);

    if (motor_hw_ok) {
        // 2. PWM băm xung, bánh đứng yên (PID đang tắt nên ngắt TIM9 không ghi đè PWM này)
        Motor_SetPWM(MPU_TEST_PWM_LOW, MPU_TEST_PWM_LOW);
        HAL_Delay(300);
        IMU_Measure(MPU_TEST_SAMPLES, &res[n_res++]);

        // 3. Bánh quay đều: chờ tăng tốc xong rồi mới đo
        Motor_SetSpeed(MPU_TEST_SPEED, MPU_TEST_SPEED);
        HAL_Delay(600);
        IMU_Measure(MPU_TEST_SAMPLES, &res[n_res++]);
        Motor_SetSpeed(0.0f, 0.0f);
        HAL_Delay(300);
    }
    Motor_Stop();

    // 4. Theo dõi nhiệt độ: mỗi giây 1 điểm (trung bình 500 mẫu)
    float yaw_start = yaw_angle;
    for (uint16_t i = 0; i < n_temp; i++) {
        IMU_Result r;
        IMU_Measure(500, &r);
        imu_temp_log[i].temp = IMU_TempC(r.mean[CH_TEMP]);
        imu_temp_log[i].gx   = (r.mean[CH_GX] - gyro_x_offset) / 16.4f;
        imu_temp_log[i].gy   = (r.mean[CH_GY] - gyro_y_offset) / 16.4f;
        imu_temp_log[i].gz   = (r.mean[CH_GZ] - gyro_z_cal) / 16.4f;
        imu_temp_log[i].gz_off = (r.gz_off - gyro_z_cal) / 16.4f;
        imu_temp_log[i].yaw  = yaw_angle - yaw_start;
    }

    // 5. Chờ máy tính rồi in báo cáo
    CDC_WaitHost();
    CDC_Print("\r\n================ TEST MPU6500 ================\r\n");
    snprintf(msg, sizeof(msg), "Chip: WHO_AM_I=0x%02X (%s)\r\n", mpu_whoami, IMU_ChipName());
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Offset gyro luc khoi dong (do/s): X=%+.3f Y=%+.3f Z=%+.3f | Offset Z tu chinh hien tai %+.3f (da sua %+.3f)\r\n",
             gyro_x_offset / 16.4f, gyro_y_offset / 16.4f, gyro_z_cal / 16.4f,
             gyro_z_offset / 16.4f, (gyro_z_offset - gyro_z_cal) / 16.4f);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Bo loc: gyro DLPF cfg %d, accel DLPF cfg %d, bu pitch/roll tau %.2f s | Dung yen: banh <%.1f mm/s, |Gz| <%.1f do/s trong %.2f s\r\n",
             GYRO_DLPF_CFG, ACCEL_DLPF_CFG, ANGLE_COMP_TAU_S, STILL_SPEED_MMS, STILL_GZ_DPS, STILL_HOLD_S);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Moi dieu kien %d mau (%.1f s) | PWM bam xung = %d/%d | Banh quay = %.0f mm/s\r\n",
             MPU_TEST_SAMPLES, MPU_TEST_SAMPLES * CONTROL_DT, MPU_TEST_PWM_LOW, PWM_MAX, MPU_TEST_SPEED);
    CDC_Print(msg);
    if (!motor_hw_ok) CDC_Print("(Dong co dang bi khoa -> bo qua dieu kien 2, 3)\r\n");
    snprintf(msg, sizeof(msg), "%-22s %6s %7s %7s %8s %7s %7s %7s %7s %7s %8s %6s %11s %3s %9s\r\n",
             "Dieu kien", "Nhiet", "GzTB", "GzNhieu", "GzDinh", "GxNhieu", "GyNhieu",
             "AxNhieu", "AyNhieu", "AzNhieu", "YawTroi", "DYen%", "Mau/DuKien", "Loi", "VbanhT/P");
    CDC_Print(msg);
    for (int k = 0; k < n_res; k++) IMU_PrintRow(names[k], &res[k]);
    CDC_Print("Don vi: Nhiet C | Gz/Gx/Gy do/s | Ax/Ay/Az mg | YawTroi do/phut | Vbanh mm/s\r\n");
    CDC_Print("GzTB: Gz trung binh tru offset dang dung = bias con lai | Nhieu: do lech chuan | GzDinh: dinh-dinh\r\n");
    CDC_Print("DYen%: % mau nhan ra dung yen (yaw giu nguyen, offset Z tu chinh) - dieu kien 1, 2 phai ~100, 3 phai 0\r\n");
    CDC_Print("Mau/DuKien: mau nhan duoc / le ra (1 mau 2ms) | Loi: loi I2C + lenh doc DMA bi tu choi\r\n");

    // Nhận xét: bias còn lại là phần yaw cộng nhầm khi xe chạy (nhiễu trung bình 0 thì tích phân tự triệt tiêu)
    float base = res[0].std[CH_GZ] / 16.4f;
    for (int k = 0; k < n_res; k++) {
        float noise = res[k].std[CH_GZ] / 16.4f;
        float resid = fabsf(IMU_GzResidual(&res[k]));
        uint8_t bad_bus = (res[k].errors > 0) || (IMU_Lost(&res[k]) > res[k].expected / 100);
        uint8_t spun = (k == 1) && (fabsf(res[k].speed_l) > 20.0f || fabsf(res[k].speed_r) > 20.0f);
        uint8_t no_still = (k < 2) && !spun && (res[k].still_pct < 50.0f);
        snprintf(msg, sizeof(msg), "%s: nhieu Gz x%.1f so voi tat PWM | bias con lai %.3f do/s (~%.1f do/phut khi chay) %s%s%s%s\r\n",
                 names[k], (base > 0.0f) ? noise / base : 0.0f, resid, resid * 60.0f,
                 (resid > YAW_BIAS_WARN_DPS) ? "CAO" : "OK",
                 no_still ? " | CANH BAO: khong nhan ra dung yen (encoder rung?)" : "",
                 bad_bus ? " | CANH BAO: mat mau/loi I2C" : "",
                 spun ? " | banh da quay -> giam MPU_TEST_PWM_LOW" : "");
        CDC_Print(msg);
    }

    // Nhiệt độ: in từng giây + hệ số trôi bias theo nhiệt độ (đường thẳng bình phương nhỏ nhất)
    if (n_temp > 0) {
        snprintf(msg, sizeof(msg), "---------------- THEO DOI NHIET DO %u s (xe dung yen, tat PWM) ----------------\r\n", n_temp);
        CDC_Print(msg);
        CDC_Print("gz: bias Gz so voi luc khoi dong | gz_off: offset tu chinh so voi luc khoi dong (phai bam theo gz)\r\n");
        CDC_Print("TEMP,t_s,nhiet_C,gx_dps,gy_dps,gz_dps,gz_off_dps,yaw_do\r\n");
        for (uint16_t i = 0; i < n_temp; i++) {
            snprintf(msg, sizeof(msg), "TEMP,%u,%.2f,%+.4f,%+.4f,%+.4f,%+.4f,%.2f\r\n", i + 1,
                     imu_temp_log[i].temp, imu_temp_log[i].gx, imu_temp_log[i].gy, imu_temp_log[i].gz,
                     imu_temp_log[i].gz_off, imu_temp_log[i].yaw);
            CDC_Print(msg);
        }
        double mt = 0.0, mg[3] = { 0.0, 0.0, 0.0 };
        for (uint16_t i = 0; i < n_temp; i++) {
            mt    += imu_temp_log[i].temp;
            mg[0] += imu_temp_log[i].gx;
            mg[1] += imu_temp_log[i].gy;
            mg[2] += imu_temp_log[i].gz;
        }
        mt /= n_temp;
        for (int a = 0; a < 3; a++) mg[a] /= n_temp;
        double stt = 0.0, stg[3] = { 0.0, 0.0, 0.0 };
        for (uint16_t i = 0; i < n_temp; i++) {
            double d = imu_temp_log[i].temp - mt;
            stt    += d * d;
            stg[0] += d * (imu_temp_log[i].gx - mg[0]);
            stg[1] += d * (imu_temp_log[i].gy - mg[1]);
            stg[2] += d * (imu_temp_log[i].gz - mg[2]);
        }
        float t_first = imu_temp_log[0].temp, t_last = imu_temp_log[n_temp - 1].temp;
        snprintf(msg, sizeof(msg), "Nhiet do: %.2f -> %.2f C (thay doi %+.2f C) | Yaw troi %.2f do trong %u s (dung yen: phai ~0)\r\n",
                 t_first, t_last, t_last - t_first, imu_temp_log[n_temp - 1].yaw, n_temp);
        CDC_Print(msg);
        snprintf(msg, sizeof(msg), "Bias Gz tu luc khoi dong: %+.4f do/s | offset tu chinh con lech %+.4f do/s\r\n",
                 imu_temp_log[n_temp - 1].gz, imu_temp_log[n_temp - 1].gz - imu_temp_log[n_temp - 1].gz_off);
        CDC_Print(msg);
        if (fabsf(t_last - t_first) >= 1.0f && stt > 0.0) {
            snprintf(msg, sizeof(msg), "He so troi bias theo nhiet do: Gx %+.4f  Gy %+.4f  Gz %+.4f do/s/C\r\n",
                     stg[0] / stt, stg[1] / stt, stg[2] / stt);
        } else {
            snprintf(msg, sizeof(msg), "Nhiet do thay doi < 1 C: chua du de tinh he so troi (chay lau hon tu luc nguoi, hoac lam nong MPU)\r\n");
        }
        CDC_Print(msg);
    }
    CDC_Print("================ HET TEST MPU6500 ================\r\n");
}
