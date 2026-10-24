/**
  ******************************************************************************
  * @file    mpu6500.h
  * @brief   IMU MPU6500 qua I2C1: khởi tạo, hiệu chỉnh gyro, đọc DMA mỗi 2ms,
  *          tính góc pitch/roll (bộ lọc bù) và yaw (tích phân gyro)
  ******************************************************************************
  */
#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"

// ===== THANH GHI / ĐỊA CHỈ =====
#define MPU6500_ADDR          (0x68 << 1) // Địa chỉ I2C 8-bit (0xD0) nếu AD0 nối GND
#define REG_SMPLRT_DIV        0x19        // Sample Rate Divider
#define REG_CONFIG            0x1A        // Cấu hình DLPF & FSYNC (MPU6500: chỉ lọc GYRO)
#define REG_GYRO_CONFIG       0x1B        // Dải đo Gyro
#define REG_ACCEL_CONFIG      0x1C        // Dải đo Accel
#define REG_ACCEL_CONFIG2     0x1D        // Bộ lọc riêng của Accel (MPU6500)
#define REG_PWR_MGMT_1        0x6B        // Quản lý nguồn & Clock Source
#define REG_WHO_AM_I          0x75        // Mã nhận dạng chip
#define MPU6500_WHO_AM_I      0x70        // MPU6500
#define MPU9250_WHO_AM_I      0x71        // Lõi MPU9250 (nhiều module bán tên MPU6500): gyro/accel/nhiệt độ giống hệt

// ===== BỘ LỌC PHẦN CỨNG (chọn theo dữ liệu MPU6500_Test) =====
// Gyro 41Hz (trễ ~5.9ms): rung cơ khí lúc bánh quay chỉ là nhiễu trung bình 0, tích phân yaw tự triệt tiêu,
// hạ xuống 20Hz chỉ làm trễ gần gấp đôi (xấu cho vòng điều khiển góc quay) mà yaw không tốt hơn
#define GYRO_DLPF_CFG         0x03
// Accel ~20Hz: accel chỉ dùng neo pitch/roll (thay đổi rất chậm) -> lọc mạnh không mất gì, giảm rung lọt vào
#define ACCEL_DLPF_CFG        0x04
// Dải đo accel (ACCEL_CONFIG): 0x00 ±2g 16384 LSB/g | 0x08 ±4g 8192 | 0x10 ±8g 4096 | 0x18 ±16g 2048.
// ±8g từ 22/10/2026 (trước ±2g): chip lắp úp, nằm yên trục Z đã -1.17g -> ±2g chỉ còn 0.83g tới giới hạn, cú xóc qua gờ
// sàn / va chạm bị cắt 1 phía NGAY TRONG CHIP (trước DLPF) -> vận tốc đứng lệch giả (báo nhấc nhầm), độ giật va chạm bị
// cụt. ±8g còn ~6.8g; độ phân giải 0.24mg vẫn nhỏ hơn nhiễu chip
#define ACCEL_FS_CFG          0x10
#define ACCEL_LSB_PER_G       4096.0f
// Bộ lọc bù pitch/roll: hằng số thời gian (s). Alpha cũ 0.96 ~ 0.05s: xe tăng tốc 2000mm/s2 (0.2g)
// là pitch sai ~11 độ; 0.5s thì gia tốc ngắn của xe gần như không làm lệch góc
#define ANGLE_COMP_TAU_S      0.5f

// ===== YAW: CHIỀU + NHẬN BIẾT ĐỨNG YÊN + TỰ CHỈNH OFFSET GYRO Z =====
// Quy ước: yaw_angle TĂNG khi xe quay TRÁI (nhìn từ trên xuống). Dấu phụ thuộc chiều gắn chip (ngửa/úp):
// xoay tay xe sang trái mà yaw giảm thì đổi dấu. Xe này: gyro Z ngược quy ước -> -1 (đã kiểm tra trên xe)
#define YAW_SIGN              (-1)
// Đứng yên = 2 bánh không quay (encoder) VÀ Gz ÊM (không dao động) VÀ |Gz| nhỏ, liên tục một khoảng thời gian.
// Khi đứng yên: giữ nguyên yaw và cho offset Gz bám theo bias thật (sai số hiệu chỉnh, trôi theo nhiệt độ).
// Khi chạy: cộng TOÀN BỘ gyro vào yaw, không có ngưỡng chết (ngưỡng chết cũ 0.8 độ/s nuốt mất các
// góc lệch nhỏ khi chạy thẳng và biến nhiễu rung thành trôi góc ~1.4 độ/phút)
// Vì sao phải xét Gz êm: xoay xe bằng tay lúc động cơ đang phanh -> bánh trượt chứ không lăn, encoder báo
// đứng yên -> đoạn xoay chậm cuối mỗi lần chỉnh tay bị coi là đứng yên: yaw bị giữ + offset bị kéo lệch
// (đo trên xe 09/10/2026: xoay tay về hướng cũ mà yaw lệch -5..+2 độ; mô phỏng 6 lần xoay lệch 22 độ).
// Xe nằm yên trên sàn: Gz gần như không đổi (dao động ~0.05 độ/s). Tay cầm / xoay tay: Gz luôn dao động.
#define STILL_SPEED_MMS       1.0f        // |vận tốc bánh đã lọc| dưới mức này coi như bánh đứng
#define STILL_DEV_DPS         0.2f        // Độ dao động Gz (TB |Gz - TB trượt 0.1s|, lọc 0.25s) phải dưới mức này
#define STILL_GZ_DPS          1.0f        // |Gz| < 1 độ/s: đứng yên sau STILL_HOLD_S
#define STILL_HOLD_S          0.25f
// 1..5 độ/s mà vẫn êm: chỉ có thể là offset sai (vd hiệu chỉnh lúc bật nguồn bị xoay nhẹ, từng lệch 2.68 độ/s)
// -> vẫn tính là đứng yên nhưng phải giữ lâu hơn, để offset tự sửa được (MPU6500: offset xuất xưởng +-5 độ/s)
#define STILL_GZ_MAX_DPS      5.0f
#define STILL_HOLD_LONG_S     2.0f
#define GYRO_BIAS_TAU_S       2.0f        // Tốc độ offset Gz bám theo bias khi đứng yên (hằng số thời gian)
#define GYRO_REZERO_SAMPLES   250         // MPU6500_ZeroGyroZ: lấy trung bình 250 mẫu (0.5s)

// ===== HIỆU CHỈNH GYRO LÚC KHỞI ĐỘNG =====
#define GYRO_CAL_SAMPLES      500         // Số mẫu lấy trung bình (~1.2s)
#define GYRO_CAL_DISCARD      50          // Bỏ các mẫu đầu: gyro vừa khởi động / DLPF vừa đổi cấu hình
#define GYRO_CAL_MAX_STD_DPS  0.3f        // Nhiễu lớn hơn = xe bị chạm/rung lúc hiệu chỉnh -> đo lại
// Trung bình Gz nửa đầu và nửa sau lệch nhau hơn mức này = xe đang bị xoay chậm (đặt xe xuống) -> đo lại
// (nhiễu chỉ làm 2 trung bình lệch ~0.01 độ/s)
#define GYRO_CAL_MAX_DRIFT_DPS 0.1f
#define GYRO_CAL_RETRY        5           // Số lần đo lại tối đa (~1.5s mỗi lần: 550 mẫu x (HAL_Delay(2) ~2.5ms + đọc I2C))
// GIỚI HẠN THỜI GIAN hiệu chỉnh lúc khởi động (người dùng yêu cầu 14/10/2026, trước đây xe bị cầm / rung thì đo lại tới
// ~9s như bị treo): không đủ thời gian đo thêm 1 lần nữa thì thôi -> HAL_TIMEOUT, xe khởi động ở trạng thái "bị nhấc"
// (khóa động cơ, LED PB15 tắt) và tự hiệu chỉnh lại khi được đặt nằm yên trên sàn (Lift_Update)
#define GYRO_CAL_MAX_MS       3000
#define GYRO_CAL_I2C_TIMEOUT_MS 5         // 1 lần đọc 6 byte ~0.3ms; bus I2C kẹt thì không chờ 100ms mỗi mẫu

// ===== CẤU HÌNH MPU6500_Test =====
#define MPU_TEST_SAMPLES      1000        // Số mẫu mỗi điều kiện đo nhiễu (1000 x 2ms = 2s)
#define MPU_TEST_PWM_LOW      150         // PWM băm xung mà bánh chưa quay (300: bánh kê lên đã quay 30-50mm/s)
#define MPU_TEST_SPEED        200.0f      // Vận tốc bánh khi đo nhiễu lúc động cơ quay (mm/s)
#define MPU_TEMP_LOG_MAX      600         // Theo dõi nhiệt độ tối đa 600s, 1 điểm/giây (~14KB RAM)
#define YAW_BIAS_WARN_DPS     0.05f       // Bias Gz còn lại lớn hơn mức này (~3 độ/phút) thì báo cao

// ===== NHẤC XE / ĐẶT XE (tự động, chạy trong ngắt IMU) =====
// NHẤC XE LÊN: tắt động cơ ngay (phanh), hủy chuyển động đang chạy (Move_Straight / Turn_Angle... trả về 0),
//   chặn Motor_SetSpeed / Motor_SetPWM, TẮT LED PB15.
// ĐẶT XE XUỐNG, nằm yên LIFT_PLACE_S: chỉnh lại offset gyro 3 trục, yaw = 0 (góc ban đầu = góc lúc đặt xuống),
//   gốc vị trí + hướng đích = chỗ đặt, BẬT LED PB15 = xe sẵn sàng hoạt động.
// Nhận biết nhấc (22/10/2026: thêm tốc độ lật + duy trì 200ms theo yêu cầu người dùng), 1 trong 3 dấu hiệu:
//   - nghiêng: hướng trọng lực (accel lọc LIFT_TILT_TAU_S) lệch hướng lúc nằm trên sàn > LIFT_TILT_DEG
//     (xe chạy trên sàn: tăng/giảm tốc, va tường chỉ làm lệch < ~10 độ)
//   - nâng thẳng lên: vận tốc theo phương thẳng đứng (tích phân gia tốc, rò LIFT_VZ_TAU_S) > LIFT_VZ_MS
//   - tay lật xe: tốc độ quay quanh 2 trục NẰM NGANG (roll + pitch, tính theo hướng trọng lực lúc nằm trên sàn chứ
//     không theo trục chip - chip có thể lắp nghiêng) |w1| + |w2| > LIFT_TILT_RATE_DPS, chỉ xét khi xe không tự
//     quay nhanh (|tốc độ quay quanh trục đứng| < LIFT_TILT_RATE_YAW_DPS): chip nghiêng ~10 độ thì quay tại chỗ
//     300 độ/s làm gx, gy ~74 độ/s -> không chặn thì lần quay nào cũng bị coi là bị nhấc
//   phải đúng LIÊN TỤC LIFT_CONFIRM_S (200ms): vấp mối ghép sàn chỉ gây giật vài chục ms rồi hết -> bỏ qua, chạy tiếp
//   (trước đây 20ms: vấp gờ có thể bị nhận nhầm là bị nhấc)
// Nhận biết đặt xuống: liên tục LIFT_PLACE_S có |gyro| mọi trục < LIFT_PLACE_GYRO_DPS, Gz êm (như nhận biết
//   đứng yên: gz_dev < STILL_DEV_DPS), độ rung gia tốc < LIFT_PLACE_ACC_DEV_G, 2 bánh đứng yên,
//   nghiêng < LIFT_PLACE_TILT_DEG. Tay cầm xe luôn run (gia tốc > ~5mg, gyro > ~0.5 độ/s) -> cầm yên trên tay
//   không bị nhận nhầm là đã đặt; đếm chỉ bắt đầu khi tay đã buông (mô phỏng: LED sáng ~1.4s sau khi buông)
// Theo dõi để chỉnh ngưỡng: cột nghieng, vz, rung, nhac của print_mpu6500()
#define LIFT_TILT_DEG         20.0f       // Nghiêng quá mức này = bị nhấc (độ)
#define LIFT_TILT_TAU_S       0.2f        // Lọc accel khi tính nghiêng (s): bỏ qua cú tăng tốc / va chạm ngắn
#define LIFT_VZ_MS            0.12f       // Vận tốc thẳng đứng quá mức này = bị nhấc (m/s)
#define LIFT_VZ_TAU_S         0.3f        // Hằng số rò của tích phân vận tốc đứng (s): sai số offset không cộng dồn
// Xóc thẳng đứng mạnh (gờ sàn, va chạm): |gia tốc đứng - lúc nằm yên| > LIFT_VZ_SHOCK_G -> vận tốc đứng = 0 trong
// LIFT_VZ_SHOCK_S. Tay nhấc không tạo gia tốc lớn vậy. Lý do: chip lắp úp, trục Z lúc nằm yên đã -1.17g, chỉ còn 0.83g
// tới giới hạn +-2g -> cú xóc bị cắt 1 phía, tích phân vz lệch giả (mô phỏng: gờ 1mm @300mm/s giữ |vz| > 0.12 hơn 200ms)
#define LIFT_VZ_SHOCK_G       0.5f
#define LIFT_VZ_SHOCK_S       0.1f
#define LIFT_TILT_RATE_DPS    30.0f       // Tốc độ lật roll + pitch (độ/s) quá mức này = tay đang nghiêng xe
#define LIFT_TILT_RATE_YAW_DPS 80.0f      // Chỉ xét tốc độ lật khi |tốc độ quay quanh trục đứng| dưới mức này
#define LIFT_CONFIRM_S        0.2f        // Điều kiện nhấc phải đúng LIÊN TỤC chừng này (vấp gờ ngắn hơn -> bỏ qua)
#define LIFT_PLACE_S          1.0f        // Nằm yên chừng này mới coi là đã đặt xuống (s)
#define LIFT_PLACE_GYRO_DPS   2.0f
#define LIFT_PLACE_TILT_DEG   10.0f
#define LIFT_PLACE_ACC_DEV_G  0.004f      // Độ rung |a| (TB |(|a| - TB trượt 0.1s)|): sàn ~1.5mg, tay cầm > 5mg
// ===== VA CHẠM (Crash Detection, 22/10/2026 theo yêu cầu người dùng) =====
// Dùng ĐỘ GIẬT = biến thiên trong CRASH_LAG_SAMPLES mẫu (10ms), KHÔNG so gia tốc tuyệt đối: PID tăng tốc / phanh gấp
// (<= 0.2g, đổi chậm vì quán tính + DLPF accel 20Hz) không bị báo. Chỉ xét khi PID đang chạy (motor_pid_enable).
//   - Lọc gai I2C: mỗi kênh qua trung vị 3 mẫu TRƯỚC khi tính giật. (1 mẫu hỏng tạo 2 lần giật liên tiếp - vọt lên rồi
//     về - nên chỉ đếm "2 chu kỳ liên tiếp" thì vẫn báo nhầm)
//   - giật ngang planar = |da.e1| + |da.e2|: e1, e2 = 2 trục NẰM NGANG theo hướng trọng lực lúc nằm trên sàn (không theo
//     trục chip: chip lắp úp, có thể nghiêng) -> vấp gờ sàn (giật theo chiều đứng) không lọt vào
//   - Đâm trực diện:  planar > CRASH_JERK_G
//   - Quệt góc tường: planar > CRASH_JERK_SIDE_G và |dGz| > CRASH_GZ_JERK_DPS (đầu xe bị văng)
//   - đúng liên tục CRASH_CONFIRM_N mẫu -> IMU_STATE_CRASHED: khóa động cơ, hủy chuyển động y như bị nhấc; chạy lại
//     khi xe nằm yên (như đặt xuống, xem trên). Mode Manager: tạm dừng, lưu Flash, đứng im PAUSE_STILL_MS thì chạy tiếp
// NGƯỠNG CHƯA ĐO TRÊN XE: số của người dùng (18000 / 12000 LSB ở thang ±2g = 1.10 / 0.73g), tính theo g nên không phụ
// thuộc dải đo. Chỉnh theo cột giat / giatz của print_mpu6500() khi chạy thử, đâm thử, vấp gờ
#define CRASH_LAG_SAMPLES     5           // So với mẫu cách 5 mẫu = 10ms (1 mẫu / 2ms)
#define CRASH_JERK_G          1.10f       // = 18000 LSB: đâm trực diện
#define CRASH_JERK_SIDE_G     0.73f       // = 12000 LSB: quệt (kèm đầu xe văng)
#define CRASH_GZ_JERK_DPS     76.0f       // Đầu xe văng: Gz đổi > 76 độ/s trong 10ms (người dùng ghi 10000 LSB theo thang
                                          // ±250 độ/s; xe dùng ±2000 độ/s 16.4 LSB/(độ/s) -> 1250 LSB)
#define CRASH_CONFIRM_N       2           // Số mẫu liên tiếp vượt ngưỡng
#define LED_RUN_PORT          GPIOB       // LED báo xe đang trên sàn, sẵn sàng hoạt động
#define LED_RUN_PIN           GPIO_PIN_15
#define LED_RUN_ON            GPIO_PIN_SET    // Mức logic làm LED sáng (LED nối ngược thì đổi 2 dòng này)
#define LED_RUN_OFF           GPIO_PIN_RESET

// ===== KẾT QUẢ =====
extern float pitch_angle;                 // Góc ngóc đầu/chúi mũi (độ)
extern float roll_angle;                  // Góc lật nghiêng xe (độ)
extern float yaw_angle;                   // Góc đã quay cộng dồn (độ), KHÔNG quấn (vd 450 = 1 vòng + 90) - điều khiển dùng
float MPU6500_Wrap360(float deg);         // Quấn góc về [0, 360) - để hiển thị / đổi ra hướng
extern volatile float yaw_rate_dps;      // Tốc độ quay quanh Z (độ/s) đã trừ offset, cùng chiều yaw (dương = trái)
extern volatile int16_t imu_raw[7];       // Mẫu thô mới nhất: ax, ay, az, nhiệt, gx, gy, gz
extern volatile uint32_t imu_sample_count; // Tăng 1 mỗi mẫu IMU xử lý xong
extern float gyro_x_offset, gyro_y_offset, gyro_z_offset;
extern volatile uint8_t imu_ready;       // 1: ngắt TIM9 đang đọc IMU bằng DMA
extern volatile uint8_t imu_still;        // 1: xe đang đứng yên (yaw giữ nguyên, offset Gz đang tự chỉnh)
extern uint8_t mpu_whoami;                // Đọc lúc khởi tạo: 0x70 MPU6500, 0x71 lõi MPU9250
extern volatile uint8_t robot_lifted;     // 1: động cơ bị khóa an toàn (bị nhấc HOẶC va chạm, chờ nằm yên), 0: chạy được
// Trạng thái an toàn IMU (lý do của robot_lifted)
#define IMU_STATE_NORMAL      0U          // Trên sàn, chạy được
#define IMU_STATE_PICKED_UP   1U          // Bị nhấc (gia tốc kế / encoder / IR) hoặc khởi động không hiệu chỉnh được
#define IMU_STATE_CRASHED     2U          // Va chạm
extern volatile uint8_t imu_state;        // IMU_STATE_*
extern float lift_tilt_deg;               // Độ nghiêng so với lúc nằm trên sàn (độ) - để theo dõi / chỉnh ngưỡng
extern float lift_vz_ms;                  // Vận tốc thẳng đứng ước lượng (m/s, dương = đi lên)
extern float lift_acc_dev;                // Độ rung gia tốc (g) - nằm yên trên sàn ~0.0015
extern float lift_tilt_rate_dps;          // Tốc độ lật roll + pitch (độ/s), so với LIFT_TILT_RATE_DPS

// ===== KHỞI ĐỘNG (luồng chính) =====
HAL_StatusTypeDef MPU6500_Init(void);
HAL_StatusTypeDef MPU6500_Calibrate(void); // HAL_BUSY: xe không đứng yên sau GYRO_CAL_RETRY lần đo, HAL_TIMEOUT: hết GYRO_CAL_MAX_MS
// Init + Calibrate + cho phép ngắt TIM9 đọc DMA. Khác HAL_OK: MPU lỗi (không đọc) hoặc hiệu chỉnh
// bị rung (vẫn chạy, offset Gz tự sửa dần khi xe đứng yên)
HAL_StatusTypeDef MPU6500_Setup(void);
// Chỉnh lại offset gyro Z khi xe đứng yên trên sàn (luồng chính, sau khi imu_ready = 1, dùng mẫu DMA):
// trung bình GYRO_REZERO_SAMPLES mẫu. Bánh quay / bị chạm (nhiễu lớn) thì giữ offset cũ, trả HAL_BUSY
HAL_StatusTypeDef MPU6500_ZeroGyroZ(void);
// Như trên nhưng KHÔNG CHẶN (Mode Manager): Start rồi gọi Poll mỗi vòng lặp tới khi khác GYRO_ZERO_RUNNING.
// Kết quả như MPU6500_ZeroGyroZ: HAL_OK = offset mới, HAL_BUSY = bánh quay / bị chạm (giữ offset cũ), HAL_TIMEOUT, HAL_ERROR
#define GYRO_ZERO_RUNNING     0xFFU
void    MPU6500_ZeroGyroZ_Start(void);
uint8_t MPU6500_ZeroGyroZ_Poll(void);
// Bộ phát hiện nhấc xe khác (encoder / IR, mode_manager.c) báo bị nhấc -> trạng thái như gia tốc kế phát hiện.
// Chỉ gọi trong ngắt TIM9 (cùng mức ưu tiên với ngắt I2C)
void    MPU6500_ForceLifted(void);

// ===== GỌI TRONG NGẮT =====
void MPU6500_Tick(void);                  // Ngắt TIM9 mỗi 2ms: ra lệnh đọc 14 byte bằng DMA
void MPU6500_Process_Data(float dt);      // Callback I2C DMA xong: tính góc

// ===== DEBUG / TEST (luồng chính) =====
// In 1 dòng giá trị MPU SAU BỘ LỌC qua USART1 (HC-05): pitch, roll, yaw, gyro x/y/z (độ/s), accel x/y/z (g),
// nhiệt độ, cờ đứng yên, các số của nhận biết nhấc xe / va chạm (nghieng, vz, rung, nhac, lat, giat, giatz, tt).
// Gọi lặp trong while(1), người gọi tự giãn nhịp. Tiêu đề in mỗi 50 dòng
void print_mpu6500(void);
// Đo nhiễu IMU khi tắt PWM / PWM băm xung / bánh quay, rồi theo dõi bias gyro theo nhiệt độ
// trong temp_seconds giây (0 = bỏ qua). KÊ BÁNH LÊN. Xong chờ máy tính mở cổng COM rồi in báo cáo.
void MPU6500_Test(uint16_t temp_seconds);

#endif /* MPU6500_H */
