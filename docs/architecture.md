# KIẾN TRÚC TỔNG THỂ HỆ THỐNG (SYSTEM ARCHITECTURE)

Tài liệu này đặc tả kiến trúc phần mềm, cấu trúc phân tầng, phân bổ tài nguyên phần cứng và trách nhiệm của từng module trong firmware Robot Micromouse trên vi điều khiển STM32F411CEU6.

---

## 1. Kiến trúc phân tầng (Layered Architecture)

Hệ thống được thiết kế theo mô hình **kiến trúc phân tầng hướng sự kiện và thời gian thực (Real-time Event-Driven Layered Architecture)**. Phần mềm được tách bạch rõ rệt giữa hai miền thực thi:

1. **Miền thời gian thực (Real-Time Interrupt Domain - 500 Hz / 2 ms):**
   - Chạy hoàn toàn bên trong ngắt timer cứng `TIM9` (`HAL_TIM_PeriodElapsedCallback`).
   - Đảm bảo tính tiền định (deterministic timing): đọc encoder, tính vận tốc tức thời, lọc cảm biến IR, cập nhật vòng điều khiển PID vận tốc bánh xe, điều khiển quỹ đạo chuyển động góc/hướng, gửi lệnh đọc DMA cảm biến quán tính IMU và giám sát các điều kiện an toàn khẩn cấp (nhấc xe, kẹt bánh, va chạm).
   - Tuyệt đối không chứa vòng lặp chờ, không chứa hàm chặn (`HAL_Delay`), không in chuỗi qua cổng nối tiếp trong miền này.

2. **Miền ứng dụng & Logic nghiệp vụ (Application / Background Domain):**
   - Chạy trong vòng lặp chính `while(1)` ở thread mode thông qua máy trạng thái `ModeManager_Update()`.
   - Xử lý các tác vụ mức cao: giao diện người dùng bằng cử chỉ che cảm biến IR trái, phát chuỗi âm thanh còi báo (buzzer pattern), điều phối thuật toán tìm đường Flood-Fill, gộp các ô đi thẳng, tính toán lại đường đi trong lúc di chuyển, lưu trữ dữ liệu vào bộ nhớ Flash Sector 7 và truyền dữ liệu bản đồ lên máy tính qua Bluetooth/USB.
   - Hoạt động hoàn toàn theo cơ chế non-blocking state machine dựa trên biến thời gian `HAL_GetTick()`.

```mermaid
flowchart TD
    subgraph HARDWARE["LỚP PHẦN CỨNG (STM32F411CEU6)"]
        IR_HW["4x IR Sensors (PA4-PA7) + Emitters (PA8-PA10)"]
        ENC_HW["2x Encoders (TIM2 PA0/PA1, TIM3 PB4/PB5)"]
        IMU_HW["MPU6500 IMU (I2C1 PB6/PB7 @300kHz)"]
        MOT_HW["DRV8833 Dual H-Bridge (TIM1 PB0/PB1, TIM5 PA3/PA2 @20kHz)"]
        BUZ_HW["Buzzer (PB8) + LEDs (PB14 IMU, PB15 Run)"]
        COM_HW["USART1 HC-05 (PA15/PB3) + USB OTG FS (PA11/PA12)"]
        FLS_HW["Flash On-Chip (Sector 7: 128 KB @ 0x08060000)"]
    end

    subgraph DRIVERS["LỚP ĐIỀU KHIỂN THIẾT BỊ (HAL & DRIVERS)"]
        ADC_DRV["ADC1 Multi-channel + DMA2 Stream0 (Trigger TIM4 CC4 ~5kHz)"]
        TIM_DRV["Timer Base & Encoder Mode Drivers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9)"]
        I2C_DRV["I2C1 Master + DMA1 Stream0 (Non-blocking Memory Read)"]
        UART_DRV["UART1 Polling / Non-blocking Driver + USB CDC Virtual COM"]
        FLS_DRV["HAL Flash Program / Erase Drivers (Voltage Range 3, 32-bit word)"]
    end

    subgraph REALTIME["TẦNG THỜI GIAN THỰC (TIM9 ISR - 500 Hz / 2 ms)"]
        ENC_TICK["Motor_Speed_Tick: Đọc encoder TIM2/TIM3, tính delta xung & vận tốc (mm/s)"]
        IR_TICK["IR_Tick: Lọc trung vị 3 mẫu + LPF bậc 1, bảng nội suy quy đổi FL->FR, L->R"]
        MOT_LOOP["Motion_Update: Profile căn bậc hai (v, w), PD giữ hướng gyro, cân giữa ô"]
        PID_LOOP["Motor_Speed_Control: Ramp gia tốc 2000 mm/s², Feedforward (Ks,Kf,Ka) + PI bánh"]
        PWM_OUT["Motor_SetPWM: Xuất CCR Drive/Brake 20kHz cho TIM1 và TIM5"]
        IMU_TICK["MPU6500_Tick: Kích hoạt I2C DMA đọc 14 byte từ MPU6500"]
        IMU_DMA_CB["HAL_I2C_MemRxCpltCallback: Tích phân góc Yaw, Lọc bù Pitch/Roll, Bám bias"]
        SAFETY_TICK["Check_Robot_Picked_Up: Giám sát nhấc xe, quá tốc độ encoder, va chạm Jerk"]
    end

    subgraph MOTION["TẦNG ĐIỀU KHIỂN CHUYỂN ĐỘNG (pid.c / pid.h)"]
        MOVE_API["API Chuyển động: Move_Straight, Move_Straight_Begin, Turn_Angle, Turn_Angle_Begin"]
        ALIGN_API["API Căn tường: Wall_Align, Wall_Align_Begin, Wall_Align_Poll"]
        CURVE_API["API Cua mượt 90°: Smooth_Turn_Left_90, Smooth_Turn_Right_90 (sin² profile)"]
    end

    subgraph ALGORITHM["TẦNG THUẬT TOÁN MÊ CUNG (solving_maze.c / solving_maze.h)"]
        MAP_RAM["Bản đồ mê cung RAM: maze_map[16][16], bit tường + bit đã biết"]
        FLOOD_FILL["Thuật toán Flood-Fill (BFS đa nguồn, hàng đợi tĩnh 256 phần tử)"]
        MERGE_CELLS["Gộp ô đi thẳng liên tục (Plan_StraightCells) & Điều chỉnh đích khi đang chạy"]
        MAZE_SM["Máy trạng thái giải mê cung: DECIDE_PATH, MOVE_STRAIGHT, TURN, ALIGN, PAUSED"]
    end

    subgraph SAFETY_MGMT["TẦNG QUẢN LÝ CHẾ ĐỘ & AN TOÀN (mode_manager.c / mode_manager.h)"]
        VIRT_BTN["Nút bấm ảo IR trái: Đếm số lần che (1=Mode 1, 2=Mode 2, 3=Mode 3)"]
        BUZ_PAT["Trình phát mẫu còi không chặn (HAL_GetTick based Buzzer Pattern Player)"]
        MM_SM["Máy trạng thái chính: IDLE, UI_BEEP, UI_WINDOW, START, EXPLORE, MODE2, PAUSED, ERROR"]
        PAUSE_RESUME["Giám sát đứng yên 3s & Khôi phục trạng thái di chuyển sau khi đặt xe xuống"]
    end

    subgraph STORAGE["TẦNG LƯU TRỮ DỮ LIỆU (flash_storage.c / flash_storage.h)"]
        FLASH_LOG["Cơ chế Append-Log Wear Leveling (468 slots, CRC-32, Magic 0xA5A5B6B6)"]
        FLASH_IO["Đọc bản ghi mới nhất (Flash_LoadMap) & Ghi bản ghi mới (Flash_SaveMap)"]
    end

    subgraph TELEMETRY["TẦNG GIÁM SÁT & MÁY TÍNH (maze_viewer.py)"]
        MAZE_TX["Maze_SendMap: Gửi khối văn bản ASCII (#MAZE_BEGIN, #ROW, #MAZE_END checksum)"]
        HOST_GUI["maze_viewer.py: Giao diện Tkinter, kết nối Bluetooth COM8, vẽ mê cung thời gian thực"]
    end

    %% Hardware Connections
    IR_HW --> ADC_DRV
    ENC_HW --> TIM_DRV
    IMU_HW --> I2C_DRV
    MOT_HW <-- TIM_DRV
    BUZ_HW <-- DRIVERS
    COM_HW <--> UART_DRV
    FLS_HW <--> FLS_DRV

    %% Drivers to Realtime
    ADC_DRV --> IR_TICK
    TIM_DRV --> ENC_TICK
    I2C_DRV --> IMU_DMA_CB
    PWM_OUT --> TIM_DRV

    %% Realtime Interconnections
    ENC_TICK --> PID_LOOP
    IR_TICK --> MOT_LOOP
    MOT_LOOP --> PID_LOOP
    PID_LOOP --> PWM_OUT
    IMU_TICK --> I2C_DRV
    IMU_DMA_CB --> MOT_LOOP
    IMU_DMA_CB --> SAFETY_TICK

    %% Realtime to Motion & Algorithm
    REALTIME <--> MOTION
    MOTION <--> ALGORITHM

    %% Algorithm to Safety & Storage
    ALGORITHM <--> SAFETY_MGMT
    SAFETY_MGMT <--> STORAGE
    STORAGE <--> FLS_DRV

    %% Telemetry
    ALGORITHM --> MAZE_TX
    MAZE_TX --> UART_DRV
    UART_DRV <--> HOST_GUI
```

---

## 2. Phân bổ tài nguyên phần cứng (Hardware Resource Allocation)

### 2.1 Cấu hình xung nhịp (Clock Tree)
- **Thạch anh ngoài (HSE):** 25.0 MHz.
- **Bộ nhân tần PLL:**
  - $PLLM = 15$, $PLLN = 144$, $PLLP = 4$, $PLLQ = 5$.
  - Tần số VCO: $25 \times 144 / 15 = 240 \text{ MHz}$.
  - **SYSCLK (CPU Clock):** $240 / 4 = 60 \text{ MHz}$.
  - **AHB Bus (HCLK):** 60 MHz (Bộ chia 1).
  - **APB1 Bus (PCLK1):** 30 MHz (Bộ chia 2). Tần số cấp cho Timer APB1: 60 MHz.
  - **APB2 Bus (PCLK2):** 60 MHz (Bộ chia 1). Tần số cấp cho Timer APB2: 60 MHz.
  - **USB OTG FS Clock:** $240 / 5 = 48 \text{ MHz}$ (Đạt chuẩn 48 MHz cho USB Full-Speed).

---

### 2.2 Bảng phân bổ ngoại vi, chân vi điều khiển và DMA

| Chức năng | Ngoại vi | Chân vi điều khiển | Cấu hình & Thông số hoạt động | Ghi chú kiến trúc |
| :--- | :--- | :--- | :--- | :--- |
| **Cảm biến IR trước-trái (FL)** | ADC1 IN4 | PA4 | ADC 12-bit, Scan mode, Right align | Quét vòng tròn liên tục bằng DMA |
| **Cảm biến IR trái (L)** | ADC1 IN5 | PA5 | ADC 12-bit, kích khởi bởi TIM4 CC4 | Mắt nhận dùng làm nút bấm ảo |
| **Cảm biến IR phải (R)** | ADC1 IN6 | PA6 | ADC 12-bit | Đo khoảng cách vách bên phải |
| **Cảm biến IR trước-phải (FR)** | ADC1 IN7 | PA7 | ADC 12-bit | Làm thang đo chuẩn cho mắt trước |
| **Kênh DMA cho ADC** | DMA2 Stream0 | — | Channel 0, Circular mode, Half-word (16-bit) | Tự động ghi vào mảng `adc_value[4]` |
| **Timer kích ADC IR** | TIM4 CH4 | — | Chu kỳ ~5 kHz (ARR = 11999, PSC = 0), Không xuất pin | Tránh phụ thuộc CPU khi lấy mẫu ADC |
| **LED phát hồng ngoại** | GPIO Output | PA8, PA9, PA10 | Mức cao (Push-Pull, Low speed) | Bật liên tục sau khi khởi động |
| **Động cơ trái (IN1 / IN2)** | TIM1 CH2N / CH3N | PB0 / PB1 | PWM 20 kHz (ARR = 2999, PSC = 0), Complementary non-inverted | Cầu H DRV8833, chế độ Drive/Brake |
| **Động cơ phải (IN1 / IN2)** | TIM5 CH4 / CH3 | PA3 / PA2 | PWM 20 kHz (ARR = 2999, PSC = 0), Standard PWM | Cầu H DRV8833, chế độ Drive/Brake |
| **Encoder bánh trái** | TIM3 | PB4 (TI1), PB5 (TI2) | Chế độ Encoder TI1, 700 xung/vòng bánh | Đọc thanh ghi `TIM3->CNT` trực tiếp |
| **Encoder bánh phải** | TIM2 | PA0 (TI1), PA1 (TI2) | Chế độ Encoder TI1, 700 xung/vòng bánh | Đọc thanh ghi `TIM2->CNT` trực tiếp |
| **Cảm biến IMU MPU6500** | I2C1 | PB6 (SCL), PB7 (SDA) | Fast Mode 300 kHz, Địa chỉ 7-bit 0x68 (`0xD0`) | Giao tiếp DMA đọc 14 bytes/tick |
| **Kênh DMA nhận I2C** | DMA1 Stream0 | — | Channel 1, Normal mode, Byte (8-bit) | Nhận dữ liệu không chặn về `imu_rx_buffer` |
| **Nhịp điều khiển thời gian thực** | TIM9 | — | Ngắt tràn Update IRQ ~500 Hz (2.0 ms, ARR = 59999, PSC = 1) | Khối điều phối thời gian thực chính |
| **Còi báo (Buzzer)** | GPIO Output | PB8 | Push-Pull, Low speed, Active-High (1 = kêu) | Trình phát âm thanh phi khóa (non-blocking) |
| **LED trạng thái IMU** | GPIO Output | PB14 | Push-Pull, Sáng khi MPU6500_Setup thành công | Đèn LED báo phần cứng IMU |
| **LED trạng thái hoạt động** | GPIO Output | PB15 | Push-Pull, Sáng khi xe trên sàn, tắt khi bị nhấc | Đèn báo sẵn sàng chuyển động |
| **Cổng Bluetooth HC-05** | USART1 | PA15 (TX), PB3 (RX) | 9600 baud, 8N1, Không DMA/IRQ | Gửi bản đồ lên PC và nhận lệnh |
| **Cổng USB Virtual COM** | USB OTG FS | PA11 (DM), PA12 (DP) | CDC Class Virtual COM Port | In báo cáo kiểm thử và gỡ lỗi |
| **Bộ nhớ Flash bản đồ** | Flash Sector 7 | 0x08060000 | Phân vùng 128 KB độc lập với mã chương trình | Lưu trữ bản đồ mê cung bền vững |

---

### 2.3 Phân cấp ưu tiên ngắt NVIC (Interrupt Priorities)

Toàn bộ hệ thống áp dụng chiến lược quản lý ngắt như sau:
- **Nhóm ưu tiên ngắt:** `NVIC_PRIORITYGROUP_4` (4 bit Preemption Priority, 0 bit Sub-priority).
- Mọi ngắt ngoại vi thời gian thực (`TIM1_BRK_TIM9_IRQn`, `DMA1_Stream0_IRQn`, `DMA2_Stream0_IRQn`, `I2C1_EV_IRQn`, `I2C1_ER_IRQn`) đều được gán mức ưu tiên cao nhất (`PreemptPriority = 0`).
- Ngắt định thời hệ thống **SysTick** được đặt ở mức ưu tiên thấp nhất (`PreemptPriority = 15`).
- **Nguyên tắc an toàn:** Không bao giờ được gọi các hàm dựa trên `HAL_GetTick()` hoặc `HAL_Delay()` bên trong các hàm phục vụ ngắt (ISR) hoặc các hàm callback của HAL, vì `SysTick` có độ ưu tiên thấp hơn và sẽ bị chiếm quyền, dẫn đến treo cứng hệ thống (Deadlock).

---

## 3. Trách nhiệm chi tiết của các module mã nguồn

### 3.1 Module Khởi tạo & Điều phối chính (`Core/Src/main.c`, `Core/Inc/main.h`)
- **Trách nhiệm:**
  - Cấu hình xung nhịp hệ thống, khởi tạo vi điều khiển và các ngoại vi qua STM32CubeMX HAL.
  - Thiết lập chuỗi khởi động phần cứng an toàn: `ADC_IR_Start()` -> `Motor_Init()` -> `HAL_TIM_Base_Start_IT(&htim9)` -> `ADC_IR_EmittersOn()` -> `MPU6500_Setup()` -> `ModeManager_Init()`.
  - Triển khai hàm ngắt thời gian thực `HAL_TIM_PeriodElapsedCallback(htim9)` gọi lần lượt các hàm chu kỳ 2 ms.
  - Cung cấp các hàm truyền thông an toàn không gây treo hệ thống: `CDC_Print()`, `CDC_WaitHost()`, `UART_Print()`, `UART_TryRead()`.
  - Vòng lặp `while(1)` chỉ phục vụ việc chạy máy trạng thái `ModeManager_Update()` hoặc các hàm gỡ lỗi độc lập.

---

### 3.2 Module Quản lý Chế độ & Hệ thống An toàn (`Core/Src/mode_manager.c`, `Core/Inc/mode_manager.h`)
- **Trách nhiệm:**
  - Quản lý toàn bộ vòng đời hoạt động của robot thông qua máy trạng thái cấp cao `RobotState`.
  - Triển khai giao diện người dùng không cần nút bấm vật lý: nhận diện cử chỉ che mắt cảm biến IR trái (`UI_Pressed`) với cơ chế chống rung số (debounce 30 ms) và cửa sổ chốt lệnh 3 giây (`UI_WINDOW_MS`).
  - Điều khiển còi buzzer phát âm báo không chặn (Non-blocking pattern player) dựa trên mốc thời gian `HAL_GetTick()`.
  - Phân định và thực thi 3 chế độ hoạt động:
    - **Mode 1:** Tải bản đồ từ Flash Sector 7, tiếp tục khám phá các vùng mê cung chưa biết.
    - **Mode 2:** Chạy bứt tốc theo lộ trình ngắn nhất tối ưu từ bản đồ Flash, tới đích reo còi, quay 180° và chạy về ô xuất phát.
    - **Mode 3:** Xóa trắng bản đồ RAM, ghi đè bản ghi mới vào Flash và khám phá mê cung từ đầu.
  - Giám sát an toàn đa tầng trong ngắt TIM9 (`Check_Robot_Picked_Up`): Phát hiện nhấc xe qua IMU, phát hiện mất tải bánh xe qua Encoder Overspeed, phát hiện mất tường đột ngột qua IR, và phát hiện va chạm qua Jerk.
  - Tự động chuyển sang trạng thái `STATE_PAUSED` khi robot bị nhấc hoặc va chạm, phanh động cơ, lưu trạng thái dở dang vào Flash, và tự động khôi phục (`STATE_RESUME`) khi robot được đặt lại yên tĩnh trên sàn đủ 3 giây (`PAUSE_STILL_MS`).

---

### 3.3 Module Điều khiển Động cơ & Chuyển động (`Core/Src/pid.c`, `Core/Inc/pid.h`)
- **Trách nhiệm:**
  - Trình điều khiển trực tiếp cầu H (DRV8833): điều chế PWM đẩy/phanh (Drive/Brake) tần số 20 kHz thông qua các thanh ghi Timer `TIM1->CCR2/CCR3` và `TIM5->CCR4/CCR3`.
  - Xử lý phản hồi Encoder bánh xe: tính toán delta xung an toàn chống tràn số 16-bit, quy đổi chính xác ra mm/s và mm dựa trên đường kính bánh xe thực tế (`WHEEL_DIAMETER_L/R`).
  - Tích hợp mô hình tính toán định vị nội suy Odometry Dead-Reckoning ($X, Y, Heading$).
  - Vòng điều khiển PID vận tốc từng bánh (Inner Loop): thuật toán PID kết hợp Feedforward 3 thành phần ($Ks$ bù ma sát tĩnh, $Kf$ bù sức điện động, $Ka$ bù quán tính gia tốc) và bộ tích phân chống bão hòa (Anti-windup).
  - Vòng điều khiển chuyển động ngoài (Outer Motion Loop):
    - Đi thẳng (`MOTION_STRAIGHT`): Profile giảm tốc căn bậc hai, bộ điều khiển PD giữ hướng theo góc con quay Gyro ($Kp = 20.0, Kd = 0.5$), cơ chế cân giữa hành lang ô (`Wall_CenterOffset`), cơ chế tránh tường bên và dừng trước tường trước.
    - Kiểm soát trần vận tốc không giới hạn (`Straight_SpeedCeiling`): tự động điều chỉnh tốc độ tối đa theo khả năng cấp dòng của pin mà không làm mất khoảng bù lái (PWM Headroom).
    - Quay tại chỗ (`MOTION_TURN`): Profile căn bậc hai góc quay với vận tốc góc tối đa 300 °/s.
    - Đứng giữ hướng tuyệt đối (`MOTION_HOLD`): Tự động quay về hướng đích khi bị tác động ngoại lực làm lệch.
    - Tự động căn vuông góc với tường trước (`Wall_Align`): Tự xoay đo độ nhạy $g$, quay chỉnh triệt tiêu sai lệch $r = (FL-FR)/(FL+FR)$, đo lại offset con quay và gán lại mốc hướng MPU.
    - Cua mượt 90 độ (`Smooth_Turn_Left_90` / `Right_90`): Quỹ đạo độ cong liên tục $\kappa(s) = \frac{\pi}{L} \sin^2\left(\frac{\pi s}{L}\right)$ không gián đoạn gia tốc bánh.

---

### 3.4 Module Cảm biến Quán tính IMU (`Core/Src/mpu6500.c`, `Core/Inc/mpu6500.h`)
- **Trách nhiệm:**
  - Khởi tạo và cấu hình phần cứng MPU6500 qua I2C1 DMA: Dải đo Gyro $\pm 2000 \text{ ^\circ/s}$, Dải đo Accel $\pm 8\text{ g}$, Bộ lọc số phần cứng DLPF Gyro 41 Hz và Accel 20 Hz.
  - Hiệu chỉnh tĩnh lúc khởi động (`MPU6500_Calibrate`): Lấy trung bình 500 mẫu loại bỏ offset ban đầu với cơ chế kiểm tra độ trôi góc và giới hạn thời gian an toàn 3 giây.
  - Đọc bất đồng bộ DMA 14 byte dữ liệu mỗi chu kỳ 2 ms từ ngắt TIM9.
  - Tính toán góc tư thế: Lọc bù (Complementary filter) góc Pitch/Roll với hằng số thời gian $\tau = 0.5\text{ s}$; tích phân liên tục góc Yaw không vùng chết (no deadband).
  - Tự động nhận diện trạng thái đứng yên (`imu_still`) và hiệu chỉnh trôi điểm không động (Dynamic Gyro Bias Tracking) với hằng số thời gian 2 giây.
  - Hệ thống phát hiện nhấc xe đa điều kiện (`Lift_Update`): Giám sát góc nghiêng $> 20^\circ$, vận tốc tịnh tiến đứng $|V_z| > 0.12\text{ m/s}$ và tốc độ lật góc $> 30\text{ ^\circ/s}$ duy trì liên tục 200 ms.
  - Hệ thống phát hiện va chạm dựa trên độ giật (`Crash_Update`): Tính toán độ giật mặt phẳng ngang (Planar Jerk) qua 10 ms kết hợp bộ lọc trung vị 3 mẫu để phát hiện đâm trực diện ($> 1.10\text{ g}$) hoặc quệt góc tường ($> 0.73\text{ g}$ kèm giật Gyro Z $> 76\text{ ^\circ/s}$).

---

### 3.5 Module Cảm biến Khoảng cách Hồng ngoại (`Core/Src/adc.c`, `Core/Inc/adc.h`)
- **Trách nhiệm:**
  - Quản lý quá trình chuyển đổi ADC1 quét 4 kênh bằng DMA2 Stream0 theo trigger chu kỳ ~5 kHz từ TIM4 CC4.
  - Bật các LED phát hồng ngoại công suất cao trên các chân GPIO PA8, PA9, PA10.
  - Xử lý tín hiệu số quang học: Lọc trung vị 3 mẫu (Median-of-3) loại bỏ xung nhiễu nhọn đột biến kết hợp bộ lọc thông thấp IIR bậc 1 (`IR_FILT_K = 0.3`).
  - Tuyến tính hóa và cân bằng độ nhạy quang học: Áp dụng bảng nội suy tuyến tính từng đoạn (Piecewise Linear Interpolation) đưa mắt trước-trái (FL) về thang đo chuẩn của mắt trước-phải (FR) và mắt trái (L) về thang đo chuẩn của mắt phải (R).
  - Phát hiện ngưỡng bão hòa của phototransistor (`IR_FL_SAT = 3800`).

---

### 3.6 Module Thuật toán Giải mê cung (`Core/Src/solving_maze.c`, `Core/Inc/solving_maze.h`)
- **Trách nhiệm:**
  - Quản lý cấu trúc dữ liệu bản đồ mê cung 16x16 ô (`maze_map[16][16]`): lưu trữ trạng thái có tường và trạng thái đã khám phá trên 4 hướng Bắc, Đông, Nam, Tây theo từng cạnh đối ngẫu.
  - Triển khai thuật toán Flood-Fill (Breadth-First Search đa nguồn): Sử dụng hàng đợi tĩnh (Static Queue) không đệ quy, tính toán ma trận khoảng cách `maze_dist[16][16]`.
  - Hỗ trợ hai chế độ tìm đường: Tìm kiếm lạc quan (Optimistic - coi ô chưa biết là thông thoáng) khi đang khám phá và tìm kiếm chặt chẽ (Known-only) khi chạy nhanh.
  - Kỹ thuật di chuyển liên tục (Continuous Motion): Thuật toán gộp các ô đi thẳng (`Plan_StraightCells`), chỉ phanh tại tâm ô cần đổi hướng; kích hoạt đọc cảm biến sớm trước tâm ô 90 mm (`MAZE_SENSE_BEFORE_MM`) để cập nhật bản đồ và mở rộng/thu hẹp lộ trình di chuyển ngay trong khi xe đang chạy ở tốc độ cao.
  - Tách biệt hoàn toàn lớp thuật toán thuần túy với lớp phần cứng thông qua các hàm trừu tượng hóa `HW_*`.
  - Định dạng và xuất dữ liệu bản đồ dạng khối ASCII có checksum để truyền lên máy tính qua cổng nối tiếp.

---

### 3.7 Module Lưu trữ Flash Nội (`Core/Src/flash_storage.c`, `Core/Inc/flash_storage.h`)
- **Trách nhiệm:**
  - Quản lý vùng nhớ Flash chuyên dụng tại Sector 7 (0x08060000 - 0x0807FFFF, kích thước 128 KB).
  - Định nghĩa cấu trúc bản ghi chuẩn hóa `FlashMapData` (kích thước 280 bytes, căn chỉnh word 32-bit).
  - Triển khai cơ chế ghi nhật ký nối tiếp phân phối hao mòn (Append-Log Wear Leveling): Sector 7 được chia thành 468 slots. Mỗi lần lưu chỉ ghi vào slot trống kế tiếp, giúp tăng tuổi thọ chip lên hàng trăm nghìn lần chạy và chỉ xóa sector khi đã đầy.
  - Bảo vệ chống mất nguồn đột ngột (Power-loss Resilience): Quét ngược từ slot trống đầu tiên để tìm bản ghi hợp lệ có mã kiểm tra CRC-32 chuẩn xác; bỏ qua các bản ghi bị ghi dở dang.
  - Đồng bộ bộ nhớ đệm lệnh và dữ liệu vi điều khiển (`Flash_FlushDataCache`) sau khi ghi/xóa để đảm bảo tính nhất quán dữ liệu đọc.

---

### 3.8 Ứng dụng Giám sát Máy tính (`maze/maze_viewer.py`)
- **Trách nhiệm:**
  - Giao diện đồ họa máy tính viết bằng Python 3 (Tkinter) kết nối với robot qua cổng nối tiếp Bluetooth HC-05 (mặc định COM8, 9600 baud).
  - Xử lý giao tiếp nối tiếp đa luồng (`SerialWorker` thread): Tách biệt hoàn toàn việc đọc/ghi cổng COM khỏi luồng giao diện người dùng, ngăn ngừa hiện tượng treo cửa sổ khi mất kết nối Bluetooth.
  - Tự động nhận diện gói tin `#MAZE_BEGIN ... #ROW ... #MAZE_END`, kiểm tra checksum 16-bit, tự động gửi ký tự 'M' yêu cầu truyền lại khi gặp lỗi.
  - Trực quan hóa bản đồ mê cung thời gian thực: hiển thị vách tường đã biết, vách chưa biết, vị trí và hướng xe, lộ trình tối ưu BFS, và lưu trữ log bản đồ vào thư mục `maze/logs/`.
