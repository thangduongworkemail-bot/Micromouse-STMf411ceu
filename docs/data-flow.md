# ĐẶC TẢ CHI TIẾT CÁC LUỒNG DỮ LIỆU (DATA FLOW SPECIFICATION)

Tài liệu này truy vết toàn diện và chi tiết đường đi của dữ liệu xuyên suốt hệ thống: từ tín hiệu phần cứng ngõ vào, qua các khâu xử lý/lọc số, biến đổi toán học trung gian, đến tác động điều khiển phần cứng ngõ ra hoặc lưu trữ lâu dài.

---

## 1. Tổng quan các luồng dữ liệu chính

Hệ thống bao gồm 6 luồng dữ liệu cốt lõi hoạt động song song và trao đổi dữ liệu với nhau qua bộ nhớ RAM dùng chung:

```mermaid
flowchart LR
    subgraph SENSING["1. CẢM BIẾN & THU THẬP"]
        DF1_IN["Quang điện IR (4 kênh)"]
        DF2_IN["MEMS IMU (MPU6500)"]
        DF3_IN["Xung Encoder (TIM2/TIM3)"]
    end

    subgraph REALTIME_PROC["2. XỬ LÝ SỐ THỜI GIAN THỰC (TIM9 2ms)"]
        DF1_PROC["Lọc trung vị + LPF + Nội suy quang học (IR_Tick)"]
        DF2_PROC["Lọc bù Pitch/Roll + Tích phân Yaw + Bám bias (MPU6500_Process_Data)"]
        DF3_PROC["Tính delta xung + Vận tốc (mm/s) + Odometry (Motor_Speed_Tick)"]
        DF_SAFETY["Giám sát an toàn: Nhấc xe / Va chạm / Quá tốc bánh (Check_Robot_Picked_Up)"]
    end

    subgraph MOTION_ALGO["3. ĐIỀU KHIỂN & THUẬT TOÁN"]
        DF_MOT["Profile căn bậc hai + PD giữ hướng gyro + Cân giữa ô (Motion_Update)"]
        DF_PID["Feedforward (Ks,Kf,Ka) + PI bánh xe (Motor_Speed_Control)"]
        DF_MAZE["Nhận diện vách tường + BFS Flood-Fill + Gộp ô thẳng (solving_maze.c)"]
        DF_MM["Máy trạng thái quản lý chế độ (mode_manager.c)"]
    end

    subgraph ACTUATION_STORAGE["4. CHẤP HÀNH & LƯU TRỮ"]
        DF_PWM["Thanh ghi CCR TIM1/TIM5 (PWM 20kHz Cầu H)"]
        DF_FLASH["Flash On-Chip Sector 7 (FlashMapData CRC32)"]
        DF_TELE["Gói tin ASCII #MAZE_BEGIN qua UART1 Bluetooth HC-05"]
    end

    DF1_IN --> DF1_PROC
    DF2_IN --> DF2_PROC
    DF3_IN --> DF3_PROC

    DF1_PROC --> DF_SAFETY
    DF2_PROC --> DF_SAFETY
    DF3_PROC --> DF_SAFETY

    DF1_PROC --> DF_MOT
    DF2_PROC --> DF_MOT
    DF3_PROC --> DF_PID

    DF1_PROC --> DF_MAZE
    DF3_PROC --> DF_MAZE
    DF_MAZE --> DF_MOT

    DF_MOT --> DF_PID
    DF_PID --> DF_PWM

    DF_SAFETY --> DF_MM
    DF_MM --> DF_FLASH
    DF_MAZE --> DF_FLASH
    DF_MAZE --> DF_TELE
```

---

## 2. Truy vết chi tiết từng luồng dữ liệu

---

### Luồng 1: Dữ liệu Cảm biến Hồng ngoại (IR Optical Wall Sensing & Filtering)

Luồng dữ liệu này chịu trách nhiệm chuyển đổi điện áp tương tự từ 4 bộ thu hồng ngoại thành thông tin nhận biết sự hiện diện của tường, đo độ lệch tâm trong hành lang ô và phục vụ thuật toán tự căn vuông góc.

```mermaid
flowchart TD
    A["Phototransistor (PA4..PA7)"] -->|"Tín hiệu điện áp analog (0 - 3.3V)"| B["ADC1 12-bit (Kích bởi TIM4 CC4 ~5 kHz)"]
    B -->|"DMA2 Stream0 chép vòng tròn"| C["Bộ đệm RAM thô: volatile adc_value[4]"]
    C -->|"Mỗi 2ms: IR_Tick() trong ngắt TIM9"| D["Bộ lọc trung vị 3 mẫu: Median3()"]
    D -->|"Loại bỏ xung gai & mẫu rớt 0"| E["Bộ lọc thông thấp IIR: y += 0.3 * (m - y)"]
    E -->|"Tín hiệu đã lọc ir_filt[4]"| F{"Khâu quy đổi cân bằng quang học"}
    
    F -->|"Kênh FL (ir_filt[0])"| G["IR_FL_ToRef(): Bảng nội suy 21 điểm ir_fl_raw -> ir_fl_ref"]
    F -->|"Kênh L (ir_filt[1])"| H["IR_L_ToRef(): Bảng nội suy 13 điểm ir_l_raw -> ir_l_ref"]
    F -->|"Kênh FR (ir_filt[3])"| I["IR_FR_Filt(): Giữ nguyên thang đo chuẩn"]
    F -->|"Kênh R (ir_filt[2])"| J["IR_R_Filt(): Giữ nguyên thang đo chuẩn"]

    G --> K["ĐẦU RA CHUẨN HÓA CẢM BIẾN IR"]
    H --> K
    I --> K
    J --> K

    K -->|"max(FL, FR) >= 1700"| L["Dừng trước tường: MOTION_WALL / Wall_Align()"]
    K -->|"e = 1 - sqrt(240/L) hoặc sqrt(280/R) - 1"| M["Cân giữa ô: Wall_CenterOffset() lái bù tối đa ±5°"]
    K -->|"Mắt bên >= 170"| N["Giải mê cung: HW_LeftWall() / HW_RightWall() cập nhật bản đồ"]
    K -->|"TB 2 mắt trước > 120"| O["Giải mê cung: HW_FrontWallFar() phát hiện tường trước"]
    K -->|"adc_value[1] > 2000 duy trì 30ms"| P["Nút bấm ảo IR trái: UI_Pressed() chọn chế độ"]
```

#### Chi tiết các bước chuyển hóa dữ liệu:
1. **Nguồn dữ liệu:** Bộ thu hồng ngoại quang điện (Phototransistors) trên các chân PA4 (FL), PA5 (L), PA6 (R), PA7 (FR).
2. **Khâu thu thập (Hardware sampling):**
   - Bộ định thời `TIM4` phát xung kích hoạt (trigger) tần số ~5 kHz tại kênh CC4.
   - Bộ chuyển đổi tương tự - số `ADC1` lấy mẫu tuần tự 4 kênh với độ phân giải 12-bit (giá trị số $0 \dots 4095$).
   - Bộ điều khiển truy cập bộ nhớ trực tiếp `DMA2_Stream0` tự động truyền kết quả vào mảng nhớ `volatile uint16_t adc_value[4]` trong RAM mà không tiêu tốn chu kỳ CPU.
3. **Khâu lọc nhiễu số (Digital Signal Filtering - `IR_Tick`):**
   - Được gọi mỗi 2 ms từ ngắt `TIM9`.
   - **Lọc trung vị 3 mẫu (`Median3`):** Duy trì cửa sổ trượt 3 mẫu gần nhất `ir_hist[c][3]`. Lấy giá trị nằm giữa giúp loại bỏ hoàn toàn các xung nhiễu nhọn từ tia lửa điện hoặc mẫu bị tụt áp tức thời.
   - **Lọc thông thấp IIR bậc 1:** 
     $$y_k = y_{k-1} + K \cdot (m_k - y_{k-1}) \quad (\text{với } K = \text{IR\_FILT\_K} = 0.3)$$
     Hằng số thời gian trễ tương đương ~6-8 ms, giúp triệt tiêu nhiễu dao động cao tần (noise reduction ~65%).
4. **Khâu chuẩn hóa và cân bằng quang học (Optical Equalization):**
   - Do sai số chế tạo linh kiện, kênh FL nhạy hơn FR gấp 2.4 - 2.8 lần và bị nén/bão hòa sớm tại `IR_FL_SAT = 3800`.
   - Kênh FR được chọn làm thang đo chuẩn tham chiếu cho phía trước; Kênh R làm thang đo chuẩn tham chiếu cho vách bên.
   - `IR_FL_ToRef(raw)` tra bảng 21 nút và nội suy tuyến tính từng đoạn (`IR_Interp`) để ánh xạ tín hiệu FL về tương đương với FR ở cùng cự ly phản xạ.
   - `IR_L_ToRef(raw)` tra bảng 13 nút ánh xạ tín hiệu L về tương đương với R.
5. **Đích nhận và sử dụng dữ liệu:**
   - **Điều khiển chuyển động (`pid.c`):** Đọc qua các getter `IR_FL_Filt()`, `IR_FR_Filt()`, `IR_L_Filt()`, `IR_R_Filt()`. Phục vụ dừng khẩn cấp trước tường (`WALL_TARGET_IR = 1700`), tránh tường bên (`WALL_SIDE_IR = 1200`) và cân giữa ô (`WALL_CENTER_L_IR = 240, WALL_CENTER_R_IR = 280`).
   - **Thuật toán mê cung (`solving_maze.c`):** Đọc qua `HW_LeftWall()`, `HW_RightWall()`, `HW_FrontWallFar()`, `HW_FrontWallNear()` để ghi nhận vách tường vào ma trận `maze_map[][]`.
   - **Giao diện người dùng (`mode_manager.c`):** Đọc trực tiếp giá trị thô `adc_value[1]` để làm nút bấm ảo che tay.

---

### Luồng 2: Dữ liệu Cảm biến Quán tính IMU (MPU6500 Gyro & Accel Processing)

Luồng dữ liệu này đo đạc gia tốc 3 trục và tốc độ góc 3 trục, tính toán góc tư thế (Pitch/Roll) và góc quay hướng (Yaw), phục vụ vòng điều khiển ổn định hướng và các thuật toán an toàn.

```mermaid
flowchart TD
    A["MPU6500 Silicon Die (I2C 0x68)"] -->|"14 bytes thanh ghi 0x3B..0x48"| B["Kích hoạt I2C1 Read DMA (MPU6500_Tick mỗi 2ms)"]
    B -->|"DMA1 Stream0 hoàn tất chuyển"| C["Ngắt HAL_I2C_MemRxCpltCallback"]
    C -->|"dt = imu_ticks * 0.002s"| D["Hàm xử lý dữ liệu: MPU6500_Process_Data(dt)"]
    
    D --> E["1. Ghép byte thô: imu_raw[7] (ax, ay, az, temp, gx, gy, gz)"]
    E --> F["2. Chuyển đổi đơn vị vật lý: Accel (g), Gyro (°/s), Temp (°C)"]
    
    F --> G["3. Lọc bù góc Pitch / Roll: tau = 0.5s"]
    F --> H["4. Kiểm tra điều kiện đứng yên (imu_still)"]
    F --> I["5. Tích phân góc hướng Yaw: yaw_angle += gz * dt (YAW_SIGN=-1)"]
    F --> J["6. Giám sát an toàn: Lift_Update() & Crash_Update()"]

    H -->|"Đứng yên (v<1mm/s, gz_dev<0.2°/s)"| K["Đóng băng Yaw, Tự bám trôi Gyro Z offset (tau=2s)"]
    H -->|"Đang di chuyển"| L["Tích phân toàn phần không vùng chết (no deadband)"]

    J -->|"Nghiêng >20° / Vz >0.12m/s / Lật >30°/s trong 200ms"| M["robot_lifted = 1 (IMU_STATE_PICKED_UP)"]
    J -->|"Độ giật Jerk > 1.10g hoặc Side Jerk > 0.73g kèm Gz Jerk > 76°/s"| N["robot_lifted = 1 (IMU_STATE_CRASHED)"]

    M --> O["Khóa động cơ Motor_SetPWM(0,0), Tắt PB15, Hủy chuyển động"]
    N --> O
    I --> P["Robot_Heading(): Cung cấp góc hướng cho PD điều khiển đi thẳng"]
```

#### Chi tiết các bước chuyển hóa dữ liệu:
1. **Nguồn dữ liệu:** Cảm biến MPU6500 trên bus I2C1 (tần số 300 kHz).
2. **Kích hoạt đọc bất đồng bộ:**
   - Trong ngắt `TIM9` mỗi 2 ms, hàm `MPU6500_Tick()` gọi `HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6500_ADDR, 0x3B, ..., 14)`.
   - Vi điều khiển không bị chiếm dụng CPU trong quá trình nhận 14 byte qua I2C.
3. **Tiếp nhận ngắt DMA (`HAL_I2C_MemRxCpltCallback`):**
   - Đo thời gian thực tế `dt = imu_ticks * CONTROL_DT` (phòng ngừa trường hợp bus bận bỏ lỡ tick).
   - Chuyển tiếp vào `MPU6500_Process_Data(dt)`.
4. **Biến đổi vật lý:**
   - Ghép byte Big-Endian: $\text{raw} = (\text{MSB} \ll 8) \mid \text{LSB}$.
   - Gia tốc: $a_x, a_y, a_z = \text{raw} / \text{ACCEL\_LSB\_PER\_G}$ (với dải $\pm 8\text{ g}$, hệ số chia là $4096.0$).
   - Vận tốc góc: $g_x, g_y, g_z = (\text{raw} - \text{offset}) / 16.4$ (Dải $\pm 2000 \text{ ^\circ/s}$).
   - Đảo chiều góc quay Z theo quy ước thực tế của khung xe: $\text{yaw\_rate\_dps} = \text{YAW\_SIGN} \times g_z$ (với $\text{YAW\_SIGN} = -1$).
5. **Cơ chế bám trôi điểm không động (Standstill Bias Tracking):**
   - Xe được coi là đứng yên (`imu_still = 1`) khi thỏa mãn đồng thời 3 điều kiện:
     1. Vận tốc cả 2 bánh đo từ encoder $< 1.0\text{ mm/s}$.
     2. Độ phân tán dao động của Gyro Z: $\text{gz\_dev} < 0.2 \text{ ^\circ/s}$.
     3. $|g_z| < 1.0 \text{ ^\circ/s}$ duy trì $0.25\text{ s}$ (hoặc $< 5.0 \text{ ^\circ/s}$ duy trì $2.0\text{ s}$).
   - Khi đứng yên: `yaw_angle` được giữ nguyên bất biến, và `gyro_z_offset` tự động cập nhật bám theo sai số tĩnh của phần cứng với bộ lọc LPF có hằng số thời gian $\tau = 2.0\text{ s}$.
   - Khi chuyển động: Toàn bộ tốc độ góc được tích phân liên tục vào `yaw_angle` mà không áp dụng bất kỳ ngưỡng chết (deadband) nào.
6. **Thuật toán phát hiện sự cố an toàn:**
   - **Nhấc xe (`Lift_Update`):** Đo vector trọng lực low-pass so với vector chuẩn trên sàn `lift_ref`. Nếu góc lệch nghiêng $> 20^\circ$, hoặc vận tốc tịnh tiến thẳng đứng $|V_z| > 0.12\text{ m/s}$, hoặc tốc độ lật góc ngoài mặt phẳng $> 30\text{ ^\circ/s}$ duy trì liên tục $200\text{ ms}$ (`LIFT_CONFIRM_S`), biến cờ `robot_lifted` lập tức được bật lên 1.
   - **Va chạm (`Crash_Update`):** Tính độ giật (Jerk - đạo hàm gia tốc) qua cửa sổ 5 mẫu (10 ms) sau khi lọc trung vị 3 mẫu. Nếu độ giật mặt phẳng ngang $> 1.10\text{ g}$ hoặc độ giật ngang $> 0.73\text{ g}$ kèm giật quay đầu xe $> 76\text{ ^\circ/s}$ duy trì 2 mẫu liên tiếp, kích hoạt `IMU_STATE_CRASHED`.

---

### Luồng 3: Dữ liệu Vòng điều khiển Vận tốc & Chuyển động Động cơ (Encoders & Motor PID Control)

Luồng dữ liệu này tạo thành vòng điều khiển phân tầng kín: đo phản hồi tốc độ thực từ bánh xe, đối chiếu với quỹ đạo đặt, tính toán bù sai số và xuất xung PWM điều khiển cầu H.

```mermaid
flowchart TD
    A["Encoders 700 xung/vòng (TIM2 PA0/PA1, TIM3 PB4/PB5)"] -->|"Đọc thanh ghi CNT mỗi 2ms: Motor_Speed_Tick()"| B["Tính delta xung an toàn: (int16_t)(uint16_t curr - prev)"]
    B -->|"Nhân MM_PER_COUNT_L/R (29.24 / 29.03 mm)"| C["Quãng đường & Vận tốc thô (mm/s)"]
    C -->|"Lọc LPF: speed += 0.3 * (raw - speed)"| D["Vận tốc thực tế phản hồi: speed_left_mms, speed_right_mms"]
    C -->|"Tích phân không lọc: dist += raw * dt"| E["Quãng đường tích lũy & Odometry (odo_x_mm, odo_y_mm)"]

    F["Lệnh chuyển động tầng ngoài: Move_Straight / Turn_Angle"] --> G["Vòng điều khiển chuyển động ngoài: Motion_Update()"]
    D --> G
    E --> G
    H["Robot_Heading() từ MPU6500"] --> G
    I["IR L/R Filt từ Cảm biến"] --> G

    G -->|"Profile căn bậc 2: v = min(v_max, sqrt(2*a*d_rem))"| J["Vận tốc tịnh tiến tâm xe v"]
    G -->|"PD hướng w = 20*err - 0.5*rate + Cân giữa ô"| K["Vận tốc góc w"]
    
    J --> L["target_speed_left = v - w*W/2"]
    K --> L
    J --> M["target_speed_right = v + w*W/2"]
    K --> M

    L --> N["Vòng điều khiển vận tốc từng bánh: Motor_Speed_Control()"]
    M --> N
    D --> N

    N -->|"Ramp giới hạn gia tốc 2000 mm/s²"| O["ref_speed_left, ref_speed_right"]
    O --> P["1. Feedforward: Ks*sign(ref) + Kf*ref + Ka*(dref/dt)"]
    O --> Q["2. Bộ điều khiển PI: Kp = 5.0, Ki = 150.0 (Anti-windup)"]

    P --> R["Tổng hợp đầu ra: pwm_left, pwm_right (Kẹp ±2999)"]
    Q --> R

    R --> S["Trình điều khiển PWM cầu H: Motor_SetPWM()"]
    S -->|"Drive / Brake Mode 20 kHz"| T["Thanh ghi phần cứng: TIM1->CCR2/CCR3 & TIM5->CCR4/CCR3"]
    T --> U["Cầu H DRV8833 -> Động cơ DC hai bánh xe"]
```

#### Chi tiết các bước chuyển hóa dữ liệu:
1. **Thu thập phản hồi Encoder:**
   - Mỗi chu kỳ 2 ms trong `Motor_Speed_Tick()`, vi điều khiển đọc giá trị đếm xung tức thời từ thanh ghi phần cứng `TIM3->CNT` (bánh trái) và `TIM2->CNT` (bánh phải).
   - Biến đổi hiệu số xung chống tràn số nguyên:
     $$\Delta \text{count} = (\text{int16\_t})((\text{uint16\_t})\text{curr} - (\text{uint16\_t})\text{prev})$$
   - Quy đổi sang quãng đường thực tế di chuyển trong chu kỳ:
     $$\Delta s = \Delta \text{count} \times \text{MM\_PER\_COUNT} \quad (\text{với } \text{MM\_PER\_COUNT} = \pi \times D / 700)$$
     Đường kính bánh xe hiệu dụng: $D_L = 29.24\text{ mm}$, $D_R = 29.03\text{ mm}$.
   - Vận tốc tức thời được lọc thông thấp qua bộ lọc IIR bậc 1 (`SPEED_LPF_ALPHA = 0.3`):
     $$v_k = v_{k-1} + 0.3 \times (v_{\text{raw}} - v_{k-1})$$
   - Quãng đường tuyệt đối `dist_left_mm`, `dist_right_mm` tích lũy trực tiếp từ $\Delta s$ **không qua bộ lọc** để loại bỏ hoàn toàn độ trễ pha khi tính toán vị trí dừng.
2. **Khâu điều khiển quỹ đạo chuyển động ngoài (`Motion_Update`):**
   - Nhận mục tiêu từ các API chuyển động: Quãng đường đích $S_{\text{target}}$ hoặc Góc quay đích $\theta_{\text{target}}$.
   - **Tạo biên dạng tốc độ (Deceleration Profiling):**
     $$v = \min\left(v_{\text{cmd}}, \sqrt{2 \times a_{\text{decel}} \times S_{\text{remain}}}\right)$$
   - **Điều khiển góc hướng (Heading PD):**
     $$\omega = K_{p,\text{head}} \times (\theta_{\text{target}} - \theta_{\text{gyro}}) - K_{d,\text{head}} \times \dot{\theta}_{\text{gyro,filt}} + \text{offset}_{\text{wall}}$$
     Trong đó: $K_{p,\text{head}} = 20.0$, $K_{d,\text{head}} = 0.5$, $\text{offset}_{\text{wall}}$ là lượng lái bù cân giữa ô từ cảm biến IR hai bên (tối đa $\pm 5^\circ$).
   - Phân rã vận tốc cho từng bánh với bề rộng cơ sở $W = \text{TRACK\_WIDTH\_MM} = 76.0\text{ mm}$:
     $$v_{L,\text{target}} = v - \omega \times \frac{W}{2}, \quad v_{R,\text{target}} = v + \omega \times \frac{W}{2}$$
3. **Khâu điều khiển vòng trong PID Vận tốc (`Motor_Speed_Control`):**
   - Giới hạn tốc độ biến thiên gia tốc đặt bằng hàm `Ramp` với gia tốc cực đại $\text{MAX\_ACCEL\_MMS2} = 2000\text{ mm/s}^2$.
   - **Khâu Feedforward (Bù trước):**
     $$\text{FF} = K_s \times \text{sign}(v_{\text{ref}}) + K_f \times v_{\text{ref}} + K_a \times \frac{dv_{\text{ref}}}{dt}$$
     Trong đó: $K_s \approx 80-85$ (thắng ma sát tĩnh), $K_f \approx 4.85-4.95$, $K_a \approx 0.17$ (bù quán tính động cơ).
   - **Khâu Feedback (Phản hồi PI):**
     $$e = v_{\text{ref}} - v_{\text{meas}}$$
     $$P = K_p \times e \quad (K_p = 5.0)$$
     $$I = I + K_i \times e \times dt \quad (K_i = 150.0)$$
     Tích phân có điều kiện (Conditional Anti-windup): Khâu $I$ chỉ tích lũy khi đầu ra chưa bão hòa hoặc sai số kéo đầu ra ra khỏi vùng bão hòa.
   - Đầu ra điều khiển $\text{Duty} = \text{FF} + P + I$, được kẹp trong dải $[-2999, +2999]$.
4. **Xuất xung phần cứng (`Motor_SetPWM`):**
   - Điều khiển cầu H ở chế độ Drive/Brake (Slow decay):
     - $\text{Duty} > 0$: Kênh IN2 kéo mức cao 100% (`CCR = 3000`), Kênh IN1 phát xung PWM tỉ lệ nghịch để dẫn dòng và phanh xen kẽ.
     - $\text{Duty} < 0$: Kênh IN1 kéo mức cao 100%, Kênh IN2 phát xung PWM.
     - $\text{Duty} = 0$: Cả hai kênh IN1 và IN2 đều kéo lên mức cao 100% để ngắn mạch cuộn dây động cơ (Phanh chủ động tức thời).

---

### Luồng 4: Dữ liệu Bản đồ Mê cung & Thuật toán Flood-Fill (Maze Mapping & Path Planning)

Luồng dữ liệu này xử lý bản đồ 16x16 ô, thực thi thuật toán loang nước (Flood-Fill) tìm đường đi ngắn nhất và cập nhật kế hoạch di chuyển thời gian thực.

```mermaid
flowchart TD
    A["Mắt hồng ngoại IR (L, R, FL, FR)"] -->|"Đọc tại vị trí cách tâm ô k 90mm"| B["Straight_SenseCell(k)"]
    B -->|"L hoặc R >= 170"| C["Xác nhận có vách bên: Map_SetEdge(x, y, dir, wall=true)"]
    B -->|"TB(FL, FR) > 120"| D["Xác nhận có tường trước: Map_SetEdge(x, y, dir, wall=true)"]

    C --> E["Cập nhật ma trận bản đồ RAM: maze_map[16][16]"]
    D --> E

    E -->|"Đích đến (4 ô tâm hoặc ô 0,0)"| F["Thuật toán Flood-Fill (BFS đa nguồn): Flood_Fill()"]
    F -->|"Hàng đợi tĩnh bfs_queue[256]"| G["Ma trận khoảng cách: maze_dist[16][16]"]

    G --> H["Lựa chọn hướng tối ưu: Plan_BestDir()"]
    H -->|"Ưu tiên: Thẳng > Phải > Trái > Quay đầu"| I["Thuật toán gộp ô thẳng: Plan_StraightCells()"]

    I -->|"Gộp được n ô liên tiếp cùng hướng"| J["Kế hoạch di chuyển: Đích mới = n * 180 mm"]
    J -->|"Xe đang chạy trên đường thẳng"| K["Move_Straight_SetTarget(): Kéo dài hoặc Hãm ngắn đích"]

    K --> L["Thực thi chuyển động: pid.c điều khiển động cơ"]
    L -->|"Gặp chướng ngại vật IR dừng sớm"| M["MOTION_STATUS_WALL -> Làm tròn ô và cập nhật vách tường"]
```

#### Cấu trúc dữ liệu bản đồ trong RAM:
Mỗi ô $(x, y)$ trong mê cung kích thước $16 \times 16$ được đại diện bởi đúng **1 byte** trong mảng `maze_map[16][16]`:
- **Bit 0:** Tường phía Bắc (North Wall) - 1: Có tường, 0: Mở.
- **Bit 1:** Tường phía Đông (East Wall).
- **Bit 2:** Tường phía Nam (South Wall).
- **Bit 3:** Tường phía Tây (West Wall).
- **Bit 4:** Trạng thái đã biết phía Bắc (North Known) - 1: Đã khám phá, 0: Chưa biết.
- **Bit 5:** Trạng thái đã biết phía Đông (East Known).
- **Bit 6:** Trạng thái đã biết phía Nam (South Known).
- **Bit 7:** Trạng thái đã biết phía Tây (West Known).

Mỗi vách ngăn giữa hai ô kề nhau luôn được cập nhật đồng thời cho cả hai ô đối ngẫu (ví dụ: tường phía Bắc của ô $(x, y)$ đồng nhất với tường phía Nam của ô $(x, y+1)$).

---

### Luồng 5: Dữ liệu Lưu trữ Flash & Phục hồi Bản đồ (Flash Persistence & State Restoration)

Luồng dữ liệu này đảm bảo trạng thái bản đồ và tọa độ hoạt động của robot không bị mất đi khi ngắt nguồn, sử dụng cơ chế ghi nhật ký nối tiếp phân phối hao mòn.

```mermaid
flowchart TD
    A["Sự kiện kích hoạt lưu Flash: Chạm đích / Dò xong / Bị nhấc xe / Bị lỗi"] --> B["1. Dừng động cơ: Motor_Stop() (Đảm bảo xe đứng yên tuyệt đối)"]
    B --> C["2. Thu thập dữ liệu: Maze_ExportMap(), Maze_GetPose(), Maze_GetStats()"]
    
    C --> D["3. Đóng gói struct FlashMapData (280 bytes):
    - magic_word = 0xA5A5B6B6
    - version = 1, record_size = 280
    - sequence = seq_cu + 1
    - maze_map[256], last_x, last_y, last_heading
    - phase, rounds, reason, flags"]

    D --> E["4. Tính mã kiểm tra toàn vẹn: Crc32() trên 276 bytes đầu"]
    E --> F["Điền checksum vào rec->checksum"]

    F --> G["5. Quét tìm slot trống: Scan(&latest, &seq, &slot)"]
    G -->|"Nếu Sector đầy (slot >= 468)"| H["Xóa toàn bộ Sector 7: Erase_Sector() (mất ~1-2s)"]
    G -->|"Còn slot trống"| I["HAL_FLASH_Unlock() & Ghi từng word 32-bit vào Slot_Addr(slot)"]
    H --> I

    I --> J["HAL_FLASH_Lock()"]
    J --> K["Đồng bộ ART Data Cache: Flash_FlushDataCache()"]
    K --> L["Đọc lại bộ nhớ so sánh: memcmp(FLASH_PTR, rec, 280)"]
    L -->|"Trùng khớp 100%"| M["Trả về FLASH_OK -> Hoàn tất lưu an toàn"]
    L -->|"Sai lệch"| N["Trả về FLASH_ERR_VERIFY"]
```

#### Cấu trúc bản ghi `FlashMapData` (Tổng kích thước: 280 bytes):
| Trường dữ liệu | Kiểu dữ liệu | Kích thước | Ý nghĩa nghiệp vụ |
| :--- | :--- | :--- | :--- |
| `magic_word` | `uint32_t` | 4 bytes | Hằng số nhận diện dữ liệu hợp lệ: `0xA5A5B6B6` |
| `version` | `uint16_t` | 2 bytes | Phiên bản cấu trúc bản ghi (Hiện tại: `1`) |
| `record_size` | `uint16_t` | 2 bytes | Kích thước bản ghi (`sizeof(FlashMapData) = 280`) |
| `sequence` | `uint32_t` | 4 bytes | Số thứ tự tăng dần của lần lưu (dùng để tìm bản ghi mới nhất) |
| `maze_map[256]`| `uint8_t[256]` | 256 bytes | Toàn bộ ma trận trạng thái tường của mê cung 16x16 ô |
| `last_x`, `last_y` | `uint8_t`, `uint8_t` | 2 bytes | Tọa độ ô cuối cùng của robot trước khi lưu |
| `last_heading`| `uint8_t` | 1 byte | Hướng nhìn của robot (0: Bắc, 1: Đông, 2: Nam, 3: Tây) |
| `phase` | `uint8_t` | 1 byte | Giai đoạn giải mê cung (`MazePhase`) |
| `rounds` | `uint8_t` | 1 byte | Số vòng khám phá đã thực hiện |
| `reason` | `uint8_t` | 1 byte | Lý do lưu: `NEW(0)`, `GOAL(1)`, `EXPLORED(2)`, `LIFTED(3)`, `ERROR(4)` |
| `flags` | `uint8_t` | 1 byte | Cờ trạng thái: `FLASH_MAP_HAS_PATH(0x01)`, `FLASH_MAP_EXPLORED(0x02)` |
| `reserved` | `uint8_t` | 1 byte | Byte đệm dự phòng căn chỉnh 32-bit |
| `checksum` | `uint32_t` | 4 bytes | Mã kiểm tra CRC-32 của 276 bytes dữ liệu phía trước |

---

### Luồng 6: Dữ liệu Truyền thông Bluetooth Telemetry lên Máy tính (Host Monitoring Stream)

Luồng dữ liệu này phục vụ giám sát không dây quá trình hoạt động của robot, gửi bản đồ giải được lên máy tính để trực quan hóa đồ họa.

```mermaid
flowchart TD
    A["Sự kiện: Kết thúc vòng chạy / Lỗi / Máy tính gửi ký tự 'M' lúc IDLE"] --> B["Hàm gửi bản đồ: Maze_SendMap()"]
    B --> C["1. Định dạng dòng đầu: #MAZE_BEGIN 16\r\n"]
    C --> D["2. Trạng thái máy: #STATE <state> <phase> <rounds>\r\n"]
    D --> E["3. Vị trí robot: #ROBOT <x> <y> <heading>\r\n"]
    E --> F["4. Thống kê chạy: #STATS <straights> <cells> <turns90> ...\r\n"]
    F --> G["5. Xuất 16 dòng bản đồ: #ROW <y> <32 ký tự Hex biểu diễn 16 ô>\r\n"]
    G --> H["6. Dòng kết thúc kèm Checksum: #MAZE_END <sum % 65536>\r\n"]

    H --> I["Truyền nối tiếp qua USART1 (PA15 TX -> Module Bluetooth HC-05 @9600 baud)"]
    I -->|"Sóng không dây Bluetooth RF"| J["Cổng COM ảo trên máy tính (Mặc định COM8)"]

    J --> K["maze_viewer.py: Luồng nền SerialWorker đọc byte"]
    K --> L["Bộ phân tích cú pháp Regex tách khối #MAZE_BEGIN ... #MAZE_END"]
    L --> M{"Kiểm tra Checksum 16-bit"}
    M -->|"Checksum Sai"| N["Gửi lại ký tự 'M' yêu cầu truyền lại sau 3s"]
    M -->|"Checksum Đúng"| O["1. Lưu log tệp văn bản vào maze/logs/maze_YYYYMMDD_HHMMSS.txt"]
    O --> P["2. Chuyển dữ liệu qua hàng đợi evt_q vào luồng giao diện Tkinter"]
    P --> Q["Vẽ đồ họa canvas thời gian thực: Tường đỏ, Cạnh xám, Đường BFS xanh, Vị trí xe cam"]
```

#### Cấu trúc khối dữ liệu ASCII truyền qua Bluetooth:
```text
#MAZE_BEGIN 16
#STATE 6 3 2
#ROBOT 0 0 0
#STATS 24 156 18 2 12 3 1 45
#ROW 15 1F1E1E1E1E1E1E1E1E1E1E1E1E1E1E1B
#ROW 14 1D101010101010101010101010101019
...
#ROW 0 1D101010101010101010101010101019
#MAZE_END 45281
```
- Khối văn bản có kích thước xấp xỉ ~800 bytes.
- Tại tốc độ truyền 9600 baud (10 bit/byte), toàn bộ khối dữ liệu được truyền xong trong khoảng thời gian ~0.83 - 1.0 giây.
- Hàm `Maze_SendMap()` chỉ được phép thực thi khi robot đã dừng hoàn toàn trên sàn để đảm bảo việc truyền dữ liệu kiểu polling không làm gián đoạn chuyển động.
