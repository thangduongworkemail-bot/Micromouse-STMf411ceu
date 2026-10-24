# TÀI LIỆU THAM CHIẾU CHI TIẾT CÁC HÀM QUAN TRỌNG (FUNCTION REFERENCE)

Tài liệu này đặc tả chi tiết mục đích, ngữ cảnh thực thi, tham số, dữ liệu truy cập, thuật toán xử lý nội bộ, quan hệ gọi hàm và cơ chế phòng chống lỗi của tất cả các hàm quan trọng trong toàn bộ mã nguồn của dự án.

---

## 1. Module Khởi tạo & Điều phối Hệ thống (`Core/Src/main.c`)

### 1.1 `HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)`
- **Vị trí:** [Core/Src/main.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/main.c#L838-L851)
- **Mục đích:** Hàm phục vụ ngắt định thời chính (Master Control Tick) của hệ thống, kích hoạt chính xác mỗi 2.0 ms (500 Hz).
- **Ngữ cảnh gọi:** Ngắt phần cứng `TIM1_BRK_TIM9_IRQn` (Preemption Priority = 0).
- **Tham số:** `htim` - Con trỏ tới cấu trúc Timer gây ngắt.
- **Dữ liệu đọc/ghi:** Kiểm tra con trỏ ngoại vi `htim->Instance == TIM9`.
- **Trình tự xử lý:**
  1. Kiểm tra điều kiện `htim->Instance == TIM9`.
  2. Gọi `Motor_Speed_Tick()` ([pid.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/pid.c)): Đọc Encoder, tính vận tốc, chạy vòng điều khiển PID và xuất xung PWM cho động cơ.
  3. Gọi `MPU6500_Tick()` ([mpu6500.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mpu6500.c)): Gửi lệnh đọc bất đồng bộ 14 bytes qua I2C DMA.
  4. Gọi `(void)Check_Robot_Picked_Up()` ([mode_manager.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mode_manager.c)): Giám sát an toàn tức thời.
- **Hàm gọi nội bộ:** `Motor_Speed_Tick()`, `MPU6500_Tick()`, `Check_Robot_Picked_Up()`.
- **Xử lý ngoại lệ / Ràng buộc an toàn:** Hàm chạy trong ngữ cảnh ISR ưu tiên cao nhất, tuyệt đối không chứa vòng lặp chờ, không gọi `HAL_Delay`, không in chuỗi nối tiếp.

---

### 1.2 `CDC_Print(const char *str)`
- **Vị trí:** [Core/Src/main.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/main.c#L75-L87)
- **Mục đích:** Truyền chuỗi ký tự qua cổng USB CDC ảo mà **không bao giờ gây treo chương trình** khi xe chạy bằng pin không cắm cáp hoặc máy tính chưa mở cổng COM.
- **Ngữ cảnh gọi:** Thread context (chỉ trong `while(1)`, không gọi trong ngắt).
- **Tham số:** `str` - Con trỏ tới chuỗi ký tự kết thúc bằng null.
- **Trình tự xử lý & Cơ chế chống treo:**
  1. Kiểm tra trạng thái USB: `if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;` $\to$ Nếu chưa cắm cáp hoặc chưa enumerate xong thì thoát ngay.
  2. Lấy thời gian bắt đầu `start = HAL_GetTick()`.
  3. Vòng lặp `while (CDC_Transmit_FS(...) == USBD_BUSY)`:
     - Nếu cờ `host_stalled == 1` hoặc thời gian chờ vượt quá $20\text{ ms}$: đánh dấu `host_stalled = 1` và thoát ngay lập tức, bỏ qua chuỗi này.
  4. Nếu truyền thành công: xóa cờ `host_stalled = 0`.
- **Giá trị trả về / Tác động phụ:** Không trả về. Bảo vệ robot chạy pin không bị đứng yên vô tận tại lệnh in USB đầu tiên.

---

### 1.3 `UART_Print(const char *str)`
- **Vị trí:** [Core/Src/main.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/main.c#L107-L114)
- **Mục đích:** Gửi chuỗi ký tự qua USART1 (PA15 TX) tới module Bluetooth HC-05 với thời gian chờ (timeout) tự động thích ứng với độ dài chuỗi và baudrate.
- **Tham số:** `str` - Chuỗi ký tự cần gửi.
- **Thuật toán tính toán thời gian chờ:**
  $$\text{timeout\_ms} = \frac{\text{len} \times 10000}{\text{huart1.Init.BaudRate}} + 20$$
  *Giải thích:* Mỗi ký tự cần 10 bit (1 Start, 8 Data, 1 Stop). Tại 9600 baud, 100 ký tự mất ~104 ms. Thời gian chờ động cộng thêm 20 ms giúp khắc phục lỗi bị cắt cụt đuôi dòng (line clipping) khi truyền ở tốc độ thấp mà không bị treo vô tận nếu HC-05 mất kết nối.

---

### 1.4 `UART_TryRead(void)`
- **Vị trí:** [Core/Src/main.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/main.c#L127-L133)
- **Mục đích:** Đọc 1 byte từ cổng nối tiếp USART1 theo cơ chế phi khóa (Non-blocking).
- **Giá trị trả về:** Giá trị ký tự nhận được ($0 \dots 255$), hoặc `-1` nếu không có dữ liệu mới.
- **Xử lý đặc biệt:** Đọc cờ `UART_FLAG_RXNE` và `UART_FLAG_ORE` (Overrun Error), sau đó đọc thanh ghi `huart1.Instance->DR`. Thao tác này tự động xóa cờ báo tràn bộ đệm ORE nếu ký tự gửi đến lúc CPU đang bận điều khiển động cơ.

---

## 2. Module Quản lý Chế độ & Hệ thống An toàn (`Core/Src/mode_manager.c`)

### 2.1 `ModeManager_Init(void)`
- **Vị trí:** [Core/Src/mode_manager.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mode_manager.c#L496-L511)
- **Mục đích:** Khởi tạo các biến nội bộ của hệ thống an toàn, tắt còi và quét bản đồ đã lưu trong Flash Sector 7.
- **Trình tự xử lý:**
  1. Tắt chân còi PB8 (`Buzzer_Out(false)`).
  2. Đặt `ui_armed = false`, xóa các bộ đếm nhấc xe `Pickup_ResetCounters()`.
  3. Gọi `Flash_LoadMap(&mm_rec)`: Đọc bản ghi hợp lệ mới nhất từ Flash.
  4. In thông tin trạng thái bản đồ qua UART (Số lần lưu, có đường tới đích hay chưa, đã dò xong chưa).
  5. Gọi `Enter_Idle()`: Phanh động cơ và chuyển sang `STATE_IDLE`.

---

### 2.2 `ModeManager_Update(void)`
- **Vị trí:** [Core/Src/mode_manager.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mode_manager.c#L513-L628)
- **Mục đích:** Máy trạng thái chính cấp cao điều phối toàn bộ hành vi của robot, chạy liên tục trong vòng lặp `while(1)`.
- **Ngữ cảnh gọi:** Thread context. Không được phép thêm `HAL_Delay` vào luồng này.
- **Trình tự xử lý theo trạng thái (`mm_state`):**
  1. Gọi `Buzzer_Update(now)`: Cập nhật nhịp còi.
  2. Gọi `UI_Pressed(now)`: Cập nhật sự kiện nút ảo IR trái.
  3. **Giám sát an toàn khẩn cấp:** Nếu đang ở trạng thái chuyển động (`State_Moving`) mà cờ `robot_lifted != 0` hoặc trạng thái giải mê cung báo `MAZE_STATE_PAUSED` $\to$ Chuyển ngay sang `Enter_Paused()` và thoát.
  4. Phân nhánh Switch-Case:
     - `STATE_IDLE`: Nếu nhận cử chỉ che tay $\to$ `ui_count = 1`, còi kêu 1 tiếng, chuyển sang `STATE_UI_BEEP`. Nếu không, kiểm tra yêu cầu xin bản đồ từ PC (`Serve_MapRequest`).
     - `STATE_UI_BEEP` / `STATE_UI_WINDOW`: Nếu che thêm $\to$ tăng `ui_count`, phát tiếp còi. Nếu còi tắt $\to$ chuyển sang `STATE_UI_WINDOW` đếm ngược 3 giây. Hết 3 giây $\to$ gọi `Select_Mode(ui_count, now)`.
     - `STATE_START`: Gọi `State_Start(now)` kiểm tra điều kiện bỏ tay, xe nằm trên sàn, còi tắt $\to$ đo lại Gyro Z tĩnh $\to$ gọi `Start_SelectedMode()`.
     - `STATE_EXPLORE`: Gọi `Maze_Step()`. Nếu đạt `CHECKPOINT` $\to$ phanh động cơ, lưu Flash (`FLASH_SAVE_GOAL`), gọi `Maze_CheckpointDone()`. Nếu `FINISHED` $\to$ lưu Flash (`FLASH_SAVE_EXPLORED`), gửi bản đồ lên PC, về `STATE_IDLE`. Nếu `ERROR` $\to$ gọi `Enter_Error()`.
     - `STATE_MODE2_RUN`: Gọi `Maze_Step()`. Khi tới đích (`FINISHED`) $\to$ phát còi reo mừng `bz_goal`, chuyển sang `STATE_GOAL_BEEP`.
     - `STATE_GOAL_BEEP`: Hết còi $\to$ gọi `Maze_StartReturn()` (quay 180°), chuyển sang `STATE_RETURN_HOME`.
     - `STATE_RETURN_HOME`: Gọi `Maze_Step()`. Khi về tới (0,0) quay Bắc $\to$ gửi bản đồ, về `STATE_IDLE`.
     - `STATE_PAUSED`: Giám sát `Still_Holding()`. Nếu xe nằm yên tĩnh liên tục đủ 3 giây (`PAUSE_STILL_MS`) và `robot_lifted == 0` $\to$ chuyển sang `STATE_RESUME`.
     - `STATE_RESUME`: Gọi `Maze_Resume()` (quét lại tường, tính lại đường), khôi phục trạng thái dở dang `mm_resume_to`.
     - `STATE_ERROR`: Hết còi lỗi 3 tiếng $\to$ về `STATE_IDLE`.

---

### 2.3 `Check_Robot_Picked_Up(void)`
- **Vị trí:** [Core/Src/mode_manager.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mode_manager.c#L159-L219)
- **Mục đích:** Giám sát đa cảm biến phát hiện sự cố nhấc bổng robot hoặc va chạm cơ học.
- **Ngữ cảnh gọi:** Ngắt thời gian thực `TIM9` (chu kỳ 2 ms).
- **Trình tự xử lý:**
  1. Kiểm tra cờ phần cứng `robot_lifted != 0`: Nếu IMU đã khóa $\to$ trả về ngay lý do `pickup_reason`.
  2. Nếu không ở trạng thái chuyển động (`!State_Moving`) $\to$ xóa bộ đếm và thoát (trả về 0).
  3. **Tầng Encoder (`PICKUP_ENC_ENABLE`):** Kiểm tra `Wheel_Overspeed()` cho cả hai bánh. Nếu bánh xe quay nhanh hơn vận tốc đặt $> 120\text{ mm/s}$ liên tục trong 15 chu kỳ ($30\text{ ms}$) khi $|v_{\text{ref}}| > 50\text{ mm/s}$ $\to$ Bật cờ `PICKUP_BY_ENCODER` (mất tải bánh khi bị nhấc lên).
  4. **Tầng Quang học IR (`PICKUP_IR_ENABLE`):** Nếu xe đang đứng yên và vừa thấy tường rõ mà đột ngột cả 4 mắt tụt về nền không tường liên tục 30 chu kỳ ($60\text{ ms}$) $\to$ Bật cờ `PICKUP_BY_IR`.
  5. Nếu `reason != 0`: Gán `pickup_reason = reason` và gọi `MPU6500_ForceLifted()`.

---

### 2.4 `UI_Pressed(uint32_t now)`
- **Vị trí:** [Core/Src/mode_manager.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mode_manager.c#L101-L123)
- **Mục đích:** Xử lý chống rung và phát hiện cử chỉ che tay trên mắt thu hồng ngoại trái.
- **Tham số:** `now` - Thời điểm `HAL_GetTick()`.
- **Giá trị trả về:** `true` đúng duy nhất một lần cho một lượt che hợp lệ.
- **Logic nhánh điều kiện:**
  - Nếu `raw < 1500`: Đánh dấu đã rút tay `ui_armed = true`, `ui_above = false`.
  - Nếu `raw > 2000` và `ui_armed == true`:
    - Nếu mới chớm vượt ngưỡng: ghi nhận mốc `ui_t_above = now`, `ui_above = true`.
    - Nếu đã duy trì liên tục $\ge 30\text{ ms}$ (`UI_DEBOUNCE_MS`): Chốt sự kiện nhấn, giải giáp `ui_armed = false`, trả về `true`.

---

### 2.5 `Save_Map(uint8_t reason)`
- **Vị trí:** [Core/Src/mode_manager.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mode_manager.c#L328-L353)
- **Mục đích:** Lưu trữ bản đồ và tọa độ dở dang vào Flash Sector 7 một cách an toàn.
- **Tham số:** `reason` - Lý do lưu (`FLASH_SAVE_NEW`, `GOAL`, `EXPLORED`, `LIFTED`, `ERROR`).
- **Trình tự xử lý an toàn:**
  1. `Motor_Stop()`: Tắt PWM và phanh cứng động cơ (Bảo vệ an toàn chống CPU stall khi ghi Flash).
  2. `Maze_ExportMap(mm_rec.maze_map)`: Chép 256 bytes bản đồ từ RAM.
  3. `Maze_GetPose(&x, &y, &h)`: Thu thập tọa độ ô và hướng nhìn.
  4. Thu thập thống kê vòng chạy, cờ `FLASH_MAP_HAS_PATH` và `FLASH_MAP_EXPLORED`.
  5. Gọi `Flash_SaveMap(&mm_rec)`: Thực thi ghi phân phối hao mòn.
  6. In kết quả trạng thái lưu qua UART.

---

## 3. Module Điều khiển Chuyển động & Động cơ (`Core/Src/pid.c`)

### 3.1 `Motor_Speed_Tick(void)`
- **Vị trí:** [Core/Src/pid.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/pid.c#L464-L525)
- **Mục đích:** Hàm cập nhật đo đạc encoder và điều phối vận tốc chu kỳ 2 ms trong ngắt TIM9.
- **Trình tự xử lý:**
  1. Đọc thanh ghi phần cứng `TIM3->CNT` (trái) và `TIM2->CNT` (phải).
  2. Tính delta xung an toàn: `(int16_t)(uint16_t curr - prev)`.
  3. Nhân với `MM_PER_COUNT_L` ($0.13123\text{ mm}$) và `MM_PER_COUNT_R` ($0.13029\text{ mm}$).
  4. Lọc thông thấp vận tốc tức thời: `speed_left_mms`, `speed_right_mms`.
  5. Tích phân quãng đường không lọc: `dist_left_mm`, `dist_right_mm`.
  6. Cập nhật vị trí Odometry dead-reckoning: `odo_x_mm`, `odo_y_mm`.
  7. Gọi `IR_Tick()` để cập nhật dữ liệu cảm biến tường.
  8. Gọi `Motion_Update()` để cập nhật quỹ đạo góc/hướng ngoài.
  9. Gọi `Motor_Speed_Control(delta_left, delta_right)` tính toán PID.

---

### 3.2 `Motor_Speed_Control(float delta_left, float delta_right)`
- **Vị trí:** [Core/Src/pid.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/pid.c#L380-L462)
- **Mục đích:** Thực thi vòng lặp điều khiển PID vận tốc từng bánh xe.
- **Trình tự xử lý:**
  1. Nếu cờ `motor_pid_enable == 0`: Reset bộ tích phân PID, xuất `Motor_SetPWM(0, 0)` và thoát.
  2. Tạo dốc gia tốc: `ref_speed_left = Ramp(ref_speed_left, target_speed_left, MAX_ACCEL_MMS2 * dt)`.
  3. Tính toán Feedforward 3 thành phần:
     $$\text{FF} = K_s \cdot \text{sign}(v_{\text{ref}}) + K_f \cdot v_{\text{ref}} + K_a \cdot \frac{dv_{\text{ref}}}{dt}$$
  4. Tính toán phản hồi `PID_Compute()`: Tính sai số $e = v_{\text{ref}} - v_{\text{meas}}$, khâu $P = K_p \cdot e$, khâu $I = I + K_i \cdot e \cdot dt$ (áp dụng Anti-windup).
  5. Cộng $\text{PWM} = \text{FF} + \text{PID}$, kẹp giá trị trong khoảng $[-\text{PWM\_MAX}, +\text{PWM\_MAX}]$ ($\pm 2999$).
  6. Xuất lệnh phần cứng qua `Motor_SetPWM(pwm_left, pwm_right)`.

---

### 3.3 `Motion_Update(void)`
- **Vị trí:** [Core/Src/pid.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/pid.c#L1256-L1428)
- **Mục đích:** Vòng điều khiển chuyển động tầng ngoài (Outer Loop), điều khiển xe đi thẳng, quay góc, hoặc đứng giữ hướng.
- **Phân nhánh xử lý:**
  - `MOTION_STRAIGHT`:
    - Tính quãng đường còn lại $S_{\text{remain}} = S_{\text{target}} - S_{\text{curr}}$.
    - Tính trần tốc độ căn bậc hai: $v = \min(v_{\text{max}}, \sqrt{2 \cdot a_{\text{decel}} \cdot S_{\text{remain}}})$.
    - Khâu bảo vệ tường trước: nếu mắt trước $> \text{WALL\_SLOW\_IR} (300)$, giảm tốc dần về $80\text{ mm/s}$. Nếu mắt trước (tính cả đà phanh) $\ge \text{WALL\_TARGET\_IR} (1700) \to$ dừng khẩn cấp với mã `MOTION_WALL`.
    - Tính sai số hướng: $\text{err} = \theta_{\text{target}} - \theta_{\text{gyro}}$.
    - Lấy lượng lái bù cân giữa ô: $\text{offset} = \text{Wall\_CenterOffset()}$.
    - Khâu PD hướng: $\omega = K_{p,\text{head}} \cdot \text{err} - K_{d,\text{head}} \cdot \dot{\theta} + \text{offset}$ (kẹp $\pm 90 \text{ ^\circ/s}$).
    - Phân bổ: $v_L = v - \omega \cdot W/2$, $v_R = v + \omega \cdot W/2$.
  - `MOTION_TURN`:
    - Tính góc quay còn lại $\theta_{\text{remain}}$.
    - Tính tốc độ quay căn bậc hai: $\omega = \min(\omega_{\text{max}}, \sqrt{2 \cdot \alpha_{\text{decel}} \cdot \theta_{\text{remain}}})$.
    - Phân bổ: $v_L = -\omega \cdot W/2$, $v_R = +\omega \cdot W/2$.
  - `MOTION_HOLD`:
    - Điều khiển đứng yên giữ hướng tuyệt đối, tự căn vuông góc tường qua máy trạng thái `Hold_Update()`.
  - `MOTION_CURVE`:
    - Điều khiển khúc cua mượt 90 độ qua `Curve_Update()`.

---

### 3.4 `Move_Straight_Begin(float dist_mm, float speed_mms)`
- **Vị trí:** [Core/Src/pid.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/pid.c#L1066-L1108)
- **Mục đích:** Bắt đầu lệnh đi thẳng không chặn (Non-blocking) phục vụ thuật toán giải mê cung chạy liên tục.
- **Tham số:** `dist_mm` - Quãng đường đích ban đầu (mm); `speed_mms` - Vận tốc tối đa (mm/s).
- **Giá trị trả về:** `1` nếu bắt đầu thành công, `0` nếu robot đang bị nhấc hoặc IMU chưa sẵn sàng.

---

### 3.5 `Wall_Align_Begin(void)` & `Wall_Align_Poll(void)`
- **Vị trí:** [Core/Src/pid.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/pid.c#L2108-L2156)
- **Mục đích:** API căn vuông góc tường trước dạng không chặn (Non-blocking), chia thành pha kích hoạt và pha thăm dò trạng thái.
- **Giá trị trả về của `Wall_Align_Poll()`:**
  - `WALL_ALIGN_BUSY (0)`: Quá trình căn chỉnh đang diễn ra (tiến/lùi, đo quang, xoay chỉnh, hoặc đo lại gyro).
  - `WALL_ALIGN_OK (1)`: Căn vuông góc thành công, hướng Gyro đã được cập nhật chuẩn xác.
  - `WALL_ALIGN_FAILED (2)`: Quá thời gian 4 giây hoặc không thấy vách tường phẳng, hệ thống giữ nguyên mốc góc cũ để tránh treo robot.

---

## 4. Module Cảm biến Quán tính IMU (`Core/Src/mpu6500.c`)

### 4.1 `MPU6500_Setup(void)`
- **Vị trí:** [Core/Src/mpu6500.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mpu6500.c#L368-L402)
- **Mục đích:** Khởi tạo phần cứng toàn diện cho MPU6500 trước khi vào vòng lặp chính.
- **Trình tự xử lý:**
  1. Gọi `MPU6500_Init()`: Cấu hình xung nhịp, dải đo Gyro $\pm 2000 \text{ ^\circ/s}$, dải đo Accel $\pm 8\text{ g}$, lọc DLPF.
  2. Đọc mã nhận dạng thanh ghi `WHO_AM_I`: Chấp nhận `0x70` (MPU6500) hoặc `0x71` (MPU9250).
  3. Gọi `MPU6500_Calibrate()`: Đo tĩnh 500 mẫu loại bỏ offset ban đầu.
  4. Nếu thành công: Thiết lập cờ `imu_ready = 1` cho phép ngắt TIM9 bắt đầu đọc I2C DMA.
  5. Nếu thất bại (xe bị rung hoặc bus I2C kẹt quá 3 giây): Đặt `imu_ready = 1` nhưng đánh dấu `robot_lifted = 1`, khóa động cơ để robot tự cân chỉnh lại khi được đặt nằm yên.

---

### 4.2 `MPU6500_Process_Data(float dt)`
- **Vị trí:** [Core/Src/mpu6500.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mpu6500.c#L464-L572)
- **Mục đích:** Xử lý 14 bytes dữ liệu nhận về từ ngắt DMA I2C, cập nhật tư thế và giám sát an toàn.
- **Trình tự xử lý:**
  1. Chuyển đổi dữ liệu số sang đơn vị vật lý ($g, \text{ ^\circ/s}, ^\circ\text{C}$).
  2. **Bộ lọc bù Pitch / Roll:**
     $$\alpha = \frac{\tau}{\tau + dt} \quad (\tau = 0.5\text{ s})$$
     $$\text{pitch} = \alpha \cdot (\text{pitch} + g_y \cdot dt) + (1 - \alpha) \cdot \text{pitch}_{\text{accel}}$$
     $$\text{roll} = \alpha \cdot (\text{roll} + g_x \cdot dt) + (1 - \alpha) \cdot \text{roll}_{\text{accel}}$$
  3. **Nhận diện đứng yên (`imu_still`):** Đánh giá vận tốc bánh xe và độ phân tán dao động của Gyro Z (`gz_dev < 0.2`).
  4. **Tích phân góc Yaw:**
     - Nếu đứng yên: `yaw_angle` giữ nguyên; `gyro_z_offset` tự động bám trôi bias với $\tau = 2.0\text{ s}$.
     - Nếu di chuyển: $\text{yaw\_angle} += \text{yaw\_rate\_dps} \cdot dt$ (không áp dụng vùng chết).
  5. Gọi `Lift_Update()` và `Crash_Update()` giám sát an toàn.

---

### 4.3 `Lift_Update(float dt, ...)`
- **Vị trí:** [Core/Src/mpu6500.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/mpu6500.c#L740-L846)
- **Mục đích:** Thuật toán phát hiện sự cố nhấc robot lên khỏi sàn đấu và phát hiện đặt robot xuống sàn.
- **Quy tắc phát hiện nhấc xe:**
  - Điều kiện nhấc xe thỏa mãn khi 1 trong 3 dấu hiệu sau xuất hiện **liên tục trong $200\text{ ms}$ (`LIFT_CONFIRM_S`)**:
    1. Góc nghiêng so với phương trọng lực sàn $\text{lift\_tilt\_deg} > 20.0^\circ$.
    2. Vận tốc tịnh tiến đứng $|\text{lift\_vz\_ms}| > 0.12\text{ m/s}$.
    3. Tốc độ lật góc ngoài mặt phẳng $|\omega_1| + |\omega_2| > 30.0 \text{ ^\circ/s}$ (chỉ xét khi tốc độ quay trục đứng $< 80 \text{ ^\circ/s}$).
  - Khi thỏa mãn: Gọi `Lift_Lock(IMU_STATE_PICKED_UP)` $\to$ ngắt động cơ, tắt đèn PB15.
- **Quy tắc phát hiện đặt xuống sàn:**
  - Yêu cầu robot phải nằm yên tĩnh liên tục trong $1.0\text{ s}$ (`LIFT_PLACE_S`):
    - Tốc độ góc mọi trục $< 2.0 \text{ ^\circ/s}$.
    - Độ rung gia tốc $\text{lift\_acc\_dev} < 4\text{ mg}$ (tay người cầm luôn rung $> 5\text{ mg}$).
    - Bánh xe đứng yên và góc nghiêng $< 10.0^\circ$.
  - Khi đạt chuẩn: Tự động đo lại mốc offset 3 trục con quay, reset `yaw_angle = 0`, bật sáng đèn LED PB15 và gọi `Motion_OnPlaced()`.

---

## 5. Module Thuật toán Giải mê cung (`Core/Src/solving_maze.c`)

### 5.1 `Maze_Step(void)`
- **Vị trí:** [Core/Src/solving_maze.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/solving_maze.c#L720-L870)
- **Mục đích:** Thực thi một bước trong máy trạng thái giải mê cung, được gọi liên tục từ `ModeManager_Update()`.
- **Phân nhánh trạng thái (`maze_state`):**
  - `MAZE_STATE_DECIDE_PATH`: Quét vách tường gần, kiểm tra đích đến, chạy `Flood_Fill()`, chọn hướng rẽ hoặc lập kế hoạch đi thẳng nhiều ô.
  - `MAZE_STATE_MOVE_STRAIGHT`: Quản lý chặng đi thẳng liên tục. Mỗi chu kỳ kiểm tra quãng đường đã đi, gọi `Straight_SenseCell()` và `Straight_Replan()` để mở rộng/thu hẹp đích đến.
  - `MAZE_STATE_ALIGN`: Khởi động và thăm dò kết quả căn vuông góc tường trước bằng `Wall_Align_Begin()` / `Wall_Align_Poll()`.
  - `MAZE_STATE_TURN_LEFT` / `RIGHT` / `U_TURN`: Bắt đầu quay tại chỗ qua `Turn_Angle_Begin()`, theo dõi cho tới khi hoàn tất qua `HW_MotionStatus()`.
  - `MAZE_STATE_CHECKPOINT`: Tạm dừng ở ô đích để Mode Manager lưu Flash, sau đó đi tiếp.
  - `MAZE_STATE_PAUSED`: Trạng thái bị nhấc xe tạm dừng an toàn.

---

### 5.2 `Flood_Fill(MazePhase ph, bool optimistic)`
- **Vị trí:** [Core/Src/solving_maze.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/solving_maze.c#L240-L310)
- **Mục đích:** Thuật toán loang khoảng cách tìm đường đi ngắn nhất dạng BFS đa nguồn (Multi-source Breadth-First Search).
- **Tham số:**
  - `ph`: Giai đoạn giải (`SEARCH_GOAL`, `SEARCH_START`, `SPEED_RUN`, `RETURN`).
  - `optimistic`: Cờ lạc quan (`true` khi khám phá - coi cạnh chưa biết là thông thoáng; `false` khi chạy nhanh - chỉ đi qua cạnh đã xác nhận mở).
- **Thuật toán chi tiết:**
  1. Khởi tạo toàn bộ ma trận khoảng cách `maze_dist[16][16] = DIST_INF (0xFFFF)`.
  2. Đưa tất cả các ô đích vào hàng đợi tĩnh `bfs_queue[256]` với khoảng cách 0:
     - Nếu đích là tâm: nạp 4 ô $(7,7), (7,8), (8,7), (8,8)$.
     - Nếu đích là xuất phát: nạp ô $(0,0)$.
  3. Vòng lặp lan tỏa BFS (FIFO):
     - Lấy ô $(x, y)$ từ đầu hàng đợi.
     - Duyệt 4 hướng kề cận (Bắc, Đông, Nam, Tây).
     - Kiểm tra khả năng thông xe qua hàm `Map_CanPass(x, y, dir, optimistic)`.
     - Nếu ô láng giềng $(nx, ny)$ chưa có khoảng cách (`maze_dist[nx][ny] == DIST_INF`):
       $$\text{maze\_dist}[nx][ny] = \text{maze\_dist}[x][y] + 1$$
       Đưa $(nx, ny)$ vào cuối hàng đợi.
  4. Quá trình kết thúc khi hàng đợi rỗng. Không dùng đệ quy, không cấp phát bộ nhớ động heap.

---

### 5.3 `Plan_StraightCells(uint8_t x0, uint8_t y0, uint8_t h, ...)`
- **Vị trí:** [Core/Src/solving_maze.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/solving_maze.c#L360-L408)
- **Mục đích:** Thuật toán gộp các ô đi thẳng liên tục trên bản đồ số.
- **Thuật toán:**
  - Từ vị trí xuất phát $(x_0, y_0)$ với hướng nhìn $h$, hàm lần theo gradient giảm dần của ma trận `maze_dist[][]`.
  - Nếu hướng bước đi tối ưu tiếp theo vẫn trùng khớp với hướng hiện tại $h$ và không có tường ngăn cách: Tăng số lượng ô thẳng $n = n + 1$, cập nhật tọa độ sang ô kế tiếp.
  - Dừng lại ngay khi gặp ô cần phải rẽ hướng hoặc chạm vào ô đích.
  - Kết quả trả về số nguyên $n$ (số ô liên tiếp có thể đi thẳng mà không cần phanh rẽ).

---

## 6. Module Lưu trữ Flash Nội (`Core/Src/flash_storage.c`)

### 6.1 `Flash_SaveMap(FlashMapData *rec)`
- **Vị trí:** [Core/Src/flash_storage.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/flash_storage.c#L129-L167)
- **Mục đích:** Lưu trữ bản ghi cấu trúc dữ liệu bản đồ vào ô nhớ trống kế tiếp trong Flash Sector 7.
- **Trình tự xử lý:**
  1. Gọi `Scan(&latest, &seq, &slot)`: Quét tìm vị trí slot trống kế tiếp và số thứ tự mới nhất.
  2. Gán các trường metadata: `magic_word = 0xA5A5B6B6`, `version = 1`, `record_size = 280`, `sequence = seq + 1`.
  3. Tính toán mã kiểm tra toàn vẹn: `rec->checksum = Crc32((uint8_t*)rec, 276)`.
  4. Xử lý slot trống an toàn: Nếu slot hiện tại chứa rác (do mất điện dở dang lần trước) $\to$ nhảy sang slot tiếp theo.
  5. Mở khóa Flash: `HAL_FLASH_Unlock()`.
  6. Nếu Sector 7 đã đầy (`slot >= 468`): Gọi `Erase_Sector()` xóa trắng toàn bộ 128 KB và đặt lại `slot = 0`.
  7. Lập trình Flash: Ghi tuần tự 70 words (32-bit) qua `HAL_FLASH_Program()`.
  8. Khóa Flash: `HAL_FLASH_Lock()`.
  9. Đồng bộ bộ đệm: Gọi `Flash_FlushDataCache()`.
  10. Xác thực đối chiếu: So sánh dữ liệu trong Flash với bản ghi `memcmp()` $\to$ Trả về `FLASH_OK` hoặc `FLASH_ERR_VERIFY`.

---

### 6.2 `Flash_LoadMap(FlashMapData *out)`
- **Vị trí:** [Core/Src/flash_storage.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/flash_storage.c#L116-L127)
- **Mục đích:** Tìm kiếm và nạp bản ghi bản đồ hợp lệ mới nhất từ Flash Sector 7 ra con trỏ `out`.
- **Trình tự xử lý:**
  1. Gọi `Scan(&latest, &seq, &next_free)`:
     - Dò tìm từ đầu sector tới ô trống đầu tiên.
     - Quét ngược từ ô trống về trước, kiểm tra tính hợp lệ của từng bản ghi qua `Rec_Valid(&r)` (Khớp Magic, Version, Size và mã Checksum CRC-32).
  2. Nếu không tìm thấy bất kỳ bản ghi hợp lệ nào: Trả về `FLASH_EMPTY`.
  3. Nếu tìm thấy slot hợp lệ mới nhất: Chép 280 bytes vào bộ nhớ RAM `memcpy(out, Slot_Addr(latest), 280)` $\to$ Trả về `FLASH_OK`.

---

### 6.3 `Crc32(const uint8_t *p, uint32_t n)`
- **Vị trí:** [Core/Src/flash_storage.c](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/Core/Src/flash_storage.c#L47-L57)
- **Mục đích:** Tính toán mã băm kiểm tra tính toàn vẹn 32-bit chuẩn IEEE 802.3.
- **Tham số:** `p` - Con trỏ mảng byte; `n` - Số lượng byte cần tính ($276\text{ bytes}$).
- **Thuật toán:** Đa thức chuẩn `0xEDB88320`, xử lý bit-by-bit với giá trị khởi tạo `0xFFFFFFFF` và đảo bit ngõ ra `~crc`. Đảm bảo tính toán tin cậy mà không tốn 1 KB RAM cho bảng tra cứu.
