# SƠ ĐỒ TRÌNH TỰ TƯƠNG TÁC (SEQUENCE DIAGRAMS)

Tài liệu này cung cấp các sơ đồ tuần tự (Sequence Diagrams) trực quan hóa thứ tự tương tác theo thời gian giữa các thành phần phần cứng, hàm ngắt (ISR), các module phần mềm và máy tính bên ngoài trong các kịch bản vận hành trọng yếu.

---

## 1. Chu kỳ ngắt điều khiển thời gian thực 2 ms (TIM9 ISR)

Sơ đồ này mô tả chính xác những gì xảy ra mỗi 2 ms bên trong hàm ngắt `HAL_TIM_PeriodElapsedCallback` và callback I2C DMA.

```mermaid
sequenceDiagram
    autonumber
    participant HW_TIM as Timer phần cứng TIM9
    participant ISR as HAL_TIM_PeriodElapsedCallback (main.c)
    participant PID as Motor_Speed_Tick (pid.c)
    participant ADC as IR_Tick (adc.c)
    participant MOT as Motion_Update (pid.c)
    participant CTRL as Motor_Speed_Control (pid.c)
    participant MPU as MPU6500_Tick (mpu6500.c)
    participant DMA as DMA1 Stream0 (I2C1 RX)
    participant I2C_CB as HAL_I2C_MemRxCpltCallback (mpu6500.c)
    participant SAFE as Check_Robot_Picked_Up (mode_manager.c)

    HW_TIM->>ISR: Ngắt chu kỳ 2ms (TIM1_BRK_TIM9_IRQn)
    activate ISR

    %% 1. Đọc encoder
    ISR->>PID: Motor_Speed_Tick()
    activate PID
    PID->>PID: Đọc TIM3->CNT (trái) & TIM2->CNT (phải)
    PID->>PID: Tính delta xung, speed_left/right_mms, quãng đường tích lũy & Odometry
    
    PID->>ADC: IR_Tick()
    activate ADC
    ADC->>ADC: Đọc adc_value[4], lọc Median-of-3 + LPF IIR
    ADC->>ADC: Tra bảng nội suy FL->FR và L->R
    ADC-->>PID: Hoàn tất cập nhật IR_*_Filt
    deactivate ADC

    PID->>MOT: Motion_Update()
    activate MOT
    MOT->>MOT: Tính toán profile (v, w), PD hướng gyro, cân giữa ô
    MOT-->>PID: Cập nhật target_speed_left, target_speed_right
    deactivate MOT

    PID->>CTRL: Motor_Speed_Control(delta_left, delta_right)
    activate CTRL
    CTRL->>CTRL: Ramp giới hạn gia tốc 2000 mm/s²
    CTRL->>CTRL: Feedforward (Ks, Kf, Ka) + PI bánh xe (Anti-windup)
    CTRL->>PID: Motor_SetPWM(pwm_left, pwm_right)
    PID->>PID: Ghi giá trị CCR vào thanh ghi TIM1 & TIM5
    CTRL-->>PID: Hoàn tất điều khiển động cơ
    deactivate CTRL
    PID-->>ISR: Hoàn tất Motor_Speed_Tick
    deactivate PID

    %% 2. Kích hoạt đọc IMU DMA
    ISR->>MPU: MPU6500_Tick()
    activate MPU
    MPU->>DMA: HAL_I2C_Mem_Read_DMA(hi2c1, 0x68, 0x3B, ..., 14 bytes)
    MPU-->>ISR: Hoàn tất kích hoạt (Non-blocking)
    deactivate MPU

    %% 3. Giám sát an toàn
    ISR->>SAFE: Check_Robot_Picked_Up()
    activate SAFE
    SAFE->>SAFE: Kiểm tra cờ robot_lifted, Encoder overspeed, IR vanish
    SAFE-->>ISR: Trả về pickup_reason (0 = An toàn)
    deactivate SAFE

    deactivate ISR

    %% DMA I2C Callback sau đó
    Note over DMA,I2C_CB: Sau khoảng ~0.4ms truyền nhận DMA I2C xong
    DMA->>I2C_CB: Ngắt DMA1_Stream0_IRQHandler -> HAL_I2C_MemRxCpltCallback
    activate I2C_CB
    I2C_CB->>I2C_CB: MPU6500_Process_Data(dt)
    I2C_CB->>I2C_CB: Lọc bù Pitch/Roll (tau=0.5s), Tích phân Yaw (không deadband)
    I2C_CB->>I2C_CB: Nhận biết đứng yên & Tự bám trôi Gyro Z offset
    I2C_CB->>I2C_CB: Giám sát nhấc xe (Lift_Update) & Va chạm (Crash_Update)
    deactivate I2C_CB
```

---

## 2. Lựa chọn Chế độ qua IR trái & Còi Buzzer

Sơ đồ thể hiện chuỗi tương tác phi khóa (non-blocking) giữa người dùng, cảm biến quang học, máy trạng thái và bộ nhớ Flash.

```mermaid
sequenceDiagram
    autonumber
    actor User as Người dùng
    participant IR as Mắt thu IR trái (adc_value[1])
    participant MM as ModeManager_Update (mode_manager.c)
    participant BUZ as Buzzer_Update (mode_manager.c)
    participant FLS as Flash_LoadMap (flash_storage.c)
    participant MAZE as solving_maze.c
    participant UART as Cổng UART1 (HC-05)

    User->>IR: Đưa tay che sát mắt IR trái
    Note over MM: Chu kỳ while(1) gọi ModeManager_Update()
    MM->>IR: UI_Pressed(): Kiểm tra adc_value[1] > 2000 duy trì 30ms
    IR-->>MM: Trả về true (Lần 1)
    
    MM->>MM: ui_count = 1, Chuyển sang STATE_UI_BEEP
    MM->>BUZ: Phát âm bíp xác nhận (UI_BEEP_MS = 250ms)
    BUZ-->>User: Còi PB8 kêu "Bíp"

    User->>IR: Rút tay ra (< 1500) rồi che lần 2
    MM->>IR: UI_Pressed(): Nhận diện lần 2
    IR-->>MM: Trả về true (Lần 2)
    MM->>MM: ui_count = 2
    MM->>BUZ: Xếp hàng phát thêm 1 tiếng bíp cách 300ms
    BUZ-->>User: Còi PB8 kêu "Bíp" lần 2

    Note over MM: Người dùng không che nữa. Còi tắt, chuyển sang STATE_UI_WINDOW
    Note over MM: Đếm ngược cửa sổ 3 giây (UI_WINDOW_MS = 3000ms)
    
    MM->>MM: Hết 3 giây -> Gọi Select_Mode(ui_count = 2)
    MM->>FLS: Check_Mode(MODE_2): Kiểm tra Flash có bản đồ không
    FLS-->>MM: FLASH_OK, cờ FLASH_MAP_HAS_PATH = 1
    MM->>MAZE: Maze_ImportMap(mm_rec.maze_map)
    MAZE-->>MM: true & Maze_HasPath() == true (Bản đồ hợp lệ)

    MM->>UART: In: "Che 2 lan: MODE 2 (chay nhanh + quay ve)"
    MM->>MM: Chuyển sang STATE_START
    Note over MM: Chờ 0.5s bỏ tay + xe nằm trên sàn
    MM->>MM: Đo lại offset Gyro Z tĩnh (0.5s)
    MM->>MAZE: Maze_StartSpeedRun()
    MM->>MM: Chuyển sang STATE_MODE2_RUN (Robot bắt đầu bứt tốc)
```

---

## 3. Đi thẳng liên tục & Tính toán lại lộ trình khi đang chạy

Sơ đồ mô tả quy trình di chuyển gộp nhiều ô thẳng, đọc cảm biến trước tâm ô 90 mm và kéo dài hoặc rút ngắn quãng đường mà không dừng xe.

```mermaid
sequenceDiagram
    autonumber
    participant MAZE as solving_maze.c
    participant PID as pid.c (Chuyển động)
    participant TIM9 as Ngắt TIM9 (Real-time)
    participant IR as Cảm biến IR (adc.c)

    Note over MAZE: Đang ở tâm ô, quyết định đi thẳng 3 ô liên tiếp (n = 3)
    MAZE->>PID: Move_Straight_Begin(3 * 180 mm = 540 mm, speed = 300 mm/s)
    activate PID
    PID->>PID: Thiết lập MOTION_STRAIGHT, tính profile tăng tốc
    PID-->>MAZE: Trả về ngay (Non-blocking)
    deactivate PID
    Note over MAZE: Chuyển sang trạng thái MAZE_STATE_MOVE_STRAIGHT

    loop Trong khi xe đang di chuyển
        MAZE->>PID: dist = Move_Straight_Traveled()
        Note over MAZE: Xe chạy tới biên ô k=1 (dist >= 180 - 90 = 90 mm)
        
        MAZE->>IR: Straight_SenseCell(1): Đọc mắt bên & mắt trước
        IR-->>MAZE: Trả về IR_L_Filt, IR_R_Filt, TB(FL,FR)
        MAZE->>MAZE: Cập nhật vách ô 1 vào maze_map[][]
        
        MAZE->>MAZE: Chạy Flood_Fill(phase, optimistic)
        MAZE->>MAZE: Straight_Replan(1): Đánh giá hướng đi từ ô 1
        
        alt Đường đi vẫn thẳng tiếp tục thêm 2 ô (Tổng thành 4 ô)
            MAZE->>PID: Move_Straight_SetTarget(4 * 180 mm = 720 mm)
            Note over PID: Kéo dài mục tiêu, xe giữ nguyên tốc độ cao không giảm tốc
        else Phía trước ô 2 phát hiện tường chắn (Chỉ đi được 2 ô)
            MAZE->>PID: Move_Straight_SetTarget(2 * 180 mm = 360 mm)
            Note over PID: Thu hẹp mục tiêu, xe chủ động hãm tốc vừa đủ dừng ở tâm ô 2
        end
    end

    PID->>MAZE: Motion_Status() == MOTION_STATUS_DONE
    MAZE->>MAZE: Straight_Finish(): Cập nhật tọa độ ô hiện tại
    Note over MAZE: Chuyển về MAZE_STATE_DECIDE_PATH để chuẩn bị rẽ
```

---

## 4. Tự căn Vuông góc Tường trước (`Wall_Align`)

Sơ đồ thể hiện chuỗi các bước vi chỉnh góc lái, đo đạc độ nhạy quang học và thiết lập lại mốc tọa độ quán tính con quay khi đối diện với tường phẳng.

```mermaid
sequenceDiagram
    autonumber
    participant MAIN as solving_maze.c / main
    participant API as Wall_Align_Begin / Poll (pid.c)
    participant TIM9 as Hold_Update (Ngắt TIM9)
    participant ADC as adc.c (Cảm biến IR)
    participant MPU as mpu6500.c (Con quay Gyro)

    MAIN->>API: Wall_Align_Begin()
    API->>TIM9: Thiết lập MOTION_HOLD, ws = WS_IDLE, ws_force = 1
    
    loop Mỗi 2ms trong ngắt TIM9 (Hold_Update)
        alt Khoảng cách chưa đúng (Mắt trước > 1725 hoặc < 1640)
            TIM9->>TIM9: Điều khiển xe bò chậm 30 mm/s tới WALL_TARGET_IR = 1700
        else Đã đạt cự ly & Đứng yên 100ms
            TIM9->>TIM9: ws = WS_MEAS
            TIM9->>ADC: Đo trung bình 25 mẫu: r = (FL - FR)/(FL + FR) - WALL_R_OFFSET
            
            alt Lần đầu căn (Độ nhạy g chưa biết)
                TIM9->>TIM9: Quay thử 4° (WALL_PROBE_DEG)
                TIM9->>MPU: Đọc delta heading thực tế từ Gyro
                TIM9->>TIM9: Tính độ nhạy g = delta_r / delta_heading
            end

            TIM9->>TIM9: Tính góc quay sửa: step = -r / g
            
            alt |step| >= 0.5° (Chưa vuông góc)
                TIM9->>TIM9: Quay tại chỗ góc step (Tối thiểu 40°/s)
                TIM9->>TIM9: Quay lại đo tiếp ở WS_MEAS (Tối đa 5 lần)
            else |step| < 0.5° (Đã vuông góc hoàn hảo)
                TIM9->>TIM9: ws = WS_ZERO
                TIM9->>MPU: MPU6500_ZeroGyroZ (Đo 250 mẫu tính lại offset Z)
                TIM9->>MPU: Wall_SetHeading(): Gán hướng vuông góc = Hướng đích
                TIM9->>TIM9: ws = WS_DONE
            end
        end
    end

    MAIN->>API: Wall_Align_Poll()
    API-->>MAIN: WALL_ALIGN_OK (Hoàn tất căn chỉnh an toàn)
```

---

## 5. Phát hiện Nhấc xe, Tạm dừng & Khôi phục Chuyển động

Sơ đồ mô tả quy trình phản ứng tức thời của firmware khi có sự cố cơ học (nhấc bổng robot hoặc va đập vào chướng ngại vật).

```mermaid
sequenceDiagram
    autonumber
    actor User as Người dùng / Chướng ngại
    participant TIM9 as Ngắt TIM9 (Real-time)
    participant MPU as Lift_Update / Crash_Update
    participant MOT as pid.c (Động cơ)
    participant MM as ModeManager_Update (mode_manager.c)
    participant FLS as Flash_SaveMap (flash_storage.c)
    participant UART as Cổng UART1 (HC-05)

    User->>MPU: Nhấc bổng robot lên khỏi sàn (Nghiêng >20° hoặc Vz >0.12m/s)
    Note over TIM9: Trong ngắt 2ms: MPU6500_Process_Data gọi Lift_Update()
    MPU->>MPU: Điều kiện duy trì liên tục 200ms -> Bật robot_lifted = 1
    MPU->>MOT: Motion_OnLifted():
    MOT->>MOT: Khóa chặt PWM về 0 (Phanh điện tử)
    MOT->>MOT: Tắt đèn LED báo chạy PB15
    MOT->>MOT: Hủy mọi lệnh chuyển động đang chờ

    Note over MM: Vòng lặp chính ModeManager_Update() phát hiện robot_lifted != 0
    MM->>MM: Chuyển sang STATE_PAUSED
    MM->>UART: In: "TAM DUNG (nhac xe: gia toc): dat lai o (x,y) mui huong ..."
    MM->>FLS: Save_Map(FLASH_SAVE_LIFTED) -> Ghi lưu tọa độ vào Flash Sector 7
    MM->>MM: Chụp ảnh mốc cảm biến tĩnh: Still_Restart()

    User->>User: Đặt lại robot xuống sàn ở đúng ô (x,y) và hướng nhìn
    
    loop Đếm tĩnh 3 giây
        MM->>MM: Still_Holding(): Kiểm tra rung lắc gia tốc <= 4mg, bánh lăn <= 1mm
        alt Có rung động / Người dùng còn chạm tay
            MM->>MM: Still_Restart(): Đếm lại từ đầu 3 giây
        else Yên tĩnh liên tục đủ 3 giây và MPU xác nhận đã nằm yên
            Note over MM: Điều kiện đặt xuống thỏa mãn
        end
    end

    MM->>MM: Chuyển sang STATE_RESUME
    MM->>UART: In: "CHAY TIEP"
    MM->>MOT: Bật lại đèn LED PB15
    MM->>MM: Khôi phục lại trạng thái đang chạy dở (STATE_EXPLORE / STATE_MODE2_RUN)
```

---

## 6. Lưu trữ và Nạp bản đồ từ Flash Sector 7

Sơ đồ mô tả cơ chế ghi tuần tự (Append-Log Wear Leveling) và đọc bản ghi mới nhất từ bộ nhớ Flash on-chip.

```mermaid
sequenceDiagram
    autonumber
    participant APP as Ứng dụng (Mode Manager / Maze)
    participant FLS as flash_storage.c
    participant HAL as HAL Flash Driver
    participant MEM as Bộ nhớ Flash Sector 7 (0x08060000)

    %% GHI FLASH
    APP->>FLS: Flash_SaveMap(&rec)
    activate FLS
    FLS->>FLS: Điền magic_word=0xA5A5B6B6, version=1, record_size=280
    FLS->>FLS: Crc32(): Tính mã CRC trên 276 bytes đầu
    FLS->>FLS: Scan(): Tìm slot trống đầu tiên (slot) và số sequence lớn nhất
    FLS->>FLS: rec->sequence = latest_seq + 1

    alt Sector 7 đã đầy (slot >= 468)
        FLS->>HAL: HAL_FLASH_Unlock()
        FLS->>HAL: HAL_FLASHEx_Erase(Sector 7, VoltageRange 3)
        HAL->>MEM: Xóa trắng toàn bộ 128 KB (Mất ~1-2s)
        FLS->>FLS: slot = 0
    else Còn slot trống
        FLS->>HAL: HAL_FLASH_Unlock()
    end

    loop Ghi 70 words (280 bytes)
        FLS->>HAL: HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, word)
        HAL->>MEM: Ghi 32-bit vào ô nhớ Flash
    end

    FLS->>HAL: HAL_FLASH_Lock()
    FLS->>FLS: Flash_FlushDataCache(): Reset ART Data Cache
    FLS->>MEM: memcmp(): Đọc lại 280 bytes so sánh đối chiếu
    MEM-->>FLS: Trùng khớp 100%
    FLS-->>APP: FLASH_OK
    deactivate FLS

    %% ĐỌC FLASH
    APP->>FLS: Flash_LoadMap(&out)
    activate FLS
    FLS->>MEM: Scan(): Dò tìm từ ô trống ngược về trước
    FLS->>MEM: Đọc bản ghi và xác thực: Magic, Version, Size, Checksum CRC-32
    MEM-->>FLS: Bản ghi slot i hợp lệ (Mới nhất)
    FLS->>APP: memcpy(&out, slot_addr, 280) -> FLASH_OK
    deactivate FLS
```

---

## 7. Đồng bộ Bản đồ Mê cung lên Máy tính qua Bluetooth HC-05

Sơ đồ mô tả giao thức truyền nhận văn bản ASCII có checksum 16-bit giữa vi điều khiển và phần mềm Python `maze_viewer.py`.

```mermaid
sequenceDiagram
    autonumber
    participant GUI as maze_viewer.py (Tkinter UI)
    participant THREAD as SerialWorker Thread
    participant HC05 as Module HC-05 (COM8)
    participant STM as Firmware Robot (solving_maze.c)

    GUI->>THREAD: Người dùng bấm nút "Xin bản đồ" (Lệnh cmd_q.put('request'))
    THREAD->>HC05: Gửi byte 'M' qua cổng COM8
    HC05->>STM: Ký tự 'M' đến thanh ghi USART1->DR

    Note over STM: Đang ở STATE_IDLE, gọi Serve_MapRequest()
    STM->>STM: UART_TryRead() nhận được 'M'
    STM->>STM: Maze_SendMap(): Bắt đầu đóng gói và gửi

    STM->>HC05: Dòng 1: "#MAZE_BEGIN 16\r\n"
    STM->>HC05: Dòng 2: "#STATE 6 3 2\r\n"
    STM->>HC05: Dòng 3: "#ROBOT 0 0 0\r\n"
    STM->>HC05: Dòng 4: "#STATS 24 156 18 2 12 3 1 45\r\n"
    
    loop 16 dòng bản đồ (y = 15 về 0)
        STM->>HC05: "#ROW y <32 ký tự Hex biểu diễn 16 ô>\r\n"
    end
    
    STM->>HC05: Dòng cuối: "#MAZE_END <sum % 65536>\r\n"
    
    HC05->>THREAD: Nhận toàn bộ khối văn bản (~800 bytes)
    THREAD->>THREAD: Phân tích cú pháp Regex bóc tách các dòng
    THREAD->>THREAD: Tính tổng các byte nhận được và so với checksum dòng #MAZE_END
    
    alt Checksum khớp 100%
        THREAD->>THREAD: Lưu tệp log: maze/logs/maze_YYYYMMDD_HHMMSS.txt
        THREAD->>GUI: Đẩy sự kiện qua hàng đợi evt_q: ('map', maze_data)
        GUI->>GUI: Vẽ lại toàn bộ mê cung trên Canvas:
        Note over GUI: - Tường đã biết: Màu đỏ đậm<br/>- Cạnh chưa biết: Nét đứt xám<br/>- Ô chưa tới: Nền xám<br/>- Đường BFS tối ưu: Đường xanh dương<br/>- Vị trí xe: Mũi tên màu cam
    else Checksum không khớp
        THREAD->>GUI: Báo log "Lỗi Checksum, gửi lại sau 3s"
        THREAD->>HC05: Tự động gửi lại ký tự 'M'
    end
```
