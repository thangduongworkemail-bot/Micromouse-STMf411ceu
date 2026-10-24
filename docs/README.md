# TÀI LIỆU KỸ THUẬT HỆ THỐNG FIRMWARE ROBOT MICROMOUSE (STM32F411CEU6)

Tài liệu này cung cấp cái nhìn toàn diện, chính xác và có hệ thống về kiến trúc phần mềm, các thuật toán điều khiển, luồng dữ liệu, các máy trạng thái và cơ chế lưu trữ của dự án Robot Micromouse (phiên bản mã nguồn hiện tại).

---

## 1. Mục lục tài liệu

Bộ tài liệu kỹ thuật trong thư mục `docs/` bao gồm các chuyên đề sau:

1. [**Kiến trúc hệ thống (`docs/architecture.md`)**](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/docs/architecture.md)
   - Kiến trúc phân tầng (Layered Architecture): Phần cứng -> Trình điều khiển ngoại vi (HAL) -> Tầng điều khiển thời gian thực (Real-time Control) -> Tầng chuyển động & PID -> Tầng an toàn & Chế độ hoạt động (Mode Manager & Safety) -> Tầng thuật toán giải mê cung (Maze Solver) -> Tầng giao tiếp & Lưu trữ.
   - Bảng phân công chân vi điều khiển (Pinout), Timers, DMA channels, NVIC Interrupt Priorities.
   - Mô tả trách nhiệm của từng module mã nguồn.
   - Sơ đồ kiến trúc tổng thể bằng Mermaid.

2. [**Đặc tả luồng dữ liệu (`docs/data-flow.md`)**](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/docs/data-flow.md)
   - Truy vết chi tiết 6 luồng dữ liệu chính xuyên suốt từ ngõ vào cảm biến, qua các khâu xử lý/lọc số, đến ngõ ra điều khiển chấp hành hoặc bộ nhớ:
     - Luồng 1: Cảm biến khoảng cách hồng ngoại (IR Sensors).
     - Luồng 2: Cảm biến góc quay & Quán tính IMU (MPU6500).
     - Luồng 3: Đo phản hồi Encoder & Vòng lặp PID vận tốc bánh xe.
     - Luồng 4: Dữ liệu bản đồ & Kế hoạch di chuyển Flood-Fill.
     - Luồng 5: Cơ chế lưu trữ và phục hồi bản đồ Flash Sector 7.
     - Luồng 6: Giao tiếp truyền thông Bluetooth Telemetry & Giao diện máy tính.
   - Sơ đồ luồng dữ liệu DFD (Data Flow Diagrams) chi tiết bằng Mermaid.

3. [**Quy trình nghiệp vụ & Máy trạng thái (`docs/business-flows.md`)**](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/docs/business-flows.md)
   - Trình tự khởi động an toàn của hệ thống (Safe Boot Sequence).
   - Máy trạng thái quản lý chế độ (Mode Manager): Giao diện nút bấm ảo IR trái, phản hồi còi buzzer, 3 chế độ chạy (Mode 1: Dò tiếp bản đồ cũ; Mode 2: Bứt tốc chạy nhanh; Mode 3: Dò mê cung mới).
   - Thuật toán giải mê cung Flood-Fill liên tục: Cơ chế gộp ô đi thẳng và tự động điều chỉnh đích khi đang di chuyển (Mid-run Re-planning).
   - Thuật toán tự căn vuông góc với tường trước (`Wall_Align`) và tái thiết lập gốc tọa độ MPU.
   - Thuật toán cua mượt 90 độ (Smooth 90° Curve Turn với profile độ cong $\sin^2$).
   - Hệ thống phát hiện bất thường & Bảo vệ an toàn: Phát hiện nhấc xe (Tilt/Vz/Tilt Rate), mất tải bánh xe (Encoder Overspeed), mất tường đột ngột, và va chạm (Jerk Crash Detection).
   - Quy trình tạm dừng (Pause) và khôi phục chuyển động an toàn (Resume).

4. [**Tham chiếu chi tiết hàm (`docs/function-reference.md`)**](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/docs/function-reference.md)
   - Tra cứu đầy đủ các hàm quan trọng trong từng module (`main.c`, `mode_manager.c`, `solving_maze.c`, `pid.c`, `mpu6500.c`, `adc.c`, `flash_storage.c`).
   - Mục đích, ngữ cảnh thực thi (Thread context vs ISR context), tham số vào/ra, dữ liệu truy cập, trình tự các bước thực thi chi tiết, xử lý lỗi và tác động phần cứng.

5. [**Sơ đồ trình tự tương tác (`docs/sequence-diagrams.md`)**](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/docs/sequence-diagrams.md)
   - Các sơ đồ tương tác thời gian thực tuần tự (Mermaid Sequence Diagrams):
     - Vòng lặp điều khiển thời gian thực chu kỳ 2 ms (TIM9 ISR).
     - Tương tác người dùng chọn chế độ qua IR trái và phản hồi còi Buzzer.
     - Luồng đi thẳng liên tục kết hợp đọc cảm biến và tính toán lại đường đi.
     - Luồng dừng trước tường và tự căn chỉnh vuông góc.
     - Luồng xử lý sự cố nhấc xe / va chạm và quy trình khôi phục tiếp tục chạy.
     - Luồng gửi bản đồ lên máy tính qua Bluetooth HC-05.

6. [**Lưu trữ Flash & Bản đồ mê cung (`docs/storage-flash.md`)**](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/docs/storage-flash.md)
   - Kiến trúc phân vùng bộ nhớ STM32F411 Flash (Sector 7: 0x08060000 - 0x0807FFFF, 128 KB độc lập với mã nạp 384 KB).
   - Cấu trúc bản ghi `FlashMapData` (280 bytes, 32-bit aligned).
   - Cơ chế ghi nhật ký nối tiếp (Append-Log Wear Leveling) với 468 slots ghi tuần tự.
   - Thuật toán kiểm tra tính toàn vẹn CRC-32 (đa thức `0xEDB88320`).
   - Xử lý mất nguồn giữa chừng (Power-loss resilience) và cơ chế xóa Sector tự động khi đầy.

7. [**Tích hợp phần cứng & Tham số hiệu chuẩn (`docs/hardware-integration.md`)**](file:///c:/Users/ADMIN/STM32CubeIDE/workspace_1.19.0/version1/docs/hardware-integration.md)
   - Cấu hình Clock Tree (25 MHz HSE -> PLL -> 60 MHz SYSCLK).
   - Mô hình động học robot vi sai, kích thước cơ khí, đường kính bánh xe hiệu dụng.
   - Đặc tính quang học và bảng nội suy tuyến tính cân bằng cảm biến IR (FL -> FR, L -> R).
   - Bảng tổng hợp các thông số hiệu chuẩn quan trọng (`#define`) và các lưu ý phần cứng sống còn (Hardware Gotchas).

---

## 2. Thông tin nền tảng phần cứng & Môi trường biên dịch

- **Vi điều khiển:** STM32F411CEU6 (ARM Cortex-M4F với FPU đơn chính xác, xung nhịp 60 MHz, Flash 512 KB, SRAM 128 KB).
- **Môi trường phát triển:** STM32CubeIDE 1.19.0 / CubeMX 6.15.0.
- **Thư viện firmware:** STM32F4xx HAL Driver V1.28.3.
- **Cấu hình biên dịch:**
  - Compiler: GCC 13.3 ARM Embedded.
  - Floating Point: Hardware FPU (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`).
  - Standard: C11 GNU (`-std=gnu11`).
  - C-Library: Newlib-nano có cờ hỗ trợ in số thực (`-u _printf_float`).
