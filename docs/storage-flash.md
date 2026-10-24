# CƠ CHẾ LƯU TRỮ FLASH NỘI & BẢN ĐỒ MÊ CUNG (FLASH PERSISTENCE ARCHITECTURE)

Tài liệu này đặc tả chi tiết kiến trúc lưu trữ dữ liệu bền vững của Robot Micromouse trên bộ nhớ Flash nội (On-chip Flash) của vi điều khiển STM32F411CEU6, bao gồm cấu trúc phân vùng bộ nhớ, định dạng bản ghi, thuật toán phân phối hao mòn (Wear Leveling) và cơ chế phòng chống mất nguồn đột ngột.

---

## 1. Phân vùng Bộ nhớ Flash & Cấu hình Linker Script

Bộ nhớ Flash của vi điều khiển STM32F411CEU6 có tổng dung lượng **512 KB**, được tổ chức thành 8 sector vật lý với kích thước không đồng đều:

| Sector | Địa chỉ bắt đầu | Địa chỉ kết thúc | Kích thước | Mục đích sử dụng trong dự án |
| :---: | :---: | :---: | :---: | :--- |
| **Sector 0** | `0x08000000` | `0x08003FFF` | 16 KB | Bảng vector ngắt (Interrupt Vector Table) & Mã khởi động |
| **Sector 1** | `0x08004000` | `0x08007FFF` | 16 KB | Mã chương trình Firmware (.text, .rodata) |
| **Sector 2** | `0x08008000` | `0x0800BFFF` | 16 KB | Mã chương trình Firmware |
| **Sector 3** | `0x0800C000` | `0x0800FFFF` | 16 KB | Mã chương trình Firmware |
| **Sector 4** | `0x08010000` | `0x0801FFFF` | 64 KB | Mã chương trình Firmware |
| **Sector 5** | `0x08020000` | `0x0803FFFF` | 128 KB | Mã chương trình Firmware |
| **Sector 6** | `0x08040000` | `0x0805FFFF` | 128 KB | Mã chương trình Firmware |
| **Sector 7** | `0x08060000` | `0x0807FFFF` | **128 KB** | **VÙNG DÀNH RIÊNG CHO BẢN ĐỒ MÊ CUNG (`FLASH_MAP_ADDR`)** |

### Cấu hình Linker Script (`STM32F411CEUX_FLASH.ld`):
Để bảo vệ vùng dữ liệu bản đồ không bị ghi đè hay xóa mất khi người dùng nạp lại phần mềm bằng ST-Link SWD, tệp kịch bản liên kết (linker script) đã giới hạn không gian bộ nhớ của chương trình:
```ld
MEMORY
{
  CCMRAM    (xrw)    : ORIGIN = 0x10000000,   LENGTH = 64K
  RAM       (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH      (rx)    : ORIGIN = 0x08000000,   LENGTH = 384K   /* Chỉ sử dụng Sector 0..6 */
}
```
> [!IMPORTANT]
> **Ý nghĩa an toàn:**
> - Dung lượng Flash dành cho mã nhị phân `.elf` bị giới hạn ở mức 384 KB (Sector 0 đến 6). Nếu mã nguồn biên dịch vượt quá 384 KB, trình liên kết (Linker) sẽ báo lỗi tràn bộ nhớ ngay tại thời điểm build.
> - Sector 7 (128 KB) hoàn toàn nằm ngoài không gian nạp code. Khi nạp firmware qua CubeIDE hoặc STM32 Programmer CLI (lệnh `-w`), trình nạp chỉ xóa và ghi các sector từ 0 đến 6; **toàn bộ dữ liệu mê cung lưu trong Sector 7 được bảo toàn nguyên vẹn**.
> - Bản đồ trong Sector 7 chỉ bị xóa khi người dùng thực hiện xóa toàn bộ chip ("Full Chip Erase", cờ `-e all`), hoặc khi chuyển mức bảo vệ RDP (Readout Protection Regression).

---

## 2. Định dạng Bản ghi `FlashMapData`

Mỗi lần lưu trữ, toàn bộ trạng thái của mê cung và của robot được đóng gói vào một cấu trúc dữ liệu nhị phân chuẩn hóa `FlashMapData` có kích thước **280 bytes**:

```c
typedef struct
{
    uint32_t magic_word;            /* 0xA5A5B6B6 (Đánh dấu bản ghi hợp lệ) */
    uint16_t version;               /* Phiên bản định dạng (Hiện tại = 1) */
    uint16_t record_size;           /* Kích thước bản ghi = 280 bytes */
    uint32_t sequence;              /* Số thứ tự lần lưu tăng dần (1, 2, 3...) */
    uint8_t  maze_map[MAZE_CELLS];  /* Ma trận 256 bytes bản đồ mê cung 16x16 */
    uint8_t  last_x;                /* Tọa độ x cuối cùng của xe (0..15) */
    uint8_t  last_y;                /* Tọa độ y cuối cùng của xe (0..15) */
    uint8_t  last_heading;          /* Hướng xe cuối cùng (0:Bắc, 1:Đông, 2:Nam, 3:Tây) */
    uint8_t  phase;                 /* Giai đoạn giải: SEARCH_GOAL, SEARCH_START, SPEED_RUN... */
    uint8_t  rounds;                /* Số vòng khám phá đã chạy */
    uint8_t  reason;                /* Lý do lưu: NEW, GOAL, EXPLORED, LIFTED, ERROR */
    uint8_t  flags;                 /* Cờ trạng thái: HAS_PATH (0x01), EXPLORED (0x02) */
    uint8_t  reserved;              /* Byte đệm dự phòng căn chỉnh 32-bit */
    uint32_t checksum;              /* Mã kiểm tra CRC-32 của 276 bytes đầu */
} FlashMapData;
```

### Các ràng buộc kỹ thuật của cấu trúc:
1. **Căn chỉnh Word 32-bit:** Kích thước `sizeof(FlashMapData) = 280 bytes` chia hết cho 4 ($280 / 4 = 70 \text{ words}$). Điều này được bảo đảm tại thời điểm biên dịch bằng chỉ thị kiểm tra tĩnh:
   ```c
   _Static_assert((sizeof(FlashMapData) % 4U) == 0U, "FlashMapData phai la boi so 4 byte");
   ```
2. **Số Magic nhận dạng:** `FLASH_MAP_MAGIC = 0xA5A5B6B6`. Khi một ô Flash bị xóa trắng, toàn bộ các bit bằng 1 (`0xFFFFFFFF`). Do đó giá trị `0xA5A5B6B6` đảm bảo phân biệt ô trống với ô đã ghi dữ liệu.
3. **Mã toàn vẹn dữ liệu (Checksum CRC-32):**
   - Thuật toán CRC-32 sử dụng đa thức chuẩn nghịch đảo IEEE 802.3 `0xEDB88320`.
   - Tính toán trên đúng 276 bytes đầu tiên của cấu trúc (`offsetof(FlashMapData, checksum)`).
   - Được triển khai dạng vòng lặp bit gọn gàng, không cần bảng tra cứu (Look-up Table) giúp tiết kiệm dung lượng RAM và Flash.

---

## 3. Cơ chế Phân phối Hao mòn (Append-Log Wear Leveling)

Flash của vi điều khiển chỉ có thể chuyển bit từ $1 \to 0$ khi lập trình, và muốn chuyển từ $0 \to 1$ thì phải xóa toàn bộ sector. Mỗi sector có tuổi thọ giới hạn khoảng $10.000$ lần xóa.

Nếu mỗi lần lưu bản đồ đều xóa Sector 7 rồi ghi vào đầu sector:
- Thao tác xóa sector 128 KB mất thời gian rất lớn: từ **1.0 đến 2.0 giây**.
- Gây mòn nhanh ô nhớ đầu tiên và làm tăng nguy cơ mất trắng dữ liệu nếu nguồn điện bị ngắt đúng lúc đang xóa.

### Giải pháp Nhật ký Nối tiếp (Append-Log):
Sector 7 (131.072 bytes) được chia thành **468 slots** kế tiếp nhau:
$$\text{SLOT\_COUNT} = \frac{128 \times 1024}{280} = 468 \text{ slots}$$

```text
Địa chỉ:   0x08060000   0x08060118   0x08060230               0x0807FF18
           +------------+------------+------------+-----+------------+
Sector 7:  |   Slot 0   |   Slot 1   |   Slot 2   | ... |  Slot 467  |
           | (280 byte) | (280 byte) | (280 byte) |     | (280 byte) |
           +------------+------------+------------+-----+------------+
Dữ liệu:   | Bản ghi #1 | Bản ghi #2 |  [TRỐNG]   |     |  [TRỐNG]   |
           | (seq = 1)  | (seq = 2)  | 0xFFFFFFFF |     | 0xFFFFFFFF |
           +------------+------------+------------+-----+------------+
```

### Nguyên lý hoạt động:
1. **Ghi tuần tự:**
   - Mỗi lần lưu, hệ thống quét tìm slot hoàn toàn trống đầu tiên (`Slot_Blank(slot) == true`).
   - Tăng số thứ tự: `rec->sequence = latest_seq + 1`.
   - Ghi bản ghi mới vào slot đó bằng lệnh `HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD)`.
   - Toàn bộ quá trình ghi 70 words chỉ mất **khoảng vài mili-giây** (rất nhanh, không gây gián đoạn).
2. **Xóa khi đầy:**
   - Chỉ khi toàn bộ 468 slots đều đã được ghi đầy (`slot >= SLOT_COUNT`), hệ thống mới tiến hành xóa toàn bộ Sector 7 (`Erase_Sector()`) và ghi vòng lại từ Slot 0.
   - Nhờ cơ chế này, để đạt tới ngưỡng giới hạn 10.000 lần xóa của phần cứng, robot phải thực hiện tới:
     $$10.000 \times 468 = 4.680.000 \text{ lần lưu bản đồ}$$
     Đây là con số đảm bảo tuổi thọ bộ nhớ vĩnh cửu trong thực tế sử dụng.

---

## 4. Khả năng Chống chịu Mất nguồn Đột ngột (Power-loss Resilience)

Trong môi trường thi đấu robot, pin có thể bị tuột hoặc công tắc nguồn bị ngắt bất cứ lúc nào:

### Kịch bản mất nguồn giữa lúc đang ghi:
- Giả sử hệ thống đang ghi bản ghi vào Slot $k$, nguồn bị sụt áp làm quá trình ghi bị ngắt quãng giữa chừng:
  - Một số từ nhớ (words) đầu đã được ghi, nhưng các từ nhớ sau chưa kịp ghi xong hoặc bị sai bit.
  - Khi khởi động lại, hàm `Scan()` duyệt qua Slot $k$:
    - Slot $k$ không còn là `0xFFFFFFFF` nên không được coi là ô trống.
    - Hàm kiểm tra tính toàn vẹn `Rec_Valid(&r)` tính lại CRC-32 của Slot $k$. Do ghi dở dang, mã CRC-32 lưu trong Flash sẽ **không bao giờ khớp** với dữ liệu thực tế.
    - Hàm `Scan()` lập tức đánh dấu Slot $k$ là **không hợp lệ** và bỏ qua.
    - Quá trình quét tiếp tục lùi về Slot $k-1$: Bản ghi tại Slot $k-1$ có mã CRC-32 chuẩn xác sẽ được chọn làm bản đồ khôi phục.
    - Ở lần lưu kế tiếp, vòng lặp `while (!Slot_Blank(slot)) slot++;` sẽ tự động nhảy qua Slot $k$ bị lỗi để ghi vào Slot $k+1$.

> [!NOTE]
> So với cơ chế truyền thống (xóa trước rồi ghi), nếu mất điện đúng lúc đang xóa sector, toàn bộ bản đồ cũ lẫn bản đồ mới đều biến mất hoàn toàn. Cơ chế Append-Log của dự án loại bỏ triệt để nguy cơ này.

---

## 5. Ràng buộc Phần cứng & Tác động Runtime

Vi điều khiển STM32F411 là dòng MCU kiến trúc **Single-Bank Flash** (chỉ có 1 khối Flash duy nhất chứa cả mã thực thi lẫn dữ liệu). 

### Hiện tượng CPU Stalling:
- Trong quá trình xóa sector (kéo dài ~1-2 s) hoặc lập trình Flash (~vài ms), bus Flash nội bộ bị khóa đối với các lệnh đọc lệnh của CPU (Instruction Fetch Stalling).
- CPU bị tạm dừng (stall) hoàn toàn; do đó:
  - Tất cả các chương trình ngắt, bao gồm ngắt điều khiển PID 2 ms (`TIM9`) và ngắt định thời hệ thống `SysTick`, đều bị hoãn thực thi cho tới khi thao tác Flash hoàn tất.
  - Các Timer phần cứng (`TIM1`, `TIM5`) vẫn tiếp tục phát xung PWM với giá trị thanh ghi `CCR` đã nạp trước đó. Nếu xe đang chạy, động cơ sẽ tiếp tục quay mà không có sự kiểm soát của vòng phản hồi PID, gây đâm va nguy hiểm.

### Quy tắc an toàn bắt buộc:
1. **Chỉ lưu khi xe đứng yên:** Tuyệt đối không bao giờ gọi `Flash_SaveMap()` trong khi robot đang chuyển động.
2. **Phanh chủ động trước khi thao tác:** Hàm `Save_Map()` trong `mode_manager.c` luôn gọi `Motor_Stop()` đầu tiên để ngắt PWM và khóa cứng trục động cơ trước khi mở khóa Flash:
   ```c
   static void Save_Map(uint8_t reason)
   {
       Motor_Stop();                   /* 1. Phanh động cơ ngay lập tức */
       Maze_ExportMap(mm_rec.maze_map);/* 2. Đóng gói bản đồ */
       ...
       r = Flash_SaveMap(&mm_rec);     /* 3. Thao tác ghi Flash an toàn */
   }
   ```
3. **Các thời điểm lưu Flash được phép:**
   - Khi đang dò mê cung và lần đầu tiên chạm tới 4 ô đích (`FLASH_SAVE_GOAL`).
   - Khi hoàn tất toàn bộ quá trình khám phá và quay về ô xuất phát (0,0) (`FLASH_SAVE_EXPLORED`).
   - Khi phát hiện robot bị nhấc bổng khỏi sàn hoặc va chạm (`FLASH_SAVE_LIFTED`).
   - Khi phát hiện lỗi chuyển động dừng giữa chừng (`FLASH_SAVE_ERROR`).
   - Khi bắt đầu Mode 3 (Lưu bản đồ trống đè lên bản đồ cũ) (`FLASH_SAVE_NEW`).

---

## 6. Đồng bộ Bộ nhớ đệm dữ liệu (ART Data Cache Flush)

STM32F4 được trang bị bộ tăng tốc bộ nhớ thích nghi (Adaptive Real-Time Memory Accelerator - ART Accelerator) với bộ đệm dữ liệu (Data Cache - `DCEN`).

- Khi CPU đọc một ô nhớ Flash, nội dung ô nhớ được nạp vào Data Cache.
- Nếu sau đó chương trình ghi đè dữ liệu mới vào địa chỉ đó trên Flash, nhưng không vô hiệu hóa (flush) bộ đệm, các lệnh đọc tiếp theo của CPU vẫn có thể lấy dữ liệu cũ từ Cache thay vì đọc dữ liệu mới trên Flash.
- Để khắc phục, hàm `Flash_FlushDataCache()` được gọi bắt buộc sau mọi thao tác ghi hoặc xóa:
  ```c
  static void Flash_FlushDataCache(void)
  {
      if ((FLASH->ACR & FLASH_ACR_DCEN) != 0U) {
          __HAL_FLASH_DATA_CACHE_DISABLE();
          __HAL_FLASH_DATA_CACHE_RESET();
          __HAL_FLASH_DATA_CACHE_ENABLE();
      }
  }
  ```
- Sau khi xả Cache, hàm `Flash_SaveMap()` thực hiện thao tác kiểm tra chéo `memcmp()` giữa vùng nhớ Flash vừa ghi với con trỏ dữ liệu gốc `rec`. Chỉ khi toàn bộ 280 bytes khớp nhau từng bit, hàm mới trả về `FLASH_OK`.
