# MicroPython 優化版本 - 減少記憶體分配與碎片化
from machine import RTC, UART
from utime import sleep_ms
import gc  # 垃圾回收管理

# 預分配全域緩衝區，避免在 callback 中重複建立 list
# 這些緩衝區會被重複使用，減少記憶體分配次數
pwm_buffer = [0, 0, 0, 0]  # PWM_callback 使用
pwm8_buffer = [0, 0, 0, 0, 0, 0, 0, 0]  # PWM8_callback 使用
led_buffer = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]  # LED_callback 使用

def control_motor(pwm): # pwm = [pwm1,pwm2,pwm3,pwm4] , 0~100
    pass

def PWM_callback(bytearray_data):
    # 使用預分配的 pwm_buffer，避免建立新 list
    # 移除 bytes() 轉換，直接使用 memoryview 減少記憶體複製
    try:
        mv = memoryview(bytearray_data)  # memoryview 不會複製資料
        idx = 0
        for index in range(3, 16, 4):
            # 直接從 bytearray 讀取，不用 bytes() 轉換
            # 手動組合 4 bytes 為整數 (big-endian)
            val = (mv[index] << 24) | (mv[index+1] << 16) | (mv[index+2] << 8) | mv[index+3]
            pwm_buffer[idx] = val // 10 - 150
            idx += 1
    except:
        return
    control_motor(pwm_buffer)
    print('PWM4=>', pwm_buffer)

def control_motor8(pwm): # pwm = [pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8] , 0~100
    pass

def PWM8_callback(bytearray_data):
    # 使用預分配的 pwm8_buffer，避免建立新 list
    # 移除 bytes() 轉換，直接處理 bytearray
    mv = memoryview(bytearray_data)
    idx = 0
    for index in range(3, 18, 2):
        # 直接從 bytearray 讀取兩個 ASCII 十六進制字符並轉換
        # 避免 bytes() 和字串轉換
        high = mv[index]
        low = mv[index + 1]
        # 將 ASCII 十六進制字符轉為數值: '0'-'9' (48-57), 'A'-'F' (65-70)
        high_val = (high - 48) if high <= 57 else (high - 55)
        low_val = (low - 48) if low <= 57 else (low - 55)
        pwm8_buffer[idx] = (high_val << 4) | low_val - 150
        idx += 1
    control_motor8(pwm8_buffer)
    print('PWM8=>', pwm8_buffer)

def control_led(led_color): # led_color = [[ON/OFF,R,G,B,],[ON/OFF,R,G,B,],[ON/OFF,R,G,B,],[ON/OFF,R,G,B,]]
    pass

def LED_callback(bytearray_data):
    # 使用預分配的 led_buffer，避免在循環中重複建立 list
    # 移除字串轉換，直接用整數運算處理 ASCII 十六進制字符
    mv = memoryview(bytearray_data)
    buf_idx = 0
    for index in range(3, 16, 4):
        # 直接存取預分配的 led_buffer[buf_idx]
        # 第4個 byte 判斷 ON/OFF (ASCII 'A' = 65)
        led_buffer[buf_idx][0] = 1 if mv[index + 3] == 65 else 0
        
        # 處理 RGB 三個值，避免 chr() 和字串轉換
        for i in range(3):
            byte_val = mv[index + i]
            if byte_val == 48:  # ASCII '0' = 48
                led_buffer[buf_idx][i + 1] = 0
            else:
                # ASCII 十六進制字符轉數值: '1'-'9' (49-57), 'A'-'F' (65-70)
                hex_val = (byte_val - 48) if byte_val <= 57 else (byte_val - 55)
                led_buffer[buf_idx][i + 1] = (hex_val + 1) * 16 - 1
        buf_idx += 1
    
    control_led(led_buffer)
    print('LED=>', led_buffer)


def procress_V7RC_cmd(data, mark=b'SRT', length=19, callback_proc=None):
    # 移除不必要的 bytes() 轉換 - bytearray 本身就支援 find() 方法
    # 每次 bytes(data) 都會複製整個 96 bytes 緩衝區，造成記憶體浪費
    search_index = 0
    while search_index >= 0:
        start_index = data.find(mark, search_index)  # 直接使用 bytearray.find()
        # print('start', start_index)
        if start_index >= 0:
            end_index = data.find(b'#', start_index)  # 直接使用 bytearray.find()
            # print('end', end_index)
            if end_index >= 0:
                if end_index - start_index == length:
                    callback_proc(data[start_index:end_index])
            search_index = end_index
        else:
            break

rx_buffer = bytearray(96)
def rl62m_read_data():
    if uart.any() >= 32:
        uart.readinto(rx_buffer, uart.any())
    else:
        pass
        # print("No data")

uart = UART(1, 115200, timeout=20, read_buf_len=96)

# 主循環 - 每 50ms 執行一次
while True:
    rl62m_read_data()
    procress_V7RC_cmd(rx_buffer, mark=b'SRT', length=19, callback_proc=PWM_callback)
    procress_V7RC_cmd(rx_buffer, mark=b'LED', length=19, callback_proc=LED_callback)
    procress_V7RC_cmd(rx_buffer, mark=b'LE2', length=19, callback_proc=LED_callback)
    procress_V7RC_cmd(rx_buffer, mark=b'SRV', length=19, callback_proc=PWM_callback)
    procress_V7RC_cmd(rx_buffer, mark=b'SS8', length=19, callback_proc=PWM8_callback)
    sleep_ms(50)
    gc.collect()  # 定期進行垃圾回收，避免記憶體碎片化導致當機
