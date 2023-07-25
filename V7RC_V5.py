from machine import RTC, UART
from utime import sleep_ms

def control_motor(pwm): # pwm = [pwm1,pwm2,pwm3,pwm4] , 0~100
    pass

def PWM_callback(bytearray_data):
    pwm = []
    try:
        for index in range(3,16,4):
            pwm.append( int(bytes(bytearray_data[index:index+4]))//10 - 150)
    except:
        return
    if pwm:
        control_motor(pwm)
        print('PWM4=>',pwm)

def control_motor8(pwm): # pwm = [pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8] , 0~100
    pass

def PWM8_callback(bytearray_data):
    pwm8 = []
    for index in range(3,18,2):
        pwm8.append( int(bytes(bytearray_data[index:index+2]),16)-150)
    if pwm8 :
        control_motor8(pwm8)
        print('PWM8=>',pwm8)

def control_led(led_color): # led_color = [[ON/OFF,R,G,B,],[ON/OFF,R,G,B,],[ON/OFF,R,G,B,],[ON/OFF,R,G,B,]]
    pass

def LED_callback(bytearray_data):
    led_color = []
    for index in range(3,16,4):
        temp=bytearray_data[index:index+4]
        color =[]
        if temp[3] == ord('A'):
            color.append(True)  
        else:
            color.append(False) 
        for i in range(3):
            if temp[i] == ord('0'):
                color.append(0)
            else:
                color.append((int(chr(temp[i]),16)+1)*16-1)
        led_color.append(color)
    if led_color:
        control_led(led_color)
        print('LED=>',led_color)


def procress_V7RC_cmd(data, mark=b'SRT', length=19,callback_proc=None):
    search_index = 0
    while search_index >= 0:
        start_index = bytes(data).find(mark, search_index)
        # print('start', start_index)
        if start_index >= 0:
            end_index = bytes(data).find(b'#', start_index)
            # print('end', end_index)
            if end_index >= 0:
                if end_index-start_index == length:
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

while True:
    rl62m_read_data()
    procress_V7RC_cmd(rx_buffer,mark = b'SRT',length = 19, callback_proc = PWM_callback)
    procress_V7RC_cmd(rx_buffer,mark = b'LED',length = 19,callback_proc=LED_callback)
    procress_V7RC_cmd(rx_buffer,mark = b'LE2',length = 19,callback_proc=LED_callback)
    procress_V7RC_cmd(rx_buffer,mark = b'SRV',length = 19,callback_proc = PWM_callback)
    procress_V7RC_cmd(rx_buffer,mark = b'SS8',length = 19,callback_proc=PWM8_callback)
    sleep_ms(50)
