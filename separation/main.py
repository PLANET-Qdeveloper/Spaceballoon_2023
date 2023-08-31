from machine import Pin, UART
from utime import ticks_ms, sleep

from PQ_RM92A import RM92A

'''
定数宣言
'''
signal_timing = 1000
cut_timing = 10000

'''
通信関係
'''
# UART通信
rm_uart= UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

'''
クラスからインスタンスを生成
'''
rm = RM92A(rm_uart)

#ピンのインスタンス
p2 = Pin(2, Pin.IN)  #irq用
p15 = Pin(15, Pin.IN)  #フライトピン用
IN1 = Pin(19, Pin.OUT)
IN2 = Pin(18, Pin.OUT)
led = Pin(27, Pin.OUT)

IN1.value(0)
IN2.value(0)


'''
Timerオブジェクト(周期処理用)
'''
irq_called_time = ticks_ms()

'''
変数宣言
'''
phase = 0
command = 48
mission_time = 0

block_irq = True
'''
関数
'''
#分離機構作動
def function():
    
    IN1.value(1)
    IN2.value(0)
    sleep(0.5)
    print(2)
    IN1.value(0)
    IN2.value(1)
    sleep(10)
    print(3)
    IN1.value(1)
    IN2.value(1)
    sleep(3)
    print(4)
    IN1.value(0)
    IN2.value(0)


#アップリンク
def command_handler(p2):
    global block_irq, irq_called_time, phase, command

    rx_buf = bytearray(10)

    if (ticks_ms() - irq_called_time) > signal_timing:
        block_irq = False
    if not block_irq:
        rm_uart.readinto(rx_buf, 4)
        command = rx_buf[0]
        if command == 50:     # 0 READY[1]->SAFETY[0]
            if phase == 1:
                phase = 2       
        irq_called_time = ticks_ms()
        block_irq = True
irq_obj = p2.irq(handler=command_handler, trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING))



def main():
    global phase

    while True:
        print(phase)
        if phase == 0:
            led.value(1)
            sleep(0.1)

            if p15.value() == 1:
                ini_mission_time = ticks_ms()
                print('go')
                phase = 1

        if phase == 1:
            misson_time = (ticks_ms() - ini_mission_time)/1000
            led.value(1)
            sleep(2)
            led.value(0)
            sleep(2)

            if mission_time > cut_timing:
                phase = 2
        
        if phase == 2:
            led.value(1)
            function()
            break

if __name__=='__main__':
    main()
    for i in range(10):
        led.value(1)
        sleep(1)
        led.value(0)
        sleep(1)

