from machine import Pin, I2C, PWM, SPI, Timer
from utime import sleep, ticks_ms
import math
import os

from bno055 import BNO055
import sdcard

#PID制御関連の定数
max_u = 3
max_vol = 7
I_cyc = 0.000169625  #円盤の慣性モーメント[]
I_pay = 0.023476105  #ペーロードの慣性モーメント[]
T = 0.005   #PID制御の周期[s]
KP = 0.8
KD = 0
KI = 0
ie = 0
e_pre = 0

pre_duty = 0
now_data = 0

fre = 500000


IN1 = PWM(Pin(15))
IN2 = PWM(Pin(14))
CW = Pin(22, Pin.OUT)
CCW = Pin(21, Pin.OUT)

IN1.freq(fre)
IN2.freq(fre)

try:
    i2c = I2C(1, scl=Pin(27), sda=Pin(26))
    sensor = BNO055(i2c)
except:
    pass


cs = Pin(17, Pin.OUT)    #SDカード
spi = SPI(0, baudrate=32000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))


# Timerオブジェクト
record_timer = Timer()
init_sd_time = ticks_ms()
init_mission_time = ticks_ms()

#SDカード関連
sd = sdcard.SDCard(spi, cs)
os.mount(sd, '/sd')
file_index = 1
file_name = '/sd/PQ_SpaceBalloon_AC'+str(file_index)+'.txt'
while True:
    try:
        file = open(file_name, 'r')
    except OSError: # 新しい番号であればエラーに拾われる
        file = open(file_name, 'w')
        init_sd_time = ticks_ms()
        break
    if file:    # 同じ番号が存在する場合引っかかる
        file.close()    # 一旦古いファイルなので閉じる
        file_index += 1
        file_name = '/sd/PQ_SpaceBalloon_AC'+str(file_index)+'.txt'

file.write("mission_time,gyro_z,ration\r\n")


def record(t):
    global file, init_sd_time, now_data, rat
    try:
        mission_time = (ticks_ms() - init_mission_time)/1000
        print(rat)
        file.write("%f,%f,%f\r\n"%(mission_time, now_data, rat))
        if (ticks_ms() - init_sd_time > 10000):    # 10秒ごとにclose()して保存する
            file.close()
            file = open(file_name, "a")
            init_sd_time = ticks_ms()
           
    except:
        print('record error')
        pass
    
    
record_timer.init(period=1000, callback=record)


def rotation(duty):
    global pre_duty
    per = abs(duty)
    
    if pre_duty * duty <= 0:
        IN1.duty_u16(65535)
        IN2.duty_u16(65535)
        sleep(2)
    if duty >= 0:
        IN1.duty_u16(int(per*65535))
        IN2.duty_u16(0)
        CW.value(1)
        CCW.value(0)
    if duty < 0:
        IN1.duty_u16(0)
        IN2.duty_u16(int(per*65535))
        CW.value(0)
        CCW.value(1)
        
    pre_duty = duty

'''
電圧と回転数の関係．
今回は線形と仮定して，0Vで0[rad/s]，12Vで7410*2π/60[rad/s]とした．
出力は11.1Vに対する比．
範囲は+-1.

duty比が小さすぎるとモーターが動かない．．．
なので電圧の比の基準を11.1ｖから7.0ｖに落とすことで相対的に比率が高くなるように改変した．
その関係でu_maxの値も500から450に変更した．（u_maxの値でもratioが1を超えないように設定）
'''
def convert(ang):
    vol = 12/247/math.pi*ang
    ratio = vol/max_vol
    return ratio


def main():
    global e_pre, ie, now_data, rat
    while True:
        try:
            #print('start')
            gyr = sensor.gyro()
            #print('finish')
        except:
            print("miss!")
            
        sleep(T)
        now_data = gyr[2] * math.pi / 180  #センサーの値を[rad/s]に変換
        

        e = 0 - now_data
        de = (e - e_pre)/T
        ie = ie + (e + e_pre)*T/2
        u = KP*e + KI*ie + KD*de
        
        if u > max_u:
            u = max_u
        if u < -(max_u):
            u = -(max_u)

        u_cyc = I_pay * u / I_cyc
        rat = convert(u_cyc)
        rotation(rat)

        e_pre = e

if __name__=='__main__':
    main()
