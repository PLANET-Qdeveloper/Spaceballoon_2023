'''
2024年3月，モンゴルでの放球
'''

from machine import Pin, I2C, PWM, SPI, Timer
from utime import sleep, ticks_ms
import math
import os

from bno055 import BNO055
import sdcard

#PID制御関連の定数
max_vol = 11.5
I_cyc = 0.000169625  #円盤の慣性モーメント[]
I_pay = 0.037861692  #ペーロードの慣性モーメント[]
T = 0.005   #PID制御の周期[s]
fre_PID = 1 / T  #PID制御の周期[Hz]
KP = 2.3
KD = 1.7
KI = 0.3
ie = 0
u_pre = 0
e_pre = 0
ome_pre = 0
rat = 0

pre_duty = 0
now_data = 0

#PWM制御関連の定数
fre_PWM = 30000  #[Hz]

#ADC関連の定数
fre_ADC = 60 #[Hz]
count_max = fre_PID // fre_ADC
count = 0
Vref = 1.25

#モータドライバのピンアサイン．1のみがONの時を正転とする．
IN1_UP = PWM(Pin(9))
IN2_UP = PWM(Pin(8))
IN1_LO = PWM(Pin(11))
IN2_LO = PWM(Pin(10))

FLIGHT_PIN = 3
flight_pin = Pin(FLIGHT_PIN, Pin.IN)

phase = 0

IN1_UP.freq(fre_PWM)
IN2_UP.freq(fre_PWM)
IN1_LO.freq(fre_PWM)
IN2_LO.freq(fre_PWM)

try:
    i2c = I2C(0, freq=50000, scl=Pin(1), sda=Pin(0))
    sensor = BNO055(i2c)
except:
    pass


cs_0 = Pin(17, Pin.OUT)    #SDカード
spi_0 = SPI(0, baudrate=32000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

cs_1 = Pin(13, Pin.OUT)    #ADC
spi_1 = SPI(1, baudrate=1000000, sck=Pin(14), mosi=Pin(15), miso=Pin(12))
cs_1.value(1)

# Timerオブジェクト
record_timer = Timer()
init_sd_time = ticks_ms()
init_mission_time = ticks_ms()

#SDカード関連
sd = sdcard.SDCard(spi_0, cs_0)
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

file.write("mission_time,gyro_z,voltage,ration\r\n")


def record(t):
    global file, init_sd_time, now_data, rat, max_vol
    mission_time = (ticks_ms() - init_mission_time)/1000
    print(rat)
    file.write("%f,%f,%f,%f\r\n"%(mission_time, now_data, max_vol, rat))
    if (ticks_ms() - init_sd_time > 10000):    # 10秒ごとにclose()して保存する
        file.close()
        file = open(file_name, "a")
        init_sd_time = ticks_ms()
           
record_timer.init(period=1000, callback=record)


def rotation(duty):
    global pre_duty
    per = duty + pre_duty
    per_abs = abs(per)
    #print('per',per)
    
    
    if per < -2.0:
        IN1_LO.duty_u16(65535)
        IN1_UP.duty_u16(0)
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(65535)

    elif -2.0 <= per < -1.6:
        IN1_LO.duty_u16(65535)
        IN1_UP.duty_u16(0)
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(int((per_abs - 1.0) * 65535))
        
    elif -1.6 <= per < -1.2:
        IN1_LO.duty_u16(int((per_abs - 0.6) * 65535))
        IN1_UP.duty_u16(0)
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(int(0.6 * 65535))

    elif -1.2 <= per < -1.0:
        IN1_LO.duty_u16(int(0.6 * 65535))
        IN1_UP.duty_u16(0)
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(int(0.6 * 65535))

    elif -1.0 <= per < -0.6:
        IN1_LO.duty_u16(int(per_abs * 65535))
        IN1_UP.duty_u16(65535)
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(65535)

    elif -0.6 <= per < -0.4:
        IN1_LO.duty_u16(int(0.6 * 65535))
        IN1_UP.duty_u16(65535)
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(65535)

    elif -0.4 <= per < 0.0:
        IN1_LO.duty_u16(int((per_abs + 0.6) * 65535))
        IN1_UP.duty_u16(int(0.6 * 65535))
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(0)

    elif 0.0 <= per < 0.4:
        IN1_LO.duty_u16(int(0.6 * 65535))
        IN1_UP.duty_u16(int((per_abs + 0.6) * 65535))
        IN2_LO.duty_u16(0)
        IN2_UP.duty_u16(0)

    elif 0.4 <= per < 0.6:
        IN1_LO.duty_u16(65535)
        IN1_UP.duty_u16(int(0.6 * 65535))
        IN2_LO.duty_u16(65535)
        IN2_UP.duty_u16(0)

    elif 0.6 <= per < 1.0:
        IN1_LO.duty_u16(65535)
        IN1_UP.duty_u16(int(per_abs * 65535))
        IN2_LO.duty_u16(65535)
        IN2_UP.duty_u16(0)

    elif 1.0 <= per < 1.2:
        IN1_LO.duty_u16(0)
        IN1_UP.duty_u16(int(0.6 * 65535))
        IN2_LO.duty_u16(int(0.6 * 65535))
        IN2_UP.duty_u16(0)

    elif 1.2 <= per < 1.6:
        IN1_LO.duty_u16(0)
        IN1_UP.duty_u16(int((per_abs - 0.6) * 65535))
        IN2_LO.duty_u16(int(0.6 * 65535))
        IN2_UP.duty_u16(0)

    elif 1.6 <= per < 2.0:
        IN1_LO.duty_u16(0)
        IN1_UP.duty_u16(65535)
        IN2_LO.duty_u16(int((per_abs - 1.0) * 65535))
        IN2_UP.duty_u16(0)
        
    elif per <= 2.0:
        IN1_LO.duty_u16(0)
        IN1_UP.duty_u16(65535)
        IN2_LO.duty_u16(65535)
        IN2_UP.duty_u16(0)
        
    pre_duty = duty


'''
電圧と回転数の関係．
今回は線形と仮定して，0Vで0[rad/s]，12Vで7410*2π/60[rad/s]とした．
出力は現在の電圧に対する比．
範囲は+-2.
max関数とmin関数を組み合わせることで対処した．

ADCのサンプリングレートが60Hzであることから，数回に1回電圧を更新していくことした．
また今回は10KΩと1KΩで分圧した値を表示しているので，11倍している．

PWMでのモーター回転は60-100%しか無理そう...
'''

def convert(ang):
    global max_vol, count
    vol = 12/247/math.pi*ang

    count += 1
    if count > count_max:
        cs_1.value(0)
        data = spi_1.read(2)
        cs_1.value(1)

        data_ADC = (data[0] << 8) | data[1]
        max_vol = ((data_ADC - 32768) * Vref / 32768) * 11
        count = 0
    '''
    if max_vol < 10.5:
        raise ValueError("error!")
    '''

    ratio = vol / max_vol
    ratio = max(-2, min(2, ratio))  

    return ratio



def main():
    global ome, ome_pre, u, u_pre, e_pre, ie, now_data, rat, phase
    while True:
    
        if phase == 0:
            if flight_pin.value() == 1:
                phase = 1
            sleep(0.1)
            print('wait')
            
        if phase == 1:
            #print('start')
            gyr = sensor.gyro()
            #print('finish')

            now_data = gyr[2] * math.pi / 180  #センサーの値を[rad/s]に変換
            print(now_data)

            e = 0 - now_data
            de = (e - e_pre)/T
            ie = ie + (e + e_pre)*T/2
            u = KP*e + KI*ie + KD*de
            
            
            ome = -I_pay * (u - u_pre) / I_cyc + ome_pre
            rat = convert(ome)
            print('rat : ',rat)
            rotation(rat)

            ome_pre = ome
            u_pre = u
            e_pre = e
            sleep(T)
        '''
        except:
            print("miss!")
            #print(gyr)
        '''
        

if __name__=='__main__':
    main()


