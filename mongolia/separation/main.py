from machine import Pin, PWM, ADC, SPI, Timer
from utime import sleep, ticks_ms
import os
import sdcard

# Timerオブジェクト
record_timer = Timer()
init_sd_time = ticks_ms()
init_mission_time = ticks_ms()

#phase
phase = 0 
'''
phase 0 :放球前
phase 1 :フライトピン抜けた
phase 2 :分離
'''

#LED
led = Pin(27,Pin.OUT)
#フライトピン
flight_pin = Pin(21,Pin.IN)

#気圧センサ/ADC pin
Pin(26,Pin.IN)
a = ADC(0) #GPIO26に対応するADCは0番
coeff = 3.2/4095 #0~3.3V入力, 12ビット分解能
#気圧センサ関連
Prange = 103421 #Pa (=15psi)
Voff = 0.20
designated_press = 1000000 #Pa,大体高度10km
press = 101300

def take_press():
    global press

    Vout = a.read_u16()*coeff
    #print('a.read_u16() : ',a.read_u16())
    #print('Vout : ',Vout) 
    #print('Vout-Voff : ',Vout-Voff) 
    press = Prange * (Vout-Voff)/2.7


#SDカード pin
cs = Pin(17, Pin.OUT)   
spi = SPI(0, baudrate=32000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

#SDカード関連
sd = sdcard.SDCard(spi, cs)
os.mount(sd, '/sd')
file_index = 1
file_name = '/sd/PQ_SpaceBalloon_sep'+str(file_index)+'.txt'
#新ファイル作る
while True:
    try:
        file = open(file_name, 'r')
    except OSError: # 新しい番号であればエラーに拾われる
        file = open(file_name, 'w')
        init_sd_time = ticks_ms() #ファイルを作った時刻ってこと？
        break
    if file:    # 同じ番号が存在する場合引っかかる
        file.close()    # 一旦古いファイルなので閉じる
        file_index += 1
        file_name = '/sd/PQ_SpaceBalloon_sep'+str(file_index)+'.txt'
#１行目に取得データ項目名記入
file.write("mission_time,phase,atm_pressure\r\n")


def record(t):
    global file, init_sd_time, press
    try:
        mission_time = (ticks_ms() - init_mission_time)/1000
        take_press()
        print(press)
        file.write("%f,%f,%f\r\n"%(mission_time, phase, press))
        if (ticks_ms() - init_sd_time > 10000):    # 10秒ごとにclose()して保存する
            file.close()
            file = open(file_name, "a")
            init_sd_time = ticks_ms()
           
    except:
        print('record error')
        pass
    
    
record_timer.init(period=2000, callback=record)

#サーボモータ関連
freq = 50 #20ms = 50Hz
duty_offset = 0.5 / 20 #制御パルスが0.5~2.4msのため、オフセットを設定 
duty_coef = (2.4 - 0.5) / 20 / 180 #角度をデューティ比に変換する係数
servo = PWM(Pin(0))
servo.freq(freq)

initial_angle = 90
final_angle = -90
separation_timing = 3900



#指定した角度にサーボモータを回転させる関数
def set_angle(angle):
    global duty_offset, duty_coef
    degree = angle + 90
    duty = int((duty_offset + duty_coef * degree) * 65535)
    servo.duty_u16(duty)
    sleep(1)

    servo.deinit() #PWM出力を無効化…しない方がいいよね？


#メインプログラム
def main():
    global phase, press
    
    set_angle(initial_angle)
    sleep(1)


    while True:
        print(phase)
        if phase == 0:
            led.value(1)
            sleep(0.1)

            if flight_pin.value() == 1:
                initial_mission_time = ticks_ms()
                print('go')
                phase = 1
                

        if phase == 1:
            mission_time = (ticks_ms() - initial_mission_time) /1000
            led.value(1)
            sleep(1)
            led.value(0)
            sleep(1)
            
            if mission_time > separation_timing and press < designated_press:
                phase = 2
                

                '''
                SDにphase書ける？
                気圧のこと別フェーズにする必要はないよね．
                センサの誤作動時用にtry-exceptやれるんかな
                '''
        
        if phase == 2:
            
            set_angle(final_angle)
            print("motor is rotated!")
            sleep(3)
            servo.deinit()
            break
    
    while True:
        led.value(1)
        sleep(0.5)
        led.value(0)
        sleep(0.5)
        led.value(1)
        sleep(2)
        led.value(0)
        sleep(0.5)




if __name__=='__main__':
    main()