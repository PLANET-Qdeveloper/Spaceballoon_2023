#ライブラリのインポート
from machine import Pin, UART, I2C, SPI, Timer
import time, os
from utime import ticks_ms
import sdcard
import sys
from micropyGPS import MicropyGPS

#自作ライブラリのインポート
from bme280 import BME280
from PQ_LPS22HB import LPS22HB
from PQ_RM92A import RM92A

#i2c関係の設定
i2c_0 = I2C(0, scl=Pin(21), sda=Pin(20),freq=400000)
i2c_1 = I2C(1, scl=Pin(27), sda=Pin(26),freq=400000)

#i2cを用いたセンサのインスタンスの生成
lps = LPS22HB(i2c_0)
bme_0 = BME280(i2c=i2c_0, address = 0x76)
bme_1 = BME280(i2c=i2c_1, address = 0x76)

#SPI関係の設定
cs = Pin(17, Pin.OUT)
spi = SPI(0, baudrate=32000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

#UARTの設定
rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
gps_uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))

rm = RM92A(rm_uart)
gps = MicropyGPS()

# ピンアサインの設定
LED_PIN = 14  # LEDピンの番号

#変数の設定
mission_time = 0
temp = []
pres = []
PRESS = 0
TEMP = 0
lat = 0
lon = 0

flight_time = 0


#タイマーの設定(周期処理)
record_timer = Timer()
downlink_timer = Timer()
read_timer = Timer()

init_flight_time = ticks_ms()

# LEDピンの初期化
led1 = Pin(LED_PIN, Pin.OUT)
led1.value(0)

#SD用ファイル生成
sd = sdcard.SDCard(spi, cs)
os.mount(sd, '/sd')
file_index = 1
file_name = '/sd/PQ_BALLOON_MONGOLIA'+str(file_index)+'.csv'
init_sd_time = 0
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
        file_name = '/sd/PQ_BALLOON_MONGOLIA'+str(file_index)+'.csv'

file.write("TIME,INNER_PRESSURE,OUTER_PRESSURE,INNER_TEMPERATURE,OUTERTEMPERATURE,LATITUDE,LONGITUDE,ALTITUDE\r\n")

#READ関数．センサの値を取得
def read(t):
    global PRESS_IN, TEMP_IN, PRESS_OUT, TEMP_OUT, lat, lon, alt    
    try:
        t1,p1,h1 = bme_0.read_compensated_data()
    except OSError: 
        t1 = p1 = h1 = 0
    p1 = p1 // 256
    pi1 = p1 / 100
    ti1 = t1 / 100
    
    pres.append(pi1)
    temp.append(ti1)
    
    try:
        press = lps.read_pressure()
        temperature = lps.read_temperature()
    except OSError: 
        press = temperature = 0
    
    pres.append(press)
    temp.append(temperature)
    
    pres.sort()
    temp.sort()

    PRESS_IN = pres[-1]
    TEMP_IN = temp[-1]

    pres.clear()
    temp.clear()


    try:
        t1_out,p1_out,h1_out = bme_1.read_compensated_data()
    except OSError: 
        t1_out = p1_out = h1_out = 0
    pi1_out = p1_out // 256
    PRESS_OUT = pi1_out / 100
    TEMP_OUT = t1_out / 100

    
    try:
        len = gps_uart.any()
        if len > 0:
            b = gps_uart.read(len)
            for x in b:
                if 10 <= x <=126:
                    status = gps.update(chr(x))
                    if status:
                        lat = gps.latitude[0] + gps.latitude[1]/60
                        lon = gps.longitude[0] + gps.longitude[1]/60
                        alt = gps.altitude
    except OSError:
        lat = 0
        lon = 0
        alt = 0
read_timer.init(period=100, callback=read)


#record関数．取得値の保存
def record(t):
    global PRESS_IN, TEMP_IN, PRESS_OUT, TEMP_OUT, lat, lon, alt, flight_time, init_sd_time, file
    try:
        flight_time = (ticks_ms() - init_flight_time)/1000
        file.write("%d,%f,%f,%f,%f,%f,%f,%f\r\n"%(flight_time, PRESS_IN, PRESS_OUT, TEMP_IN, TEMP_OUT, lat, lon, alt))
        if (ticks_ms() - init_sd_time > 10000):    # 10秒ごとにclose()して保存する
            file.close()
            file = open(file_name, "a")
            init_sd_time = ticks_ms()
    except:
        pass
record_timer.init(period=1000, callback=record)


#ダウンリンク関数
def downlink(t):
    global PRESS_IN, TEMP_IN, PRESS_OUT, TEMP_OUT, lat, lon, alt, flight_time

    PRESS1 = PRESS_IN * 10
    TEMP1 = TEMP_IN * 100
    PRESS2 = PRESS_OUT * 10
    TEMP2 = TEMP_OUT * 100
    LAT = lat
    LON = lon
    ALT = alt
    TIME = int(flight_time)
    
    press_in1 = int(PRESS1 // 256)
    press_in2 = int(PRESS1 % 256)
    
    temp_in1 = int(TEMP1 // 256)
    temp_in2 = int(TEMP1 % 256)

    press_out1 = int(PRESS2 // 256)
    press_out2 = int(PRESS2 % 256)
    
    temp_out1 = int(TEMP2 // 256)
    temp_out2 = int(TEMP2 % 256)
    
    lat1 = int(LAT)
    lat2 = int((LAT - lat1) * 100)
    lat3 = int(((LAT - lat1) * 100 - lat2) * 100)
    
    lon1 = int(LON)
    lon2 = int((LON - lon1) * 100)
    lon3 = int(((LON - lon1) * 100 - lon2) * 100)
    
    alt1 = int(ALT // 256)
    alt2 = int(ALT % 256)
    
    time1 = int(TIME // 256)
    time2 = int(TIME % 256)
    
    
    send_data = bytearray(26)
    send_data[0] = 0x24
    send_data[1] = press_in1
    send_data[2] = press_in2
    send_data[3] = temp_in1
    send_data[4] = temp_in2
    send_data[5] = press_out1
    send_data[6] = press_out2
    send_data[7] = temp_out1
    send_data[8] = temp_out2
    send_data[9] = lat1
    send_data[10] = lat2
    send_data[11] = lat3
    send_data[12] = lon1
    send_data[13] = lon2
    send_data[14] = lon3
    send_data[15] = alt1
    send_data[16] = alt2
    send_data[17] = time1
    send_data[18] = time2
    
    rm.send(0xFFFF, send_data)
    print('sending......')
downlink_timer.init(period=2000, callback=downlink)

# メイン関数
def main():
    while True:
        time.sleep(1)
        led1.value(1)
        time.sleep(1)
        led1.value(0)


if __name__ == '__main__':
    main()

