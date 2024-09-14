import network
import urequests
import ujson
import machine
import time


ssid="Carlitosh"
password="1223456rt"


# serverUrl = "https://piaback-816ac3976233.herokuapp.com/sensorData";


conversionFactor = 0.2
valorSensor= 0
ppm = 0


pin33 = machine.Pin(33)        #Se inicializa el PIN34
adc33 = machine.ADC(pin33)

led_verde=machine.Pin(25,machine.Pin.OUT)
led_amarillo=machine.Pin(26,machine.Pin.OUT)
led_rojo=machine.Pin(27,machine.Pin.OUT)


station = network.WLAN(network.STA_IF)

station.active(True)
station.connect(ssid, password)


while station.isconnected() == False:
    ticks2=time.ticks_ms()
    if time.ticks_diff(ticks1, ticks2) % 10000 ==0:
        print("conectando")
    pass

print('Connection successful')
print(station.ifconfig())


request_url="http://ae2e-186-31-211-115.ngrok-free.app"


res = urequests.request("GET",request_url+"/cursos").text

print(res)

post_data = ujson.dumps({ 'codigo': '123456', 'nombre': 'hola','creditos':4})

res = urequests.post(request_url+"/cursos", headers = {'content-type': 'application/json'}, data = post_data).json()

print(res)


request_url="http://ae2e-186-31-211-115.ngrok-free.app"


res = urequests.request("GET",request_url+"/cursos").text


print(res)
print(res)

"""
while True:
    valorSensor=adc33.read_u16()*1.5*1023/65535
    
    ppm=valorSensor*conversionFactor
    
    
    print("Lectura de CO2: {} ppm	".format(ppm))
    
    if (ppm<400):
        led_verde.value(1)
        led_amarillo.value(0)
        led_rojo.value(0)
        
    elif (ppm >= 400 and ppm < 700):
        led_verde.value(0)
        led_amarillo.value(1)
        led_rojo.value(0)
        
    else:
        led_verde.value(0)
        led_amarillo.value(0)
        led_rojo.value(1)
        
    time.sleep(.2)
        
    
"""
