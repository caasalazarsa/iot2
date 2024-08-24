import machine
import time
import _thread 

# inicializo el pin 2 como salida
led=machine.Pin(2,machine.Pin.OUT)

pin14 = machine.Pin(14)        #Se inicializa el PIN34
adc14 = machine.ADC(pin14)

def func():
    
    while True:
        print("hola mundo")
        time.sleep(1)
        

_thread.start_new_thread(func,())


def func2 ():
    
    global adc14
    
    while True:
        value=adc14.read_u16()*3.3/65535
        value=round(value,2)
        print(value)
        time.sleep(.5)
    
_thread.start_new_thread(func2,())

while True:
    led.value(1)
    time.sleep(.5)
    led.value(0)
    time.sleep(.5)