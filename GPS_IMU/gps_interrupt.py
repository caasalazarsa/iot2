from machine import Pin, UART
from math import radians, cos, sin, asin, sqrt
from utime import sleep,sleep_ms

uart = UART(0, 9600,timeout=200, rx=17, tx=16)


lectura=""

lecturalist=[]

E=0
N=0
U=0




uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x01\x00\xFB\x11')) #GLL
lectura=uart.readline()
sleep_ms(200)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x02\x00\xFC\x13')) #GSA
lectura=uart.readline()
sleep_ms(200)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x03\x00\xFD\x15')) #GSV
lectura=uart.readline()
sleep_ms(200)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x04\x00\xFE\x17')) #RMC
lectura=uart.readline()
sleep_ms(200)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x05\x00\xFF\x19')) #VTG
lectura=uart.readline()
sleep_ms(200)
uart.write(bytearray(b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A')) #samplerate 5hz
lectura=uart.readline()
sleep_ms(200)

def IRQ(pin):
    global lectura
    try:
        if uart.any()>10:
            lectura=uart.readline().decode()
            #print("muestra:"+lectura)
        
    except:
        print("hola")
        pass
        
Pin(17).irq(IRQ, trigger=Pin.IRQ_FALLING)

#uart.irq(UART.RX_ANY, priority=1, handler=IRQ, wake=machine.IDLE)

flag=True
def haversine(lon1, lat1, lon2, lat2):
    #convert degrees to radians
    lon1 = radians(lon1)
    lat1 = radians(lat1)
    lon2 = radians(lon2)
    lat2 = radians(lat2)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    distance = 2 * asin(sqrt(a)) * 6371
    return distance


def sph2geocentric(lat,lon,h):
    
    
    lon = radians(lon)
    lat = radians(lat)

    
    v=6371*1000
    
    x=(v+h)*cos(lat)*cos(lon)
    y=(v+h)*cos(lat)*sin(lon)
    z=(v+h)*sin(lat)
    
    return x,y,z
    
def relativepos(gX,gY,gZ,pX,pY,pZ,glat,glon):
    
    glon = radians(glon)
    glat = radians(glat)

    E = (-1)*(pX - gX)*(sin (glon)) + (pY - gY)*(cos(glon))
    N = (-1)*(pX - gX)*(sin(glat) * cos(glon)) - (pY - gY)*(sin(glat) * sin(glon)) + (pZ - gZ)*(cos(glat))
    U = (pX - gX)*(cos(glat) * cos(glon)) + (pY - gY)*(cos(glat) * sin(glon)) + (pZ - gZ)*(sin(glat))
    
    return E,N,U
    
counter=0

for i in range(120):
    
    counter+=1
    
        
    if counter>=10:
        
        counter=0
        print("dropeando muestras")
            
    
    sleep(1)
    
    

pcounter=0

while True:
    try:
        lecturalist=lectura.split(",")
        if lecturalist[0]=="$GPGGA":
            latitud=float(lecturalist[2][:2])+(float(lecturalist[2][2:])/60.0)
            longitud=-1.0*float(lecturalist[4][:3])+(float(lecturalist[4][3:])/60.0)
            altura=float(lecturalist[9])
            #print("latitud: "+str(latitud)+lectura[3])
            #print("Longitud: "+str(longitud)+lectura[5])
            #print("coordenadas: "+ str(latitud)+str(longitud))
            if flag==True:
                latI=latitud
                lonI=longitud
                altI=altura
                print("Coordenadas iniciales definidas...")
                flag=False
                gX,gY,gZ=sph2geocentric(latI,lonI,0)
                
                
            pX,pY,pZ=sph2geocentric(latitud,longitud,0)
            
            E,N,U=relativepos(gX,gY,gZ,pX,pY,pZ,latI,lonI)
            
            
        sleep_ms(100)
        
        if pcounter>=10:
            pcounter=0
            print(E,N,U)
        
        pcounter+=1
        
    except:
        pass
        