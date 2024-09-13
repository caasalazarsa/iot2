from machine import Pin, UART,I2C
from math import radians, cos, sin, asin, sqrt,atan,atan2
from utime import sleep,sleep_ms
from mpu9250 import MPU9250
from mpu6500 import MPU6500, SF_G, SF_DEG_S
from ak8963 import AK8963
import mtx


i2c = I2C(1,scl=Pin(19), sda=Pin(18))
dummy = MPU9250(i2c) # this opens the bybass to access to the AK8963

ak8963 = AK8963(
    i2c,
    offset=(-9.829102, 62.57461, -62.91797),
    scale=(0.9520254, 1.007409, 1.044973)
)
mpu6500 = MPU6500(i2c, accel_sf=SF_G)

sensor = MPU9250(i2c, ak8963=ak8963,mpu6500=mpu6500)

print("MPU9250 id: " + hex(sensor.whoami))


rate_roll=0; rate_pitch=0; rate_yaw=0
rate_calibration_roll=0; rate_calibrationpitch=0; rate_calibration_yaw=0
rate_calibration_number=0
acc_x=0; acc_y=0; acc_z=0
angle_roll=0; angle_pitch=0
kalman_angle_roll=0; kalman_uncertainty_angle_roll=2**2
kalman_angle_pitch=0; kalman_uncertainty_angle_pitch=2**2

kalman_pos_x=0; kalman_uncertainty_pos_x=2**2
kalman_pos_y=0; kalman_uncertainty_pos_y=2**2

Kalman1DOutput =[0,0]
KalmanPosOutput =[0,0,0]

sample_time=0.02
process_time=0.01

print_count=0

alpha=0
beta=0
gamma=0

Rot_imu=[[1,0,0],[0,1,0],[0,0,1]]
Rot_earth=[[1,0,0],[0,1,0],[0,0,1]]
G_imu=[0,0,0]
Pos_imu=[0,0,0]

Acc_earth=[0,0,0]

acc_x_adj=0
acc_y_adj=0
acc_z_adj=0

offset_acc=[-0.075,-0.068,0.165]

offset_gyro=[0.011,-0.017]


bufferE=[]
bufferN=[]


pX=0
pY=0
pZ=0

pX_=0
pY_=0
pZ_=0



rapi=0
curso=0

for i in range(10):
    bufferE.append(0)
    bufferN.append(0)


def kalman_1d(KalmanState,KalmanUncertainty,KalmanInput,KalmanMeasurement,sampleTime,stdev1,stdev2):
    KalmanState=KalmanState+sampleTime*KalmanInput
    KalmanUncertainty=KalmanUncertainty+(sampleTime*stdev1)**2
    KalmanGain=KalmanUncertainty*1/(1*KalmanUncertainty+stdev2**2)
    KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState)
    KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty
    
    Kalman1DOutput[0]=KalmanState
    Kalman1DOutput[1]=KalmanUncertainty
    
    
    
    
    
def kalman_1d_pos(KalmanState,KalmanUncertainty,KalmanInput1,KalmanInput2,KalmanMeasurement,KalmanMeasurement1,sampleTime,stdev1,stdev2,update):
    
    KalmanInput1=KalmanInput1+KalmanInput2*sampleTime
    KalmanState=KalmanState+sampleTime*KalmanInput1
    KalmanUncertainty=KalmanUncertainty+(sampleTime*stdev1)**2
    if(update):
        KalmanGain=KalmanUncertainty*1/(1*KalmanUncertainty+stdev2**2)
        KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState)
        KalmanInput1=KalmanInput1+KalmanGain*(KalmanMeasurement1-KalmanInput1)
        KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty
    
    KalmanPosOutput[0]=KalmanState
    KalmanPosOutput[1]=KalmanInput1
    KalmanPosOutput[2]=KalmanUncertainty
    

uart = UART(0, 9600,timeout=200, rx=17, tx=16)

uart2 = UART(1, 9600,timeout=200, rx=9, tx=8)
lectura=""

lecturalist=[]

E=0
N=0
U=0

Eavg=0
Navg=0

Vx=0
Vy=0

VxG=0
VyG=0

Unc1=0
Unc2=0




uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x00\x00\xFA\x0F')) #GGA
lectura=uart.readline()
sleep_ms(100)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x01\x00\xFB\x11')) #GLL
lectura=uart.readline()
sleep_ms(100)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x02\x00\xFC\x13')) #GSA
lectura=uart.readline()
sleep_ms(100)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x03\x00\xFD\x15')) #GSV
lectura=uart.readline()
sleep_ms(100)
#uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x04\x00\xFE\x17')) #RMC
#lectura=uart.readline()
#sleep_ms(100)
uart.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x05\x00\xFF\x19')) #VTG
lectura=uart.readline()
sleep_ms(100)
uart.write(bytearray(b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A')) #samplerate 5hz
lectura=uart.readline()
sleep_ms(100)


uart2.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x00\x00\xFA\x0F')) #GGA
lectura=uart2.readline()
sleep_ms(100)
uart2.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x01\x00\xFB\x11')) #GLL
lectura=uart2.readline()
sleep_ms(100)
uart2.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x02\x00\xFC\x13')) #GSA
lectura=uart2.readline()
sleep_ms(100)
uart2.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x03\x00\xFD\x15')) #GSV
lectura=uart2.readline()
sleep_ms(100)
#uart2.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x04\x00\xFE\x17')) #RMC
#lectura=uart2.readline()
#sleep_ms(100)
uart2.write(bytearray(b'\xB5\x62\x06\x01\x03\x00\xF0\x05\x00\xFF\x19')) #VTG
lectura=uart2.readline()
sleep_ms(100)
uart2.write(bytearray(b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A')) #samplerate 5hz
lectura=uart2.readline()
sleep_ms(100)


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

for i in range(300):
    
    counter+=1
    
        
    if counter>=10:
        
        counter=0
        print("dropeando muestras")
            
    
    sleep(1)
    
    

pcounter=0
    

while True:

    
    if pcounter>=10:
        pcounter=0
        try:
            lecturalist=lectura.split(",")
            if lecturalist[0]=="$GPRMC":
                latitud=float(lecturalist[3][:2])+(float(lecturalist[3][2:])/60.0)
                longitud=-1.0*(float(lecturalist[5][:3])+(float(lecturalist[5][3:])/60.0))

                
                
                
                #altura=float(lecturalist[9])
                #print("latitud: "+str(latitud)+lecturalist[4])
                #print("Longitud: "+str(longitud)+lecturalist[6])
                #print("coordenadas: "+ str(latitud)+str(longitud))
                if flag==True:
                    latI=latitud
                    lonI=longitud
                    print("Coordenadas iniciales definidas...")
                    flag=False
                    gX,gY,gZ=sph2geocentric(latI,lonI,0)
                    pX_=gX;pY_=gY
                    
                
                rapi=float(lecturalist[7])*0.514
                curso=lecturalist[8]
                #print(rapi)
                
                
                pX,pY,pZ=sph2geocentric(latitud,longitud,0)
                
                dirx=pX-pX_
                diry=pY-pY_
                
                angle=atan2(dirx,diry)
                
                #print(angle)
                
                VxG=rapi*cos(angle)
                VyG=rapi*sin(angle)
                
            
                E,N,U=relativepos(gX,gY,gZ,pX,pY,pZ,latI,lonI)
                
                bufferE.pop(0)
                bufferN.pop(0)
             
                
                bufferE.append(E)
                bufferN.append(N)
            
            
                pX_=pX;pY_=pY
                
                Navg=sum(bufferN)/len(bufferN)
                Eavg=sum(bufferE)/len(bufferE)
                
                #9.80665*0.05355371135598354  ; 9.80665*.033436506994600976
                
                kalman_1d_pos(kalman_pos_y,kalman_uncertainty_pos_y,Vy,Acc_earth[1]*-9.80665,Navg,VyG,sample_time,9.80665*0.05355371135598354,.3,True)
                kalman_pos_y=KalmanPosOutput[0]
                Vy=KalmanPosOutput[1]
                kalman_uncertainty_pos_y=KalmanPosOutput[2]
                kalman_1d_pos(kalman_pos_x,kalman_uncertainty_pos_x,Vx,Acc_earth[0]*-9.80665,Eavg,VxG,sample_time,9.80665*.033436506994600976,.3,True)
                kalman_pos_x=KalmanPosOutput[0]
                Vx=KalmanPosOutput[1]
                kalman_uncertainty_pos_x=KalmanPosOutput[2]
            
            
        
        
        
        except Exception as e:
            print(e)
        
    else:
        pcounter+=1
        pass
             
        kalman_1d_pos(kalman_pos_y,kalman_uncertainty_pos_y,Vy,Acc_earth[1]*-9.80665,kalman_pos_y,Vy,sample_time,9.80665*0.05355371135598354,.3,False)
        kalman_pos_y=KalmanPosOutput[0]
        Vy=KalmanPosOutput[1]
        kalman_uncertainty_pos_x=KalmanPosOutput[2]
        kalman_1d_pos(kalman_pos_x,kalman_uncertainty_pos_x,Vx,Acc_earth[0]*-9.80665,kalman_pos_x,Vx,sample_time,9.80665*.033436506994600976,.3,False)
        kalman_pos_x=KalmanPosOutput[0]
        Vx=KalmanPosOutput[1]
        kalman_uncertainty_pos_y=KalmanPosOutput[2]
        
    
    acc_x=sensor.acceleration[0]+offset_acc[0]
    acc_y=sensor.acceleration[1]+offset_acc[1]
    acc_z=sensor.acceleration[2]+offset_acc[2]
    
    rate_roll=sensor.gyro[0]+offset_gyro[0]
    rate_pitch=sensor.gyro[1]+offset_gyro[1]
    
    angle_roll=atan(acc_y/(acc_x**2+acc_z**2)**0.5)
    angle_pitch=-atan(acc_x/(acc_y**2+acc_z**2)**0.5)
    
    
    kalman_1d(kalman_angle_roll,kalman_uncertainty_angle_roll,rate_roll,angle_roll,sample_time,3,4)
    kalman_angle_roll=Kalman1DOutput[0]
    kalman_uncertainty_angle_roll=Kalman1DOutput[1]
    kalman_1d(kalman_angle_pitch,kalman_uncertainty_angle_pitch,rate_pitch,angle_pitch,sample_time,3,4)
    kalman_angle_pitch=Kalman1DOutput[0]
    kalman_uncertainty_angle_pitch=Kalman1DOutput[1]
    
    
    Yh = (sensor.magnetic[1] * cos(kalman_angle_roll)) - (sensor.magnetic[2] * sin(kalman_angle_roll));
    Xh = (sensor.magnetic[0] * cos(kalman_angle_pitch))+(sensor.magnetic[1] * sin(kalman_angle_roll)*sin(kalman_angle_pitch)) + (sensor.magnetic[2] * cos(kalman_angle_roll) * sin(kalman_angle_pitch));
    
    angle_yaw =  atan2(Yh, Xh);
    
    alpha=kalman_angle_roll
    beta=kalman_angle_pitch
    gamma=angle_yaw

    Rot_imu=[[cos(beta)*cos(gamma),cos(gamma)*sin(alpha)*sin(beta)-cos(alpha)*sin(gamma),cos(alpha)*cos(gamma)*sin(beta)+sin(alpha)*sin(gamma)],[cos(beta)*sin(gamma),cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma),-cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)],[-sin(beta),cos(beta)*sin(alpha),cos(alpha)*cos(beta)]]
    
    Rot_earth = [[Rot_imu[j][i] for j in range(len(Rot_imu))] for i in range(len(Rot_imu[0]))]
    
    G_imu=mtx.mul(Rot_earth,[0,0,-1])
    
    acc_x_adj=acc_x+G_imu[0]
    acc_y_adj=acc_y+G_imu[1]
    acc_z_adj=acc_z+G_imu[2]
    
    Acc_earth=mtx.mul(Rot_imu,[acc_x_adj,acc_y_adj,acc_z_adj])
    
    
    

    
    
    
    
    
    if print_count>=50:
        print_count=0
        print(angle_yaw)
        print(acc_x,acc_y,acc_z)
        #print(rate_roll,rate_pitch)
        #print(Rot_imu)
        print(G_imu)
        print(acc_x_adj,acc_y_adj,acc_z_adj)
        print(Acc_earth)
        print(E,N)
        print(kalman_pos_x,kalman_pos_y)
        print(Vx,Vy)
        print("")
        #print(Rot_earth)
    else:
        print_count+=1
        
    
    
    

    sleep_ms(int((sample_time-process_time)*1000))