import utime
from machine import I2C, Pin
from mpu9250 import MPU9250
from mpu6500 import MPU6500, SF_G, SF_DEG_S
from ak8963 import AK8963
from math import atan,atan2,cos,sin
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



Kalman1DOutput =[0,0]

sample_time=0.01

print_count=0

alpha=0
beta=0
gamma=0

Rot_imu=[]
Rot_earth=[]

acc_x_adj=0
acc_y_adj=0
acc_z_adj=0

offset_acc=[-0.075,-0.068,0.165]

offset_gyro=[0.011,-0.017]

def kalman_1d(KalmanState,KalmanUncertainty,KalmanInput,KalmanMeasurement,sampleTime):
    KalmanState=KalmanState+sampleTime*KalmanInput
    KalmanUncertainty=KalmanUncertainty+(sampleTime*4)**2
    KalmanGain=KalmanUncertainty*1/(1*KalmanUncertainty+3**2)
    KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState)
    KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty
    
    Kalman1DOutput[0]=KalmanState
    Kalman1DOutput[1]=KalmanUncertainty
    

while True:

    
    acc_x=sensor.acceleration[0]+offset_acc[0]
    acc_y=sensor.acceleration[1]+offset_acc[1]
    acc_z=sensor.acceleration[2]+offset_acc[2]
    
    rate_roll=sensor.gyro[0]+offset_gyro[0]
    rate_pitch=sensor.gyro[1]+offset_gyro[1]
    
    angle_roll=atan(acc_y/(acc_x**2+acc_z**2)**0.5)
    angle_pitch=-atan(acc_x/(acc_y**2+acc_z**2)**0.5)
    
    
    kalman_1d(kalman_angle_roll,kalman_uncertainty_angle_roll,rate_roll,angle_roll,sample_time)
    kalman_angle_roll=Kalman1DOutput[0]
    kalman_1d(kalman_angle_pitch,kalman_uncertainty_angle_pitch,rate_pitch,angle_pitch,sample_time)
    kalman_angle_pitch=Kalman1DOutput[0]
    
    
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
        #print(kalman_angle_roll,kalman_angle_pitch,angle_yaw)
        print(acc_x,acc_y,acc_z)
        #print(rate_roll,rate_pitch)
        #print(Rot_imu)
        print(G_imu)
        print(acc_x_adj,acc_y_adj,acc_z_adj)
        print(Acc_earth)
        print("")
        #print(Rot_earth)
    else:
        print_count+=1
        
    
    
    

    utime.sleep_ms(int(sample_time*1000))