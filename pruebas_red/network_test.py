import network
import config
import time
import json

ssid=config.ssid
password=config.password

sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)

try:
    sta_if.connect(ssid,password)
    for i in range(10):
        if not sta_if.isconnected():
            time.sleep(1)
            print("conectando...")
    
    if sta_if.isconnected():
        print("conectado")
    else:
        print("no conectado")
    
except Exception as e:
    print(e)
    
finally:
    import urequests
    """
    rest=urequests.post("http://192.168.0.1/goform/goform_set_cmd_process",data="isTest=false&goformId=LOGIN&password=52Yf3555",headers={"Referer": "http://192.168.0.1/index.html"})
    print(rest.text)
    rest1=urequests.request("GET","http://192.168.0.1/goform/goform_set_cmd_process?isTest=false&cmd=sms_data_total",headers={"Referer": "http://192.168.0.1/index.html"})
    print(rest1.text)
    rest2=urequests.post("http://192.168.0.1/goform/goform_set_cmd_process",data="isTest=false&goformId=SEND_SMS&notCallback=true&Number={}&sms_time={}&MessageBody={}&ID=-1&encode_type=UNICODE".format(config.num,"28-08-24 14:00:10","hola"),headers={"Referer": "http://192.168.0.1/index.html"})
    print(rest2.text)
    time.sleep(5)
    """
    
    rest=urequests.request("GET","https://pokeapi.co/api/v2/pokemon/ditto")
    #print(rest.text)
    obj=json.loads(rest.text)
    print(obj['abilities'])
    print(obj['abilities'][0])
    
    pass
    

"""

Method: GET
curl -s --header "Referer: http://<modem_ip>/index.html" http://<modem_ip>//goform/goform_get_cmd_process\?isTest\=false\&cmd\=sms_data_total\&page=0\&data_per_page\=500\&mem_store\=1\&tags\=10\&order_by\=order+by+id+desc


"""


"""
Method: POST

curl -s --header "Referer: http://<modem_ip>/index.html" -d "isTest=false&goformId=SEND_SMS&notCallback=true&Number=<phone_number>&sms_time=<date>&MessageBody=<message>&ID=-1&encode_type=UNICODE"
http://<modem_ip>/goform/goform_set_cmd_process

phone_number is urlencoded
message is hexencoded

if is OK {"result":"success"}
"""

"""
Method: POST

curl -s --header "Referer: http://192.168.0.1/index.html" -d 'isTest=false&goformId=LOGIN&password=52Yf3555' http://192.168.0.1/goform/goform_set_cmd_process

"""