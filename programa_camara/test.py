import camera
import network
import time

try:
    print('Tomando una foto')
    camera.init(0, format=camera.JPEG, fb_location=camera.PSRAM)
    buffer =camera.capture()
    print(len(buffer))
    filepath = "captured_image.jpg"
    file=open(filepath,"w")
    file.write(buffer)
    file.close()
except Exception as e:
    print(e)
    
finally:
    print('finalizando camara')
    camera.deinit()