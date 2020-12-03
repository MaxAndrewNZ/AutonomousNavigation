
import time,zmq,pickle,sys,cv2
import numpy as np

############ connecting to landrov ##################
context = zmq.Context()
sensor_socket = context.socket(zmq.SUB)
sensor_socket.connect("tcp://192.168.8.106:5557")
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'depthimage') 
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'depthraw') 
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'rgbimage') 

print('connected to landrov sensor server')


while 1:
    k = cv2.waitKey(10)

    if k!=-1:
        if k  == 27 or k == ord('q'):
            break
    if  len(zmq.select([sensor_socket],[],[],0)[0]):
        topic,buf = sensor_socket.recv_multipart()
        if topic == b'rgbimage':
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8),cv2.IMREAD_COLOR)
            cv2.imshow('img',img)
        if topic == b'depthimage':
            depth = cv2.imdecode(np.fromstring(buf, dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            cv2.imshow('depth', depth)
        if topic == b'depthraw':
            depth = cv2.fromstring(buf, dtype=np.float32)
            print(depth)
            cv2.imshow('depth raw', depth)
