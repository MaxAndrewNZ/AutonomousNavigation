
import pygame,time,zmq,pickle,sys,cv2
import numpy as np
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import open3d as o3d


clock = pygame.time.Clock()

############ connecting to landrov ##################
context = zmq.Context()
control_socket = context.socket(zmq.PUB)
control_socket.connect("tcp://192.168.8.106:5556")
print('connected to landrov server')

sensor_socket = context.socket(zmq.SUB)
sensor_socket.connect("tcp://192.168.8.106:5557")
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'depthimage') 
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'rgbimage') 
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'pointcloud')

print('connected to landrov sensor server')

def npToPcd(np):
    print(np)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np)

    return pcd

def followWall():
    if len(zmq.select([sensor_socket],[],[],0)[0]):
        topic,buf = sensor_socket.recv_multipart()
        if topic == b'rgbimage':
            img = np.fromstring(buf, dtype=np.uint8)
        if topic == b'depthimage':
            depth = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), cv2.IMREAD_GRAYSCALE) 
        if topic == b'depthraw':
            depth = cv2.fromstring(buf, dtype=np.float32)
        if topic == b'pointcloud':
            pcd = np.fromstring(buf, dtype=[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')])
            # print("Recieved pointcloud")
            # print(pcd)

        # H, W, _ = img.shape
        # fx, fy, cx, cy = [1, 1, 1, 1]
        # intrinsic = o3d.camera.PinholeCameraIntrinsic(W,H,fx,fy,cx,cy)
        # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img, depth)
        # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        # o3d.visualization.draw_geometries([pcd])
        # Save pointcloud to file
        # filename = "pointCloud.ply"
        # pc = rs.pointcloud()
        # pc.map_to(colour)
        # pointcloud = pc.calculate(depth)
        # pointcloud.export_to_ply(filename, colour)
        
    return 0.0, 0.0

worldx = 50
worldy = 50
world = pygame.display.set_mode([worldx,worldy])

done = False
cnt = 0
leftMotor = 0.0 
rightMotor = 0.0
speed = 0.3
turningSpeed = 0.6
while not done:
    cnt += 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit(); sys.exit()
            done = True

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT or event.key == ord('a'):
                print('left')
                leftMotor -= turningSpeed
                rightMotor += turningSpeed
            if event.key == pygame.K_RIGHT or event.key == ord('d'):
                leftMotor += turningSpeed
                rightMotor -= turningSpeed
                print('right')
            if event.key == pygame.K_UP or event.key == ord('w'):
                leftMotor += speed
                rightMotor += speed
                print('forward')
            if event.key == pygame.K_DOWN or event.key == ord('s'):
                leftMotor -= speed
                rightMotor -= speed
                print('back')

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT or event.key == ord('a'):
                leftMotor += turningSpeed
                rightMotor -= turningSpeed
                print('left stop')
            if event.key == pygame.K_RIGHT or event.key == ord('d'):
                leftMotor -= turningSpeed
                rightMotor += turningSpeed
                print('right stop')
            if event.key == pygame.K_UP or event.key == ord('w'):
                leftMotor -= speed
                rightMotor -= speed
                print('forwards stop')
            if event.key == pygame.K_DOWN or event.key == ord('s'):
                leftMotor += speed
                rightMotor += speed
                print('back stop')
            if event.key == ord('q'):
                leftMotor = 0.0
                rightMotor = 0.0
                pygame.quit()
                sys.exit()
                done = True 

        cmd = (leftMotor, rightMotor)
        # print(cmd)
        control_socket.send_multipart([b'motor',pickle.dumps(cmd,0)]) 
    clock.tick(10)
    
pygame.quit()

