import pyrealsense2 as rs           # 版本目前不支援python3.10
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)
#sensor.set_option(rs.option.exposure, 500.000)
#sensor.backlight_compensation

# 每种样本50个图像，一共八种，50*8=400个初始图像，之后再做数据增广
filePath = "D:/ICLAB/"
id = "1"

i = 0
j=0
#只是采集图像，不用深度信息

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('D:/ICLAB/output8.avi', fourcc, 30.0, (1280, 720))


while 1:

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    # Convert images to numpy arrays 把图像转换为numpy data
    color_image = np.asanyarray(color_frame.get_data())

    out.write(color_image)

    cv2.namedWindow('MyD415')
    cv2.imshow('MyD415', color_image)

    if cv2.waitKey(1) == ord('q'):
       break

out.release()
cv2.destroyAllWindows()
    
