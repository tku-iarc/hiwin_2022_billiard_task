import pyrealsense2 as rs           # 版本目前不支援python3.10
import numpy as np
import cv2
import YOLO_Detect

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


if __name__ == '__main__':
    # 每种样本50个图像，一共八种，50*8=400个初始图像，之后再做数据增广
    filePath = "/home/oscaryang/picture/second_height/"
    id = "1"

    i = 1000
    j = 1000
    #只是采集图像，不用深度信息
    while 1:

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # Convert images to numpy arrays 把图像转换为numpy data
        color_image = np.asanyarray(color_frame.get_data())
        # YOLO_Detect.detect_ALL(color_image,0.5)

        cv2.namedWindow('MyD415')
        # cv2.line(color_image, (640,0), (640,720), (0,0,255), 1)
        # cv2.line(color_image, (0,360), (1280,360), (0,0,255), 1)
        # cv2.circle(color_image, (640,360), 25, (0,0,255), 1)
        cv2.imshow('MyD415', color_image)

        key = cv2.waitKey(1)

        imageName = "second_height" + "_" + str(i) + ".jpg"

        imagePath = filePath + imageName



        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            pipeline.stop()
            break
        elif key == ord('s'):  # 按下s键时保存并退出
            cv2.imwrite(imagePath, color_image)
            i += 1
            #cv2.destroyAllWindows()

        j=j+1
        #方便看保存到第几张图了
        
        print("iter:",j,"i:", i)