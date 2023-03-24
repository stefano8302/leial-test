# encoding:utf-8

from tokenize import Pointfloat
import cv2
import numpy as np
import time
import json
import os
import rospy
from visualization_msgs.msg import Marker
from PIL import Image
from threading import Thread
import tkFileDialog as filedialog
import Tkinter as tk
import RPi.GPIO as GPIO
from moving_utils import Movement

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"  # Adaptive seeed


class Object_detect(Movement):
    def __init__(self, camera_x=150, camera_y=-10):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # get path of file
        dir_path = os.path.dirname(__file__)
	    # 移动角度
        self.move_angles = [
           [-7.11, -6.94, -55.01, -24.16, 0, -38.84],  # init the point
            [-1.14, -10.63, -87.8, 9.05, -3.07, -37.7],  # point to grab
            [17.4, -10.1, -87.27, 5.8, -2.02, -37.7],  # point to grab
        ]
        # 移动坐标
        self.move_coords = [
        [120.1, -141.6, 240.9, -173.34, -8.15, -83.11],  # above the red bucket
        [208.2, -127.8, 260.9, -157.51, -17.5, -71.18],  # above the yello bucket
        [209.7, -18.6, 230.4, -168.48, -9.86, -39.38],
        [196.9, -64.7, 232.6, -166.66, -9.44, -52.47],
        [126.6, -118.1, 305.0, -157.57, -13.72, -75.3],
        ]
        # 判断连接设备:ttyUSB*为M5，ttyACM*为seeed
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20,GPIO.OUT)
        GPIO.setup(21,GPIO.OUT)
        GPIO.output(20,1)
        GPIO.output(21,1)

        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # load model of img recognition
        # self.model_path = os.path.join(dir_path, "frozen_inference_graph.pb")
        # self.pbtxt_path = os.path.join(dir_path, "graph.pbtxt")
        # self.label_path = os.path.join(dir_path, "labels.json")
        # load class labels
        # self.labels = json.load(open(self.label_path))
        # use to calculate coord between cube and mycobot
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mycobot
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mycobot
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # if IS_CV_4:
        #     self.net = cv2.dnn.readNetFromTensorflow(self.model_path, self.pbtxt_path)
        # else:
        #     print('Load tensorflow model need the version of opencv is 4.')
        #     exit(0)
        # init a node and a publisher
        rospy.init_node("marker", anonymous=True)
        self.pub = rospy.Publisher('/cube', Marker, queue_size=1)
        # init a Marker
        self.marker = Marker()
        self.marker.header.frame_id = "base" # it was "/joint1"
        self.marker.ns = "cube"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0

        # marker position initial
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

        self.cache_x = self.cache_y = 0
    # publish marker

    def pub_marker(self, x, y, z=0.03):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.g = self.color
        self.pub.publish(self.marker)

    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mycobot
        self.pub_angles(self.move_angles[0], 30)
        time.sleep(1.5)
        self.pub_angles(self.move_angles[1], 30)
        time.sleep(1.5)
        self.pub_angles(self.move_angles[2], 30)
        time.sleep(1.5)
        # send coordinates to move mycobot
        self.pub_coords([x, y, 165,  -178.9, -1.57, -25.95], 20, 1)
        time.sleep(1.5)
        self.pub_coords([x, y, 90,  -178.9, -1.57, -25.95], 20, 1)
        GPIO.output(20,0)
        GPIO.output(21,0)
        time.sleep(1.5)
        # open pump
        time.sleep(0.5)
        self.pub_angles(self.move_angles[2], 30)
        time.sleep(3)
        self.pub_marker(
            self.move_coords[2][0]/1000.0, self.move_coords[2][1]/1000.0, self.move_coords[2][2]/1000.0)

        self.pub_angles(self.move_angles[1], 30)
        time.sleep(1.5)
        self.pub_marker(
            self.move_coords[3][0]/1000.0, self.move_coords[3][1]/1000.0, self.move_coords[3][2]/1000.0)

        self.pub_angles(self.move_angles[0], 30)
        time.sleep(1.5)
        self.pub_marker(
            self.move_coords[4][0]/1000.0, self.move_coords[4][1]/1000.0, self.move_coords[4][2]/1000.0)

        self.pub_coords(self.move_coords[color], 20, 1)
        self.pub_marker(self.move_coords[color][0]/1000.0, self.move_coords[color]
                        [1]/1000.0, self.move_coords[color][2]/1000.0)
        time.sleep(2)
        # close pump
        GPIO.output(20,1)
        GPIO.output(21,1)
        if color == 1:
           self.pub_marker(
               self.move_coords[color][0]/1000.0+0.04, self.move_coords[color][1]/1000.0-0.02)
        elif color == 0:
           self.pub_marker(
               self.move_coords[color][0]/1000.0+0.03, self.move_coords[color][1]/1000.0)
        self.pub_angles(self.move_angles[0], 20)
        time.sleep(3)

    # decide whether grab cube

    def decide_move(self, x, y, color):
        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
	    if y>20:
		x-=5
                # print x,y
            self.move(x+15, y+15, color)

    # init mycobot
    def run(self):
        for _ in range(5):
            self.pub_angles([-7.11, -6.94, -55.01, -24.16, 0, -38.84], 20)
            print(_)
            time.sleep(0.5)


    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img
        cv2.rectangle(
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)

    # get points of two aruco
    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and mycobot
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0/ratio

    # calculate the coords between cube and mycobot
    def get_position(self, x, y):
        return ((y - self.c_y)*self.ratio + self.camera_x), ((x - self.c_x)*self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """

    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2*0.2):int(self.y1*1.15),
                              int(self.x1*0.7):int(self.x2*1.15)]
        return frame

    # according the class_id to get object name
    def id_class_name(self, class_id):
        for key, value in self.labels.items():
                if class_id == int(key):
                    return value
    # detect object

    def obj_detect(self, img, goal):
        # rows, cols = frame.shape[:-1]
        # Resize image and swap BGR to RGB.
        # blob = cv2.dnn.blobFromImage(
        #     frame,
        #     size=(300, 300),
        #     mean=(0, 0, 0),
        #     swapRB=True,
        #     crop=False,
        # )

        # Detecting.
        # self.net.setInput(blob)
        # out = self.net.forward()
        # x, y = 0, 0

        # Processing result.
        # for detection in out[0, 0, :, :]:
        #     score = float(detection[2])
        #     if score > 0.3:
        #         class_id = detection[1]
        #         left = detection[3] * cols
        #         top = detection[4] * rows
        #         right = detection[5] * cols
        #         bottom = detection[6] * rows
        #         if abs(right + bottom - left - top) > 380:
        #             continue
        #         x, y = (left + right) / 2.0, (top + bottom) / 2.0
        #         cv2.rectangle(
        #             frame,
        #             (int(left), int(top)),
        #             (int(right), int(bottom)),
        #             (0, 230, 0),
        #             thickness=2,
        #         )
        #         cv2.putText(
        #             frame,
        #             "{}: {}%".format(self.id_class_name(class_id),round(score * 100, 2)),
        #             (int(left), int(top) - 10),
        #             cv2.FONT_HERSHEY_COMPLEX_SMALL,
        #             1,
        #             (243, 0, 0),
        #             2,
        #         )
        i = 0
        MIN_MATCH_COUNT = 10
        sift = cv2.SIFT_create() # it was sift = cv2.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp = []
        des = []

        for i in goal:
            kp0, des0 = sift.detectAndCompute(i, None)
            kp.append(kp0)
            des.append(des0)
        # kp1, des1 = sift.detectAndCompute(goal, None)
        kp2, des2 = sift.detectAndCompute(img, None)

        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)   # or pass empty dictionary
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        x, y = 0, 0
        try:
            for i in range(len(des)):
                matches = flann.knnMatch(des[i], des2, k=2)
                # store all the good matches as per Lowe's ratio test.  根据Lowe比率测试存储所有良好匹配项。
                good = []
                for m, n in matches:
                    if m.distance < 0.7*n.distance:
                        good.append(m)

                # When there are enough robust matching point pairs 当有足够的健壮匹配点对（至少个MIN_MATCH_COUNT）时
                if len(good) > MIN_MATCH_COUNT:

                    # extract corresponding point pairs from matching 从匹配中提取出对应点对
                    # query index of small objects, training index of scenarios 小对象的查询索引，场景的训练索引
                    src_pts = np.float32(
                        [kp[i][m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                    dst_pts = np.float32(
                        [kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

                    # Using matching points to find homography matrix in cv2.ransac 利用匹配点找到CV2.RANSAC中的单应矩阵
                    M, mask = cv2.findHomography(
                        src_pts, dst_pts, cv2.RANSAC, 5.0)
                    matchesMask = mask.ravel().tolist()
                    # Calculate the distortion of image, that is the corresponding position in frame 计算图1的畸变，也就是在图2中的对应的位置
                    h, w, d = goal[i].shape
                    pts = np.float32(
                        [[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
                    dst = cv2.perspectiveTransform(pts, M)
                    ccoord = (dst[0][0]+dst[1][0]+dst[2][0]+dst[3][0])/4.0
                    cv2.putText(img, "{}".format(ccoord), (50, 60), fontFace=None,
                                fontScale=1,  color=(0, 255, 0), lineType=1)
                    print(format(dst[0][0][0]))
                    x = (dst[0][0][0]+dst[1][0][0] +
                         dst[2][0][0]+dst[3][0][0])/4.0
                    y = (dst[0][0][1]+dst[1][0][1] +
                         dst[2][0][1]+dst[3][0][1])/4.0

                    # bound box  绘制边框
                    img = cv2.polylines(
                        img, [np.int32(dst)], True, 244, 3, cv2.LINE_AA)
                    # cv2.polylines(mixture, [np.int32(dst)], True, (0, 255, 0), 2, cv2.LINE_AA)
        except Exception as e:
            pass

        # else:
        #     if len(good) < MIN_MATCH_COUNT:
        #         i += 1
        #         if (i % 10) == 0:
        #             print("Not enough matches are found - %d/%d" %
        #                   (len(good), MIN_MATCH_COUNT))

        #     matchesMask = None
        if x+y > 0:
            return x, y
        else:
            return None


def run():

    # Object_detect().take_photo()
    # Object_detect().cut_photo()
    # goal = Object_detect().distinguist()
    goal = []
    path = '/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_ai/local_photo/img'

    for i,j,k in os.walk(path):
        for l in k:
            goal.append(cv2.imread(path+'/{}'.format(l)))
    cap_num = 0
    cap = cv2.VideoCapture(cap_num)
    if not cap.isOpened():
        cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot
    detect.run()
    
    _init_ = 20  # 
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
    while cv2.waitKey(1) < 0:
       # read camera
        _,frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)
        

        if _init_ > 0:
            _init_-=1
            continue
        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure",frame)
                continue
            else:
                x1,x2,y1,y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame,x1,y1)
                detect.draw_marker(frame,x2,y2)
                detect.sum_x1+=x1
                detect.sum_x2+=x2
                detect.sum_y1+=y1
                detect.sum_y2+=y2
                init_num+=1
                continue
        elif init_num==20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0, 
                (detect.sum_y1)/20.0, 
                (detect.sum_x2)/20.0, 
                (detect.sum_y2)/20.0, 
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num+=1
            continue

        # calculate params of the coords between cube and mycobot
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure",frame)
                continue
            else:
                x1,x2,y1,y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame,x1,y1)
                detect.draw_marker(frame,x2,y2)
                detect.sum_x1+=x1
                detect.sum_x2+=x2
                detect.sum_y1+=y1
                detect.sum_y2+=y2
                nparams+=1
                continue
        elif nparams==10:
            nparams+=1
            # calculate and set params of calculating real coord between cube and mycobot
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0, 
                (detect.sum_y1+detect.sum_y2)/20.0, 
                abs(detect.sum_x1-detect.sum_x2)/10.0+abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print "ok"
            continue
        # get detect result
        detect_result = detect.obj_detect(frame,goal)
        if detect_result is None:
            cv2.imshow("figure",frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mycobot
            real_x, real_y = detect.get_position(x, y)
            if num == 5:
                detect.pub_marker(real_sx/5.0/1000.0, real_sy/5.0/1000.0)
                detect.decide_move(real_sx/5.0, real_sy/5.0, detect.color)
                num = real_sx = real_sy = 0

            else:
                num += 1
                real_sy += real_y
                real_sx += real_x

        cv2.imshow("figure",frame)
    
    
if __name__ == "__main__":
    run()
    # Object_detect().take_photo()
    # Object_detect().cut_photo()

