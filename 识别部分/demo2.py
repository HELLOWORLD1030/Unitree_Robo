import numpy as np
import collections
import pyrealsense2 as rs
import socket
import cv2
from pyzbar import pyzbar

# 定义字典存放颜色分量上下限
# 例如：{颜色: [min分量, max分量]}
# {'red': [array([160,  43,  46]), array([179, 255, 255])]}

def getColorList():
    dict = collections.defaultdict(list)

    # 黑色
   # lower_black = np.array([0, 0, 0])
   # upper_black = np.array([180, 255, 46])
   # color_list = []
   # color_list.append(lower_black)
   # color_list.append(upper_black)
   # dict['black'] = color_list

    # #灰色
    # lower_gray = np.array([0, 0, 46])
    # upper_gray = np.array([180, 43, 220])
    # color_list = []
    # color_list.append(lower_gray)
    # color_list.append(upper_gray)
    # dict['gray']=color_list

    # # 白色
    # lower_white = np.array([0, 0, 221])
    # upper_white = np.array([180, 30, 255])
    # color_list = []
    # color_list.append(lower_white)
    # color_list.append(upper_white)
    # dict['white'] = color_list

    # 红色
    lower_red = np.array([156, 100, 90])
    upper_red = np.array([180, 225, 200])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red'] = color_list

 
  #  红色2
    lower_red = np.array([0, 100, 90])
    upper_red = np.array([10, 225, 200])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red2'] = color_list
    
    # 橙色
   # lower_orange = np.array([11, 43, 46])
   # upper_orange = np.array([25, 255, 255])
   # color_list = []
   # color_list.append(lower_orange)
   # color_list.append(upper_orange)
   # dict['orange'] = color_list

    # 黄色
    lower_yellow = np.array([15, 80, 100])
    upper_yellow = np.array([34, 225, 200])
    color_list = []
    color_list.append(lower_yellow)
    color_list.append(upper_yellow)
    #dict['yellow'] = color_list

   #  绿色
    lower_green = np.array([50, 150, 100])
    upper_green = np.array([77, 225, 200])
    color_list = []
    color_list.append(lower_green)
    color_list.append(upper_green)
    dict['green'] = color_list

    # 青色
    #lower_cyan = np.array([78, 120, 90])
    #upper_cyan = np.array([89, 255, 255])
    #color_list = []
    #color_list.append(lower_cyan)
    #color_list.append(upper_cyan)
    #dict['cyan'] = color_list

    # 蓝色
    lower_blue = np.array([85, 100, 75])
    upper_blue = np.array([124, 255, 200])
    color_list = []
    color_list.append(lower_blue)
    color_list.append(upper_blue)
    dict['blue'] = color_list

   # 紫色
    lower_purple = np.array([125, 90, 60])
    upper_purple = np.array([155, 225, 200])
    color_list = []
    color_list.append(lower_purple)
    color_list.append(upper_purple)
    dict['purple'] = color_list

    return dict

def single_np(arr,target):
    arr=np.array(arr)
    mask=(arr==target)
    arr_new=arr[mask]
    return arr_new.size


def get_color():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    ip_remote = '192.168.123.161'  # upboard IP
    port_remote = 32000  # port
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # setup socket

    max_center = 0
    max_x = 0
    flag=0
    color="green"
    isScanQr=False
    while (1):
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        barcodes=pyzbar.decode(color_image)
        if len(barcodes)!=0:
            isScanQr=True
            for barcode in barcodes:
                barcodeData=barcode.data.decode("utf-8")
                print(barcodeData)
                if barcodeData=="get down":
                    # udp_socket.sendto(str(-1).encode("utf-8"), (ip_remote, port_remote))
                    send=str(-1).encode("utf-8")
                if barcodeData=="nod":
                    send=str(-2).encode("utf-8")
                if barcodeData=="turn left":
                    send=str(-3).encode("utf-8")
                if barcodeData=="turn right":
                    send=str(-4).encode("utf-8")
                
        else:

            cv2.GaussianBlur(color_image,(9,9),sigmaX=5)
            hsvFrame = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            
            color_dict =getColorList()
            flag=flag+1
            if flag==50:
                print(flag)
                for d in color_dict: 
                    color_mask= cv2.inRange(hsvFrame, color_dict[d][0], color_dict[d][1])
                    if (single_np(color_mask,255))>=150:
                        print(d+"10086")
                        color=d
                        break
            else:
                print(color)
                color_mask=cv2.inRange(hsvFrame, color_dict[color][0], color_dict[color][1])
            kernal = np.ones((5, 5), "uint8")
            color_mask = cv2.dilate(color_mask, kernal)
            res = cv2.bitwise_and(color_image, color_image, mask=color_mask)

            contours, hierarchy = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv_contours = []
            for contour in contours:
                area1 = cv2.contourArea(contour)
                if area1 <= 8000:
                    cv_contours.append(contour)
                    x, y, w, h = cv2.boundingRect(contour)
                    res[y:y + h, x:x + w] = 255
                else:
                    continue
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if (area > 250):
                    x, y, w, h = cv2.boundingRect(contour)
                    max_x = max(x, max_x)
                    max_center = x + (int)(0.5 * w)
                    cv2.circle(color_image, (x + (int)(0.5 * w), y + (int)(0.5 * h)), 1, (0, 0, 255), -1)
                    color_image = cv2.rectangle(color_image, (x, y),
                                                (x + w, y + h),
                                                (0, 0, 255), 2)
            if not isScanQr:
                send=str(max_center).encode("utf-8")
        print(send)
        udp_socket.sendto(send, (ip_remote, port_remote))

        # show image
        cv2.imshow("Color Detection in Real-Time", color_image)

        # press Esc to stop
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()

if __name__ == '__main__':

    getColorList()
    get_color()
