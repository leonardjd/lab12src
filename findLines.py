#!/usr/bin/python3
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError

cam_vs_file = "FILE"                #CAM if camera, FILE if file
Xc = -1
yc = -1
numberOfClicks = 0


def get_mouse_cb(event, x, y, flags, param):
    global xc, yc, numberOfClicks
    if event == cv2.EVENT_LBUTTONDOWN:
        xc = x
        yc = y
        numberOfClicks += 1

class TakePhoto:
    def __init__(self):
        self.image_received = False
        # img_topic = "/raspicam_node/image/compressed"
        img_topic = "/camera/image/compressed"
        self.image_sub = rospy.Subscriber(
            img_topic, CompressedImage, self.callback, queue_size=10)
        

    def callback(self, data):
        self.image_received = True
        np_arr = np.frombuffer(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#        cv2.imshow("Orignal", self.img)
#        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

    def get_img(self):
        if self.image_received == False:
            print("None")
            return
        return self.img

    def save_img(self, img_title):
        if self.image_received == False:
            print("None")
            return
        cv2.imwrite(img_title, self.img)

    def disp_img(self, img_title):
        if self.image_received == False:
            print("None")
            return
        cv2.imshow(img_title, self.img)

rospy.init_node("Color_Lane_Following")
camera = TakePhoto()
time.sleep(1)
rate = rospy.Rate(10)
if(cam_vs_file == "CAM"):      #camera if CAM, or FILE from file
   img = camera.get_img()
else:
   # read image
   img = cv2.imread("/home/parallels/Documents/color/Lab13/smallRoadFar.jpg")
   #img = cv2.imread('Lab13/smallRoadClose.jpg') # Seccond Imageimg 
   print("img size ", img.shape)

imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# show image
cv2.namedWindow("image")

# define the events for the
# mouse_click.

# array to store picture values
rows, cols = (10, 3)
hsvValues = np.zeros((rows, cols), dtype = int)
pixelXYCords = np.zeros((rows, 2), dtype = int)
print("************** Click Information ************")
old_num_clk = numberOfClicks
cv2.setMouseCallback('image', get_mouse_cb)
while numberOfClicks < 9:
   cv2.imshow("image", img)
   key = cv2.waitKey(1) & 0xFF
   if key == ord("c"):
      break
   if(old_num_clk < numberOfClicks):
       print("numberOfClicks ", numberOfClicks)
       r, g, b = img[yc, xc]
       h, s, v = imgHSV[yc,xc]
       hsvValues[numberOfClicks][0] = h
       hsvValues[numberOfClicks][1] = s
       hsvValues[numberOfClicks][2] = v
       pixelXYCords[numberOfClicks][0] = xc
       pixelXYCords[numberOfClicks][1] = yc
       print("Coordinates ", pixelXYCords[numberOfClicks])   #x,y
       a = (b, g, r)
       colr = np.uint8([[a]])
       hsvColr = cv2.cvtColor(colr, cv2.COLOR_BGR2HSV)
       print(" RGB, HSV ", r, g, b, "  ",h,s,v)
       old_num_clk += 1
       print()
print("DONE Clicking Image")
print("*********** Clicking Results ***************")
print("pixelXYCords ", pixelXYCords)
print("hsvValues    ", hsvValues)
print("********************************************")
#Set Color Limits
green_hsv_lower = (75, 125, 0)       #was 39,142
green_hsv_upper = (81, 176, 255)     #was 42,174
red_hsv_lower = ( 172, 125, 0)
red_hsv_upper = (176, 142, 255)
blk_hsv_lower = (100, 24, 0)
blk_hsv_upper = (112, 46,255)
wht_hsv_lower = (50,3,0)
wht_hsv_upper = (94, 46, 255)
lower_color = np.array(green_hsv_lower)
upper_color = np.array(green_hsv_upper)

    # Create a mask of all pixels that fall within the color range
print("************ Color Limits ****************")
print("green_hsv_lower ", green_hsv_lower)
print("green_hsv_upper ", green_hsv_upper)
print()
print("red_hsv_lower   ", red_hsv_lower)
print("red_hsv_upper   ", red_hsv_upper)
print()
print("white_hsv_lower ", wht_hsv_lower)
print("white_hsv_upper ", wht_hsv_upper)
print()
print("black_hsv_lower ", blk_hsv_lower)
print("black_hsv_upper ", blk_hsv_upper)
#mask = cv2.inRange(imgHSV, lower_color, upper_color)
maskg = cv2.inRange(imgHSV, green_hsv_lower, green_hsv_upper)
maskr = cv2.inRange(imgHSV, red_hsv_lower, red_hsv_upper)
maskw = cv2.inRange(imgHSV, wht_hsv_lower, wht_hsv_upper)
#width, height = mask.size
cv2.imshow("HSV image ",imgHSV)
cv2.imshow("green_mask", maskg)
cv2.imshow("red_mask", maskr)
cv2.imshow("White_mask", maskw)
print()

# get green straight line  x = m*y + b   x = [y 1] (m b)'
print("*********** Green ************")
V = cv2.findNonZero(maskg)
(x,y) = (V[:,0,0],V[:,0,1])
A = np.vstack([y, np.ones(len(y))]).T
x1,y1 = np.array(pixelXYCords[1], dtype=np.float32)
x2,y2 = np.array(pixelXYCords[2], dtype=np.float32)
print("int pixel XY Coor 1 ", pixelXYCords[1])
print("int pixel XY Coor 2 ", pixelXYCords[2])
mg = (x2-x1)/(y2-y1)
cg = x2 - mg*y2
print(" mg, cg " , mg,cg)
      #m, c = np.linalg.lstsq(A,x, rcond=None)[0]
      #print("m c ", m,c)
R = np.abs(x - A.dot([mg, cg]))
Rmax = np.amax(R,axis = None)
iRmax = np.where(R < 50.)[0]
print("size iRmax ", len(iRmax), len(R))
xs = x[iRmax]
ys = y[iRmax]
As = np.vstack([ys, np.ones(len(ys))]).T
msg,csg = np.linalg.lstsq(As, xs, rcond=None)[0]
print("Green ms cs ",msg,csg)
maskgs = np.zeros_like(maskg)
maskgs[ys,xs] = 250
cv2.imshow("green mask sel ", maskgs)
print("*********** Red Right ************")
V = cv2.findNonZero(maskr)
(x,y) = (V[:,0,0],V[:,0,1])
A = np.vstack([y, np.ones(len(y))]).T
x3,y3 = np.array(pixelXYCords[3], dtype=np.float32)
x4,y4 = np.array(pixelXYCords[4], dtype=np.float32)
print("int pixel XY Coor 3 ", pixelXYCords[3])
print("int pixel XY Coor 4 ", pixelXYCords[4])
mr1 = (x4-x3)/(y4-y3)
cr1 = x4 - mr1*y4
print(" mr1, cr1 " , mr1,cr1)
R = np.abs(x - A.dot([mr1, cr1]))
iRmax = np.where(R < 50.)[0]
print("size iRmax ", len(iRmax), len(R))
xs = x[iRmax]
ys = y[iRmax]

As = np.vstack([ys, np.ones(len(ys))]).T
msr1,csr1 = np.linalg.lstsq(As, xs, rcond=None)[0]
print("Red 1 ms cs ",msr1,csr1)
maskr1s = np.zeros_like(maskr)
maskr1s[ys,xs] = 250
cv2.imshow("red Right mask sel ", maskr1s)

print("*********** Red Left ************")
x5,y5 = np.array(pixelXYCords[5], dtype=np.float32)
x6,y6 = np.array(pixelXYCords[6], dtype=np.float32)
print("int pixel XY Coor 5 ", pixelXYCords[5])
print("int pixel XY Coor 6 ", pixelXYCords[6])
mr2 = (x6-x5)/(y6-y5)
cr2 = x5 - mr2*y5
print(" mr2, cr2 " , mr2,cr2)
R = np.abs(x - A.dot([mr2, cr2]))
Rmax = np.amax(R,axis = None)
iRmax = np.where(R < 50.)[0]
print("size iRmax ", len(iRmax), len(R))
xs = x[iRmax]
ys = y[iRmax]

As = np.vstack([ys, np.ones(len(ys))]).T
msr2,csr2 = np.linalg.lstsq(As, xs, rcond=None)[0]
print("Red 2 ms cs ",msr2,csr2)
maskr2s = np.zeros_like(maskr)
maskr2s[ys,xs] = 250
cv2.imshow("red Left mask sel ", maskr2s)



#
##  x = m*y + c
## yo = (c2-c1)/(m1-m2)
## xo = m1*yo + c1
#
yo = (csr1 - csg)/(msg - msr1)
xo = msg*yo + csg
y1 = (csr2 - csg)/(msg - msr2)
x1 = msg*y1 + csg
y12= (csr2 - csr1)/(msr1 - msr2)
x12= msr2*y12 + csr2
print()
print("************* Vanishing Points **************")
print("Right red and green vanishing point xo,yo ",xo, yo)
print("Left red and green vanishing point  x1,y1 ", x1,y1)
print("Right and Left reds vanishing point x12,y12 ", x12,y12)
print()

cv2.waitKey(0)
cv2.destroyAllWindows()
#############################################################################################
