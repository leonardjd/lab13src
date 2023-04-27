#!/usr/bin/python3
#use ransac on green and red using findMm() no left red 4/24/2023
# find black
import numpy as np
from sklearn import linear_model, datasets
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
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

def jabs(a,b):
   if(a > b):
      ab = a-b
   else:
      ab = b-a
   return ab

def findMm(im, xgb,ygb, xgt, ygt,prt=0):
   H, S, V = im[ygb,xgb]
   if prt == 1:
      print("Initial HSV infindMm ",H,S,V)
   Hl = H
   Hh = H
   Sl = S
   Sh = S
   yp = ygb-1       #next line up
   xp = xgb
   yf = ygt-1
   while yp > yf:
      for xtst in range(xp-5,xp+5):     #find closest Hue to test value H
         hq,sq,V = im[yp,xtst]          #q question
         if(xtst == xp-5):
            he = jabs(hq,H)              #e evaluate
            hxpt = xtst
            se = jabs(sq, S)
            if prt == 1:
               print("xtst,yp, hq,sq ", xtst,yp,  hq,sq)
         else:
            if jabs(hq, H) < he:
               he = jabs(hq, H)             
               hxpt = xtst
            if jabs(sq, S) < se:
               se = jabs(sq, S)
      xp = hxpt
      if Hh < hq:
         Hh = hq
      if Hl > hq: 
         Hl = hq
      if Sh < sq:
         Sh = sq
      if Sl > sq:
         Sl = sq 
      if prt == 1:
         print("hxpt, Hl,Hh,Sl,Sh ",hxpt, Hl,Hh,  Sl, Sh)
      yp -= 1
   if prt == 1:
      print("inTest yp,xp ", yp,xp)
      print()
   return Hl, Hh, Sl, Sh


rospy.init_node("Color_Lane_Following")
camera = TakePhoto()
time.sleep(1)
rate = rospy.Rate(10)
if(cam_vs_file == "CAM"):      #camera if CAM, or FILE from file
   img = camera.get_img()
   imgsz = img.shape
   print("img size ", imgsz)
	
else:
   # read image
   #img = cv2.imread("/home/parallels/Documents/color/Lab13/smallRoadFar.jpg")
   #img=cv2.imread('/home/parallels/Documents/color/Lab13/smallRoadClose.jpg')
   #img = cv2.imread('/home/parallels/Documents/color/Lab13/black30.jpg')
   #img = cv2.imread('/home/parallels/Documents/color/Lab13/black20_5.jpg')
   img = cv2.imread('/home/parallels/Documents/color/Lab13/black15.jpg')
   #img = cv2.imread('/home/parallels/Documents/color/Lab13/black12.jpg')
   #img = cv2.imread('/home/parallels/Documents/color/Lab13/black8.jpg')


   imgsz = img.shape
   print("img size ", imgsz)

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
start = time.process_time()
#Set Color Limits
#scan green line from lower click to upper click
xgl = pixelXYCords[1][0]
ygl = pixelXYCords[1][1]
xgu = pixelXYCords[2][0]
ygu = pixelXYCords[2][1]
gH = hsvValues[1][0]
gS = hsvValues[1][1]
gHl,gHh,gSl,gSh = findMm(imgHSV, xgl,ygl, xgu, ygu)     #find Green M&m
print("findMm green", gHl, gHh, gSl, gSh)
green_hsv_lower = (75, 125, 0)       #was 39,142
green_hsv_upper = (81, 176, 255)     #was 42,174
green_hsv_lower = (gHl, gSl,0)     #using routine
green_hsv_upper = (gHh, gSh,255)   #using routine
green_hsv_lower = np.array(green_hsv_lower)
green_hsv_upper = np.array(green_hsv_upper)

xrl = pixelXYCords[3][0]
yrl = pixelXYCords[3][1]
xru = pixelXYCords[4][0]
yru = pixelXYCords[4][1]

rHl,rHh, rSl,rSh = findMm(imgHSV, xrl,yrl, xru, yru)   #find Red M&m
print("findMm red ", rHl,rHh, rSl,rSh)
red_hsv_lower = ( 172, 125, 0)
red_hsv_upper = (176, 142, 255)
red_hsv_lower = (rHl, rSl, 0)
red_hsv_upper = (rHh, rSh, 255)
red_hsv_lower = np.array(red_hsv_lower)
red_hsv_upper = np.array(red_hsv_upper)
blk_hsv_lower = (35, 28, 40)
blk_hsv_upper = (90, 47,90)

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

imgHSV[0:290,:] = 0
#mask = cv2.inRange(imgHSV, lower_color, upper_color)
maskg = cv2.inRange(imgHSV, green_hsv_lower, green_hsv_upper)
maskr = cv2.inRange(imgHSV, red_hsv_lower, red_hsv_upper)
maskw = cv2.inRange(imgHSV, wht_hsv_lower, wht_hsv_upper)
maskbk= cv2.inRange(imgHSV, blk_hsv_lower, blk_hsv_upper)
#width, height = mask.size
cv2.imshow("HSV image ",imgHSV)
cv2.imshow("green_mask", maskg)
cv2.imshow("red_mask", maskr)
cv2.imshow("White_mask", maskw)
cv2.imshow("Black_mask", maskbk)
print()

# get green straight line  x = m*y + b   x = [y 1] (m b)'
print("*********** Green ********************************")
V = cv2.findNonZero(maskg)
(x,y) = (V[:,0,0],V[:,0,1])
#Insert Ransac code
X = x.reshape(-1,1)
Y = y.reshape(-1,1)
# Fit line using all data
lr = linear_model.LinearRegression()
lr.fit(Y, X)
print("linear fit ", lr.coef_, lr.intercept_)

# Robustly fit linear model with RANSAC algorithm
#ransac = linear_model.RANSACRegressor(residual_threshold = 15.)
ransac = linear_model.RANSACRegressor()
ransac.fit(Y,X)
mg = ransac.estimator_.coef_
cg = ransac.estimator_.intercept_
print("ransac fit ", mg, cg)
inlier_mask = ransac.inlier_mask_
iRmax = np.where(inlier_mask > 0)[0]
iRmin = np.where(inlier_mask == 0)[0]     #Outliers
#print(f"number of non-zero: {np.count_nonzero(inlier_mask)}" )
print("**** size inlier_mask ***", len(iRmax), inlier_mask.shape)
print("**** size outlier_mask ***", len(iRmin))

#A = np.vstack([y, np.ones(len(y))]).T
#x1,y1 = np.array(pixelXYCords[1], dtype=np.float32)
#x2,y2 = np.array(pixelXYCords[2], dtype=np.float32)
#print("int pixel XY Coor 1 ", pixelXYCords[1])
#print("int pixel XY Coor 2 ", pixelXYCords[2])
#mg = (x2-x1)/(y2-y1)
#cg = x2 - mg*y2
#print(" mg, cg " , mg,cg)
      #m, c = np.linalg.lstsq(A,x, rcond=None)[0]
      #print("m c ", m,c)
#R = np.abs(x - A.dot([mg, cg]))

xs = x[iRmax]
ys = y[iRmax]
As = np.vstack([ys, np.ones(len(ys))]).T
msg,csg = np.linalg.lstsq(As, xs, rcond=None)[0]
print("Green ms cs ",msg,csg)
maskgs = np.zeros_like(maskg)
maskgs[ys,xs] = 250
xg_robot = msg*imgsz[0] + csg
print("Green at robot ",xg_robot)
cv2.imshow("green mask sel ", maskgs)

print("*********** Red Right ************")
V = cv2.findNonZero(maskr)
(x,y) = (V[:,0,0],V[:,0,1])

#Insert Ransac code
X = x.reshape(-1,1)
Y = y.reshape(-1,1)
# Fit line using all data
lr = linear_model.LinearRegression()
lr.fit(Y, X)
print("linear fit ", lr.coef_, lr.intercept_)

# Robustly fit linear model with RANSAC algorithm
#ransac = linear_model.RANSACRegressor(residual_threshold = 15.)
ransac = linear_model.RANSACRegressor()
ransac.fit(Y,X)
mr1 = ransac.estimator_.coef_
cr1 = ransac.estimator_.intercept_
print("ransac fit ", mr1, cr1)
inlier_mask = ransac.inlier_mask_
iRmax = np.where(inlier_mask > 0)[0]
iRmin = np.where(inlier_mask == 0)[0]     #Outliers
#print(f"number of non-zero: {np.count_nonzero(inlier_mask)}" )
print("**** size inlier_mask ***", len(iRmax), inlier_mask.shape)
print("**** size outlier_mask ***", len(iRmin))


xs = x[iRmax]
ys = y[iRmax]
As = np.vstack([ys, np.ones(len(ys))]).T
msr1,csr1 = np.linalg.lstsq(As, xs, rcond=None)[0]
print("Red 1 ms cs ",msr1,csr1)
maskr1s = np.zeros_like(maskr)
maskr1s[ys,xs] = 250
xr1_robot = msr1*imgsz[0] + csr1
print("Right red at robot ",xr1_robot)
cv2.imshow("red Right mask sel ", maskr1s)

#print("*********** Red Left ************")
#x = x[iRmin]
#y = y[iRmin]
#X = x.reshape(-1,1)
#Y = y.reshape(-1,1)
# Fit line using all data
#lr = linear_model.LinearRegression()
#lr.fit(Y, X)
#print("linear fit ", lr.coef_, lr.intercept_)

# Robustly fit linear model with RANSAC algorithm
#ransac = linear_model.RANSACRegressor(residual_threshold = 15.)
#ransac = linear_model.RANSACRegressor()
#ransac.fit(Y,X)
#mr2 = ransac.estimator_.coef_
#cr2 = ransac.estimator_.intercept_
#print("ransac fit ", mr2, cr2)
#inlier_mask = ransac.inlier_mask_
#iRmax = np.where(inlier_mask > 0)[0]
#iRmin = np.where(inlier_mask == 0)[0]     #Outliers
##print(f"number of non-zero: {np.count_nonzero(inlier_mask)}" )
#print("**** size inlier_mask ***", len(iRmax), inlier_mask.shape)
#print("**** size outlier_mask ***", len(iRmin))

#print("size iRmax ", len(iRmax), len(x))
#xs = x[iRmax]
#ys = y[iRmax]

#As = np.vstack([ys, np.ones(len(ys))]).T
#msr2,csr2 = np.linalg.lstsq(As, xs, rcond=None)[0]
#print("Red 2 ms cs ",msr2,csr2)
#maskr2s = np.zeros_like(maskr)
#maskr2s[ys,xs] = 250
#xr2_robot = msr2*imgsz[0] + csr2
#print("Left red at robot ",xr2_robot)
#cv2.imshow("red Left mask sel ", maskr2s)



#
##  x = m*y + c
## yo = (c2-c1)/(m1-m2)
## xo = m1*yo + c1
#
yo = (csr1 - csg)/(msg - msr1)
xo = msg*yo + csg
#y1 = (csr2 - csg)/(msg - msr2)
#x1 = msg*y1 + csg
#y12= (csr2 - csr1)/(msr1 - msr2)
#x12= msr2*y12 + csr2
print()
print("************* Vanishing Points **************")
print("Right red and green vanishing point xo,yo ",xo, yo)
#print("Left red and green vanishing point  x1,y1 ", x1,y1)
#print("Right and Left reds vanishing point x12,y12 ", x12,y12)
print()
print("Processing time ", time.process_time() - start)
ixo = int(xo)
iyo = int(yo)
image2 = cv2.circle(img, (ixo,iyo), radius=3, color=(0, 0, 255), thickness=-1)
cv2.imshow("Drive To ", image2)

cv2.waitKey(0)
cv2.destroyAllWindows()

