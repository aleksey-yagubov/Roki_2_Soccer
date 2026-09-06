import cv2
import math
import time
import numpy as np

class Blob:
    def __init__ (self, x_, y_, w_, h_, a_ = 0):
        self.x_ = x_
        self.y_ = y_
        self.w_ = w_
        self.h_ = h_
        self.a = a_

    def x (self):
        return self.x_

    def y (self):
        return self.y_

    def w (self):
        return self.w_

    def h (self):
        return self.h_

    def cx (self):
        return int (self.x_ + self.w_ / 2)

    def cy (self):
        return int (self.y_ + self.h_ / 2)

    def rect (self):
        return (self.x_, self.y_, self.w_, self.h_)

    def pixels(self):
        return self.a

class Line:
    def __init__ (self, x1_, y1_, x2_, y2_, theta_):
        self.x1_ = x1_
        self.y1_ = y1_
        self.x2_ = x2_
        self.y2_ = y2_
        self.theta_ = theta_

    def x1 (self):
        return self.x1_

    def y1 (self):
        return self.y1_

    def x2 (self):
        return self.x2_

    def y2 (self):
        return self.y2_

    def theta (self):
        return self.theta_

    def line (self):
        return (self.x1_, self.y1_, self.x2_, self.y2_)

class Image:
    def __init__ (self, img_, copy = True):
        self.img = img_.copy () if copy else img_
        self._lab_img = None

    def lab(self):
        if self._lab_img is None:
            self._lab_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB)
        return self._lab_img

    def find_blobs (self, ths, roi = None, pixels_threshold = 100, area_threshold = 100, merge=False, margin=0, invert = False):
        #print("find blobs")
        
        blobs = []
        if roi == None:
            w = h = 1
            labimg = self.lab()
        else:
            x, y, w, h = roi
            labimg = self.lab()[y:(y+h), x:(x+w), :]
        masks = []
        if(w > 0 and h > 0):
            for th in ths:
                low_th = (int(th[0] * 2.55), th[2] + 128, th[4] + 128)
                high_th = (math.ceil(th[1] * 2.55), th[3] + 128, th[5] + 128)

                mask = cv2.inRange (labimg, low_th, high_th)

                #cv2.imshow("mask for debug", mask)

                if (invert == True):
                    mask = cv2.bitwise_not (mask)

                masks.append (mask)

            #cv2.imshow ("a", mask)

            final_mask = masks [0].copy ()
            if (len (masks) > 1):
                for m in masks [1:]:
                    final_mask = cv2.bitwise_and (final_mask, m)

            output = cv2.connectedComponentsWithStats (final_mask, 8, cv2.CV_32S)

            #labels_count = output      [0]
            #labels       = output      [1]
            stats        = output      [2]
            #labels_count, labels, stats = output[:3]
            sz = stats.shape[0]
            for label_num in range (1, sz):
                area = stats [label_num, cv2.CC_STAT_AREA]
            
                if (area >= pixels_threshold):
                    new_blob = Blob (stats [label_num, cv2.CC_STAT_LEFT],
                                     stats [label_num, cv2.CC_STAT_TOP],
                                     stats [label_num, cv2.CC_STAT_WIDTH],
                                     stats [label_num, cv2.CC_STAT_HEIGHT],
                                     area)

                    #print ("append", area)
                    blobs.append (new_blob)

        return blobs

    def binary (self, th, to_three_channels = False):
        #low  = (th [0], th [2], th [4])
        #high = (th [1], th [3], th [5])
        low = (int(th[0] * 2.55), th[2] + 128, th[4] + 128)
        high = (math.ceil(th[1] * 2.55), th[3] + 128, th[5] + 128)
        labimg = self.lab()
        mask = cv2.inRange(labimg, low, high)
        if to_three_channels:
            return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        return mask

    def find_line_segments (self):
        # - Почему Колумб приплыл в Америку, а не в Индию?
        # - Потому что он плавал по одометрии

        #gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(self.img, 50, 150, apertureSize=3)

        #cv2.imshow ("a", edges)

        lines = cv2.HoughLinesP(edges, rho=1, theta = np.pi / 20, threshold = 50, minLineLength =40, maxLineGap =30)
        #lines = cv2.HoughLinesP(edges, rho=1, theta = np.pi / 20, threshold = 10, minLineLength =40, maxLineGap =30)

        resultant_lines = []

        #print (lines)
        try:
            for line in lines:
                x1, y1, x2, y2 = line [0]
                if x1 == x2: theta = 0
                else: theta = math.atan ((y2 - y1) / (x2 - x1))

                new_line = Line (x1, y1, x2, y2, theta)

                resultant_lines.append (new_line)
        except Exception: pass

        return resultant_lines

    def find_lines (self):
        # - Почему Колумб приплыл в Америку, а не в Индию?
        # - Потому что он плавал по одометрии

        if len(self.img.shape) == 2:
            gray = self.img
        else:
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        #cv2.imshow ("a", edges)

        lines = cv2.HoughLines(edges, rho=2, theta = np.pi / 6, threshold = 20)

        resultant_lines = []
        #print (lines)
        try:
            for line in lines:
                #print(line)
                rho, theta = line[0][0], line[0][1]
                if math.sin(theta) == 0: x1, x2, y1, y2 = rho, rho, 0, 240
                elif math.cos(theta) == 0: x1, x2, y1, y2 = 0, 320, rho, rho
                else:
                    ind = []
                    y = int(rho/math.sin(theta))
                    if 0 <= y <= 240 : 
                        x = 0
                        ind.append([x,y])
                    y = int((rho - 320* math.cos(theta))/math.sin(theta))
                    if 0 <= y <= 240: 
                        x = 320
                        ind.append([x,y])
                    x = int(rho/math.cos(theta))
                    if 0 <= x <= 320:
                        y = 0
                        ind.append([x,y])
                    x = int((rho - 240 * math.sin(theta))/math.cos(theta))
                    if 0 <= x <= 320:
                        y =240
                        ind.append([x,y])
                new_line = Line (ind[0][0], ind[0][1], ind[1][0], ind[1][1], theta)
                print(ind[0][0], ind[0][1], ind[1][0], ind[1][1], rho, theta)
                resultant_lines.append (new_line)
        except Exception: pass

        return resultant_lines

    def draw_line (self, line):
        (x1, y1, x2, y2) = line.line()
        cv2.line (self.img, (x1, y1), (x2, y2), (0, 255, 255), thickness = 1)

    def draw_rectangle (self, rect, color = (255, 0, 0) ):
        (x, y, w, h) = rect
        cv2.rectangle (self.img, (x, y), (x+w, y+h), color, 2)

class Sensor:
    def __init__ (self, filename_):
        self.filename = filename_
        self.img = cv2.imread (self.filename)

    def snapshot (self):
        return Image (self.img)

def main ():
    sensor = Sensor ("rgb_basket.jpg")

    while (True):
        #print ("a")
        img = sensor.snapshot ()

        #blobs = img.find_blobs ((40, 80, -28, 72, -28, 72), 200, 20, True, 10)
        blobs = img.find_blobs ((35, 50, 15, 75, 50, 135), 200, 20, True, 10)

        for blob in blobs:
            #print ("a")
            img.draw_rectangle (blob.rect ())

        #lines = img.find_lines ()

        #for line in lines:
        #    img.draw_line (line.line ())

        cv2.imshow ("objects", img.img)

        time.sleep (0.02)
        
        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()
