#!/usr/bin/env python
import cv2
import base64
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scipy.interpolate import UnivariateSpline


class PencilSketch:
    """Pencil sketch effect
        A class that applies a pencil sketch effect to an image.
        The processed image is overlayed over a background image for visual
        effect.
    """

    def __init__(self, (width, height), bg_gray='pencilsketch_bg.jpg'):
        """Initialize parameters
            :param (width, height): Image size.
            :param bg_gray: Optional background image to improve the illusion
                            that the pencil sketch was drawn on a canvas.
        """
        self.width = width
        self.height = height

        # try to open background canvas (if it exists)
        self.canvas = cv2.imread(bg_gray, cv2.CV_8UC1)
        if self.canvas is not None:
            self.canvas = cv2.resize(self.canvas, (self.width, self.height))

    def render(self, img_rgb):
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (21, 21), 0, 0)
        img = cv2.divide(img_gray, img_blur, scale=256)
        img = cv2.equalizeHist(img)
        if self.canvas is not None:
            img_blend = cv2.multiply(img_blend, self.canvas, scale=1./256)

        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)


class WarmingFilter:

    def __init__(self):
        pass
    def render(self, image):
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            return gray_image


class Vignette:
    def __init__(self):
       pass

    def render(self, img):
       # generating vignette mask using Gaussian kernels
       rows, cols = img.shape[:2]
       kernel_x = cv2.getGaussianKernel(cols,200)
       kernel_y = cv2.getGaussianKernel(rows,300)
       kernel = kernel_y * kernel_x.T
       mask = 256 * kernel / np.linalg.norm(kernel) * 2
       output = np.copy(img)

       # applying the mask to each channel in the input image
       for i in range(3):
         output[:,:,i] = output[:,:,i] * mask
       return output

class Cartoonizer:

    def __init__(self):
        pass

    def render(self, img_rgb):
        numDownSamples = 4       # number of downscaling steps
        numBilateralFilters = 14  # number of bilateral filtering steps

        img_color = img_rgb
        for _ in xrange(numDownSamples):
            img_color = cv2.pyrDown(img_color)

        for _ in xrange(numBilateralFilters):
            img_color = cv2.bilateralFilter(img_color, 9, 9, 7)

        for _ in xrange(numDownSamples):
            img_color = cv2.pyrUp(img_color)

        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
        img_blur = cv2.medianBlur(img_gray, 7)

        img_edge = cv2.adaptiveThreshold(img_blur, 255,
                                         cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY, 9, 2)
        img_edge = cv2.cvtColor(img_edge, cv2.COLOR_GRAY2RGB)
        return cv2.bitwise_and(img_color, img_edge)


class UiPublisher(object):
    def __init__(self):
        self.imagePub = rospy.Publisher("/filters_base64", String)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/event_trigger",String,self.callback)#/camera/rgb/image_color
        self.counter = 0
    def callback(self, msg):
        filters = [Cartoonizer(), PencilSketch((570, 380)), WarmingFilter(), Vignette()]
        if ("filter" in msg.data):
            
            filename='/home/jan/photobot/camera_image.jpg'
            img = cv2.imread(filename,1)
            imgs = []
            img = cv2.resize(img,  (img.shape[0]/3, img.shape[1]/3))
            print img.shape
            for filter in filters:
              filtered = filter.render(img)
              retval, buffer = cv2.imencode('.jpg', filtered)
              jpg_as_text = base64.b64encode(buffer) 
              imgs.append(jpg_as_text)
            import json
            print "Sending data"
            self.imagePub.publish(str(json.dumps({"imgs": imgs})))

def main():
    rospy.init_node('ui')
    UiPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
