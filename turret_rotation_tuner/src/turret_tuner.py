#!/usr/bin/env python
from __future__ import print_function

import cv2
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

axcolor = 'lightgoldenrodyellow'
ax1 = plt.axes([0.25, 0.1, 0.65, 0.03])
ax2 = plt.axes([0.25, 0.15, 0.65, 0.03])
ax3 = plt.axes([0.25, 0.2, 0.65, 0.03])
ax4 = plt.axes([0.25, 0.25, 0.65, 0.03])

kp_slider = Slider(ax1, 'kp', 0.0, 1.0)
ki_slider = Slider(ax2, 'ki', 0.0, 1.0)
kd_slider = Slider(ax3, 'kd', 0.0, 1.0)
kf_slider = Slider(ax4, 'kf', 0.0, 1.0)

class VisionTargetingTuner:

  def __init__(self):
      self.kp = Float32()
      self.ki = Float32()
      self.kd = Float32()
      self.kf = Float32()

      self.kp_pub = rospy.Publisher("turret/kp", Float32, queue_size=50)
      self.ki_pub = rospy.Publisher("turret/ki", Float32, queue_size=50)
      self.kd_pub = rospy.Publisher("turret/kd", Float32, queue_size=50)
      self.kf_pub = rospy.Publisher("turret/kf", Float32, queue_size=50)

  def update(self,val):
    self.kp.data = float(kp_slider.val)
    self.ki.data = float(ki_slider.val)
    self.kd.data = float(kd_slider.val)
    self.kf.data = float(kf_slider.val)

    self.kp_pub.publish(self.kp)
    self.ki_pub.publish(self.ki)
    self.kd_pub.publish(self.kd)
    self.kf_pub.publish(self.kf)



def main(args):
  dvt = VisionTargetingTuner()
  rospy.init_node('turret_tuner', anonymous=True)

  kp_slider.on_changed(dvt.update)
  ki_slider.on_changed(dvt.update)
  kd_slider.on_changed(dvt.update)
  kf_slider.on_changed(dvt.update)

  plt.show()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)