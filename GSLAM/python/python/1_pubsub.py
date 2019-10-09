from gslam import *

def showMsg(msg):
  print("received",msg)

sub=messenger.subscribe("topic",0,showMsg)

messenger.publish("topic",Point3d(1,2,3))

pub=messenger.advertise("topic",0)

pub.publish(Point3d(2,3,4))

