import tf
import rospy
from testfunc import tfPrint

rospy.init_node('test', anonymous=False)
listener = tf.TransformListener()
tfPrint(listener)
