#!/usr/bin/env python

import roslib
import rospy

from xmlrpclib import ServerProxy
from std_msgs.msg import String

def cyan(text):
    return text

class TriggerClient():

    def __init__(self, server_ip, port, remote_robot_name):
        self.sub = rospy.Subscriber("/%s/trigger" % remote_robot_name, String, self.callback)
        self.sp  = ServerProxy("http://%s:%d"%(server_ip,port))

    def callback(self, msg):
        rospy.loginfo("Trigger: [%s]"%cyan(msg.data))

        # RPC request to server
        self.sp.get(msg.data)

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('trigger_client')
        if rospy.has_param('~ip'):
            ip = rospy.get_param('~ip')
            port = rospy.get_param('~port')
            remote_robot_name = rospy.get_param('~remote_robot_name')
            client = TriggerClient(ip, port, remote_robot_name)
            rospy.loginfo("Trigger client that pushes changes initialized [connecting to server on ip %s]"%ip)
            rospy.spin()
        else:
            rospy.logerr("Trigger client: no server ip set; please specify the local 'ip' parameter")
    except rospy.ROSInterruptException:
        pass
