#!/usr/bin/env python

import sys, os

import time
import socket

# XML RPC SERVER
import thread
import xmlrpclib
from xmlrpclib import ServerProxy

from SimpleXMLRPCServer import SimpleXMLRPCServer as Server
import rospy

from std_msgs.msg import String

def error(error):
    print "ERROR: %s"%error
    sys.exit()

class TriggerServer():

    def __init__(self, ip, port):
        self._server = Server((ip, port), allow_none=True)
        self._server.register_function(self.get, 'get')
        self._server.register_function(lambda: 'OK', 'ping')
        self._sp  = ServerProxy("http://%s:%d"%(ip,port))

        self._ros_publisher = rospy.Publisher('trigger', String, queue_size=10)

        self._stop = False

    # RPC METHOD
    def get(self, data):
        self._ros_publisher.publish(String(data=data))

    def serve(self):
        thread.start_new_thread(self._serve, ())
        rospy.spin()
        self._stop = True
        self._sp.ping()

    def _serve(self):
        while not self._stop:
            self._server.handle_request()
        rospy.loginfo("Shutting down TriggerServer")

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('trigger_server')
        if rospy.has_param('~ip'):
            ip = rospy.get_param('~ip')
            port = rospy.get_param('~port')
            server = TriggerServer(ip, port)
            print "Trigger server active at %s:%d"%(ip,port)
            server.serve()

        else:
            rospy.logerr("Trigger server : no server ip set; please specify the local 'ip' parameter")
    except rospy.ROSInterruptException:
        pass
