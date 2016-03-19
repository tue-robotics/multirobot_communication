#!/usr/bin/env python

import sys, os

import time
import socket

from ed.srv import Query

# XML RPC SERVER
import thread
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer as Server
import rospy


def error(error):
    print "ERROR: %s" % error
    sys.exit()

class SyncServer():

    def __init__(self, ip, port):
        self._server = Server((ip, port), allow_none=True)
        self._server.register_function(self.get, 'get')

        self._ros_service_proxy = rospy.ServiceProxy('ed/query', Query)

        self._sp  = ServerProxy("http://%s:%d"%(ip,port))
        self._stop = False

    # RPC METHOD
    def get(self, ids, properties, since_revision):

        try:
            resp  = self._ros_service_proxy(ids=ids, properties=properties, since_revision=since_revision)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            resp = None

        result = None

        if resp:
            result = {}
            result["human_readable"] = resp.human_readable
            result["new_revision"] = resp.new_revision

        return result

    def serve(self):
        try:
            self._server.serve_forever()
        except KeyboardInterrupt:
            pass

    def serve(self):
        thread.start_new_thread(self._serve, ())
        rospy.spin()
        self._stop = True
        self._sp.ping()

    def _serve(self):
        while not self._stop:
            self._server.handle_request()
        rospy.loginfo("Shutting down SyncServer")

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('sync_server')
        if rospy.has_param('~ip') and rospy.has_param('~port'):
            ip = rospy.get_param('~ip')
            port = rospy.get_param('~port')
            server = SyncServer(ip, port)
            print "Sync server active at %s:%d - returns the world model update with use of RPC"%(ip,port)
            server.serve()

        else:
            rospy.logerr("Sync server : no server ip or port set; please specify the local 'ip' and 'port' parameter")
    except rospy.ROSInterruptException:
        pass
