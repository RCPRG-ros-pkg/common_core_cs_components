#!/usr/bin/env python

# Copyright (c) 2015, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from ros_utils import marker_publisher
import xml.dom.minidom as minidom
import PyKDL
import copy

class CollisionListener:

    def callback(self, data):
        for v in data.status[1].values:
            if v.key == 'ColDetRep':
                self.xml = copy.copy(v.value)
#                print v.value

#        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def __init__(self):
        rospy.init_node('show_collisions', anonymous=False)
#        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.Subscriber("/velma_core_cs/diag", DiagnosticArray, self.callback)

        self.pub = marker_publisher.MarkerPublisher('/velma_markers')
        self.links = {}
        self.xml = None

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.xml:
                xml = minidom.parseString(self.xml)
                cd = xml.getElementsByTagName("cd")
                if len(cd) != 1:
                    raise Exception('subsystem_definition', 'subsystem_definition is missing')
                link = cd[0].getElementsByTagName("l")
                for l in link:
                    name = l.getAttribute("name")
                    idx = l.getAttribute("idx")
                    geom = l.getElementsByTagName("g")
                    geom_list = []
                    for g in geom:
                        g_x = float(g.getAttribute("x"))
                        g_y = float(g.getAttribute("y"))
                        g_z = float(g.getAttribute("z"))
                        g_qx = float(g.getAttribute("qx"))
                        g_qy = float(g.getAttribute("qy"))
                        g_qz = float(g.getAttribute("qz"))
                        g_qw = float(g.getAttribute("qw"))
                        fr = PyKDL.Frame( PyKDL.Rotation.Quaternion(g_qx, g_qy, g_qz, g_qw), PyKDL.Vector(g_x, g_y, g_z) )
                        g_type = g.getAttribute("type")
                        if g_type == "SPHERE":
                            g_r = float(g.getAttribute("r"))
                            geom_list.append( (fr, g_type, g_r) )
                        elif g_type == "CAPSULE":
                            g_r = float(g.getAttribute("r"))
                            g_l = float(g.getAttribute("l"))
                            geom_list.append( (fr, g_type,  g_r, g_l) )
                    self.links[name] = (idx, geom_list)
                col = cd[0].getElementsByTagName("c")
                col_list = []
                for c in col:
                    v1 = PyKDL.Vector( float(c.getAttribute("p1x")), float(c.getAttribute("p1y")), float(c.getAttribute("p1z")) )
                    v2 = PyKDL.Vector( float(c.getAttribute("p2x")), float(c.getAttribute("p2y")), float(c.getAttribute("p2z")) )
                    col_list.append( (v1, v2) )
                m_id = 0
                for name in self.links:
                    link = self.links[name]
                    idx = link[0]
                    geom_list = link[1]
                    for g in geom_list:
                        fr = g[0]
                        g_type = g[1]
                        if g_type == "SPHERE":
                            g_r = g[2]
                            m_id = self.pub.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=1, b=0, a=0.5, namespace='default', frame_id=name, m_type=Marker.SPHERE, scale=Vector3(2*g_r, 2*g_r, 2*g_r), T=fr)
                        elif g_type == "CAPSULE":
                            g_r = g[2]
                            g_l = g[3]
                            m_id = self.pub.publishSinglePointMarker(PyKDL.Vector(0,0,-g_l/2), m_id, r=0, g=1, b=0, a=0.5, namespace='default', frame_id=name, m_type=Marker.SPHERE, scale=Vector3(2*g_r, 2*g_r, 2*g_r), T=fr)
                            m_id = self.pub.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=1, b=0, a=0.5, namespace='default', frame_id=name, m_type=Marker.CYLINDER, scale=Vector3(2*g_r, 2*g_r, g_l), T=fr)
                            m_id = self.pub.publishSinglePointMarker(PyKDL.Vector(0,0,g_l/2), m_id, r=0, g=1, b=0, a=0.5, namespace='default', frame_id=name, m_type=Marker.SPHERE, scale=Vector3(2*g_r, 2*g_r, 2*g_r), T=fr)
                for c in col_list:
                    m_id = self.pub.publishVectorMarker(c[0], c[1], m_id, 1, 0, 0, frame='torso_base', namespace='default', scale=0.005)
                self.pub.eraseMarkers(m_id, 200, frame_id='torso_base', namespace='default')
            rate.sleep()
    
if __name__ == '__main__':
    col = CollisionListener()
    col.spin()


