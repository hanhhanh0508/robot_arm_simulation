#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple service để attach/detach objects trong Gazebo
"""
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def handle_attach(req):
    """Handle attach request"""
    rospy.loginfo("Received attach request:")
    rospy.loginfo("  Model 1: %s / Link 1: %s" % (req.model_name_1, req.link_name_1))
    rospy.loginfo("  Model 2: %s / Link 2: %s" % (req.model_name_2, req.link_name_2))
    
    # Gọi Gazebo service thực tế
    try:
        rospy.wait_for_service('/link_attacher_node/attach', timeout=2.0)
        attach_service = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        response = attach_service(req)
        return response
    except Exception as e:
        rospy.logerr("Failed to call attach service: %s" % str(e))
        return AttachResponse(ok=False)

def handle_detach(req):
    """Handle detach request"""
    rospy.loginfo("Received detach request:")
    rospy.loginfo("  Model 1: %s / Link 1: %s" % (req.model_name_1, req.link_name_1))
    rospy.loginfo("  Model 2: %s / Link 2: %s" % (req.model_name_2, req.link_name_2))
    
    # Gọi Gazebo service thực tế
    try:
        rospy.wait_for_service('/link_attacher_node/detach', timeout=2.0)
        detach_service = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        response = detach_service(req)
        return response
    except Exception as e:
        rospy.logerr("Failed to call detach service: %s" % str(e))
        return AttachResponse(ok=False)

def main():
    rospy.init_node('simple_attach_service')
    
    rospy.loginfo("Starting simple attach/detach service...")
    
    # Tạo services
    attach_srv = rospy.Service('/attach', Attach, handle_attach)
    detach_srv = rospy.Service('/detach', Attach, handle_detach)
    
    rospy.loginfo("Services ready:")
    rospy.loginfo("  /attach")
    rospy.loginfo("  /detach")
    
    rospy.spin()

if __name__ == '__main__':
    main()