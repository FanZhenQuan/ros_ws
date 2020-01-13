#!/usr/bin/env python

import rospy
import cv2


if __name__ == "__main__":
    rospy.init_node('subdiv')

    img_path = '/home/davide/ros_ws/src/learning/learning_toponav/maps/office/metric.pgm'
    img = cv2.imread(img_path)
    
    rect = (0, 0, img.shape[1], img.shape[0])
    subdiv = cv2.Subdiv2D(rect)
    
    while not rospy.has_param('/interest_points'):
        rospy.sleep(0.1)
    points = rospy.get_param('/interest_points')
    # for i in points:
    #     p = (
    #         i['pose']['position']['x'],
    #         i['pose']['position']['y']
    #     )
    #     subdiv.insert(p)

    for lm in img.face_landmarks:
        if is_in_frame(img.shape[0], img.shape[1], lm):
            subdiv.insert(lm)
    print("triangles: {}".format(len(subdiv.getTriangleList())))
    for t in subdiv.getTriangleList():
        t = np.reshape(t, (3, 2)).astype(np.int32)
        pt1 = scale_point(tuple(t[0]), 1)
        pt2 = scale_point(tuple(t[1]), 1)
        pt3 = scale_point(tuple(t[2]), 1)
        cv2.line(img, pt1, pt2, (255, 255, 255), 1, 8, 0)
        cv2.line(img, pt2, pt3, (255, 255, 255), 1, 8, 0)
        cv2.line(img, pt3, pt1, (255, 255, 255), 1, 8, 0)
        
    plt.imshow(img)
    
    rospy.spin()