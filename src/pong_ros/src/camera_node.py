#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Replace with your preferred face detection and facial landmark estimation library
# Here's an example using Dlib
import dlib

predictor_path = "/home/mujtahid/rospong_ws/src/pong_ros/src/shape_predictor_68_face_landmarks.dat"  # Path to facial landmark predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

def detect_face_and_landmarks(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)

    if len(faces) > 0:
        face = faces[0]
        landmarks = predictor(gray, face)
        return landmarks
    else:
        return None

#def estimate_head_tilt(landmarks, frame):
    # Calculate head tilt based on facial landmarks (e.g., nose and eyes)
    # Customize this logic based on your chosen library and desired tilt estimation
#    nose_tip = landmarks.part(33)
#    left_eye = landmarks.part(36)
#    right_eye = landmarks.part(45)

#    center_x = (left_eye.x + right_eye.x) / 2
#    tilt = (nose_tip.x - center_x) / frame.shape[1]  # Normalize to 0-1 range
    

#    return tilt


def estimate_head_tilt(landmarks, frame):
    # Calculate head tilt based on facial landmarks and frame position
    # Normalize to 0-1 based on face position in frame

    # Adjust landmark access based on your chosen library
    nose_tip = landmarks.part(33)
    leftmost_point = min(landmark.x for landmark in landmarks.parts())
    rightmost_point = max(landmark.x for landmark in landmarks.parts())

    # Calculate face center and width
    face_center = (leftmost_point + rightmost_point) / 2
    face_width = rightmost_point - leftmost_point

    # Calculate normalized offset from center
    offset = (nose_tip.x - face_center) / (face_width/3)

    # Normalize to 0-1 range, considering left extreme as 0 and right extreme as 1
    tilt = (offset + 1) / 2  # Map -1 to 1 range to 0 to 1

    return tilt




def main():
    rospy.init_node('camera_node')
    pub = rospy.Publisher('/paddle_position', Float32, queue_size=10)
    bridge = CvBridge()


    cap = cv2.VideoCapture(0)  # Replace with your camera device or video file path

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        landmarks = detect_face_and_landmarks(frame)

        if landmarks is not None:
            tilt = estimate_head_tilt(landmarks, frame)
            pub.publish(tilt)

        # Optional: Display video with face detection and landmarks (for debugging)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
