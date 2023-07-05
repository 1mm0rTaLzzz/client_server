import serial
import time
import threading as thr
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from homogen import matrixFromVectors
import cv2
from camera_props import *
from cv2 import aruco


class Robot:
    def __init__(self, port="/dev/ttyUSB0"):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.ser = serial.Serial(port, 115200, timeout=.1)
        time.sleep(2)
        self.thread = thr.Thread(target=self.mainLoop)
        self.angPID = np.array([0.0, 0, 0])
        self.angPIDk = np.array([230.0, 0, -18000])
        self.targetAng = 0
        self.currAng = 0
        self.movementAllowed = False
        self.lastTime = time.time()
        self.thread.start()
        self.targetSpeed = 0
        self.pos = np.array([0, 0])
        self.robotID = 1


    def move(self, fov, side):
        assert fov == int(fov)
        assert side == int(side)
        fov = int(fov)
        side = int(side)
        self.ser.write(b'l' + (fov + side).to_bytes(2, 'big', signed=True))
        self.ser.write(b'r' + (fov - side).to_bytes(2, 'big', signed=True))

    def lift(self, val):
        assert val == int(val)
        assert 90 <= val <= 210
        self.ser.write(b'u' + int(val).to_bytes(2, 'big', signed=True))

    def grab(self, val):
        assert val == int(val)
        assert 110 <= val <= 240
        self.ser.write(b'g' + int(val).to_bytes(2, 'big', signed=True))

    def holdAng(self, ang):
        self.targetAng = ang

    def setSpeed(self, speed):
        self.targetSpeed = speed

    def mainLoop(self):
        counter = 0
        while True:
            self.update()

    def update(self):
        counter = 0
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        '''image = cv2.putText(frame_markers, str(time.time() - t), (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255, 255, 255), 2, cv2.LINE_AA)'''

        matrices = []

        if corners:
            for c in corners:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, 0.072, camera_matrix, distortion_coefficient)
                image = cv2.drawFrameAxes(frame_markers, camera_matrix, distortion_coefficient, rvec, tvec, 0.08)
                matrices.append(matrixFromVectors(tvec[0, 0]*1000, rvec[0, 0]))
        cv2.imshow('frame', frame_markers)

        if ids is not None:
            ids = ids.T[0]
            if 0 in ids and self.robotID in ids and len(matrices) > 1:
                tr = np.linalg.inv(matrices[np.where(ids == 0)[0][0]]) @ matrices[np.where(ids == self.robotID)[0][0]]
                tr = tr/tr[3, 3]
                # tr = tr/tr[3,3]
                rob_rot = Rot.from_matrix(tr[:3, :3]).as_rotvec()
                self.pos = tr[:2, 3]
                # print(self.pos)
                self.movementAllowed = True
                # print(Rot.from_matrix(tr[:3, :3]).as_rotvec())
                self.currAng = rob_rot[2]
            else:
                self.movementAllowed = False
        if self.movementAllowed:
            #print(self.currAng)
            if abs(self.currAng - self.targetAng) > 0.05:
                self.angPID[0] = self.currAng - self.targetAng
                if abs(self.currAng - self.targetAng) < 0.3:
                    self.angPID[1] += self.currAng - self.targetAng
                    self.angPID[1] = min(max(-80, self.angPID[1]), 80)
                else:
                    self.angPID[1] = 0
                self.angPID[2] = (self.currAng - self.targetAng)*(time.time()-self.lastTime)
                self.lastTime = time.time()
                #print(self.angPID*self.angPIDk, "\t", self.currAng - self.targetAng, '\t',
                # np.sum(self.angPID*self.angPIDk))
                self.move(self.targetSpeed, max(-70, min(70, -int(np.sum(self.angPID*self.angPIDk)))))
                #print(self.targetSpeed, max(-70, min(70, -int(np.sum(self.angPID*self.angPIDk)))))

            else:
                self.move(self.targetSpeed, 0)
        else:
            self.move(0, 0)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            pass
        if key & 0xFF == ord('s'):
            cv2.imwrite(f"latency_test_{counter}.jpg", frame_markers)
