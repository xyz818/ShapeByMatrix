# -*- coding: utf-8 -*
import thread

import cv2
import serial
import binascii
import time

import crcmod.predefined
# 打开串口
ser = serial.Serial("/dev/ttyAMA0", 57600)
isRunnig = False
# com recv handler
def recvser():
    count = ser.inWaiting()# 获得接收缓冲区字符
    if count != 0:
        recv = ser.read(count)
        hlen = len(recv)
        recvv = ''
        for i in xrange(hlen):
            hvol = ord(recv[i])
            hhex = '%02x' % hvol
            recvv += hhex + ''
        # print recvv
        # string handling
        handleMsg(recvv)
        # clear buffer
        ser.flushInput()
        # 必要的软件延时
        time.sleep(0.1)
    else:
        time.sleep(0.1)

def computeCrc8(hexData):
    crc8 = crcmod.predefined.Crc('crc-8-maxim')
    hexData = binascii.unhexlify(hexData)
    crc8.update(hexData)
    result = hex(crc8.crcValue)
    hhex = '%02x' % int(result, 16)  # compute crc8 check
    return hhex

def handleMsg(recv):
    size = len(recv)
    global  isRunnig
    # fe 1f 82 00 00 00 02 05 00 00 00 00 04 7c 03 00 00 04 2c ff
    if size >= 28:
        print size
        print recv
        print recv[0:2],recv[size-2:size]
        if recv[0:2] == 'fe' and recv[size - 2:size] == 'ff':
            hexData = recv[2:size - 4]
            hhex = computeCrc8(hexData)
            if hhex == recv[size - 4:size -2]:
                if recv[24:28] == '047c': # touch key
                    if recv[34:36] == '00':
                        isRunnig = True
                    else :
                        isRunnig = False



def writeser(datt):
    hexstr = binascii.a2b_hex(datt)
    ser.write(hexstr)


def controlLed():
    buf = '1f0a62000000000000020806290F007844444438000000000000000000'
    crc = computeCrc8(buf)
    buf = 'fe' + buf+crc+'ff'
    print buf
    return buf


# camera init  read
def Camera_Init():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
    global  isRunnig

    # path = r'/root/opencv/opencv-3.2.0/data/haarcascades/'
    # detector = cv2.CascadeClassifier(path + 'haarcascade_frontalface_default.xml')
    # eye_detector = cv2.CascadeClassifier(path + 'haarcascade_eye.xml')
    while cap.isOpened():
        ret,frame = cap.read()

        # cv2.imshow('capture',frame)
        if isRunnig:
            # print 'start check'
            # frame = cv2.cvtColor(output.array, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            ret, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)  # binary
            cv2.bitwise_not(thresh, thresh)
            image = cv2.Canny(thresh, 50, 150)
            # print 'shape'+str(dst.shape)
            # cv2.imshow('none', image)
            circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=30, minRadius=40,maxRadius=100)
            if circles is not None:
                writeser(controlLed())
                for circle in circles[0]:
                    x = int(circle[0])
                    y = int(circle[1])
                    r = int(circle[2])
                    frame = cv2.circle(frame, (x, y), r, (0, 0, 255), -1)

        cv2.imshow('gra', frame)
        # else:
        #     print 'stop check'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def Com_Recv():
    while True:
        recvser()




if __name__ == '__main__':
    try:
        # start com threadwriteser( controlLed())
        thread.start_new_thread(Com_Recv,())
        # controlMotor('123')
        Camera_Init()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()