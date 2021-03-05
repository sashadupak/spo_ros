#!/usr/bin/env python
# coding: utf-8

import cv2
import cv2.xfeatures2d as cv # only for a new version
import numpy as np
from matplotlib import pyplot as plt
import glob
from math import sqrt
import json
import csv
import os
import time
import traceback
import sys


class SignDetector:

    def __init__(self, ref_folder):
        self.ref_folder = ref_folder
        self.sift = cv.SIFT_create()

        # load reference signs
        self.ref = {}
        self.kp2 = {}
        self.des2 = {}
        self.ref_names = glob.glob(self.ref_folder + '*.jpg')
        for ref_name in self.ref_names:
            self.ref[ref_name] = cv2.imread(ref_name, 0)
            self.kp2[ref_name], self.des2[ref_name] = self.sift.detectAndCompute(self.ref[ref_name], None)

        self.sign_h, self.sign_w = self.ref[self.ref_names[0]].shape

    def match_features(self, des1, des2, kp1, kp2):
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2, k=2)
        
        good1 = []
        try:
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    good1.append(m)
        except ValueError:
            print("not enough matches")
            return None

        matches = bf.knnMatch(des2,des1, k=2)

        good2 = []
        try:
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    good2.append(m)
        except ValueError:
            print("not enough matches")
            return None

        good=[]
        for i in good1:
            ref1_id1=i.queryIdx 
            img2_id1=i.trainIdx
            (x1,y1)=kp1[ref1_id1].pt
            (x2,y2)=kp2[img2_id1].pt

            for j in good2:
                ref1_id2=j.queryIdx
                img2_id2=j.trainIdx

                (a1,b1)=kp2[ref1_id2].pt
                (a2,b2)=kp1[img2_id2].pt

                if (a1 == x2 and b1 == y2) and (a2 == x1 and b2 == y1):
                    good.append(i)
        return good

    def transform(self, img, src_pts, dst_pts):

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w,_ = img.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)

        cropped = cv2.warpPerspective(img, M, (w, h))
        corners = [np.int32(dst)]
        return cropped, corners

    def detect_sign(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        #blur = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(gray, 25, 100)
        cv2.imshow('edges', edges)
        contours, h = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours.sort(key=lambda x: cv2.arcLength(x, True), reverse=True)
        #cv2.drawContours(image, contours, -1, (255, 0, 0), 3, cv2.LINE_AA, h, 1)
        sign_found = False
        sign = None
        for i, cnt in enumerate(contours):
            if cv2.arcLength(cnt, True) < 100:
                break
            #cv2.drawContours(image, [cnt], -1, (0, 0, 255), 3)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            a = sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
            b = sqrt((box[2][0] - box[1][0])**2 + (box[2][1] - box[1][1])**2)
            if a != 0:
                aspect_ratio = float(b)/a
                if (0.8 < aspect_ratio < 1.2): # check if rect is a square
                    if abs(a*b - cv2.contourArea(cnt)) < 0.5*a*b:
                        #square = np.array([[0, self.sign_h], [0, 0], [self.sign_w, 0], [self.sign_w, self.sign_h]])
                        square = np.array([[0, 480], [0, 0], [480, 0], [480, 480]])
                        sign, _ = self.transform(image, box, square)
                        sign = sign[0:480, 0:480]
                        sign_found = True
                        image = cv2.drawContours(image,[box],0,(0,255,0),2)
                        cv2.imshow('image', image)
                        break

        return sign_found, sign

    def classify_sign(self, sign):
        #sign = cv2.resize(sign, (self.sign_w, self.sign_h))
        kp1, des1 = self.sift.detectAndCompute(sign, None)
        
        match_count = 0
        matches_list = []
        for ref_name in self.ref_names:
            matches = self.match_features(des1, self.des2[ref_name], kp1, self.kp2[ref_name])
            if matches is None:
                continue
            matches_list.append(matches)
            if len(matches) > match_count:
                match_count = len(matches)
                ind = self.ref_names.index(ref_name)

        if len(matches_list) > 0:
            matches = matches_list[ind]
            ref_name = self.ref_names[ind]
            sign_type = ((ref_name.split("\\")[-1]).split("/")[-1]).split(".")[0]
        else:
            matches = []
            sign_type = "UNDEFINED"
        print("Sign type: " + str(sign_type))
        return sign_type, len(matches)



if __name__ == "__main__":
    output_file_name = 'results.csv'
    ref_folder = 'signs/'
    data_folder = 'photos/'
    min_match = 10 # min number of matches
    detector = None
    fh = None

    try:
        start_time = time.time()

        os.chdir("../")
        detector = SignDetector(ref_folder)
        fh = open(output_file_name, 'w')
        writer = csv.writer(fh)
        writer.writerow(["File name", "number of matches", "Sign type"])

        file_names = glob.glob(data_folder + '*.jpg')
        sign_count = 0

        for file_name in file_names:
            print("ok")
            sign_count += 1
            img = cv2.imread(file_name)

            sign_found, sign = detector.detect_sign(img)
            if sign_found:
                cv2.imshow('sign', sign)
                sign_type, num_matches = detector.classify_sign(sign)
                if num_matches > min_match:
                    writer.writerow([file_name.split("\\")[-1], str(num_matches), str(sign_type)])

            k = cv2.waitKey(0) & 0xff
            if k == ord('q') or (k == 27):
                exit()

    except Exception:
        traceback.print_exc(file=sys.stdout)
    else:
        elapsed_time = time.time() - start_time
        print("Elapsed time: %.2f s" % elapsed_time)
        if sign_count > 0:
            print("Estimated: %.2f seconds per sample" % float(elapsed_time / sign_count))
    finally:
        cv2.destroyAllWindows()
        if fh is not None:
            fh.close()
        if detector is not None:
            del detector
