# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os
import os.path
from matplotlib import pyplot as plt

def get_histGBR(path):
    img = cv2.imread(path)
    
    pixal = img.shape[0] * img.shape[1]
    total = np.array([0])
    for i in range(3):
        histSingle = cv2.calcHist([img], [i], None, [256], [0, 256])
        total = np.vstack((total, histSingle))
        
    return (total, pixal)

def hist_similar(lhist, rhist, lpixal, rpixal):
    rscale = rpixal/lpixal
    rhist = rhist/rscale
    assert len(lhist) == len(rhist)
    likely = sum(1 - (0 if l == r else float(abs(l-r))/max(l,r)) for l,r in zip(lhist, rhist)) / len(lhist)
    if likely ==1.0:
        return [1.0]
    return likely

if __name__ == '__main__':
    targetHist, targetPixal = get_histGBR('target.jpg')
    rootdir = "/Users/FarisYang/Desktop/9431/match_color_histogram"
    for parent, dirnames, filenames in os.walk(rootdir):
        #print(filenames)
        for filename in filenames:
            if filename[-3:] == 'jpg':
                testHist, testPixal = get_histGBR(filename)
                print(filename, hist_similar(targetHist, testHist, targetPixal, testPixal))