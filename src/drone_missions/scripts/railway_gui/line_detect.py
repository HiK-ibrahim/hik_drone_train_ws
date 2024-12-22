#!/usr/bin/env python

import cv2
import numpy as np

class LineDetector:
    def __init__(self, lower_red1, upper_red1, lower_red2, upper_red2):
        self.lower_red1 = lower_red1
        self.upper_red1 = upper_red1
        self.lower_red2 = lower_red2
        self.upper_red2 = upper_red2

    def detect_red_lines(self, frame):
        """
        Çizgiyi tespit eder ve bir maske ile uzunlamasına hat bilgisi döndürür.
        """
        # Görüntüyü HSV renk uzayına dönüştür
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Kırmızı rengi maskele
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Gürültü temizleme
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        
        # Çizgiyi tespit et
        edges = cv2.Canny(mask, 50, 150)  # Kenar algılama

        # Hough Transform ile çizgi tespiti
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        
        # Orijinal görüntü üzerine çizgileri çiz
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

        return mask, frame