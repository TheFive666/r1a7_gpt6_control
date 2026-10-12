"""Lightweight yellow/orange cake candidate detector, no depth inference."""
import cv2
import numpy as np


def detect_cake(frame, tabletop_only=False):
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([4,85,70]),np.array([42,255,255]))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    h,w=frame.shape[:2];candidates=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if area<max(40,h*w*.00015) or area>h*w*.3:continue
        x,y,bw,bh=cv2.boundingRect(contour)
        if tabletop_only and y+bh/2 < h*.55:continue
        if not .45<bw/max(bh,1)<2.5:continue
        fill=area/(bw*bh)
        if fill<.55:continue
        if x<=1 or y<=1 or x+bw>=w-1 or y+bh>=h-1:continue
        candidates.append(dict(bbox_xyxy=[x,y,x+bw,y+bh],center_px=[x+bw/2,y+bh/2],
                               area_px=area,fill_ratio=fill,method='HSV yellow-orange connected contour'))
    candidates.sort(key=lambda d:d['area_px'],reverse=True)
    if not candidates:return None
    if len(candidates)>1 and candidates[1]['area_px']>.4*candidates[0]['area_px']:return None
    return candidates[0]


def annotate(frame,result):
    image=frame.copy()
    if result:
        x,y,x2,y2=result['bbox_xyxy']
        cv2.rectangle(image,(x,y),(x2,y2),(0,255,0),2)
        cv2.putText(image,'yellow cake candidate',(x,max(20,y-8)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
    return image
