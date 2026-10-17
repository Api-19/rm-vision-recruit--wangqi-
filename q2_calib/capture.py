"""生成棋盘格或手动启动相机采集；不会在导入脚本时访问相机。"""
import argparse
from pathlib import Path
import time
import cv2
import numpy as np


def board(output,cols,rows,pixels):
    if min(cols,rows,pixels)<2: raise ValueError('参数过小')
    margin=pixels
    img=np.full(((rows+1)*pixels+2*margin,(cols+1)*pixels+2*margin),255,np.uint8)
    for r in range(rows+1):
        for c in range(cols+1):
            if (r+c)%2==0:
                img[margin+r*pixels:margin+(r+1)*pixels,margin+c*pixels:margin+(c+1)*pixels]=0
    Path(output).parent.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(output,img): raise OSError(output)
    print('这是内角点',cols,'×',rows,'的棋盘。打印/显示后必须实际测量单格边长，像素不等于毫米。')


def capture(output,index,width,height,cols,rows):
    out=Path(output); out.mkdir(parents=True,exist_ok=True)
    cap=cv2.VideoCapture(index)
    if not cap.isOpened(): raise RuntimeError('相机打不开；可直接用手机照片放入输入目录')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,width); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
    print('S保存原图；Q退出。不要改变镜头/变焦。先检查样张，采集15–25张不同姿态。')
    try:
        while True:
            ok,frame=cap.read()
            if not ok: raise RuntimeError('读取相机帧失败')
            view=frame.copy()
            found,corners=cv2.findChessboardCorners(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),(cols,rows))
            if found: cv2.drawChessboardCorners(view,(cols,rows),corners,found)
            cv2.putText(view,f'{frame.shape[1]}x{frame.shape[0]} S:save Q:quit',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,200,0),2)
            cv2.imshow('capture',view); key=cv2.waitKey(30)&255
            if key==ord('q'): break
            if key==ord('s'):
                target=out/f'capture_{time.time_ns()}.png'
                if not cv2.imwrite(str(target),frame): raise OSError(target)
                print(target,'corners_found=',found)
    finally:
        cap.release(); cv2.destroyAllWindows()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); s=p.add_subparsers(dest='command',required=True)
    b=s.add_parser('board'); b.add_argument('--output',default='q2_calib/board.png'); b.add_argument('--cols',type=int,default=9); b.add_argument('--rows',type=int,default=6); b.add_argument('--pixels',type=int,default=100)
    c=s.add_parser('camera'); c.add_argument('--output',default='q2_calib/input/chessboard'); c.add_argument('--index',type=int,default=0); c.add_argument('--width',type=int,default=1920); c.add_argument('--height',type=int,default=1080); c.add_argument('--cols',type=int,default=9); c.add_argument('--rows',type=int,default=6)
    a=p.parse_args()
    if a.command=='board': board(a.output,a.cols,a.rows,a.pixels)
    else: capture(a.output,a.index,a.width,a.height,a.cols,a.rows)
