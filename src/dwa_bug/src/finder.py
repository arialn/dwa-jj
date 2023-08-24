import cv2
import numpy as np
from grid import gridmaker
import matplotlib.pyplot as plt
from pathsolver import solver
from tqdm import tqdm

obj=gridmaker(200)

solve=solver()


def line(route):
    xc=[]
    yc=[]
    for i in (range(0,len(route))):
        x=route[i][0]
        y=route[i][1]
        xc.append(x)
        yc.append(y)
    return xc,yc


# cap=cv2.VideoCapture(1)
# while 1:
#     ret,frame=cap.read()
#     frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#     th,frame=cv2.threshold(frame,100,255,cv2.THRESH_BINARY)
#     frame=cv2.bitwise_not(frame)
#     frame=cv2.dilate(frame,None,iterations=5)
#     cv2.imshow("frame",frame)
#     if(cv2.waitKey(1) & 0XFF==ord('q')):
#         cv2.imwrite("frame.jpg",frame)
#         break
# cap.release()
# cv2.destroyAllWindows()
def get_png():
    img=cv2.imread("00.png",0)
    x,y = img.shape[0:2]
    img_one_of_four = cv2.resize(img,(int(y/10),int(x/10)))
    #it's a tuple
    return img_one_of_four

def main():
    global obj, solve
    fig,ax=plt.subplots()
    #grid=obj.returnGrid()
    img = get_png()
    block_img = ~img
    grid = block_img
    #print(grid[2200,2000])  #1
    img_h, img_w = grid.shape


    ax.imshow(grid,cmap=plt.cm.Spectral)
    plt.show()
    dx = img_w*0.5
    dy = img_h*0.5
    ds = 0.1
    st = (-20,0)
    ed = (20,0)
    start=(int((st[1]/ds)+dy),int((st[0]/ds)+dx))
    #start =(500,1500)
    print("start point:{}".format(start))
    #end=(124,340)#end point:(2480, 4405)  No path
    end = (int((ed[1]/ds)+dy),int((ed[0]/ds)+dx))
    print("end point:{}".format(end))
    
    route=solve.astar(start,end,grid)
    if(route==False):
        print("No path")
        return 0
    route+=[start]
    route=route[::-1]
    print(route)

    print(dx,dy)
    
    path=[]
    for i in route:
        px = (i[0]-dy)*ds
        py = (i[1]-dx)*ds
        path.append((py,px))
    print('=================')
    print(path)
    xc,yc=line(route)
    fig,ax=plt.subplots()
    ax.imshow(grid,cmap=plt.cm.Spectral)
    ax.plot(yc,xc,color="black")
    ax.scatter(start[1],start[0])
    ax.scatter(end[1],end[0])
    plt.show()
if(__name__=="__main__"):
    main()
