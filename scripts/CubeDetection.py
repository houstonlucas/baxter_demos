import numpy as np
import cv2
import math
import copy


#TODO DELETE WHEN/IF YOU PULL OUT THE CLUSTER GRAPH
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class Sticker:
    def __init__(self, color, loc, face):
        self.color = color
        self.loc = loc
        self.face = face
    def setColorLabel(self, label):
        self.label = label

def identifyColors(found):
    found = found.reshape(9,-1,3)
    found = np.uint8(found)
    found = cv2.cvtColor(found, cv2.COLOR_BGR2HSV)
    found = found.reshape(-1,3)
    found = np.float32(found)
    print found.shape
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 70, 0.1)
    ret,label,center=cv2.kmeans(found,6,criteria,20,cv2.KMEANS_RANDOM_CENTERS)
    
    ############## CLUSTER PLOT #################
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ix = found[:,0]
    iy = found[:,1]
    iz = found[:,2]
    ax.scatter(ix, iy, iz, c='b', marker='x')
    x = center[:,0]
    y = center[:,1]
    z = center[:,2]
    ax.scatter(x, y, z, c='r', marker='o')
    ax.set_xlabel('H Label')
    ax.set_ylabel('S Label')
    ax.set_zlabel('V Label')
    plt.show()
    #############################################
    
    center = center.reshape(2,-1,3)
    center = np.uint8(center)
    center = cv2.cvtColor(center, cv2.COLOR_HSV2BGR)
    center = center.reshape(-1,3)
    #print center

    return center    

def dist(a,b):
    return np.linalg.norm(a-b)

def getProperFace(face):
    newOrder = [0,3,6,1,4,7,2,5,8]
    newFace = []
    for i in newOrder:
        newFace.append(face[i])
    return newFace

def findAverageColor(img, coords):
    #sample pixels near coords and return the average
    w = 15
    h = 15
    x, y = coords
    places = [(j,i) for i in range(x-w, x+w) for j in range(y-h, y+h)]
    fs, fh, fv = (0,0,0)
    num = 0
    for place in places:
        try:
            h,s,v = img[place]
            num += 1
            fh += h
            fs += s
            fv += v
        except:
            pass
    if num != 0:
        fh /= num
        fs /= num
        fv /= num
    return (fh, fs, fv)

def findClosestColor(color1, colors):
    #pull bgr values from color

    h,s,v = color1
    #TODO NOT ACTUALLY USING HSV ..... FIX THAT
    # Tried fixing, didn't work so well reverted.
    distances = {}
    #place color names into a dictionary with the key of distance from that color
    for i in range(len(colors)):
        color2 = colors[i]
        dh = (color2[0] - h)*1.3
        ds = color2[1] - s
        dv = (color2[2] - v)*.5 # TODO CHECK IF I NEED SCALING
        d = math.sqrt(dh**2 + ds**2 + dv**2)
        distances[d] = i
    #take the color that has minimum distance from the given color
    closest = distances[min(distances)]
    return closest

#finds approximate locations of stickers
def getStickerCoords(contours):
    # rActual and cActual are row and column  y and x coords
    #   - defaulted to 0 so if none can be found the 0 acts as a flag to approximate a location
    # rCount and cCount are counters for how many stickers have been found in each row and col
    rActual = [0,0,0]
    rCount = [0,0,0]
    cActual = [0,0,0]
    cCount = [0,0,0]
    
    #Fill in actuals and counts from contours
    for cont in contours:
        r, c = getRowCol(cont)
        x, y = getCenter(cont)
        rActual[r] += y
        rCount[r] += 1
        cActual[c] += x
        cCount[c] += 1

    #finish averaging process for actuals
    for i in range(3):
        if rCount[i] != 0:
            rActual[i] /= rCount[i]
        if cCount[i] != 0:    
            cActual[i] /= cCount[i]

    #approximate the Unknown stickers from existing data
    rActual = approximateUnknown(rActual, rCount)
    cActual = approximateUnknown(cActual, cCount)
    #return the pairs of x and y coords
    return [(int(i),int(j)) for i in cActual for j in rActual]

def approximateUnknown(actual, count, ed = 80):

    #actual is a list of the average location of known stickers in that row/col
    #   -a value of 0 in actual implies that there were no stickers found in that col/row
    #count is a list of the number of stickers found in that row/col
    #ed is an approximation for the distance between stickers incase one cannot be determined
    
    #itterate over all (row/col)s
    for i in range(3):
        #if the spot is empty
        if count[i] == 0:
            #if this is the only empty spot
            if count[(i+1)%3] != 0 and count[(i+2)%3] != 0:
                #approximate stickers position based on the other two known averages
                if i==0:
                    dd = actual[2]-actual[1]
                    actual[i] = actual[1] - np.mean((dd,dd ,ed))
                elif i==1:
                    dd = (actual[2]-actual[0])/2
                    actual[i] = actual[0] + np.mean((dd,dd,ed))
                elif i==2:
                    dd = actual[1]-actual[0]
                    actual[i] = actual[1] + np.mean((dd,dd,ed))            
            #If this is not the only empty spot
            else:
                #approximate sticker position from the known value
                if i==0:
                    if actual[1] != 0:
                        actual[i] = actual[1] - ed
                    else:
                        actual[i] = actual[2] - (2*ed)
                elif i==1:
                    if actual[2] != 0:
                        actual[i] = actual[2] - ed
                    else:
                        actual[i] = actual[0] + ed
                elif i==2:
                    if actual[1] != 0:
                        actual[i] = actual[1] + ed
                    else:
                        actual[i] = actual[0] + (2*ed)
    return actual

#Returns a tuple of the contours center coords
def getCenter(contour):
    M = cv2.moments(contour)
    if M['m00'] == 0:
        return None
    cx = M['m10']/M['m00']
    cy = M['m01']/M['m00']
    return (cx,cy)

def getRowCol(contour):
    cx, cy = getCenter(contour)
    #approximate x and y coords of where stickers should be found
    rows = [77, 162, 243]
    cols = [344, 417, 481]
    #Find which of the values the sticker center is closest to
    dc = [abs(cx-i) for i in cols]
    dr = [abs(cy-i) for i in rows]
    c = dc.index(min(dc))
    r = dr.index(min(dr))
    return (r,c)

#reduces contours to the actual number of stickers identified
def reduceToStickers(contours, areaConstraints = [3500, 6500]):
    valid = []
    for cont in contours:
        area = cv2.contourArea(cont)
        #restrict area of contour to roughly that of a sticker
        if (area > areaConstraints[0] and area < areaConstraints[1]):
            valid.append(cont)
    contours = valid
    #print("valid", len(valid))
    #remove any duplicates
    centers = [ getCenter(i) for i in contours]
    for i in range(len(centers)-1):
        for j in range(i+1, len(centers)):
            if j >= len(centers): break
            dx = centers[i][0] - centers[j][0]
            dy = centers[i][1] - centers[j][1]
            if abs(dx) + abs(dy) < 30:
                del contours[j]
                del centers[j]
    newContours = []
    #Create Apporximations for the shapes of the contours
    for cont in contours:
        eps = 0.12 * cv2.arcLength(cont, True)
        approx = cv2.approxPolyDP(cont, eps, True)
        if len(approx) == 4:
            lengths = [dist(approx[i], approx[(i+1)%4]) for i in range(4)]
            if max(lengths) - min(lengths) < 0.1*max(lengths):
                newContours.append(approx)
    return newContours