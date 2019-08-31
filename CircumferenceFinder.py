import numpy as np
import open3d
import pandas as pd
import math

#To use with your merged CSV file, change the path on line 491.

##Gets the whole point cloud and finds the horizontal circumfrence of 
#the body part based on the last point in allPointsList.\n
#The last point in the allPointsList is the target point and It is changed during 
#the usage of app.\n
#The logic behind while calculating horizontal circumfrence: 
#Points are recorded with kinect v2 so they represents certain distances. If x coordinate 
#of a point is 0.10, y is 0.60 and z is 3.00 that means the point stays 10 cm right, 60 cm top, 
#and 3 meter front of the kinect. The circumferences are considered as ellipses and calculated based on it.\n
#Step 1: Find the left most and the right most points on the same Y, Z axis with target point.
#To do it, app draws a virtual line that stays on that axis then finds the intersections. 
#Step 2: Use these left most and right most points to find the all points on the target axis.\n
#Step 3: Find the left most and right most points in this new target circumference. It must be the same with the last ones. 
#Find distance between left most and right most points. This edge will be our first edge.\n
#Step 4: Do the same thing to find closest and farthest points. This time Y, X axis will be static. 
#After closest and farthest points are found, find the distance between them. This edge will be our second edge.\n
#Step 5: Calculate circumference of the ellipse with these 2 edges.
#@param allPointsList list: The whole list of point cloud
def HorizontalCircumference(allPointsList):
          
    #Find the left most and right most points.
    leftPointFinder = []
    leftPointFinderX = allPointsList[-1][0]
    leftPointFinderY = allPointsList[-1][1]
    leftPointFinderZ = allPointsList[-1][2]
    
    leftPointFound = False
    leftPoint = []
    
    rightPointFinder = []
    rightPointFinderX = allPointsList[-1][0]
    rightPointFinderY = allPointsList[-1][1]
    rightPointFinderZ = allPointsList[-1][2]
                
    rightPointFound = False
    rightPoint = []            
    
    for i in range (300):
        leftPointFinderX = leftPointFinderX - 0.001  
        leftPointFinder.append([leftPointFinderX, leftPointFinderY, leftPointFinderZ])
        for j in range(len(allPointsList)):
            if (leftPointFound == False and allPointsList[j][0] < leftPointFinder[i][0] and leftPointFinder[i][0] - allPointsList[j][0] < 0.05 and allPointsList[j][1] > leftPointFinder[i][1] - 0.003 and allPointsList[j][1] < leftPointFinder[i][1] + 0.003 and allPointsList[j][2] > leftPointFinder[i][2] - 0.005 and allPointsList[j][2] < leftPointFinder[i][2] + 0.005):
                leftPoint.append([allPointsList[j][0], allPointsList[j][1], allPointsList[j][2]])
                leftPointFound = True
    
        rightPointFinderX = rightPointFinderX + 0.001  
        rightPointFinder.append([rightPointFinderX, rightPointFinderY, rightPointFinderZ])
        for j in range(len(allPointsList)):
            if (rightPointFound == False and allPointsList[j][0] > rightPointFinder[i][0] and allPointsList[j][0] - rightPointFinder[i][0] < 0.05 and allPointsList[j][1] > rightPointFinder[i][1] - 0.003 and allPointsList[j][1] < rightPointFinder[i][1] + 0.003 and allPointsList[j][2] > rightPointFinder[i][2] - 0.005 and allPointsList[j][2] < rightPointFinder[i][2] + 0.005):
                rightPoint.append([allPointsList[j][0], allPointsList[j][1], allPointsList[j][2]])
                rightPointFound = True
    

    #find the point list of target circumference.       
    horizontalCircumferenceList = []
    for i in range(len(allPointsList)):
        if (allPointsList[i][1] < allPointsList[-1][1] + 0.004 and allPointsList[i][1] > allPointsList[-1][1] - 0.004):
            if(allPointsList[i][0] < rightPoint[0][0] + 0.01 and allPointsList[i][0] > leftPoint[0][0] - 0.01):
                horizontalCircumferenceList.append([allPointsList[i][0],allPointsList[i][1],allPointsList[i][2]])
                   
    horizontalPoints = np.asarray(horizontalCircumferenceList)       
    
    horizontalPointCloud = open3d.PointCloud()  
    horizontalPointCloud.points = open3d.Vector3dVector(horizontalPoints)
    open3d.draw_geometries([horizontalPointCloud])        
    


    #find the left most and right most points in this target list.              
    leftMost = horizontalCircumferenceList[0][0]
    leftMostPoint = []
    leftMostIndex = 0
    
    rightMost = horizontalCircumferenceList[0][0]
    rightMostPoint = []
    rightMostIndex = 0
    
    for i in range(len(horizontalCircumferenceList)):
        if(horizontalCircumferenceList[i][0] < leftMost):
            leftMost = horizontalCircumferenceList[i][0]
            leftMostIndex = i
        if(i == len(horizontalCircumferenceList)-1):
            leftMostPoint.append([horizontalCircumferenceList[leftMostIndex][0], horizontalCircumferenceList[leftMostIndex][1], horizontalCircumferenceList[leftMostIndex][2]])
    
        if(horizontalCircumferenceList[i][0] > rightMost):
            rightMost = horizontalCircumferenceList[i][0]
            rightMostIndex = i
        if(i == len(horizontalCircumferenceList)-1):
            rightMostPoint.append([horizontalCircumferenceList[rightMostIndex][0], horizontalCircumferenceList[rightMostIndex][1], horizontalCircumferenceList[rightMostIndex][2]])
    
    #calculate the first edge
    horizontalEdge1 = (rightMostPoint[0][0] - leftMostPoint[0][0])/2            
    
    
    #find the closest and farthest points in this target list.
    closest = horizontalCircumferenceList[0][2]
    closestPoint = []
    closestIndex = 0
    
    farthest = horizontalCircumferenceList[0][2]
    farthestPoint = []
    farthestIndex = 0
    
    for i in range(len(horizontalCircumferenceList)):
        if(horizontalCircumferenceList[i][2] > closest):
            closest = horizontalCircumferenceList[i][2]
            closestIndex = i
        if(i == len(horizontalCircumferenceList)-1):
            closestPoint.append([horizontalCircumferenceList[closestIndex][0], horizontalCircumferenceList[closestIndex][1], horizontalCircumferenceList[closestIndex][2]])
    
        if(horizontalCircumferenceList[i][2] < farthest):
            farthest = horizontalCircumferenceList[i][2]
            farthestIndex = i
        if(i == len(horizontalCircumferenceList)-1):
            farthestPoint.append([horizontalCircumferenceList[farthestIndex][0], horizontalCircumferenceList[farthestIndex][1], horizontalCircumferenceList[farthestIndex][2]])
    
    
    #Calculate the second edge.
    horizontalEdge2 = (closestPoint[0][2] - farthestPoint[0][2])/2
    
    #Calculate the circumference.
    horizontalCircumferenceMeter = 3.14*math.sqrt(2*(horizontalEdge1*horizontalEdge1 + horizontalEdge2*horizontalEdge2))
    
    print("****************************************")
    print("*                                      *")
    print("*                                      *")
    print("--->  Circumference = " + str(horizontalCircumferenceMeter))
    print("*                                      *")
    print("*                                      *")
    print("****************************************")
    
    #Draw the target area.
    DrawTargetCircumference(allPointsList, horizontalCircumferenceList)

    
##Gets the whole point cloud and finds the 45º circumfrence of 
#the body part based on the last point in allPointsList.\n
#The last point in the allPointsList is the target point and It is changed during 
#the usage of app.\n
#The logic behind while calculating horizontal circumfrence: 
#Points are recorded with kinect v2 so they represents certain distances. If x coordinate 
#of a point is 0.10, y is 0.60 and z is 3.00 that means the point stays 10 cm right, 60 cm top, 
#and 3 meter front of the kinect. The circumferences are considered as ellipses and calculated based on it.\n
#Step 1: Find the highest and the lowest points on the same Z axis with target point.
#To do it, app draws a virtual line that stays on that axis then finds the intersections. While drawing the virtual line, X and Y 
#axis must be changed to create a 45 degree degreed ellipse. Also the way ellipse stays is changed depands on the second parameter 
#leftorRight. It determines the direction of how ellipse goes.
#To do it, app draws a virtual line that stays on that axis then finds the intersections. 
#Step 2: Use these left most and right most points to find the all points on the target axis.\n
#Step 3: Find the highest and lowest points in this new target circumference. It must be the same with the last ones. 
#Find distance between lowest and highest points. This edge will be our first edge.\n
#Step 4: Do the same thing to find closest and farthest points. This time Y, X axis will be static. This edge is 
#90 degree so we keep these axises static.
#Step 5: Calculate circumference of the ellipse with these 2 edges. 
#@param allPointsList list: The whole list of point cloud   
#@param leftOrRight string: Represents how the circumference ellipse will stay.  
def DegreedCircumference(allPointsList, leftOrRight):
    
    rightUp = []
    rightUpX = allPointsList[-1][0]
    rightUpY = allPointsList[-1][1]
    rightUpZ = allPointsList[-1][2]
    
    #Draw the virtual 45 degreed line and find left most and right most points.
    if(leftOrRight == "Right"):
        for i in range (50):
            rightUpX = rightUpX - 0.001
            rightUpY = rightUpY + 0.001
            rightUp.append([rightUpX, rightUpY, rightUpZ])
        
        rightUpX = allPointsList[-1][0]
        rightUpY = allPointsList[-1][1]
        rightUpZ = allPointsList[-1][2]
        
        for i in range (50):
            rightUpX = rightUpX + 0.001
            rightUpY = rightUpY - 0.001
            rightUp.append([rightUpX, rightUpY, rightUpZ])
    
    elif(leftOrRight == "Left"):
        for i in range (50):
            rightUpX = rightUpX + 0.001
            rightUpY = rightUpY + 0.001
            rightUp.append([rightUpX, rightUpY, rightUpZ])
        
        rightUpX = allPointsList[-1][0]
        rightUpY = allPointsList[-1][1]
        rightUpZ = allPointsList[-1][2]
        
        for i in range (50):
            rightUpX = rightUpX - 0.001
            rightUpY = rightUpY - 0.001
            rightUp.append([rightUpX, rightUpY, rightUpZ])
    

    #Find the target area.
    degreedCircumferenceList = []
    for j in range(len(rightUp)):
        minX = rightUp[j][0] - 0.002
        maxX = rightUp[j][0] + 0.002
        minY = rightUp[j][1] - 0.002
        maxY = rightUp[j][1] + 0.002
        for i in range(len(allPointsList)):
            if(allPointsList[i][0] > minX and allPointsList[i][0] < maxX and allPointsList[i][1] > minY and allPointsList[i][1] < maxY):
                degreedCircumferenceList.append([allPointsList[i][0], allPointsList[i][1], allPointsList[i][2]])
    

    #Find highest and lowest points. Then calculate the distance between them.    
    highest = degreedCircumferenceList[0][1]
    highestPoint = []
    highestIndex = 0
    for i in range(len(degreedCircumferenceList)):
        if(degreedCircumferenceList[i][1] > highest):
            highest = degreedCircumferenceList[i][1]
            highestIndex = i
        if(i == len(degreedCircumferenceList)-1):
            highestPoint.append([degreedCircumferenceList[highestIndex][0], degreedCircumferenceList[highestIndex][1], degreedCircumferenceList[highestIndex][2]])
    
    lowest = degreedCircumferenceList[0][1]
    lowestPoint = []
    lowestIndex = 0
    for i in range(len(degreedCircumferenceList)):
        if(degreedCircumferenceList[i][1] < lowest):
            lowest = degreedCircumferenceList[i][1]
            lowestIndex = i
        if(i == len(degreedCircumferenceList)-1):
            lowestPoint.append([degreedCircumferenceList[lowestIndex][0], degreedCircumferenceList[lowestIndex][1], degreedCircumferenceList[lowestIndex][2]])
    
    #Calculate the edge.
    degreedEdge1 = (math.sqrt(
            (highestPoint[0][0]-lowestPoint[0][0])*(highestPoint[0][0]-lowestPoint[0][0]) + 
            (highestPoint[0][1]-lowestPoint[0][1])*(highestPoint[0][1]-lowestPoint[0][1]) +
            (highestPoint[0][2]-lowestPoint[0][2])*(highestPoint[0][2]-lowestPoint[0][2])))/2
         
            
    #Find closest and farthest points.        
    closest = degreedCircumferenceList[0][2]
    closestPoint = []
    closestIndex = 0
    for i in range(len(degreedCircumferenceList)):
        if(degreedCircumferenceList[i][2] > closest):
            closest = degreedCircumferenceList[i][2]
            closestIndex = i
        if(i == len(degreedCircumferenceList)-1):
            closestPoint.append([degreedCircumferenceList[closestIndex][0], degreedCircumferenceList[closestIndex][1], degreedCircumferenceList[closestIndex][2]])
    
    farthest = degreedCircumferenceList[0][2]
    farthestPoint = []
    farthestIndex = 0
    for i in range(len(degreedCircumferenceList)):
        if(degreedCircumferenceList[i][2] < farthest):
            farthest = degreedCircumferenceList[i][2]
            farthestIndex = i
        if(i == len(degreedCircumferenceList)-1):
            farthestPoint.append([degreedCircumferenceList[farthestIndex][0], degreedCircumferenceList[farthestIndex][1], degreedCircumferenceList[farthestIndex][2]])
    
    #Calculate the second edge.
    degreedEdge2 = (closestPoint[0][2] - farthestPoint[0][2])/2
    
    degreedPoints = np.asarray(degreedCircumferenceList) 
    degreedPointCloud = open3d.PointCloud()      
    degreedColors = []
    
    for i in range(len(degreedCircumferenceList)):
        if(degreedCircumferenceList[i][1] == highest or degreedCircumferenceList[i][1] == lowest or degreedCircumferenceList[i][2] == closest or degreedCircumferenceList[i][2] == farthest):
            degreedColors.append([0, 0, 0])           
        else:
            degreedColors.append([0.286, 0.858, 0.992])
            
    degreedPointCloud.points = open3d.Vector3dVector(degreedPoints)
    degreedPointCloud.colors = open3d.Vector3dVector(degreedColors)
    open3d.draw_geometries([degreedPointCloud])
    
    #Find circumference.
    degreedCircumferenceMeter = 3.14*math.sqrt(2*(degreedEdge1*degreedEdge1 + degreedEdge2*degreedEdge2))
    
    print("****************************************")
    print("*                                      *")
    print("*                                      *")
    print("--->  Circumference = " + str(degreedCircumferenceMeter))
    print("*                                      *")
    print("*                                      *")
    print("****************************************")
    
    DrawTargetCircumference(allPointsList, degreedCircumferenceList)

##Gets the whole point cloud and finds the vertical circumfrence of 
#the body part based on the last point in allPointsList.\n
#The last point in the allPointsList is the target point and It is changed during 
#the usage of app.\n
#The logic behind while calculating horizontal circumfrence: 
#Points are recorded with kinect v2 so they represents certain distances. If x coordinate 
#of a point is 0.10, y is 0.60 and z is 3.00 that means the point stays 10 cm right, 60 cm top, 
#and 3 meter front of the kinect. The circumferences are considered as ellipses and calculated based on it.\n
#This time ellipse is staying vertical. Here left, right points will be the top,bottom points and front, back points will be the left, right points. If we think them 
#like we think it in other examples.
#Step 1: Find the left most and the right most points on the same X, Z axis with target point. These will be our top and bottom points.
#To do it, app draws a virtual line that stays on that axis then finds the intersections. 
#Step 2: Use these left most and right most points to find the all points on the target axis.\n
#Step 3: Find the left most and right most points in this new target circumference. It must be the same with the last ones. 
#Find distance between left most and right most points. This edge will be our first edge.\n
#Step 2: Do the same thing to find closest and farthest points. This time Y, X axis will be static. These will be our left, right points.
#After closest and farthest points are found, find the distance between them. This edge will be our second edge.\n
#Step 3: Calculate circumference of the ellipse with these 2 edges. 
#@param allPointsList list: The whole list of point cloud
def VerticalCircumference(allPointsList):
          
    leftPointFinder = []
    leftPointFinderX = allPointsList[-1][0]
    leftPointFinderY = allPointsList[-1][1]
    leftPointFinderZ = allPointsList[-1][2]
    
    leftPointFound = False
    leftPoint = []
    
    rightPointFinder = []
    rightPointFinderX = allPointsList[-1][0]
    rightPointFinderY = allPointsList[-1][1]
    rightPointFinderZ = allPointsList[-1][2]
                
    rightPointFound = False
    rightPoint = []            
    
    for i in range (300):
        leftPointFinderY = leftPointFinderY - 0.001  
        leftPointFinder.append([leftPointFinderX, leftPointFinderY, leftPointFinderZ])
        for j in range(len(allPointsList)):
            if (leftPointFound == False and allPointsList[j][1] < leftPointFinder[i][1] and leftPointFinder[i][1] - allPointsList[j][1] < 0.05 and allPointsList[j][0] > leftPointFinder[i][0] - 0.003 and allPointsList[j][0] < leftPointFinder[i][0] + 0.003 and allPointsList[j][2] > leftPointFinder[i][2] - 0.005 and allPointsList[j][2] < leftPointFinder[i][2] + 0.005):
                leftPoint.append([allPointsList[j][0], allPointsList[j][1], allPointsList[j][2]])
                leftPointFound = True
    
        rightPointFinderY = rightPointFinderY + 0.001  
        rightPointFinder.append([rightPointFinderX, rightPointFinderY, rightPointFinderZ])
        for j in range(len(allPointsList)):
            if (rightPointFound == False and allPointsList[j][1] > rightPointFinder[i][1] and allPointsList[j][1] - rightPointFinder[i][1] < 0.05 and allPointsList[j][0] > rightPointFinder[i][0] - 0.003 and allPointsList[j][0] < rightPointFinder[i][0] + 0.003 and allPointsList[j][2] > rightPointFinder[i][2] - 0.005 and allPointsList[j][2] < rightPointFinder[i][2] + 0.005):
                rightPoint.append([allPointsList[j][0], allPointsList[j][1], allPointsList[j][2]])
                rightPointFound = True
           
    verticalCircumferenceList = []
    for i in range(len(allPointsList)):
        if (allPointsList[i][0] < allPointsList[-1][0] + 0.004 and allPointsList[i][0] > allPointsList[-1][0] - 0.004):
            if(allPointsList[i][1] < rightPoint[0][1] + 0.01 and allPointsList[i][1] > leftPoint[0][1] - 0.01):
                verticalCircumferenceList.append([allPointsList[i][0],allPointsList[i][1],allPointsList[i][2]])
                   
    verticalPoints = np.asarray(verticalCircumferenceList)       
    
    verticalPointCloud = open3d.PointCloud()  
    verticalPointCloud.points = open3d.Vector3dVector(verticalPoints)
    open3d.draw_geometries([verticalPointCloud])        
                  
    leftMost = verticalCircumferenceList[0][1]
    leftMostPoint = []
    leftMostIndex = 0
    
    rightMost = verticalCircumferenceList[0][1]
    rightMostPoint = []
    rightMostIndex = 0
    
    for i in range(len(verticalCircumferenceList)):
        if(verticalCircumferenceList[i][1] < leftMost):
            leftMost = verticalCircumferenceList[i][1]
            leftMostIndex = i
        if(i == len(verticalCircumferenceList)-1):
            leftMostPoint.append([verticalCircumferenceList[leftMostIndex][0], verticalCircumferenceList[leftMostIndex][1], verticalCircumferenceList[leftMostIndex][2]])
    
        if(verticalCircumferenceList[i][1] > rightMost):
            rightMost = verticalCircumferenceList[i][1]
            rightMostIndex = i
        if(i == len(verticalCircumferenceList)-1):
            rightMostPoint.append([verticalCircumferenceList[rightMostIndex][0], verticalCircumferenceList[rightMostIndex][1], verticalCircumferenceList[rightMostIndex][2]])
    
    verticalEdge1 = (rightMostPoint[0][1] - leftMostPoint[0][1])/2            
    
    closest = verticalCircumferenceList[0][2]
    closestPoint = []
    closestIndex = 0
    
    farthest = verticalCircumferenceList[0][2]
    farthestPoint = []
    farthestIndex = 0
    
    for i in range(len(verticalCircumferenceList)):
        if(verticalCircumferenceList[i][2] > closest):
            closest = verticalCircumferenceList[i][2]
            closestIndex = i
        if(i == len(verticalCircumferenceList)-1):
            closestPoint.append([verticalCircumferenceList[closestIndex][0], verticalCircumferenceList[closestIndex][1], verticalCircumferenceList[closestIndex][2]])
    
        if(verticalCircumferenceList[i][2] < farthest):
            farthest = verticalCircumferenceList[i][2]
            farthestIndex = i
        if(i == len(verticalCircumferenceList)-1):
            farthestPoint.append([verticalCircumferenceList[farthestIndex][0], verticalCircumferenceList[farthestIndex][1], verticalCircumferenceList[farthestIndex][2]])
    
    verticalEdge2 = (closestPoint[0][2] - farthestPoint[0][2])/2
    
    verticalCircumferenceMeter = 3.14*math.sqrt(2*(verticalEdge1*verticalEdge1 + verticalEdge2*verticalEdge2))
    
    print("****************************************")
    print("*                                      *")
    print("*                                      *")
    print("--->  Circumference = " + str(verticalCircumferenceMeter))
    print("*                                      *")
    print("*                                      *")
    print("****************************************")
    
    DrawTargetCircumference(allPointsList, verticalCircumferenceList)
           
##Gets the whole point cloud and paints the last element of it. 
#This function is used to show the target point to the user.
#@param allPointsList list: The whole list of point cloud
def PaintTheLastElement(allPointsList):
    
    allPoints = np.asarray(allPointsList)                     
    allPointCloud= open3d.PointCloud()
                    
    allColors = []
    index = 0;
    for i in range(len(allPoints)):
        if(index < 25):
            allColors.append([1, 0.039, 0.172])
        elif(index < len(df) and index >= 25):
            allColors.append([0.286, 0.858, 0.992])
        else:
            allColors.append([0, 0, 0])
        index = index +1
                               
    allPointCloud.points = open3d.Vector3dVector(allPoints)
    allPointCloud.colors = open3d.Vector3dVector(allColors)
    open3d.draw_geometries([allPointCloud])

##Gets the whole point cloud and the point list of target area.
#This function is used in vertical, horizontal, and 45 degree degreed functions.
#@param allPointsList list: The whole list of point cloud
#@param targetPointsList list: The list of target area
def DrawTargetCircumference(allPointsList, targetPointsList):   
    selectHorizontalColors = []
    for i in range(len(allPointsList)-1):
        for j in range(len(targetPointsList)):
            if(allPointsList[i][0] == targetPointsList[j][0] and allPointsList[i][1] == targetPointsList[j][1] and allPointsList[i][2] == targetPointsList[j][2]):
                listPoints2Check = 1
                break
            else:
                listPoints2Check = 0                 
        if(listPoints2Check == 1):            
            selectHorizontalColors.append([1, 0.039, 0.172])
            
        else:
            selectHorizontalColors.append([0.286, 0.858, 0.992])
        
    bellyPointCloud = open3d.PointCloud()
    bellyPointCloud.points = open3d.Vector3dVector(allPoints)
    bellyPointCloud.colors = open3d.Vector3dVector(selectHorizontalColors)
    open3d.draw_geometries([bellyPointCloud])
    
#%%

#Read the merged csv file.
df = pd.read_csv("MergedCSV.csv",sep = ",")

#Convert the dataframe into list
allPointsList = df.values.tolist()

#Convert the list into array to be used with open3d functions.
allPoints = np.asarray(allPointsList) 

#Create point cloud
allPointCloud= open3d.PointCloud()

#List of colors will be used in point cloud. First 25 line gets a different color.
allColors = []
index = 0;
for i in range(len(allPoints)):
    if(index < 25):
        allColors.append([1, 0.039, 0.172])
    elif(index < len(df) and index >= 25):
        allColors.append([0.286, 0.858, 0.992])
    else:
        allColors.append([0, 0, 0])
    index = index +1
           
allPointCloud.points = open3d.Vector3dVector(allPoints)
allPointCloud.colors = open3d.Vector3dVector(allColors)
open3d.draw_geometries([allPointCloud])

#%%

inpt = ""
while(True):
    print("1 - Horizontal\n2 - 45º\n3 - Vertical\n0 - Exit")
    inpt = input("Please choose one: ")
    if(inpt == "1" or inpt == "2" or inpt == "3" or inpt == "0"):
        break

if(inpt == "1"):
    print("1 - Belly\n2 - Right Leg Calf\n3 - Left Leg Calf\n0 - Exit")
    inpt2 = input("Please choose one: ")
    
#The rate of Y axis. Can be modified to change the Y coordinate of target point.
    midBaseRate = 1
            
    while(True):
        
        if(inpt2 == "1"):
#Belly is the target point. Creates the point then add it to the allPointsList.
#So the last element will be the Belly point.                    
            bellyX = (df.values[0][0] + df.values[1][0])/2
            bellyY = (df.values[0][1] + (midBaseRate * df.values[1][1]))/2   
            bellyZ = (df.values[0][2] + df.values[1][2])/2 
            allPointsList.append([bellyX,bellyY,bellyZ])
            print("1 - Paint to Black and Show the Belly Point\n2 - Go Up\n3 - Go Down\n4 - Calculate\n0 - Exit")

#Right Calf is the target point. Creates the point then add it to the allPointsList.
#So the last element will be the Right Calf.
        elif(inpt2 == "2"):            
                rightCalfX = (df.values[17][0] + df.values[16][0])/2
                rightCalfY = (df.values[17][1] + (midBaseRate * df.values[16][1]))/2   
                rightCalfZ = (df.values[17][2] + df.values[16][2])/2 
                allPointsList.append([rightCalfX,rightCalfY,rightCalfZ])
                print("1 - Paint to Black and Show the Right Calf Point\n2 - Go Up\n3 - Go Down\n4 - Calculate\n0 - Exit")
                
#Left Calf is the target point. Creates the point then add it to the allPointsList.
#So the last element will be the Left Calf point.            
        elif(inpt2 == "3"):         
                leftCalfX = (df.values[13][0] + df.values[12][0])/2
                leftCalfY = (df.values[13][1] + (midBaseRate * df.values[12][1]))/2   
                leftCalfZ = (df.values[13][2] + df.values[12][2])/2 
                allPointsList.append([leftCalfX,leftCalfY,leftCalfZ])
                print("1 - Paint to Black and Show the Left Calf Point\n2 - Go Up\n3 - Go Down\n4 - Calculate\n0 - Exit")
                
        elif(inpt2 == "0"):
            break
        
        inpt3 = input("Please choose one: ")                
        if(inpt3 == "1"):
            PaintTheLastElement(allPointsList)
#After the options the last element will be deleted because It will be added again after 
#while loop is activated. With this way, target point can be modified.
            del allPointsList[-1]
        elif(inpt3 == "2"):
#Rate rises if go up option is selected.
            midBaseRate = midBaseRate + 0.1
            del allPointsList[-1]
        elif(inpt3 == "3"):
#Rate downs if go down option is selected.
            midBaseRate = midBaseRate - 0.1
            del allPointsList[-1]
        elif(inpt3 == "4"):
            HorizontalCircumference(allPointsList)
            del allPointsList[-1]
        elif(inpt3 == "0"):
            break
                                        
elif(inpt == "2"):
    midBaseRate = 0.00
    print("1 - Right\n2 - Left\n0 - Exit")
    inpt2 = input("Please choose one: ")
    
    if(inpt2 == "1"):    
        print("1 - Right Biceps\n0 - Exit")
        inpt3 = input("Please choose one: ")
               
        if(inpt3 == "1"):           
            while(True):
#                rightBicepsX = (df.values[8][0] + (midBaseRate * df.values[9][0])/2
#                rightBicepsY = (df.values[8][1] + df.values[9][1])/2 + midBaseRate
#                rightBicepsZ = (df.values[8][2] + df.values[9][2])/2 
                rightBicepsX = df.values[9][0] + midBaseRate
                rightBicepsY = df.values[9][1] + midBaseRate
                rightBicepsZ = df.values[9][2] 
                allPointsList.append([rightBicepsX,rightBicepsY,rightBicepsZ])
                print("1 - Paint to Black and Show the Right Biceps Point\n2 - Go Up\n3 - Go Down\n4 - Calculate\n0 - Exit")
                inpt3 = input("Please choose one: ")                
                if(inpt3 == "1"):
                    PaintTheLastElement(allPointsList)
                    del allPointsList[-1]
                elif(inpt3 == "2"):
                    midBaseRate = midBaseRate + 0.01
                    del allPointsList[-1]
                elif(inpt3 == "3"):
                    midBaseRate = midBaseRate - 0.01
                    del allPointsList[-1]
                elif(inpt3 == "4"):
                    DegreedCircumference(allPointsList, "Right")
                    del allPointsList[-1]
                elif(inpt3 == "0"):
                    break            

    elif(inpt2 == "2"):
        print("1 - Left Biceps\n0 - Exit")
        inpt3 = input("Please choose one: ")
        
        if(inpt3 == "1"):           
            while(True):
#                rightBicepsX = (df.values[4][0] + df.values[5][0])/2 + midBaseRate
#                rightBicepsY = (df.values[4][1] + df.values[5][1])/2 + midBaseRate
#                rightBicepsZ = (df.values[4][2] + df.values[5][2])/2 
                leftBicepsX = df.values[5][0] - midBaseRate
                leftBicepsY = df.values[5][1] + midBaseRate
                leftBicepsZ = df.values[5][2] 
                allPointsList.append([leftBicepsX,leftBicepsY,leftBicepsZ])
                print("1 - Paint to Black and Show the Left Biceps Point\n2 - Go Up\n3 - Go Down\n4 - Calculate\n0 - Exit")
                inpt3 = input("Please choose one: ")                
                if(inpt3 == "1"):
                    PaintTheLastElement(allPointsList)
                    del allPointsList[-1]
                elif(inpt3 == "2"):
                    midBaseRate = midBaseRate + 0.01
                    del allPointsList[-1]
                elif(inpt3 == "3"):
                    midBaseRate = midBaseRate - 0.01
                    del allPointsList[-1]
                elif(inpt3 == "4"):
                    DegreedCircumference(allPointsList, "Left")
                    del allPointsList[-1]
                elif(inpt3 == "0"):
                    break   
                

elif(inpt == "3"):
    print("1 - Right Biceps\n2 - Left Biceps\n0 - Exit")
    inpt2 = input("Please choose one: ")
    midBaseRate = 1
    
    while(True):
        
        if(inpt2 == "1"):       
            rightBicepsX = (df.values[8][0] + (midBaseRate * df.values[9][0]))/2 
            rightBicepsY = (df.values[8][1] + df.values[9][1])/2
            rightBicepsZ = (df.values[8][2] + df.values[9][2])/2             
            allPointsList.append([rightBicepsX,rightBicepsY,rightBicepsZ])
            print("1 - Paint to Black and Show the Right Biceps Point\n2 - Go Up\n3 - Go Down\n4 - Calculate\n0 - Exit")
            
        elif(inpt2 == "2"):
            rightBicepsX = (df.values[4][0] + (midBaseRate * df.values[5][0]))/2 
            rightBicepsY = (df.values[4][1] + df.values[5][1])/2
            rightBicepsZ = (df.values[4][2] + df.values[5][2])/2             
            allPointsList.append([rightBicepsX,rightBicepsY,rightBicepsZ])
            print("1 - Paint to Black and Show the Left Biceps Point\n2 - Go Up\n3 - Go Down\n4 - Calculate\n0 - Exit")
        
        elif(inpt2 == "0"):
            break
                
        inpt3 = input("Please choose one: ")                
        if(inpt3 == "1"):
            PaintTheLastElement(allPointsList)
            del allPointsList[-1]
        elif(inpt3 == "2"):
            midBaseRate = midBaseRate + 0.05
            del allPointsList[-1]
        elif(inpt3 == "3"):
            midBaseRate = midBaseRate - 0.05
            del allPointsList[-1]
        elif(inpt3 == "4"):
            VerticalCircumference(allPointsList)
            del allPointsList[-1]
        elif(inpt3 == "0"):
            break

#%%
            
#SPINEBASE = 0;
#SPINEMID = 1;
#NECK = 2;
#HEAD = 3;
#SHOULDERLEFT = 4;
#ELBOWLEFT = 5;
#WRISTLEFT = 6;
#HANDLEFT = 7;
#SHOULDERRIGHT = 8;
#ELBOWRIGHT = 9;
#WRISTRIGHT = 10;
#HANDRIGHT = 11;
#HIPLEFT = 12;
#KNEELEFT = 13;
#ANKLELEFT = 14;
#FOOTLEFT = 15;
#HIPRIGHT = 16;
#KNEERIGHT = 17;
#ANKLERIGHT = 18;
#FOOTRIGHT = 19;
#SPINESHOULDER = 20;
#HANDTIPLEFT  = 21;
#THUMBLEFT = 22;
#HANDTIPRIGHT = 23;
#THUMBRIGHT = 24;                                       
    