# -*- coding: utf-8 -*-
"""
Created on Sun Aug  5 17:47:19 2018

@author: Rodolfo Luis Tonoli

bvh importer based on https://github.com/20tab/bvh-python/blob/master/bvh.py

joint names:
Hips
Spine
Spine1
Spine2
Spine3
Neck
Neck1
Head
RightShoulder
RightArm
RightForeArm
RightHand
RightHandThumb1
RightHandMiddle1
LeftShoulder
LeftArm
LeftForeArm
LeftHand
LeftHandThumb1
LeftHandMiddle1
RightUpLeg
RightLeg
RightFoot
RightForeFoot
RightToeBase
LeftUpLeg
LeftLeg
LeftFoot
LeftForeFoot
LeftToeBase

"""
import numpy as np
import plotanimation
import matplotlib.pyplot as plt
import mathutils

class Joints:
    listofjointsnames = []
    listofjoints = []
    
    def __init__(self, name, depth=0,parent=None):
        self.listofjoints.append(self)
        self.listofjointsnames.append(name)
        self.name = name
        self.depth = depth
        self.children = []
        self.parent = parent
        self.endsite = []
        self.translation = []
        self.rotation = []
        self.order = []
        self.position = []
        if self.parent:
            self.parent.addChild(self)

        
    def addChild(self, item):
        item.parent = self
        self.children.append(item)
        
    def addOffset(self, offset):
        self.offset = offset
        
    def addEndSite(self, endsite=None):
        self.endsite = endsite
        self.endsiteposition = []
        
    def getLastDepth(self, depth, listJoints = []):
        if depth==0:
            return self
        else:
            for child in self.children:
                if child.depth == depth:
                    listJoints.append(child)
                child.getLastDepth(depth, listJoints)
            return listJoints[-1]
        
    def addPosition(self, pos, frame):
        if len(self.position)==0:
            totalframes = self.translation.shape[0]
            self.position = np.zeros([totalframes,3])
            if len(self.endsite) > 0:
                self.endsiteposition = np.zeros([totalframes,3])
        self.position[frame,:] = pos.ravel()
        
        
    def addEndSitePosition(self, pos, frame):
        self.endsiteposition[frame,:] = pos.ravel()
            
        
    def getByName(self, name):
        if self.name == name:
            return self
        for child in self.children:
            if child.name == name:
                return child
            else:
                found = child.getByName(name)
                if found:
                    return found
        
    def __iter__(self):
        for child in self.children:
            yield child
    
    def printHierarchy(self, hierarchy=[]):
        """Call this funtion with hierarchy=[]"""
        hierarchy.append(str.format("%s%s %s" % (' '*2*int(self.depth),self.name, self.offset)))
        #print("%s%s %s" % (' '*2*int(self.depth),self.name, self.offset))
        try:
            if len(self.endsite)>0:
                hierarchy.append("%s%s %s" % (' '*2*(int(self.depth+1)),"End Site", self.endsite))
#                print("%s%s %s" % (' '*2*(int(self.depth+1)),"End Site", self.endsite))
        except:
            pass
        for child in self.children:
            child.printHierarchy(hierarchy)
        return hierarchy

    
    def PoseBottomUp(self, value=np.zeros(3)):
        aux = np.asarray([float(number) for number in self.offset])
        value = value + aux
        if self.parent:
            value = self.parent.PoseBottomUp(value)   
        
        return value
    
    @classmethod
    def GetBones(cls, bonesPositions=[], count=[], joint=None):
        numberofframes = cls.listofjoints[0].translation.shape[0]
        if len(bonesPositions)==0:
            numberofbones = len(cls.listofjoints[0].printHierarchy(hierarchy=[]))-1
            bonesPositions = np.zeros([numberofbones, 8, numberofframes])
            #Get root
            joint = cls.listofjoints[0]
        else:
            print(len(count))
            print(joint.name)
            pp = joint.parent.position.T
            cp = joint.position.T
            bonesPositions[len(count),:,:] = np.asarray([[pp[0,frame], pp[1,frame], pp[2,frame], 1.0, cp[0,frame], cp[1,frame], cp[2,frame],1.0] for frame in range(numberofframes)]).T
            count.append([])
            if len(joint.endsite)>0:
                es = joint.endsiteposition.T
                bonesPositions[len(count),:,:] = np.asarray([[cp[0,frame], cp[1,frame], cp[2,frame], 1.0, es[0,frame], es[1,frame], es[2,frame],1.0] for frame in range(numberofframes)]).T
                count.append([])
            
        for child in joint.children:
            cls.GetBones(bonesPositions,count,child)
            #bonesPositions
            
        return bonesPositions
            

def GetData(path):      
    
    def GetMotion(joint, frame, translation=[], rotation=[]):
        """
        Get rotation and translation data for the joint at the given frame.
        """
        translation.append(frame.split(' ')[len(translation)*6:len(translation)*6+3])
        rotation.append(frame.split(' ')[len(rotation)*6+3:len(rotation)*6+6])
        joint.translation.append(translation[-1])
        joint.rotation.append(rotation[-1])
        for child in joint.children:
            GetMotion(child,frame, translation, rotation)
        return translation, rotation
    
    def Motion2NP(joint):
        joint.translation = np.asarray(joint.translation, float)
        joint.rotation = np.asarray(joint.rotation, float)
        joint.offset = np.asarray(joint.offset.split(' '),float)
        if joint.order=="ZXY":
            aux = np.copy(joint.rotation[:,0])
            joint.rotation[:,0] = np.copy(joint.rotation[:,1])
            joint.rotation[:,1] = np.copy(joint.rotation[:,2])
            joint.rotation[:,2] = np.copy(aux)
        if joint.endsite:
            joint.endsite = np.asarray(joint.endsite.split(' '),float)
        for child in joint:
            Motion2NP(child)
    
    with open(path) as file:
        data = [line for line in file]
    offsetList = []
    listofjoints = []
    
    flagEndSite = False
    for line in data:
        if line.find("ROOT") >= 0:
            root = Joints(name = line[5:-1])
            lastJoint = root
            listofjoints.append(line[5:-1])
            
        if line.find("JOINT") >= 0:
            depth = line.count('\t')
            if depth == 0:
                depth = line[:line.find('JOINT')].count(' ')/2
            parent = root.getLastDepth(depth-1)
            lastJoint = Joints(name = line[line.find("JOINT")+6:-1], depth=depth, parent=parent)
            listofjoints.append(line[line.find("JOINT")+6:-1])
    
        if line.find("End Site") >= 0:
            flagEndSite = True
            listofjoints.append("End Site")
            
        if (line.find("OFFSET") >= 0) and (not flagEndSite):
            lastJoint.addOffset(line[line.find("OFFSET")+7:-1])
            offsetList.append(line[line.find("OFFSET")+7:-1])
        elif (line.find("OFFSET") >= 0) and (flagEndSite):
            lastJoint.addEndSite(line[line.find("OFFSET")+7:-1])
            flagEndSite = False
        
        if (line.find("CHANNELS")) >= 0:
            X = line.find("Xrotation")
            Y = line.find("Yrotation")
            Z = line.find("Zrotation")
            if Z < X and X < Y:
                lastJoint.order = "ZXY"
            elif X < Y and Y < Z:
                lastJoint.order = "XYZ"
            else:
                lastJoint.order("XYZ")
                print("Invalid Channels order. XYZ chosen.")
        
        if (line.find("MOTION")) >= 0:
            index = data.index('MOTION\n')
            totalFrame = data[index+1]
            frameTime = data[index+2]
            frames = data[index+3:]
            break
    
    for counter in range(len(frames)):
        GetMotion(root, frames[counter], translation=[], rotation=[])
    
    Motion2NP(root)
    
    return root.listofjoints, frames, data

def GetPositions(joint, frame=0, parentTransform=[], surfaceinfo=None, calibrating=None):
    # Recebe o frame e a junta root, recursivamente calcula a posição de todos
    # os filhos para esse frame
    # Salva a posição em cada frame dentro de cada instância junta
    
    
    pos = joint.translation[frame]
    rot = joint.rotation[frame]
    rotx = mathutils.matrixRotation(rot[0],1,0,0)
    roty = mathutils.matrixRotation(rot[1],0,1,0)
    rotz = mathutils.matrixRotation(rot[2],0,0,1)
    transform = mathutils.matrixTranslation(pos[0], pos[1], pos[2])
    if joint.order == "ZXY":
        transform = mathutils.matrixMultiply(transform, rotz)
        transform = mathutils.matrixMultiply(transform, rotx)
        transform = mathutils.matrixMultiply(transform, roty)
    elif joint.order == "XYZ":
        transform = mathutils.matrixMultiply(transform, rotx)
        transform = mathutils.matrixMultiply(transform, roty)
        transform = mathutils.matrixMultiply(transform, rotz)
    
    if len(parentTransform) == 0:
        positionfinal = mathutils.matrixMultiply(transform, [0,0,0,1])
    else:
        transform = mathutils.matrixMultiply(parentTransform,transform)
        positionfinal = mathutils.matrixMultiply(transform,[0,0,0,1])
    
    parentTransform = np.copy(transform)
    joint.addPosition(np.asarray(positionfinal[:-1]), frame)
    
    
    #if there is surface information and a surface point or limb is attached
    #to this joint, calculate the surface point or limb position
    if surfaceinfo:
        for point in surfaceinfo:
            if point.jointlock == joint.name:
                if len(point.position) == 0:  
                    globalTransform = mathutils.matrixTranslation(point.calibrationPosition[0], point.calibrationPosition[1], point.calibrationPosition[2])
                    parentInverse = mathutils.invertMatrix(parentTransform)
                    point.calibrationLocalTransform = mathutils.matrixMultiply(parentInverse,globalTransform)
                transform = mathutils.matrixMultiply(parentTransform,point.calibrationLocalTransform)
                point.position.append(mathutils.matrixMultiply(transform,[0,0,0,1]))
    
    
    #Caso a junta tenha um endsite (é um end effector)
    if len(joint.endsite)>0:
        transform = mathutils.matrixTranslation(joint.endsite[0], joint.endsite[1], joint.endsite[2])  
        transform = mathutils.matrixMultiply(parentTransform,transform)
        endsitepos = mathutils.matrixMultiply(transform,[0,0,0,1])
        
        #Salva a posição do endsite da junta no frame atual
        joint.addEndSitePosition(np.asarray(endsitepos[:-1]), frame)
    
    
#    #if not parentTransform and not joint.parent:
#    if len(parentTransform) == 0:
#        
#        #creates root (or first joint) transform
#        posroot = joint.translation[frame]
#        rotroot = joint.rotation[frame]
#        rotx = mathutils.matrixRotation(rotroot[0],1,0,0)
#        roty = mathutils.matrixRotation(rotroot[1],0,1,0)
#        rotz = mathutils.matrixRotation(rotroot[2],0,0,1)
#        parentTransform = mathutils.matrixTranslation(posroot[0], posroot[1], posroot[2])  
#        if joint.order == "ZXY":
#            parentTransform = mathutils.matrixMultiply(parentTransform, rotz)
#            parentTransform = mathutils.matrixMultiply(parentTransform, rotx)
#            parentTransform = mathutils.matrixMultiply(parentTransform, roty)
#        elif joint.order == "XYZ":
#            parentTransform = mathutils.matrixMultiply(parentTransform, rotx)
#            parentTransform = mathutils.matrixMultiply(parentTransform, roty)
#            parentTransform = mathutils.matrixMultiply(parentTransform, rotz)
#        position = mathutils.matrixMultiply(parentTransform, [0,0,0,1])
#        #Salva a posição da junta no frame atual
#        joint.addPosition(np.asarray(position[:-1]), frame)
#        #if there is surface information and a surface point or limb is attached
#        #to this joint, calculate the surface point or limb position
#        if surfaceinfo:
#            for point in surfaceinfo:
#                if point.jointlock == joint.name:
#                    if len(point.position) == 0:  
#                        globalTransform = mathutils.matrixTranslation(point.calibrationPosition[0], point.calibrationPosition[1], point.calibrationPosition[2])
#                        parentInverse = mathutils.invertMatrix(parentTransform)
#                        point.calibrationLocalTransform = mathutils.matrixMultiply(parentInverse,globalTransform)
#                    transform = mathutils.matrixMultiply(parentTransform,point.calibrationLocalTransform)
#                    point.position.append(mathutils.matrixMultiply(transform,[0,0,0,1]))
# 
#    else:
#        pos = joint.translation[frame]
#        rot = joint.rotation[frame]
#        rotx = mathutils.matrixRotation(rot[0],1,0,0)
#        roty = mathutils.matrixRotation(rot[1],0,1,0)
#        rotz = mathutils.matrixRotation(rot[2],0,0,1)
#        transform = mathutils.matrixTranslation(pos[0], pos[1], pos[2])
#        if joint.order == "ZXY":
#            transform = mathutils.matrixMultiply(transform, rotz)
#            transform = mathutils.matrixMultiply(transform, rotx)
#            transform = mathutils.matrixMultiply(transform, roty)
#        elif joint.order == "XYZ":
#            transform = mathutils.matrixMultiply(transform, rotx)
#            transform = mathutils.matrixMultiply(transform, roty)
#            transform = mathutils.matrixMultiply(transform, rotz)
#        
#        transform = mathutils.matrixMultiply(parentTransform,transform)
#        positionfinal = mathutils.matrixMultiply(transform,[0,0,0,1])
#        parentTransform = np.copy(transform)
#        
#        #Salva a posição da junta no frame atual
#        joint.addPosition(np.asarray(positionfinal[:-1]), frame)
#        
#        #if there is surface information and a surface point or limb is attached
#        #to this joint, calculate the surface point or limb position
#        if surfaceinfo:
#            for point in surfaceinfo:
#                if point.jointlock == joint.name:
#                    if len(point.position) == 0:  
#                        globalTransform = mathutils.matrixTranslation(point.calibrationPosition[0], point.calibrationPosition[1], point.calibrationPosition[2])
#                        parentInverse = mathutils.invertMatrix(parentTransform)
#                        point.calibrationLocalTransform = mathutils.matrixMultiply(parentInverse,globalTransform)
#                    transform = mathutils.matrixMultiply(parentTransform,point.calibrationLocalTransform)
#                    point.position.append(mathutils.matrixMultiply(transform,[0,0,0,1]))
#        
#        
#        
#        #Caso a junta tenha um endsite (é um end effector)
#        if len(joint.endsite)>0:
#            transform = mathutils.matrixTranslation(joint.endsite[0], joint.endsite[1], joint.endsite[2])  
#            transform = mathutils.matrixMultiply(parentTransform,transform)
#            endsitepos = mathutils.matrixMultiply(transform,[0,0,0,1])
#            
#            #Salva a posição do endsite da junta no frame atual
#            joint.addEndSitePosition(np.asarray(endsitepos[:-1]), frame)
        
    for child in joint.children:
        GetPositions(child, frame, parentTransform, surfaceinfo)
    parentTransform=[]
    return None
            


        
def ReadFile(path, surfaceinfo=None):
            
    #path =r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\File 1.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\simplebvh.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\simplebvh2.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\simplebvh3.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\simplebvh4.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\Macarena_mcp.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\Toque_Braco_mcp.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\2_Toque_Braco_mcp.bvh'
    #path=r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\simplebvh2_mbexported_simplebvh2.bvh'
    #path=r'C:\Users\Pichau\Dropbox\_Mestrado\MBTeste\File 1.bvh'
    #path=r'C:\Users\Pichau\Dropbox\_Mestrado\MBTeste\simplebvh4.bvh'
    #path=r'C:\Users\Pichau\Dropbox\_Mestrado\MBTeste\Macarena_mcp.bvh'
    #path =r'C:\Users\Rodolfo\Dropbox\_Mestrado\MBTeste\File 1.bvh'
    #path = r'C:\Users\Rodolfo\Dropbox\_Mestrado\MBTeste\simplebvh.bvh'
    #path = r'C:\Users\Rodolfo\Dropbox\_Mestrado\MBTeste\simplebvh2.bvh'
    #path = r'C:\Users\Rodolfo\Dropbox\_Mestrado\MBTeste\Macarena_mcp.bvh'
    #path = r'C:\Users\Rodolfo\Dropbox\_Mestrado\MBTeste\Toque_Braco_mcp.bvh'
    #path = r'C:\Users\Rodolfo\Dropbox\_Mestrado\MBTeste\Superficie\Cabeca_2_mcp.bvh'
    #path = r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\Superficie\Cabeca_2_mcp.bvh'
    #path = r'C:\Users\Pichau\Dropbox\_Mestrado\MBTeste\Superficie\Cabeca_2_mcp.bvh'
    
    listofjoints, frames, data = GetData(path)
    
    for frame in range(len(frames)):
        if surfaceinfo:
            GetPositions(listofjoints[0], frame=frame, surfaceinfo=surfaceinfo)
        else:
            GetPositions(listofjoints[0], frame=frame)

        
    #plotanimation.JointVelocity(jointPos)
    #plotanimation.AnimPlotBones(bonesPos)
    #plotanimation.AnimPlotBones2D(bonesPos, 'yz')
 
    bonesPosition = listofjoints[0].GetBones()
    return listofjoints, bonesPosition


