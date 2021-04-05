# -*- coding: utf-8 -*-
"""
Created on Tue Sep  4 15:12:57 2018

@author: Rodolfo Luis Tonoli
"""

import os
import numpy as np
import bvhsdk
import surface
import mathutils
import time
from copy import deepcopy
import egocentriccoord
import ik
import plotanimation


def PostureInitialization(tgtMap, srcMap, heightRatio, frame, getpositions=False, headAlign = True, spineAlign = False, handAlign = True):
    """
    Copy the rotation from the mocap joints to the corresponding avatar joint.

    ani_ava: Avatar animation
    ani_mocap: Mocap animation
    """
    #start=time.time()
    #Expand the number of frames of the avatar animation to match the mocap animation

    #Adapt pose each frame
    # print('Starting Posture Initialization')
    # for frame in range(ani_mocap.frames):
    #     if np.mod(frame+1,100) == 0:
    #         print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
    #         start=time.time()



    ground_normal = np.array([0,1,0])
    #Adjust roots/hips height
    #Eray Molla Eq 1
    srcPosHips = srcMap.hips.getPosition(frame)
    srcGroundHips = np.asarray([srcPosHips[0], 0, srcPosHips[2]])
    tgtGroundHips = srcGroundHips*heightRatio
    srcHHips = np.dot(srcMap.hips.getPosition(frame), ground_normal)
    tgtHHips = srcHHips*heightRatio
    tgtMap.root.translation[frame] = [0,tgtHHips,0] + tgtGroundHips

    if frame == 0:
        mocapbones = []
        avabones = []
        if spineAlign:
            mocapbones.append([srcMap.hips, srcMap.spine3])
            avabones.append([tgtMap.hips, tgtMap.spine3])
        if headAlign:
            mocapbones.append([srcMap.neck, srcMap.head])
            avabones.append([tgtMap.neck, tgtMap.head])
        mocapbones = mocapbones + [[srcMap.rarm, srcMap.rforearm],[srcMap.larm, srcMap.lforearm],[srcMap.rforearm, srcMap.rhand],[srcMap.lforearm, srcMap.lhand],[srcMap.rupleg, srcMap.rlowleg],[srcMap.lupleg, srcMap.llowleg],[srcMap.rlowleg, srcMap.rfoot],[srcMap.llowleg, srcMap.lfoot]]
        avabones = avabones + [[tgtMap.rarm, tgtMap.rforearm],[tgtMap.larm, tgtMap.lforearm],[tgtMap.rforearm, tgtMap.rhand],[tgtMap.lforearm, tgtMap.lhand],[tgtMap.rupleg, tgtMap.rlowleg],[tgtMap.lupleg, tgtMap.llowleg],[tgtMap.rlowleg, tgtMap.rfoot],[tgtMap.llowleg, tgtMap.lfoot]]
        if handAlign and srcMap.lhandmiddle and srcMap.rhandmiddle and tgtMap.lhandmiddle and tgtMap.rhandmiddle:
            mocapbones = mocapbones + [[srcMap.rhand, srcMap.rhandmiddle],[srcMap.lhand, srcMap.lhandmiddle]]
            avabones = avabones + [[tgtMap.rhand, tgtMap.rhandmiddle],[tgtMap.lhand, tgtMap.lhandmiddle]]
        for mocapbone, avabone in zip(mocapbones,avabones):
            #Get source and target global transform and rotation matrices from the start of the bone
            p0 = mocapbone[0].getPosition(0)
            p1 = mocapbone[1].getPosition(0)
            srcDirection = mathutils.unitVector(p1-p0)
            #Get source and target global transform and rotation matrices from the end of the bone
            p0 = avabone[0].getPosition(0)
            p1 = avabone[1].getPosition(0)
            tgtDirection = mathutils.unitVector(p1-p0)
            #Align vectors
            alignMat = mathutils.alignVectors(tgtDirection, srcDirection)
            #Get new global rotation matrix
            tgtGlbTransformMat = avabone[0].getGlobalTransform(frame)
            tgtGlbRotationMat = mathutils.shape4ToShape3(tgtGlbTransformMat)
            tgtNewGblRotationMat = np.dot(alignMat,tgtGlbRotationMat)
            #Get new local rotation matrix
            if not avabone[0] == tgtMap.root: #Does not have a parent, transform is already local
                tgtParentGblRotationMat = mathutils.shape4ToShape3(avabone[0].parent.getGlobalTransform(frame))
                tgtNewLclRotationMat = np.dot(tgtParentGblRotationMat.T, tgtNewGblRotationMat)
            else:
                tgtNewLclRotationMat = tgtNewGblRotationMat[:]
            #Get new local rotation euler angles
            tgtNewLclRotationEuler, warning = mathutils.eulerFromMatrix(tgtNewLclRotationMat, avabone[0].order)
            avabone[0].setRotation(frame,tgtNewLclRotationEuler)

    else:
        for joint_ava, joint_mocap in zip(tgtMap.getJointsNoRootHips(), srcMap.getJointsNoRootHips()):
            if joint_ava is not None and joint_mocap is not None:
                previousframe = frame-1 if frame!= 0 else 0
                #Get source and target global transform and rotation matrices
                #Even if frame == 0 the matrices need to be recalculated
                srcGlbTransformMat = joint_mocap.getGlobalTransform(frame)
                srcGlbRotationMat = mathutils.shape4ToShape3(srcGlbTransformMat)
                tgtGlbTransformMat = joint_ava.getGlobalTransform(previousframe)
                tgtGlbRotationMat = mathutils.shape4ToShape3(tgtGlbTransformMat)
                #Get previous source global transform and rotation matrices
                srcPreviousGlbTransformMat = joint_mocap.getGlobalTransform(previousframe)
                srcPreviousGlbRotationMat = mathutils.shape4ToShape3(srcPreviousGlbTransformMat)
                #Get the transform of the source from the previous frame to the present frame
                transform = np.dot(srcGlbRotationMat, srcPreviousGlbRotationMat.T)
                #Apply transform
                tgtNewGblRotationMat = np.dot(transform, tgtGlbRotationMat)
                #Get new local rotation matrix
                tgtParentGblRotationMat = mathutils.shape4ToShape3(joint_ava.parent.getGlobalTransform(frame))
                tgtNewLclRotationMat = np.dot(tgtParentGblRotationMat.T, tgtNewGblRotationMat)
                #Get new local rotation euler angles
                tgtNewLclRotationEuler, warning = mathutils.eulerFromMatrix(tgtNewLclRotationMat, joint_ava.order)
                joint_ava.setRotation(frame,tgtNewLclRotationEuler[:])







def checkName(name):
    """
    Return True if the file in the path/name provided exists inside current path

    :type name: string
    :param name: Local path/name of the back file
    """
    currentpath = os.path.dirname(os.path.realpath(__file__))
    fullpath = os.path.join(currentpath, name)
    return os.path.isfile(fullpath)



def MotionRetargeting(sourceAnimationPath, sourceSurfacePath, targetSkeletonPath, targetSurfacePath, customSkeletomMap = None, computeEgo = True, computeIK = True, adjustOrientation = True, saveFile = True, saveInitAndFull = True, out_path=None):
    retargettime = time.time()

    # Surface Calibration #####################################################
    start = time.time()
    srcSurface = surface.GetMoCapSurfaceFromTXT(sourceSurfacePath, highpolymesh = False)
    print('Surface from file done. %s seconds.' % (time.time()-start))

    #Read mocap bvh file #####################################################
    source_filename = os.path.basename(sourceAnimationPath)
    start = time.time()
    #TODO: Eu mandava a superfície para calcular a posição, mas não estou mais fazendo isso
    #pq eu implementei o pointsurface.getposition. Remover e testar se alguém ainda usa isso
    srcAnimation = bvhsdk.ReadFile(sourceAnimationPath, surfaceinfo=srcSurface)
    srcMap = srcAnimation.getskeletonmap()
    print('MoCap BVH read done. %s seconds.' % (time.time()-start))


    #Read TPose bvh file #####################################################
    start = time.time()
    tgtAnimation = bvhsdk.ReadFile(targetSkeletonPath)
    #Get skeleton map
    #TODO: Acho que não é necessário pegar o mapa aqui, remover e testar
    tgtMap = tgtAnimation.getskeletonmap(mapfile = customSkeletomMap)
    #Get the avatar surface data
    tgtSurface = surface.GetAvatarSurfaceFromCSV(targetSurfacePath, highpolymesh = False)
    #Scale the avatar surface data accordingly to the TPose bvh data
    tgtSurface.NormalizeSurfaceData(hipsjoint = tgtMap.hips)
    surface.GetAvatarSurfaceLocalTransform(tgtAnimation, tgtSurface)
    print('Avatar BVH read done. %s seconds.' % (time.time()-start))


    # Initialize pose #########################################################
    tgtAnimation.frames = srcAnimation.frames
    tgtAnimation.frametime = srcAnimation.frametime
    #Save the reference TPose
    for joint in tgtAnimation.root:
        joint.tposerot = joint.rotation[0]
        joint.tposetrans = joint.translation[0]
    tgtAnimation.expandFrames(srcMap.root.translation.shape[0])

    #Get the Height of the root in the base position (Frame = 0)
    #If the source animation (mocap) is not in the TPose, it will fail
    ground_normal = np.array([0,1,0])
    srcHHips = np.dot(srcMap.hips.getPosition(0), ground_normal)
    tgtHHips = np.dot(tgtMap.hips.getPosition(0), ground_normal)
    heightRatio = tgtHHips/srcHHips

    JacRHand = ik.SimpleJacobian(tgtAnimation, tgtAnimation.getskeletonmap().rhand, depth = 5)
    JacLHand = ik.SimpleJacobian(tgtAnimation, tgtAnimation.getskeletonmap().lhand, depth = 5)

    iklogRHand = []
    iklogLHand = []

    start=time.time()
    print('Starting Motion Retargeting')
    start = time.time()
    for frame in range(srcAnimation.frames):

        PostureInitialization(tgtMap, srcMap, heightRatio, frame, getpositions = False, headAlign = True, spineAlign = False)

        #TODO: DEBUG
        # print('PI: %.4f seconds.' % (time.time()-start))


        #TODO: VERIFICAR SE AINDA PRECISO DISSO
        #print('Starting avatar surface position estimation')
        #surface.AvatarSurfacePositionEstimation(tgtAnimation, tgtSurface)
        #print('Done. %s seconds.' % (time.time()-start))

        # if not computeEgo:
        #     return tgtAnimation, tgtSurface, srcAnimation, srcSurface, None, None
        # else:
        #tgtAnimation_onlyInitial = deepcopy(tgtAnimation)

        # Calculate egocentric coordinates ############################################
        #start = time.time()
        egocoord = egocentriccoord.GetEgocentricCoordinatesTargets(srcAnimation, srcSurface, tgtAnimation, tgtSurface, frame)

        MotionRetargeting.importance.append(egocoord[0].importance)

        #TODO: DEBUG
        # print('EC: %.4f seconds.' % (time.time()-start))


        # Aplica Cinemática Inversa ###################################################
        #start = time.time()
        targetRHand = egocoord[0].getTarget(frame)
        targetLHand = egocoord[1].getTarget(frame)
        logR = JacRHand.jacobianTranspose(frame=frame, target=targetRHand)
        logL = JacLHand.jacobianTranspose(frame=frame, target=targetLHand)
        iklogRHand.append(logR)
        iklogLHand.append(logL)

        #TODO: DEBUG
        # print('IK: %.4f seconds.' % (time.time()-start))


        # targetRHand = [0,0,0]
        # targetLHand = [0,0,0]
        # if np.mod(frame+1,100) == 0:
        #     print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
        #     start=time.tim'e()

        # # Adjust Limb Extremities ##################################################
        #start = time.time()
        egocentriccoord.AdjustExtremityOrientation(tgtAnimation, tgtSurface, egocoord, srcAnimation,frame)

        #TODO: DEBUG
        #print('LE: %.4f seconds.' % (time.time()-start))


        #egocentriccoord.AdjustExtremityOrientation2(tgtAnimation, srcAnimation)

        if np.mod(frame+1,100) == 0:
            print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
            start=time.time()

    if not saveFile:
            #return tgtAnimation, tgtSurface, srcAnimation, srcSurface, tgtAnimation_onlyInitial, egocoord
            return tgtAnimation, tgtSurface, srcAnimation, srcSurface, None, egocoord



    if not out_path:
        currentpath = os.path.dirname(os.path.realpath(__file__))
    else:
        currentpath = out_path
    # # Save File ###################################################
    output_filename = source_filename[:-4] + '_retarget'
    of_aux = output_filename
    i = 0
    while checkName(output_filename+'.bvh'):
        i = i + 1
        output_filename = of_aux + str(i)

    bvhsdk.WriteBVH(tgtAnimation, currentpath, output_filename, refTPose = True)

    # if saveInitAndFull:
    #     output_filename = source_filename[:-4] + '_initialRetarget'
    #     of_aux = output_filename
    #     i = 0
    #     while checkName(output_filename+'.bvh'):
    #         i = i + 1
    #         output_filename = of_aux + str(i)
    #     bvhsdk.WriteBVH(tgtAnimation_onlyInitial, currentpath, output_filename,refTPose=True)
    #return tgtAnimation, tgtSurface, srcAnimation, srcSurface, tgtAnimation_onlyInitial, egocoord

    print('Done! Total time: %s seconds.' % (time.time()-retargettime))
    return tgtAnimation, tgtSurface, srcAnimation, srcSurface, None, egocoord
