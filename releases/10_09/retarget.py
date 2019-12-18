# -*- coding: utf-8 -*-
"""
Created on Tue Sep  4 15:12:57 2018

@author: Rodolfo Luis Tonoli
"""

import os
import numpy as np
import pyanimation
import surface
import mathutils
import time
from copy import deepcopy
import egocentriccoord
import ik
import plotanimation
########################################################
#TODO: https://web.mit.edu/m-i-t/articles/index_furniss.html
#PRE ANIMAR AVATAR, FAZER CORRESPONDENCIA DE TODAS AS JUNTAS
########################################################


def PostureInitialization(ani_ava, ani_mocap, getpositions=False, headAlign = True, spineAlign = False, handAlign = True):
    """
    Copy the rotation from the mocap joints to the corresponding avatar joint.

    ani_ava: Avatar animation
    ani_mocap: Mocap animation
    """
    #Get mapping
    mocapmap = ani_mocap.getskeletonmap()
    avamap = ani_ava.getskeletonmap()
    #
    #Save the reference TPose
    for joint in ani_ava.getlistofjoints():
        joint.tposerot = joint.rotation[0]
        joint.tposetrans = joint.translation[0]

    start=time.time()
    #Expand the number of frames of the avatar animation to match the mocap animation
    ani_ava.expandFrames(mocapmap.root.translation.shape[0])
    #Adapt pose each frame
    print('Starting Posture Initialization')
    for frame in range(ani_mocap.frames):
        if np.mod(frame+1,100) == 0:
            print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
            start=time.time()

        #Get the Height of the root in the base position (Frame = 0)
        #If the source animation (mocap) is not in the TPose, it will fail
        if frame == 0:
            ground_normal = np.array([0,1,0])
            srcHHips = np.dot(mocapmap.hips.getPosition(0), ground_normal)
            tgtHHips = np.dot(avamap.hips.getPosition(0), ground_normal)
            ratio = tgtHHips/srcHHips

        #Adjust roots/hips height
        #Eray Molla Eq 1
        srcPosHips = mocapmap.hips.getPosition(frame)
        srcGroundHips = np.asarray([srcPosHips[0], 0, srcPosHips[2]])
        tgtGroundHips = srcGroundHips*ratio
        srcHHips = np.dot(mocapmap.hips.getPosition(frame), ground_normal)
        tgtHHips = srcHHips*ratio
        avamap.root.translation[frame] = [0,tgtHHips,0] + tgtGroundHips

        if frame == 0:
            mocapbones = []
            avabones = []
            if spineAlign:
                mocapbones.append([mocapmap.hips, mocapmap.spine3])
                avabones.append([avamap.hips, avamap.spine3])
            if headAlign:
                mocapbones.append([mocapmap.neck, mocapmap.head])
                avabones.append([avamap.neck, avamap.head])                
            mocapbones = mocapbones + [[mocapmap.rarm, mocapmap.rforearm],[mocapmap.larm, mocapmap.lforearm],[mocapmap.rforearm, mocapmap.rhand],[mocapmap.lforearm, mocapmap.lhand],[mocapmap.rupleg, mocapmap.rlowleg],[mocapmap.lupleg, mocapmap.llowleg],[mocapmap.rlowleg, mocapmap.rfoot],[mocapmap.llowleg, mocapmap.lfoot]]
            avabones = avabones + [[avamap.rarm, avamap.rforearm],[avamap.larm, avamap.lforearm],[avamap.rforearm, avamap.rhand],[avamap.lforearm, avamap.lhand],[avamap.rupleg, avamap.rlowleg],[avamap.lupleg, avamap.llowleg],[avamap.rlowleg, avamap.rfoot],[avamap.llowleg, avamap.lfoot]]
            if handAlign and mocapmap.lhandmiddle and mocapmap.rhandmiddle and avamap.lhandmiddle and avamap.rhandmiddle:
                mocapbones = mocapbones + [[mocapmap.rhand, mocapmap.rhandmiddle],[mocapmap.lhand, mocapmap.lhandmiddle]]
                avabones = avabones + [[avamap.rhand, avamap.rhandmiddle],[avamap.lhand, avamap.lhandmiddle]]
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
                if not avabone[0] == avamap.root: #Does not have a parent, transform is already local
                    tgtParentGblRotationMat = mathutils.shape4ToShape3(avabone[0].parent.getGlobalTransform(frame))
                    tgtNewLclRotationMat = np.dot(tgtParentGblRotationMat.T, tgtNewGblRotationMat)
                else:
                    tgtNewLclRotationMat = tgtNewGblRotationMat[:]
                #Get new local rotation euler angles
                tgtNewLclRotationEuler, warning = mathutils.eulerFromMatrix(tgtNewLclRotationMat, avabone[0].order)
                avabone[0].rotation[frame] = tgtNewLclRotationEuler
                
        else:
            for joint_ava, joint_mocap in zip(avamap.getJointsNoRootHips(), mocapmap.getJointsNoRootHips()):
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
                    joint_ava.rotation[frame] = tgtNewLclRotationEuler[:]

        ani_ava.frames = ani_mocap.frames
        ani_ava.frametime = ani_mocap.frametime


# def checkColisions(animation, surface):
#     def jointColision(joint,surface, frame):
#         min_distance = np.inf
#         p = joint.getPosition(frame)
#         for i in range(len(surface.headmesh)+len(surface.bodymesh)):
#             if i<len(surface.headmesh):
#                 triangle = surface.headmesh[i]
#             else:
#                 triangle = surface.bodymesh[i-len(surface.headmesh)]
#             p0 = triangle[0].getPosition(frame)
#             p1 = triangle[1].getPosition(frame)
#             p2 = triangle[2].getPosition(frame)
#             #Get projection onto triangle
#             normal, baryCoord, distance_vector, projectedpoint = mathutils.projectedBarycentricCoord(p,p0,p1,p2)
#             #if projected point is inside triangle
#             if (baryCoord>0).all():
#                 #if the joint is inside the mesh, that is, the normal and the distance from the projected point have a angle between them bigger then 90 (or -90) degrees
#                 if (mathutils.cosBetween(normal, distance_vector, absolute = False) < 0):
#                     print(i)
            
            
            
#     lforearm, rforearm = animation.getskeletonmap().lforearm, animation.getskeletonmap().rforearm
#     lhand, rhand = animation.getskeletonmap().lhand, animation.getskeletonmap().rhand
#     llowleg, rlowleg = animation.getskeletonmap().llowleg, animation.getskeletonmap().rlowleg
#     lfoot, rfoot = animation.getskeletonmap().lfoot, animation.getskeletonmap().rfoot
#     limbsjoint = [lforearm, rforearm, lhand, rhand, llowleg, rlowleg, lfoot, rfoot]
#     forearm_radius = min(surface.getPoint('foreLeft').radius, surface.getPoint('foreRight').radius)
#     arm_radius = min(surface.getPoint('armLeft').radius, surface.getPoint('armRight').radius)
#     shin_radius = min(surface.getPoint('shinLeft').radius, surface.getPoint('shinRight').radius)
#     thight_radius = min(surface.getPoint('thightLeft').radius, surface.getPoint('thightRight').radius)
#     for frame in range(animation.frames):
#         for joint in limbsjoint:
            
#     # Pega juntas dos membros (as 8)
#     # Para cada uma delas:
#     #     pega a normal dos componentes
#     #     verificar a distancia e a produto interno da posicao com a normal
#     #     se for o componente mais proximo e o produto interno for negativo, significa que esta penetrando
#     #     calcular a penetracao e empurrar a junta pra fora
#     # pegar todas as juntas que sao filhas das mãos e pés, fazer a mesma coisa que de cima
        


def checkName(name):
    """
    Return True if the file in the path/name provided exists inside current path

    :type name: string
    :param name: Local path/name of the back file
    """
    currentpath = os.path.dirname(os.path.realpath(__file__))
    fullpath = os.path.join(currentpath, name)
    return os.path.isfile(fullpath)



def MotionRetargeting(sourceAnimationPath, sourceSurfacePath, targetSkeletonPath, targetSurfacePath, customSkeletomMap = None, computeEgo = True, computeIK = True, adjustOrientation = True, saveFile = True, saveInitAndFull = True):
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
    srcAnimation = pyanimation.ReadFile(sourceAnimationPath, surfaceinfo=srcSurface)
    print('MoCap BVH read done. %s seconds.' % (time.time()-start))


    #Read TPose bvh file #####################################################
    start = time.time()
    tgtAnimation = pyanimation.ReadFile(targetSkeletonPath)
    #Get skeleton map
    #TODO: Acho que não é necessário pegar o mapa aqui, remover e testar
    avatarmap = tgtAnimation.getskeletonmap(mapfile = customSkeletomMap)
    #Get the avatar surface data
    tgtSurface = surface.GetAvatarSurfaceFromCSV(targetSurfacePath, highpolymesh = False)
    #Scale the avatar surface data accordingly to the TPose bvh data
    tgtSurface.NormalizeSurfaceData(hipsjoint = avatarmap.hips)
    surface.GetAvatarSurfaceLocalTransform(tgtAnimation, tgtSurface)
    print('Avatar BVH read done. %s seconds.' % (time.time()-start))


    # Initialize pose (should be done at each frame?) ########################
    PostureInitialization(tgtAnimation, srcAnimation, getpositions = False, headAlign = True, spineAlign = False)
    start = time.time()
    print('Starting avatar surface position estimation')
    # surface.AvatarSurfacePositionEstimation(tgtAnimation, tgtSurface)
    print('Done. %s seconds.' % (time.time()-start))

    if not computeEgo:
        return tgtAnimation, tgtSurface, srcAnimation, srcSurface, None, None
    else:
        tgtAnimation_onlyInitial = deepcopy(tgtAnimation)
    
    # Calculate egocentric coordinates ############################################
    start = time.time()
    print('Getting Egocentric coordinates')
    #egocoord, targets = GetEgocentricCoordinates(mocap, mocapSurface, avatar, avatarSurface)
    egocoord = egocentriccoord.GetEgocentricCoordinatesTargets(srcAnimation, srcSurface, tgtAnimation, tgtSurface)
    print('Egocentric coordinates done. %s seconds.' % (time.time()-start))

    if not computeIK:
        return tgtAnimation, tgtSurface, srcAnimation, srcSurface, tgtAnimation_onlyInitial, egocoord

    # Aplica Cinemática Inversa ###################################################
    targetRHand = [0,0,0]
    targetLHand = [0,0,0]
    
    JacRHand = ik.SimpleJacobian(tgtAnimation, tgtAnimation.getskeletonmap().rhand, depth = 5)
    JacLHand = ik.SimpleJacobian(tgtAnimation, tgtAnimation.getskeletonmap().lhand, depth = 5)
    iklogRHand = []
    iklogLHand = []
    start=time.time()
    print('Starting IK')
    for frame in range(tgtAnimation.frames):
        targetRHand = egocoord[0].getTarget(frame)
        targetLHand = egocoord[1].getTarget(frame)
        logR = JacRHand.jacobianTranspose(frame=frame, target=targetRHand)
        logL = JacLHand.jacobianTranspose(frame=frame, target=targetLHand)
        iklogRHand.append(logR)
        iklogLHand.append(logL)
        targetRHand = [0,0,0]
        targetLHand = [0,0,0]
        if np.mod(frame+1,100) == 0:
            print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
            start=time.time()


    if not adjustOrientation:
        return tgtAnimation, tgtSurface, srcAnimation, srcSurface, tgtAnimation_onlyInitial, egocoord

    # # Adjust Limb Extremities ##################################################      
    egocentriccoord.AdjustExtremityOrientation(tgtAnimation, tgtSurface, egocoord, srcAnimation)
    #egocentriccoord.AdjustExtremityOrientation2(tgtAnimation, srcAnimation)


    if not saveFile:
        return tgtAnimation, tgtSurface, srcAnimation, srcSurface, tgtAnimation_onlyInitial, egocoord

    currentpath = os.path.dirname(os.path.realpath(__file__))
    # # Save File ###################################################
    output_filename = source_filename[:-4] + '_retarget'
    of_aux = output_filename
    i = 0
    while checkName(output_filename+'.bvh'):
        i = i + 1
        output_filename = of_aux + str(i)

    pyanimation.WriteBVH(tgtAnimation, currentpath, output_filename,refTPose=True)
    
    if saveInitAndFull:
        output_filename = source_filename[:-4] + '_initialRetarget'
        of_aux = output_filename
        i = 0
        while checkName(output_filename+'.bvh'):
            i = i + 1
            output_filename = of_aux + str(i)
    
        pyanimation.WriteBVH(tgtAnimation_onlyInitial, currentpath, output_filename,refTPose=True)


    return tgtAnimation, tgtSurface, srcAnimation, srcSurface, tgtAnimation_onlyInitial, egocoord
    print('Done! Total time: %s seconds.' % (time.time()-retargettime))

# Generate Surface Calibration ###############################################
#start = time.time()
# surface.GetCalibrationFromBVHS('Superficie\Rodolfo\Frente001_mcp.bvh', 'Superficie\Rodolfo\Cabeca001_mcp.bvh', 'Superficie\Rodolfo\Costas001_mcp.bvh',savefile = True, debugmode = False)
#surface.GetCalibrationFromBVHS('Superficie\Emely2\Frente2_mcp.bvh', 'Superficie\Emely2\Cabeca_mcp.bvh', 'Superficie\Emely2\Costas1_mcp.bvh',True,True)
#surface.GetCalibrationFromBVHS('Superficie\Paula\Frente_mcp.bvh', 'Superficie\Paula\Cabeca_mcp.bvh', 'Superficie\Paula\Costas_mcp.bvh',savefile = True,debugmode = True, minrangewide = 30, minpeakwide=40)
#surface.GetCalibrationFromBVHS('Superficie\Paula\Laura_Frente_mcp.bvh', 'Superficie\Paula\Laura_Cabecas_mcp.bvh', 'Superficie\Laura_Paula\Costas_mcp.bvh',True,True)
#surface.GetCalibrationFromBVHS('Superficie\Rodolfo3\Frente_mcp.bvh', 'Superficie\Rodolfo3\Cabeca001_mcp.bvh', 'Superficie\Rodolfo3\costas_mcp.bvh',savefile = True, debugmode = False)
# surface.GetCalibrationFromBVHS('Superficie\Rodolfo4\Frente001_mcp.bvh', 'Superficie\Rodolfo4\Cabeca_mcp.bvh', 'Superficie\Rodolfo4\Costas002_mcp.bvh',savefile = True, debugmode = True)
#print('Surface Calibration done. %s seconds.' % (time.time()-start))


# Path to Source Animation ###################################################
realpath = os.path.dirname(os.path.realpath(__file__))
# sourceanimations = ['Superficie/Rodolfo3/Abraco_mcp.bvh', 'Superficie/Rodolfo3/Boca001_mcp.bvh', 'Superficie/Rodolfo3/Boca_mcp.bvh', 'Superficie/Rodolfo3/Braco001_mcp.bvh', 'Superficie/Rodolfo3/Braco002_mcp.bvh', 'Superficie/Rodolfo3/Braco_mcp.bvh', 'Superficie/Rodolfo3/Cabeca001_mcp.bvh', 'Superficie/Rodolfo3/Cabeca_mcp.bvh', 'Superficie/Rodolfo3/Calma_mcp.bvh', 'Superficie/Rodolfo3/Calor_mcp.bvh', 'Superficie/Rodolfo3/Cansado_mcp.bvh', 'Superficie/Rodolfo3/Chateado001_mcp.bvh', 'Superficie/Rodolfo3/Chateado_mcp.bvh', 'Superficie/Rodolfo3/Cintura_mcp.bvh', 'Superficie/Rodolfo3/Continencia001_mcp.bvh', 'Superficie/Rodolfo3/Continencia_mcp.bvh', 'Superficie/Rodolfo3/Corpo_mcp.bvh', 'Superficie/Rodolfo3/costas_mcp.bvh', 'Superficie/Rodolfo3/Dedos_mcp.bvh', 'Superficie/Rodolfo3/Dormir_mcp.bvh', 'Superficie/Rodolfo3/Dor_mcp.bvh', 'Superficie/Rodolfo3/Entender_mcp.bvh', 'Superficie/Rodolfo3/Exercicio_mcp.bvh', 'Superficie/Rodolfo3/Frente_mcp.bvh', 'Superficie/Rodolfo3/Frio_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo001_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo_mcp.bvh', 'Superficie/Rodolfo3/Macarena_mcp.bvh', 'Superficie/Rodolfo3/Observar_mcp.bvh', 'Superficie/Rodolfo3/OlhoD_mcp.bvh', 'Superficie/Rodolfo3/OlhoE_mcp.bvh', 'Superficie/Rodolfo3/Orelha001_mcp.bvh', 'Superficie/Rodolfo3/Orelha_mcp.bvh', 'Superficie/Rodolfo3/Procurar_mcp.bvh', 'Superficie/Rodolfo3/Teste_Braco_mcp.bvh', 'Superficie/Rodolfo3/Teste_Geral_mcp.bvh']
#sourceanimations = ['Superficie\Rodolfo\Avo_mcp.bvh','Superficie\Rodolfo\Bisavo_mcp.bvh','Superficie\Rodolfo\Bota001_mcp.bvh','Superficie\Rodolfo\Bronzear_mcp.bvh','Superficie\Rodolfo\Cabeca001_mcp.bvh','Superficie\Rodolfo\Calma_mcp.bvh','Superficie\Rodolfo\Caminhada_mcp.bvh','Superficie\Rodolfo\Casas_mcp.bvh','Superficie\Rodolfo\Costas001_mcp.bvh','Superficie\Rodolfo\Edificio_mcp.bvh','Superficie\Rodolfo\Entender_mcp.bvh','Superficie\Rodolfo\Frente001_mcp.bvh','Superficie\Rodolfo\Macarena_mcp.bvh','Superficie\Rodolfo\Maos1_mcp.bvh','Superficie\Rodolfo\Maos2_mcp.bvh','Superficie\Rodolfo\Palmas1_mcp.bvh','Superficie\Rodolfo\Palmas2_mcp.bvh','Superficie\Rodolfo\Procurar_mcp.bvh','Superficie\Rodolfo\Salto001_mcp.bvh','Superficie\Rodolfo\Sapo_mcp.bvh','Superficie\Rodolfo\Sentar1_mcp.bvh','Superficie\Rodolfo\Sentar2_mcp.bvh','Superficie\Rodolfo\Surdo_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo2\Avo_mcp.bvh','Superficie\Rodolfo2\Bisavo_mcp.bvh', 'Superficie\Rodolfo2\Bota_mcp.bvh','Superficie\Rodolfo2\Bota001_mcp.bvh','Superficie\Rodolfo2\Bronzear_mcp.bvh','Superficie\Rodolfo2\Calma_mcp.bvh','Superficie\Rodolfo2\Caminhar_mcp.bvh','Superficie\Rodolfo2\Casas_mcp.bvh','Superficie\Rodolfo2\Edificio_mcp.bvh','Superficie\Rodolfo2\Entender_mcp.bvh','Superficie\Rodolfo2\Macarena_mcp.bvh','Superficie\Rodolfo2\Maos_mcp.bvh','Superficie\Rodolfo2\Maos001_mcp.bvh','Superficie\Rodolfo2\Palmas_mcp.bvh','Superficie\Rodolfo2\Palmas001_mcp.bvh','Superficie\Rodolfo2\Procurar_mcp.bvh','Superficie\Rodolfo2\Salto_mcp.bvh','Superficie\Rodolfo2\Sapo_mcp.bvh','Superficie\Rodolfo2\Sentar_mcp.bvh','Superficie\Rodolfo2\Sentar001_mcp.bvh','Superficie\Rodolfo2\Surdo_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo2\Avo_mcp.bvh','Superficie\Rodolfo2\Bisavo_mcp.bvh', 'Superficie\Rodolfo2\Bota_mcp.bvh','Superficie\Rodolfo2\Bota001_mcp.bvh','Superficie\Rodolfo2\Bronzear_mcp.bvh','Superficie\Rodolfo2\Calma_mcp.bvh','Superficie\Rodolfo2\Caminhar_mcp.bvh','Superficie\Rodolfo2\Casas_mcp.bvh','Superficie\Rodolfo2\Edificio_mcp.bvh','Superficie\Rodolfo2\Entender_mcp.bvh','Superficie\Rodolfo2\Macarena_mcp.bvh','Superficie\Rodolfo2\Maos_mcp.bvh','Superficie\Rodolfo2\Maos001_mcp.bvh','Superficie\Rodolfo2\Palmas_mcp.bvh','Superficie\Rodolfo2\Palmas001_mcp.bvh','Superficie\Rodolfo2\Procurar_mcp.bvh','Superficie\Rodolfo2\Salto_mcp.bvh','Superficie\Rodolfo2\Sapo_mcp.bvh','Superficie\Rodolfo2\Sentar_mcp.bvh','Superficie\Rodolfo2\Sentar001_mcp.bvh','Superficie\Rodolfo2\Surdo_mcp.bvh','Superficie/Rodolfo3/Abraco_mcp.bvh', 'Superficie/Rodolfo3/Boca001_mcp.bvh', 'Superficie/Rodolfo3/Boca_mcp.bvh', 'Superficie/Rodolfo3/Braco001_mcp.bvh', 'Superficie/Rodolfo3/Braco002_mcp.bvh', 'Superficie/Rodolfo3/Braco_mcp.bvh', 'Superficie/Rodolfo3/Cabeca001_mcp.bvh', 'Superficie/Rodolfo3/Cabeca_mcp.bvh', 'Superficie/Rodolfo3/Calma_mcp.bvh', 'Superficie/Rodolfo3/Calor_mcp.bvh', 'Superficie/Rodolfo3/Cansado_mcp.bvh', 'Superficie/Rodolfo3/Chateado001_mcp.bvh', 'Superficie/Rodolfo3/Chateado_mcp.bvh', 'Superficie/Rodolfo3/Cintura_mcp.bvh', 'Superficie/Rodolfo3/Continencia001_mcp.bvh', 'Superficie/Rodolfo3/Continencia_mcp.bvh', 'Superficie/Rodolfo3/Corpo_mcp.bvh', 'Superficie/Rodolfo3/costas_mcp.bvh', 'Superficie/Rodolfo3/Dedos_mcp.bvh', 'Superficie/Rodolfo3/Dormir_mcp.bvh', 'Superficie/Rodolfo3/Dor_mcp.bvh', 'Superficie/Rodolfo3/Entender_mcp.bvh', 'Superficie/Rodolfo3/Exercicio_mcp.bvh', 'Superficie/Rodolfo3/Frente_mcp.bvh', 'Superficie/Rodolfo3/Frio_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo001_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo_mcp.bvh', 'Superficie/Rodolfo3/Macarena_mcp.bvh', 'Superficie/Rodolfo3/Observar_mcp.bvh', 'Superficie/Rodolfo3/OlhoD_mcp.bvh', 'Superficie/Rodolfo3/OlhoE_mcp.bvh', 'Superficie/Rodolfo3/Orelha001_mcp.bvh', 'Superficie/Rodolfo3/Orelha_mcp.bvh', 'Superficie/Rodolfo3/Procurar_mcp.bvh', 'Superficie/Rodolfo3/Teste_Braco_mcp.bvh', 'Superficie/Rodolfo3/Teste_Geral_mcp.bvh']
#sourceanimations = ['Superficie\Paula\Avo_mcp.bvh', 'Superficie\Paula\Avo001_mcp.bvh','Superficie\Paula\Bisavo_mcp.bvh', 'Superficie\Paula\Bota_mcp.bvh','Superficie\Paula\Bronzear_mcp.bvh','Superficie\Paula\Cabeca_mcp.bvh','Superficie\Paula\Costas_mcp.bvh','Superficie\Paula\Danca_mcp.bvh','Superficie\Paula\Edificio_mcp.bvh','Superficie\Paula\Entender_mcp.bvh', 'Superficie\Paula\Frente_mcp.bvh','Superficie\Paula\Macarena_mcp.bvh','Superficie\Paula\Palmas_mcp.bvh','Superficie\Paula\Polichinelo_mcp.bvh','Superficie\Paula\Pulo_mcp.bvh','Superficie\Paula\Sapo_mcp.bvh','Superficie\Paula\Sentar_mcp.bvh','Superficie\Paula\Surdo_mcp.bvh']
sourceanimations = ['Superficie\Rodolfo4\Abraco_mcp.bvh', 'Superficie\Rodolfo4\Avo_mcp.bvh', 'Superficie\Rodolfo4\Baleia_mcp.bvh', 'Superficie\Rodolfo4\Barriga001_mcp.bvh', 'Superficie\Rodolfo4\Barriga2_mcp.bvh', 'Superficie\Rodolfo4\Barriga_mcp.bvh', 'Superficie\Rodolfo4\Bisavo_mcp.bvh', 'Superficie\Rodolfo4\Boca_mcp.bvh', 'Superficie\Rodolfo4\Bronzear001_mcp.bvh', 'Superficie\Rodolfo4\Bronzear_mcp.bvh', 'Superficie\Rodolfo4\Cabeca_mcp.bvh', 'Superficie\Rodolfo4\Cabelo_mcp.bvh', 'Superficie\Rodolfo4\Casas001_mcp.bvh', 'Superficie\Rodolfo4\Casas_mcp.bvh', 'Superficie\Rodolfo4\Chocado_mcp.bvh', 'Superficie\Rodolfo4\Continencia_mcp.bvh', 'Superficie\Rodolfo4\Costas001_mcp.bvh', 'Superficie\Rodolfo4\Costas002_mcp.bvh', 'Superficie\Rodolfo4\Costas_mcp.bvh', 'Superficie\Rodolfo4\Dance_mcp.bvh', 'Superficie\Rodolfo4\Entender001_mcp.bvh', 'Superficie\Rodolfo4\Entender_mcp.bvh', 'Superficie\Rodolfo4\Esconder_mcp.bvh', 'Superficie\Rodolfo4\Frente001_mcp.bvh', 'Superficie\Rodolfo4\Frente_mcp.bvh', 'Superficie\Rodolfo4\Girar_mcp.bvh', 'Superficie\Rodolfo4\Juramento_mcp.bvh', 'Superficie\Rodolfo4\Mao_na_frente_mcp.bvh', 'Superficie\Rodolfo4\Observar001_mcp.bvh', 'Superficie\Rodolfo4\Observar_mcp.bvh', 'Superficie\Rodolfo4\Olhos2_mcp.bvh', 'Superficie\Rodolfo4\Olhos3_mcp.bvh', 'Superficie\Rodolfo4\Olho_mcp.bvh', 'Superficie\Rodolfo4\Pescoco_mcp.bvh', 'Superficie\Rodolfo4\Preocupado001_mcp.bvh', 'Superficie\Rodolfo4\Preocupado_mcp.bvh', 'Superficie\Rodolfo4\Relaxar_mcp.bvh', 'Superficie\Rodolfo4\Sapo_mcp.bvh', 'Superficie\Rodolfo4\Surdo_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo3\Orelha001_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo4\Baleia_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo4\Mao_na_frente_mcp.bvh']



#sourceanimations = ['Superficie\Emely2\RotMaos_mcp.bvh']
#sourceanimations = ['Superficie\Paula\Avo_mcp.bvh']
# sourceanimations = [os.path.join(realpath, sourceanimations[i]) for i in range(len(sourceanimations))]

# Path to Source Calibration #################################################
# sourcesurface = os.path.join(realpath, 'Superficie\Rodolfo\surface_rodolfo.txt')
# sourcesurface = os.path.join(realpath, 'Superficie\Rodolfo3\surface_rodolfo.txt')
sourcesurface = os.path.join(realpath, 'Superficie\Rodolfo4\surface_rodolfo.txt')
#sourcesurface = os.path.join(realpath, 'Superficie\Emely2\surface_emely2.txt')
#sourcesurface = os.path.join(realpath, 'Superficie\Paula\surface_1.txt')

# Path to Target TPose BVH File ##############################################
# Path to Target Calibration File ############################################
# Path to Target Skeletom Map File ###########################################
# targettpose = os.path.join(realpath, 'TalitaTPose.bvh')
# targetsurface = os.path.join(realpath, 'surface_avatar.csv')
# skeletonmappath = None #Use standard
# targettpose = os.path.join(realpath, 'AragorTPose.bvh')
# targetsurface = os.path.join(realpath, 'surface_aragor.csv')
# skeletonmappath = os.path.join(realpath, 'skeletonmap_aragor.csv')
targettpose = os.path.join(realpath, 'GremlinTPose2.bvh')
targetsurface = os.path.join(realpath, 'surface_gremlin.csv')
skeletonmappath = os.path.join(realpath, 'skeletonmap_gremlin.csv')


for path in sourceanimations:
    #Get only surface
    # tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = False, computeIK = False, adjustOrientation = False, saveFile = False, saveInitAndFull = False)
    #ComputeEgo
    # tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = True, computeIK = False, adjustOrientation = False, saveFile = False, saveInitAndFull = False)
    # Get everything, dont save
    # tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = True, computeIK = True, adjustOrientation = True, saveFile = False, saveInitAndFull = False)
   # Everything and save
    tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = True, computeIK = True, adjustOrientation = True, saveFile = True, saveInitAndFull = True)
    


