# -*- coding: utf-8 -*-
#http://help.autodesk.com/view/FBX/2019/ENU/
#Export options:
#FBX binary (*.fbx)
#FBX ascii (*.fbx)
#FBX encrypted (*.fbx)
#FBX 6.0 binary (*.fbx)
#FBX 6.0 ascii (*.fbx)
#FBX 6.0 encrypted (*.fbx)
#AutoCAD DXF (*.dxf)
#Alias OBJ (*.obj)
#Collada DAE (*.dae)
#Biovision BVH (*.bvh)
#Motion Analysis HTR (*.htr)
#Motion Analysis TRC (*.trc)
#Acclaim ASF (*.asf)
#Acclaim AMC (*.amc)
#Vicon C3D (*.c3d)
#Adaptive Optics AOA (*.aoa)
#Superfluo MCD (*.mcd)

import fbx
import sys
import numpy as np
import plotanimation

class Joint:

    listofjoints = []

    def __init__(self, name):
        self.name = name
        self.listofjoints.append(name)

    def SetTRS(self, trans, rot, scl=[]):
        self.translation = trans
        self.rotation = rot
        self.scale = scl

    def SetLcl(self, lclTrans, lclRot, lclScl=[]):
        self.lclTrans = lclTrans
        self.lclRot = lclRot
        self.lclScl = lclScl

def printSkeletonLocals(node,j=0):
    #print("%s%s" % (" "*j,node.GetName()))
    localTrans = [node.LclTranslation.Get()[0],node.LclTranslation.Get()[1],node.LclTranslation.Get()[2]]
    localRot = [node.LclRotation.Get()[0],node.LclRotation.Get()[1],node.LclRotation.Get()[2]]
    skel = node.GetSkeleton()
    print("%s%s lcltrans: (%d,%d,%d) lclrot: (%d,%d,%d) "
        % (" "*j,
        node.GetName(),
        localTrans[0], localTrans[1], localTrans[2],
        localRot[0], localRot[1], localRot[2]
        )
        )
    for i in range(0,node.GetChildCount()):
        printSkeleton(node.GetChild(i),j+1)

def printSkeletonGlobals(node,j=0):
    globalTrans = [node.GblTranslation.Get()[0],node.GblTranslation.Get()[1],node.GblTranslation.Get()[2]]
    globalRot = [node.GblRotation.Get()[0],node.GblRotation.Get()[1],node.GblRotation.Get()[2]]
    skel = node.GetSkeleton()
    print("%s%s lcltrans: (%d,%d,%d) lclrot: (%d,%d,%d) "
        % (" "*j,
        node.GetName(),
        globalTrans[0], globalTrans[1], globalTrans[2],
        globalRot[0], globalRot[1], globalRot[2]
        )
        )
    for i in range(0,node.GetChildCount()):
        printSkeleton(node.GetChild(i),j+1)

def getNodeCurves(node, animLayer=0, j=0, joints=[]):
    #Retorna as curvas dos nós. Assume que todas as curvas possui o mesmo número de keys

    newjoint = Joint(node.GetName())

    lclTrans = node.LclTranslation.Get()
    lclRot = node.LclRotation.Get()
    newjoint.SetLcl(np.array([lclTrans[0],lclTrans[1],lclTrans[2]]), np.array([lclRot[0],lclRot[1],lclRot[2]]))

    lclTransX = node.LclTranslation.GetCurve(animLayer,"X" ,False)
    lclTransY = node.LclTranslation.GetCurve(animLayer,"Y" ,False)
    lclTransZ = node.LclTranslation.GetCurve(animLayer,"Z" ,False)
    lclRotX = node.LclRotation.GetCurve(animLayer, "X",False)
    lclRotY = node.LclRotation.GetCurve(animLayer, "Y",False)
    lclRotZ = node.LclRotation.GetCurve(animLayer, "Z",False)

    if lclTransX:
        keyCount = lclTransX.KeyGetCount()
    elif lclRotX:
        keyCount = lclRotX.KeyGetCount()
    else:
        keyCount = 0

    #if (gbljointpos==None):
    #    gbljointpos = np.empty([keyCount,3, node.GetChildCount(True)])

    #else:
    #    if (keyCount > gbljointpos.shape[0]):
    #        gbljointpos.resize([keyCount,3,gbljointpos.shape[2]], refcheck=False)

    transVector = np.empty(shape=(keyCount,4))
    rotVector = np.empty(shape=(keyCount,4))
    sclVector = np.empty(shape=(keyCount,4))

    time = fbx.FbxTime()
    for i in range(keyCount):
        time.SetFrame(i+1, fbx.FbxTime.eFrames120)
        glbTransMatrix = node.EvaluateGlobalTransform(time)
        print(node.GetName())
        print(glbTransMatrix.GetColumn(0))
        print(glbTransMatrix.GetColumn(1))
        print(glbTransMatrix.GetColumn(2))
        print(glbTransMatrix.GetColumn(3))
        auxVector = glbTransMatrix.GetT()
        transVector[i,:] = np.array([auxVector[0],auxVector[1],auxVector[2],auxVector[3]])
        auxVector = glbTransMatrix.GetR()
        rotVector[i,:] = np.array([auxVector[0],auxVector[1],auxVector[2],auxVector[3]])
        auxVector = glbTransMatrix.GetS()
        sclVector[i,:] = np.array([auxVector[0],auxVector[1],auxVector[2],auxVector[3]])

    newjoint.SetTRS(transVector, rotVector, sclVector)
    joints.append(newjoint)

    for i in range(0,node.GetChildCount()):
        getNodeCurves(node.GetChild(i), animLayer, j+1)
    if j==0:
        return joints




def curve2np(curves):
    #Transforma uma FbxAnimCurve em um np array de dimensões (x,y,z) nas colunas
    #e frames nas linhas.
    npCurve = None
    if len(curves)>0:
        totalKeys = max([curves[i].KeyGetCount() for i in range(0,len(curves))]);
        npCurve = np.empty([totalKeys,len(curves)])
        for curveCount in range(len(curves)):
            for keyCount in range(totalKeys):
                npCurve[keyCount,curveCount] = np.array(curves[curveCount].KeyGetValue(keyCount))
    return npCurve

def getLclTranslationCurve(node,animLayer):
    animCurve = node.LclTranslation.GetCurve(animLayer, "X")
    #Pega o numero de keys
    keysCount = animCurve.KeyGetCount()
    timeInterval = animCurve.KeyGetTime(keysCount-1)
    #print(timeInterval.GetSecondDouble())
    print(keysCount)
    for i in range(0, keysCount):
        animCurve.KeyModifyBegin()
        #print(animCurve.KeyGet(i).GetValue())
        animCurve.KeySet(i,animCurve.KeyGetTime(i),10.0)
        #print(animCurve.KeyGet(i).GetValue())
        animCurve.KeyModifyEnd()



filepath = r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\Superficie\TesteToque.fbx'
manager = fbx.FbxManager.Create()
importer = fbx.FbxImporter.Create(manager, 'myImporter')
status = importer.Initialize(filepath)

if status == False:
    print("Falha na importação.")
    filepath = r'C:\Users\Pichau\Dropbox\_Mestrado\MBTeste\Superficie\TesteToque.fbx'
    status = importer.Initialize(filepath)
    if status == False:
        print("Usando outro diretório")
        filepath = r'C:\Users\Rodolfo\Dropbox\_Mestrado\MBTeste\Superficie\TesteToque.fbx'
        status = importer.Initialize(filepath)
        if status == False:
            print("Não consegui importar.")
    else:
        print("Caminho secundário selecionado, arquivo encontrado.")
else:
    print("Arquivo encontrado.")




scene = fbx.FbxScene.Create(manager, 'myScene')

importer.Import(scene)

#Pega o numero de animation stacks do arquivo
#numstacks = importer.GetAnimStackCount()
#print(numstacks)

importer.Destroy()

evaluator = scene.GetAnimationEvaluator()

rootnode = scene.GetRootNode()

#print("Name: %s \nChildCount: %d" %(rootnode.GetName(), rootnode.GetChildCount()))
rootchild = rootnode.GetChild(0)
#print("Name: %s \nChildCount: %d\nClassId: %s" %(rootchild.GetName(), rootchild.GetChildCount(), rootchild.GetSkeleton()))

#Imprime a hierarquia do esqueleto
#printSkeleton(rootnode)

#Pega o ClassId do Animation Stack
animStackClassId = fbx.FbxAnimStack.ClassId

#Pega a quantidade de Animation Stacks do arquivo (da cena na verdade)
numStacks = scene.GetSrcObjectCount(fbx.FbxCriteria.ObjectType(animStackClassId))

for stack in range(numStacks):

    #Escolhe qual stack quer
    currentAnimationStack = scene.GetSrcObject(fbx.FbxCriteria.ObjectType(animStackClassId),stack)

    #Seta como stack atual
    scene.SetCurrentAnimationStack(currentAnimationStack)


    #Pega a quantidade de Animation Layers
    numAnimLayers = currentAnimationStack.GetMemberCount(fbx.FbxAnimLayer.ClassId)


    for animLayer in range(numAnimLayers):

        #Escolhe qual layer quer, no caso 0
        currentAnimLayer = currentAnimationStack.GetMember(fbx.FbxAnimLayer.ClassId, animLayer)

        joints = getNodeCurves(node = rootnode, animLayer = currentAnimLayer)
        dataToPlot = np.empty([0,0])
        for i in joints:
            #Checo para ver se é none, se não for deixa quieto (tratar depois)
            if i.translation.shape[0]>0:
                if dataToPlot.shape==(0,0):
                    count = 0
                    #Crio do tamanho de joints, mas tem umas que é None, ficar ligado
                    dataToPlot = np.empty([i.translation.shape[0],4,len(joints)])
                dataToPlot[:,:,count]=i.translation[:,:]
                count=count+1

        #print(i.translation[0,:])
        #print(np.min(dataToPlot[:,0,:]))

#        printSkeletonGlobals(rootchild)
        #plotanimation.AnimPlot(dataToPlot)
        print("Término da execução")























#exporter = fbx.FbxExporter.Create(manager, "")
#lFormatCount = manager.GetIOPluginRegistry().GetWriterFormatCount()
#for lFormatIndex in range(lFormatCount):
#    #if manager.GetIOPluginRegistry().WriterIsFBX(lFormatIndex):
#    lDesc = manager.GetIOPluginRegistry().GetWriterFormatDescription(lFormatIndex)
#    if "ascii" in lDesc:
#        pFileFormat = lFormatIndex
#result = exporter.Initialize("ExportedScript", pFileFormat, manager.GetIOSettings())
#if result == True:
#    result = exporter.Export(scene)
#exporter.Export(scene)
#exporter.Destroy()
