# -*- coding: utf-8 -*-
#http://help.autodesk.com/view/FBX/2019/ENU/
"""
Autor: Rodolfo Luis Tonoli
E-mail: rltonoli@gmail.com
Conversor FBX para BVH
Versão binária do FBX output padrão do Shogun Post

"""

import fbx
import sys
import numpy as np

class Joint:
    listofjoints = []
    def __init__(self, name, depth):
        self.name = name
        self.depth = depth
        self.listofjoints.append(name)
    def SetTRS(self, trans, rot, scl=[]):
        self.translation = trans
        self.rotation = rot
        self.scale = scl
    def SetLcl(self, lclTrans, lclRot, lclScl=[]):
        self.lclTrans = lclTrans
        self.lclRot = lclRot
        self.lclScl = lclScl

class BVHFile:
    files = []
    def __init__(self, name):
        self.name = name
        self.files.append(name)
        self.header = ["HIERARCHY"]
        self.content = []
        self.bvhJointList = []

    def AddJointList(self, name):
        self.bvhJointList.append(name)

    def AddToHeader(self, line):
        self.header.append(line)

    def AddToContent(self,line):
        self.content.append(line)

    def PrintFile(self):
        for line in self.header:
            print(line)
        for line in self.content:
            print(line)

    def SaveFile(self):
        with open(str.format("%s.bvh" % self.name), "w") as file:
            for line in self.header:
                file.write(line+"\n")
            for line in self.content:
                file.write(line+"\n")
        print("Arquivo salvo.")

def printSkeleton(node,j=0):
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

def getNodeCurves(scene, node, animLayer=0, j=0, joints=[], file=[], keysCount=[]):
    #TODO: Mudar nome da função
    #Prepara os dados para escrever o arquivo BVH

    #Pega o nome da junta e cadastra ela na classe Joints
    newjoint = Joint(node.GetName(), j)

    #Se for a primeira vez passando pela função (recursiva), começa a escrever
    #o header do bvh
    if j==0:
        bvhfile = BVHFile(str.format("File %i" % (len(BVHFile.files)+1)))
        bvhfile.AddToHeader(str.format("ROOT %s" % node.GetName()))
        bvhfile.AddJointList(newjoint)
    #Nos outros casos, continua escrevendo o header
    else:
        depth=j
        bvhfile=file
        #Verifica se tem pelo menos um filho OU se é o primeiro irmão
        if (node.GetChildCount() > 0) or (node.GetParent().GetChildCount() > 1):
            bvhfile.AddToHeader(str.format("%sJOINT %s" % (j*"\t",node.GetName())))
            bvhfile.AddJointList(newjoint)
        else: #Se não tiver filho nem irmão, "vira" End Site
            bvhfile.AddToHeader(str.format("%sEnd Site" % (j*"\t")))

    bvhfile.AddToHeader("%s{" % (j*"\t"))



    #Pega a translação e rotação inicial (não em função do frame) e cadastra
    #o OFFSET no BVH
    lclTrans = node.LclTranslation.Get()
    lclRot = node.LclRotation.Get()
    bvhfile.AddToHeader(str.format("%sOFFSET %.5f %.5f %.5f" % ((j+1)*"\t",lclTrans[0],lclTrans[1],lclTrans[2])))

    #Se não for End Site, define os canais
    if (node.GetChildCount() > 0) or (node.GetParent().GetChildCount() > 1):
        #if node.RotationOrder.Get()==0:
        #    bvhfile.AddToHeader(str.format("%sCHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation" % ((j+1)*"\t")))
        #else:
        bvhfile.AddToHeader(str.format("%sCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation" % ((j+1)*"\t")))
        #Se não for End Site e não tiver filho, adiciona End Site
        if (node.GetChildCount() == 0):
            bvhfile.AddToHeader(str.format("%sEnd Site" % ((j+1)*"\t")))
            bvhfile.AddToHeader("%s{" % ((j+1)*"\t"))
            bvhfile.AddToHeader(str.format("%sOFFSET %i %i %i" % ((j+2)*"\t",0,0,0)))
            bvhfile.AddToHeader("%s}" % ((j+1)*"\t"))

    #Cadastra valores de translação e rotação na classe Joints (não em função do frame)
    newjoint.SetLcl(np.array([lclTrans[0],lclTrans[1],lclTrans[2]]), np.array([lclRot[0],lclRot[1],lclRot[2]]))

    #Procura a quantidade de keys, usada para descobrir a quantidade de Frames
    #TODO: Achar um jeito melhor para encontrar a quantidade de Frames
    keyCount = 0
    for axes in ["X","Y","Z"]:
        transcurve = node.LclTranslation.GetCurve(animLayer,axes ,False)
        rotcurve = node.LclRotation.GetCurve(animLayer, axes,False)
        if transcurve:
            if transcurve.KeyGetCount()>keyCount:
                keyCount = transcurve.KeyGetCount()
        if rotcurve:
            if rotcurve.KeyGetCount()>keyCount:
                keyCount = rotcurve.KeyGetCount()
    keysCount.append(keyCount)

    #Inicia os vetores de translação, rotação e escala para cada frame (quantidade
    #de keys)
    transVector = np.empty(shape=(keyCount,4))
    rotVector = np.empty(shape=(keyCount,4))
    sclVector = np.empty(shape=(keyCount,4))

    time = fbx.FbxTime()

    #Para cada key (frame, supostamente) da animação do nó, pega os valores
    for i in range(keyCount):
        time.SetFrame(i, fbx.FbxTime.eFrames120)
        lclTransMatrix = node.EvaluateLocalTransform(time)
        auxVector = lclTransMatrix.GetT()
        transVector[i,:] = np.array([auxVector[0],auxVector[1],auxVector[2],auxVector[3]])
        auxVector = lclTransMatrix.GetR()
        #auxVector = node.EvaluateLocalRotation(time)
        rotVector[i,:] = np.array([auxVector[0],auxVector[1],auxVector[2],auxVector[3]])
        auxVector = lclTransMatrix.GetS()
        sclVector[i,:] = np.array([auxVector[0],auxVector[1],auxVector[2],auxVector[3]])

        # DEBBUG:
        #if (node.GetName().count('Spine 1') > 0) and i<5:
            #PARENTW =node.GetParent().EvaluateGlobalTransform (time)
            #WORLD = node.EvaluateLocalTransform (time)
            #TESTE = WORLD*PARENTW
            #quat = PARENTW.GetQ()
            #print(quat.DecomposeSphericalXYZ())
            #print("Parent")
            # print("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n" % (
            # PARENTW.Get(0,0),PARENTW.Get(1,0),PARENTW.Get(2,0),PARENTW.Get(3,0),
            # PARENTW.Get(0,1),PARENTW.Get(1,1),PARENTW.Get(2,1),PARENTW.Get(3,1),
            # PARENTW.Get(0,2),PARENTW.Get(1,2),PARENTW.Get(2,2),PARENTW.Get(3,2),
            # PARENTW.Get(0,3),PARENTW.Get(1,3),PARENTW.Get(2,3),PARENTW.Get(3,3),
            # ))
            # print("Transform")
            # print("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n" % (
            # WORLD.Get(0,0),WORLD.Get(1,0),WORLD.Get(2,0),WORLD.Get(3,0),
            # WORLD.Get(0,1),WORLD.Get(1,1),WORLD.Get(2,1),WORLD.Get(3,1),
            # WORLD.Get(0,2),WORLD.Get(1,2),WORLD.Get(2,2),WORLD.Get(3,2),
            # WORLD.Get(0,3),WORLD.Get(1,3),WORLD.Get(2,3),WORLD.Get(3,3),
            # ))
            # print("Teste")
            # print("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n" % (
            # TESTE.Get(0,0),TESTE.Get(1,0),TESTE.Get(2,0),TESTE.Get(3,0),
            # TESTE.Get(0,1),TESTE.Get(1,1),TESTE.Get(2,1),TESTE.Get(3,1),
            # TESTE.Get(0,2),TESTE.Get(1,2),TESTE.Get(2,2),TESTE.Get(3,2),
            # TESTE.Get(0,3),TESTE.Get(1,3),TESTE.Get(2,3),TESTE.Get(3,3),
            # ))
            # print("Rotation")
            # print(WORLD.GetR())
            # print(node.EvaluateLocalRotation(time))
            # print("---------------")
            #print("%f %f %f"%(TESTE[0],TESTE[1],TESTE[2]))
            #TESTE = scene.GetAnimationEvaluator().GetNodeLocalRotation(node,time)
            #TESTE = scene.GetAnimationEvaluator().GetNodeLocalTransform(node,time).GetR()
            #TESTE = node.EvaluateLocalRotation(time)
            #print(TESTE)
            #print("%f %f %f"%(TESTE[0],TESTE[1],TESTE[2]))

    #Cadas os valores TRS de cada frame do nó na class Joints
    newjoint.SetTRS(transVector, rotVector, sclVector)
    joints.append(newjoint)


    #Recursividade: para cada filho chama a mesma função
    for i in range(0,node.GetChildCount()):
        getNodeCurves(scene, node.GetChild(i), animLayer, j+1, file=bvhfile)

    #Encerra a participação do nó atual na escrita do header do BVH
    bvhfile.AddToHeader("%s}" % (j*"\t"))

    #Quando passar por toda a hierarquia, escreve o conteúdo do arquivo
    if j==0:
        bvhfile.AddToContent("MOTION")
        bvhfile.AddToContent(str.format("Frames: %i") % max(keysCount))
        time = fbx.FbxTime()
        #Seta 120 frames por segundo
        time.SetFrame(1, fbx.FbxTime.eFrames120)
        bvhfile.AddToContent(str.format("Frame Time: %f" % time.GetSecondDouble()))
        #Novamente assumo que a quantidade de Key é a mesma de Frame.
        #Uso a qtd de Key do nó que foi passado para essa função!
        print(keysCount)
        for i in range(max(keysCount)):
            listaAux = []
            jointsfromfile = []
            for joint in bvhfile.bvhJointList:
                if joint.rotation.shape[0] == 0:
                    listaAux= listaAux + [joint.lclTrans[0],joint.lclTrans[1],joint.lclTrans[2],0,0,0]
                else:
                    if i>=joint.rotation.shape[0]:
                        #listaAux=listaAux + [joint.translation[-1][0], joint.translation[-1][1], joint.translation[-1][2],joint.rotation[-1][0], joint.rotation[-1][1], joint.rotation[-1][2]]
                        listaAux=listaAux + [joint.translation[-1][0], joint.translation[-1][1], joint.translation[-1][2], joint.rotation[-1][2], joint.rotation[-1][0], joint.rotation[-1][1]]
                    else:
                        #listaAux=listaAux + [joint.translation[i][0], joint.translation[i][1], joint.translation[i][2],joint.rotation[i][0], joint.rotation[i][1], joint.rotation[i][2]]
                        listaAux=listaAux + [joint.translation[i][0], joint.translation[i][1], joint.translation[i][2], joint.rotation[i][2],joint.rotation[i][0], joint.rotation[i][1]]

            stringToAdd = " ".join(str.format("%.5f"%number) for number in listaAux)
            bvhfile.AddToContent(stringToAdd)


        return joints, bvhfile

#filepath = r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\CapturaErrosClean\Rodolfo\Rodolfo_Macarena.fbx'
#filepath = r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\simplefbx4.fbx'
filepath = r'C:\Users\Paula D. Paro Costa\Dropbox\_Mestrado\MBTeste\Superficie\Cabeca_1_export.fbx'
manager = fbx.FbxManager.Create()
importer = fbx.FbxImporter.Create(manager, 'myImporter')
status = importer.Initialize(filepath)
if status == False:
    print("Falha na importação.")
else:
    print("Arquivo encontrado.")

scene = fbx.FbxScene.Create(manager, 'myScene')
importer.Import(scene)
importer.Destroy()
evaluator = scene.GetAnimationEvaluator()
#Pega o nó root, nos arquivos .fbx esse nó pode não pertencer ao esqueleto, e
#sim à origem
rootnode = scene.GetRootNode()
#Pega o filho do root node, que seria o nó root do esqueleto. Aqui assume-se
#que existe apenas um esqueleto na cena. O script precisa ser adaptado caso
#existam mais de um esqueleto, se não pegará o primeiro, sejá lá qual for.
rootchild = rootnode.GetChild(0)
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
        joints, bvhfile = getNodeCurves(scene=scene, node = rootchild, animLayer = currentAnimLayer)

        #printSkeleton(rootchild)
        bvhfile.SaveFile()
