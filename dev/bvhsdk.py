import numpy as np
import plotanimation
import mathutils
from os.path import basename as getfilename
from os.path import join as pathjoin
import time
import skeletonmap


class Animation:

    def __init__(self, filename, root):
        self.name = filename
        self.root = root
        self.listofjoints = []
        self.surfaceinfo = []
        self.skeletonmap = []
        self.frames = None
        self.frametime = None

    def printlist(self):
        for joint in self.getlistofjoints():
            print(joint.name)

    def getlistofjoints(self):
        """
        Get list of joints in the animation
        """
        if not self.listofjoints:
            self.listofjoints = self.__auxgetlist(None, [])
        return self.listofjoints

    def __auxgetlist(self, joint=None, listofjoints=[], skip=[]):
        """
        Create and return list of joints in the animation

        skip: list of joint names to not include them and their children in the list.
        """
        if not joint:
            joint = self.root
        listofjoints.append(joint)
        for child in joint.children:
            if child.name not in skip:
                self.__auxgetlist(child, listofjoints, skip)
        if joint == self.root:
            return listofjoints

    def getskeletonmap(self, mapfile=None):
        """
        Pega o mapeamento do esqueleto. Procura juntas como hips, spine, arm,
        hand, leg, etc. na instância de animação comparando os nomes das juntas
        da animação com os nomes correspondentes no mapeamento descrito em skeletonmap
        """
        if not self.skeletonmap:
            self.skeletonmap = skeletonmap.SkeletonMap(self, mapfile)
        return self.skeletonmap

    def RootReferences(self):
        """
        Get references points for the root
        """
        root = self.root
        self.rootNormalizedReferencePoint = [[root.position[frame][0]/root.position[0][2], root.position[frame][1]/root.position[0][2]] for frame in range(len(root.position))]
        self.rootNormalizedHeight = [root.position[frame][2]/root.position[0][2] for frame in range(len(root.position))]
        return self.rootNormalizedReferencePoint, self.rootNormalizedHeight

    def getJoint(self, jointname):
        """
        Find the joint with jointname in the animation hierarchy
        """
        return self.root.getByName(jointname)

    def __erasepositions(self):
        for joint in self.getlistofjoints():
            joint.position=[]
            joint.orientation=[]
            joint.endsiteposition=[]

    def checkExtraRoot(self):
        """
        O esqueleto da Talita exportado em bvh pelo MotionBuilder possui uma
        junta root extra chamada 'Talita' que atrapalha o algoritmo e precisa
        ser removida. Ela não foi removida durante a exportação pois as
        distâncias das juntas (tamanho dos ossos) são normalizados.
        Aqui verifico se a junta root do bvh possui um dos nomes válidos.
        """
        # TODO: Função obsoleta. O usuário tem que se certificar que não existe juntas indesejadas
        # REMOVER
        if not skeletonmap.isroot(self.root.name):
            if not self.root.children:
                print('The animation seems to have an extra root joint without a child joint.')
                return None
            else:
                if type(self.root.children)==list:
                    for child in self.root.children:
                        child.parent = None
                    self.root = self.root.children[0]
                else:
                    self.root = self.root.children

    def RecalculatePositions(self):
        """
        Work in progress
        """
        self.__erasepositions()
        start = time.time()
        # if self.surfaceinfo:
        for frame in range(self.root.translation.shape[0]):
            if self.surfaceinfo:
                GetPositions(self.root, frame=frame, surfaceinfo=self.surfaceinfo)
            else:
                GetPositions(self.root, frame=frame)
            if np.mod(frame+1,100) == 0:
                print('%i frames done. %s seconds.' % (int((frame+1)/100)*100, time.time()-start))
                start = time.time()


    def checkPose(self):
        """
        Check the first frame of the animation for a valid T Pose, that is,
        hips facing the positive direction of the Z axis, the arms parallel to
        the X axis (left arm along positive X) and standing in the positive
        direction of Y.
        """
        root, head, lforearm, larm = None, None, None, None
        for joint in self.getlistofjoints():
            print(joint.name)
            if skeletonmap.isroot(joint.name):
                root = joint
            elif skeletonmap.ishead(joint.name):
                head = joint
            elif skeletonmap.isleftforearm(joint.name):
                lforearm = joint
            elif skeletonmap.isleftarm(joint.name):
                larm = joint
        if not (root or head or lforearm or larm):
            print('One or more joints could not be found')
            return None
        else:
            standvector = head.position[0]-root.position[0]
            print(standvector)
            larmvector = lforearm.position[0]-larm.position[0]
            print(larmvector)
            if max(standvector)!=standvector[1]:
                #if (np.abs(standvector[1]) < np.abs(standvector[0])) or (np.abs(standvector[1]) < np.abs(standvector[2]))
                #root.rotation=np.asarray([[0,0,0] for _ in range(root.rotation.shape[0])],dtype='float')
                self.RecalculatePositions()

    def PlotPose(self, frame=0, surface=None):
        """
        Plot the pose in the frame
        """
        if not surface:
            totaljoints = len(self.getlistofjoints())
            joints = np.zeros([totaljoints,3])
            for joint,i in zip(self.getlistofjoints(),range(totaljoints)):
                joints[i] = [joint.position[frame,0],joint.position[frame,1],joint.position[frame,2]]
            joints = np.asarray(joints)
            bones = self.getBones(frame)
            plotanimation.PosePlotBones(joints, bones)
        else:

            plotanimation.PlotPoseAndSurface(self, surface, frame)
#            totaljoints = len(self.getlistofjoints())
#            joints = np.zeros([totaljoints,3])
#            for joint,i in zip(self.getlistofjoints(),range(totaljoints)):
#                joints[i] = [joint.getPosition(frame)[0],joint.getPosition(frame)[1],joint.getPosition(frame)[2]]
#            joints = np.asarray(joints)
#            bones = self.getBones(frame)
#            plotanimation.PlotPoseAndSurface(self,surface, frame)
#
#            for triangle in surface.headmesh:
#                vertices = [[vert.getPosition(self,0)[0],vert.getPosition(self,0)[1],vert.getPosition(self,0)[2]] for vert in triangle]



    def PlotAnimation(self, surface=None):
        if surface:
            bones = []
            for frame in range(self.frames):
                bones.append(self.getBones(frame))
            bones = np.asarray(bones).transpose(1,2,0)
            listofpoints = []
            for point in surface.points:
                if point.pointtype == 'mesh':
                    listofpoints.append(point)
            plotanimation.AnimationSurface(self, bones, listofpoints)
        else:
            plotanimation.PlotBVH(self)

    def getBonesInFrames(self, *args):
        raise Exception('This method is no longer available, please use getBones()')


    def getBones(self, frame = 0, bonesPositions=[], joint=None):
        """
        Return the bones to plot
        """
        if joint == None:
            bonesPositions = []
            joint = self.root
        else:
            pp = joint.parent.getPosition(frame)
            cp = joint.getPosition(frame)
            bonesPositions.append([pp[0], pp[1], pp[2], cp[0], cp[1], cp[2]])
            if len(joint.endsite)>0:
                es = joint.getEndSitePosition(frame)
                bonesPositions.append([cp[0], cp[1], cp[2], es[0], es[1], es[2]])
        for child in joint.children:
            self.getBones(frame,bonesPositions,child)
        if joint == self.root:
            return np.asarray(bonesPositions)


    def plotPoseSurfFrame(self, surfaceinfo, frame=0):
        """
        Plot pose in the frame with surface points
        """
        totaljoints = len(self.getlistofjoints())
        joints = np.zeros([totaljoints,3])
        for joint,i in zip(self.getlistofjoints(),range(totaljoints)):
            joints[i] = [joint.position[frame,0],joint.position[frame,1],joint.position[frame,2]]
        joints = np.asarray(joints)
        bones = self.getBones(frame)

        #Getting surface info, testing if there is frames information or just
        #the points
        surface = []
        for point in surfaceinfo:
            if len(point.position)>0:
                surface.append(point.position[frame])
#            try:
#                if (point.position.shape[1] > 0):
#                    surface.append(point.position[frame])
#            except:
#                if (point.position.shape[0] == 3):
#                    surface.append(point.position[frame])

        plotanimation.PosePlotBonesSurface(joints, bones, np.asarray(surface))

    def expandFrames(self, frames, set_empty=False):
        """
        Expand the number of frames of the animation by coping the rotation and translation of frame zero several times.

        frames: desired amount of frames
        """
        self.frames = frames
        if set_empty:
            for joint in self.getlistofjoints():
                joint.translation = np.empty(shape=(frames, 3))
                joint.rotation = np.empty(shape=(frames, 3))
        else:
            for joint in self.getlistofjoints():
                joint.translation = np.asarray([joint.translation[0] for _ in range(frames)])
                joint.rotation = np.asarray([joint.rotation[0] for _ in range(frames)])



    def downSample(self, target_fps):
        current_fps = np.round(1/self.frametime)
        if target_fps == current_fps:
            print('Animation is already at the target frame rate.')
            return False
        elif target_fps > current_fps:
            print('Can\'t perform upsampling.')
            return False
        else:
            total_time = self.frames/current_fps
            target_frametime = 1/target_fps
            target_frames = int(np.floor(total_time/target_frametime))
            extra_frames = self.frames - target_frames
        step_to_remove = np.round(self.frames/extra_frames)
        indexes_to_remove = [i for i in range(1, self.frames, int(step_to_remove))]
        for joint in self.getlistofjoints():
            joint.rotation = np.delete(joint.rotation, indexes_to_remove, axis=0)
            joint.translation = np.delete(joint.translation, indexes_to_remove, axis=0)
        self.frames = joint.rotation.shape[0]
        self.frametime = target_frametime
        print(str.format('Animation downsampled to %i fps. %i frames.' % (target_fps, self.frames)))
        return True



    def MLPreProcess(self, skipjoints=[], root_translation=False, root_rotation=False, local_rotation=False):
        root_position = self.root.getGlobalTranslation(frame=0)
        list_of_joints = self.__auxgetlist(None, [], skip=skipjoints)
        dim1 = len(list_of_joints)  # Number of joints
        dim2 = 3  # Three values for position
        dim3 = self.frames  # Number of frames
        np_array = np.empty(shape=(dim1, dim2, dim3))
        if local_rotation:
            np_array_rot = np.empty(shape=(dim1, dim2, dim3))
        for frame in range(self.frames):
            # Center root
            if not root_translation:
                aux_trans = self.root.getLocalTranslation(frame)
                self.root.setTranslation(frame=frame, translation=np.array((0, 0, 0)))
            # Fix root rotation
            if not root_rotation:
                aux_rot = self.root.getLocalRotation(frame)
                self.root.setRotation(frame=frame, rotation=np.array((0, 0, 0)))
            np_array[:,:,frame] = np.asarray([joint.getPosition(frame) for joint in list_of_joints])
            if local_rotation:
                np_array_rot[:,:,frame] = np.asarray([joint.getLocalRotation(frame) for joint in list_of_joints])
            self.root.setTranslation(frame=frame, translation=aux_trans)
            self.root.setRotation(frame=frame, rotation=aux_rot)
        if not local_rotation:
            return np_array
        else:
            return np_array, np_array_rot
        #if lowerbody is False:
            #list_of_joints = [joint if ['leg', 'Leg', 'Foot', 'foot', 'Toe', 'toe'] not in ]
        #np_array = numpy.empty()
        #for joint in self.getlistofjoints():
        #    if joint is self.root:


        #TODO Include translation


class Joints:
#    listofjointsnames = []
#    listofjoints = []

    def __init__(self, name, depth=0, parent=None):
        self.name = name
        self.depth = depth
        self.children = []
        self.parent = parent
        self.endsite = []
        self.translation = []
        self.rotation = []
        self.order = []
        self.n_channels = 0
        self.position = []
        self.orientation = []
        self.egocoord = []
        self.length = []
        self.baserotation = []
        self.tposerot = []
        self.tposetrans = []
        if self.parent:
            self.parent.addChild(self)
        #variável para debbugar o retargeting
        self.frameswarning = []

        #Check when it is needed to recompute
        self.flagGlobalTransformComputed = None
        self.currentGlobalTransform = []

    def __iter__(self):
        for child in self.children:
            yield child

    def getChildren(self, index=-1):
        """
        Returns a list of references to the childrens of the joint. If index is equal or bigger
        than 0, return the index-th child

        :type index: int
        :param index: Index-th child to be returned
        """
        if not self.children:
#            print('The joint %s does not have children.' % self.name)
            return []
        else:
            if index >= 0:
                try:
                    if len(self.children) < index:
                        print('Index greater than the amount of children.')
                        return None
                    else:
                        return self.children[index]
                except:
                    print('Something went wrong in readbvh->getChildren(%s, %i)' %(self.name, index))
                    return None
            else:
                return self.children

    def getJointsBelow(self, first=True):
        """
        Generator for all joints below the hierarchy
        Parameters
        ----------
        first : bool, optional
            Internal control, do not change. The default is True.

        Yields
        ------
        Joint
            joints below the hierarchy.

        """
        for child in self:
            yield child
            yield from child.getJointsBelow(first=False)

    def __reversed__(self):
        while self.parent:
            yield self.parent
            self = self.parent

    def pathFromRootToJoint(self):
        # path = list(reversed([joint for joint in reversed(self)]))
        path = ([joint for joint in reversed(self)])[::-1]
        for joint in path:
            yield joint
        yield self

    def pathFromDepthToJoint(self, depth=-1):
        """
        Generator of the path between the joint located depth nodes up the hierarchy to the joint.

        Example: Given the subtree node1, node2, node3, node4, node5 and node6. node5.pathFromDepthToJoint(2) will
        return the joints node3, node4 and node5.

        :type depth: int
        :param depth: Position above the current joint in the hierarchy
        """
        # path = list(reversed([joint for joint in reversed(self)]))
        path = ([joint for joint in reversed(self)])[::-1]
        if depth > len(path) or depth==-1:
            depth = len(path)
        path = path[-depth:]
        for joint in path:
            yield joint
        yield self

    def pathFromJointToJoint(self, parentjoint):
        """
        Returns the kinematic path between a parentjoint up in the hierarchy to the
        current joint. If they are not in the same kinematic path (same subtree), raises error

        Returns a list, not a generator!
        """
        path = []
        found = False
        for joint in self.pathFromRootToJoint():
            if joint == parentjoint or found:
                path.append(joint)
                found = True
        if not found:
            print('Warning at mathutils.pathFromJointToJoint(): the joints are not in the same subtree.')
            return None
        return path

    def getRoot(self):
        return [joint for joint in reversed(self)][-1]

    def addChild(self, item):
        """
        Called after initialization of every joint except root

        self: the parent joint to add child
        item: the child joint initialized
        """
        item.parent = self
        self.children.append(item)

    def addOffset(self, offset):
        self.offset = offset

    def addEndSite(self, endsite=None):
        self.endsite = endsite
        self.endsiteposition = []

    def __addBoneLength(self, bonelength):
        """
        The BVH file format does not inform the length of the bone directly.
        The length can be calculated using the offset of its child, but in the
        case that the joint has multiple children, the first one is used.
        """
        self.length = bonelength

    def addPosition(self, pos, frame):
        """
        In the first time, instantiate the global position variable of a joint.
        Then fill in values at the frame provided

        self: joint to fill position
        pos: position of the joint in the frame
        frame: current frame
        """
        if len(self.position)==0:
            totalframes = self.translation.shape[0]
            self.position = np.zeros([totalframes,3])
            if len(self.endsite) > 0:
                self.endsiteposition = np.zeros([totalframes,3])
        self.position[frame,:] = pos.ravel()

    def addOrientation(self, ori, frame):
        """
        In the first time, instantiate the global orientation variable of a joint.
        Then fill in values at the frame provided

        self: joint to fill orientation
        ori: orientation of the joint in the frame
        frame: current frame
        """
        if len(self.orientation)==0:
            totalframes = self.translation.shape[0]
            self.orientation = np.zeros([totalframes,3])
        self.orientation[frame,:] = ori.ravel()

    def addEndSitePosition(self, pos, frame):
        self.endsiteposition[frame,:] = pos.ravel()

    def getLocalTransform(self, frame=0):
        """
        Get joint local transform
        """
        trans = self.getLocalTranslation(frame)
        rot = self.getLocalRotation(frame)
        rotx = mathutils.matrixRotation(rot[0],1,0,0)
        roty = mathutils.matrixRotation(rot[1],0,1,0)
        rotz = mathutils.matrixRotation(rot[2],0,0,1)
        transform = mathutils.matrixTranslation(trans[0], trans[1], trans[2])
        if self.order == "ZXY":
            transform = np.dot(transform, rotz)
            transform = np.dot(transform, rotx)
            transform = np.dot(transform, roty)
        elif self.order == "XYZ":
            transform = np.dot(transform, rotx)
            transform = np.dot(transform, roty)
            transform = np.dot(transform, rotz)
        return transform

    def getLocalTransformBaseRotation(self, frame):
        #OBSOLETE
        print('Do not use this function!')
        localRotMatrix = mathutils.expandShape3ToShape4(self.getRecalcRotationMatrix(frame))
        translation = mathutils.matrixTranslation(0, self.getLength(), 0)
        localTransform = mathutils.matrixMultiply(translation, localRotMatrix)
        #return localTransform
        return None

    def __getCurrentGlobalTransform(self):
        return self.currentGlobalTransform

    def __setCurrentGlobalTransform(self, globalTransform, frame):
        self.flagGlobalTransformComputed = frame
        self.currentGlobalTransform = globalTransform

    def getGlobalTransform(self, frame=0, includeBaseRotation=False):
        """
        Get joint global transform
        """
        #if the global transform of this joint was already computed for this frame return it
        if self.flagGlobalTransformComputed != frame:
            #Get the hierarchy from joint to root and reverse it
            # tree = list(reversed([joint for joint in reversed(self)]))
            # tree.append(self)
            #parentTransform = np.empty(0)
            for joint in self.pathFromRootToJoint():
                #if the global transform of this joint was already computed for this frame, move on to the next
                if joint.flagGlobalTransformComputed == frame:
                    pass
                else:
                    if includeBaseRotation:
                        transform = joint.getLocalTransformBaseRotation(frame)
                    else:
                        transform = joint.getLocalTransform(frame)
                    if joint.parent:
                        #If this joint is not the root, multiply its local transform by its parent global transform
                        transform = np.dot(joint.parent.__getCurrentGlobalTransform(),transform)
                    #parentTransform = np.copy(transform)
                    joint.__setCurrentGlobalTransform(np.copy(transform), frame)

            if np.sum(transform, axis=1)[3] > 1:
                print('getGlobalTransform: Something is wrong. Pleas check this case')
        return self.__getCurrentGlobalTransform()

    def getLocalRotation(self, frame=0):
        return self.rotation[frame]

    def getGlobalRotation(self, frame=0,includeBaseRotation=False):
        return mathutils.eulerFromMatrix(self.getGlobalTransform(frame, includeBaseRotation), self.order)

    def getLocalTranslation(self, frame=0):
        try:
            return self.translation[frame]
        except:
            return self.translation[0]

    def getGlobalTranslation(self, frame=0):
        """
        Doesn't include base rotation
        """
        transform = self.getGlobalTransform(frame)
        return np.asarray([transform[0,3],transform[1,3],transform[2,3]])

    def getPosition(self, frame=0):
        # return np.dot(self.getGlobalTransform(frame, False), [0,0,0,1])[:-1]
        return self.getGlobalTransform(frame, False)[:-1,-1]

    def setRotation(self, frame, rotation):
        """
        Updates rotation angles in the current frame and sets all global transform matrices down the hierarchy to
        not reliable.
        Parameters
        ----------
        frame : int
            Current frame.
        rotation : numpy.ndarray
            Rotation angles.

        Returns
        -------
        None.

        """
        self.flagGlobalTransformComputed = None
        for joint in self.getJointsBelow():
            joint.flagGlobalTransformComputed = None
        self.rotation[frame] = rotation

    def setTranslation(self, frame, translation):
        """
        Updates translation values in the current frame and sets all global transform matrices down the hierarchy to
        not reliable.
        Parameters
        ----------
        frame : int
            Current frame.
        translation : numpy.ndarray
            Rotation angles.

        Returns
        -------
        None.

        """
        self.flagGlobalTransformComputed = None
        for joint in self.getJointsBelow():
            joint.flagGlobalTransformComputed = None
        self.translation[frame] = translation

    def getEndSitePosition(self, frame=0):
        if len(self.endsite) == 0:
            print('Unable to get joint\'s EndSite at readbvh.getEndSitePosition(joint, frame) because the joint %s does not have an EndSite.' % self.name)
            return None
        transform = mathutils.matrixTranslation(self.endsite[0], self.endsite[1], self.endsite[2])
        transform = np.dot(self.getGlobalTransform(frame),transform)
        # return np.dot(transform,[0,0,0,1])[:-1]
        return np.asarray(transform[:-1,-1])

    def getByName(self, name):
        """
        Returns the joint object with the provided name

        self: first joint in the hierarchy
        name: name of the joint
        """
        if self.name == name:
            return self
        for child in self.children:
            if child.name == name:
                return child
            else:
                found = child.getByName(name)
                if found:
                    return found

    def getDepth(self):
        depth = 0
        joint = self
        if joint.parent:
            while joint.parent:
                depth = depth + 1
                joint = joint.parent
        return depth

    def getLastDepth(self, depth, jointsInDepth = []):
        """
        Returns the last joint initializated with the depth provided

        self: root of the hierarchy
        depth: hierarchy level
        jointsInDepth: list of joints at depth level
        """
        if depth==0:
            return self
        else:
            for child in self.children:
                if child.depth == depth:
                    jointsInDepth.append(child)
                child.getLastDepth(depth, jointsInDepth)
            return jointsInDepth[-1]

    def getLength(self):
        """
        Returns the length of the bone (distance between the joint to its first
        child)
        """
        if not self.length:
            if len(self.endsite)>0:
                value = np.linalg.norm(self.endsite)
            else:
                value = np.linalg.norm(self.getChildren(0).offset)
            self.__addBoneLength(value)
        return self.length

    def getBaseRotation(self):
        """
        DON'T USE (it's not necessary)

        Returns the base rotation matrix of the bone associated with this joint.
        The base rotation matrix rotates the vector [0 length 0] to the offset
        vector. "length" is computed through self.getLength and "offset" through
        self.offset.

        Motion Capture File Formats Explained. M. Meredith S.Maddock
        """
        if len(self.baserotation)==0:
            self.getLength()
            try:
                euler, warning = mathutils.eulerFromMatrix(mathutils.alignVectors(self.getChildren(0).offset,[0,1.0,0]), self.order)
            except:
                print('Endsite used in getBaseRotation(joint)')
                euler, warning = mathutils.eulerFromMatrix(mathutils.alignVectors(self.endsite,[0,1.0,0]), self.order)
            if warning:
                print('Instability found in getBaseRotation(%s)' % self.name)
            self.baserotation = euler
        return self.baserotation

    def getRecalcRotationMatrix(self, frame):
        """
        ESSA FUNçÂO NÂO DEVE SER UTILIZADA
        Returns the recalculated local rotation matrix

        Equation 4.9 from Motion Capture File Formats Explained. M. Meredith S.Maddock
        """
        #TODO: Eu preciso de um jeito de calcular a rotação global com a rotação base
        # pq minha rotação global atual não leva em conta a base, então pode estar
        # apontando para o lugar errado
        #TODO: Função obsoleta, testei e não deu certo, fui por outro caminho.
        #ESSA FUNçÂO NÂO DEVE SER UTILIZADA

        #Get the hierarchy from joint to root
        tree_parentjointTOroot = list([joint for joint in reversed(self)])
        stack = mathutils.matrixIdentity()

        for joint in tree_parentjointTOroot:
            #Creates base matrix
            try:
                basematrix = mathutils.alignVectors(joint.getChildren(0).offset,[0,1.0,0]).T
            except:
                print('Endsite used in getBaseRotation(joint)')
                basematrix = mathutils.alignVectors(joint.endsite,[0,1.0,0]).T
            stack = np.dot(stack, basematrix)

        matrix = np.dot(stack, mathutils.matrixR(self.getLocalRotation(frame), self.order))
        stack = stack.T
        try:
            basematrix = mathutils.alignVectors(self.getChildren(0).offset,[0,1.0,0])
        except:
            print('Endsite used in getBaseRotation(joint)')
            basematrix = mathutils.alignVectors(self.endsite,[0,1.0,0])
        stack = np.dot(stack, basematrix)
        matrix = np.dot(matrix, stack)
        return matrix


    def printHierarchy(self, hierarchy=[]):
        """
        Print hierarchy below self

        self: first joint in the hierarchy
        hierarchy: formatted hierarchy list
        """
        flag = False
        if len(hierarchy)==0:
            flag = True
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
        if flag:
            return hierarchy

    #TODO: testar e verificar se é necessário
    def PoseBottomUp(self, value=np.zeros(3)):
        aux = np.asarray([float(number) for number in self.offset])
        value = value + aux
        if self.parent:
            value = self.parent.PoseBottomUp(value)
        return value




    def GetBones(self, *args):
        raise Exception('This method is no longer available, please call getBones() from the Animation instance')
#    def GetBones(self, bonesPositions=[], count=[], joint=None):
#        """
#        Returns an array to plot the animation using lines for bones and scatter
#        to represent the joints.
#
#        axis 0: each line of the array contains eigth values to construct a line: x1, y1, z1, 1.0, x2, y2, z2, 1.0
#        axis 2: each 2x2 matrix corresponds to a frame
#
#        TODO: Excluir os 1.0 daqui e do plotanimation
#        """
#        numberofframes = self.position.shape[0]
#        if len(bonesPositions)==0:
#            numberofbones = len(self.printHierarchy(hierarchy=[]))-1
#            bonesPositions = np.zeros([numberofbones, 8, numberofframes])
#            #Get root
#            joint = self
#        else:
#            pp = joint.parent.position.T
#            cp = joint.position.T
#            bonesPositions[len(count),:,:] = np.asarray([[pp[0,frame], pp[1,frame], pp[2,frame], 1.0, cp[0,frame], cp[1,frame], cp[2,frame],1.0] for frame in range(numberofframes)]).T
#            count.append([])
#            if len(joint.endsite)>0:
#                es = joint.endsiteposition.T
#                bonesPositions[len(count),:,:] = np.asarray([[cp[0,frame], cp[1,frame], cp[2,frame], 1.0, es[0,frame], es[1,frame], es[2,frame],1.0] for frame in range(numberofframes)]).T
#                count.append([])
#
#        for child in joint.children:
#            self.GetBones(bonesPositions,count,child)
#        return bonesPositions




def WriteBVH(animation, path, name='_export', frametime = 0.00833333, refTPose = True, writeTranslation=True):
    """
    Create a bvh file with the motion contained in the animation.

    :type animation: pyanimation.Animation
    :param animation: Animation containing the motion

    :type path: str
    :param path: Full path to save the file

    :type name: str
    :param name: Filename

    :type frametime: float
    :param frametime: 1/(Frame per second), the time duration of each frame

    :type refTPose: bool
    :param refTPose: If True, the first frame of the animation is the input TPose reference

    :type writeTranslation: bool
    :param writeTranslation: If True, write translations for every joint. If False, only write translation for the root joint
    """
    path = pathjoin(path, name)
    endsiteflag = False
    depth = 0
    with open(str.format("%s.bvh" % path), "w") as file:
        file.write('HIERARCHY\n')
        for section in ['header', 'content']:
            if section == 'header':
                for joint in animation.getlistofjoints():
                    if joint==animation.root:
                        file.write(str.format('ROOT %s\n' % joint.name))
                        file.write('{\n')
                        file.write(str.format('\tOFFSET %.5f %.5f %.5f\n' % (joint.offset[0],joint.offset[1],joint.offset[2])))
                        if joint.order == 'XYZ':
                            file.write(str.format("\tCHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation\n"))
                        elif joint.order == 'ZXY':
                            file.write(str.format("\tCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n"))

                    else:
                        if endsiteflag:
                            endsiteflag = False
                            next_depth = joint.getDepth()
                            while depth >= next_depth:
                                file.write('%s}\n' % ((depth)*'\t'))
                                depth = depth-1
                        depth = joint.getDepth()
                        file.write(str.format('%sJOINT %s\n' % (depth*'\t', joint.name)))
                        file.write('%s{\n' % (depth*'\t'))
                        file.write(str.format('%sOFFSET %.5f %.5f %.5f\n' % ((depth+1)*'\t',joint.offset[0],joint.offset[1],joint.offset[2])))
                        if writeTranslation:
                            aux_string = str.format("%sCHANNELS 6 Xposition Yposition Zposition " % ((depth+1)*"\t"))
                        else:
                            aux_string = str.format("%sCHANNELS 3 " % ((depth+1)*"\t"))
                        if joint.order == 'XYZ':
                            file.write(aux_string + "Xrotation Yrotation Zrotation\n")
                        elif joint.order == 'ZXY':
                            file.write(aux_string + "Zrotation Xrotation Yrotation\n")
                        else:
                            print('Order is not implemented')
                        if len(joint.endsite) > 0:
                            endsiteflag = True
                            file.write(str.format('%sEnd Site\n' % ((depth+1)*'\t')))
                            file.write('%s{\n' % ((depth+1)*'\t'))
                            file.write(str.format('%sOFFSET %.5f %.5f %.5f\n' % ((depth+2)*'\t',joint.endsite[0],joint.endsite[1],joint.endsite[2])))
                            file.write('%s}\n' % ((depth+1)*'\t'))
            elif section == 'content':
                while depth > 0:
                    file.write('%s}\n' % ((depth)*'\t'))
                    depth = depth-1
                file.write('}\n')
                file.write(str.format('MOTION\n'))
                totalframes = animation.root.translation.shape[0]
                #Check if the TPose should be added and check if at least the root has this information
                #It assumes that if the root has this information, so do the other joints

                if refTPose == True and len(animation.root.tposerot)>0 and len(animation.root.tposetrans)>0:
                     totalframes = totalframes + 1
                else:
                    refTPose = False
                file.write(str.format('Frames: %i\n' % totalframes))
                file.write(str.format('Frame Time: %.5f\n' % frametime))

                # Write the reference TPose line
                if refTPose == True:
                    line = []
                    for joint in animation.getlistofjoints():
                        if writeTranslation or joint==animation.root:
                            line = line + [joint.tposetrans[0], joint.tposetrans[1], joint.tposetrans[2]]
                        if joint.order=='XYZ':
                            line = line + [joint.tposerot[0], joint.tposerot[1], joint.tposerot[2]]
                        elif joint.order=='ZXY':
                            line = line + [joint.tposerot[2], joint.tposerot[0], joint.tposerot[1]]
                    string = " ".join(str.format("%.2f"%number) for number in line)
                    file.write(string+'\n')

                #Write the rest of the file
                for frame in range(animation.root.translation.shape[0]):
                    line = []
                    for joint in animation.getlistofjoints():
                        if writeTranslation or joint==animation.root:
                            line = line + [joint.translation[frame,0], joint.translation[frame,1], joint.translation[frame,2]]
                        if joint.order=='XYZ':
                            line = line + [joint.rotation[frame,0], joint.rotation[frame,1], joint.rotation[frame,2]]
                        elif joint.order=='ZXY':
                            line = line + [joint.rotation[frame,2], joint.rotation[frame,0], joint.rotation[frame,1]]
                    string = " ".join(str.format("%.2f"%number) for number in line)
                    file.write(string+'\n')
    print('File Saved: %s' % (path+'.bvh'))


def GetBVHDataFromFile(path, skipmotion=False):
    """
    Read a bvh file.

    :type path: string or path
    :param path: Complete path to the bvh file

    :type skipmotion: bool
    :param skipmotion: Whether to read the motion of the file (False) or to
    read only the skeleton definition.

    :rtype bvhfile: Animation
    :rparam bvhfile: An Animation object containing the information from the
    bvh file.
    """
    # TODO: Account for BVH files without translation
    frame = 0
    bvhfile = None
    with open(path) as file:
        flagEndSite = False
        flagMotionDataBegin = False
        for line in file:
            if not flagMotionDataBegin:
                # Starts here (first joint)
                if line.find("ROOT") >= 0:
                    # Creates root joint
                    root = Joints(name=line[5:-1])
                    lastJoint = root

                    # Create the Animation object of this file
                    filename = getfilename(path)[:-4]
                    bvhfile = Animation(filename, root)

                # Every other joint goes through here
                # Identention should be tabular or with pairs of spaces
                elif line.find("JOINT") >= 0:
                    depth = line.count('\t')
                    if depth == 0:
                        depth = line[:line.find('JOINT')].count(' ')/2
                    parent = root.getLastDepth(depth-1)
                    # Creates joint
                    lastJoint = Joints(name=line[line.find("JOINT")+6:-1],
                                       depth=depth, parent=parent)

                elif line.find("End Site") >= 0:
                    flagEndSite = True

                elif (line.find("OFFSET") >= 0) and (not flagEndSite):
                    lastJoint.addOffset(np.asarray(line[line.find("OFFSET")+7:-1].split(' '),float))
                elif (line.find("OFFSET") >= 0) and (flagEndSite):
                    lastJoint.addEndSite(np.asarray(line[line.find("OFFSET")+7:-1].split(' '),float))
                    flagEndSite = False

                elif (line.find("CHANNELS")) >= 0:
                    lastJoint.n_channels = int(line[line.find("CHANNELS")+9])
                    if lastJoint.n_channels != 6:
                        print("Number of channels must be 6")
                        raise ValueError
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

                elif (line.find("Frames")) >= 0:
                    bvhfile.frames = int(line[8:])
                    for joint in bvhfile.getlistofjoints():
                        joint.translation = np.empty(shape=(bvhfile.frames, 3))
                        joint.rotation = np.empty(shape=(bvhfile.frames, 3))
                elif (line.find("Frame Time")) >= 0:
                    bvhfile.frametime = float(line[12:])
                    flagMotionDataBegin = True
                else:
                    # Lines only with { and } pass through here
                    pass
            elif flagMotionDataBegin and not skipmotion:
                line = [float(item) for item in line.replace('\n', '').split(' ') if item]
                for i, joint in enumerate(bvhfile.getlistofjoints()):
                    joint.translation[frame] = np.asarray(line[i*6:i*6+3],
                                                          float)
                    joint.rotation[frame] = np.asarray(line[i*6+3:i*6+6],
                                                       float)
                frame = frame + 1

    # Check channels' order
    if not skipmotion:
        for joint in bvhfile.getlistofjoints():
            if joint.order == "ZXY":
                # joint.rotation = joint.rotation[:, [2, 0, 1]]
                joint.rotation = joint.rotation[:, [1, 2, 0]]

    return bvhfile

def GetPositions(joint, frame=0, parentTransform=[], surfaceinfo=None, calibrating=None):
    # Recebe o frame e a junta root, recursivamente calcula a posição de todos
    # os filhos para esse frame
    # Salva a posição em cada frame dentro de cada instância junta

    # Caso precise recalcular as posições das juntas, os dados antigos precisam
    # ser apagados

    #TODO: Orientation não significa nada, arrumar

    rot = joint.rotation[frame]
    transform = joint.getLocalTransform(frame)

    if len(parentTransform) == 0:
        #Se for root apenas calcula a posição
        positionfinal = np.dot(transform, [0,0,0,1])
        orientation = np.asarray([rot[0], rot[1], rot[2]])
    else:
        #Nos outros casos, multiplica pela transformada da junta pai
        transform = np.dot(parentTransform,transform)
        positionfinal = np.dot(transform,[0,0,0,1])
        orientation = joint.parent.orientation[frame,:] + np.asarray([rot[0], rot[1], rot[2]])

    if not calibrating:
        joint.addPosition(np.asarray(positionfinal[:-1]), frame)
        joint.addOrientation(orientation, frame)

    #Caso a junta tenha um endsite (é um end effector)
    if len(joint.endsite)>0:
        ee_transform = mathutils.matrixTranslation(joint.endsite[0], joint.endsite[1], joint.endsite[2])
        ee_transform = np.dot(transform,ee_transform)
        endsitepos = np.dot(ee_transform,[0,0,0,1])
        if not calibrating:
            #Salva a posição do endsite da junta no frame atual
            joint.addEndSitePosition(np.asarray(endsitepos[:-1]), frame)

    #if there is surface information and a surface point or limb is attached
    #to this joint, calculate the surface point or limb position
    if surfaceinfo:
        for point in surfaceinfo.points:
            if point.jointlock == joint.name:
                if len(point.calibrationLocalTransform)>0:
                    local_transform = np.dot(transform,point.calibrationLocalTransform)
                    point.position.append(np.dot(local_transform,[0,0,0,1]))

    # Get calibration local transform of the actor surface points for the mocap animation
    if calibrating:
        point = calibrating
        if skeletonmap.isamatch(point.jointlock,joint.name):
            if point.pointtype == 'mesh':
                globalTransform = mathutils.matrixTranslation(point.calibrationPosition[0], point.calibrationPosition[1], point.calibrationPosition[2])
                parentInverse = mathutils.inverseMatrix(transform)
                point.calibrationLocalTransform = np.dot(parentInverse,globalTransform)
            if point.pointtype == 'limb':
#                TODO: testar esse caso!!!
                p1 = np.dot(transform,[0,0,0,1])[:-1]
#                aux_transform = np.dot(parentTransform,joint.getChildren(0).getLocalTransform(frame))
                aux_transform = np.dot(transform,joint.getChildren(0).getLocalTransform(frame))
                p2 = np.dot(aux_transform,[0,0,0,1])[:-1]
                #calibration point = p0
                p0 = np.asarray(point.calibrationPosition)
                #distance between point p0 and line passing through p1 and p2:
                # d = |(p0 - p1) x (p0 - p2)|/|p2-p1| = |(p0p1) x (p0p2)|/|p2p1|
                p0p1 = p0-p1
                p0p2 = p0-p2
                p2p1 = p2-p1
                d = np.linalg.norm(np.cross(p0p1,p0p2))/np.linalg.norm(p2p1)
                point.radius = d


    parentTransform = np.copy(transform)
    for child in joint.children:
        GetPositions(child, frame, parentTransform, surfaceinfo, calibrating)
    parentTransform=[]


def ReadFile(path, surfaceinfo=None):
    """
    Read BVH file, create animation a joints instances and calculate joint positions
    """
    animation = GetBVHDataFromFile(path)
    animation.surfaceinfo = surfaceinfo

    #for frame in range(animation.frames):
    #    GetPositions(animation.root, frame=frame, surfaceinfo=surfaceinfo)

    return animation

#animation = ReadFile('./teste.bvh')
