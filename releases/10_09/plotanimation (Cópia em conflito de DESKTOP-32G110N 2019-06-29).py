import numpy as np
import matplotlib.pyplot as plt
import mathutils

from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

def AnimPlot(data):

    def update(frame):
        xdata, ydata, zdata = [], [], []
        #print("-------------------Frame: %f"%(frame))
        #xdata.append(data[frame,0,:])
        #ydata.append(data[frame,1,:])
        #zdata.append(data[frame,2,:])

        xdata = data[:,0,frame]
        ydata = data[:,1,frame]
        zdata = data[:,2,frame]
        ln.set_data(xdata, ydata)
        ln.set_3d_properties(zdata)
        return ln,

    def animate():
        #ax.set_xlim(np.min(data[:,0,:]),np.max(data[:,0,:]))
        #ax.set_xlim(np.min(data[:,2,:]),np.max(data[:,2,:]))
        #ax.set_zlim(np.min(data[:,1,:]),np.max(data[:,1,:]))
        mini,maxi = np.min(data), np.max(data)
        ax.set_xlim(mini,maxi)
        ax.set_ylim(mini,maxi)
        ax.set_zlim(mini,maxi)
        ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), interval=10,
                             blit=True)

        plt.show()


    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
    xdata, ydata, zdata = [], [], []
    ln, = plt.plot([], [], 'ro', animated=True)
    animate()

def AnimPlotBones(data):

    def update(frame, lines, data):
        print(frame)
        for line, dataBone in zip(lines, data):
            x = np.asarray([dataBone[0,frame], dataBone[4,frame]])
            y = np.asarray([dataBone[1,frame], dataBone[5,frame]])
            z = np.asarray([dataBone[2,frame], dataBone[6,frame]])
            line.set_data(x,y)
            line.set_3d_properties(z)
        return lines

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    lines = []
    for i in range(len(data)):
        lines.append(ax.plot([data[i,0,0], data[i,4,0]], [data[i,1,0], data[i,5,0]], [data[i,2,0], data[i,6,0]],'-o', c='black')[0])


    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(data), np.max(data)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), fargs=(lines, data),interval=2,
                             blit=True)

    plt.show()


def AnimPlotBones2D(data, plotax='xy'):

    def update(frame, lines, data, ax1, ax2):
        for line, dataBone in zip(lines, data):
            x = np.asarray([dataBone[ax1[0],frame], dataBone[ax1[1],frame]])
            y = np.asarray([dataBone[ax2[0],frame], dataBone[ax2[1],frame]])
            line.set_data(x,y)
        return lines

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111)

    choices = {'xy': ([0,4], [1,5]),
               'xz': ([0,4], [2,6]),
               'yx': ([1,5], [0,4]),
               'yz': ([1,5], [2,6]),
               'zx': ([2,6], [0,4]),
               'zy': ([2,6], [1,5])}


    ax1, ax2 = choices.get(plotax, ([0,4],[1,5]))


    lines = []
    for i in range(len(data)):
        lines.append(ax.plot([data[i,ax1[0],0], data[i,ax1[1],0]], [data[i,ax2[0],0], data[i,ax2[1],0]],'-o')[0])


    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    mini,maxi = np.min(data), np.max(data)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), fargs=(lines, data, ax1, ax2),interval=100,
                             blit=True)

    plt.show()


def JointVelocity(data, box=0):
    x,y,z = data.shape
    lines = np.zeros([x,1,z-1])
    for jointCount in range(x):
        for frameCount in range(z-1):
            deltaXsquare = np.square(data[jointCount,0,frameCount+1]-data[jointCount,0,frameCount])
            deltaYsquare = np.square(data[jointCount,1,frameCount+1]-data[jointCount,1,frameCount])
            deltaZsquare = np.square(data[jointCount,2,frameCount+1]-data[jointCount,2,frameCount])
            lines[jointCount, 0, frameCount] = np.sqrt(deltaXsquare + deltaYsquare + deltaZsquare)

    fig = plt.figure(figsize=(12,8))

    for joint in range(x):
        plt.plot(lines[joint, 0, :])
    plt.show()


def PosePlot(data, label=[]):
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
    xdata = data[:,0]
    #ydata = -data[:,2]
    #zdata = data[:,1]
    ydata = data[:,1]
    zdata = data[:,2]
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.scatter(xdata,ydata,zdata)
    if label:
        for i in range(len(label)): #plot each point + it's index as text above
            ax.text(data[i,0],-data[i,2],data[i,1],  '%s' % (label[i]), size=8, zorder=1, color='k')

def PosePlotBones(joints,bones):
    """
    Plot a pose (just one frame) with joints and bones
    """
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(joints), np.max(joints)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ax.scatter(joints[:,0],joints[:,1],joints[:,2])
    for i in range(len(bones)):
        ax.plot([bones[i,0], bones[i,3]], [bones[i,1], bones[i,4]], [bones[i,2], bones[i,5]], color='black')

def PosePlotBonesSurface(joints,bones,surface):
    """
    Plot a pose (just one frame) with joints, bones and surface data
    """
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(joints), np.max(joints)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ax.scatter(joints[:,0],joints[:,1],joints[:,2])
    ax.scatter(surface[:,0], surface[:,1], surface[:,2], s=40,c='r',marker = '^')
    for i in range(len(bones)):
        ax.plot([bones[i,0], bones[i,3]], [bones[i,1], bones[i,4]], [bones[i,2], bones[i,5]], color='black')
    plt.show()


def AnimationSurface(data, listofpoints):
    """
    Plot animation with surface information
    """
    def update(frame, lines, data, scatters, listofpoints):
        print(frame)
        for line, dataBone in zip(lines, data):
            x = np.asarray([dataBone[0,frame], dataBone[3,frame]])
            y = np.asarray([dataBone[1,frame], dataBone[4,frame]])
            z = np.asarray([dataBone[2,frame], dataBone[5,frame]])
            line.set_data(x,y)
            line.set_3d_properties(z)
        for scat, point in zip(scatters,listofpoints):
            scat.set_data([point.position[frame][0]],[point.position[frame][1]])
            scat.set_3d_properties([point.position[frame][2]])

        return lines+scatters

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    lines = []
    for i in range(len(data)):
        lines.append(ax.plot([data[i,0,0], data[i,3,0]], [data[i,1,0], data[i,4,0]], [data[i,2,0], data[i,5,0]],'-o', color='black')[0])

    scatters = []
    for i in range(len(listofpoints)):
        scatters.append(ax.plot([listofpoints[i].position[0][0]],[listofpoints[i].position[0][1]],[listofpoints[i].position[0][2]],'o', color='red', markersize=1)[0])


    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(data), np.max(data)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), fargs=(lines, data, scatters, listofpoints),interval=1,
                             blit=True)

    plt.show()


def AnimationSurfaceAndVectors(data, listofpoints, dispvectors):
    """
    Plot animation with surface information and displacement vectors
    """
    def update(frame, bones, data, scatters, listofpoints, vectors, dispvectors):
        print(frame)
        for line, dataBone in zip(bones, data):
            x = np.asarray([dataBone[0,frame], dataBone[4,frame]])
            y = np.asarray([dataBone[1,frame], dataBone[5,frame]])
            z = np.asarray([dataBone[2,frame], dataBone[6,frame]])
            line.set_data(x,y)
            line.set_3d_properties(z)
        for scat, point in zip(scatters,listofpoints):
            scat.set_data([point.position[frame,0]],[point.position[frame,1]])
            scat.set_3d_properties([point.position[frame,2]])
        for vec, dispvec in zip(vectors, dispvectors[frame]):
            x = np.asarray([dispvec[0][0], dispvec[1][0]])
            y = np.asarray([dispvec[0][1], dispvec[1][1]])
            z = np.asarray([dispvec[0][2], dispvec[1][2]])
            vec.set_data(x,y)
            vec.set_3d_properties(z)

        return bones+scatters+vectors

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    bones = []
    for i in range(len(data)):
        bones.append(ax.plot([data[i,0,0], data[i,4,0]], [data[i,1,0], data[i,5,0]], [data[i,2,0], data[i,6,0]],'-o', color='black')[0])

    scatters = []
    for i in range(len(listofpoints)):
        scatters.append(ax.plot([listofpoints[i].position[0,0]],[listofpoints[i].position[0,1]],[listofpoints[i].position[0,2]],'o', color='red', markersize=1)[0])

    vectors = []
    for i in range(len(dispvectors[0])):
        #[frame][vectorfromtriangle][p1orp2][xyz]
        vectors.append(ax.plot([dispvectors[0][i][0][0], dispvectors[0][i][1][0]], [dispvectors[0][i][0][1], dispvectors[0][i][1][1]], [dispvectors[0][i][0][2], dispvectors[0][i][1][2]],'-', color='red')[0])

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(data), np.max(data)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), fargs=(bones, data, scatters, listofpoints, vectors, dispvectors),interval=1,
                             blit=True)

    plt.show()

def AnimationSurface2(data, listofpoints):
    """
    OLD (bones with 6 float array)
    Plot animation with surface information
    """
    def update(frame, lines, data, scatters, listofpoints):
        print(frame)
        for line, dataBone in zip(lines, data):
            x = np.asarray([dataBone[0,frame], dataBone[3,frame]])
            y = np.asarray([dataBone[1,frame], dataBone[4,frame]])
            z = np.asarray([dataBone[2,frame], dataBone[5,frame]])
            line.set_data(x,y)
            line.set_3d_properties(z)
        for scat, point in zip(scatters,listofpoints):
            scat.set_data([point.position[frame,0]],[point.position[frame,1]])
            scat.set_3d_properties([point.position[frame,2]])

        return lines+scatters

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    lines = []
    for i in range(len(data)):
        lines.append(ax.plot([data[i,0,0], data[i,3,0]], [data[i,1,0], data[i,4,0]], [data[i,2,0], data[i,5,0]],'-o', color='black')[0])

    scatters = []
    for i in range(len(listofpoints)):
        scatters.append(ax.plot([listofpoints[i].position[0,0]],[listofpoints[i].position[0,1]],[listofpoints[i].position[0,2]],'o', color='red', markersize=1)[0])


    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(data), np.max(data)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), fargs=(lines, data, scatters, listofpoints),interval=1,
                             blit=True)

    plt.show()


def plotAxesFromMatrix(matrix):
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
#    ax.plot([0,0,0],[matrix[0,0], matrix[1,0], matrix[2,0]], color='red')
#    ax.plot([0,0,0],[matrix[0,1], matrix[1,1], matrix[2,1]], color='green')
#    ax.plot([0,0,0],[matrix[0,2], matrix[1,2], matrix[2,2]], color='blue')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.plot([0,matrix[0,0]],[0,matrix[1,0]],[0, matrix[2,0]], color='red')
    ax.plot([0,matrix[0,1]],[0,matrix[1,1]],[0, matrix[2,1]], color='green')
    ax.plot([0,matrix[0,2]],[0,matrix[1,2]],[0, matrix[2,2]], color='blue')
    plt.show()

def plotAxesVectors(x,y,z=False):
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
#    ax.plot([0,0,0],[matrix[0,0], matrix[1,0], matrix[2,0]], color='red')
#    ax.plot([0,0,0],[matrix[0,1], matrix[1,1], matrix[2,1]], color='green')
#    ax.plot([0,0,0],[matrix[0,2], matrix[1,2], matrix[2,2]], color='blue')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.plot([0,x[0]],[0,x[1]],[0, x[2]], color='red')
    ax.plot([0,y[0]],[0,y[1]],[0, y[2]], color='green')
    if z:
        ax.plot([0,z[0]],[0,z[1]],[0, z[2]], color='blue')
    plt.show()
    
def plotVector(vec):
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.plot([0,vec[0]],[0,vec[1]],[0, vec[2]], color='red')
    plt.show()

def AnimationSurfaceAndVectors2(data, listofpoints, dispvectors):
    """
    OLD (bones with 6 float array)
    Plot animation with surface information and displacement vectors
    """
    def update(frame, bones, data, scatters, listofpoints, vectors, dispvectors):
        print(frame)
        for line, dataBone in zip(bones, data):
            x = np.asarray([dataBone[0,frame], dataBone[3,frame]])
            y = np.asarray([dataBone[1,frame], dataBone[4,frame]])
            z = np.asarray([dataBone[2,frame], dataBone[5,frame]])
            line.set_data(x,y)
            line.set_3d_properties(z)
        for scat, point in zip(scatters,listofpoints):
            scat.set_data([point.position[frame,0]],[point.position[frame,1]])
            scat.set_3d_properties([point.position[frame,2]])
        for vec, dispvec in zip(vectors, dispvectors[frame]):
            x = np.asarray([dispvec[0][0], dispvec[1][0]])
            y = np.asarray([dispvec[0][1], dispvec[1][1]])
            z = np.asarray([dispvec[0][2], dispvec[1][2]])
            vec.set_data(x,y)
            vec.set_3d_properties(z)

        return bones+scatters+vectors

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    bones = []
    for i in range(len(data)):
        bones.append(ax.plot([data[i,0,0], data[i,3,0]], [data[i,1,0], data[i,4,0]], [data[i,2,0], data[i,5,0]],'-o', color='black')[0])

    scatters = []
    for i in range(len(listofpoints)):
        scatters.append(ax.plot([listofpoints[i].position[0,0]],[listofpoints[i].position[0,1]],[listofpoints[i].position[0,2]],'o', color='red', markersize=1)[0])

    vectors = []
    for i in range(len(dispvectors[0])):
        #[frame][vectorfromtriangle][p1orp2][xyz]
        vectors.append(ax.plot([dispvectors[0][i][0][0], dispvectors[0][i][1][0]], [dispvectors[0][i][0][1], dispvectors[0][i][1][1]], [dispvectors[0][i][0][2], dispvectors[0][i][1][2]],'-', color='red')[0])

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(data), np.max(data)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), fargs=(bones, data, scatters, listofpoints, vectors, dispvectors),interval=1,
                             blit=True)

    plt.show()





def DebugEgoCoord(animation, frame, proj = '3d'):

    def getVectors(animation, frame):
        """
        Get vectors to calculate the kinematic path

        :type animation: pyanimation.Animation
        :param animation: Animation (skeleton) to get the distance between mapped joints
        """
        lhand = animation.getskeletonmap().lhand
        rhand = animation.getskeletonmap().rhand
        lforearm = animation.getskeletonmap().lforearm
        rforearm = animation.getskeletonmap().rforearm
        larm = animation.getskeletonmap().larm
        rarm = animation.getskeletonmap().rarm
        upspine = animation.getskeletonmap().spine3
        #não estou procurando "head" pq o 'head' da Talita não corresponde ao 'head' do shogun
        neck = animation.getskeletonmap().neck
        hips = animation.getskeletonmap().hips

        lvec_fore = lhand.getPosition(frame) - lforearm.getPosition(frame)
        rvec_fore = rhand.getPosition(frame) - rforearm.getPosition(frame)

        lvec_arm = lforearm.getPosition(frame) - larm.getPosition(frame)
        rvec_arm = rforearm.getPosition(frame) - rarm.getPosition(frame)

        lvec_clavicle = larm.getPosition(frame) - upspine.getPosition(frame)
        rvec_clavicle = rarm.getPosition(frame) - upspine.getPosition(frame)

        vec_neck = upspine.getPosition(frame) - neck.getPosition(frame)

        vec_hips = upspine.getPosition(frame) - hips.getPosition(frame)

        return lvec_fore, rvec_fore, lvec_arm, rvec_arm, lvec_clavicle, rvec_clavicle, vec_neck, vec_hips


    lvec_fore, rvec_fore, lvec_arm, rvec_arm, lvec_clavicle, rvec_clavicle, vec_neck, vec_hips = getVectors(animation, frame)
    hips = [0,0,0]
    Upspine = vec_hips - hips
    Neck = - vec_neck + Upspine
    RArm = rvec_clavicle + Upspine
    LArm = lvec_clavicle + Upspine
    RFore = rvec_arm + RArm
    LFore = lvec_arm + LArm
    RHand = rvec_fore + RFore
    LHand = lvec_fore + LFore
    print(rvec_fore)
    print(rvec_arm)
    print(rvec_clavicle)
    print(vec_neck)
    print(vec_hips)
    if proj=='3d':
        fig = plt.figure(figsize=(12,8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.plot([hips[0],Upspine[0]],[hips[1],Upspine[1]],[hips[2],Upspine[2]], '-o',color='black')
        ax.plot([Upspine[0],Neck[0]],[Upspine[1],Neck[1]],[Upspine[2],Neck[2]], '-o',color='gray')
        ax.plot([Upspine[0],RArm[0]],[Upspine[1],RArm[1]],[Upspine[2],RArm[2]], '-o',color='yellow')
        ax.plot([Upspine[0],LArm[0]],[Upspine[1],LArm[1]],[Upspine[2],LArm[2]], '-o',color='yellow')
        ax.plot([RArm[0],RFore[0]],[RArm[1],RFore[1]],[RArm[2],RFore[2]], '-o',color='red')
        ax.plot([LArm[0],LFore[0]],[LArm[1],LFore[1]],[LArm[2],LFore[2]], '-o',color='red')
        ax.plot([RFore[0],RHand[0]],[RFore[1],RHand[1]],[RFore[2],RHand[2]], '-o',color='blue')
        ax.plot([LFore[0],LHand[0]],[LFore[1],LHand[1]],[LFore[2],LHand[2]], '-o',color='blue')
        plt.show()
    else:
        fig = plt.figure(figsize=(12,8))
        ax = fig.add_subplot(111)
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.plot([hips[0],Upspine[0]],[hips[1],Upspine[1]], '-o',color='black')
        ax.plot([Upspine[0],Neck[0]],[Upspine[1],Neck[1]], '-o',color='gray')
        ax.plot([Upspine[0],RArm[0]],[Upspine[1],RArm[1]], '-o',color='yellow')
        ax.plot([Upspine[0],LArm[0]],[Upspine[1],LArm[1]], '-o',color='yellow')
        ax.plot([RArm[0],RFore[0]],[RArm[1],RFore[1]], '-o',color='red')
        ax.plot([LArm[0],LFore[0]],[LArm[1],LFore[1]], '-o',color='red')
        ax.plot([RFore[0],RHand[0]],[RFore[1],RHand[1]], '-o',color='blue')
        ax.plot([LFore[0],LHand[0]],[LFore[1],LHand[1]], '-o',color='blue')
        plt.show()

def PlotBVH(animation):
    """
    Plot animation as BVH, calculate the position inside this funtion

    :type animation: Animation class object
    :param animation: Animation to be draw
    """
    def update(frame, scatters):
        print(frame)
        for scat, joint in zip(scatters,animation.getlistofjoints()):
            position = joint.getPosition(frame)
            scat.set_data([position[0]],[position[1]])
            scat.set_3d_properties([position[2]])

        return scatters

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    scatters = []
    maxdata = -np.inf
    mindata = np.inf
    for joint in animation.getlistofjoints():
        position = joint.getPosition(frame = 0)
        scatters.append(ax.plot([position[0]],[position[1]],[position[2]],'o', color='red', markersize=1)[0])
        if np.min(position)<mindata:
            mindata = np.min(position)
        if np.max(position)>maxdata:
            maxdata = np.max(position)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim(mindata,maxdata)
    ax.set_ylim(mindata,maxdata)
    ax.set_zlim(mindata,maxdata)
    ani = FuncAnimation(fig, update, frames=np.arange(animation.frames), fargs=(scatters) ,interval=1,
                             blit=True)

    plt.show()


def PlotBVHSurface(animation, surface):       
    def update(frame, surf, lines):
        for triangle_plot, i in zip(surf, range(len(surface.headmesh)+len(surface.bodymesh))):
            if i< len(surface.headmesh):
                vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1],vert.getPosition(animation,frame)[2]] for vert in surface.headmesh[i]]
                vertices.append([surface.headmesh[i][0].getPosition(animation,frame)[0],surface.headmesh[i][0].getPosition(animation,frame)[1],surface.headmesh[i][0].getPosition(animation,frame)[2]])
                vertices = np.asarray(vertices)
            else:
                j = i - len(surface.headmesh)
                vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1],vert.getPosition(animation,frame)[2]] for vert in surface.bodymesh[j]]
                vertices.append([surface.bodymesh[j][0].getPosition(animation,frame)[0],surface.bodymesh[j][0].getPosition(animation,frame)[1],surface.bodymesh[j][0].getPosition(animation,frame)[2]])
                vertices = np.asarray(vertices)
            triangle_plot.set_data(vertices[:,0],vertices[:,1])
            triangle_plot.set_3d_properties(vertices[:,2])
        
        for line, bone in zip(lines, animation.getBones(frame)):
            line.set_data([bone[0], bone[3]], [bone[1], bone[4]])
            line.set_3d_properties([bone[2], bone[5]])
        return surf+lines

    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
    scatters = []
    maxdata = -np.inf
    mindata = np.inf
    for joint in animation.getlistofjoints():
        position = joint.getPosition(frame = 0)
        #scatters.append(ax.plot([position[0]],[position[1]],[position[2]],'o', color='red', markersize=1)[0])
        if np.min(position)<mindata:
            mindata = np.min(position)
        if np.max(position)>maxdata:
            maxdata = np.max(position)
            
    surf = []
    for triangle in surface.headmesh:
        vertices = [[vert.getPosition(animation,0)[0],vert.getPosition(animation,0)[1],vert.getPosition(animation,0)[2]] for vert in triangle]
        vertices.append([triangle[0].getPosition(animation,0)[0],triangle[0].getPosition(animation,0)[1],triangle[0].getPosition(animation,0)[2]])
        vertices = np.asarray(vertices)
        surf.append(ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],'-o', color='red', markersize=1, alpha = 0.5)[0])

    for triangle in surface.bodymesh:
        vertices = [[vert.getPosition(animation,0)[0],vert.getPosition(animation,0)[1],vert.getPosition(animation,0)[2]] for vert in triangle]
        vertices.append([triangle[0].getPosition(animation,0)[0],triangle[0].getPosition(animation,0)[1],triangle[0].getPosition(animation,0)[2]])
        vertices = np.asarray(vertices)
        surf.append(ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],'-o', color='red', markersize=1, alpha = 0.5)[0])
            
    lines = []
    for bone in animation.getBones(0):
        lines.append(ax.plot([bone[0], bone[3]], [bone[1], bone[4]], [bone[2], bone[5]],'-o', color='black')[0])
        
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim(mindata,maxdata)
    ax.set_ylim(mindata,maxdata)
    ax.set_zlim(mindata,maxdata)
    ani = FuncAnimation(fig, update, frames=np.arange(animation.frames), fargs=(surf,lines) ,interval=1, blit=True)
    return ani

def PlotPoseAndSurface(animation, surface, frame):
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    aux = animation.getBones(frame)
    bones = []
    for i in range(len(aux)):
        bones.append(ax.plot([aux[i,0], aux[i,3]], [aux[i,1], aux[i,4]], [aux[i,2], aux[i,5]],'-o', color='black')[0])


    surf = []
    for triangle in surface.headmesh:
        vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1],vert.getPosition(animation,frame)[2]] for vert in triangle]
        vertices.append([triangle[0].getPosition(animation,frame)[0],triangle[0].getPosition(animation,frame)[1],triangle[0].getPosition(animation,frame)[2]])
        vertices = np.asarray(vertices)
        surf.append(ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],'-o', color='red', markersize=1, alpha = 0.5)[0])

    for triangle in surface.bodymesh:
        vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1],vert.getPosition(animation,frame)[2]] for vert in triangle]
        vertices.append([triangle[0].getPosition(animation,frame)[0],triangle[0].getPosition(animation,frame)[1],triangle[0].getPosition(animation,frame)[2]])
        vertices = np.asarray(vertices)
        surf.append(ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],'-o', color='red', markersize=1, alpha = 0.5)[0])

#    vectors = []
#    for i in range(len(dispvectors[0])):
#        #[frame][vectorfromtriangle][p1orp2][xyz]
#        vectors.append(ax.plot([dispvectors[0][i][0][0], dispvectors[0][i][1][0]], [dispvectors[0][i][0][1], dispvectors[0][i][1][1]], [dispvectors[0][i][0][2], dispvectors[0][i][1][2]],'-', color='red')[0])

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    mini,maxi = np.min(aux), np.max(aux)
    ax.set_xlim(mini,maxi)
    ax.set_ylim(mini,maxi)
    ax.set_zlim(mini,maxi)
    ax.set_axis_off()
#    ani = FuncAnimation(fig, update, frames=np.arange(len(data[0,0,:])), fargs=(bones, data, scatters, listofpoints, vectors, dispvectors),interval=1,
#                             blit=True)

    plt.show()
    
    
def CheckTargets(animation, joint, joint1, ego):
    target = np.asarray([ego.getTarget(frame) for frame in range(animation.frames)])
    position = np.asarray([joint.getPosition(frame) for frame in range(animation.frames)])
    position1 = np.asarray([joint1.getPosition(frame) for frame in range(animation.frames)])
    
    
    fig = plt.figure(figsize=(12,6))
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(target[:,0], label='X', color = 'red', linestyle = '--')
    ax.plot(target[:,1], label='Y', color = 'green', linestyle = '--')
    ax.plot(target[:,2], label='Z', color = 'blue', linestyle = '--')
    ax.plot(position[:,0], color = 'red', linestyle = '-')
    ax.plot(position[:,1], color = 'green', linestyle = '-')
    ax.plot(position[:,2], color = 'blue', linestyle = '-')
    ax.plot(position1[:,0], color = 'black', linestyle = '-')
    ax.plot(position1[:,1], color = 'black', linestyle = '-')
    ax.plot(position1[:,2], color = 'black', linestyle = '-')
    
    plt.legend(title='Target position:')
    plt.show()


def PlotDispvectorsFrames(srcAnim, srcSurface, tgtAnim, tgtSurface, egoCoord, f1, f2, f3):
    fig, axs = plt.subplots(2, 3, figsize=(16,8))
    
    min_X, max_X = np.inf,np.NINF
    min_Y, max_Y = np.inf,np.NINF
    for animation, surface, line in zip([srcAnim, tgtAnim],[srcSurface, tgtSurface],range(2)):
        for frame, column in zip([f1,f2,f3],range(3)):
            aux = animation.getBones(frame)

            for i in range(len(aux)):
                axs[line, column].plot([aux[i,0], aux[i,3]], [aux[i,1], aux[i,4]],'-o', color='black')

            for triangle in surface.headmesh:
                vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1]] for vert in triangle]
                vertices.append([triangle[0].getPosition(animation,frame)[0],triangle[0].getPosition(animation,frame)[1]])
                vertices = np.asarray(vertices)
                axs[line, column].plot(vertices[:,0],vertices[:,1],'-o', color='red', markersize=1, alpha = 0.5)

            for triangle in surface.bodymesh:
                vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1]] for vert in triangle]
                vertices.append([triangle[0].getPosition(animation,frame)[0],triangle[0].getPosition(animation,frame)[1]])
                vertices = np.asarray(vertices)
                axs[line, column].plot(vertices[:,0],vertices[:,1],'-o', color='red', markersize=1, alpha = 0.5)
            mini_X,maxi_X = np.min([aux[:,0].min(),aux[:,3].min()]), np.max([aux[:,0].max(),aux[:,3].max()])
            mini_Y, maxi_Y = np.min([aux[:,1].min(),aux[:,4].min()]), np.max([aux[:,1].max(),aux[:,4].max()])
            if mini_X<min_X: 
                min_X=mini_X
            if maxi_X>max_X: 
                max_X=maxi_X
            if mini_Y<min_Y: 
                min_Y=mini_Y
            if maxi_Y>max_Y: 
                max_Y=maxi_Y
                
    lenNoLimb = len(srcSurface.headmesh)+len(srcSurface.bodymesh)
    for frame, column in zip([f1,f2,f3],range(3)):        
        dispvector = np.asarray([np.asarray(egoCoord.framecoord[frame].dispvector[i]*egoCoord.framecoord[frame].tau[i])+egoCoord.framecoord[frame].refpoint[i] for i in range(lenNoLimb)])
        refpoint = np.asarray(egoCoord.framecoord[frame].refpoint[:lenNoLimb])
        norm_importance = egoCoord.framecoord[frame].importance/egoCoord.framecoord[frame].importance.max()
        for i in range(lenNoLimb):
            axs[0, column].plot([refpoint[i,0],dispvector[i,0]],[refpoint[i,1],dispvector[i,1]], color=[1*(1-norm_importance[i]),1*(1-norm_importance[i]),1])
        dispvector = np.asarray([np.asarray(egoCoord.framecoord[frame].tgt_dispvector[i])+egoCoord.framecoord[frame].tgt_refpoint[i] for i in range(lenNoLimb)])
        refpoint = np.asarray(egoCoord.framecoord[frame].tgt_refpoint[:lenNoLimb])
        norm_importance = egoCoord.framecoord[frame].importance/egoCoord.framecoord[frame].importance.max()
        for i in range(lenNoLimb):
            axs[1, column].plot([refpoint[i,0],dispvector[i,0]],[refpoint[i,1],dispvector[i,1]], color=[1*(1-norm_importance[i]),1*(1-norm_importance[i]),1])
    for line in range(2):
        for column in range(3):
            axs[line, column].set_xlim(min_X-5,max_X+5)
            axs[line, column].set_ylim(min_Y-10,max_Y+10)
            axs[line, column].set_frame_on(False)
            axs[line, column].tick_params(axis='both', which='both', length=0)
            axs[line, column].set_xticks([])
            if column!=0:
                axs[line, column].set_yticks([])
            #axs[line, column].set_axis_off()
    frames = [f1,f2,f3]
    font_size = 16
    for column in range(3):
        axs[1,column].set_xlabel(str.format('Frame %i' % frames[column]), fontsize=font_size)
        
    axs[0,0].set_ylabel('Source', fontsize=font_size)
    axs[1,0].set_ylabel('Target', fontsize=font_size)
    
    plt.tight_layout()
    plt.show()
    
    
def PlotDispvectorsFrames3D(animation, surface, egoCoord, frame, target = True, skipHead = False, skipBody = False):
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')
    aux = animation.getBones(frame)

    for i in range(len(aux)):
        ax.plot([aux[i,0], aux[i,3]], [aux[i,1], aux[i,4]],[aux[i,2], aux[i,5]],'-o', color='black')

    for triangle in surface.headmesh:
        vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1],vert.getPosition(animation,frame)[2]] for vert in triangle]
        vertices.append([triangle[0].getPosition(animation,frame)[0],triangle[0].getPosition(animation,frame)[1],triangle[0].getPosition(animation,frame)[2]])
        vertices = np.asarray(vertices)
        ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],'-o', color='red', markersize=1, alpha = 0.5)

    for triangle in surface.bodymesh:
        vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1],vert.getPosition(animation,frame)[2]] for vert in triangle]
        vertices.append([triangle[0].getPosition(animation,frame)[0],triangle[0].getPosition(animation,frame)[1],triangle[0].getPosition(animation,frame)[2]])
        vertices = np.asarray(vertices)
        ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],'-o', color='red', markersize=1, alpha = 0.5)
    mini_X,maxi_X = np.min([aux[:,0].min(),aux[:,3].min()]), np.max([aux[:,0].max(),aux[:,3].max()])
    mini_Y, maxi_Y = np.min([aux[:,1].min(),aux[:,4].min()]), np.max([aux[:,1].max(),aux[:,4].max()])
    mini_Z, maxi_Z = np.min([aux[:,2].min(),aux[:,5].min()]), np.max([aux[:,2].max(),aux[:,5].max()])
    
    lenNoLimb = len(surface.headmesh)+len(surface.bodymesh)
    #Print target or source ego coords refpoint 
    if not target:
        dispvector = np.asarray([np.asarray(egoCoord.framecoord[frame].dispvector[i]*egoCoord.framecoord[frame].tau[i])+egoCoord.framecoord[frame].refpoint[i] for i in range(lenNoLimb)])
        refpoint = np.asarray(egoCoord.framecoord[frame].refpoint[:lenNoLimb])
        norm_importance = egoCoord.framecoord[frame].importance/egoCoord.framecoord[frame].importance.max()
        for i in range(lenNoLimb):
            ax.plot([refpoint[i,0],dispvector[i,0]],[refpoint[i,1],dispvector[i,1]],[refpoint[i,2],dispvector[i,2]], color=[np.clip(1*(1-norm_importance[i]),0,0.8),np.clip(1*(1-norm_importance[i]),0,0.8),1,0.8])
            ax.scatter(refpoint[i,0], refpoint[i,1], refpoint[i,2], color='red', alpha=0.5)
    else:
        dispvector = np.asarray([np.asarray(egoCoord.framecoord[frame].tgt_dispvector[i])+egoCoord.framecoord[frame].tgt_refpoint[i] for i in range(len(egoCoord.framecoord[frame].tgt_refpoint))])
        refpoint = np.asarray(egoCoord.framecoord[frame].tgt_refpoint[:])
        norm_importance = egoCoord.framecoord[frame].importance/egoCoord.framecoord[frame].importance.max()
        for i in range(len(egoCoord.framecoord[frame].tgt_refpoint)):
            if skipHead:
                if i>len(surface.headmesh):
                    ax.plot([refpoint[i,0],dispvector[i,0]],[refpoint[i,1],dispvector[i,1]],[refpoint[i,2],dispvector[i,2]], color=[np.clip(1*(1-norm_importance[i]),0,1),np.clip(1*(1-norm_importance[i]),0,1),1,0.8])
                    ax.scatter(refpoint[i,0], refpoint[i,1], refpoint[i,2], color='red', alpha=0.5)
            elif skipBody:
                if i<len(surface.headmesh):
                    ax.plot([refpoint[i,0],dispvector[i,0]],[refpoint[i,1],dispvector[i,1]],[refpoint[i,2],dispvector[i,2]], color=[np.clip(1*(1-norm_importance[i]),0,1),np.clip(1*(1-norm_importance[i]),0,1),1,0.8])
                    ax.scatter(refpoint[i,0], refpoint[i,1], refpoint[i,2], color='red', alpha=0.5)
            else:
                ax.plot([refpoint[i,0],dispvector[i,0]],[refpoint[i,1],dispvector[i,1]],[refpoint[i,2],dispvector[i,2]], color=[np.clip(1*(1-norm_importance[i]),0,1),np.clip(1*(1-norm_importance[i]),0,1),1,0.8])
                ax.scatter(refpoint[i,0], refpoint[i,1], refpoint[i,2], color='red', alpha=0.5)
        target = egoCoord.getTarget(frame)
        ax.scatter(target[0], target[1], target[2], color='red', marker="X")
            
            
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    # ax.set_xlim(mini_X,maxi_X)
    # ax.set_ylim(mini_Y,maxi_Y)
    # ax.set_zlim(mini_Z,maxi_Z)
    ax.set_xlim(-100,100)
    ax.set_ylim(-20,200)
    ax.set_zlim(-100,100)
    plt.show()
    
    
    
    
def MSc_printSurfaces(animation, surface, step = 45, frame = 0):
    """
    Imprime surperficies em diferentes posições
    Parameters
    ----------
    anim : TYPE
        DESCRIPTION.
    surf : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    fig, axs = plt.subplots(1, 6, figsize=(16,8))
    rot = mathutils.matrixR([0,-90+step,0])
    
    
    
    for i in range(6):
        rot = mathutils.matrixR([0,-90+step*i,0])
        for triangle in surface.bodymesh + surface.headmesh:
            vertices = [[vert.getPosition(animation,frame)[0],vert.getPosition(animation,frame)[1],vert.getPosition(animation,frame)[2]] for vert in triangle]
            vertices.append([triangle[0].getPosition(animation,frame)[0],triangle[0].getPosition(animation,frame)[1],triangle[0].getPosition(animation,frame)[2]])
            vertices = [np.dot(rot,vertices[i]) for i in range(len(vertices))]
            vertices = np.asarray(vertices)
            if (vertices[:,2]<0).any():
                alpha = 0.3
            else:
                alpha = 1
            #zcolor = np.abs(vertices[:,2])/max(np.abs(vertices[:,2]))
            axs[i].plot(vertices[:,0],vertices[:,1],'-o', color='red', markersize=5, alpha = 0.5)
            axs[i].set_axis_off()
            # print(vertices[:,2])
            # print('----------------------')
    
    
def MSc_PlotOrtho(which, epsilon=0.001, step=0.1, start = -np.pi, stop = np.pi):
    x = np.arange(start,stop,step)   # start,stop,step
    if which == 'cos':
        y = np.cos(x)
    elif which == 'sen':
        y = np.sin(x)
    elif which == 'molla_ortho':
        y = [np.cos(n) if np.cos(n)>epsilon else np.cos(epsilon) for n in x]
    elif which == 'tonoli_ortho':
        y = (np.cos(x) +1)/2
    else:
        print('Escolha algum')
        return None
    plt.plot(x,y)
    plt.axhline(y=0, color='gray', alpha=0.3)
    plt.axvline(x=0, color='gray', alpha=0.3)
    plt.show()
        
                
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    