import mathutils
import numpy as np
import time
import ik

def checkColisions(animation, surface):
    
    def __returnChildren(joint, depth = 0):
        yield joint, depth
        for child in joint.children:
            for child_aux, depth_aux in __returnChildren(child, depth = depth + 1):
                yield child_aux, depth_aux
        
        
        
    def performIK(animation, joint, frame, target, depth = 3):
        print(animation.getJoint('RightHand').getLocalRotation(frame))
        jacobian = ik.SimpleJacobian(animation, joint, depth)
        log = jacobian.jacobianTranspose(frame, target, epsilon = 1e-4, maxiter=100)
        print(animation.getJoint('RightHand').getLocalRotation(frame))
        return log
            
    
    def jointColision(animation, joint,surface, frame, mesh):
        min_distance_head = np.inf
        i_head = None
        min_distance_body = np.inf
        i_body = None
        p = joint.getPosition(frame)
        p_last = joint.getPosition(frame-1)
        for i in range(len(surface.headmesh)+len(surface.bodymesh)):
            
            triangle = mesh[i]
            p0 = triangle[0]
            p1 = triangle[1]
            p2 = triangle[2]
            
            n, baryCoord, distance_vector, projectedpoint, _ = mathutils.projectedBarycentricCoord(p,p0,p1,p2)
            n_last, baryCoord_last, distance_vector_last, projectedpoint_last, insideTriangle_last = mathutils.projectedBarycentricCoord(p_last,p0,p1,p2)
            
            centroid,normal=mathutils.getCentroid(p0,p1,p2)
            centroid_last,normal_last=mathutils.getCentroid(p0,p1,p2)
            
            cos =  mathutils.cosBetween(normal, p-centroid, absolute = False)
            cos_last =  mathutils.cosBetween(normal_last, p_last-centroid_last, absolute = False)
            
            centroid,normal=mathutils.getCentroid(p0,p1,p2)
            if i<len(surface.headmesh):
                # if cos >0 and cos_last<0:
                if np.linalg.norm(p-centroid) < 5 and cos>0:
                    print(frame)
                    print(joint.name)
                    # print('Component: ', i)
                    # print('cos: ', cos)
                    # print('bary coord: ', baryCoord)
                    print('joint position: ', p)
                    # print('projected point: ', projectedpoint)
                    # print('Start IK', joint.name, p, p_last)
                    # print(np.linalg.norm(p-projectedpoint))
                    headpos = animation.getskeletonmap().head.getPosition(frame)
                    # print(headpos)
                    direction = mathutils.unitVector(p-headpos)
                    # print(direction)
                    newpos = p+(direction*np.linalg.norm(p-projectedpoint))
                    print(newpos)
                    print(np.linalg.norm(p-newpos))
                    logIK = performIK(animation, joint, frame, newpos)
                    print(logIK)
                    
                    p = joint.getPosition(frame)
                    p_last = joint.getPosition(frame-1)
                if np.linalg.norm(p-centroid) < min_distance_head:
                    min_distance_head = np.linalg.norm(p-centroid)
                    i_head = i
            else:
                if np.linalg.norm(p-centroid) < min_distance_body:
                    min_distance_body = np.linalg.norm(p-centroid)
                    i_body = i
            
            
        #Get projection onto triangle for head mesh component
        # triangle = mesh[i_head]
        # p0 = triangle[0]
        # p1 = triangle[1]
        # p2 = triangle[2]
        # n, baryCoord, distance_vector, projectedpoint, insideTriangle = mathutils.projectedBarycentricCoord(p,p0,p1,p2)
        # centroid,normal=mathutils.getCentroid(p0,p1,p2)
        # normal_last, baryCoord_last, distance_vector_last, projectedpoint_last, insideTriangle_last = mathutils.projectedBarycentricCoord(p_last,p0,p1,p2)
        # cos =  mathutils.cosBetween(normal, distance_vector, absolute = False)
        # cos_last =  mathutils.cosBetween(normal_last, distance_vector_last, absolute = False)
        # if joint == animation.getJoint('finger1-2.R'):
        #     if np.linalg.norm(centroid-p) < 10:
        #         print(i_head)
        #         print(baryCoord)
        #         print(cos)
            # if insideTriangle == True:
                # print('Head')
                # print(frame)
                # print(joint.name, joint.getPosition(frame))
                # print('Mesh: ')
                # print(p0)
                # print(p1)
                # print(p2)
                # print('Data:')
                # print(baryCoord)
                # print(projectedpoint)
                # print(distance_vector)
                # print(insideTriangle)
                # performIK(animation, joint, frame, p+distance_vector)
            
        
        
        # triangle = mesh[i_body]
        # p0 = triangle[0]
        # p1 = triangle[1]
        # p2 = triangle[2]
        # normal, baryCoord, distance_vector, projectedpoint = mathutils.projectedBarycentricCoord(p,p0,p1,p2)
        # normal_last, baryCoord_last, distance_vector_last, projectedpoint_last= mathutils.projectedBarycentricCoord(p_last,p0,p1,p2)
        # cos =  mathutils.cosBetween(normal, distance_vector, absolute = False)
        # cos_last =  mathutils.cosBetween(normal_last, distance_vector_last, absolute = False)
        
        # if cos_last>0 and cos<=0:
        #     print('Body')
        #     print(joint.name, frame)
        #     print(projectedpoint)
        #     print(projectedpoint_last)
        #     performIK(animation, joint, frame, p+distance_vector)
            
        
        # #if projected point is inside triangle
        # if (baryCoord>0).all():
        #     print(joint.name, i)
        #     #if the joint is inside the mesh, that is, the normal and the distance from the projected point have a angle between them bigger then 90 (or -90) degrees
        #     if (mathutils.cosBetween(normal, distance_vector, absolute = False) < 0):
        #         print(joint.name, i)
            
            
            
    lforearm, rforearm = animation.getskeletonmap().lforearm, animation.getskeletonmap().rforearm
    lhand, rhand = animation.getskeletonmap().lhand, animation.getskeletonmap().rhand
    llowleg, rlowleg = animation.getskeletonmap().llowleg, animation.getskeletonmap().rlowleg
    lfoot, rfoot = animation.getskeletonmap().lfoot, animation.getskeletonmap().rfoot
    limbsjoint = [lforearm, rforearm, lhand, rhand, llowleg, rlowleg, lfoot, rfoot]
    forearm_radius = min(surface.getPoint('foreLeft').radius, surface.getPoint('foreRight').radius)
    arm_radius = min(surface.getPoint('armLeft').radius, surface.getPoint('armRight').radius)
    shin_radius = min(surface.getPoint('shinLeft').radius, surface.getPoint('shinRight').radius)
    thight_radius = min(surface.getPoint('thightLeft').radius, surface.getPoint('thightRight').radius)
    start = start=time.time()
    
    # for frame in range(animation.frames):
    for frame in range(1730,1830):
        if np.mod(frame+1,100) == 0:
            print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
            start=time.time()
        
        mesh = [[triangle[0].getPosition(animation, frame) ,triangle[1].getPosition(animation, frame),triangle[2].getPosition(animation, frame)] for triangle in surface.headmesh+surface.bodymesh]
        
        # for joint in limbsjoint:
        for joint in [rhand]:
            jointColision(animation, joint, surface, frame, mesh)
            if joint == lfoot or joint == rfoot or joint == lhand or joint == rhand:
                for childjoint, i in __returnChildren(joint):
                    jointColision(animation, childjoint, surface, frame, mesh)
                
                

                
    # Pega juntas dos membros (as 8)
    # Para cada uma delas:
    #     pega a normal dos componentes
    #     verificar a distancia e a produto interno da posicao com a normal
    #     se for o componente mais proximo e o produto interno for negativo, significa que esta penetrando
    #     calcular a penetracao e empurrar a junta pra fora
    # pegar todas as juntas que sao filhas das mãos e pés, fazer a mesma coisa que de cima