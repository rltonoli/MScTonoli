import os
from retarget import MotionRetargeting
from surface import GetCalibrationFromBVHS
import time
import mathutils

# Generate Surface Calibration ###############################################
#start = time.time()
# surface.GetCalibrationFromBVHS('Superficie\Rodolfo\Frente001_mcp.bvh', 'Superficie\Rodolfo\Cabeca001_mcp.bvh', 'Superficie\Rodolfo\Costas001_mcp.bvh',savefile = True, debugmode = False)
#surface.GetCalibrationFromBVHS('Superficie\Emely2\Frente2_mcp.bvh', 'Superficie\Emely2\Cabeca_mcp.bvh', 'Superficie\Emely2\Costas1_mcp.bvh',True,True)
#surface.GetCalibrationFromBVHS('Superficie\Paula\Frente_mcp.bvh', 'Superficie\Paula\Cabeca_mcp.bvh', 'Superficie\Paula\Costas_mcp.bvh',savefile = True,debugmode = True, minrangewide = 30, minpeakwide=40)
#surface.GetCalibrationFromBVHS('Superficie\Paula\Laura_Frente_mcp.bvh', 'Superficie\Paula\Laura_Cabecas_mcp.bvh', 'Superficie\Laura_Paula\Costas_mcp.bvh',True,True)
#surface.GetCalibrationFromBVHS('Superficie\Rodolfo3\Frente_mcp.bvh', 'Superficie\Rodolfo3\Cabeca001_mcp.bvh', 'Superficie\Rodolfo3\costas_mcp.bvh',savefile = True, debugmode = False)
#surface.GetCalibrationFromBVHS('Superficie\Rodolfo4\Frente001_mcp.bvh', 'Superficie\Rodolfo4\Cabeca_mcp.bvh', 'Superficie\Rodolfo4\Costas002_mcp.bvh',savefile = True, debugmode = True)
# surface.GetCalibrationFromBVHS('D:\Documents\Rodolfo\MoCap\Kroton\Calib_Frente_mcp.bvh', 'D:\Documents\Rodolfo\MoCap\Kroton\Calib_Cabeca_mcp.bvh', 'D:\Documents\Rodolfo\MoCap\Kroton\Calib_Costas_mcp.bvh',savefile = True, debugmode = True)
#print('Surface Calibration done. %s seconds.' % (time.time()-start))


# Path to Source Animation ###################################################
realpath = os.path.dirname(os.path.realpath(__file__))
# sourceanimations = ['Superficie/Rodolfo3/Abraco_mcp.bvh', 'Superficie/Rodolfo3/Boca001_mcp.bvh', 'Superficie/Rodolfo3/Boca_mcp.bvh', 'Superficie/Rodolfo3/Braco001_mcp.bvh', 'Superficie/Rodolfo3/Braco002_mcp.bvh', 'Superficie/Rodolfo3/Braco_mcp.bvh', 'Superficie/Rodolfo3/Cabeca001_mcp.bvh', 'Superficie/Rodolfo3/Cabeca_mcp.bvh', 'Superficie/Rodolfo3/Calma_mcp.bvh', 'Superficie/Rodolfo3/Calor_mcp.bvh', 'Superficie/Rodolfo3/Cansado_mcp.bvh', 'Superficie/Rodolfo3/Chateado001_mcp.bvh', 'Superficie/Rodolfo3/Chateado_mcp.bvh', 'Superficie/Rodolfo3/Cintura_mcp.bvh', 'Superficie/Rodolfo3/Continencia001_mcp.bvh', 'Superficie/Rodolfo3/Continencia_mcp.bvh', 'Superficie/Rodolfo3/Corpo_mcp.bvh', 'Superficie/Rodolfo3/costas_mcp.bvh', 'Superficie/Rodolfo3/Dedos_mcp.bvh', 'Superficie/Rodolfo3/Dormir_mcp.bvh', 'Superficie/Rodolfo3/Dor_mcp.bvh', 'Superficie/Rodolfo3/Entender_mcp.bvh', 'Superficie/Rodolfo3/Exercicio_mcp.bvh', 'Superficie/Rodolfo3/Frente_mcp.bvh', 'Superficie/Rodolfo3/Frio_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo001_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo_mcp.bvh', 'Superficie/Rodolfo3/Macarena_mcp.bvh', 'Superficie/Rodolfo3/Observar_mcp.bvh', 'Superficie/Rodolfo3/OlhoD_mcp.bvh', 'Superficie/Rodolfo3/OlhoE_mcp.bvh', 'Superficie/Rodolfo3/Orelha001_mcp.bvh', 'Superficie/Rodolfo3/Orelha_mcp.bvh', 'Superficie/Rodolfo3/Procurar_mcp.bvh', 'Superficie/Rodolfo3/Teste_Braco_mcp.bvh', 'Superficie/Rodolfo3/Teste_Geral_mcp.bvh']
#sourceanimations = ['Superficie\Rodolfo\Avo_mcp.bvh','Superficie\Rodolfo\Bisavo_mcp.bvh','Superficie\Rodolfo\Bota001_mcp.bvh','Superficie\Rodolfo\Bronzear_mcp.bvh','Superficie\Rodolfo\Cabeca001_mcp.bvh','Superficie\Rodolfo\Calma_mcp.bvh','Superficie\Rodolfo\Caminhada_mcp.bvh','Superficie\Rodolfo\Casas_mcp.bvh','Superficie\Rodolfo\Costas001_mcp.bvh','Superficie\Rodolfo\Edificio_mcp.bvh','Superficie\Rodolfo\Entender_mcp.bvh','Superficie\Rodolfo\Frente001_mcp.bvh','Superficie\Rodolfo\Macarena_mcp.bvh','Superficie\Rodolfo\Maos1_mcp.bvh','Superficie\Rodolfo\Maos2_mcp.bvh','Superficie\Rodolfo\Palmas1_mcp.bvh','Superficie\Rodolfo\Palmas2_mcp.bvh','Superficie\Rodolfo\Procurar_mcp.bvh','Superficie\Rodolfo\Salto001_mcp.bvh','Superficie\Rodolfo\Sapo_mcp.bvh','Superficie\Rodolfo\Sentar1_mcp.bvh','Superficie\Rodolfo\Sentar2_mcp.bvh','Superficie\Rodolfo\Surdo_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo2\Avo_mcp.bvh','Superficie\Rodolfo2\Bisavo_mcp.bvh', 'Superficie\Rodolfo2\Bota_mcp.bvh','Superficie\Rodolfo2\Bota001_mcp.bvh','Superficie\Rodolfo2\Bronzear_mcp.bvh','Superficie\Rodolfo2\Calma_mcp.bvh','Superficie\Rodolfo2\Caminhar_mcp.bvh','Superficie\Rodolfo2\Casas_mcp.bvh','Superficie\Rodolfo2\Edificio_mcp.bvh','Superficie\Rodolfo2\Entender_mcp.bvh','Superficie\Rodolfo2\Macarena_mcp.bvh','Superficie\Rodolfo2\Maos_mcp.bvh','Superficie\Rodolfo2\Maos001_mcp.bvh','Superficie\Rodolfo2\Palmas_mcp.bvh','Superficie\Rodolfo2\Palmas001_mcp.bvh','Superficie\Rodolfo2\Procurar_mcp.bvh','Superficie\Rodolfo2\Salto_mcp.bvh','Superficie\Rodolfo2\Sapo_mcp.bvh','Superficie\Rodolfo2\Sentar_mcp.bvh','Superficie\Rodolfo2\Sentar001_mcp.bvh','Superficie\Rodolfo2\Surdo_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo2\Avo_mcp.bvh','Superficie\Rodolfo2\Bisavo_mcp.bvh', 'Superficie\Rodolfo2\Bota_mcp.bvh','Superficie\Rodolfo2\Bota001_mcp.bvh','Superficie\Rodolfo2\Bronzear_mcp.bvh','Superficie\Rodolfo2\Calma_mcp.bvh','Superficie\Rodolfo2\Caminhar_mcp.bvh','Superficie\Rodolfo2\Casas_mcp.bvh','Superficie\Rodolfo2\Edificio_mcp.bvh','Superficie\Rodolfo2\Entender_mcp.bvh','Superficie\Rodolfo2\Macarena_mcp.bvh','Superficie\Rodolfo2\Maos_mcp.bvh','Superficie\Rodolfo2\Maos001_mcp.bvh','Superficie\Rodolfo2\Palmas_mcp.bvh','Superficie\Rodolfo2\Palmas001_mcp.bvh','Superficie\Rodolfo2\Procurar_mcp.bvh','Superficie\Rodolfo2\Salto_mcp.bvh','Superficie\Rodolfo2\Sapo_mcp.bvh','Superficie\Rodolfo2\Sentar_mcp.bvh','Superficie\Rodolfo2\Sentar001_mcp.bvh','Superficie\Rodolfo2\Surdo_mcp.bvh','Superficie/Rodolfo3/Abraco_mcp.bvh', 'Superficie/Rodolfo3/Boca001_mcp.bvh', 'Superficie/Rodolfo3/Boca_mcp.bvh', 'Superficie/Rodolfo3/Braco001_mcp.bvh', 'Superficie/Rodolfo3/Braco002_mcp.bvh', 'Superficie/Rodolfo3/Braco_mcp.bvh', 'Superficie/Rodolfo3/Cabeca001_mcp.bvh', 'Superficie/Rodolfo3/Cabeca_mcp.bvh', 'Superficie/Rodolfo3/Calma_mcp.bvh', 'Superficie/Rodolfo3/Calor_mcp.bvh', 'Superficie/Rodolfo3/Cansado_mcp.bvh', 'Superficie/Rodolfo3/Chateado001_mcp.bvh', 'Superficie/Rodolfo3/Chateado_mcp.bvh', 'Superficie/Rodolfo3/Cintura_mcp.bvh', 'Superficie/Rodolfo3/Continencia001_mcp.bvh', 'Superficie/Rodolfo3/Continencia_mcp.bvh', 'Superficie/Rodolfo3/Corpo_mcp.bvh', 'Superficie/Rodolfo3/costas_mcp.bvh', 'Superficie/Rodolfo3/Dedos_mcp.bvh', 'Superficie/Rodolfo3/Dormir_mcp.bvh', 'Superficie/Rodolfo3/Dor_mcp.bvh', 'Superficie/Rodolfo3/Entender_mcp.bvh', 'Superficie/Rodolfo3/Exercicio_mcp.bvh', 'Superficie/Rodolfo3/Frente_mcp.bvh', 'Superficie/Rodolfo3/Frio_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo001_mcp.bvh', 'Superficie/Rodolfo3/Lavar_Cabelo_mcp.bvh', 'Superficie/Rodolfo3/Macarena_mcp.bvh', 'Superficie/Rodolfo3/Observar_mcp.bvh', 'Superficie/Rodolfo3/OlhoD_mcp.bvh', 'Superficie/Rodolfo3/OlhoE_mcp.bvh', 'Superficie/Rodolfo3/Orelha001_mcp.bvh', 'Superficie/Rodolfo3/Orelha_mcp.bvh', 'Superficie/Rodolfo3/Procurar_mcp.bvh', 'Superficie/Rodolfo3/Teste_Braco_mcp.bvh', 'Superficie/Rodolfo3/Teste_Geral_mcp.bvh']
#sourceanimations = ['Superficie\Paula\Avo_mcp.bvh', 'Superficie\Paula\Avo001_mcp.bvh','Superficie\Paula\Bisavo_mcp.bvh', 'Superficie\Paula\Bota_mcp.bvh','Superficie\Paula\Bronzear_mcp.bvh','Superficie\Paula\Cabeca_mcp.bvh','Superficie\Paula\Costas_mcp.bvh','Superficie\Paula\Danca_mcp.bvh','Superficie\Paula\Edificio_mcp.bvh','Superficie\Paula\Entender_mcp.bvh', 'Superficie\Paula\Frente_mcp.bvh','Superficie\Paula\Macarena_mcp.bvh','Superficie\Paula\Palmas_mcp.bvh','Superficie\Paula\Polichinelo_mcp.bvh','Superficie\Paula\Pulo_mcp.bvh','Superficie\Paula\Sapo_mcp.bvh','Superficie\Paula\Sentar_mcp.bvh','Superficie\Paula\Surdo_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo4\Abraco_mcp.bvh', 'Superficie\Rodolfo4\Avo_mcp.bvh', 'Superficie\Rodolfo4\Baleia_mcp.bvh', 'Superficie\Rodolfo4\Barriga001_mcp.bvh', 'Superficie\Rodolfo4\Barriga2_mcp.bvh', 'Superficie\Rodolfo4\Barriga_mcp.bvh', 'Superficie\Rodolfo4\Bisavo_mcp.bvh', 'Superficie\Rodolfo4\Boca_mcp.bvh', 'Superficie\Rodolfo4\Bronzear001_mcp.bvh', 'Superficie\Rodolfo4\Bronzear_mcp.bvh', 'Superficie\Rodolfo4\Cabeca_mcp.bvh', 'Superficie\Rodolfo4\Cabelo_mcp.bvh', 'Superficie\Rodolfo4\Casas001_mcp.bvh', 'Superficie\Rodolfo4\Casas_mcp.bvh', 'Superficie\Rodolfo4\Chocado_mcp.bvh', 'Superficie\Rodolfo4\Continencia_mcp.bvh', 'Superficie\Rodolfo4\Costas001_mcp.bvh', 'Superficie\Rodolfo4\Costas002_mcp.bvh', 'Superficie\Rodolfo4\Costas_mcp.bvh', 'Superficie\Rodolfo4\Dance_mcp.bvh', 'Superficie\Rodolfo4\Entender001_mcp.bvh', 'Superficie\Rodolfo4\Entender_mcp.bvh', 'Superficie\Rodolfo4\Esconder_mcp.bvh', 'Superficie\Rodolfo4\Frente001_mcp.bvh', 'Superficie\Rodolfo4\Frente_mcp.bvh', 'Superficie\Rodolfo4\Girar_mcp.bvh', 'Superficie\Rodolfo4\Juramento_mcp.bvh', 'Superficie\Rodolfo4\Mao_na_frente_mcp.bvh', 'Superficie\Rodolfo4\Observar001_mcp.bvh', 'Superficie\Rodolfo4\Observar_mcp.bvh', 'Superficie\Rodolfo4\Olhos2_mcp.bvh', 'Superficie\Rodolfo4\Olhos3_mcp.bvh', 'Superficie\Rodolfo4\Olho_mcp.bvh', 'Superficie\Rodolfo4\Pescoco_mcp.bvh', 'Superficie\Rodolfo4\Preocupado001_mcp.bvh', 'Superficie\Rodolfo4\Preocupado_mcp.bvh', 'Superficie\Rodolfo4\Relaxar_mcp.bvh', 'Superficie\Rodolfo4\Sapo_mcp.bvh', 'Superficie\Rodolfo4\Surdo_mcp.bvh']
#sourceanimations = [os.path.join(realpath, sourceanimations[i]) for i in range(len(sourceanimations))]

sourceanimations = ['..\..\Superficie\Rodolfo4\Mao_na_frente_mcp.bvh']
#sourceanimations = ['D:\Documents\Rodolfo\MoCap\Kroton\Calib_Texto001_mcp.bvh']

# Path to Source Calibration #################################################
# sourcesurface = os.path.join(realpath, 'Superficie\Rodolfo\surface_rodolfo.txt')
# sourcesurface = os.path.join(realpath, 'Superficie\Rodolfo3\surface_rodolfo.txt')
# sourcesurface = os.path.join(realpath, 'Superficie\Rodolfo4\surface_rodolfo.txt')
#sourcesurface = os.path.join(realpath, 'Superficie\Emely2\surface_emely2.txt')
#sourcesurface = os.path.join(realpath, 'Superficie\Paula\surface_1.txt')
#sourcesurface = os.path.join(realpath, 'D:\Documents\Rodolfo\MoCap\Kroton\surface_soraia.txt')
sourcesurface = os.path.join(realpath, '..\..\Superficie\Rodolfo4\surface_rodolfo.txt')

# Path to Target TPose BVH File ##############################################
# Path to Target Calibration File ############################################
# Path to Target Skeletom Map File ###########################################
info_path = os.path.join(realpath, '..\data\input')
targettpose = os.path.join(info_path, 'tpose_talita.bvh')
targetsurface = os.path.join(info_path, 'surface_talita.csv')
skeletonmappath = None #Use standard
# targettpose = os.path.join(realpath, 'AragorTPose.bvh')
# targetsurface = os.path.join(realpath, 'surface_aragor.csv')
# skeletonmappath = os.path.join(realpath, 'skeletonmap_aragor.csv')
# targettpose = os.path.join(realpath, 'GremlinTPose2.bvh')
# targetsurface = os.path.join(realpath, 'surface_gremlin.csv')
# skeletonmappath = os.path.join(realpath, 'skeletonmap_gremlin.csv')

out_path = os.path.join(realpath, '..\data\output')
if not os.path.exists(out_path):
    os.makedirs(out_path)
for path in sourceanimations:
    starttime=time.time()
    #Get only surface
    # tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = False, computeIK = False, adjustOrientation = False, saveFile = False, saveInitAndFull = False)
    #ComputeEgo
    # tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = True, computeIK = False, adjustOrientation = False, saveFile = False, saveInitAndFull = False)
    # Get everything, dont save
    # tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = True, computeIK = True, adjustOrientation = True, saveFile = False, saveInitAndFull = False)
    # Everything and save
    MotionRetargeting.importance = []
    tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath, computeEgo = True, computeIK = True, adjustOrientation = True, saveFile = True, saveInitAndFull = True)
    print('Total time: %s seconds.' % (time.time()-starttime))
    mathutils.printLog()


#tgtAnim.PlotAnimation(tgtSurf)



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
