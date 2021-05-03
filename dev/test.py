import os
from retarget import MotionRetargeting
from surface import GetCalibrationFromBVHS
import time
import mathutils


# Generate Surface Calibration ###############################################
def GenerateSurfaceCalibration():
    start = time.time()
    GetCalibrationFromBVHS('..\\data\\surface\\calibration_takes\\rodolfo\\Frente001_mcp.bvh', '..\\data\\surface\\calibration_takes\\rodolfo\\Cabeca_mcp.bvh', '..\\data\\surface\\calibration_takes\\rodolfo\\Costas002_mcp.bvh', savefile=True, debugmode=True)
    print('Surface Calibration done. %s seconds.' % (time.time()-start))


def SurfaceMotionRetargeting(computeEgo=True, computeIK=True, adjustOrientation=True, saveFile=True, saveInitAndFull=True):
    targettpose = os.path.abspath('..\\data\\input\\tpose_talita.bvh')
    targetsurface = os.path.abspath('..\\data\\surface\\surface_talita.csv')
    skeletonmappath = None  # Use standard
    sourcesurface = os.path.abspath('..\\data\\surface\\surface_rodolfo.txt')

    sourceanimations = ['..\\data\\input\\Mao_na_frente_mcp.bvh']

    out_path = os.path.abspath('..\\data\\output')
    if not os.path.exists(out_path):
        os.makedirs(out_path)
    for path in sourceanimations:
        starttime = time.time()
        MotionRetargeting.importance = []
        tgtAnim, tgtSurf, srcAnim, srcSurf, tgtAnim_onlyInitial, ego = MotionRetargeting(path, sourcesurface, targettpose, targetsurface, skeletonmappath,  computeEgo=computeEgo, computeIK=computeIK, adjustOrientation=adjustOrientation, saveFile=saveFile, saveInitAndFull=saveInitAndFull, out_path=out_path)
        print('Total time: %s seconds.' % (time.time()-starttime))
        mathutils.printLog()


# GenerateSurfaceCalibration()
SurfaceMotionRetargeting()
print('Done')
