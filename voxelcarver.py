import numpy as np
from matplotlib import image
from matplotlib import pyplot as plt
import open3d as o3d
import xml.etree.ElementTree as ET

voxel_dim = 50
voxel_grid = np.ones((voxel_dim, voxel_dim, voxel_dim))

def parseCameraFile(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    cam = root[0].attrib

    #translation vector
    t = cam.get('TranslationVector')
    t = np.array(t.split()).astype(float)
    t = np.reshape(t, (4, 1))

    #rotation matrix
    r = cam.get('RotationMatrix')
    r = np.array(r.split()).astype(float)
    r = np.reshape(r, (4, 4))

    Rt = np.hstack((r[:,0:3], t))

    f = float(cam.get('FocalMm'))
    o = cam.get('CenterPx')
    o = np.array(o.split()).astype(float)

    K = np.asmatrix(np.reshape(np.array([f, 0, o[0], 0, 0, f, o[1], 0, 0, 0, 1, 0]), (3, 4)))

    return np.dot(K, Rt)



def carve(filename):
    cameraFile = filename+".xml"
    imgFile = filename+".png"

    M = parseCameraFile(cameraFile)

    img = image.imread(imgFile)
    image_dim = img.shape

    sum = 0
    for i in range(0, voxel_dim):
        for j in range(0, voxel_dim):
            for k in range(0, voxel_dim):
                v = np.reshape(np.array([i, j, k, 1]), (4, 1))
                projected_v = np.dot(M, v)
                # print(projected_v)
                if (projected_v[0] > 0 and projected_v[1] > 0 and projected_v[0] < image_dim[0] and projected_v[1] < image_dim[1]):
                    projected_v = projected_v.astype(int)
                    sum = sum+1
                    #print(img[projected_v[0], projected_v[1]])
                    if img[projected_v[0], projected_v[1]].sum() == 0:
                        #if the background is black, then carve
                        voxel_grid[i, j, k] = 0
                else:
                    #if the background is black, then carve
                    voxel_grid[i, j, k] = 0

    print(voxel_grid.sum())
    print(sum)


carve("/Users/aalia/Desktop/bean_img01")
# carve("/Users/aalia/Desktop/bean_img02")
points = []
for i in range(0, voxel_dim):
    for j in range(0, voxel_dim):
        for k in range(0, voxel_dim):
            if voxel_grid[i, j, k] > 0:
                points.append([i, j, k])

points = np.matrix(points, dtype='float')
print(points.shape)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
o3d.visualization.draw_geometries([pcd])
