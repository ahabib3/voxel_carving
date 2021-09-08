import numpy as np
import open3d as o3d
import cv2

BASE_DIR = "./temple/temple"
PARAMS = open("./temple/temple_par.txt").readlines()

sum = 0
voxel_dim = 200
voxel_size = 0.005
voxel_grid = np.ones((voxel_dim, voxel_dim, voxel_dim))

def extract_foreground_from_image(num):
    # reference: http://creativemorphometrics.co.vu/blog/2014/08/05/automated-outlines-with-opencv-in-python
    image = cv2.cvtColor(cv2.imread(BASE_DIR+str(format(num, "04"))+".png"), cv2.COLOR_BGR2GRAY)
    r, threshold = cv2.threshold(image,10,255,cv2.THRESH_BINARY)
    kernel = np.ones((5,5),np.uint8)
    img = cv2.morphologyEx(cv2.erode(threshold, kernel,iterations = 1), cv2.MORPH_OPEN, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    # plt.imshow(img, 'gray')
    # plt.show()
    # img = Image.open(BASE_DIR+str(format(num, "04"))+".png").convert('1').filter(ImageFilter.BLUR).filter(ImageFilter.MinFilter(3)).filter(ImageFilter.MinFilter)
    img = np.asarray(img, dtype='float32')
    return img

def build_matrices(num):
    camdata = PARAMS[num+1].split()[1:]
    K = np.reshape(np.array(camdata[0:9]).astype(float), (3, 3))
    R = np.reshape(np.array(camdata[9:18]).astype(float), (3, 3))
    t = np.reshape(np.array(camdata[18:21]).astype(float), (3, 1))
    return K, R, t



num = 1

for i in range(0, 10):
    img = extract_foreground_from_image(num)
    K, R, t = build_matrices(num)

    print("done setting camera params")

    for i in range(0, voxel_dim):
        for j in range(0, voxel_dim):
            for k in range(0, voxel_dim):
                v = np.reshape(np.array([i*voxel_size, j*voxel_size, k*voxel_size]), (3, 1))
                projected_v = np.dot(R, v) + t
                projected_v = np.dot(K, projected_v)
                v[0] = v[0]/v[2]
                v[1] = v[1]/v[2]
                # print(projected_v)
                projected_v = np.round(projected_v).astype(int)
                if (projected_v[0] > 0 and projected_v[1] > 0 and projected_v[0] < img.shape[0] and projected_v[1] < img.shape[1]):
                    sum = sum+1
                    if img[projected_v[0], projected_v[1]].sum() == 0:
                        #if the background is black, then carve
                        voxel_grid[i, j, k] = 0
                else:
                    #if the voxels are outside the image bounds, then carve
                    voxel_grid[i, j, k] = 0

    print(voxel_grid.sum())
    print(sum)
    num=num+1


points = []
for i in range(0, voxel_dim):
    for j in range(0, voxel_dim):
        for k in range(0, voxel_dim):
            if voxel_grid[i, j, k] > 0:
                points.append([i*voxel_size, j*voxel_size, k*voxel_size])
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
o3d.visualization.draw_geometries([pcd])

