def absolute_orientation(points_in_left, points_in_right):
    """
    point left should be the image space markers

    point right should be the robot base reference for the markers
    """

    import numpy as np

    num_points = len(points_in_left)
    dim_points = len(points_in_left[0])

    left_mat = np.array(points_in_left).T
    right_mat = np.array(points_in_right).T

    left_mean = left_mat.mean(1)
    right_mean = right_mat.mean(1)

    left_M = left_mat - np.tile(left_mean, (num_points, 1)).T
    right_M = right_mat - np.tile(right_mean, (num_points, 1)).T

    M = left_M.dot(right_M.T)

    U, S, Vt = np.linalg.svd(M)

    V = Vt.T

    R = V.dot(np.diag((1,1,np.linalg.det(U.dot(V))))).dot(U.T)

    t = right_mean - R.dot(left_mean)

    return R, t

if __name__ == "__main__":
    # Example usage
    point_voxel = [
        [3, 3, 0],
        [21, 3, 0],
        [21, 27, 0],
        [3, 27, 0],
        [17, 13, 0]
    ]
    
    points_in_image = []
    for pose in point_voxel:
        x, y, z = pose
        points_in_image.append([x*0.8/100, y*0.8/100, z*0.32/100])
            
    print(points_in_image)
    
    points_in_robot = [[0.532254, -0.115884, 0.00262767], 
                       [0.391544, -0.115448, 0.00187763], 
                       [0.390487, 0.0806181, 0.00305815], 
                       [0.533372, 0.0788155, 0.00324756], 
                       [0.430024, -0.0320096, 0.00302584]]
    

    
    R, t = absolute_orientation(points_in_image, points_in_robot)
    print("Rotation matrix:")
    print(R)
    print("Translation vector:")
    print(t)