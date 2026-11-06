"""Point cleaning and registration for merged cross-laser points.

consistency(P, voxel, min_sources): keep a point only if at least `min_sources`
independent views (scan x arm) have points in its voxel or the 26 around it.
Reflections usually show in one view; real surfaces, walls included, in several.

sor(xyz, k, std): Open3D statistical outlier removal (isolated specks).

register_icp(A, B, init): rigid fit of camera B's points onto camera A's using raised
features (heights > min_h), Open3D point-to-plane ICP from the grid-shift guess;
returns the XY translation and yaw (Z left to the height calibration).
"""
import numpy as np


def _vkey(ijk):
    return (ijk[:, 0] * 1_000_003 + ijk[:, 1]) * 1_000_003 + ijk[:, 2]


def consistency(xyz, view, voxel=1.0, min_sources=2):
    """Boolean mask of points seen by >= min_sources distinct views around their voxel."""
    ijk = np.floor(xyz / voxel).astype(np.int64)
    ijk -= ijk.min(0) - 1
    key = _vkey(ijk)
    # occupied (voxel, view) pairs -> per voxel a bit mask of views
    nv = int(view.max()) + 1
    assert nv <= 62
    uk, inv = np.unique(key, return_inverse=True)
    bits = np.zeros(len(uk), np.int64)
    np.bitwise_or.at(bits, inv, (np.int64(1) << view.astype(np.int64)))
    uijk = ijk[np.unique(inv, return_index=True)[1]]  # one ijk per unique key, same order as uk
    # neighbourhood union of view masks
    acc = bits.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == dy == dz == 0:
                    continue
                nk = _vkey(uijk + np.array([dx, dy, dz]))
                pos = np.searchsorted(uk, nk)
                pos = np.clip(pos, 0, len(uk) - 1)
                hit = uk[pos] == nk
                acc[hit] |= bits[pos[hit]]
    # popcount
    cnt = np.zeros(len(uk), np.int64)
    a = acc.copy()
    while a.any():
        cnt += a & 1
        a >>= 1
    return cnt[inv] >= min_sources


def sor(xyz, k=20, std=2.5):
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
    _, ind = pc.remove_statistical_outlier(nb_neighbors=k, std_ratio=std)
    m = np.zeros(len(xyz), bool)
    m[np.asarray(ind)] = True
    return m


def register_icp(A_xyz, B_xyz, init_shift, min_h=4.0, voxel=0.75, max_dist=2.0):
    """B -> A rigid transform restricted to XY translation + yaw, from raised points."""
    import open3d as o3d

    def cloud(p):
        p = p[p[:, 2] > min_h]
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p.astype(np.float64)))
        pc = pc.voxel_down_sample(voxel)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3 * voxel, max_nn=30))
        return pc

    a, b = cloud(A_xyz), cloud(B_xyz)
    T0 = np.eye(4)
    T0[:2, 3] = init_shift
    res = o3d.pipelines.registration.registration_icp(
        b, a, max_dist, T0, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    T = res.transformation
    yaw = np.arctan2(T[1, 0], T[0, 0])
    return {"shift": [float(T[0, 3]), float(T[1, 3])], "yaw_deg": float(np.degrees(yaw)), "dz": float(T[2, 3]),
            "tilt_deg": float(np.degrees(np.arccos(np.clip(T[2, 2], -1, 1)))),
            "fitness": float(res.fitness), "rmse": float(res.inlier_rmse), "T": T.tolist()}


def apply_xy_yaw(xy, reg):
    c, s = np.cos(np.radians(reg["yaw_deg"])), np.sin(np.radians(reg["yaw_deg"]))
    out = np.empty_like(xy)
    out[:, 0] = c * xy[:, 0] - s * xy[:, 1] + reg["shift"][0]
    out[:, 1] = s * xy[:, 0] + c * xy[:, 1] + reg["shift"][1]
    return out
