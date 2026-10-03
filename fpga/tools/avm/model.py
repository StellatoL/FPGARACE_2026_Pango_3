"""Bit-exact AVM arithmetic and camera geometry. See implementation-contract.md."""
from __future__ import annotations
import itertools
import numpy as np
import cv2

CAMERAS = ('front', 'back', 'left', 'right')


def round_shift(n, shift):
    n = np.asarray(n, dtype=np.int64)
    if shift == 0:
        return n
    return np.sign(n) * ((np.abs(n) + (1 << (shift - 1))) >> shift)


def quantize_coords(uv):
    uv = np.nan_to_num(uv, nan=0, posinf=4095, neginf=-4096)
    return np.clip(np.sign(uv) * np.floor(np.abs(uv) * 256 + .5), -(1 << 20), (1 << 20)-1).astype(np.int64)


def pack565(rgb):
    x = np.clip(rgb, 0, 255).astype(np.uint16)
    return ((x[..., 0] >> 3) << 11) | ((x[..., 1] >> 2) << 5) | (x[..., 2] >> 3)


def expand565(p):
    p = np.asarray(p, dtype=np.int64)
    r, g, b = p >> 11, (p >> 5) & 63, p & 31
    return np.stack(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)), -1)


def grid_interpolate(nodes, step):
    """Return 16x16 Q8 coordinates with horizontal, then vertical rounding."""
    yy, xx = np.mgrid[:16, :16]
    ix, iy = xx // step, yy // step
    fx, fy = (xx % step)[..., None], (yy % step)[..., None]
    shift = step.bit_length()-1
    h0 = round_shift(nodes[iy, ix]*(step-fx)+nodes[iy, ix+1]*fx, shift)
    h1 = round_shift(nodes[iy+1, ix]*(step-fx)+nodes[iy+1, ix+1]*fx, shift)
    return round_shift(h0*(step-fy)+h1*fy, shift)


def sample565(frame, coords):
    uv = np.asarray(coords, dtype=np.int64)
    x, y = uv[..., 0] >> 8, uv[..., 1] >> 8
    valid = (x >= 0) & (y >= 0) & (x+1 < frame.shape[1]) & (y+1 < frame.shape[0])
    x = np.clip(x, 0, frame.shape[1]-2)
    y = np.clip(y, 0, frame.shape[0]-2)
    fx, fy = (uv[..., 0] & 255)[..., None], (uv[..., 1] & 255)[..., None]
    h0 = expand565(frame[y, x])*(256-fx)+expand565(frame[y, x+1])*fx
    h1 = expand565(frame[y+1, x])*(256-fx)+expand565(frame[y+1, x+1])*fx
    result = (h0*(256-fy)+h1*fy+32768) >> 16
    return np.where(valid[..., None], result, 0), valid


def sample_float(frame, coords):
    x, y = np.floor(coords[..., 0]).astype(int), np.floor(coords[..., 1]).astype(int)
    fx, fy = (coords[..., 0]-x)[..., None], (coords[..., 1]-y)[..., None]
    valid = (x >= 0) & (y >= 0) & (x+1 < frame.shape[1]) & (y+1 < frame.shape[0])
    x, y = np.clip(x, 0, frame.shape[1]-2), np.clip(y, 0, frame.shape[0]-2)
    p = expand565(frame)
    out = (p[y, x]*(1-fx)+p[y, x+1]*fx)*(1-fy)+(p[y+1, x]*(1-fx)+p[y+1, x+1]*fx)*fy
    return np.where(valid[..., None], out, 0)


def gain_blend(a, b, ga, gb, alpha, va=True, vb=True):
    aa = np.minimum((np.asarray(a, np.int64)*np.asarray(ga)+2048) >> 12, 255)
    bb = np.minimum((np.asarray(b, np.int64)*np.asarray(gb)+2048) >> 12, 255)
    aa, bb = np.where(np.asarray(va)[..., None], aa, 0), np.where(np.asarray(vb)[..., None], bb, 0)
    alpha = np.where(va, np.where(vb, alpha, 256), 0)
    return pack565((aa*np.asarray(alpha)[..., None]+bb*(256-np.asarray(alpha)[..., None])+128) >> 8)


def fisheye_project(camera_xyz, K, D):
    """OpenCV fisheye model, no intermediate undistorted image."""
    z = camera_xyz[..., 2]
    xy = camera_xyz[..., :2]/np.where(np.abs(z[..., None]) > 1e-12, z[..., None], 1e-12)
    r = np.linalg.norm(xy, axis=-1)
    theta = np.arctan(r)
    t2 = theta*theta
    td = theta*(1+t2*(D[0]+t2*(D[1]+t2*(D[2]+t2*D[3]))))
    ratio = np.divide(td, r, out=np.ones_like(r), where=r > 1e-12)
    return np.stack((K[0, 0]*xy[..., 0]*ratio+K[0, 2], K[1, 1]*xy[..., 1]*ratio+K[1, 2]), -1)


def unproject_rays(raw_uv, K, D):
    """Independent raw ray inversion, used for rendering, not the remap generator."""
    q = (raw_uv-np.array([K[0, 2], K[1, 2]]))/np.array([K[0, 0], K[1, 1]])
    td = np.linalg.norm(q, axis=-1)
    theta = np.minimum(td.copy(), 1.55)
    for _ in range(12):
        t2 = theta*theta
        f = theta*(1+t2*(D[0]+t2*(D[1]+t2*(D[2]+t2*D[3]))))-td
        df = 1+t2*(3*D[0]+t2*(5*D[1]+t2*(7*D[2]+t2*9*D[3])))
        theta = np.clip(theta-f/df, 0, 1.56)
    scale = np.divide(np.sin(theta), td, out=np.ones_like(td), where=td > 1e-12)
    rays = np.concatenate((q*scale[..., None], np.cos(theta)[..., None]), -1)
    return rays, td < (1.56*(1+sum(D[k]*1.56**(2*k+2) for k in range(4))))


def ground_texture(x, y):
    """Continuous ground truth: metre grid, road/parking lines and asymmetric marks."""
    rgb = np.stack((80+14*np.sin(x*.9)+8*np.cos(y), 95+12*np.cos(x*.6+y*.4), 100+9*np.sin(y*.8)), -1)
    grid = (np.abs(x-np.round(x)) < .025) | (np.abs(y-np.round(y)) < .025)
    rgb[grid] = (155, 160, 165)
    road = (np.abs(np.abs(x)-2.5) < .045) | ((np.abs(y-3.4) < .045) & (np.abs(x) > 1))
    rgb[road] = (230, 220, 175)
    # Four colored direction markers at known metric positions.
    for px, py, color in [(0, 3, (220, 60, 60)), (0, -3, (60, 100, 220)), (-3, 0, (220, 175, 40)), (3, 0, (60, 190, 80))]:
        mask = (x-px)**2+(y-py)**2 < .25**2
        rgb[mask] = color
    return np.clip(np.floor(rgb+.5), 0, 255).astype(np.uint8)


def synthetic_cameras(width, height):
    centers = [[0, 1.2, 1.2], [0, -1.2, 1.2], [-.65, 0, 1.2], [.65, 0, 1.2]]
    directions = [[0, 1, -.8], [0, -1, -.8], [-1, 0, -.8], [1, 0, -.8]]
    cameras = []
    for name, center, direction in zip(CAMERAS, centers, directions):
        z = np.array(direction)/np.linalg.norm(direction)
        x = np.cross(z, [0, 0, 1]); x /= np.linalg.norm(x)
        y = np.cross(z, x)
        R = np.stack((x, y, z))
        K = np.array([[width*.28, 0, (width-1)/2], [0, width*.28, (height-1)/2], [0, 0, 1.]])
        D = np.array([-.045, .008, -.001, .0001])
        cameras.append(dict(name=name, K=K, D=D, R=R, t=-R@center, center=np.array(center), model='opencv_fisheye'))
    return cameras


def synthetic_map(camera, xx, yy, width, height):
    x = (xx-(width-1)/2)*16/width
    y = ((height-1)/2-yy)*16/width
    world = np.stack((x, y, np.zeros_like(x)), -1)
    cam = world@camera['R'].T+camera['t']
    uv = fisheye_project(cam, camera['K'], camera['D'])
    valid = (cam[..., 2] > .05) & ~((np.abs(x) < .8) & (np.abs(y) < 1.4))
    return uv, valid


def render_synthetic(camera, width, height, exposure):
    yy, xx = np.mgrid[:height, :width]
    rays, lens_valid = unproject_rays(np.stack((xx, yy), -1), camera['K'], camera['D'])
    world_rays = rays@camera['R']
    distance = -camera['center'][2]/np.where(np.abs(world_rays[..., 2]) > 1e-8, world_rays[..., 2], 1e-8)
    pts = camera['center']+distance[..., None]*world_rays
    rgb = ground_texture(pts[..., 0], pts[..., 1]).astype(float)*exposure
    valid = lens_valid & (distance > 0) & (distance < 100)
    rgb[~valid] = (25, 35, 50)
    return pack565(np.floor(np.clip(rgb, 0, 255)+.5))


def geometry_check(cameras):
    """Independent OpenCV forward projections versus analytic ray/plane inversion."""
    errors = []
    for cam in cameras:
        points = np.array([[x, y, 0.] for x in [-6, -3, 0, 3, 6] for y in [-4, -2, 0, 2, 4]])
        cc = points@cam['R'].T+cam['t']
        points = points[cc[:, 2] > .1]
        rvec, _ = cv2.Rodrigues(cam['R'])
        uv, _ = cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), rvec, cam['t'], cam['K'], cam['D'])
        rays, valid = unproject_rays(uv[:, 0], cam['K'], cam['D'])
        rays = rays@cam['R']
        distance = -cam['center'][2]/rays[:, 2]
        recovered = cam['center']+distance[:, None]*rays
        errors.extend(np.linalg.norm(recovered[valid]-points[valid], axis=-1).tolist())
    return dict(control_points=len(errors), max_ray_plane_error_m=max(errors))


def bounded_log_gains(means, counts, previous=None, smoothing=.1):
    """Solve each RGB channel exactly by enumerating bound-active sets (4 cameras).

    means[(a,b)] is two pre-gain RGB means; missing/dark/saturated edges are removed.
    Equality sum(log(g))=0 and bounds are enforced together, not by post-hoc clipping.
    """
    previous = np.ones((4, 3)) if previous is None else np.asarray(previous, float)
    result = previous.copy()
    bounds = np.log(2.)
    held = []
    for channel in range(3):
        rows, targets, weights = [], [], []
        for pair, mean in means.items():
            ma, mb = np.asarray(mean)[:, channel]
            count = counts[pair]
            if count < 16 or min(ma, mb) < 8 or max(ma, mb) > 245:
                continue
            row = np.zeros(4); row[pair[0]], row[pair[1]] = 1, -1
            rows.append(row); targets.append(np.log(mb/ma)); weights.append(np.sqrt(count))
        A = np.asarray(rows).reshape(-1, 4)
        if np.linalg.matrix_rank(np.vstack((A, np.ones(4)))) < 4:
            held.append(channel); continue
        w = np.asarray(weights); w /= w.max()
        A = A*w[:, None]; b = np.array(targets)*w
        best, best_cost = None, float('inf')
        for status in itertools.product((-1, 0, 1), repeat=4):
            st = np.array(status); free = np.flatnonzero(st == 0); fixed = np.flatnonzero(st != 0)
            x = st*bounds
            if len(free):
                Af, bf = A[:, free], b-A[:, fixed]@x[fixed]
                n = len(free)
                K = np.block([[Af.T@Af, np.ones((n, 1))], [np.ones((1, n)), np.zeros((1, 1))]])
                rhs = np.r_[Af.T@bf, -x[fixed].sum()]
                sol = np.linalg.lstsq(K, rhs, rcond=None)[0]
                x[free] = sol[:-1]
            if abs(x.sum()) > 1e-8 or np.max(np.abs(x)) > bounds+1e-8:
                continue
            cost = np.linalg.norm(A@x-b)**2
            if cost < best_cost:
                best, best_cost = x, cost
        if best is None:
            raise RuntimeError('Bounded gain solver found no feasible solution')
        result[:, channel] = np.exp((1-smoothing)*np.log(previous[:, channel])+smoothing*best)
    return result, held


def image_metrics(a, b, mask=None):
    a, b = np.asarray(a, float), np.asarray(b, float)
    mask = np.ones(a.shape[:2], bool) if mask is None else mask
    mse = float(np.mean((a[mask]-b[mask])**2))
    mu_a, mu_b = cv2.GaussianBlur(a, (11, 11), 1.5), cv2.GaussianBlur(b, (11, 11), 1.5)
    var_a = cv2.GaussianBlur(a*a, (11, 11), 1.5)-mu_a*mu_a
    var_b = cv2.GaussianBlur(b*b, (11, 11), 1.5)-mu_b*mu_b
    cov = cv2.GaussianBlur(a*b, (11, 11), 1.5)-mu_a*mu_b
    ssim = ((2*mu_a*mu_b+6.5025)*(2*cov+58.5225))/((mu_a*mu_a+mu_b*mu_b+6.5025)*(var_a+var_b+58.5225))
    return dict(mse=mse, psnr_db=10*np.log10(255**2/mse) if mse else None, ssim=float(ssim[mask].mean()))
