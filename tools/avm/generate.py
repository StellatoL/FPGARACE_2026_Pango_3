"""Generate AVM camera frames, adaptive LUT DDR images and bit-exact outputs.

python tools/avm/generate.py --case small|synthetic720|public [--matlab]
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import urllib.request
import numpy as np
import cv2
from scipy.io import savemat
from model import (CAMERAS, quantize_coords, pack565, expand565, grid_interpolate,
                   sample565, sample_float, gain_blend, fisheye_project,
                   synthetic_cameras, synthetic_map, render_synthetic,
                   geometry_check, image_metrics, bounded_log_gains)

ROOT = Path(__file__).resolve().parents[2]
REVISION = '0c40d7e12443a7f16b2adae6f79fae2f6d6ee071'
BASE_URL = f'https://raw.githubusercontent.com/neozhaoliang/surround-view-system-introduction/{REVISION}/'


def download_public(destination):
    destination.mkdir(parents=True, exist_ok=True)
    pinned = json.loads((Path(__file__).with_name('public_source_manifest.json')).read_text(encoding='utf-8'))
    names = ['LICENSE', 'README.md', 'surround_view/param_settings.py', 'surround_view/fisheye_camera.py']
    names += [f'{folder}/{name}.{ext}' for name in CAMERAS for folder, ext in [('images', 'png'), ('yaml', 'yaml')]]
    hashes = {}
    for name in names:
        path = destination/name
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            with urllib.request.urlopen(BASE_URL+name, timeout=60) as response:
                data = response.read()
            path.write_bytes(data)
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        if hashes[name] != pinned['files_sha256'][name]:
            raise ValueError(f'Pinned source SHA256 mismatch: {path}')
    provenance = dict(repository='https://github.com/neozhaoliang/surround-view-system-introduction',
                      revision=REVISION, license='MIT', files_sha256=hashes)
    (destination/'provenance.json').write_text(json.dumps(provenance, indent=2), encoding='utf-8')
    return provenance


def public_cameras(destination):
    cameras, frames = [], []
    for name in CAMERAS:
        fs = cv2.FileStorage(str(destination/f'yaml/{name}.yaml'), cv2.FILE_STORAGE_READ)
        K, D = fs.getNode('camera_matrix').mat(), fs.getNode('dist_coeffs').mat().ravel()
        P = K.copy()
        scale, shift = fs.getNode('scale_xy').mat().ravel(), fs.getNode('shift_xy').mat().ravel()
        P[0, 0] *= scale[0]; P[1, 1] *= scale[1]
        P[0, 2] += shift[0]; P[1, 2] += shift[1]
        H = fs.getNode('project_matrix').mat(); fs.release()
        cameras.append(dict(name=name, K=K, D=D, P=P, H=H, model='opencv_fisheye_project_adapter'))
        frame = cv2.imread(str(destination/f'images/{name}.png'))
        if frame is None:
            raise ValueError(f'Cannot decode public image {name}')
        frames.append(pack565(frame[..., ::-1]))
    return cameras, frames


def public_map(cam, xx, yy, width, height):
    scale = min(width/1200, height/1600)
    x = (xx-(width-1200*scale)/2)/scale
    y = (yy-(height-1600*scale)/2)/scale
    inside = (x >= 0) & (x <= 1199) & (y >= 0) & (y <= 1599)
    name = cam['name']
    if name == 'front':
        px, py, region = x, y, y < 550
    elif name == 'back':
        px, py, region = 1199-x, 1599-y, y >= 1050
    elif name == 'left':
        px, py, region = 1599-y, x, x < 500
    else:
        px, py, region = y, 1199-x, x >= 700
    q = np.stack((px, py, np.ones_like(px)), -1)@np.linalg.inv(cam['H']).T
    q = q/np.where(np.abs(q[..., 2:]) > 1e-12, q[..., 2:], 1e-12)
    ray = q@np.linalg.inv(cam['P']).T
    uv = fisheye_project(ray, cam['K'], cam['D'])
    return uv, inside & region


def build_grid(cameras, map_fn, width, height, source_width, source_height):
    padded_w, padded_h = (width+15)//16*16, (height+15)//16*16
    yy, xx = np.mgrid[:padded_h, :padded_w]
    mapped, validity = [], []
    for cam in cameras:
        uv, valid = map_fn(cam, xx, yy, width, height)
        valid &= np.isfinite(uv).all(-1) & (uv[..., 0] >= 0) & (uv[..., 1] >= 0)
        valid &= (uv[..., 0] < source_width-1) & (uv[..., 1] < source_height-1) & (xx < width) & (yy < height)
        mapped.append(uv); validity.append(valid)
    mapped, validity = np.array(mapped), np.array(validity)
    qcoords = np.zeros(mapped.shape, np.int64)
    patches = []
    counts, max_error, node_bytes = Counter(), 0., 0
    error_map = np.zeros((padded_h, padded_w))
    # All visible cameras get their own patch, even where the route later uses another pair.
    for y in range(0, padded_h, 16):
        for x in range(0, padded_w, 16):
            for c, cam in enumerate(cameras):
                mask = validity[c, y:y+16, x:x+16]
                if not mask.any():
                    patches.append(None); continue
                exact = mapped[c, y:y+16, x:x+16]
                for step in (16, 8, 4, 2, 1):
                    ny, nx = np.mgrid[0:17:step, 0:17:step]
                    uv, _ = map_fn(cam, nx+x, ny+y, width, height)
                    nodes = quantize_coords(uv)
                    result = grid_interpolate(nodes, step)
                    error = np.linalg.norm(result/256-exact, axis=-1)
                    if error[mask].max() <= .25:
                        break
                else:
                    raise AssertionError('Step1 failed coordinate error gate')
                qcoords[c, y:y+16, x:x+16] = result
                safe = (result[..., 0] >= 0) & (result[..., 1] >= 0)
                safe &= ((result[..., 0] >> 8)+1 < source_width) & ((result[..., 1] >> 8)+1 < source_height)
                validity[c, y:y+16, x:x+16] &= safe
                error_map[y:y+16, x:x+16] = np.maximum(error_map[y:y+16, x:x+16], np.where(mask, error, 0))
                max_error = max(max_error, float(error[mask].max()))
                packed = ((nodes[..., 0] & 0x1fffff).astype(np.uint64) |
                          ((nodes[..., 1] & 0x1fffff).astype(np.uint64) << 21)).astype('<u8').tobytes()
                patches.append((step, packed)); counts[step] += 1
                node_bytes += (len(packed)+15)//16*16
    metrics = dict(max_coordinate_error_px=max_error, step_histogram=dict(sorted(counts.items())), node_bytes=node_bytes,
                   descriptor_bytes=len(patches)*16, route_bytes=padded_w*padded_h*2)
    return mapped[:, :height, :width], validity[:, :height, :width], qcoords[:, :height, :width], patches, error_map[:height, :width], metrics


def geometry_ownership_scores(cameras, width, height):
    """Camera-facing horizontal distance, in the calibrated world ground plane."""
    yy, xx = np.indices((height, width))
    world = np.stack(((xx-(width-1)/2)*16/width, ((height-1)/2-yy)*16/width), -1)
    scores=[]
    for camera in cameras:
        center=-camera['R'].T@camera['t']
        forward=camera['R'][2, :2]
        norm=np.linalg.norm(forward)
        if norm < 1e-8:
            raise ValueError('Finite seam ownership needs a nonvertical camera optical axis')
        scores.append((world-center[:2])@(forward/norm))
    return np.array(scores)


def make_routes(valid, feather_width=0, ownership_scores=None):
    if feather_width:
        if ownership_scores is None or ownership_scores.shape != valid.shape:
            raise ValueError('Finite feather needs calibrated per-camera ownership scores')
        va=valid.any(0)
        a=np.where(valid, ownership_scores, -np.inf).argmax(0)
        # Pixels outside all cameras (including the car mask) own no region.
        outside=np.array([cv2.distanceTransform((~((a==c)&va)).astype(np.uint8), cv2.DIST_L2,
                                               cv2.DIST_MASK_PRECISE) for c in range(4)])
        adjacent=valid & ((np.arange(4)[:,None,None]<2)!=(a[None]<2))
        b=np.where(adjacent,outside,np.inf).argmin(0)
        distance=np.take_along_axis(outside,b[None],0)[0]
        vb=np.take_along_axis(adjacent,b[None],0)[0]
        alpha=np.floor(np.clip(.5+np.maximum(0,distance-.5)/feather_width,.5,1)*256+.5).astype(np.uint16)
        alpha=np.where(va,np.where(vb,alpha,256),0).astype(np.uint16)
        vb &= alpha < 256
        route=a.astype(np.uint16)|(b.astype(np.uint16)<<2)|(va.astype(np.uint16)<<4)|(vb.astype(np.uint16)<<5)|(alpha<<6)
        return route,a,b,va,vb,alpha
    distances = np.array([cv2.distanceTransform(np.pad(v.astype(np.uint8), 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1] for v in valid])
    pairs = np.array([(0, 2), (0, 3), (1, 2), (1, 3)])
    pair_scores = np.array([np.where(valid[i] & valid[j], distances[i]+distances[j], -1) for i, j in pairs])
    choice = pair_scores.argmax(0)
    a, b = pairs[choice, 0], pairs[choice, 1]
    no_pair = pair_scores.max(0) < 0
    strongest = distances.argmax(0)
    a = np.where(no_pair, strongest, a)
    b = np.where(no_pair, (strongest+1) % 4, b)
    da = np.take_along_axis(distances, a[None], 0)[0]
    db = np.take_along_axis(distances, b[None], 0)[0]
    swap = ~no_pair & ((db > da) | ((db == da) & (b < a)))
    a, b = np.where(swap, b, a), np.where(swap, a, b)
    va = np.take_along_axis(valid, a[None], 0)[0]
    vb = np.take_along_axis(valid, b[None], 0)[0] & ~no_pair
    da = np.take_along_axis(distances, a[None], 0)[0]
    db = np.take_along_axis(distances, b[None], 0)[0]
    alpha = np.floor(np.divide(da, da+db, out=np.zeros_like(da), where=(da+db) > 0)*256+.5).astype(np.uint16)
    alpha = np.where(va, np.where(vb, alpha, 256), 0).astype(np.uint16)
    route = a.astype(np.uint16) | (b.astype(np.uint16) << 2) | (va.astype(np.uint16) << 4) | (vb.astype(np.uint16) << 5) | (alpha << 6)
    return route, a, b, va, vb, alpha


def compute_output(frames, mapped, qcoords, valid, gains, feather_width=0, ownership_scores=None):
    route, a, b, va, vb, alpha = make_routes(valid,feather_width,ownership_scores)
    rgb = np.array([sample565(f, q)[0] for f, q in zip(frames, qcoords)])
    float_rgb = np.array([sample_float(f, np.nan_to_num(q)) for f, q in zip(frames, mapped)])
    yy, xx = np.indices(a.shape)
    aa, bb = rgb[a, yy, xx], rgb[b, yy, xx]
    out = gain_blend(aa, bb, gains[a], gains[b], alpha, va, vb)
    fa = np.minimum(float_rgb[a, yy, xx]*gains[a]/4096, 255)*va[..., None]
    fb = np.minimum(float_rgb[b, yy, xx]*gains[b]/4096, 255)*vb[..., None]
    float_out = fa*alpha[..., None]/256+fb*(256-alpha[..., None])/256
    means, counts, statistics = {}, {}, []
    for ca in range(4):
        for cb in range(ca+1, 4):
            mask = va & vb & (((a == ca) & (b == cb)) | ((a == cb) & (b == ca)))
            mask &= ((rgb[ca] >= 16) & (rgb[ca] <= 235) & (rgb[cb] >= 16) & (rgb[cb] <= 235)).all(-1)
            if (ca, cb) not in ((0, 2), (0, 3), (1, 2), (1, 3)):
                mask[:] = False
            n = int(mask.sum())
            sums = np.array([rgb[ca][mask].sum(0), rgb[cb][mask].sum(0)], np.int64)
            if n:
                means[ca, cb] = sums/n; counts[ca, cb] = n
            statistics.append(dict(cameras=[ca, cb], count=n, sums=sums.tolist()))
    return route, out, np.clip(np.floor(float_out+.5), 0, 255).astype(np.uint8), statistics, means, counts


def route_continuity(routes, limit):
    """Audit all horizontal/vertical visible edges in camera-weight space."""
    a=routes&3; b=(routes>>2)&3; va=(routes&16)!=0; vb=(routes&32)!=0; alpha=(routes>>6)&511
    yy,xx=np.indices(routes.shape)
    weights=np.zeros((4,)+routes.shape,np.int16)
    weights[a,yy,xx]=np.where(va,alpha,0)
    weights[b,yy,xx]+=np.where(vb,256-alpha,0).astype(np.int16)
    abrupt=0;max_lost=0;max_l1=0;tested=0
    for axis in (0,1):
        s0=(slice(None,-1),slice(None)) if axis==0 else (slice(None),slice(None,-1))
        s1=(slice(1,None),slice(None)) if axis==0 else (slice(None),slice(1,None))
        w0=weights[(slice(None),)+s0];w1=weights[(slice(None),)+s1]
        edge=va[s0]&va[s1]
        lost=np.maximum(np.max(np.where(w1==0,w0,0),axis=0),np.max(np.where(w0==0,w1,0),axis=0))
        tested+=int(edge.sum());abrupt+=int(np.count_nonzero(edge&(lost>limit)))
        if edge.any():
            max_lost=max(max_lost,int(lost[edge].max()))
            max_l1=max(max_l1,int(np.abs(w0-w1).sum(0)[edge].max()))
    return dict(tested_visible_edges=tested,abrupt_camera_support_edges=abrupt,
                max_enter_leave_weight_q8=max_lost,max_weight_l1_step_q8=max_l1,
                camera_support_threshold_q8=limit)


class Memory:
    def __init__(self):
        self.data = bytearray()

    def append(self, data):
        address = len(self.data)
        assert address % 16 == 0
        self.data.extend(data)
        self.data.extend(bytes((-len(self.data)) % 16))
        return address


def write_fixture(directory, frames, patches, route, expected, gains, metadata):
    height, width = expected.shape
    macros = (width+15)//16*((height+15)//16)
    memory = Memory()
    desc_base = memory.append(bytes(macros*64))
    route_blocks = bytearray()
    for y in range(0, height, 16):
        for x in range(0, width, 16):
            block = np.zeros((16, 16), '<u2')
            sub = route[y:y+16, x:x+16]; block[:sub.shape[0], :sub.shape[1]] = sub
            route_blocks.extend(block.tobytes())
    route_base = memory.append(route_blocks)
    for i, patch in enumerate(patches):
        if patch is None:
            continue
        step, packed = patch
        address = memory.append(packed)
        descriptor = address | ((step.bit_length()-1) << 32) | (1 << 35)
        memory.data[desc_base+i*16:desc_base+i*16+8] = struct.pack('<Q', descriptor)
    sh, sw = frames[0].shape
    stride = (sw*2+15)//16*16
    bases = []
    for frame in frames:
        padded = np.zeros((sh, stride//2), '<u2'); padded[:, :sw] = frame
        bases.append(memory.append(padded.tobytes()))
    (directory/'memory.bin').write_bytes(memory.data)
    with (directory/'memory.hex').open('w', encoding='ascii') as out:
        out.writelines(f'{int.from_bytes(memory.data[i:i+16], "little"):032x}\n' for i in range(0, len(memory.data), 16))
    with (directory/'expected.hex').open('w', encoding='ascii') as out:
        out.writelines(f'{int(p):04x}\n' for p in expected.ravel())
    (directory/'expected.bin').write_bytes(expected.astype('<u2').tobytes())
    frame = dict(abi_version=1, width=width, height=height, src_width=sw, src_height=sh,
                 src_stride=stride, camera_bases=bases, desc_base=desc_base, route_base=route_base,
                 memory_words=len(memory.data)//16, memory_bytes=len(memory.data), gains=gains.tolist(),
                 alpha_lut=list(range(257)), **metadata)
    frame['paths'] = {name: str((directory/file).resolve()) for name, file in [('memory', 'memory.hex'), ('expected', 'expected.hex'), ('output', 'rtl_output.hex')]}
    # Also provide simple top-level paths for command runners.
    frame['memory_file'], frame['expected_file'] = frame['paths']['memory'], frame['paths']['expected']
    (directory/'frame.json').write_text(json.dumps(frame, indent=2), encoding='utf-8')
    return frame


def matlab_fixture(directory, cameras, map_fn, width, height, is_public, frames, routes, gains, float_out):
    yy, xx = np.mgrid[0:height:13, 0:width:17]
    uv = np.array([map_fn(c, xx, yy, width, height)[0] for c in cameras])
    payload = dict(xx=xx, yy=yy, width=width, height=height, expected_uv=uv,
                   K=np.array([c['K'] for c in cameras]), D=np.array([c['D'] for c in cameras]), public=int(is_public),
                   frames=np.array(frames), routes=routes, gains=gains, expected_float=float_out)
    for key in (('P', 'H') if is_public else ('R', 't')):
        payload[key] = np.array([c[key] for c in cameras])
    savemat(str(directory/'matlab_input.mat'), payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['small', 'synthetic720', 'synthetic720-wide', 'public'], default='small')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--matlab', action='store_true')
    parser.add_argument('--feather-width', type=int, choices=[0,64], default=None,
                        help='0: full overlap distance weighting; 64: calibrated finite seam (synthetic720 default)')
    args = parser.parse_args()
    feather_width=(64 if args.case=='synthetic720' else 0) if args.feather_width is None else args.feather_width
    if feather_width and args.case!='synthetic720':
        raise ValueError('64-pixel finite feather is calibrated for synthetic720; small/public retain wide routing')
    directory = args.output or ROOT/'sim/data'/args.case
    directory.mkdir(parents=True, exist_ok=True)
    width, height = (64, 48) if args.case == 'small' else (1280, 720)
    if args.case == 'public':
        data_dir = ROOT/'sim/data/public_source'
        provenance = download_public(data_dir)
        cameras, frames = public_cameras(data_dir)
        map_fn = public_map
        geometry = dict(adapter='Inverse project matrix and original flip, followed by original fisheye model',
                        native_bev=[1200, 1600], letterbox=[370, 0, 540, 720])
        gains = np.full((4, 3), 4096, np.int64)
    else:
        sw, sh = (128, 96) if args.case == 'small' else (1280, 720)
        cameras = synthetic_cameras(sw, sh)
        exposures = np.array([[.9, .98, 1.02], [1.08, .92, .95], [1.02, 1.06, .9], [.96, 1.02, 1.07]])
        frames = [render_synthetic(c, sw, sh, e) for c, e in zip(cameras, exposures)]
        gains = np.floor(np.exp(np.log(exposures).mean(0))/exposures*4096+.5).astype(np.int64)
        map_fn = synthetic_map
        provenance = dict(generator='independent pixel ray -> ground plane -> analytic texture', seed='deterministic', exposure_rgb=exposures.tolist())
        geometry = geometry_check(cameras)
        if geometry['max_ray_plane_error_m'] > 1e-8:
            raise AssertionError(geometry)
    sw, sh = frames[0].shape[1], frames[0].shape[0]
    print(f'Generating {args.case}: source {sw}x{sh}, output {width}x{height}', flush=True)
    mapped, valid, qcoords, patches, errors, grid_metrics = build_grid(cameras, map_fn, width, height, sw, sh)
    ownership_scores=geometry_ownership_scores(cameras,width,height) if feather_width else None
    route, expected, float_out, stats, means, counts = compute_output(frames, mapped, qcoords, valid, gains,feather_width,ownership_scores)
    visible = valid.any(0)
    fixed_rgb = expand565(expected).astype(np.uint8)
    quality = image_metrics(fixed_rgb, float_out, visible)
    # Exposure feedback is driven by the very pre-gain sums exported for RTL comparison.
    feedback = np.ones((4, 3))
    for _ in range(60):
        feedback, held = bounded_log_gains(means, counts, feedback)
    def seam_error(g):
        residuals = [np.abs(m[0]*g[a]-m[1]*g[b])/(.5*(m[0]*g[a]+m[1]*g[b])) for (a, b), m in means.items()]
        return float(np.mean(residuals)) if residuals else 0.
    exposure = dict(updates=60, frequency_hz=5, smoothing=.1, gains=feedback.tolist(), held_channels=held,
                    mean_overlap_residual_before=seam_error(np.ones((4, 3))), mean_overlap_residual_after=seam_error(feedback))
    metrics = dict(grid=grid_metrics, geometry=geometry, quality_vs_float=quality,
                   visible_pixels=int(visible.sum()), overlap_pixels=int(((route & 48) == 48).sum()), exposure_feedback=exposure)
    metrics['sample_requests']=metrics['visible_pixels']+metrics['overlap_pixels']
    metrics['seam_strategy']='calibrated_ownership_finite_feather' if feather_width else 'full_overlap_distance'
    metrics['feather_width_pixels']=feather_width
    if feather_width:
        metrics['route_continuity']=route_continuity(route,4)
        if metrics['route_continuity']['abrupt_camera_support_edges']:
            raise AssertionError('Finite seam has discontinuous camera support')
    if args.case != 'public':
        yy, xx = np.indices((height, width)); x=(xx-(width-1)/2)*16/width; y=((height-1)/2-yy)*16/width
        ground = ~((np.abs(x) < .8) & (np.abs(y) < 1.4))
        metrics['ground_coverage'] = float(visible[ground].mean())
        if not visible[ground].all():
            raise AssertionError(f'Uncovered ground pixels: {np.count_nonzero(ground & ~visible)}')
    fixture_case='synthetic720-wide' if args.case=='synthetic720' and not feather_width else args.case
    metadata = dict(case=fixture_case, provenance=provenance, metrics=metrics, statistics=stats,
                    seam_strategy=metrics['seam_strategy'],feather_width_pixels=feather_width)
    frame = write_fixture(directory, frames, patches, route, expected, gains, metadata)
    for i, raw in enumerate(frames):
        cv2.imwrite(str(directory/f'{CAMERAS[i]}_raw.png'), expand565(raw).astype(np.uint8)[..., ::-1])
    cv2.imwrite(str(directory/'golden.png'), fixed_rgb[..., ::-1])
    cv2.imwrite(str(directory/'float_reference.png'), float_out[..., ::-1])
    cv2.imwrite(str(directory/'coordinate_error.png'), cv2.applyColorMap(np.minimum(errors/.25*255, 255).astype(np.uint8), cv2.COLORMAP_TURBO))
    np.savez_compressed(directory/'algorithm_arrays.npz', expected=expected, routes=route, qcoords=qcoords,
                        valid=valid, error=errors, float_uv=mapped)
    (directory/'metrics.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
    calibration = [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in cam.items()} for cam in cameras]
    (directory/'calibration.json').write_text(json.dumps(dict(cameras=calibration, ground_width_m=16 if args.case != 'public' else None,
                        coordinates='world X right, Y front, Z up; camera X right, Y down, Z forward', source_dimensions=[sw, sh]), indent=2), encoding='utf-8')
    matlab_fixture(directory, cameras, map_fn, width, height, args.case == 'public', frames, route, gains, float_out)
    if args.matlab:
        exe = Path('C:/Program Files/MATLAB/R2024b/bin/matlab.exe')
        command = f"addpath('{(ROOT/'tools/avm').as_posix()}'); verify_mapping('{directory.resolve().as_posix()}');"
        subprocess.run([str(exe), '-batch', command], check=True, timeout=180)
    print(json.dumps(dict(case=args.case, memory_bytes=frame['memory_bytes'], metrics=metrics), indent=2), flush=True)


if __name__ == '__main__':
    main()
