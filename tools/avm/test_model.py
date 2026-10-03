"""Algorithm and ABI regressions; run python -m unittest discover -s tools/avm -v."""
import json
from pathlib import Path
import unittest
import numpy as np
from model import (round_shift, quantize_coords, pack565, expand565, grid_interpolate,
                   sample565, gain_blend, bounded_log_gains, synthetic_cameras, geometry_check)
from generate import make_routes, route_continuity

ROOT = Path(__file__).resolve().parents[2]


def scalar_round(n, shift):
    if not shift:
        return n
    mag, sign = abs(int(n)), -1 if n < 0 else 1
    return sign*((mag+(1 << (shift-1)))//(1 << shift))


def scalar_expand(p):
    r, g, b = p//2048, (p//32) % 64, p % 32
    return [r*8+r//4, g*4+g//16, b*8+b//4]


class ArithmeticTests(unittest.TestCase):
    def test_signed_ties_and_saturation(self):
        np.testing.assert_array_equal(round_shift([-5, -3, -1, 0, 1, 3, 5], 1), [-3,-2,-1,0,1,2,3])
        np.testing.assert_array_equal(quantize_coords([-.5/256, .5/256, -9000, 9000]), [-1,1,-1048576,1048575])

    def test_rgb565_exhaustive_roundtrip(self):
        p = np.arange(65536, dtype=np.uint16)
        np.testing.assert_array_equal(pack565(expand565(p)), p)

    def test_grid_against_scalar_signed_oracle(self):
        rng = np.random.default_rng(481)
        for step in (1,2,4,8,16):
            nodes = rng.integers(-1048576, 1048576, (16//step+1, 16//step+1, 2), dtype=np.int64)
            actual = grid_interpolate(nodes, step)
            for y in range(16):
                for x in range(16):
                    for ch in range(2):
                        ix, iy, fx, fy = x//step, y//step, x%step, y%step
                        h0=scalar_round(nodes[iy,ix,ch]*(step-fx)+nodes[iy,ix+1,ch]*fx, step.bit_length()-1)
                        h1=scalar_round(nodes[iy+1,ix,ch]*(step-fx)+nodes[iy+1,ix+1,ch]*fx, step.bit_length()-1)
                        golden=scalar_round(h0*(step-fy)+h1*fy, step.bit_length()-1)
                        self.assertEqual(actual[y,x,ch], golden)

    def test_pixel_bilinear_corners_and_scalar_random(self):
        rng=np.random.default_rng(77)
        frame=rng.integers(0,65536,(35,67),dtype=np.uint16)
        coords=rng.integers([0,0],[66*256,34*256],(300,2))
        coords=np.vstack((coords,[[0,0],[65*256+255,33*256+255],[31*256+127,31*256+129]]))
        actual, valid=sample565(frame,coords)
        self.assertTrue(valid.all())
        for i,(u,v) in enumerate(coords):
            x,y,fx,fy=int(u)//256,int(v)//256,int(u)%256,int(v)%256
            neighbors=[scalar_expand(int(frame[y+dy,x+dx])) for dy,dx in [(0,0),(0,1),(1,0),(1,1)]]
            expect=[(neighbors[0][ch]*(256-fx)*(256-fy)+neighbors[1][ch]*fx*(256-fy)+neighbors[2][ch]*(256-fx)*fy+neighbors[3][ch]*fx*fy+32768)//65536 for ch in range(3)]
            np.testing.assert_array_equal(actual[i],expect)
        invalid=np.array([[-1,0],[66*256,0],[0,34*256]])
        self.assertFalse(sample565(frame,invalid)[1].any())

    def test_gain_blend_endpoints_validity_and_saturation(self):
        a=np.array([255,100,20]); b=np.array([10,30,250])
        ga=np.array([8192,4096,2048]); gb=np.array([4096,8192,4096])
        self.assertEqual(gain_blend(a,b,ga,gb,0),pack565([10,60,250]))
        self.assertEqual(gain_blend(a,b,ga,gb,256),pack565([255,100,10]))
        self.assertEqual(gain_blend(a,b,ga,gb,128),pack565([133,80,130]))
        self.assertEqual(gain_blend(a,b,ga,gb,128,False,False),0)
        self.assertEqual(gain_blend(a,b,ga,gb,17,True,False),pack565([255,100,10]))

    def test_adjacent_route_and_single_camera_fallback(self):
        mask=np.zeros((4,8,8),bool);mask[0]=True;mask[1]=True
        route,a,b,va,vb,alpha=make_routes(mask)
        self.assertTrue(va.all());self.assertFalse(vb.any());self.assertTrue((alpha==256).all())
        mask[2,:,4:]=True
        _,a,b,va,vb,_=make_routes(mask)
        self.assertTrue(vb[:,4:].all())
        self.assertTrue(np.all((a[vb]<2)&(b[vb]>=2)))

    def test_independent_opencv_forward_and_ray_plane_inverse(self):
        result=geometry_check(synthetic_cameras(1280,720))
        self.assertGreaterEqual(result['control_points'],40)
        self.assertLess(result['max_ray_plane_error_m'],1e-8)


class GainTests(unittest.TestCase):
    def setUp(self):
        self.exposure=np.array([[.8,1.1,.95],[1.1,.9,1.05],[.95,1.05,.9],[1.05,.95,1.1]])
        self.means={(a,b):self.exposure[[a,b]]*100 for a,b in [(0,2),(0,3),(1,2),(1,3)]}
        self.counts={pair:10000 for pair in self.means}

    def test_closed_loop_sixty_updates(self):
        gains=np.ones((4,3))
        for _ in range(60):
            gains,held=bounded_log_gains(self.means,self.counts,gains)
        self.assertEqual(held,[])
        corrected=self.exposure*gains
        residual=np.max(np.ptp(corrected,axis=0)/np.mean(corrected,axis=0))
        self.assertLessEqual(residual,.02)
        np.testing.assert_allclose(np.log(gains).sum(0),0,atol=1e-12)
        self.assertTrue(((gains>=.5)&(gains<=2)).all())

    def test_disconnected_dark_saturated_hold(self):
        previous=np.exp(np.array([[-.1,.2,-.05],[.1,-.2,.05],[0,0,0],[0,0,0]]))
        for mode in ('disconnect','dark','saturated'):
            means={p:m.copy() for p,m in self.means.items()}
            counts=self.counts.copy()
            if mode=='disconnect':
                counts[(0,2)]=counts[(1,2)]=0
            elif mode=='dark':
                for p,m in means.items():
                    if 2 in p:m[p.index(2),:]=0
            else:
                for p,m in means.items():
                    if 2 in p:m[p.index(2),:]=255
            gains,held=bounded_log_gains(means,counts,previous)
            np.testing.assert_array_equal(gains,previous);self.assertEqual(held,[0,1,2])

    def test_bound_active_solution_keeps_equality(self):
        exposures=np.array([10.,200.,40.,180.])
        means={p:np.repeat(exposures[list(p),None],3,axis=1) for p in self.means}
        gains,held=bounded_log_gains(means,self.counts,smoothing=1)
        self.assertEqual(held,[])
        self.assertTrue(((gains>=.5-1e-10)&(gains<=2+1e-10)).all())
        np.testing.assert_allclose(np.log(gains).sum(0),0,atol=1e-12)
        self.assertTrue(np.any(np.isclose(gains,.5)) or np.any(np.isclose(gains,2)))


class FixtureTests(unittest.TestCase):
    def test_finite_seam_source_weights_and_continuity(self):
        directory=ROOT/'sim/data/synthetic720'
        if not (directory/'frame.json').exists():
            self.skipTest('Generate synthetic720 fixture first')
        info=json.loads((directory/'frame.json').read_text())
        if info.get('feather_width_pixels') != 64:
            self.skipTest('Fixture uses the wide stress configuration')
        arrays=np.load(directory/'algorithm_arrays.npz');route=arrays['routes'];valid=arrays['valid']
        a=route&3;b=(route>>2)&3;va=(route&16)!=0;vb=(route&32)!=0;alpha=(route>>6)&511
        yy,xx=np.indices(route.shape)
        np.testing.assert_array_equal(va,valid.any(0))
        self.assertTrue(valid[a[va],yy[va],xx[va]].all())
        self.assertTrue(valid[b[vb],yy[vb],xx[vb]].all())
        self.assertTrue(np.all((a[vb]<2)!=(b[vb]<2)))
        self.assertTrue(np.all((alpha[vb]>=128)&(alpha[vb]<256)))
        self.assertTrue(np.all(alpha[va&~vb]==256))
        result=route_continuity(route,4)
        self.assertEqual(result['abrupt_camera_support_edges'],0)
        self.assertLessEqual(result['max_weight_l1_step_q8'],8)
        self.assertGreater(int(vb.sum()),0)
        self.assertLess(float(vb.sum()/va.sum()),.12)

    def test_finite_seam_preserves_calibration_raw_and_mapping(self):
        finite=ROOT/'sim/data/synthetic720';wide=ROOT/'sim/data/synthetic720-wide'
        if not (finite/'frame.json').exists() or not (wide/'frame.json').exists():
            self.skipTest('Generate finite and wide fixtures first')
        self.assertEqual((finite/'calibration.json').read_bytes(),(wide/'calibration.json').read_bytes())
        f=json.loads((finite/'frame.json').read_text());w=json.loads((wide/'frame.json').read_text())
        for key in ('camera_bases','desc_base','route_base','src_width','src_height','src_stride'):
            self.assertEqual(f[key],w[key])
        fm=(finite/'memory.bin').read_bytes();wm=(wide/'memory.bin').read_bytes()
        self.assertEqual(fm[:f['route_base']],wm[:w['route_base']])
        end_routes=f['route_base']+f['metrics']['grid']['route_bytes']
        self.assertEqual(fm[end_routes:],wm[end_routes:])
        fa=np.load(finite/'algorithm_arrays.npz');wa=np.load(wide/'algorithm_arrays.npz')
        for key in ('float_uv','qcoords','valid','error'):
            np.testing.assert_array_equal(fa[key],wa[key])
        self.assertLessEqual(f['metrics']['grid']['max_coordinate_error_px'],.25)
        self.assertEqual(f['metrics']['ground_coverage'],1)

    def test_small_ddr_binary_abi_and_scalar_complete_frame(self):
        directory=ROOT/'sim/data/small'
        if not (directory/'frame.json').exists():
            self.skipTest('Generate small fixture first')
        info=json.loads((directory/'frame.json').read_text())
        memory=(directory/'memory.bin').read_bytes()
        hex_bytes=b''.join(int(line,16).to_bytes(16,'little') for line in (directory/'memory.hex').read_text().splitlines())
        self.assertEqual(memory,hex_bytes)
        expected=np.array([int(x,16) for x in (directory/'expected.hex').read_text().splitlines()]).reshape(info['height'],info['width'])
        def uint(addr,n):return int.from_bytes(memory[addr:addr+n],'little')
        def coord(node):
            u=node&0x1fffff;v=(node>>21)&0x1fffff
            return [u-(0x200000 if u&0x100000 else 0),v-(0x200000 if v&0x100000 else 0)]
        for y in range(info['height']):
            for x in range(info['width']):
                macro=(y//16)*((info['width']+15)//16)+x//16
                route=uint(info['route_base']+macro*512+2*((y%16)*16+x%16),2)
                camera=[route&3,(route>>2)&3]; valid=[bool(route&16),bool(route&32)]
                rgb=[]
                for c,v in zip(camera,valid):
                    if not v:rgb.append([0,0,0]);continue
                    descriptor=uint(info['desc_base']+macro*64+c*16,16)
                    self.assertTrue(descriptor&(1<<35))
                    addr=descriptor&0xffffffff;shift=(descriptor>>32)&7;step=1<<shift
                    ix,iy=(x%16)//step,(y%16)//step;fx,fy=x%step,y%step;n=16//step+1
                    ns=[coord(uint(addr+8*((iy+dy)*n+ix+dx),8)) for dy,dx in [(0,0),(0,1),(1,0),(1,1)]]
                    uv=[]
                    for ch in range(2):
                        h0=scalar_round(ns[0][ch]*(step-fx)+ns[1][ch]*fx,shift)
                        h1=scalar_round(ns[2][ch]*(step-fx)+ns[3][ch]*fx,shift)
                        uv.append(scalar_round(h0*(step-fy)+h1*fy,shift))
                    sx,sy=uv[0]//256,uv[1]//256;fx,fy=uv[0]%256,uv[1]%256
                    self.assertTrue(0<=sx<info['src_width']-1 and 0<=sy<info['src_height']-1)
                    ns=[scalar_expand(uint(info['camera_bases'][c]+(sy+dy)*info['src_stride']+(sx+dx)*2,2)) for dy,dx in [(0,0),(0,1),(1,0),(1,1)]]
                    sample=[(ns[0][ch]*(256-fx)*(256-fy)+ns[1][ch]*fx*(256-fy)+ns[2][ch]*(256-fx)*fy+ns[3][ch]*fx*fy+32768)//65536 for ch in range(3)]
                    rgb.append([min((sample[ch]*info['gains'][c][ch]+2048)//4096,255) for ch in range(3)])
                alpha=(route>>6)&511
                if not valid[1]:alpha=256
                if not valid[0]:alpha=0
                color=[(rgb[0][ch]*alpha+rgb[1][ch]*(256-alpha)+128)//256 for ch in range(3)]
                packed=(color[0]//8)*2048+(color[1]//4)*32+color[2]//8
                self.assertEqual(packed,expected[y,x],f'pixel{x},{y}')


if __name__=='__main__':
    unittest.main()
