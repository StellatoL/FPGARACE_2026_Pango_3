"""Create a second frame configuration/reference without rewriting camera/LUT DDR.

python tools/avm/apply_gains.py --fixture sim/data/small/frame.json --gains-json feedback.json
"""
import argparse
import json
from pathlib import Path
import numpy as np
import cv2
from model import sample565, gain_blend, expand565


def apply(fixture, gains_json, tag='feedback'):
    fixture=Path(fixture);directory=fixture.parent
    frame=json.loads(fixture.read_text(encoding='utf-8-sig'))
    payload=json.loads(Path(gains_json).read_text(encoding='utf-8-sig'))
    if isinstance(payload,dict):
        gains=payload.get('gains_q4_12')
        if gains is None:raise ValueError('Gain JSON needs gains_q4_12; floating gains are not register values')
    else:gains=payload
    gains=np.array(gains,np.int64)
    if gains.shape!=(4,3) or not ((gains>=2048)&(gains<=8192)).all():
        raise ValueError('Expected 4x3 Q4.12 gains within 0.5..2.0')
    memory=(directory/'memory.bin').read_bytes()
    frames=[]
    for base in frame['camera_bases']:
        count=frame['src_height']*frame['src_stride']//2
        raw=np.frombuffer(memory,dtype='<u2',count=count,offset=base).reshape(frame['src_height'],frame['src_stride']//2)
        frames.append(raw[:,:frame['src_width']])
    arrays=np.load(directory/'algorithm_arrays.npz')
    route=arrays['routes'];coords=arrays['qcoords']
    rgb=np.array([sample565(raw,q)[0] for raw,q in zip(frames,coords)])
    a=route&3;b=(route>>2)&3;va=(route&16)!=0;vb=(route&32)!=0;alpha=(route>>6)&511
    yy,xx=np.indices(route.shape)
    expected=gain_blend(rgb[a,yy,xx],rgb[b,yy,xx],gains[a],gains[b],alpha,va,vb)
    expected_file=directory/f'expected_{tag}.hex'
    with expected_file.open('w',encoding='ascii') as out:
        out.writelines(f'{int(p):04x}\n' for p in expected.ravel())
    cv2.imwrite(str(directory/f'golden_{tag}.png'),expand565(expected).astype(np.uint8)[...,::-1])
    frame['gains']=gains.tolist()
    frame['configuration_version']=2
    frame['gain_source']=str(Path(gains_json).resolve())
    frame['expected_file']=str(expected_file.resolve())
    frame['paths']['expected']=str(expected_file.resolve())
    frame['paths']['output']=str((directory/f'rtl_output_{tag}.hex').resolve())
    new_frame=directory/f'frame_{tag}.json'
    new_frame.write_text(json.dumps(frame,indent=2),encoding='utf-8')
    return dict(frame=str(new_frame.resolve()),expected=str(expected_file.resolve()),gains_q4_12=gains.tolist(),
                memory_preserved=True,configuration_version=2)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture',type=Path,required=True)
    parser.add_argument('--gains-json',type=Path,required=True)
    parser.add_argument('--tag',default='feedback')
    args=parser.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Tag must be alphanumeric or underscore')
    print(json.dumps(apply(args.fixture,args.gains_json,args.tag),indent=2))


if __name__=='__main__':main()
