#!/usr/bin/env python3
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bundle', ROOT/'tools/prepare_adas_bundle.py')
bundle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bundle)

class BundleTest(unittest.TestCase):
    def test_mandatory_classes_hashes_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            for f in ['scene.rknn','lane.rknn']:
                (root/f).write_bytes(b'fixture, not an RKNN model')
            sm = {'output':'yolov5_raw_nchw_v1','classes':sorted(bundle.REQUIRED_CLASSES),
                  'width':640,'height':640,'anchors':[[10,13,16,30,33,23]]*3}
            lm = {'output':'lane_rows_4x96_v1','width':512,'height':256,'road_roi':[0,.42,1,.58]}
            (root/'scene.json').write_text(json.dumps(sm))
            (root/'lane.json').write_text(json.dumps(lm))
            def prepare(out):
                return bundle.prepare(root/'scene.rknn',root/'scene.json',root/'lane.rknn',root/'lane.json',root/out,'test','test-toolkit','test-runtime')
            manifest = prepare('good')
            self.assertEqual(len(manifest['scene']['sha256']),64)
            self.assertEqual(manifest['lane']['input']['std'],[255,255,255])
            self.assertEqual(json.loads((root/'good/manifest.json').read_text())['target'],'rk3568')
            with self.assertRaises(ValueError):prepare('good')
            sm['classes'].remove('debris');(root/'scene.json').write_text(json.dumps(sm))
            with self.assertRaises(ValueError):prepare('missing-capability')
            self.assertFalse((root/'missing-capability').exists())

    def test_eight_class_scene_without_lane(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);(root/'scene.rknn').write_bytes(b'fixture, not an RKNN model')
            names=['person','vehicle','crosswalk','light_red','light_green','cone','barrier','debris']
            sm={'classes':names,'output':'yolov5_rk_sigmoid_nchw_v1','width':640,'height':640,'anchors':[[10,13,16,30,33,23]]*3}
            (root/'scene.json').write_text(json.dumps(sm))
            m=bundle.prepare(root/'scene.rknn',root/'scene.json',None,None,root/'out','adas8','2.3.2','2.3.2 (')
            self.assertEqual(m['schema'],2);self.assertIsNone(m['lane'])
            self.assertEqual(m['scene']['classes'],names)
            self.assertEqual(m['scene']['output'],'yolov5_rk_sigmoid_nchw_v1')
            self.assertFalse((root/'out/lane.rknn').exists())
            names.remove('debris');(root/'scene.json').write_text(json.dumps(sm))
            with self.assertRaises(ValueError):
                bundle.prepare(root/'scene.rknn',root/'scene.json',None,None,root/'unsafe','v','t','r')

    def test_reject_incompatible_shapes_and_roi(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name)
            sm={'classes':list(bundle.REQUIRED_CLASSES),'output':'wrong'}
            lm={}
            for path,value in [('scene.json',sm),('lane.json',lm)]:
                (root/path).write_text(json.dumps(value))
            with self.assertRaises(ValueError):
                bundle.prepare(root/'s',root/'scene.json',root/'l',root/'lane.json',root/'out','v','t','r')

if __name__=='__main__':unittest.main()
