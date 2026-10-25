import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace

SCRIPT = Path(__file__).resolve().parents[1] / 'tools/adas_models/export_rk_scene.py'
CLASSES = ['person', 'vehicle', 'crosswalk', 'light_red', 'light_green', 'cone', 'barrier', 'debris']
ANCHORS = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]


def fixture(op='Sigmoid', channels=39, names=CLASSES):
    def tensor(name, dimensions):
        return SimpleNamespace(name=name, type=SimpleNamespace(tensor_type=SimpleNamespace(
            shape=SimpleNamespace(dim=[SimpleNamespace(dim_value=value) for value in dimensions]))))

    nodes, outputs = [], []
    for i, side in enumerate((80, 40, 20)):
        name = 'p' + str(i)
        nodes.append(SimpleNamespace(op_type=op, output=[name]))
        outputs.append(tensor(name, [1, channels, side, side]))
    return SimpleNamespace(
        graph=SimpleNamespace(node=nodes, input=[tensor('images', [1, 3, 640, 640])], output=outputs),
        metadata_props=[SimpleNamespace(key='names', value=str(dict(enumerate(names))))])


class ExportContractTest(unittest.TestCase):
    def module(self):
        self.assertTrue(SCRIPT.is_file(), 'official RK export wrapper is missing')
        spec = importlib.util.spec_from_file_location('export_rk_scene_under_test', SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_records_actual_classes_anchors_and_sigmoid_contract(self):
        result = self.module().validate_contract(fixture(), ANCHORS)
        self.assertEqual(result['output'], 'yolov5_rk_sigmoid_nchw_v1')
        self.assertEqual(result['classes'], CLASSES)
        self.assertEqual(result['anchors'], ANCHORS)
        self.assertEqual(result['normalization'], {'mean': [0, 0, 0], 'std': [255, 255, 255]})

    def test_rejects_raw_logits_under_sigmoid_contract(self):
        with self.assertRaisesRegex(ValueError, 'Sigmoid'):
            self.module().validate_contract(fixture(op='Conv'), ANCHORS)

    def test_rejects_ninth_head_channel_without_ninth_class(self):
        with self.assertRaisesRegex(ValueError, 'shape'):
            self.module().validate_contract(fixture(channels=42), ANCHORS)

    def test_rejects_missing_mandatory_trained_class(self):
        with self.assertRaisesRegex(ValueError, 'classes'):
            self.module().validate_contract(fixture(names=CLASSES[:-1] + ['untrained']), ANCHORS)

    def test_rejects_invalid_anchor(self):
        bad = [list(row) for row in ANCHORS]
        bad[0][0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'anchors'):
            self.module().validate_contract(fixture(), bad)


if __name__ == '__main__':
    unittest.main(verbosity=2)
