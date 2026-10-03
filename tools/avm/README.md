# AVM algorithm and fixture tools

Run from the ADAS repository root, with Python 3.10+ and `requirements.txt` installed:

```powershell
python tools/avm/generate.py --case small --matlab
python tools/avm/generate.py --case synthetic720 --matlab
python tools/avm/generate.py --case synthetic720-wide --matlab
python tools/avm/generate.py --case public --matlab
python -m unittest discover -s tools/avm -v
python tools/avm/exposure.py --statistics sim/data/small/frame.json --output sim/data/small/feedback.json
python tools/avm/apply_gains.py --fixture sim/data/small/frame.json --gains-json sim/data/small/feedback.json
```

`--matlab` invokes `C:/Program Files/MATLAB/R2024b/bin/matlab.exe` in batch mode. To use another installation, invoke `verify_mapping(fixture_directory)` directly from MATLAB after adding `tools/avm` to its path. MATLAB evaluates the fisheye mapping independently, renders the entire double-precision reference, and writes its verification JSON and PNG. It does not use Computer Vision Toolbox's differently parameterized fisheye model.

The public case downloads 12 files from the pinned commit, verifies every SHA256 against `public_source_manifest.json`, and preserves upstream MIT licensing. The exact inputs remain in `sim/data/public_source/`; images and coefficients are never fetched from a moving branch. Subsequent runs require no network access while the verified cache exists.

Each fixture lives in `sim/data/<case>/` unless `--output` supplies another directory:

| Artifact | Meaning |
|---|---|
| `memory.hex`, `memory.bin` | Identical DDR bytes in 128-bit ModelSim hex / binary form |
| `expected.hex`, `expected.bin` | Bit-exact raster RGB565 output |
| `frame.json` | Dimensions, byte addresses, gains, statistics and provenance |
| `calibration.json` | Camera models, intrinsics and original extrinsics or project adapter |
| `algorithm_arrays.npz` | Quantized interpolated coordinates, floating maps, visibility, routes, error map |
| `golden.png`, `float_reference.png` | Python fixed-point / double reference rendered to RGB |
| `matlab_reference.png`, `matlab_verification.json` | Independent MATLAB full-frame reference and actual comparison |
| `coordinate_error.png` | Source-coordinate Euclidean error, color range 0–0.25 pixels |
| `metrics.json` | Mesh memory/error, geometry, masked PSNR/SSIM, gain feedback results |

The normative shared byte-level interfaces and arithmetic are in `doc/avm/implementation-contract.md`. All cameras have one common source resolution/stride per fixture. Output is RGB565, and the mesh carries signed 21-bit Q8 coordinates. RGB channels are expanded, sampled and gain-corrected separately before final truncation back to RGB565.

`synthetic720` defaults to a 64-pixel finite seam (`--feather-width 64`): calibrated world-space camera ownership defines four adjacent-camera transitions, then distance to the neighboring ownership region produces a continuous feather band. Outside it, only one camera is sampled. All original source frames, calibration, valid domains and mapping nodes are retained. `synthetic720-wide` preserves full-overlap distance weighting as a supported stress case. The equivalent explicit command is `--case synthetic720 --feather-width 0 --output sim/data/synthetic720-wide`. `small` and the public homography adapter retain their wide routes; finite feather requires the synthetic physical camera geometry. Performance claims must name the seam strategy: a 30fps result for the finite configuration does not imply 30fps for the wide stress case.

`apply_gains.py` writes `frame_feedback.json`, `expected_feedback.hex` and `golden_feedback.png` while preserving the original memory and first-frame fixture. It accepts only explicit `gains_q4_12` register values (or a 4×3 integer array); this avoids accidentally treating floating gains as registers. Use this second configuration to verify a frame-boundary gain update in RTL. It reuses cached coordinates/route arrays and the exact RGB565 source bytes in `memory.bin`.

`exposure.py` also accepts actual RTL statistics as a JSON list of `{cameras:[a,b],count:N,sums:[[R,G,B],[R,G,B]]}` records. The canonical pair order is front-left, front-right, back-left, back-right. Sums are pre-gain interpolated RGB, with a common sample accepted only when all six components are in the inclusive 16–235 range. Q4.12 values are written to hardware, while the host retains unquantized log-EMA state between updates. This replay evaluates a static frame's repeated measurement, not a live driver or camera control loop.

The bounded log least-squares gain solver enumerates active bounds for the four-camera problem, enforces zero summed log gain, and holds a channel's previous gains when usable edges do not connect all four cameras. `--previous` resumes the feedback state. The synthetic ≤2% exposure acceptance applies to connected, non-saturated, reachable gain cases; real public calibration samples may contain scene/view inconsistencies that one gain per camera cannot remove.
