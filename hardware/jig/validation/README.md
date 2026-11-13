# Adafruit BNO085 jig fit validation

Date: 2026-11-13. Target: `../imu_jig.stl`. Verdict: **nominal seated fit passes; secure, repeatable fixation remains unvalidated. The loaded jig cannot use its top face as a flat calibration reference.**

No jig geometry or frozen architecture was changed. Added a read-only validation script, numerical results and two inspection images.

## Evidence and method

- Imported the actual binary STL, not just its preview.
- Independently loaded the official Adafruit PID 4754 STEP assembly; tessellated its solids and Boolean-unioned them for collision checks.
- Compared STEP board dimensions and holes against the manufacturer Eagle-derived coordinates. All checks passed within the script's stated tolerances (0.02 mm generally; 0.1 mm for component origins).
- Rebuilt the current Python generator in memory. STL/source symmetric volume difference is about 0.0222 mm³, consistent with small coordinate precision differences; not bit-identical geometry. The STL is manifold according to manifold3d.
- Tested seated position, sampled rigid insertion positions, a 180° board rotation, and the stated wall-datum translation. Detailed outputs and the STL SHA-256 are in `fit_results.json`.
- STL units were interpreted as mm, as established by the generator; STL itself has no unit metadata.

## Findings

| Check | Result | Implication |
|---|---|---|
| PCB size | Official CAD 25.40 × 22.86 × 1.57 mm | Bare Adafruit board, not a SparkFun board or wearable pod |
| Mounting pattern | Four Ø2.50 mm holes; 20.32 × 17.78 mm pitch | Jig pin centres match |
| Pocket | 25.90 × 23.00 mm | Centred CAD board has 0.25 mm X and **0.07 mm Y clearance per side**; print fit is not assured |
| Fully seated nominal assembly | Intersection effectively zero (numerical roundoff) | Board and modelled components do not clash at the intended seat |
| Snap shaft | Ø2.30 mm in Ø2.50 mm CAD hole | Nominal radial clearance 0.10 mm, not a clamped fit |
| Snap barb | Ø2.90 mm | Must compress by roughly 0.20 mm radially per side to pass the nominal hole; rigid insertion simulation has about 0.995 mm³ interference at sampled intermediate positions |
| Vertical retention | Shoulder begins 0.10 mm above PCB top | The design permits vertical movement before engaging the shoulder; no preload is modelled |
| Orientation key | 180° rotated assembly also has effectively zero overlap | The hole pattern does not prevent reversed installation |
| Negative-wall datum | CAD board must translate (-0.25, -0.07) mm to contact both walls | Shift magnitude ~0.260 mm exceeds nominal pin radial clearance; simulation reports ~1.336 mm³ interference. Do not force a seated board into that corner |
| Top reference face | Jig plane Z=40; board bottom Z=39; component maximum Z=43.53 mm | Components project **3.53 mm** above the reference plane. Resting the loaded assembly on this side loads components, not the reference surface |
| Bare jig height | Body 40.00; pin tips reach 41.87 mm | Even the unloaded snap jig is not flat on its nominal top face |
| Headers | Slots provide nominal 8 mm below the PCB (3 mm pillar + 5 mm slot depth) | Actual header plastic, tails and solder fillets are absent from the board STEP and were not collision-validated |
| Cables | Board-mounted STEMMA connectors fit in the model | Mating plugs, cable bends and routing during six-face placement remain unverified |

The generator names 25.6 × 22.7 mm as `MEASURED_X/Y`. These match Adafruit's product listing, but no independent measurement record was found. They must not be treated as validated measurements of the user's particular board. The difference from the official CAD explains the very small Y clearance.

## Relationship to the integration schematic

The local `architecture.html` and the supplied frozen brief describe seven SparkFun BNO086 sensors in 54 × 34 × 13 mm pods. This STL instead has eight bare-Adafruit PCB pockets. It can serve as a separate Adafruit bench fixture, but **does not validate the frozen pod/plank assembly**. The `brace_v4.scad` and `cal_kit_v4.scad` files referenced in the brief were not present in this workspace, so their geometry was not inspected.

## Required checks before approval

1. **MEASURE_PCB_X/Y/T and MEASURE_HOLE_D:** measure the actual board and mounting holes; do not infer fit from catalog rounding.
2. **MEASURE_PIN_D/BARB_D and MEASURE_INSERTION_RETENTION:** print the existing `imu_jig_test_coupon_snap.stl` using the intended PETG process. Check insertion without excessive board bending, complete seating on all four pillars, retention when inverted, and removal/reinsertion. No elastic/strength or fatigue analysis was performed.
3. **MEASURE_SEATED_PLAY:** check axial/rotational play under gentle cable load. “Does not fall out” is insufficient for a repeatable orientation fixture.
4. **MEASURE_PLUG_AND_HEADER_ENVELOPE:** fit the actual cables and headers on that coupon. Confirm access, strain relief and clearance in every intended pose.
5. Before using six reference faces, resolve the top-face protrusion. Before relying on repeatable axes, resolve or control 180° reversal and use a datum strategy compatible with the pins. These are identified design issues, not implemented changes.

## Sources

- [Adafruit official downloads](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/downloads)
- [Official 4754 STEP/F3D/STL repository](https://github.com/adafruit/Adafruit_CAD_Parts/tree/main/4754%20BNO085%20STEMMA%20QT)
- [Official Eagle PCB repository](https://github.com/adafruit/Adafruit-BNO08x-PCB)
- [Product listing](https://www.adafruit.com/product/4754)
- Local originals: `../../dimension/adafruit_bno085/`; download provenance: `../../dimension/sources.json`.
- Local design: `../imu_jig.py`, `../imu_jig.stl`; integration context: `../../../architecture.html`.

Reproduce from the project root with `python hardware/jig/validate_fit.py`. This writes validation outputs only, not the production STL.
