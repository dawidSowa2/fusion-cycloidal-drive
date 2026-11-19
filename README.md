# Cycloidal Drive add-in for Fusion

Generates a complete 1, 2 or 3 disc cycloidal reducer for a stepper motor. Made using Claude.

## Install
1. Copy this whole `CycloidalDrive` folder to
   - macOS: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/`
   - Windows: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\`
2. In Fusion: **Utilities > Add-Ins** (Shift+S) > **Add-Ins** tab > select *CycloidalDrive* > **Run**
   (tick *Run on Startup* to keep it).
3. The command shows up under **SOLID > CREATE > Cycloidal Drive**.

## What it builds
Each run creates one `Cycloidal Drive XXto1 N-disc` component containing:

| Component | Notes |
|---|---|
| Disc 1..n | Each disc is phased to its own eccentric (2 discs: 0/180°, 3 discs: 0/120/240°) and has the bore for its bearing and the output holes. |
| Eccentric Cam | One lobe per disc, with a D-flat bore for the motor shaft. |
| Ring Pins | Separate bodies, for dowel pins or printed pins. Each end sits in a blind seat. |
| Housing | Ring with pin seats plus an optional NEMA 14/17/23 base plate (pilot + 4 screw holes + blind pin seats). The bore clears the lobe tips by 0.5 mm all the way round. |
| Top Cover | Ring with blind pin seats that holds the top ends of the ring pins. |
| Output Carrier | Plate with the output pins built in. It stays inside the ring pins. |

The model is placed at input angle 0, so all parts mesh with each other in the assembly.

## Parameters worth knowing
- **Ratio = ring pins − 1** (the output turns the opposite way to the motor).
- **Eccentricity**: must be less than ring radius ÷ pins. Larger values give taller lobes and
  more torque, but sharper lobe tips.
- **Profile clearance** shrinks the disc profile. Start around 0.1 mm for FDM printing and 0–0.02 mm for machined parts.
- **Output pin diameter** is the diameter that touches the disc. If you put rollers or bushings on the pins, use the roller diameter.
- Disc bore = bearing OD, cam OD = bearing ID. The defaults suit a 6704 bearing
  (20×27×4 mm). With the default 6 mm discs, the bearing sits recessed in the disc.

The summary box shows the ratio, disc sizes and any problems: lobe undercut, pins overlapping, output holes cutting into the bore or the lobe roots, or a cam that's too thin. **Generate** is disabled while there is an error.
The settings you used last are saved in `last_settings.json`.

## Two discs vs three
- **2 discs (180°)**: the forces in the plane of the discs cancel, but because the discs sit at different heights they leave a rocking couple.
  This is simple and common.
- **3 discs (120°)**: the forces in the plane cancel and the torque ripple is lower. The load is shared
  across more lobes, so each disc can be thinner. A smaller rocking couple still remains because
  the discs are stacked.

## Not included (add these yourself)
- A bearing between the output carrier and the housing.
- Screws to hold the top cover and base to the housing, and fasteners on the output.

## Files
- `CycloidalDrive.py`: the Fusion command (UI and modelling).
- `cycloid_geometry.py`: pure-Python profile math and validation, which you can test without Fusion.

## Tests
The geometry tests run without Fusion:

```bash
python3 -m unittest discover tests
```
They check that every disc touches every ring pin at several shaft angles, that the discs clear the housing bore, and that the input validation works.
