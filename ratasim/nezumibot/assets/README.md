# NezumiBot assembled

From `nezumi_3d` (`build_nezumi.py`, its `sim/` output): NezumiBot's printed parts (its repository's
`Nezumi_base`, `Nezumi_tapa`, `gorra`, `Rueda`, and its `Ratatron/` rear cover and motor supports) with its
electronics placed as in its photos (Blackpill F401CC, TB6612FNG, two N20 with encoders, four Sharp
GP2Y0A51SK0F, 2S LiPo, start module, IMU), its Ø30 tyres and the two foam pads under it.

- `robot_*.stl`, `wheel_l_*.stl`, `wheel_r_*.stl`: one mesh per colour and body; `parts.json` gives each one's
  body and rgba. Frame: mm, x right, y forward, z up, **origin between the wheels at the axle's height**
  (the floor at z −15); the wheels' meshes in this frame too.
- `nezumi_params.json`: its measures (tyres, floor, foam pads, sensors, estimated mass and centre of mass).

Measured: the printed parts, the tyres' Ø30. Assumed: the electronics' placement (photos, datasheets), its mass
and centre of mass (by parts), which Sharp goes to which ADC pin.
