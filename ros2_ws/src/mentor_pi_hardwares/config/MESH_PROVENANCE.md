# Mentor Pi Mesh Provenance

The binary STL files below were copied byte-for-byte from
`git@github.com:ZamesNg/ng_planner.git`, branch `mpc`, commit
`94e104c46d6f5791134cbef1f18b3a9711ef4e66`. They were extracted with
`git archive`; the source repository's dirty working tree was not read or
modified.

No mesh was scaled, transformed, simplified, or otherwise rewritten. The
recorded source and installed files therefore have identical SHA-256 values,
triangle counts, and metre-scale bounds. All retained body, wheel, camera, and
lidar meshes are referenced as visualization geometry. The unreferenced
Ackermann `imu_link.stl` from the source branch is intentionally excluded.

The source package declared its license as `TODO: License declaration`.
Consequently these mesh assets are marked `NOASSERTION` and are not covered by
the Apache-2.0 declaration for the Mentor Pi source code. No additional
redistribution grant is asserted here.

| Asset | Bytes | Triangles | SHA-256 | Minimum XYZ (m) | Maximum XYZ (m) |
| --- | ---: | ---: | --- | --- | --- |
| `ackermann/meshes/base_link.stl` | 830384 | 16606 | `5673bcdcb55d1631f71d567687db05ba7daf78c96b3389ec973a74a75296347c` | `(-0.103781, -0.050426, -0.043)` | `(0.109563, 0.050426, 0.057511)` |
| `ackermann/meshes/cam_link.stl` | 4752784 | 95054 | `10698ec8ab4ca4126243feb947c8e1133b515556f0b2f3f17204deb2128c8f37` | `(-0.026777, -0.044217, -0.025831)` | `(0.013735, 0.045583, 0.000353)` |
| `ackermann/meshes/lidar_link.stl` | 911284 | 18224 | `c902e4018546df554096c5d063b5d831646384d04a7a6d4962e8a937c2c90f1b` | `(-0.026803, -0.018745, -0.035)` | `(0.026934, 0.027787, -0.0)` |
| `ackermann/meshes/wheel_left_front_link.stl` | 3171784 | 63434 | `33f6b38472ac44a39b9fb14a050c1dd460d14b94abfd05c82033062bf9844911` | `(-0.032346, -0.016076, -0.032348)` | `(0.032348, 0.009317, 0.032346)` |
| `ackermann/meshes/wheel_left_rear_link.stl` | 3242784 | 64854 | `dd9f2a8e4f85a4f86eddd296b3d5a58eb3d43b82ff0ef65d597ed389a60a444d` | `(-0.032346, -0.019214, -0.032348)` | `(0.032348, 0.009317, 0.032346)` |
| `ackermann/meshes/wheel_right_front_link.stl` | 3190784 | 63814 | `1f08a58badc36baeb4286e9eae04d9856518eb065c6a092bb48130eaf7a57dc5` | `(-0.032349, -0.009317, -0.032345)` | `(0.032345, 0.016076, 0.032349)` |
| `ackermann/meshes/wheel_right_rear_link.stl` | 3265684 | 65312 | `301cd8642c44dedf3e14e50652b96ebab04d7dc1d38bab478e715fc2d67e7c6c` | `(-0.032349, -0.009317, -0.032345)` | `(0.032345, 0.019214, 0.032349)` |
| `mecanum/meshes/base_link.stl` | 696484 | 13928 | `52b2819186ff186a49b9a3d3cb9e9943214ecffd773a56ae6c41f898cf6a2b37` | `(-0.103002, -0.05, -0.03485)` | `(0.108942, 0.05, 0.057501)` |
| `mecanum/meshes/cam_link.stl` | 4717884 | 94356 | `4bf9e75f7d7070db2263b978cbb34b159f981f6db57a50a64b64547c8835292e` | `(-0.026777, -0.044217, -0.025831)` | `(0.013735, 0.045583, 0.000353)` |
| `mecanum/meshes/lidar_link.stl` | 876284 | 17524 | `93c5329be97117f2b2f0196586627730e48798565415bcdcd7be4799ffb7047d` | `(-0.026803, -0.018745, -0.035)` | `(0.026934, 0.027787, 0.0)` |
| `mecanum/meshes/wheel_left_front_link.stl` | 10168884 | 203376 | `e44b584d2f6945443527811cad9aabba4e27f8851b711d6d55420650ae03d38b` | `(-0.03232, -0.024624, -0.032345)` | `(0.032332, 0.009776, 0.032331)` |
| `mecanum/meshes/wheel_left_rear_link.stl` | 10206484 | 204128 | `a0624c1830906c7b8299a7af09a2fceff7e8fb906ca6549dfd03825891eedc21` | `(-0.032302, -0.024624, -0.032339)` | `(0.032338, 0.009776, 0.032338)` |
| `mecanum/meshes/wheel_right_front_link.stl` | 10252084 | 205040 | `2e2ba3bd1a423b14a643fe31f58964b703482fbec2ea8d2b58eefe66ee7a5bf3` | `(-0.03232, -0.009776, -0.032345)` | `(0.032321, 0.024624, 0.032331)` |
| `mecanum/meshes/wheel_right_rear_link.stl` | 10193884 | 203876 | `11fd9a6e3c1cbbe139a458ef7e3792b45439b22625e600e2a35f8ebd764ebf88` | `(-0.032332, -0.009776, -0.032345)` | `(0.03232, 0.024624, 0.032331)` |
