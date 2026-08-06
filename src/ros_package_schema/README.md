# Offline ROS package schema

These files are immutable copies of the ROS package-format-three schema used
to validate both project-owned `package.xml` files without network access.
They were copied from `ros-infrastructure/rep` commit
`11ca24a41f31480dfb9562ba99f2a5b93d3ebda5`:

- `xsd/package_format3.xsd`, SHA-256
  `f096a197ed6d7878984bb2501a55f7f1bd4895d254399fb7857e154bfb644f41`;
- `xsd/package_common.xsd`, SHA-256
  `941ea8645344f3c4b7b9d7e68799898309d65a18225fa9cbef4169d95d1a3211`.

The upstream Robotics Enhancement Proposal content is published under
Creative Commons Attribution 4.0. Keep this provenance with redistributed
copies. Do not update either schema without changing the pinned commit and
hashes together and rerunning the offline ARM64 ROS test suite.
