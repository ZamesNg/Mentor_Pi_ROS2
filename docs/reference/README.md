# Legacy Reference Evidence

This directory contains the raw material used to audit the original RRCLite
firmware, ROS 2 host packages, schematic, and hardware documentation. It is
historical evidence, not maintained RRCLite v2 source.

The parent repository tracks this policy README and intentionally ignores the
rest of `docs/reference/`. No v2 build, test, runtime, package, or release
artifact may read from the raw snapshot. Requirements and retained legacy
behavior must instead be captured in the tracked specifications under
`docs/framework/`.

## Why the raw tree is separate

The current evidence tree is approximately 163 MiB. It includes vendor source
trees, prebuilt libraries and firmware images, generated IDE/build material,
PDFs and images, and independently sourced project trees that may carry nested
Git history or other repository metadata. Its redistribution rights, exact
origin, revision history, and completeness have not been reviewed uniformly.

Committing the complete tree would therefore mix unaudited licensing and
provenance with maintained source, retain large binary/generated material, and
risk flattening nested repository boundaries. Its absence from Git is
intentional and is not permission to download an arbitrary replacement during a
build.

## Transfer and custody

Transfer the full evidence set separately as a checksummed archive. Preserve
the directory layout and include an inventory recording the archive SHA-256,
individual file hashes, byte size, acquisition date, source/custodian, and any
known repository revisions. Verify those checksums before using the material
for an audit. The archive and its checksum manifest must not be treated as a
project build dependency or pushed to the RRCLite v2 Git remote.

This README should travel with the separate archive so recipients understand
the boundary even when the evidence is copied outside this workspace.

## Future curated inclusion

Specific evidence may be proposed for tracked inclusion only after a license
and provenance review. A curated item should be minimal, directly needed for
traceability, attributed to its source and revision, covered by an identified
redistribution license, and accompanied by a checksum. Generated output,
prebuilt vendor binaries, and nested Git metadata should remain outside the
maintained repository unless a review records a compelling exception.
