# Tube perception: legacy and configured requests

This onboard ROS1 package owns conservative RGB target selection and metric tube
cap/layout observations. It consumes camera images, stamped TF, and measured lift
feedback. It never issues robot motion commands. Existing `/select_tube_target_2d`
and `/detect_tube_cap` interfaces and their message definitions remain unchanged.

Two additive endpoints accept a frozen perception configuration per request:

- `/select_tube_target_2d_configured` (`agx_pick_msg/SelectTubeTarget2DConfigured`)
- `/detect_tube_cap_configured` (`agx_pick_msg/DetectTubeCapConfigured`)

Both requests carry `config_id`, `fixture_instance_id`, `fixture_profile_id`,
`projection_sha256`, `perception` (`TubePerceptionConfig`), `target_cap_color`, and
`not_before`. Metric requests additionally carry `mode` (`TARGET_ONLY` or
`FULL_LAYOUT`). Both responses preserve their legacy payload/status definitions
and echo the four identity strings. A caller must verify all echoed identities
before using the result, including when it requested a cached configuration.

`TubePerceptionConfig.msg` is the exact versioned wire schema. Version 1 contains
only geometry, camera/planning frame names, measured-lift feedback interpretation,
workspace limits, HSV/ROI/shape settings, and perception timing/consistency limits.
It contains no host paths, gripper parameters, arm motion settings, or generic YAML.
The field order is part of the digest contract. The Python `PerceptionConfig`
dataclass validates and copies each field, converts arrays to immutable tuples,
and rejects booleans in numeric fields, invalid lengths, nonfinite numbers,
unsupported versions, incompatible geometry, and malformed strings.

## Configuration ownership and immutable histories

The C++ domain resolver supplies `ResolvedTubeTaskConfig`; its public helper
`MakeTubePerceptionProjection(config, color)` selects one qualified classifier
and builds the narrow projection. `FillConfiguredTubePerceptionRequest` copies
that projection into either generated ROS request and computes its digest. The
caller sets `mode` and `not_before` for the observation it needs. Target color is
operation intent, distinct from fixture identity.

The node validates admission before creating a processing session. Each
`TubePerceptionSession` retains exactly one frozen `PerceptionRequestContext` and
its own metric and 2D observation epochs. Its history key includes the full config
ID, physical fixture instance, rack profile, projection digest, and selected
color. Two tables using one rack profile therefore cannot consume each other's
previous observations. Separate requests with the same complete identity reuse
that session; their `not_before` cutoffs still select only eligible fresh frames.
The request's identity, mode, and timestamp are copied before service waiting.

Legacy startup parameters are validated through the existing profile/context
loaders and then converted through `legacy_request_context` into the same narrow
immutable projection/session implementation. Later changes to ROS parameters do
not change either legacy or already-admitted configured sessions. There are no
parameter writes and no temporary global profile replacement.

The image-only service still selects a target before any metric lift/TF checks.
Metric services preserve measured-lift validation, stamped transforms, physical
cap-diameter checks, workspace limits, temporal consistency, and layout fitting.
The existing functions retain historical green-oriented names, but configured
sessions use the requested classifier's HSV bounds; tests exercise independent
green and blue selections.

The node owns one camera/lift transport established at startup. Version 1 requires
configured `planning_frame`, `color_frame`, `lift_state_topic`,
`synchronizer_queue_size`, and `max_sync_skew_sec` to match that transport. A
request asking for unsupported transport fails before binding or recording
history. Changing robot sensor interfaces requires restarting the node with the
commissioned startup configuration. This endpoint does not create subscriptions
from arbitrary request strings or resolve installation/TF graphs.

## Exact identity contract and trust boundary

`config_id` identifies the complete resolved task graph and must have the form
`sha256:` followed by 64 lowercase hexadecimal characters. The perception server
cannot reconstruct that full graph from a perception-only subset. The onboard
workflow is the trusted producer of the graph identity; this protocol is not an
authentication mechanism or proof of the omitted motion/robot configuration.

The server independently recomputes `projection_sha256` from the actual typed
content and identities. Its canonical byte stream is:

1. UTF-8 bytes `agx_tube_perception_projection_v1` followed by one NUL byte.
2. Fixture instance ID, fixture profile ID, and selected color, in that order.
   Each string is a little-endian unsigned 32-bit byte length followed by its UTF-8
   bytes. Strings are nonempty, contain no NUL, and are at most 4096 bytes.
3. Every `TubePerceptionConfig.msg` field in declaration order: strings encoded
   as above; `uint32`/`int32` as little-endian 32-bit integers; `float64` as IEEE754
   little-endian binary64. Fixed arrays contain their elements without a length
   prefix. All floating signed zero values normalize to positive zero.
4. SHA256 over those bytes, formatted as `sha256:` plus lowercase hexadecimal.

After checking the digest, the registry binds a full graph ID to the fixture
identities and every non-HSV projection fact. Each `(config_id, target_cap_color)`
then binds one exact classifier and projection digest. A graph can choose another
qualified color while retaining its geometry; changing geometry, frames, timing,
fixture identity, or an existing color classifier under the same graph ID is
rejected. First admission therefore pins verified projection content, rather than
merely caching a caller-supplied label. Cross-language tests compare the C++ and
Python byte streams and hashes directly.

Bindings are process-local and never silently evicted while histories exist.
`~maximum_configured_contexts` defaults to 16 and accepts integers from 1 through
128. Admission fails closed at capacity; a node restart starts a new binding
lifetime. Individual projections permit at most 128 consistency/synchronizer
frames, 4096 rack slots, and a 3600-second detection timeout. These limits bound
retained observations and waiting resources; exceeding them is an invalid request.

## Validation

`tests/test_tube_config_snapshot.py` exercises production Python admission, the
real node's handlers/vision code with ROS transport shells, and the production C++
resolver/projection helper. Tests cover interleaved installations, cached identity
reuse, conflicting digests, identity rebinding, parameter/request mutation after
admission, isolated histories, metric success with measured lift/TF, missing-lift
failure, transport rejection, resource limits, and byte-for-byte language parity.
The original ROS rostests remain registered for actual middleware integration.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  tests/test_tube_config_snapshot.py tests/test_ros1_tube_perception_contract.py \
  tests/test_tube_cap_service_logic.py tests/test_tube_target_selection.py \
  tests/test_tube_fixture_profile.py
```

Python production code remains compatible with Ubuntu 20.04 / Python 3.8. Test
fixtures are synthetic offboard inputs, not physical production calibration.
