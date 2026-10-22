"""ROS-free validation for the physical rover camera readiness contract."""

from dataclasses import dataclass
import math
from numbers import Real
from typing import Tuple


EXPECTED_WIDTH = 640
EXPECTED_HEIGHT = 480
EXPECTED_FRAME_ID = "cam2_color_optical_frame"
MAXIMUM_AGE_S = 0.5
MINIMUM_RATE_HZ = 20.0
MINIMUM_SAMPLE_COUNT = 10
MINIMUM_OBSERVATION_SPAN_S = 0.25
MAXIMUM_SAMPLE_WINDOW = 20


@dataclass(frozen=True)
class CameraReadinessSample:
    """One exact Image and CameraInfo pair sampled by the ROS executable."""

    image_width: object
    image_height: object
    image_frame_id: object
    image_stamp_s: object
    camera_info_width: object
    camera_info_height: object
    camera_info_frame_id: object
    camera_info_stamp_s: object
    camera_matrix_k: object
    arrival_monotonic_s: object


class SynchronizedCameraSampleWindow:
    """Bounded paired-sample store whose snapshots are immutable tuples."""

    def __init__(self, maximum_sample_count):
        if (
            isinstance(maximum_sample_count, bool)
            or not isinstance(maximum_sample_count, int)
            or maximum_sample_count <= 0
        ):
            raise ValueError("maximum_sample_count must be a positive integer")
        self._maximum_sample_count = maximum_sample_count
        self._samples = ()

    def append(self, sample):
        if not isinstance(sample, CameraReadinessSample):
            raise ValueError("sample must be a CameraReadinessSample")
        self._samples = (self._samples + (sample,))[-self._maximum_sample_count :]

    def snapshot(self):
        return self._samples


@dataclass(frozen=True)
class CameraReadinessObservation:
    """Immutable facts evaluated by the ROS-free readiness contract."""

    image_width: object
    image_height: object
    image_frame_id: object
    image_stamp_s: object
    camera_info_width: object
    camera_info_height: object
    camera_info_frame_id: object
    camera_info_stamp_s: object
    camera_matrix_k: object
    observed_rate_hz: object
    arrival_rate_hz: object
    sample_count: object
    header_observation_span_s: object
    arrival_observation_span_s: object
    header_timestamps_valid: object
    arrival_timestamps_valid: object
    tf_available: object


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        converted = float(value)
    except OverflowError:
        return None
    if not math.isfinite(converted):
        return None
    return converted


def _has_expected_dimensions(width, height):
    converted_width = _finite_number(width)
    converted_height = _finite_number(height)
    return converted_width == EXPECTED_WIDTH and converted_height == EXPECTED_HEIGHT


def _has_valid_camera_matrix(camera_matrix_k):
    if isinstance(camera_matrix_k, (str, bytes)):
        return False
    try:
        values = tuple(camera_matrix_k)
    except TypeError:
        return False
    if len(values) != 9:
        return False
    converted = [_finite_number(value) for value in values]
    if any(value is None for value in converted):
        return False
    return (
        converted[0] > 0.0
        and converted[4] > 0.0
        and converted[6:9] == [0.0, 0.0, 1.0]
    )


def _stamp_error(stamp_s, ros_now_s, prefix):
    stamp = _finite_number(stamp_s)
    if stamp is None or stamp <= 0.0:
        return "{0}_stamp_invalid".format(prefix)
    if ros_now_s is None:
        return None
    if stamp > ros_now_s:
        return "{0}_stamp_in_future".format(prefix)
    if ros_now_s - stamp > MAXIMUM_AGE_S:
        return "{0}_stamp_stale".format(prefix)
    return None


def _stamps_match(image_stamp_s, camera_info_stamp_s):
    image_stamp = _finite_number(image_stamp_s)
    camera_info_stamp = _finite_number(camera_info_stamp_s)
    if image_stamp is None or camera_info_stamp is None:
        return False
    return image_stamp == camera_info_stamp


def _series_rate_and_span(samples, attribute):
    if len(samples) < 2:
        return None, None, False
    values = [_finite_number(getattr(sample, attribute)) for sample in samples]
    if any(value is None or value <= 0.0 for value in values):
        return None, None, False
    for previous, current in zip(values, values[1:]):
        if current <= previous:
            return None, None, False
    span_s = values[-1] - values[0]
    return float(len(values) - 1) / span_s, span_s, True


def camera_readiness_observation(samples, tf_available):
    """Build one complete immutable observation from synchronized samples."""

    snapshot = tuple(samples)
    if snapshot:
        latest = snapshot[-1]
        if not all(isinstance(sample, CameraReadinessSample) for sample in snapshot):
            raise ValueError("samples must be CameraReadinessSample instances")
    else:
        latest = None
    header_rate_hz, header_span_s, header_timestamps_valid = _series_rate_and_span(
        snapshot, "image_stamp_s"
    )
    arrival_rate_hz, arrival_span_s, arrival_timestamps_valid = _series_rate_and_span(
        snapshot, "arrival_monotonic_s"
    )
    return CameraReadinessObservation(
        image_width=getattr(latest, "image_width", None),
        image_height=getattr(latest, "image_height", None),
        image_frame_id=getattr(latest, "image_frame_id", None),
        image_stamp_s=getattr(latest, "image_stamp_s", None),
        camera_info_width=getattr(latest, "camera_info_width", None),
        camera_info_height=getattr(latest, "camera_info_height", None),
        camera_info_frame_id=getattr(latest, "camera_info_frame_id", None),
        camera_info_stamp_s=getattr(latest, "camera_info_stamp_s", None),
        camera_matrix_k=getattr(latest, "camera_matrix_k", None),
        observed_rate_hz=header_rate_hz,
        arrival_rate_hz=arrival_rate_hz,
        sample_count=len(snapshot),
        header_observation_span_s=header_span_s,
        arrival_observation_span_s=arrival_span_s,
        header_timestamps_valid=header_timestamps_valid,
        arrival_timestamps_valid=arrival_timestamps_valid,
        tf_available=tf_available,
    )


def _sample_count_error(sample_count):
    converted = _finite_number(sample_count)
    if converted is None or not converted.is_integer():
        return "sample_count_invalid"
    if converted < MINIMUM_SAMPLE_COUNT:
        return "sample_count_insufficient"
    return None


def _span_error(span_s, prefix):
    converted = _finite_number(span_s)
    if converted is None or converted <= 0.0:
        return "{0}_invalid".format(prefix)
    if converted < MINIMUM_OBSERVATION_SPAN_S:
        return "{0}_insufficient".format(prefix)
    return None


def _rate_error(rate_hz, prefix):
    converted = _finite_number(rate_hz)
    if converted is None:
        return "{0}_invalid".format(prefix)
    if converted < MINIMUM_RATE_HZ:
        return "{0}_below_minimum".format(prefix)
    return None


def camera_readiness_errors(
    observation: CameraReadinessObservation, ros_now_s: object
) -> Tuple[str, ...]:
    """Return every failed readiness diagnostic in deterministic order."""

    errors = []
    now = _finite_number(ros_now_s)
    if now is None or now <= 0.0:
        errors.append("ros_time_invalid")
        now = None

    if not _has_expected_dimensions(
        observation.image_width, observation.image_height
    ):
        errors.append("image_dimensions_invalid")
    if not _has_expected_dimensions(
        observation.camera_info_width, observation.camera_info_height
    ):
        errors.append("camera_info_dimensions_invalid")
    if observation.image_frame_id != EXPECTED_FRAME_ID:
        errors.append("image_frame_invalid")
    if observation.camera_info_frame_id != EXPECTED_FRAME_ID:
        errors.append("camera_info_frame_invalid")

    image_stamp_error = _stamp_error(observation.image_stamp_s, now, "image")
    if image_stamp_error is not None:
        errors.append(image_stamp_error)
    camera_info_stamp_error = _stamp_error(
        observation.camera_info_stamp_s, now, "camera_info"
    )
    if camera_info_stamp_error is not None:
        errors.append(camera_info_stamp_error)
    if not _stamps_match(
        observation.image_stamp_s, observation.camera_info_stamp_s
    ):
        errors.append("image_camera_info_stamp_mismatch")
    if not _has_valid_camera_matrix(observation.camera_matrix_k):
        errors.append("camera_info_k_invalid")

    observed_rate_error = _rate_error(observation.observed_rate_hz, "observed_rate")
    if observed_rate_error is not None:
        errors.append(observed_rate_error)
    arrival_rate_error = _rate_error(observation.arrival_rate_hz, "arrival_rate")
    if arrival_rate_error is not None:
        errors.append(arrival_rate_error)
    sample_count_error = _sample_count_error(observation.sample_count)
    if sample_count_error is not None:
        errors.append(sample_count_error)
    header_span_error = _span_error(
        observation.header_observation_span_s, "header_observation_span"
    )
    if header_span_error is not None:
        errors.append(header_span_error)
    arrival_span_error = _span_error(
        observation.arrival_observation_span_s, "arrival_observation_span"
    )
    if arrival_span_error is not None:
        errors.append(arrival_span_error)
    if observation.header_timestamps_valid is not True:
        errors.append("header_timestamps_invalid")
    if observation.arrival_timestamps_valid is not True:
        errors.append("arrival_timestamps_invalid")
    if observation.tf_available is not True:
        errors.append("camera_tf_unavailable")
    return tuple(errors)
