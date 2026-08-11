import importlib.util
from pathlib import Path


_SPEC = importlib.util.spec_from_file_location(
    "mentor_pi_vehicle_launch", Path(__file__).with_name("vehicle_launch.py")
)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def generate_launch_description():
    return _MODULE.generate_vehicle_launch("ackermann")
