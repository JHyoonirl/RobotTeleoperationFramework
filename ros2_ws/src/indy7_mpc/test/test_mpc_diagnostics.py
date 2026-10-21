import json
from types import SimpleNamespace

import numpy as np

from indy7_mpc.mpc_diagnostics import MpcDiagnosticsMixin


class _Logger:
    def info(self, _message):
        pass


class _DiagnosticOwner(MpcDiagnosticsMixin):
    diagnostic_logging = True
    diagnostic_log_directory = ""
    _diagnostic_file = None

    def __init__(self, log_directory):
        self.diagnostic_log_directory = str(log_directory)
        self.torque_limits = np.ones(6)
        self.model = SimpleNamespace(effortLimit=np.full(6, 2.0))

    def get_parameter(self, name):
        return SimpleNamespace(value=name)

    def get_logger(self):
        return _Logger()


def test_diagnostic_snapshot_serializes_parameters_after_helper_extraction(tmp_path):
    owner = _DiagnosticOwner(tmp_path)

    MpcDiagnosticsMixin._open_diagnostic_log(owner, tmp_path / "robot.urdf")

    assert owner.diagnostic_logging
    assert owner._diagnostic_file is not None
    parameter_file = owner._diagnostic_path.parent / "parameters.json"
    parameters = json.loads(parameter_file.read_text(encoding="utf-8"))
    assert parameters["diagnostic_rate"] == "diagnostic_rate"
    assert parameters["effective_torque_limits"] == [1.0] * 6
    assert parameters["urdf_effort_limits"] == [2.0] * 6
    owner._diagnostic_file.close()


def test_json_safe_recursively_handles_numpy_values():
    result = MpcDiagnosticsMixin._json_safe(
        {"values": np.array([1.0, np.nan]), "flag": np.bool_(True)}
    )

    assert result == {"values": [1.0, None], "flag": True}
