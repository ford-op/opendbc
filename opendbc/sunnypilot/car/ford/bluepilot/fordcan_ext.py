"""
BluePilot (full BluePilot only): Ford CAN messages BluePilot changed from upstream's
opendbc/car/ford/fordcan.py, from BluePilot 7.
"""
from opendbc.car.ford.fordcan import CanBus


def create_acc_msg(packer, CAN: CanBus, long_active: bool, gas: float, accel: float, stopping: bool,
                   brake_actuate: bool, precharge_actuate: bool, v_ego_kph: float):
  """Upstream's ACC command (ACCDATA, 50 Hz) with the pre-charge and deceleration requests set
  separately, each with its own hysteresis (longitudinal_ext.py), instead of one brake request."""
  values = {
    "AccBrkTot_A_Rq": accel,                            # Brake total accel request: [-20|11.9449] m/s^2
    "Cmbb_B_Enbl": 1 if long_active else 0,             # Enabled: 0=No, 1=Yes
    "AccPrpl_A_Rq": gas,                                # Acceleration request: [-5|5.23] m/s^2
    "AccPrpl_A_Pred": -5.0,                             # Acceleration request: [-5|5.23] m/s^2
    "AccResumEnbl_B_Rq": 1 if long_active else 0,
    "AccVeh_V_Trg": v_ego_kph,                          # Target speed: [0|255] km/h
    "AccBrkPrchg_B_Rq": 1 if precharge_actuate else 0,  # Pre-charge brake request: 0=No, 1=Yes
    "AccBrkDecel_B_Rq": 1 if brake_actuate else 0,      # Deceleration request: 0=Inactive, 1=Active
    "AccStopStat_B_Rq": 1 if stopping else 0,
  }
  return packer.make_can_msg("ACCDATA", CAN.main, values)
