"""BluePilot (bp-full): Ford car state extras (sunnypilot/car/ford/bluepilot/carstate_ext.py)."""
import unittest
from types import SimpleNamespace as NS

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC
from opendbc.sunnypilot.car.ford.bluepilot.carstate_ext import (CarStateExt, FORD_FLAG_BP_HEV_BATTERY_DATA,
                                                                FORD_FLAG_BP_HEV_CLUSTER_DATA)

ButtonType = structs.CarState.ButtonEvent.Type


def _cs(cruise_enabled: bool) -> structs.CarState:
  ret = structs.CarState()
  ret.cruiseState.enabled = cruise_enabled
  return ret


def _cp(dec_press=0, inc_press=0, cncl_res_press=0, on_off_press=0):
  return NS(vl={"Steering_Data_FD1": {"CcAslButtnSetDecPress": dec_press, "CcAslButtnSetIncPress": inc_press,
                                       "CcAslButtnCnclResPress": cncl_res_press, "CcButtnOnOffPress": on_off_press}})


class TestComboButtonEvents(unittest.TestCase):
  """BluePilotDev/bluepilot#193: holding a combo cruise button produced a press/release storm."""
  def setUp(self):
    self.ext = CarStateExt(structs.CarParams())

  def _update(self, ret, **presses):
    self.ext.update_buttons(ret, _cp(**presses))
    return [(e.type, e.pressed) for e in self.ext.button_events]

  def test_sustained_hold_emits_exactly_one_press_and_one_release(self):
    ret = _cs(cruise_enabled=False)
    self.assertEqual(self._update(ret, dec_press=1), [(ButtonType.setCruise, True)])
    for _ in range(50):
      self.assertEqual(self._update(ret, dec_press=1), [], "spurious event while held (#193)")
    ret = _cs(cruise_enabled=True)  # cruise engages partway through the hold
    for _ in range(20):
      self.assertEqual(self._update(ret, dec_press=1), [], "spurious event after cruise engaged mid-hold (#193)")
    self.assertEqual(self._update(ret, dec_press=0), [(ButtonType.setCruise, False)])

  def test_inc_and_dec_do_not_cross_contaminate_setcruise(self):
    ret = _cs(cruise_enabled=False)
    self.assertEqual(self._update(ret, dec_press=1), [(ButtonType.setCruise, True)])
    for _ in range(10):
      self.assertEqual(self._update(ret, dec_press=1), [])

  def test_tap_while_engaged_emits_decel_not_setcruise(self):
    ret = _cs(cruise_enabled=True)
    self.assertEqual(self._update(ret, dec_press=1), [(ButtonType.decelCruise, True)])
    self.assertEqual(self._update(ret, dec_press=0), [(ButtonType.decelCruise, False)])

  def test_cancel_resume_combo_unaffected(self):
    ret = _cs(cruise_enabled=True)
    self.assertEqual(self._update(ret, cncl_res_press=1), [(ButtonType.cancel, True)])
    for _ in range(10):
      self.assertEqual(self._update(ret, cncl_res_press=1), [])
    self.assertEqual(self._update(ret, cncl_res_press=0), [(ButtonType.cancel, False)])

  def test_main_cruise_turning_cruise_on_sets_speed(self):
    self.assertEqual(self._update(_cs(False), on_off_press=1), [(ButtonType.mainCruise, True)])
    self.assertEqual(self._update(_cs(True), on_off_press=0),  # cruise comes on the next frame
                     [(ButtonType.mainCruise, False), (ButtonType.setCruise, True), (ButtonType.setCruise, False)])


def _car_state(car, extra_addrs=()):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0].update({a: 8 for a in extra_addrs})
  CP = CarInterface.get_params(car, fingerprint, [], False, False, False)
  return CP, CarState(CP, structs.CarParamsSP())


class TestCarStateExt(unittest.TestCase):
  def test_hev_flags_from_fingerprint(self):
    CP, _ = _car_state(CAR.FORD_ESCAPE_MK4, (0x365, 0x07A, 0x24B, 0x24C))
    self.assertTrue(CP.flags & FORD_FLAG_BP_HEV_CLUSTER_DATA and CP.flags & FORD_FLAG_BP_HEV_BATTERY_DATA)
    CP, _ = _car_state(CAR.FORD_ESCAPE_MK4, (0x07A, 0x24B))  # one battery message missing
    self.assertFalse(CP.flags & (FORD_FLAG_BP_HEV_CLUSTER_DATA | FORD_FLAG_BP_HEV_BATTERY_DATA))

  def test_values_from_can(self):
    for car in (CAR.FORD_F_150_MK14, CAR.FORD_ESCAPE_MK4, CAR.FORD_MUSTANG_MACH_E_MK1):
      with self.subTest(car=car):
        CP, CS = _car_state(car, (0x365, 0x07A, 0x24B, 0x24C))
        parsers = CS.get_can_parsers(CP, structs.CarParamsSP())
        packer = CANPacker(DBC[car][Bus.pt])
        pt, cam = parsers[Bus.pt].bus, parsers[Bus.cam].bus
        msgs = [
          packer.make_can_msg("Traffic_RecognitnData", cam, {"TsrVLim1MsgTxt_D_Rq": 55, "TsrVlUnitMsgTxt_D_Rq": 2}),
          packer.make_can_msg("Cluster_HEV_Data2", pt, {"PwrFlowTxt_D_Dsply": 5, "EngOnMsg1_D_Dsply": 1, "EffWhlLvl2_Pc_Dsply": 40}),
          packer.make_can_msg("Battery_Traction_1_FD1", pt, {"BattTrac_U_Actl": 350, "BattTrac_I_Actl": -20}),
          packer.make_can_msg("Battery_Traction_4_FD1", pt, {"BattTracSoc2_Pc_Actl": 61}),
          packer.make_can_msg("MtrTracData_1_FD1", pt, {"MtrTrac2_I_Actl": 30}),
          packer.make_can_msg("BCM_Lamp_Stat_FD1", pt, {"StopLghtOn_B_Stat": 1}),
        ]
        for p in parsers.values():
          p.update([(0, msgs)])
        _, ret_sp = CS.update(parsers)
        self.assertAlmostEqual(ret_sp.speedLimit, 55 * CV.MPH_TO_MS, places=3)
        bp = CS.bp.car_state_bp
        self.assertEqual((bp["hybridDrive"]["powerFlowMode"], bp["hybridDrive"]["engineOnReason"]), ("Electric Drive", "Acceleration"))
        self.assertEqual((bp["hybridBattery"]["voltActual"], bp["hybridBattery"]["socActual"]), (350, 61))
        self.assertEqual(bp["hybridBattery"]["ampsActual"], -30 if car == CAR.FORD_MUSTANG_MACH_E_MK1 else -20)
        self.assertTrue(bp["brakeLightStatus"]["brakeLightsOn"])

  def test_optional_messages_never_required(self):
    # a car without the hybrid, lamp or sign recognition messages keeps CAN validity, CAN and CAN FD alike
    for car in (CAR.FORD_ESCAPE_MK4, CAR.FORD_F_150_MK14):
      with self.subTest(car=car):
        CP, CS = _car_state(car, (0x365, 0x07A, 0x24B, 0x24C))
        pt, cam = CS.get_can_parsers(CP, structs.CarParamsSP()).values()
        for p in (pt, cam):
          required = {s.name for s in p.message_states.values() if not s.ignore_alive}
          self.assertFalse(required & {"Cluster_HEV_Data2", "Battery_Traction_1_FD1", "BCM_Lamp_Stat_FD1", "Traffic_RecognitnData"})


if __name__ == '__main__':
  unittest.main()
