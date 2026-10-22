import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.structs import CarParams
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.values import CAR, DBC, ToyotaFlags, FW_QUERY_CONFIG, PLATFORM_CODE_ECUS, \
                                                  FUZZY_EXCLUDED_PLATFORMS, get_platform_codes
from opendbc.testing import fuzzy_test

Ecu = CarParams.Ecu


def check_fw_version(fw_version: bytes) -> bool:
  # TODO: just use the FW patterns, need to support all chunks
  return b'?' not in fw_version and b'!' not in fw_version


def cars_with(flags):
  return {c for c in CAR if c.config.flags & flags}


class TestToyotaInterfaces(unittest.TestCase):
  def test_car_flags(self):
    # Angle and radar-ACC cars are always TSS2 cars
    assert not (cars_with(ToyotaFlags.ANGLE_CONTROL | ToyotaFlags.RADAR_ACC) - cars_with(ToyotaFlags.TSS2))

  def test_lta_platforms(self):
    # At this time, only RAV4 2023 is expected to use LTA/angle control
    assert cars_with(ToyotaFlags.ANGLE_CONTROL) == {CAR.TOYOTA_RAV4_TSS2_2023}

  def test_tss2_dbc(self):
    # We make some assumptions about TSS2 platforms,
    # like looking up certain signals only in this DBC
    for car_model, dbc in DBC.items():
      if car_model.config.flags & ToyotaFlags.TSS2 and not (car_model.config.flags & ToyotaFlags.SECOC):
        assert dbc[Bus.pt] == "toyota_nodsu_pt_generated"

  def test_essential_ecus(self):
    # Asserts standard ECUs exist for each platform
    common_ecus = {Ecu.fwdRadar, Ecu.fwdCamera}
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        present_ecus = {ecu[0] for ecu in ecus}
        missing_ecus = common_ecus - present_ecus
        assert len(missing_ecus) == 0

        # Some exceptions for other common ECUs
        if car_model not in (CAR.TOYOTA_ALPHARD_TSS2,):
          assert Ecu.abs in present_ecus

        if car_model not in (CAR.TOYOTA_MIRAI,):
          assert Ecu.engine in present_ecus

        if car_model not in (CAR.TOYOTA_PRIUS_V, CAR.LEXUS_CTH):
          assert Ecu.eps in present_ecus


class TestToyotaSDSU(unittest.TestCase):
  def sdsu_fingerprint(self):
    return {0: {0x2FF: b'\x00' * 8}, 1: {}, 2: {}}

  def test_sdsu_detection(self):
    # TSS-P car with an sDSU dongle: openpilot longitudinal + stop and go
    CP = CarInterface.get_params(CAR.TOYOTA_COROLLA, self.sdsu_fingerprint(), [], False, False, False)
    assert CP.flags & ToyotaFlags.SDSU.value
    assert CP.openpilotLongitudinalControl
    assert CP.autoResumeSng
    assert CP.minEnableSpeed < 0

    # NO_DSU TSS-P car (radar-based ACC): dongle not detected
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY, self.sdsu_fingerprint(), [], False, False, False)
    assert not (CP.flags & ToyotaFlags.SDSU.value)
    assert not CP.openpilotLongitudinalControl

    # no dongle: TSS-P car keeps stock longitudinal
    CP = CarInterface.get_params(CAR.TOYOTA_COROLLA, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    assert not (CP.flags & ToyotaFlags.SDSU.value)
    assert not CP.openpilotLongitudinalControl

    # TSS2 car: dongle not detected, longitudinal works via the camera anyway
    CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS2, self.sdsu_fingerprint(), [], False, False, False)
    assert not (CP.flags & ToyotaFlags.SDSU.value)
    assert CP.openpilotLongitudinalControl

  def test_sdsu_distance_button(self):
    CP = CarParams(carFingerprint=CAR.TOYOTA_COROLLA.value, flags=ToyotaFlags.SDSU.value)
    parsers = CarState.get_can_parsers(CP)
    assert Bus.sdsu in parsers

    CP2 = CarParams(carFingerprint=CAR.TOYOTA_COROLLA.value)
    assert Bus.sdsu not in CarState.get_can_parsers(CP2)

    cs = CarState(CP)
    packer = CANPacker("toyota_sdsu")
    for btn in (1, 0, 1):
      addr, dat, bus = packer.make_can_msg("SDSU", 0, {"FD_BUTTON": btn})
      parsers[Bus.sdsu].update([1, [(addr, dat, bus)]])
      cs.update(parsers)
      assert cs.distance_button == btn


class TestToyotaSwappedTSS2Rack(unittest.TestCase):
  # 2021+ TSS2 steering rack swapped into a TSS-P Prius (EPS 8965B47070)
  TSS2_RACK_FW = b'8965B47070\x00\x00\x00\x00\x00\x00'
  GOOD_EPS_FW = b'8965B47060\x00\x00\x00\x00\x00\x00'
  BAD_SENSOR_FW = b'8965B47021\x00\x00\x00\x00\x00\x00'
  empty_fp = {0: {}, 1: {}, 2: {}}
  sdsu_fp = {0: {0x2FF: b'\x00' * 8}, 1: {}, 2: {}}

  def eps_car_fw(self, fw):
    return [CarParams.CarFw(ecu="eps", fwVersion=fw, address=0x7a1, subAddress=0)]

  def test_swapped_rack_supported(self):
    # No dashcam gate, treated like a native TSS2 EPS: no delay bump, no deadzone
    CP = CarInterface.get_params(CAR.TOYOTA_PRIUS, self.empty_fp, self.eps_car_fw(self.TSS2_RACK_FW), False, False, False)
    assert not CP.dashcamOnly
    self.assertAlmostEqual(CP.steerActuatorDelay, 0.12)
    self.assertAlmostEqual(CP.lateralTuning.torque.steeringAngleDeadzoneDeg, 0.0)
    # TSS2-rack steerRatio and EPS_SCALE
    self.assertAlmostEqual(CP.steerRatio, 15.86, places=2)
    from opendbc.car.toyota.values import EPS_SCALE
    self.assertEqual(CP.safetyConfigs[0].safetyParam & 0xff, EPS_SCALE[CAR.TOYOTA_PRIUS])
    self.assertEqual(CP.safetyConfigs[0].safetyParam & 0xff, 73)
    # Native TOYOTA_PRIUS_TSS2 torque parameters
    self.assertAlmostEqual(CP.lateralTuning.torque.latAccelFactor, 1.9726, places=3)
    self.assertAlmostEqual(CP.lateralTuning.torque.friction, 0.170968, places=4)
    self.assertAlmostEqual(CP.maxLateralAccel, 1.9104, places=3)

  def test_swapped_rack_keeps_sdsu_long(self):
    CP = CarInterface.get_params(CAR.TOYOTA_PRIUS, self.sdsu_fp, self.eps_car_fw(self.TSS2_RACK_FW), False, False, False)
    assert not CP.dashcamOnly
    assert CP.flags & ToyotaFlags.SDSU.value
    assert CP.openpilotLongitudinalControl

  def test_good_sensor_prius_unchanged(self):
    CP = CarInterface.get_params(CAR.TOYOTA_PRIUS, self.empty_fp, self.eps_car_fw(self.GOOD_EPS_FW), False, False, False)
    assert not CP.dashcamOnly
    self.assertAlmostEqual(CP.steerActuatorDelay, 0.12)
    self.assertAlmostEqual(CP.lateralTuning.torque.steeringAngleDeadzoneDeg, 0.0)
    # TSS-P rack keeps TSS-P torque tune (the TSS2 override must not leak)
    self.assertAlmostEqual(CP.lateralTuning.torque.latAccelFactor, 1.60, places=2)
    self.assertAlmostEqual(CP.lateralTuning.torque.friction, 0.151515, places=4)

  def test_bad_sensor_prius_still_delayed(self):
    # Original bad-angle-sensor behavior preserved for genuine 2016 cars
    CP = CarInterface.get_params(CAR.TOYOTA_PRIUS, self.empty_fp, self.eps_car_fw(self.BAD_SENSOR_FW), False, False, False)
    assert not CP.dashcamOnly
    self.assertAlmostEqual(CP.steerActuatorDelay, 0.25)
    self.assertAlmostEqual(CP.lateralTuning.torque.steeringAngleDeadzoneDeg, 0.2)


class TestToyotaFingerprint(unittest.TestCase):
  def test_non_essential_ecus(self):
    # Ensures only the cars that have multiple engine ECUs are in the engine non-essential ECU list
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        engine_ecus = {ecu for ecu in ecus if ecu[0] == Ecu.engine}
        assert (len(engine_ecus) > 1) == (car_model in FW_QUERY_CONFIG.non_essential_ecus[Ecu.engine]), \
          f"Car model unexpectedly {'not ' if len(engine_ecus) > 1 else ''}in non-essential list"

  def test_valid_fw_versions(self):
    # Asserts all FW versions are valid
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for fws in ecus.values():
          for fw in fws:
            assert check_fw_version(fw), fw

  # Tests for part numbers, platform codes, and sub-versions which Toyota will use to fuzzy
  # fingerprint in the absence of full FW matches:
  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_platform_code_ecus_available(self):
    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu == Ecu.eps and car_model in (CAR.TOYOTA_PRIUS_V, CAR.LEXUS_CTH,):
            continue
          if platform_code_ecu == Ecu.abs and car_model in (CAR.TOYOTA_ALPHARD_TSS2,):
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU sub-versions

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          codes = dict()
          for fw in fws:
            result = get_platform_codes([fw])
            # Check only one platform code and sub-version
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            assert 1 == len(list(result.values())[0]), f"Unable to parse FW: {fw}"
            codes |= result

          # Toyota places the ECU part number in their FW versions, assert all parsable
          # Note that there is only one unique part number per ECU across the fleet, so this
          # is not important for identification, just a sanity check.
          assert all(code.count(b"-") > 1 for code in codes), f"FW does not have part number: {fw} {codes}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b"F152607140\x00\x00\x00\x00\x00\x00",
      b"F152607171\x00\x00\x00\x00\x00\x00",
      b"F152607110\x00\x00\x00\x00\x00\x00",
      b"F152607180\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {b"F1526-07-1": {b"10", b"40", b"71", b"80"}}

    results = get_platform_codes([
      b"\x028646F4104100\x00\x00\x00\x008646G5301200\x00\x00\x00\x00",
      b"\x028646F4104100\x00\x00\x00\x008646G3304000\x00\x00\x00\x00",
    ])
    assert results == {b"8646F-41-04": {b"100"}}

    # Short version has no part number
    results = get_platform_codes([
      b"\x0235870000\x00\x00\x00\x00\x00\x00\x00\x00A0202000\x00\x00\x00\x00\x00\x00\x00\x00",
      b"\x0235883000\x00\x00\x00\x00\x00\x00\x00\x00A0202000\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {b"58-70": {b"000"}, b"58-83": {b"000"}}

    results = get_platform_codes([
      b"F152607110\x00\x00\x00\x00\x00\x00",
      b"F152607140\x00\x00\x00\x00\x00\x00",
      b"\x028646F4104100\x00\x00\x00\x008646G5301200\x00\x00\x00\x00",
      b"\x0235879000\x00\x00\x00\x00\x00\x00\x00\x00A4701000\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {b"F1526-07-1": {b"10", b"40"}, b"8646F-41-04": {b"100"}, b"58-79": {b"000"}}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        # If a platform has multiple matches, add it and its matches
        platforms_with_shared_codes |= {str(platform), *matches}

    assert platforms_with_shared_codes == FUZZY_EXCLUDED_PLATFORMS, (len(platforms_with_shared_codes), len(FW_VERSIONS))
