# Ported from the DragonPilot fork (originally contributed to upstream
# openpilot, removed in 2024). The sDSU is a community-made hardware unit
# for TSS-P Toyotas: https://github.com/RetroPilot/ocelot/tree/main/firmware/smart_dsu

from opendbc.can.parser import CANParser
from opendbc.car.toyota.values import ToyotaFlags


class SDSU:
  def __init__(self, flags: int):
    self.enabled = bool(flags & ToyotaFlags.SDSU.value)
    self.dist_btn = 0

  def update_states(self, cp: CANParser):
    self.dist_btn = cp.vl["SDSU"]["FD_BUTTON"]
