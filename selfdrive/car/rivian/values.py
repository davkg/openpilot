from collections import namedtuple
from dataclasses import dataclass, field

from cereal import car
from openpilot.selfdrive.car import AngleRateLimit
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


@dataclass
class RivianCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass
class RivianPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('rivian_can', None))


class CAR(Platforms):
  RIVIAN_R1S = RivianPlatformConfig(
    [RivianCarDocs("Rivian R1S 2022")],
    CarSpecs(mass=3206., wheelbase=3.076, steerRatio=12.0),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=0x10,
      bus=1,
      obd_multiplexing=False,
    ),
  ]
)

GEAR_MAP = {
  "VDM_PRNDL_STATUS_NOT_DEFINED": car.CarState.GearShifter.unknown,
  "VDM_PRNDL_STATUS_PARK": car.CarState.GearShifter.park,
  "VDM_PRNDL_STATUS_REVERSE": car.CarState.GearShifter.reverse,
  "VDM_PRNDL_STATUS_NEUTRAL": car.CarState.GearShifter.neutral,
  "VDM_PRNDL_STATUS_DRIVE": car.CarState.GearShifter.drive,
}

BUTTONS = [
  Button(car.CarState.ButtonEvent.Type.leftBlinker, "SCCM_leftStalk", "SCCM_turnIndicatorStalkStatus", [3, 4]),
  Button(car.CarState.ButtonEvent.Type.rightBlinker, "SCCM_leftStalk", "SCCM_turnIndicatorStalkStatus", [1, 2]),
  Button(car.CarState.ButtonEvent.Type.accelCruise, "VCLEFT_switchStatus", "VCLEFT_swcRightScrollTicks", list(range(1, 10))),
  Button(car.CarState.ButtonEvent.Type.decelCruise, "VCLEFT_switchStatus", "VCLEFT_swcRightScrollTicks", list(range(-9, 0))),
  Button(car.CarState.ButtonEvent.Type.cancel, "SCCM_rightStalk", "SCCM_rightStalkStatus", [1, 2]),
  Button(car.CarState.ButtonEvent.Type.resumeCruise, "SCCM_rightStalk", "SCCM_rightStalkStatus", [3, 4]),
]


class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 7.0, 0.8])
  JERK_LIMIT_MAX = 8
  JERK_LIMIT_MIN = -8

  def __init__(self, CP):
    pass


DBC = CAR.create_dbc_map()