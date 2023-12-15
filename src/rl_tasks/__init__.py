from .door_env        import DoorEnv
from .drawer_env      import DrawerEnv
from .hatch_env       import HatchEnv
from .peg_env         import PegEnv
from .real_drawer_env import RealDrawerEnv
from .real_door_env   import RealDoorEnv
from .utils           import Gamepad

ENV_TYPES = {'door': DoorEnv,
             'peg' : PegEnv,
             'drawer': DrawerEnv,
             'real_drawer' : RealDrawerEnv,
             'real_door'   : RealDoorEnv,
             'hatch'       : HatchEnv}
