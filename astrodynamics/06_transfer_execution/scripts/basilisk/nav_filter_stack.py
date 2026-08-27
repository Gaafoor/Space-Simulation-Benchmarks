"""Navigation filter / bridge for hardware sensor GNC."""

from __future__ import annotations

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import simpleNav, spacecraft
from Basilisk.utilities import RigidBodyKinematics, macros

from gnc_stack import NavNoiseConfig, configure_noisy_simple_nav
from sensor_stack import SensorStackConfig, attach_sensor_stack


class NavAttFromStarImu(sysModel.SysModel):
    """Direct-feed NavAtt message from star tracker quaternion + IMU rate."""

    def __init__(self):
        super().__init__()
        self.stInMsg = messaging.STSensorMsgReader()
        self.imuInMsg = messaging.IMUSensorMsgReader()
        self.navAttOutMsg = messaging.NavAttMsg()

    def Reset(self, CurrentSimNanos):
        return

    def UpdateState(self, CurrentSimNanos):
        st = self.stInMsg()
        imu = self.imuInMsg()
        out = messaging.NavAttMsgPayload()
        q = np.asarray(st.qInrtl2Case, dtype=float).reshape(4)
        out.sigma_BN = RigidBodyKinematics.EP2MRP(q).tolist()
        out.omega_BN_B = list(imu.AngVelPlatform)
        out.timeTag = CurrentSimNanos * 1e-9
        self.navAttOutMsg.write(out, CurrentSimNanos, self.moduleID)


def attach_hardware_nav(
    sc_sim,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    *,
    nav_noise: NavNoiseConfig | None = None,
    sensor_cfg: SensorStackConfig | None = None,
    sample_step_s: float = 60.0,
) -> dict:
    """Attach ST+IMU sensors and a direct NavAtt bridge; SimpleNav retained for translation."""
    sensors = attach_sensor_stack(
        sc_sim,
        task_name,
        sc_object,
        nav_noise=nav_noise,
        sensor_cfg=sensor_cfg,
        sample_step_s=sample_step_s,
    )

    trans_nav = simpleNav.SimpleNav()
    trans_nav.ModelTag = "HandoffTransNav"
    trans_nav.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    configure_noisy_simple_nav(trans_nav, NavNoiseConfig(enabled=False))

    nav_bridge = NavAttFromStarImu()
    nav_bridge.ModelTag = "HandoffNavAttBridge"
    nav_bridge.stInMsg.subscribeTo(sensors["star_tracker"].sensorOutMsg)
    nav_bridge.imuInMsg.subscribeTo(sensors["imu"].sensorOutMsg)

    nav_att_log = nav_bridge.navAttOutMsg.recorder(macros.sec2nano(max(sample_step_s, 0.5)))
    sc_sim.AddModelToTask(task_name, trans_nav, 109)
    sc_sim.AddModelToTask(task_name, nav_bridge, 108)
    sc_sim.AddModelToTask(task_name, nav_att_log)

    sensors.update(
        {
            "trans_nav": trans_nav,
            "nav_bridge": nav_bridge,
            "nav_att_msg": nav_bridge.navAttOutMsg,
            "nav_att_log": nav_att_log,
            "nav_mode": "hardware_direct",
        }
    )
    return sensors
