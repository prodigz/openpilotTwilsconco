from common.numpy_fast import interp
from common.params import Params
from selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE

_DP_E2E_LEAD_COUNT = 5

_DP_E2E_STOP_BP = [0., 10., 20., 30., 40., 50., 55.]
_DP_E2E_STOP_DIST = [10, 30., 50., 70., 80., 90., 120.]
_DP_E2E_STOP_COUNT = 3

_DP_E2E_SNG_COUNT = 3
_DP_E2E_SNG_ACC_COUNT = 5
_DP_E2E_SWAP_COUNT = 10

_DP_E2E_TF_COUNT = 5


class DynamicLongitudinalController:

  def __init__(self):
    self._params = Params()
    self._dynamic_e2e_longitudinal = False
    self._mode = 'blended'

    # conditional e2e
    self.dp_e2e_has_lead = False
    self.dp_e2e_lead_last = False
    self.dp_e2e_lead_count = 0
    self.dp_e2e_sng = False
    self.dp_e2e_sng_count = 0
    self.dp_e2e_standstill_last = False
    self.dp_e2e_swap_count = 0
    self.dp_e2e_stop_count = 0
    self.dp_e2e_tf_count = 0
    pass

  def read_params(self):
    self._dynamic_e2e_longitudinal = self._params.get_bool('DynamicLongitudinalControl')
    pass

  def _set_dp_e2e_mode(self, mode, force=False):
    if force:
      self.dp_e2e_swap_count = 0
      self._mode = mode
      return

    else:
      # prevent switching in a short period of time.
      if self._mode == mode:
        self.dp_e2e_swap_count = 0
      else:
        self.dp_e2e_swap_count += 1

      if self.dp_e2e_swap_count >= _DP_E2E_SWAP_COUNT:
        self._mode = mode

  def _process_conditional_e2e(self, radar_unavailable, car_state, lead_one, md):
    v_ego_kph = car_state.vEgo * 3.6

    # make sure it see lead enough time
    if lead_one.status != self.dp_e2e_lead_last:
      self.dp_e2e_lead_count = 0
    else:
      self.dp_e2e_lead_count += 1
      if self.dp_e2e_lead_count >= _DP_E2E_LEAD_COUNT:
        self.dp_e2e_has_lead = lead_one.status
    self.dp_e2e_lead_last = lead_one.status

    # when standstill, always e2e
    if car_state.standstill:
      self.dp_e2e_sng_count = 0
      self.dp_e2e_sng = False
      return self._set_dp_e2e_mode('blended')

    if self.dp_e2e_standstill_last and not car_state.standstill:
      self.dp_e2e_sng = True

    # when sng, we e2e for 0.5 secs
    if self.dp_e2e_sng:
      self.dp_e2e_sng_count += 1
      if self.dp_e2e_sng_count > _DP_E2E_SNG_COUNT:
        if self.dp_e2e_sng_count > _DP_E2E_SNG_ACC_COUNT:
          self.dp_e2e_sng = False
        return self._set_dp_e2e_mode('acc', True)
      return self._set_dp_e2e_mode('blended')

    # when we see a lead
    # voacc cars only
    if radar_unavailable and self.dp_e2e_has_lead and lead_one.vRel != 0:
      ttc = lead_one.dRel / lead_one.vRel
      if ttc <= interp(car_state.vEgo, [0., 22.2, 25.], [.85, 1., 1.22]):
        self.dp_e2e_tf_count += 1
      else:
        self.dp_e2e_tf_count = 0
      if self.dp_e2e_tf_count > _DP_E2E_TF_COUNT:
        return self._set_dp_e2e_mode('blended', True)

    # stop sign detection
    if abs(car_state.steeringAngleDeg) <= 60 and len(md.orientation.x) == len(md.position.x) == TRAJECTORY_SIZE:
      if md.position.x[TRAJECTORY_SIZE - 1] < interp(v_ego_kph, _DP_E2E_STOP_BP, _DP_E2E_STOP_DIST):
        self.dp_e2e_stop_count += 1
      else:
        self.dp_e2e_stop_count = 0
    else:
      self.dp_e2e_stop_count = 0

    if self.dp_e2e_stop_count >= _DP_E2E_STOP_COUNT:
      return self._set_dp_e2e_mode('blended', True)

    return self._set_dp_e2e_mode('acc')

  def set_mpc_mode(self, mode, radar_unavailable, car_state, lead_one, md):
    if not self._dynamic_e2e_longitudinal:
      return 'blended'

    self._mode = mode
    self._process_conditional_e2e(radar_unavailable, car_state, lead_one, md)
    return self._mode
