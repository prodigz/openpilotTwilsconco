from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.modeld.constants import T_IDXS

BRAKE_SOURCES = {'lead0',
                 'lead1',
                 'lead0p1'}

LongCtrlState = car.CarControl.Actuators.LongControlState

STOPPING_EGO_SPEED = 0.02
STOPPING_TARGET_SPEED_OFFSET = 0.01
STARTING_TARGET_SPEED = 0.5
DECEL_THRESHOLD_TO_PID = 0.8

DECEL_STOPPING_TARGET = 0.25  # apply at least this amount of brake to maintain the vehicle stationary

RATE = 100.0

# As NOT per ISO 15622:2018 for all speeds
ACCEL_MIN_ISO = -3.5 # m/s^2
ACCEL_MAX_ISO = 3.5 # m/s^2


# TODO this logic isn't really car independent, does not belong here
def long_control_state_trans(active, long_control_state, v_ego, v_target, v_target_future, v_pid,
                             output_accel, brake_pressed, cruise_standstill, min_speed_can, 
                             MADS_lead_braking_enabled=False, gas=0.0, gear_shifter='',
                             long_plan_source=''):
  """Update longitudinal control state machine"""
  accelerating = v_target_future > v_target
  stopping_condition = (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and
                        ((v_target_future < STOPPING_EGO_SPEED and not accelerating) or
                         brake_pressed))

  starting_condition = v_target_future > STARTING_TARGET_SPEED and accelerating and not cruise_standstill

  if not active:
    if not brake_pressed \
        and gear_shifter in ['drive','low'] \
        and MADS_lead_braking_enabled \
        and gas < 1e-5 \
        and long_plan_source in BRAKE_SOURCES:
      long_control_state = LongCtrlState.pid
    else:
      long_control_state = LongCtrlState.off
  else:
    if long_control_state == LongCtrlState.off:
      if active:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.starting

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif output_accel >= -DECEL_THRESHOLD_TO_PID:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP):
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                            (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                            (CP.longitudinalTuning.kdBP, CP.longitudinalTuning.kdV),
                            derivative_period=0.1,
                            k_11 = 0.5, k_12 = 0.5, k_13 = 0.5, k_period=0.1,
                            rate=RATE,
                            sat_limit=0.8)
    self.v_pid = 0.0
    self.lead_present_last = False
    self.lead_gone_t = 0.
    self.lead_gone_smooth_accel_time = 4. # seconds after lead gone during which we'll smooth positive jerk
    self.last_output_accel = 0.0
    self.output_accel_pos_rate_ema_k = 1/10 # use exponential moving average on output accel, but only for positive jerk
    self.a_target = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, CP, long_plan, accel_limits, t_since_plan, MADS_lead_braking_enabled=False):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(CP.longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target) / CP.longitudinalActuatorDelayLowerBound - a_target

      v_target_upper = interp(CP.longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target) / CP.longitudinalActuatorDelayUpperBound - a_target
      a_target = min(a_target_lower, a_target_upper)

      v_target_future = speeds[-1]
    else:
      v_target = 0.0
      v_target_future = 0.0
      a_target = 0.0

    # TODO: This check is not complete and needs to be enforced by MPC
    a_target = clip(a_target, ACCEL_MIN_ISO, ACCEL_MAX_ISO)

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Update state machine
    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans(active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_future, self.v_pid, output_accel,
                                                       CS.brakePressed, CS.cruiseState.standstill, CP.minSpeedCan, MADS_lead_braking_enabled=MADS_lead_braking_enabled,gas=CS.gas,
                                                       gear_shifter=CS.gearShifter,
                                                       long_plan_source=long_plan.longitudinalPlanSource)

    v_ego_pid = max(CS.vEgo, CP.minSpeedCan)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off \
        or CS.gasPressed \
        or CS.brakePressed \
        or CS.gearShifter not in ['drive','low']:
      self.reset(v_ego_pid)
      output_accel = 0.
      
    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target
      
      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7 and v_target_future < v_target
      deadzone = interp(CS.vEgo, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      output_accel = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=freeze_integrator)

      if prevent_overshoot:
        output_accel = min(output_accel, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not CS.standstill or output_accel > -DECEL_STOPPING_TARGET:
        output_accel -= CP.stoppingDecelRate / RATE
      output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

      self.reset(CS.vEgo)

    # Intention is to move again, release brake fast before handing control to PID
    elif self.long_control_state == LongCtrlState.starting:
      if output_accel < -DECEL_THRESHOLD_TO_PID:
        output_accel += CP.startingAccelRate / RATE
      self.reset(CS.vEgo)
      
    t = sec_since_boot()
    lead_present = long_plan.leadDist > 0.
    if not lead_present and self.lead_present_last:
      self.lead_gone_t = t
    self.lead_present_last = lead_present
      
    if CS.vEgo > 0.5 and not lead_present and t - self.lead_gone_t < self.lead_gone_smooth_accel_time and output_accel > self.last_output_accel:
      output_accel =  self.output_accel_pos_rate_ema_k * output_accel + (1. - self.output_accel_pos_rate_ema_k) * self.last_output_accel    
    
    self.last_output_accel = output_accel
    final_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    self.a_target = a_target

    return final_accel
