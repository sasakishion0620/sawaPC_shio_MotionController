namespace mc {
  enum control_mode
  {
    idle,
    DA_check,
    ForceOffsetInit,
    EMG_iden,
    // remote,
    Record,
    // Bilateral,
    // Angle_EMS,
    PI_EMS,
    Motor_Point_Check,
    step_response_mode,
    NONLINEAR_EMS,
    control_mode_size
  };
} // namespace mc
