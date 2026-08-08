set pagination off
set confirm off
set logging file build/board-characterization-gdb.txt
set logging overwrite on
set logging enabled on

set $saved_dbgmcu_apb1_fz = *(unsigned int *)0xE0042008
set *(unsigned int *)0xE0042008 = $saved_dbgmcu_apb1_fz | 0x00001000

define rrclite_imu
  monitor halt
  p/x rrclite_imu_characterization_snapshot.sequence
  p rrclite_imu_characterization_snapshot
  p/x rrclite_imu_characterization_snapshot.sequence
  continue
end

document rrclite_imu
Capture one coherent raw IMU snapshot. Run once in each of the six stationary
orientations and during positive rotation about robot X, Y, and Z.
end

define rrclite_encoders
  monitor halt
  p/x *(unsigned int *)0x40000c24
  p/x *(unsigned int *)0x40000024
  p/x (*(unsigned int *)0x40000824) & 0xffff
  p/x (*(unsigned int *)0x40000424) & 0xffff
  continue
end

document rrclite_encoders
Capture TIM5/TIM2/TIM4/TIM3 counters. Run before and after moving exactly one
wheel by hand in each direction.
end

define rrclite_finish
  monitor halt
  set *(unsigned int *)0xE0042008 = $saved_dbgmcu_apb1_fz
  set logging enabled off
  monitor reset
  continue
end

document rrclite_finish
Restore the exact debugger watchdog-freeze register, stop logging, and reset.
Always run this before detaching.
end

echo Motor power must remain disconnected. Never run load.\n
echo Commands: rrclite_imu, rrclite_encoders, rrclite_finish\n
