# The calibration's bench

Space Calibrator's driver records every session's poses (`src/driver/session_recorder.cpp`). The bench reads those recordings
and answers, after the fact, the question nobody can answer by eye in the headset: how good was the calibration, and what would
other calibration code have done with the same session.

It is a project of its own and not part of the app's build:

```
git submodule update --init vendor/openvr vendor/fmt vendor/glaze vendor/eigen
cmake -S bench -B build-bench -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-bench
ctest --test-dir build-bench
```

## Recordings

One file a session, `poses_YYYYMMDD_HHMMSS.screc`, in the `recordings` folder beside Space Calibrator's logs and configuration.
The oldest are deleted when the folder passes `recordingsMaxMB` (1024 by default), and `recordPoses: false` switches recording
off; both are settings of the `driver_01spacecalibrator` section of SteamVR's settings.

A recording holds, each with the time it happened:

- every pose of the headset and of the calibration's two devices as their own driver gave it, before Space Calibrator changes
  anything (position, rotation, velocities, the pose's stated age, validity), and every other device's five times a second;
- each device's world-from-driver and driver-from-head frames, when they change;
- every calibration the overlay hands the driver, and the calibration the driver actually applies as it blends toward it;
- the driver's latency estimate between the two devices, and each device's serial, model and tracking system.

The format is `src/common/pose_recording.h`: a magic, then records of a 12 byte header (type, device, payload length, time) and
a payload. A reader skips record types it does not know, and a file cut short by a crash reads up to its last whole record.

## Commands

```
spacecal-bench info RECORDING        what is in it: devices, pose rates, gaps, what the overlay set
spacecal-bench reference RECORDING   the session's calibration solved in hindsight
spacecal-bench score RECORDING       what was applied during the session, judged against that
spacecal-bench check RECORDING       each driver's velocities and pose ages against its own poses
spacecal-bench replay RECORDING      the overlay's calibration code run over the recording, judged the same way
spacecal-bench simulate RECORDING    write a made-up session with a known answer
```

### The reference

With the whole session to look at, three things can be solved far better than anything running live can know them:

1. **The time offset** between the two devices' poses, from the cross-correlation of their turning speeds. Turning speed does
   not depend on any calibration, so this needs nothing else to be right first.
2. **The mount**: the target's pose in the reference's frame, which is constant while the target is fixed to the reference. From
   pairs of poses a moment apart, `H_a^-1 H_b = L (T_a^-1 T_b) L^-1`: the two devices' own motions between the same two moments
   differ only by the mount, whatever the world calibration is. Thousands of pairs across the session, trimmed of outliers.
3. **The world calibration over time**, `H L T^-1` for every pose pair, smoothed over a few seconds and split where it steps
   (a playspace recentre, a tracking system losing its place).

Errors are given **at the feet**, 1.5 m below the target device: a calibration's rotation error is invisible at the device and
is the whole of what is seen at the far end of a body.

### The replay

`replay` builds the overlay's own `src/overlay/calibration.cpp`, unchanged, against stand-ins for SteamVR, the driver's shared
memory, the configuration and the log (`stubs/`, `src/replay.cpp`), and runs it over the recording as a session started fresh:
the same samples at the same times lead to the same calibrations, accepted and rejected for the same reasons. A change to the
calibration code is judged by building the bench with it and replaying the same recordings before and after.

### The simulation

`simulate` writes a session through the same writer the driver uses, with a mount and time offset it states in the recording.
The bench's test holds `reference` to finding them: within 0.05 degrees, 1 mm and 1 ms.
