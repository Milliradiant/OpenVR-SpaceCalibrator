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
The folder is kept under `recordingsMaxMB` (1024 by default): the oldest recordings go as the current one grows, and a session
that reaches the limit by itself stops being recorded there. `recordPoses: false` switches recording off. Both are settings of
the `driver_01spacecalibrator` section of SteamVR's settings.

A recording holds, each with the time it happened:

- every pose of the headset and of the calibration's two devices as their own driver gave it, before Space Calibrator changes
  anything (position, rotation, velocities, the pose's stated age, validity), and every other device's five times a second;
- each device's world-from-driver and driver-from-head frames, when they change;
- every calibration the overlay hands the driver, and the calibration the driver actually applies as it follows it;
- each device's serial, model and tracking system. (Recordings of drivers before the latency estimate was taken out also have
  the latency the driver used.)

The format is `src/common/pose_recording.h`: a magic, then records of a 12 byte header (type, device, payload length, time) and
a payload. A reader skips record types it does not know, and a file cut short by a crash reads up to its last whole record.

## Commands

```
spacecal-bench info RECORDING        what is in it: devices, pose rates, gaps, what the overlay set
spacecal-bench reference RECORDING   the session's calibration solved in hindsight
spacecal-bench score RECORDING       what was applied during the session, judged against that
spacecal-bench check RECORDING       each driver's velocities and pose ages against its own poses
spacecal-bench replay RECORDING      the overlay's calibration code run over the recording, judged the same way
spacecal-bench driver RECORDING      what the driver applies, its code run over the recording, judged the same way
spacecal-bench mount RECORDING       the mount estimator fed from the recording as the overlay feeds it
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

Errors are given **at the feet**: 1.5 m below the target device and half a metre to each side of that point, the worst of the
five. A calibration's rotation error is invisible at the device and is the whole of what is seen at the far end of a body, and
one turned about the vertical moves nothing right under the device.

`score` also follows what the driver applied under relative calibration, stretch by stretch: how far off it was, and how far it
moved from where it began. For a recording of a driver that still had a latency estimate, `check` gives its confidence, which
that driver's blend toward a relative calibration was multiplied by.

### The replay

`replay` builds the overlay's own `src/overlay/calibration.cpp`, unchanged, against stand-ins for SteamVR, the driver's shared
memory, the configuration and the log (`stubs/`, `src/replay.cpp`), and runs it over the recording as a session started fresh:
the same samples at the same times lead to the same calibrations, accepted and rejected for the same reasons. A change to the
calibration code is judged by building the bench with it and replaying the same recordings before and after.

`driver` does the same for what the driver applies of a continuous calibration (`src/driver/applied_calibration.cpp`, built into
the bench as it is): every pose of the two devices goes through it in the order and at the times they came in, with the
calibrations the overlay set live (mounts under relative calibration, world calibrations otherwise), or with `--reference-mount`
the hindsight mount throughout, which shows the driver's part alone. It also compares the calibration the recorded driver began
each stretch of relative calibration with against this code's at the same pose: a stretch begins from its first pose pair either
way. `replay --driver` runs the driver's code on what the replayed overlay sets: the two together are what would have reached
SteamVR. `--follow`, `--turn-scale`, `--catch-up` and the rest of `driverSettings` in `src/main.cpp` try other settings.

Both say how still what was applied held, which is what a user sees of a calibration's corrections: how fast it moved the feet
from one reading to the next (at every pose of the reference, and ten times a second to set beside a recorded driver's), the same
while the head turned faster than 60 degrees a second (a calibration that moves with the head is the jitter that follows it), how
far it strayed from its own five second mean, and how far off the reference it was. From 30 s after the first reading, where the
reference device sent its poses at 50 a second or more (a headset taken off sends a few) and away from a step of the reference.
`--trace FILE` writes what was applied at every pose of the reference, its error and what the code was doing.

### The mount

Under relative calibration the calibration is the mount: where the target device sits on the reference device. The overlay's
mount estimator (`src/overlay/mount_estimator.cpp`) adds it up from how the two devices move together, over a session and from
one session to the next, and `replay` runs it as part of the overlay's code: `--save-mount FILE` and `--load-mount FILE` carry
what a profile would keep of it from one replayed session to another. With relative calibration off (`--relative 0`) a known
mount gives the world calibration at every moment, with no batch of samples solved; `--no-mount` is the profile's
`use_mount_estimate` switched off, every calibration from a batch as before, which is what to compare against.

`mount` runs the estimator alone, fed each device's latest pose twenty times a second as the overlay has them (`--after
RECORDING[,RECORDING...]` puts it through those sessions first, one after the other; `--carry N` lets it take at most N motions'
worth of them into the session), and says how far its mount is from the hindsight one as the session goes. It
also judges mounts without taking any mount's word for it: with the right mount the world calibration one pose pair gives does
not care which way the head is turned, and with one that is off it swings as the head turns. So a mount is as good as that
calibration is steady, at the feet, and the hindsight mount, the estimator's and the ones the overlay set live are each put to
that.

### The simulation

`simulate` writes a session through the same writer the driver uses, with a mount and time offset it states in the recording.
The bench's test holds `reference` to finding them: within 0.05 degrees, 1 mm and 1 ms. `--frame-step S` and
`--reference-frame-step S` have a tracking system re-base its world at that time (the device's world-from-driver steps while its
driver pose runs on), which the overlay's code has to come through: a world calibration recovered at once, a relative
calibration's mount left where it was.

## Tests

`ctest` runs seven: the mount estimator on made-up motions of two devices fixed to each other (`tests/test_mount_estimator.cpp`:
it finds the mount, leaves out motions that do not fit, comes back the same from what a profile keeps, and starts over when the
target is moved on the reference); the recording's writer and reader against each other (`tests/test_recording.cpp`); the driver's recorder
as the driver builds it against a stand-in for SteamVR's settings and device properties, called from several threads at once
(`tests/test_session_recorder.cpp`); what the driver applies on made-up poses (`tests/test_applied_calibration.cpp`): the
calibration the poses were made with, followed without a step when the spaces drift apart and caught up with when one recentres,
held while the headset's tracking has stopped and it still says its pose is valid, a step of a world taken back at once, and a
world calibration gone to without a step; the slots the driver and the overlay
share their poses, transforms and the headset's metadata through (`tests/test_shared_state.cpp`, built once with each side's
DriverPose_t): written and read from several threads and from two processes at once, every copy read has to be one a writer wrote
whole, and a writer that stops halfway has to leave the slot readable; and the bench on simulated sessions: the reference against
the known answer, the overlay's and the driver's code through a step of each tracking system's frame.
