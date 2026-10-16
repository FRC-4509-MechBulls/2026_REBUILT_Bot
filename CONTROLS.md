# Enkidu Controls

Driver is on port 0, operator on port 1 (Driver Station USB tab). Both are Xbox controllers.

## Driver (port 0)

| Input | Action |
|---|---|
| Left stick | Drive (field-centric: up on the stick = away from you) |
| Right stick left/right | Turn |
| Right trigger (hold) | Slow mode: half speed and half turning |
| Left trigger (hold) | Auto-aim: keep driving with the left stick while the robot turns to face the target. In your alliance zone it aims at your hub; outside it, it aims back into your alliance zone for passing. |
| Y (hold) | Face away from you |
| A (hold) | Face toward you |
| X (hold) | Face left |
| B (hold) | Face right |
| Start | Reset field-centric. Point the robot straight away from you first, then press. |
| D-pad up | Reset position: robot is parked at the hub reset spot |
| D-pad left | Reset position: robot is parked at the left trench reset spot |
| D-pad right | Reset position: robot is parked at the right trench reset spot |
| D-pad down | Not used (no cameras) |
| Left bumper (hold) | Drive backward over the bump (stops after 1.3 s or when you let go) |

While holding Y/A/X/B or the left trigger, the left stick still drives, but the right stick doesn't turn the robot.

**If driving feels rotated or backwards** (for example, when no auto ran), point the robot away from you and press **Start**.

**If auto-aim is off**, the robot's position has drifted (there are no cameras to correct it). Park at a reset spot and press the matching D-pad button. The D-pad resets only position, not which way the robot is facing.

## Operator (port 1)

| Input | Action |
|---|---|
| Right trigger (hold) | Spin up the shooter. Releasing stops the shooter and the indexer. |
| X (hold) | Feed balls into the shooter. Waits until the shooter is up to speed (at most 1 s), then feeds until you let go. |
| Y (hold) | Run the indexer backwards (unjam) |
| Left trigger (hold) | Spin the shooter backwards (unjam). Releasing stops the shooter and the indexer. |
| A (hold) | Run the intake roller in at full speed |
| Right stick up/down | Intake roller by hand: down = in, up = out |
| Left stick up/down | Hopper by hand: up = extend, down = retract |
| Left bumper | Extend the hopper (stops on its own) |
| Right bumper | Retract the hopper (stops on its own) |
| B | Stop everything: shooter, indexer, roller and hopper |
| Start | Zero the hopper position. **Only press with the hopper fully retracted.** Works while disabled. |

**To shoot:** hold the right trigger, wait for **Shooter Ready** on the dashboard, then hold X. Let go of both when you're done.

While the left or right bumper is moving the hopper (up to 1.75 s), the operator sticks and A don't do anything.

## Dashboard (Teleoperated tab)

| Widget | Meaning |
|---|---|
| Hub Active | Green when your hub is scoring this shift |
| Shift Time Left | Time until the hub switches; turns yellow at 10 s and red at 5 s |
| Shooter Ready | Green when the shooter is up to speed. Feed now. |
| Target / Distance | What auto-aim is pointing at and how far away it is |
| Intaking / Hopper Extended | Roller running / hopper out |

## Before each match

- Power the robot on with the **hopper fully retracted**. The hopper position zeroes at power-on.
- Pick the auto on the **Autonomous** tab. "nothing" doesn't tell the robot which way it's facing, so if driving feels rotated at the start of teleop, use Start (see above).
