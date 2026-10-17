# Enkidu Controls

Driver is on port 0, operator on port 1 (Driver Station USB tab). Both are Xbox controllers.

## Driver (port 0)

| Input | Action |
|---|---|
| Left stick | Drive (field-centric: up on the stick = away from you) |
| Right stick left/right | Turn (gentle near the middle of the stick, full speed at the edge) |
| Right trigger (hold) | Slow mode: half speed and half turning |
| Left trigger (hold) | Auto-aim: keep driving with the left stick while the robot turns to face the target. In your alliance zone it aims at your hub; outside it, it aims back into your alliance zone for passing. |
| Y (hold) | Face away from you |
| A (hold) | Face toward you |
| X (hold) | Face left |
| B (hold) | Face right |
| Right bumper (hold) | Lock the wheels in an X so you're hard to push (defense, or while shooting) |
| Left bumper (hold) | Drive backward over the bump (stops after 1.3 s or when you let go) |
| Start | Reset field-centric. Point the robot straight away from you first, then press. |
| D-pad up | Reset position: robot is parked at the hub reset spot |
| D-pad left | Reset position: robot is parked at the left trench reset spot |
| D-pad right | Reset position: robot is parked at the right trench reset spot |

While holding Y/A/X/B or the left trigger, the left stick still drives, but the right stick doesn't turn the robot.

Top speed is the full drivetrain speed (about 5.2 m/s), and turning goes up to 1.5 turns per second. The robot takes about two-thirds of a second to reach full speed. That's on purpose: it stops the wheels from slipping, which keeps the robot's position tracking accurate.

**If driving feels rotated or backwards** (for example, when no auto ran), point the robot away from you and press **Start**.

**If auto-aim is off**, the robot's position has drifted (there are no cameras to correct it). Park at a reset spot and press the matching D-pad button. The D-pad resets only position, not which way the robot is facing.

## Operator (port 1)

| Input | Action |
|---|---|
| D-pad up (hold) | **One-button shot:** spins up, waits for Shooter Ready (at most 1 s), then feeds and retracts the hopper to push fuel in. Let go to stop. |
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
| D-pad right / left (robot **disabled** only) | Pick the next / previous auto. Watch **Robot Will Run** on the dashboard. |

**To shoot:** hold D-pad up. Or hold the right trigger, wait for **Shooter Ready**, then hold X.

If the indexer jams while feeding, it backs up for a moment and tries again on its own.

While the hopper is moving (bumpers or a one-button shot), the operator sticks and A don't do anything.

## Rumble (teleop only)

| Rumble | Who feels it | Meaning |
|---|---|---|
| One long buzz | Driver and operator | Your hub just became active |
| Two short buzzes | Driver and operator | The hub switches in 3 seconds |
| Light steady rumble | Operator | Shooter is up to speed. Feed now. |

## Autos

Every auto is defense only. None of them shoot; the robot keeps its preload.

| Auto | Place the robot | What it does |
|---|---|---|
| Defend - Left Trench | In the left trench lane on our side, intake facing the other alliance | Drives through the left trench to the middle of the neutral zone, stops just short of the center line facing the opponents, and locks its wheels until teleop |
| Defend - Right Trench | In the right trench lane on our side, intake facing the other alliance | Same, through the right trench |
| Defend - Left Bump | Same spot as the old left-bump autos: our side of the left bump, intake facing **your driver station** | Backs over the bump, turns to face the opponents, drives to the same defense spot, and locks its wheels |
| Defend - Right Bump | Same, at the right bump | Same, over the right bump |
| Nothing - Hub (default) | At the hub reset spot, intake facing the other alliance | Only sets the robot's position |
| Nothing - Left Trench | At the left trench reset spot, intake facing the other alliance | Only sets the robot's position |
| Nothing - Right Trench | At the right trench reset spot, intake facing the other alliance | Only sets the robot's position |

Left and right are as seen from your driver station. The trench autos reach their spot in about 2.7 seconds; the bump autos take about 1.3 seconds longer because they cross the bump first.

**Picking an auto:** while the robot is disabled, the operator presses D-pad right/left to step through the list. You can also use the dashboard dropdown. Whichever was changed last is what runs, and **Robot Will Run** always shows what the robot will actually do. When you're not on a real field, the robot on the dashboard field jumps to the picked auto's starting spot, so you can check placement. On a real field it moves there when auto starts. The list order is: Nothing - Hub, Defend - Left Trench, Defend - Right Trench, Defend - Left Bump, Defend - Right Bump, Nothing - Left Trench, Nothing - Right Trench, nothing.

## Dashboard

| Widget | Meaning |
|---|---|
| Hub Active | Green when your hub is scoring this shift |
| Shift Time Left | Time until the hub switches; turns yellow at 10 s and red at 5 s |
| Shooter Ready | Green when the shooter is up to speed. Feed now. |
| Target / Distance | What auto-aim is pointing at and how far away it is |
| Intaking / Hopper Extended | Roller running / hopper out. Hopper Extended starts working after the first left-bumper extend each time the robot powers on; before that it stays red. |
| Robot Will Run (Autonomous tab) | The auto the robot actually received |
| Alerts (Autonomous and Pit tabs) | Problems: a motor not responding, low battery, plain "nothing" picked, a motor config that failed, or a flywheel slow to spin up |

## Before each match

- Power the robot on with the **hopper fully retracted**. The hopper position zeroes at power-on.
- Pick the auto with the **operator D-pad right/left** (robot disabled) until **Robot Will Run** shows the one you want. This works even when the dashboard dropdown doesn't reach the robot.
- Check the **Alerts** panel is empty (or only shows warnings you expect).
- **Nothing - Hub is the default.** If nobody picks an auto, put the robot at the hub spot with the intake facing the other alliance.
- Don't pick plain "nothing". It doesn't tell the robot which way it's facing, so driving can come out backwards. If that happens, point the robot away from you and press Start.

## Pit tab: tuning distance shots

1. Turn on **Shot Tuning Mode**.
2. Set **Shot Tuning Output** (0 to 1). Both shot buttons now use that output.
3. Shoot from a spot. Adjust the output until the shots go in.
4. Write down **Distance to Hub** and the output. Repeat at several distances.
5. Turn **Shot Tuning Mode** off.

Send the distance/output pairs to programming so they can go into the shot table.
