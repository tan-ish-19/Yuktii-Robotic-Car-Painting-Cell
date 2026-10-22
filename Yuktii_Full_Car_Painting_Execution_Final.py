# ============================================================
# YUKTII ROBOTIC CAR PAINTING CELL - FULL EXECUTION
# ABB IRB 5400-22 | RoboDK Python API
#
# Executes the original-side sequence through Target 212, then:
#   rotate -135 deg; Target 224..Target 311; Home
#   Top_Rear_Right..Target 327; Home
#   rotate +180 deg; Front_Side..Target 419; Home
#   rotate -45 deg; Top_Front_Left..Target 433; Home
#   rotate -45 deg; After-45_Top_Rear_Left..Target 446; Home
#   rotate +225 deg; Top_Front_Right..Target 457; final Home
#
# Added sequences are taken from RoboDK's target-item order so skipped
# numbers and duplicate target names shown in the station tree are preserved.
# Target 144 is explicitly excluded because it was identified as an erroneous waypoint.
# This script drives the live station; it does not create a saved RoboDK program.
# MoveL failures (TargetReachError) are retried with MoveJ for that waypoint.\n# Test in simulation at reduced speed before running on real hardware.
# ============================================================

from robodk.robolink import *
import time
import math

RDK = Robolink()

ROBOT_NAME = "ABB IRB 5400-22"
FRAME_NAME = "Painting Cell Frame"
TOOL_NAME = "Generic Paint Sprayer"
FIXTURE_NAME = "Car Rotary Fixture"
ROBOT_SPEED = 50
STEP_ANGLE = 2.0
STEP_DELAY = 0.10

robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
frame = RDK.Item(FRAME_NAME, ITEM_TYPE_FRAME)
tool = RDK.Item(TOOL_NAME)
fixture = RDK.Item(FIXTURE_NAME)

for item, name in [(robot, ROBOT_NAME), (frame, FRAME_NAME), (tool, TOOL_NAME), (fixture, FIXTURE_NAME)]:
    if not item.Valid():
        raise Exception("Station item not found: " + name)

all_targets = RDK.ItemList(ITEM_TYPE_TARGET)
if not all_targets:
    raise Exception("No targets found in the station.")

# Keep RoboDK's target-list order, including duplicate names.
targets_by_name = {}
for item in all_targets:
    if item.Valid():
        targets_by_name.setdefault(item.Name(), []).append(item)

used_occurrences = {}
def resolve_sequence(names, description):
    resolved = []
    for name in names:
        matches = targets_by_name.get(name, [])
        occurrence = used_occurrences.get(name, 0)
        if occurrence >= len(matches):
            raise Exception(f"{description}: missing occurrence {occurrence + 1} of '{name}'. No robot movement has started.")
        resolved.append(matches[occurrence])
        used_occurrences[name] = occurrence + 1
    return resolved

def segment_between(start_name, end_name, description):
    """Return station-order targets from the named start to the first matching end after it.

    RoboDK stations can contain duplicate target names. In that case, the endpoint
    is selected by its position after the segment's start, rather than requiring
    the name to be globally unique.
    """
    start_matches = [i for i, item in enumerate(all_targets)
                     if item.Valid() and item.Name() == start_name]
    if not start_matches:
        raise Exception("Missing segment start target: " + start_name + ". No robot movement has started.")

    # These segment labels are intended to identify the beginning of each group.
    # If a start label itself is duplicated, choose the first occurrence that has
    # a matching endpoint later in the target list.
    end_matches = [i for i, item in enumerate(all_targets)
                   if item.Valid() and item.Name() == end_name]
    if not end_matches:
        raise Exception("Missing segment endpoint target: " + end_name + ". No robot movement has started.")

    chosen = None
    for start_i in start_matches:
        later_ends = [i for i in end_matches if i >= start_i]
        if later_ends:
            chosen = (start_i, later_ends[0])
            break
    if chosen is None:
        raise Exception(description + f": no '{end_name}' occurs at or after '{start_name}'. No robot movement has started.")

    start_i, end_i = chosen
    # Target 144 was identified by the user as an erroneous waypoint.
    # Exclude it from every generated segment even if it still exists in the
    # station list; the script does not require or execute Target 144.
    seq = [
        item for item in all_targets[start_i:end_i + 1]
        if item.Valid() and item.Name() != "Target 144"
    ]
    if not seq or seq[0].Name() != start_name or seq[-1].Name() != end_name:
        raise Exception(description + ": could not construct target sequence. No robot movement has started.")
    if len(end_matches) > 1:
        print(f"NOTE: '{end_name}' occurs {len(end_matches)} times; selected the first occurrence after '{start_name}' in RoboDK target order.")
    return seq

def home_after(target):
    """Find the first Home target after the supplied target in station order."""
    target_i = next((i for i, item in enumerate(all_targets) if item == target), None)
    if target_i is None:
        raise Exception("Cannot locate target in station order: " + target.Name())
    for item in all_targets[target_i + 1:]:
        if item.Valid() and item.Name() == "Home":
            return item
    raise Exception("No Home target appears after " + target.Name() + ". No robot movement has started.")

def move_paint_sequence(sequence, label, first_movej=True, special_target_196=False):
    if not sequence:
        raise Exception("Empty sequence: " + label)
    print(f"\n--- {label}: {len(sequence)} targets ---")
    if first_movej:
        robot.MoveJ(sequence[0])
    else:
        robot.MoveL(sequence[0])
    for idx, target in enumerate(sequence[1:], start=2):
        prev = sequence[idx - 2]
        print(f"{label} {idx}/{len(sequence)}: {target.Name()}")
        if special_target_196 and target.Name() == "Target 196":
            print("Using MoveJ for Target 196")
            robot.MoveJ(target)
        else:
            # Prefer linear motion for painting. If RoboDK reports that the
            # straight-line path is unreachable (TargetReachError), retry this
            # waypoint with a joint move. This is useful for targets such as
            # Target 239 where the pose may be reachable but the Cartesian
            # path from the preceding waypoint is not.
            try:
                robot.MoveL(target)
            except TargetReachError as exc:
                print(
                    f"MoveL failed at {target.Name()}: {exc}. "
                    "Retrying this waypoint with MoveJ..."
                )
                robot.MoveJ(target)
                print(f"Reached {target.Name()} using MoveJ fallback.")

def rotate_fixture(delta_degrees, label):
    """Rotate by a relative signed angle, safely handling 0/360 wrap."""
    joints = fixture.Joints().list()
    if len(joints) != 1:
        raise Exception("Expected exactly one rotary joint; got " + str(joints))
    start = float(joints[0])
    # Fixture joint is treated as a 0..360 degree angle. The interpolation
    # follows the requested signed relative turn and wraps only commanded values.
    steps = max(1, int(math.ceil(abs(delta_degrees) / STEP_ANGLE)))
    print(f"\n--- {label}: {delta_degrees:+.1f} degrees from {start:.2f} ---")
    for i in range(1, steps + 1):
        unwrapped = start + delta_degrees * i / steps
        commanded = unwrapped % 360.0
        fixture.setJoints([commanded])
        RDK.Render(True)
        time.sleep(STEP_DELAY)
    final_angle = fixture.Joints().list()[0]
    print(f"Fixture angle after {label}: {final_angle:.2f} degrees")

# Build the original sequences. Target 144 is intentionally omitted because it was deleted from the station.
original_names = ["Paint_Side_P" + str(i).zfill(2) for i in range(1, 60)]
original_names += ["Target " + str(i) for i in range(64, 92)]
original_names += ["Target 93", "Target 93", "Target 94", "Target 96", "Target 96", "Target 97", "Target 99", "Target 99"]
original_names += ["Target " + str(i) for i in range(100, 107)]
outline_names = ["New_Side1"] + ["Target " + str(i) for i in range(108, 139)] + ["Target 174", "Target 175"] + ["Target " + str(i) for i in range(140, 144)] + ["Target " + str(i) for i in range(145, 173)]
boundary_names = ["Target " + str(i) for i in range(175, 206)] + ["Target 207", "Target 207"] + ["Target " + str(i) for i in range(208, 213)]

# Resolve the original sequences before movement begins.
original_targets = resolve_sequence(original_names, "Original-side painting")
outline_targets = resolve_sequence(outline_names, "New-side outline")
boundary_targets = resolve_sequence(boundary_names, "Additional boundary")

# Resolve all added target ranges from the visible station-tree order.
seq_224_311 = segment_between("Target 224", "Target 311", "Targets 224-311")
seq_top_rear_right = segment_between("Top_Rear_Right", "Target 327", "Top Rear Right")
seq_front_side = segment_between("Front_Side", "Target 419", "Front Side")
seq_top_front_left = segment_between("Top_Front_Left", "Target 433", "Top Front Left")
seq_after45_rear_left = segment_between("After-45_Top_Rear_Left", "Target 446", "After-45 Top Rear Left")
seq_top_front_right = segment_between("Top_Front_Right", "Target 457", "Top Front Right")

# Select Home targets based on the next Home after each segment's endpoint.
homes = targets_by_name.get("Home", [])
if len(homes) < 5:
    raise Exception(f"Expected at least 5 Home targets for the existing sequence; found {len(homes)}. No robot movement has started.")
initial_home = homes[0]
home_after_106 = homes[2]
home_after_172 = homes[3]
home_after_212 = home_after(boundary_targets[-1])
home_after_311 = home_after(seq_224_311[-1])
home_after_327 = home_after(seq_top_rear_right[-1])
home_after_419 = home_after(seq_front_side[-1])
home_after_433 = home_after(seq_top_front_left[-1])
home_after_446 = home_after(seq_after45_rear_left[-1])
home_after_457 = home_after(seq_top_front_right[-1])

# Final validation summary - all missing items fail before robot motion.
print("\nTARGET VALIDATION COMPLETE")
print("Original-side:", len(original_targets), "targets")
print("New-side outline:", len(outline_targets), "targets")
print("Additional boundary:", len(boundary_targets), "targets")
for label, seq in [("Target 224-311", seq_224_311), ("Top_Rear_Right-327", seq_top_rear_right), ("Front_Side-419", seq_front_side), ("Top_Front_Left-433", seq_top_front_left), ("After-45_Top_Rear_Left-446", seq_after45_rear_left), ("Top_Front_Right-457", seq_top_front_right)]:
    print(label + ":", len(seq), "targets |", seq[0].Name(), "->", seq[-1].Name())

robot.setPoseFrame(frame)
robot.setPoseTool(tool)
robot.setSpeed(ROBOT_SPEED)

# PHASE 1-3: Existing first-side sequence.
print("\nPHASE 1: INITIAL HOME")
robot.MoveJ(initial_home)
print("\nPHASE 2: ORIGINAL-SIDE PAINTING")
move_paint_sequence(original_targets, "Original-side painting")
print("Original-side painting completed through Target 106.")
print("\nPHASE 3: HOME AFTER TARGET 106")
robot.MoveJ(home_after_106)

# PHASE 4-8: Existing 180-degree rotation and outline/boundary sequence.
print("\nPHASE 4: ROTATE CAR BY +180 DEGREES")
rotate_fixture(+180.0, "Initial side change")
print("\nPHASE 5: NEW-SIDE OUTLINE")
move_paint_sequence(outline_targets, "New-side outline")
print("\nPHASE 6: HOME AFTER TARGET 172")
robot.MoveJ(home_after_172)
print("\nPHASE 7: ADDITIONAL BOUNDARY THROUGH TARGET 212")
move_paint_sequence(boundary_targets, "Additional boundary", special_target_196=True)
print("\nPHASE 8: HOME AFTER TARGET 212")
robot.MoveJ(home_after_212)

# PHASE 9: Ensure the fixture is at 180 degrees, then rotate -135 degrees.
# The intended transition is 180 -> 45 degrees. The previous run's log showed
# 0 degrees at this point, so explicitly verify/correct the starting orientation
# before applying the requested relative -135-degree turn.
print("\\nPHASE 9: ENSURE 180-DEGREE START, THEN ROTATE -135 DEGREES")
current_angle = float(fixture.Joints().list()[0])
print(f"Fixture angle before Phase 9: {current_angle:.2f} degrees")

# If the fixture did not retain the expected 180-degree orientation, move it
# smoothly to 180 degrees first. This prevents accidentally doing 0 -> -135.
angle_error = (180.0 - current_angle + 180.0) % 360.0 - 180.0
if abs(angle_error) > 0.5:
    rotate_fixture(angle_error, "Correct fixture to 180-degree starting orientation")

# Now execute the requested relative turn: 180 - 135 = 45 degrees.
rotate_fixture(-135.0, "Rotate from 180 to 45 degrees")
final_phase9_angle = float(fixture.Joints().list()[0])
print(f"Fixture angle for Targets 224-311: {final_phase9_angle:.2f} degrees")
if abs(((final_phase9_angle - 45.0 + 180.0) % 360.0) - 180.0) > 1.0:
    raise Exception(
        "Fixture did not reach the expected 45-degree orientation. "
        "Painting sequence stopped before Target 224."
    )

move_paint_sequence(seq_224_311, "Targets 224-311")
robot.MoveJ(home_after_311)

# PHASE 10: Top_Rear_Right through Target 327, same car orientation.
print("\nPHASE 10: TOP_REAR_RIGHT THROUGH TARGET 327")
move_paint_sequence(seq_top_rear_right, "Top_Rear_Right to Target 327")
robot.MoveJ(home_after_327)

# PHASE 11: rotate +180 degrees, then Front_Side through Target 419.
print("\nPHASE 11: ROTATE +180 DEGREES; FRONT_SIDE THROUGH TARGET 419")
rotate_fixture(+180.0, "Rotate +180 before Front_Side")
move_paint_sequence(seq_front_side, "Front_Side to Target 419")
robot.MoveJ(home_after_419)

# PHASE 12: rotate -45 degrees, then Top_Front_Left through Target 433.
print("\nPHASE 12: ROTATE -45 DEGREES; TOP_FRONT_LEFT THROUGH TARGET 433")
rotate_fixture(-45.0, "Rotate -45 before Top_Front_Left")
move_paint_sequence(seq_top_front_left, "Top_Front_Left to Target 433")
robot.MoveJ(home_after_433)

# PHASE 13: rotate -45 degrees, then After-45_Top_Rear_Left through Target 446.
print("\nPHASE 13: ROTATE -45 DEGREES; AFTER-45_TOP_REAR_LEFT THROUGH TARGET 446")
rotate_fixture(-45.0, "Rotate -45 before After-45_Top_Rear_Left")
move_paint_sequence(seq_after45_rear_left, "After-45_Top_Rear_Left to Target 446")
robot.MoveJ(home_after_446)

# PHASE 14: rotate +225 degrees, then Top_Front_Right through Target 457.
print("\nPHASE 14: ROTATE +225 DEGREES; TOP_FRONT_RIGHT THROUGH TARGET 457")
rotate_fixture(+225.0, "Rotate +225 before Top_Front_Right")
move_paint_sequence(seq_top_front_right, "Top_Front_Right to Target 457")
robot.MoveJ(home_after_457)

print("\n============================================")
print("CAR PAINTING EXECUTION COMPLETED")
print("Final fixture joint:", fixture.Joints().list())
print("Robot returned to Home after Target 457.")
print("============================================")
