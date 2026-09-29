"""
Test Suite for Delta Robot Kinematics
"""
import math
from delta_kinematics import DeltaKinematics

def test_kinematics():
    print("Initializing Delta Kinematics (L=150mm, l=400mm)...")
    robot = DeltaKinematics(bicep_length=150.0, forearm_length=400.0, base_radius=110.0, platform_radius=45.0)
    
    # Test 1: Center position
    target = [0.0, 0.0, -350.0]
    thetas, status = robot.inverse_kinematics(*target)
    print(f"Test 1 - Center {target}: IK={thetas}, Status={status}")
    assert thetas is not None, "Center IK failed"
    assert abs(thetas[0] - thetas[1]) < 0.01 and abs(thetas[1] - thetas[2]) < 0.01, "Symmetry check failed"
    
    fk_pos, status = robot.forward_kinematics(*thetas)
    print(f"Test 1 - FK roundtrip: Pos={fk_pos}, Status={status}")
    assert fk_pos is not None, "Center FK failed"
    for a, b in zip(target, fk_pos):
        assert abs(a - b) < 0.01, f"FK/IK discrepancy: {a} vs {b}"
        
    # Test 2: Off-center positions across quadrants
    test_points = [
        [50.0, 50.0, -380.0],
        [-60.0, 30.0, -320.0],
        [40.0, -70.0, -410.0],
        [0.0, 80.0, -360.0],
        [100.0, 0.0, -420.0],
    ]
    for pt in test_points:
        th, st = robot.inverse_kinematics(*pt)
        assert th is not None, f"Point {pt} IK failed: {st}"
        calc_pt, st2 = robot.forward_kinematics(*th)
        assert calc_pt is not None, f"Point {pt} FK failed: {st2}"
        err = math.sqrt(sum((a - b)**2 for a, b in zip(pt, calc_pt)))
        print(f"Test 2 - Pt {pt} -> Thetas: {th} -> FK: {calc_pt} (Err: {err:.4e} mm)")
        assert err < 0.01, f"Error too high for {pt}: {err}"

    # Test 3: Joint positions for 3D rendering
    joints = robot.get_joint_positions(thetas, target)
    print("Test 3 - Joint positions structure:", list(joints.keys()))
    assert len(joints["base_joints"]) == 3
    assert len(joints["elbow_joints"]) == 3
    assert len(joints["wrist_joints"]) == 3
    
    # Test 4: Out of reach detection
    out_pt = [450.0, 0.0, -200.0]
    th_out, st_out = robot.inverse_kinematics(*out_pt)
    print(f"Test 4 - Out of bounds {out_pt}: IK={th_out}, Status={st_out}")
    assert th_out is None, "Should fail out-of-reach test"

    print("\nALL KINEMATICS TESTS PASSED SUCCESSFULLY!")

if __name__ == "__main__":
    test_kinematics()
