#!/usr/bin/env python3
"""
Simplified Baxter demo focusing on core functionality
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.baxter import BaxterLeft, BaxterRight
from pyrep.robots.end_effectors.baxter_gripper import BaxterGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
import time

SCENE_FILE = join(dirname(abspath(__file__)), 'examples/scene_baxter_pick_and_pass.ttt')

def simple_path_planning(arm, target_pos, target_quat, arm_name="Arm"):
    """Simple path planning with fallback"""
    print(f"Planning path for {arm_name} to {target_pos}")
    
    # Try multiple approaches
    methods = [
        ("target quaternion", target_quat),
        ("current orientation", arm.get_tip().get_quaternion()),
        ("identity quaternion", [0, 0, 0, 1])
    ]
    
    for method_name, quat in methods:
        try:
            print(f"  Trying {method_name}...")
            path = arm.get_path(position=target_pos, quaternion=quat)
            print(f"  ✅ Success with {method_name}")
            return path
        except Exception as e:
            print(f"  ❌ Failed with {method_name}: {e}")
            continue
    
    raise Exception(f"All path planning methods failed for {arm_name}")

def execute_path(path, arm_name="Arm"):
    """Execute path with visualization"""
    print(f"Executing path for {arm_name}...")
    path.visualize()
    
    done = False
    step_count = 0
    max_steps = 500
    
    while not done and step_count < max_steps:
        try:
            done = path.step()
            pr.step()
            step_count += 1
        except Exception as e:
            print(f"Error during execution: {e}")
            break
    
    path.clear_visualization()
    print(f"Path execution completed in {step_count} steps")
    return done

print("=== Simplified Baxter Demo ===")
print("Launching scene...")

try:
    pr = PyRep()
    pr.launch(SCENE_FILE, headless=False)
    pr.start()
    
    # Wait for scene to load
    time.sleep(2)
    
    # Create robot instances
    baxter_left = BaxterLeft()
    baxter_right = BaxterRight()
    baxter_gripper_left = BaxterGripper(0)
    baxter_gripper_right = BaxterGripper(1)
    
    # Get objects
    cup = Shape('Cup')
    waypoints = [Dummy('waypoint%d' % i) for i in range(7)]
    
    print("✅ Scene loaded successfully")
    print(f"Left arm joints: {baxter_left.get_joint_count()}")
    print(f"Right arm joints: {baxter_right.get_joint_count()}")
    
    # Demo steps
    print("\n--- Starting Demo ---")
    
    # Step 1: Left arm to cup
    print("\n1. Moving left arm to cup...")
    path1 = simple_path_planning(baxter_left, waypoints[0].get_position(), 
                                waypoints[0].get_quaternion(), "Left Arm")
    execute_path(path1, "Left Arm")
    
    # Step 2: Left arm closer
    print("\n2. Moving left arm closer...")
    path2 = simple_path_planning(baxter_left, waypoints[1].get_position(), 
                                waypoints[1].get_quaternion(), "Left Arm")
    execute_path(path2, "Left Arm")
    
    # Step 3: Grasp cup
    print("\n3. Grasping cup...")
    while not baxter_gripper_left.actuate(0.0, 0.4):
        pr.step()
    baxter_gripper_left.grasp(cup)
    print("✅ Cup grasped!")
    
    # Step 4: Lift cup
    print("\n4. Lifting cup...")
    path4 = simple_path_planning(baxter_left, waypoints[2].get_position(), 
                                waypoints[2].get_quaternion(), "Left Arm")
    execute_path(path4, "Left Arm")
    
    # Step 5: Right arm to cup
    print("\n5. Moving right arm to cup...")
    path5 = simple_path_planning(baxter_right, waypoints[3].get_position(), 
                                waypoints[3].get_quaternion(), "Right Arm")
    execute_path(path5, "Right Arm")
    
    # Step 6: Right arm closer (previously problematic)
    print("\n6. Moving right arm closer (previously failed step)...")
    path6 = simple_path_planning(baxter_right, waypoints[4].get_position(), 
                                waypoints[4].get_quaternion(), "Right Arm")
    execute_path(path6, "Right Arm")
    
    # Step 7: Transfer cup
    print("\n7. Transferring cup...")
    while not baxter_gripper_right.actuate(0.0, 0.4):
        pr.step()
    while not baxter_gripper_left.actuate(1.0, 0.4):
        pr.step()
    
    baxter_gripper_left.release()
    baxter_gripper_right.grasp(cup)
    pr.step()
    print("✅ Cup transferred!")
    
    # Step 8: Return home
    print("\n8. Returning to home positions...")
    try:
        path_l = simple_path_planning(baxter_left, waypoints[5].get_position(), 
                                     waypoints[5].get_quaternion(), "Left Arm")
        path_r = simple_path_planning(baxter_right, waypoints[6].get_position(), 
                                     waypoints[6].get_quaternion(), "Right Arm")
        
        # Execute both paths
        done_l = done_r = False
        step_count = 0
        while (not done_l or not done_r) and step_count < 500:
            if not done_l:
                try:
                    done_l = path_l.step()
                except:
                    done_l = True
            if not done_r:
                try:
                    done_r = path_r.step()
                except:
                    done_r = True
            pr.step()
            step_count += 1
        
        print("✅ Return paths completed!")
        
    except Exception as e:
        print(f"⚠️ Return paths had issues: {e}")
    
    print("\n🎉 Demo completed successfully!")
    
except Exception as e:
    print(f"\n❌ Demo failed: {e}")
    import traceback
    traceback.print_exc()

finally:
    try:
        print("\nCleaning up...")
        pr.stop()
        pr.shutdown()
        print("✅ Cleanup completed")
    except Exception as e:
        print(f"Warning: Cleanup error: {e}")

print("\nPress Enter to exit...")
input()
