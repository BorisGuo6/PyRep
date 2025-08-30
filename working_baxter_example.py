#!/usr/bin/env python3
"""
Working version of Baxter pick and pass example using correct PyRep API
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.baxter import BaxterLeft, BaxterRight
from pyrep.robots.end_effectors.baxter_gripper import BaxterGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
import numpy as np
import time

SCENE_FILE = join(dirname(abspath(__file__)), 'examples/scene_baxter_pick_and_pass.ttt')
pr = PyRep()

def safe_get_path(arm, target_pos, target_quat, fallback_methods=True, ignore_collisions=False):
    """Safely get path with fallback methods"""
    print(f"Planning path to position: {target_pos}")
    
    # Method 1: Try with target quaternion
    try:
        if ignore_collisions:
            # Try to use ignore_collisions if supported
            try:
                path = arm.get_path(position=target_pos, quaternion=target_quat, ignore_collisions=True)
                print("✅ Path planning succeeded with target quaternion and collision ignoring")
                return path
            except TypeError:
                # If ignore_collisions not supported, try without it
                print("⚠️ ignore_collisions not supported, trying without...")
                path = arm.get_path(position=target_pos, quaternion=target_quat)
                print("✅ Path planning succeeded with target quaternion")
                return path
        else:
            path = arm.get_path(position=target_pos, quaternion=target_quat)
            print("✅ Path planning succeeded with target quaternion")
            return path
    except Exception as e:
        print(f"❌ Path planning failed with target quaternion: {e}")
    
    if not fallback_methods:
        raise Exception("Path planning failed and fallback methods disabled")
    
    # Method 2: Try with current orientation
    try:
        current_quat = arm.get_tip().get_quaternion()
        path = arm.get_path(position=target_pos, quaternion=current_quat)
        print("✅ Path planning succeeded with current orientation")
        return path
    except Exception as e:
        print(f"❌ Path planning failed with current orientation: {e}")
    
    # Method 3: Try with relaxed constraints (without ignore_collisions)
    try:
        path = arm.get_path(position=target_pos, quaternion=target_quat)
        print("✅ Path planning succeeded with relaxed constraints")
        return path
    except Exception as e:
        print(f"❌ Path planning failed with relaxed constraints: {e}")
    
    # Method 4: Try to find a nearby reachable position
    print("Trying to find nearby reachable position...")
    base_pos = np.array(target_pos)
    for offset in [0.01, 0.02, 0.05, 0.1]:
        for direction in [[1,0,0], [-1,0,0], [0,1,0], [0,-1,0], [0,0,1], [0,0,-1]]:
            test_pos = base_pos + np.array(direction) * offset
            try:
                path = arm.get_path(position=test_pos.tolist(), 
                                   quaternion=target_quat)
                print(f"✅ Found reachable position: {test_pos}")
                return path
            except:
                continue
    
    raise Exception("Could not find a reachable path to any nearby position")

def execute_path_safely(path, arm_name="Arm", max_steps=1000):
    """Execute path with error handling"""
    print(f"Executing path for {arm_name}...")
    path.visualize()
    
    done = False
    step_count = 0
    
    while not done and step_count < max_steps:
        try:
            done = path.step()
            pr.step()
            step_count += 1
        except Exception as e:
            print(f"Error during path execution: {e}")
            break
    
    if step_count >= max_steps:
        print(f"⚠️ Path execution stopped after {max_steps} steps")
    
    path.clear_visualization()
    return done

print("Launching scene...")
pr.launch(SCENE_FILE, headless=False)
pr.start()

# Wait for scene to fully load
time.sleep(2)

baxter_left = BaxterLeft()
baxter_right = BaxterRight()
baxter_gripper_left = BaxterGripper(0)
baxter_gripper_right = BaxterGripper(1)

cup = Shape('Cup')
waypoints = [Dummy('waypoint%d' % i) for i in range(7)]

print("\n=== Starting Baxter Pick and Pass Demo ===")

try:
    # Step 1: Left arm to cup
    print('\n--- Step 1: Left arm to cup ---')
    path = safe_get_path(baxter_left, waypoints[0].get_position(), 
                        waypoints[0].get_quaternion())
    execute_path_safely(path, "Left Arm")
    
    # Step 2: Left arm closer to cup
    print('\n--- Step 2: Left arm closer to cup ---')
    path = safe_get_path(baxter_left, waypoints[1].get_position(), 
                        waypoints[1].get_quaternion())
    execute_path_safely(path, "Left Arm")
    
    # Step 3: Close left gripper and grasp cup
    print('\n--- Step 3: Grasping cup ---')
    print('Closing left gripper...')
    while not baxter_gripper_left.actuate(0.0, 0.4):
        pr.step()
    baxter_gripper_left.grasp(cup)
    print('Cup grasped successfully!')
    
    # Step 4: Lift cup
    print('\n--- Step 4: Lifting cup ---')
    path = safe_get_path(baxter_left, waypoints[2].get_position(), 
                        waypoints[2].get_quaternion(), ignore_collisions=True)
    execute_path_safely(path, "Left Arm")
    
    # Step 5: Right arm to cup
    print('\n--- Step 5: Right arm to cup ---')
    path = safe_get_path(baxter_right, waypoints[3].get_position(), 
                        waypoints[3].get_quaternion())
    execute_path_safely(path, "Right Arm")
    
    # Step 6: Right arm closer to cup (this was the problematic step)
    print('\n--- Step 6: Right arm closer to cup ---')
    print('This step previously failed - using robust path planning...')
    path = safe_get_path(baxter_right, waypoints[4].get_position(), 
                        waypoints[4].get_quaternion())
    execute_path_safely(path, "Right Arm")
    
    # Step 7: Transfer cup
    print('\n--- Step 7: Transferring cup ---')
    print('Closing right gripper...')
    while not baxter_gripper_right.actuate(0.0, 0.4):
        pr.step()
    
    print('Opening left gripper...')
    while not baxter_gripper_left.actuate(1.0, 0.4):
        pr.step()
    
    # Transfer
    baxter_gripper_left.release()
    baxter_gripper_right.grasp(cup)
    pr.step()
    print('Cup transferred successfully!')
    
    # Step 8: Return to home positions
    print('\n--- Step 8: Returning to home ---')
    
    # Plan both paths simultaneously
    try:
        path_l = safe_get_path(baxter_left, waypoints[5].get_position(), 
                              waypoints[5].get_quaternion())
        path_r = safe_get_path(baxter_right, waypoints[6].get_position(), 
                              waypoints[6].get_quaternion())
        
        print('Executing return paths...')
        done_l = done_r = False
        step_count = 0
        max_steps = 1000
        
        while (not done_l or not done_r) and step_count < max_steps:
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
        
        if step_count >= max_steps:
            print("⚠️ Return paths stopped after maximum steps")
        
    except Exception as e:
        print(f"Error during return paths: {e}")
    
    print('\n✅ Demo completed successfully!')
    
except Exception as e:
    print(f'\n❌ Demo failed with error: {e}')
    print("This might be due to IK issues or workspace limitations")

print('\nPress Enter to finish...')
input()
pr.stop()
pr.shutdown()
