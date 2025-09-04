#!/usr/bin/env python3

import rospy
import turtlesim.srv
import turtlesim.msg
import geometry_msgs.msg
import std_srvs.srv
import math
import time
import random
import os

class Door:
    def __init__(self, name, x, y, turtle_name, true_state=None, push_success_prob=0.7):
        self.name = name
        self.x = x
        self.y = y
        self.turtle_name = turtle_name
        self.true_state = true_state if true_state else random.choice(["open", "closed"])
        self.push_success_prob = push_success_prob
        
    def spawn(self):
        try:
            spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
            spawner(self.x, self.y, 0, self.turtle_name)
            time.sleep(0.5)  # Wait for turtle to spawn
            
            # Set door color based on its state for better visualization
            set_pen = rospy.ServiceProxy(f'/{self.turtle_name}/set_pen', turtlesim.srv.SetPen)
            if self.true_state == "open":
                set_pen(0, 255, 0, 5, 0)  # Green for open doors
            else:
                set_pen(255, 0, 255, 5, 0)  # Magenta for closed doors
            
            rospy.loginfo(f"Spawned door {self.name} at ({self.x}, {self.y}) - True state: {self.true_state}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed for {self.name}: {e}")
    
    def simulate_push(self):
        """Simulate pushing the door with detailed logging"""
        rospy.loginfo(f"\n{'='*40}")
        rospy.loginfo(f"DOOR INTERACTION SIMULATION")
        rospy.loginfo(f"{'='*40}")
        rospy.loginfo(f"Door: {self.name}")
        rospy.loginfo(f"Initial true state: {self.true_state}")
        rospy.loginfo(f"Push success probability: {self.push_success_prob}")
        
        if self.true_state == "open":
            rospy.loginfo(f"Door is already open - remains open")
            return "open"
        
        # Door is closed, attempt to push
        push_result = random.random()
        rospy.loginfo(f"Push attempt: random value = {push_result:.3f}")
        
        if push_result < self.push_success_prob:
            self.true_state = "open"
            rospy.loginfo(f"SUCCESS: Door opened (pushed successfully)")
            
            # Update door color to show it's now open
            try:
                set_pen = rospy.ServiceProxy(f'/{self.turtle_name}/set_pen', turtlesim.srv.SetPen)
                set_pen(0, 255, 0, 5, 0)  # Green for open doors
            except rospy.ServiceException:
                pass
                
            return "open"
        else:
            rospy.loginfo(f"FAILED: Door remains closed (push unsuccessful)")
            return "closed"

class Robot:
    def __init__(self, robot_id, turtle_name, x, y, doors, threshold=0.5, comm_weight=0.3, comm_range=3.0):
        self.robot_id = robot_id
        self.turtle_name = turtle_name
        self.x = x
        self.y = y
        self.doors = doors
        self.beliefs = {door.name: {"open": 0.5, "closed": 0.5} for door in doors}
        self.threshold = threshold
        self.doors_passed = []
        self.comm_weight = comm_weight
        self.comm_range = comm_range  # Communication range
        self.log_entries = []
        
        # Wait for turtle to be spawned by main
        time.sleep(1)
        self.velocity_publisher = rospy.Publisher(f'/{turtle_name}/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(f'/{turtle_name}/pose', turtlesim.msg.Pose, self.update_pose)
        self.pose = None
        
        # Wait for the pose to be updated
        timeout = 10
        start_time = time.time()
        while self.pose is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
            
        if self.pose is None:
            rospy.logerr(f"Failed to get pose for {self.turtle_name}")
    
    def update_pose(self, data):
        """Callback function to update the turtle's position"""
        self.pose = data
        self.x = data.x
        self.y = data.y
    
    def set_color(self, r, g, b):
        """Set the robot's color"""
        try:
            set_pen = rospy.ServiceProxy(f'/{self.turtle_name}/set_pen', turtlesim.srv.SetPen)
            set_pen(r, g, b, 3, 0)
            rospy.loginfo(f"Set {self.turtle_name} color to RGB({r}, {g}, {b})")
        except rospy.ServiceException as e:
            rospy.logerr(f"SetPen service call failed for {self.turtle_name}: {e}")
    
    def set_pen_thickness(self, thickness):
        """Set the robot's pen thickness for better visualization"""
        try:
            set_pen = rospy.ServiceProxy(f'/{self.turtle_name}/set_pen', turtlesim.srv.SetPen)
            # Get current color settings, just change thickness
            if self.robot_id == 1:
                set_pen(255, 0, 0, thickness, 0)  # Red with variable thickness
            else:
                set_pen(0, 255, 0, thickness, 0)  # Green with variable thickness
        except rospy.ServiceException as e:
            rospy.logerr(f"SetPen thickness service call failed for {self.turtle_name}: {e}")
    
    def move_to_door(self, door):
        """Move the robot to a specified door"""
        rospy.loginfo(f"{self.turtle_name} moving to {door.name} at ({door.x}, {door.y})")
        self.log_entries.append(f"Moving to {door.name} at ({door.x:.1f}, {door.y:.1f})")
        
        if self.pose is None:
            rospy.logerr(f"No pose available for {self.turtle_name}")
            return
        
        # Calculate distance and angle to the door
        distance = math.sqrt((door.x - self.x) ** 2 + (door.y - self.y) ** 2)
        angle = math.atan2(door.y - self.y, door.x - self.x) - self.pose.theta
        
        # Rotate to face the door
        self.rotate(angle)
        
        # Move to the door
        self.move(distance)
    
    def rotate(self, angle):
        """Rotate the robot by a certain angle"""
        vel_msg = geometry_msgs.msg.Twist()
        angular_speed = 2.0
        
        # Normalize angle
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        
        vel_msg.linear.x = 0
        vel_msg.angular.z = angular_speed if angle > 0 else -angular_speed
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        while current_angle < abs(angle) and not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            
        # Stop rotation
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
    def move(self, distance):
        """Move the robot forward by a certain distance"""
        vel_msg = geometry_msgs.msg.Twist()
        speed = 2.0
        
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        
        while current_distance < distance and not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)
            
        # Stop moving
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
    
    def is_within_communication_range(self, other_robot):
        """Check if another robot is within communication range"""
        distance = math.sqrt((self.x - other_robot.x) ** 2 + (self.y - other_robot.y) ** 2)
        return distance <= self.comm_range
    
    def print_bayes_filter_step(self, step_name, values, description=""):
        """Print Bayes filter steps in formal mathematical notation"""
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"BAYES FILTER - {step_name}")
        rospy.loginfo(f"{'='*60}")
        if description:
            rospy.loginfo(f"Description: {description}")
        
        for key, value in values.items():
            if isinstance(value, dict):
                rospy.loginfo(f"{key}:")
                for sub_key, sub_value in value.items():
                    if isinstance(sub_value, float):
                        rospy.loginfo(f"  P({sub_key}) = {sub_value:.6f}")
                    else:
                        rospy.loginfo(f"  {sub_key} = {sub_value}")
            else:
                if isinstance(value, float):
                    rospy.loginfo(f"P({key}) = {value:.6f}")
                else:
                    rospy.loginfo(f"{key} = {value}")
        rospy.loginfo(f"{'='*60}\n")
    
    def update_belief_bayes(self, door, action, observation):
        """
        Update belief using Bayes Filter Algorithm
        
        Bayes Filter Steps:
        1. Prediction Step: bel̄(x_t) = ∫ P(x_t|u_t, x_{t-1}) bel(x_{t-1}) dx_{t-1}
        2. Correction Step: bel(x_t) = η P(z_t|x_t) bel̄(x_t)
        
        Where:
        - x_t: state at time t (door state: open/closed)
        - u_t: action at time t (push/do_nothing)
        - z_t: observation at time t (observed door state)
        - η: normalization constant
        """
        door_name = door.name
        
        # Prior beliefs: bel(x_{t-1})
        bel_open_prev = self.beliefs[door_name]["open"]
        bel_closed_prev = self.beliefs[door_name]["closed"]
        
        self.print_bayes_filter_step(
            "STEP 1: PRIOR BELIEFS",
            {
                "door": door_name,
                "robot": self.turtle_name,
                "prior_beliefs": {
                    "open": bel_open_prev,
                    "closed": bel_closed_prev
                }
            },
            "Prior probability distribution over door states"
        )
        
        self.log_entries.append(f"BAYES FILTER STEP 1 - Prior beliefs for {door_name}: P(open)={bel_open_prev:.6f}, P(closed)={bel_closed_prev:.6f}")
        
        # PREDICTION STEP: bel̄(x_t) = ∫ P(x_t|u_t, x_{t-1}) bel(x_{t-1}) dx_{t-1}
        # Action Model P(x_t|u_t, x_{t-1}): Probability of state transition given action
        if action == "push":
            # P(open_t | push, closed_{t-1}) = 0.8 (high chance of opening closed door)
            # P(open_t | push, open_{t-1}) = 1.0 (open door stays open)
            # P(closed_t | push, closed_{t-1}) = 0.2 (low chance door stays closed)
            # P(closed_t | push, open_{t-1}) = 0.0 (open door cannot become closed by pushing)
            bel_open_bar = 0.8 * bel_closed_prev + 1.0 * bel_open_prev
            bel_closed_bar = 0.2 * bel_closed_prev + 0.0 * bel_open_prev
            action_model = {
                "P(open_t|push,closed_{t-1})": 0.8,
                "P(open_t|push,open_{t-1})": 1.0,
                "P(closed_t|push,closed_{t-1})": 0.2,
                "P(closed_t|push,open_{t-1})": 0.0
            }
        else:  # do nothing
            # P(open_t | do_nothing, closed_{t-1}) = 0.0 (closed door stays closed)
            # P(open_t | do_nothing, open_{t-1}) = 1.0 (open door stays open)
            # P(closed_t | do_nothing, closed_{t-1}) = 1.0 (closed door stays closed)
            # P(closed_t | do_nothing, open_{t-1}) = 0.0 (open door stays open)
            bel_open_bar = 0.0 * bel_closed_prev + 1.0 * bel_open_prev
            bel_closed_bar = 1.0 * bel_closed_prev + 0.0 * bel_open_prev
            action_model = {
                "P(open_t|do_nothing,closed_{t-1})": 0.0,
                "P(open_t|do_nothing,open_{t-1})": 1.0,
                "P(closed_t|do_nothing,closed_{t-1})": 1.0,
                "P(closed_t|do_nothing,open_{t-1})": 0.0
            }
        
        # Normalization after prediction: η₁ = 1 / (bel̄(open) + bel̄(closed))
        eta_prediction = 1.0 / (bel_open_bar + bel_closed_bar)
        bel_open_bar *= eta_prediction
        bel_closed_bar *= eta_prediction
        
        self.print_bayes_filter_step(
            "STEP 2: PREDICTION",
            {
                "action": action,
                "action_model": action_model,
                "predicted_beliefs_unnormalized": {
                    "open": bel_open_bar / eta_prediction,
                    "closed": bel_closed_bar / eta_prediction
                },
                "normalization_constant_η₁": eta_prediction,
                "predicted_beliefs_normalized": {
                    "open": bel_open_bar,
                    "closed": bel_closed_bar
                }
            },
            "Prediction step using action model P(x_t|u_t,x_{t-1})"
        )
        
        self.log_entries.append(f"BAYES FILTER STEP 2 - After prediction for {door_name}: bel̄(open)={bel_open_bar:.6f}, bel̄(closed)={bel_closed_bar:.6f}")
        
        # CORRECTION STEP: bel(x_t) = η P(z_t|x_t) bel̄(x_t)
        # Observation Model P(z_t|x_t): Probability of observation given true state
        if observation == "open":
            # P(observe_open | true_open) = 0.6 (sensor accuracy for open doors)
            # P(observe_open | true_closed) = 0.2 (false positive rate)
            bel_open = 0.6 * bel_open_bar
            bel_closed = 0.2 * bel_closed_bar
            observation_model = {
                "P(observe_open|true_open)": 0.6,
                "P(observe_open|true_closed)": 0.2
            }
        else:  # observation = "closed"
            # P(observe_closed | true_open) = 0.4 (false negative rate)
            # P(observe_closed | true_closed) = 0.8 (sensor accuracy for closed doors)
            bel_open = 0.4 * bel_open_bar
            bel_closed = 0.8 * bel_closed_bar
            observation_model = {
                "P(observe_closed|true_open)": 0.4,
                "P(observe_closed|true_closed)": 0.8
            }
        
        # Final normalization: η₂ = 1 / (bel(open) + bel(closed))
        eta_correction = 1.0 / (bel_open + bel_closed)
        bel_open *= eta_correction
        bel_closed *= eta_correction
        
        self.print_bayes_filter_step(
            "STEP 3: CORRECTION",
            {
                "observation": observation,
                "observation_model": observation_model,
                "corrected_beliefs_unnormalized": {
                    "open": bel_open / eta_correction,
                    "closed": bel_closed / eta_correction
                },
                "normalization_constant_η₂": eta_correction,
                "final_posterior_beliefs": {
                    "open": bel_open,
                    "closed": bel_closed
                }
            },
            "Correction step using observation model P(z_t|x_t)"
        )
        
        # Update beliefs
        self.beliefs[door_name]["open"] = bel_open
        self.beliefs[door_name]["closed"] = bel_closed
        
        self.log_entries.append(f"BAYES FILTER STEP 3 - Final posterior beliefs for {door_name}: P(open|z_t,u_t)={bel_open:.6f}, P(closed|z_t,u_t)={bel_closed:.6f}")
        
        # Print final summary
        self.print_bayes_filter_step(
            "FINAL POSTERIOR",
            {
                "door": door_name,
                "robot": self.turtle_name,
                "posterior_distribution": {
                    "open": bel_open,
                    "closed": bel_closed
                },
                "entropy": -(bel_open * math.log2(bel_open + 1e-10) + bel_closed * math.log2(bel_closed + 1e-10)),
                "max_likelihood_state": "open" if bel_open > bel_closed else "closed",
                "confidence": max(bel_open, bel_closed)
            },
            "Final posterior probability distribution after Bayes filter update"
        )
    
    def should_communicate_belief_change(self, door, action, old_belief, new_belief):
        """
        Determine if robot should communicate based on belief change after action
        
        Communication Logic:
        1. When door is closed and robot pushes:
           - Communicate if robot believes door opened after push
           - Don't communicate if robot believes door stayed closed
        2. When door is open and robot pushes:
           - Communicate if robot believes door closed after push
           - Don't communicate if robot believes door stayed open
        """
        door_name = door.name
        old_state_belief = "open" if old_belief["open"] > old_belief["closed"] else "closed"
        new_state_belief = "open" if new_belief["open"] > new_belief["closed"] else "closed"
        
        # Only consider communication if action was "push"
        if action != "push":
            self.log_entries.append(f"No communication needed - action was '{action}', not 'push'")
            return False
        
        # Case 1: Door was believed closed, robot pushed
        if old_state_belief == "closed" and action == "push":
            if new_state_belief == "open":
                self.log_entries.append(f"COMMUNICATION TRIGGERED: Door {door_name} believed to open after push (closed → open)")
                return True
            else:
                self.log_entries.append(f"No communication: Door {door_name} still believed closed after push")
                return False
        
        # Case 2: Door was believed open, robot pushed
        elif old_state_belief == "open" and action == "push":
            if new_state_belief == "closed":
                self.log_entries.append(f"COMMUNICATION TRIGGERED: Door {door_name} believed to close after push (open → closed)")
                return True
            else:
                self.log_entries.append(f"No communication: Door {door_name} still believed open after push")
                return False
        
        return False
    
    def merge_beliefs(self, door, other_belief, other_robot):
        """
        Merge this robot's belief with another robot's belief using weighted fusion
        
        Belief Fusion Formula:
        P_fused(x) = w₁ * P₁(x) + w₂ * P₂(x)
        where w₁ + w₂ = 1 and w₁, w₂ are fusion weights
        """
        door_name = door.name
        
        before_open = self.beliefs[door_name]["open"]
        before_closed = self.beliefs[door_name]["closed"]
        
        self.print_bayes_filter_step(
            "BELIEF FUSION - BEFORE",
            {
                "door": door_name,
                "self_robot": self.turtle_name,
                "other_robot": other_robot.turtle_name,
                "self_beliefs": {
                    "open": before_open,
                    "closed": before_closed
                },
                "other_beliefs": {
                    "open": other_belief["open"],
                    "closed": other_belief["closed"]
                }
            },
            "Beliefs before fusion"
        )
        
        self.log_entries.append(f"BELIEF FUSION - Before merge {door_name}: P_self(open)={before_open:.6f}, P_self(closed)={before_closed:.6f}")
        self.log_entries.append(f"BELIEF FUSION - Other robot belief {door_name}: P_other(open)={other_belief['open']:.6f}, P_other(closed)={other_belief['closed']:.6f}")
        
        # Weighted average fusion: P_fused = w_self * P_self + w_other * P_other
        self_weight = 1 - self.comm_weight
        other_weight = self.comm_weight
        
        merged_open = self_weight * self.beliefs[door_name]["open"] + other_weight * other_belief["open"]
        merged_closed = self_weight * self.beliefs[door_name]["closed"] + other_weight * other_belief["closed"]
        
        # Normalization to ensure probability distribution sums to 1
        total = merged_open + merged_closed
        if total > 0:
            merged_open /= total
            merged_closed /= total
        
        # Update beliefs
        self.beliefs[door_name]["open"] = merged_open
        self.beliefs[door_name]["closed"] = merged_closed
        
        change_open = merged_open - before_open
        change_closed = merged_closed - before_closed
        
        self.print_bayes_filter_step(
            "BELIEF FUSION - AFTER",
            {
                "fusion_weights": {
                    f"w_self({self.turtle_name})": self_weight,
                    f"w_other({other_robot.turtle_name})": other_weight
                },
                "fused_beliefs_unnormalized": {
                    "open": merged_open * total,
                    "closed": merged_closed * total
                },
                "normalization_factor": total,
                "final_fused_beliefs": {
                    "open": merged_open,
                    "closed": merged_closed
                },
                "belief_change": {
                    "Δopen": change_open,
                    "Δclosed": change_closed
                }
            },
            "Beliefs after weighted fusion and normalization"
        )
        
        self.log_entries.append(f"BELIEF FUSION - After merge {door_name}: P_fused(open)={merged_open:.6f}, P_fused(closed)={merged_closed:.6f}")
        self.log_entries.append(f"BELIEF FUSION - Change in belief for {door_name}: Δopen={change_open:+.6f}, Δclosed={change_closed:+.6f}")
    
    def decide_action(self, door):
        """
        Decide whether to push or do nothing based on belief
        
        Decision Rule: If P(open) < threshold, then push; otherwise do nothing
        """
        door_open_prob = self.beliefs[door.name]["open"]
        action = "push" if door_open_prob < self.threshold else "do nothing"
        
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"ACTION DECISION")
        rospy.loginfo(f"{'='*50}")
        rospy.loginfo(f"Robot: {self.turtle_name}")
        rospy.loginfo(f"Door: {door.name}")
        rospy.loginfo(f"Current belief P(open) = {door_open_prob:.6f}")
        rospy.loginfo(f"Threshold = {self.threshold}")
        rospy.loginfo(f"Decision rule: Push if P(open) < {self.threshold}")
        rospy.loginfo(f"Action chosen: {action}")
        rospy.loginfo(f"{'='*50}\n")
        
        return action
    
    def can_pass_all_doors(self):
        """Check if all doors pass the threshold"""
        return all(self.beliefs[door.name]["open"] > self.threshold for door in self.doors)
    
    def save_log(self, filename):
        """Save the log entries to a file"""
        with open(filename, 'a') as f:
            f.write(f"\n--- {self.turtle_name} Log ---\n")
            for entry in self.log_entries:
                f.write(f"{self.turtle_name}: {entry}\n")
        self.log_entries = []
    
    def find_closest_door(self):
        """Find the closest unvisited door"""
        unvisited_doors = [door for door in self.doors if door.name not in self.doors_passed]
        if not unvisited_doors:
            return None
        
        min_distance = float('inf')
        closest_door = None
        
        for door in unvisited_doors:
            distance = math.sqrt((door.x - self.x) ** 2 + (door.y - self.y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_door = door
        
        return closest_door
    
    def move_to_door_smooth(self, door):
        """Move the robot to a specified door with smoother movement"""
        rospy.loginfo(f"{self.turtle_name} moving to {door.name} at ({door.x}, {door.y})")
        self.log_entries.append(f"Moving to {door.name} at ({door.x:.1f}, {door.y:.1f})")
        
        if self.pose is None:
            rospy.logerr(f"No pose available for {self.turtle_name}")
            return
        
        # Calculate distance to the door
        target_distance = 0.5  # Stop 0.5 units away from the door
        
        while True:
            # Update current distance and angle to the door
            dx = door.x - self.x
            dy = door.y - self.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            
            if distance <= target_distance:
                break
            
            # Calculate angle to the door
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.pose.theta
            
            # Normalize angle difference
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            vel_msg = geometry_msgs.msg.Twist()
            
            # Smooth angular movement
            if abs(angle_diff) > 0.1:
                vel_msg.angular.z = 1.5 * (1 if angle_diff > 0 else -1)
                vel_msg.linear.x = 0.5  # Move slowly while turning
            else:
                vel_msg.angular.z = 0
                vel_msg.linear.x = min(1.5, distance)  # Slow down as we approach
            
            self.velocity_publisher.publish(vel_msg)
            time.sleep(0.1)
        
        # Stop the robot
        vel_msg = geometry_msgs.msg.Twist()
        self.velocity_publisher.publish(vel_msg)
        
        rospy.loginfo(f"{self.turtle_name} reached {door.name}")

def main():
    """
    Multi-Robot Collaborative Door Exploration using Bayes Filter Algorithm
    
    ALGORITHM OVERVIEW:
    ==================
    
    1. INITIALIZATION:
       - Spawn robots and doors in turtlesim environment
       - Initialize uniform prior beliefs: P(door_state) = 0.5 for open/closed
    
    2. EXPLORATION LOOP (for each robot):
       a) SELECT TARGET: Choose closest unvisited door
       b) MOVE: Navigate to selected door
       c) DECIDE ACTION: Use decision rule based on belief threshold
       d) EXECUTE ACTION: Push door or observe only
       e) BAYES FILTER UPDATE:
          - Prediction step: bel̄(x_t) = ∫ P(x_t|u_t,x_{t-1}) bel(x_{t-1}) dx_{t-1}
          - Correction step: bel(x_t) = η P(z_t|x_t) bel̄(x_t)
       f) COMMUNICATION DECISION:
          - Communicate if belief change indicates state transition after push
          - Only communicate if other robot is within communication range
       g) BELIEF FUSION (if communication occurs):
          - P_fused(x) = w₁ * P₁(x) + w₂ * P₂(x)
    
    3. TERMINATION:
       - Stop when robot can pass all doors OR all doors visited
    
    PROBABILISTIC MODELS:
    ====================
    
    Action Model P(x_t|u_t,x_{t-1}):
    - P(open_t | push, closed_{t-1}) = 0.8
    - P(open_t | push, open_{t-1}) = 1.0
    - P(closed_t | push, closed_{t-1}) = 0.2
    - P(closed_t | push, open_{t-1}) = 0.0
    
    Observation Model P(z_t|x_t):
    - P(observe_open | true_open) = 0.6
    - P(observe_open | true_closed) = 0.2
    - P(observe_closed | true_open) = 0.4
    - P(observe_closed | true_closed) = 0.8
    
    Communication Model:
    - Range-based: Communicate only if distance ≤ comm_range
    - Event-triggered: Communicate only on significant belief changes
    """
    rospy.init_node('multi_robot_door_bayes', anonymous=True)
    rospy.loginfo("="*80)
    rospy.loginfo("MULTI-ROBOT COLLABORATIVE DOOR EXPLORATION")
    rospy.loginfo("Using Bayes Filter Algorithm with Smart Communication")
    rospy.loginfo("="*80)
    
    # Wait for turtlesim_node to start and spawn service to be available
    rospy.wait_for_service('spawn')
    rospy.loginfo("Spawn service is available")
    
    # Clear existing turtles
    try:
        clear = rospy.ServiceProxy('clear', std_srvs.srv.Empty)
        clear()
    except rospy.ServiceException as e:
        rospy.logwarn(f"Clear service call failed: {e}")
    
    # Create log file
    log_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'multi_robot_door_bayes_log.txt')
    with open(log_file, 'w') as f:
        f.write("Multi-Robot Door Bayes Filter Log\n")
        f.write("===============================\n\n")
    
    # Spawn robots first at opposite corners
    try:
        robot1_spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        robot1_spawn(1.0, 1.0, 0, "robot1")  # Bottom left corner
        time.sleep(0.5)
        
        robot2_spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        robot2_spawn(10.0, 10.0, 0, "robot2")  # Top right corner
        time.sleep(0.5)
        
        rospy.loginfo("Robots spawned successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Robot spawn failed: {e}")
        return
    
    # Create and spawn doors at different corners and positions
    doors = []
    door_positions = [(3.0, 3.0), (8.0, 2.0), (2.0, 8.0), (9.0, 7.0), (5.5, 5.5)]  # Spread across the map
    
    for i, (x, y) in enumerate(door_positions):
        name = f"door{i+1}"
        door = Door(name, x, y, name)
        door.spawn()
        doors.append(door)
        time.sleep(0.3)  # Small delay between spawns
    
    rospy.loginfo("All doors spawned")
    
    # Create robot objects with communication range
    robot1 = Robot(1, "robot1", 1.0, 1.0, doors, threshold=0.6, comm_weight=0.3, comm_range=4.0)
    robot2 = Robot(2, "robot2", 10.0, 10.0, doors, threshold=0.5, comm_weight=0.4, comm_range=4.0)
    
    # Set robot colors (different colors for each robot)
    robot1.set_color(255, 0, 0)  # Red
    robot2.set_color(0, 255, 0)  # Green
    
    # Set pen thickness
    robot1.set_pen_thickness(3)
    robot2.set_pen_thickness(3)
    
    # Set pen thickness
    robot1.set_pen_thickness(3)
    robot2.set_pen_thickness(3)
    
    rospy.loginfo("="*80)
    rospy.loginfo("STARTING COLLABORATIVE EXPLORATION PHASE")
    rospy.loginfo("="*80)
    rospy.loginfo(f"Robot 1 ({robot1.turtle_name}): Starting at ({robot1.x:.1f}, {robot1.y:.1f})")
    rospy.loginfo(f"Robot 2 ({robot2.turtle_name}): Starting at ({robot2.x:.1f}, {robot2.y:.1f})")
    rospy.loginfo(f"Communication range: {robot1.comm_range} units")
    rospy.loginfo(f"Total doors to explore: {len(doors)}")
    rospy.loginfo("="*80)
    
    # Track robots that are still active
    active_robots = [robot1, robot2]
    robots_beliefs_shared = {robot1.robot_id: False, robot2.robot_id: False}
    
    while len(active_robots) > 0 and not rospy.is_shutdown():
        robots_to_remove = []
        
        for robot in active_robots:
            # Check if robot can pass all doors
            if robot.can_pass_all_doors():
                rospy.loginfo(f"{robot.turtle_name} can pass all doors. Stopping this robot.")
                robots_to_remove.append(robot)
                continue
            
            # Find closest unvisited door
            closest_door = robot.find_closest_door()
            if closest_door is None:
                rospy.loginfo(f"{robot.turtle_name} has visited all doors but cannot pass all. Stopping.")
                robots_to_remove.append(robot)
                continue
            
            rospy.loginfo(f"\n--- {robot.turtle_name} processing {closest_door.name} ---")
            
            # Store beliefs before action for communication decision
            old_beliefs = {
                "open": robot.beliefs[closest_door.name]["open"],
                "closed": robot.beliefs[closest_door.name]["closed"]
            }
            
            # Move to the closest door using smooth movement
            robot.move_to_door_smooth(closest_door)
            
            # Robot decides action based on current belief
            action = robot.decide_action(closest_door)
            robot.log_entries.append(f"Decided to {action} {closest_door.name}")
            rospy.loginfo(f"{robot.turtle_name} decides to {action} {closest_door.name}")
            
            # Get observation based on action
            if action == "push":
                observation = closest_door.simulate_push()
                robot.log_entries.append(f"Pushed {closest_door.name} and observed it's {observation}")
            else:
                observation = closest_door.true_state
                robot.log_entries.append(f"Observed {closest_door.name} is {observation}")
            
            # Update robot's belief using Bayes filter
            robot.update_belief_bayes(closest_door, action, observation)
            
            # Check if robot should communicate based on belief change
            should_communicate = robot.should_communicate_belief_change(
                closest_door, action, old_beliefs, robot.beliefs[closest_door.name]
            )
            
            # Check if robot can pass through this door
            if robot.beliefs[closest_door.name]["open"] > robot.threshold:
                robot.log_entries.append(f"Can pass through {closest_door.name}")
                robot.doors_passed.append(closest_door.name)
            else:
                robot.log_entries.append(f"Cannot pass through {closest_door.name} (belief: {robot.beliefs[closest_door.name]['open']:.6f})")
            
            robot.save_log(log_file)
            
            # Share beliefs with other active robots only if communication is triggered and within range
            if should_communicate:
                for other_robot in active_robots:
                    if other_robot.robot_id != robot.robot_id:
                        # Check if robots are within communication range
                        if robot.is_within_communication_range(other_robot):
                            rospy.loginfo(f"COMMUNICATION: {robot.turtle_name} sharing belief with {other_robot.turtle_name} (distance: {math.sqrt((robot.x - other_robot.x)**2 + (robot.y - other_robot.y)**2):.2f} <= {robot.comm_range})")
                            other_robot.log_entries.append(f"COMMUNICATION: Receiving belief update from {robot.turtle_name} for {closest_door.name}")
                            other_robot.merge_beliefs(closest_door, robot.beliefs[closest_door.name], robot)
                            other_robot.save_log(log_file)
                        else:
                            rospy.loginfo(f"NO COMMUNICATION: {other_robot.turtle_name} out of range (distance: {math.sqrt((robot.x - other_robot.x)**2 + (robot.y - other_robot.y)**2):.2f} > {robot.comm_range})")
                            robot.log_entries.append(f"No communication with {other_robot.turtle_name} - out of range")
            else:
                rospy.loginfo(f"NO COMMUNICATION: Belief change does not warrant communication")
                robot.log_entries.append(f"No communication triggered - belief change doesn't warrant sharing")
            
            time.sleep(0.5)  # Short pause between robot actions
        
        # Remove robots that have finished
        for robot in robots_to_remove:
            active_robots.remove(robot)
        
        # If both robots are still active, add a small delay to prevent race conditions
        if len(active_robots) > 1:
            time.sleep(0.3)
    
    # Final summary
    with open(log_file, 'a') as f:
        f.write("\n" + "="*50 + "\n")
        f.write("FINAL SUMMARY\n")
        f.write("="*50 + "\n")
        f.write(f"{robot1.turtle_name} passed through doors: {robot1.doors_passed}\n")
        f.write(f"{robot2.turtle_name} passed through doors: {robot2.doors_passed}\n")
        f.write(f"{robot1.turtle_name} can pass all doors: {robot1.can_pass_all_doors()}\n")
        f.write(f"{robot2.turtle_name} can pass all doors: {robot2.can_pass_all_doors()}\n")
        f.write("\nFinal beliefs for all doors:\n")
        for door in doors:
            f.write(f"{door.name}: {robot1.turtle_name} P(open)={robot1.beliefs[door.name]['open']:.6f}, "
                   f"{robot2.turtle_name} P(open)={robot2.beliefs[door.name]['open']:.6f}\n")
    
    rospy.loginfo("\nSummary:")
    rospy.loginfo(f"{robot1.turtle_name} passed through doors: {robot1.doors_passed}")
    rospy.loginfo(f"{robot2.turtle_name} passed through doors: {robot2.doors_passed}")
    rospy.loginfo(f"Log saved to: {log_file}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")