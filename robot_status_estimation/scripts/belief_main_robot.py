#!/usr/bin/env python3

import rospy
import random
import math
import os
import csv
from datetime import datetime

from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String

class BeliefRobot:
    def __init__(self):
        rospy.init_node('belief_main_robot')
        self.rate = rospy.Rate(1)

        # Agents
        self.agent_names = ['turtle2', 'turtle3', 'turtle4', 'turtle5', 'turtle6']
        self.agent_positions = {}
        self.agent_beliefs = {name: 0.5 for name in self.agent_names}
        self.agent_states = {name: random.choice([0,1]) for name in self.agent_names}  # 0 = dead, 1 = alive

        self.turtle1_pose = None
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Belief sharing publisher
        self.belief_pub = rospy.Publisher('/agent/belief', String, queue_size=10)

        # Logging setup
        log_filename = os.path.expanduser(
            f"~/belief_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self.log_file = open(log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["Time", "Agent", "TrueState", "Observed", "Prior", "UpdatedBelief"])

        # Spawn agents
        self.spawn_agents()

    def update_pose(self, msg):
        self.turtle1_pose = msg

    def spawn_agents(self):
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', Spawn)

        for name in self.agent_names:
            x = random.uniform(1, 9)
            y = random.uniform(1, 9)
            self.agent_positions[name] = (x, y)
            try:
                spawner(x, y, 0, name)
                rospy.loginfo(f"Spawned {name} at ({x:.2f}, {y:.2f})")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to spawn {name}: {e}")

    def move_to_target(self, x, y):
        while not rospy.is_shutdown():
            if self.turtle1_pose is None:
                continue

            dx = x - self.turtle1_pose.x
            dy = y - self.turtle1_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.5:
                break

            angle_to_goal = math.atan2(dy, dx)

            msg = Twist()
            msg.linear.x = min(1.5 * distance, 2.0)
            msg.angular.z = 4 * (angle_to_goal - self.turtle1_pose.theta)
            self.cmd_vel_pub.publish(msg)

            self.rate.sleep()

        self.cmd_vel_pub.publish(Twist())  # Stop

    def bayesian_update(self, prior, observation, p_obs_alive=0.8, p_obs_dead=0.2):
        if observation == 1:
            numerator = p_obs_alive * prior
            denominator = p_obs_alive * prior + p_obs_dead * (1 - prior)
        else:
            numerator = (1 - p_obs_alive) * prior
            denominator = (1 - p_obs_alive) * prior + (1 - p_obs_dead) * (1 - prior)

        return numerator / denominator if denominator > 0 else 0.5

    def observe_agent(self, agent):
        true_state = self.agent_states[agent]
        sensor_noise = 0.6  # 60% accuracy

        # Simulate noisy observation
        observed = true_state if random.random() < sensor_noise else 1 - true_state

        # Prior belief
        prior = self.agent_beliefs[agent]

        # Bayesian update
        updated_belief = self.bayesian_update(prior, observed)
        self.agent_beliefs[agent] = updated_belief

        # Publish belief to topic
        belief_msg = f"{agent},{updated_belief:.2f}"
        self.belief_pub.publish(belief_msg)

        # Log to CSV
        now = rospy.get_time()
        self.csv_writer.writerow([now, agent, true_state, observed, f"{prior:.2f}", f"{updated_belief:.2f}"])

        rospy.loginfo(f"[OBSERVED {agent}] True: {true_state}, Obs: {observed}, Prior: {prior:.2f} → Belief: {updated_belief:.2f}")

    def run(self):
        rospy.sleep(2)  # Wait for turtles to spawn

        for agent in self.agent_names:
            pos = self.agent_positions[agent]
            rospy.loginfo(f"Moving to {agent} at {pos}")
            self.move_to_target(pos[0], pos[1])
            self.observe_agent(agent)

        rospy.loginfo("=== Final Beliefs ===")
        for agent, belief in self.agent_beliefs.items():
            rospy.loginfo(f"{agent}: Belief Alive = {belief:.2f}")

        # Shutdown: close log file
        self.log_file.close()
        rospy.loginfo("Belief log saved to CSV.")

if __name__ == "__main__":
    try:
        bot = BeliefRobot()
        bot.run()
    except rospy.ROSInterruptException:
        pass

