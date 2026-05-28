#!/usr/bin/env python
import rospy
import numpy as np
import csv
import os
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, Float64MultiArray


class LineFollowerColorAware(object):
    def __init__(self):
        # Subscribers and publishers
        self.color_sub = rospy.Subscriber(
            "mean_img_rgb", Float64MultiArray, self.color_callback
        )
        self.line_sub = rospy.Subscriber(
            "line_idx", UInt32, self.line_callback
        )
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.belief_pub = rospy.Publisher("belief", Float64MultiArray, queue_size=1)

        # Line following (PID)
        self.desired = 320.0
        self.gain_p = 0.005
        self.gain_i = 0.0
        self.gain_d = 0.0005
        self.integral = 0.0
        self.lasterror = 0.0
        self.omega_limit = 0.5
        self.twist = Twist()

        # Colour model (RGB distance) and mode logic
        # index: 0=orange, 1=green, 2=blue, 3=yellow, 4=line
        self.colour_codes = np.array([
            [249, 145,  89],  # orange
            [170, 178, 169],  # green
            [201, 133, 178],  # blue
            [194, 174, 159],  # yellow
            [145, 129, 130],  # line
        ], dtype=float)

        self.colour_names = ["orange", "green", "blue", "yellow", "line"]

        # Current colour classification
        self.cur_colour_index = 4
        self.cur_colour_name = "line"

        # Motion mode: "line" (PID) or "patch" (go straight)
        self.mode = "line"
        self.consecutive_line = 0
        self.consecutive_patch = 0

        # Bayes-once-per-patch logic
        self.new_patch = False
        self.patch_sample_count = 0
        self.patch_colour_samples = []  # stored samples for majority vote

        # Delivery + Bayesian localization state
        self.in_delivery = False
        self.delivery_start_time = 0.0

        self.offices = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        self.N = len(self.offices)

        self.office_colors = [
            "yellow",  # 2
            "green",   # 3
            "blue",    # 4
            "orange",  # 5
            "orange",  # 6
            "green",   # 7
            "blue",    # 8
            "orange",  # 9
            "yellow",  # 10
            "green",   # 11
            "blue"     # 12
        ]

        # Initial belief
        self.belief = [1.0 / self.N] * self.N

        # Offices to deliver at
        self.goal_offices = [2, 4, 7]
        self.delivered_offices = set()

        # CSV logging for belief
        default_path = os.path.join(os.getcwd(), "belief_log.csv")
        self.log_file_path = rospy.get_param("~belief_log_path", default_path)
        rospy.loginfo("Belief log CSV: %s", self.log_file_path)

        self.log_file = open(self.log_file_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)

        self.belief_count = 0

        header = ["count", "obs_colour", "map_office", "map_p"] + [
            str(o) for o in self.offices
        ]
        self.csv_writer.writerow(header)
        self.log_file.flush()

        rospy.loginfo("Line follower (RGB color-aware + Bayes + delivery, once-per-patch) started")

    # Color callback: RGB-distance classification and mode logic
    def color_callback(self, msg):
        rgb = np.array(msg.data, dtype=float)
        if rgb.size < 3:
            return

        rgb = rgb[:3]  # [R, G, B], expected in 0-255

        # Euclidean distance classifier
        dists = np.linalg.norm(self.colour_codes - rgb, axis=1)
        self.cur_colour_index = int(np.argmin(dists))
        self.cur_colour_name = self.colour_names[self.cur_colour_index]

        # Debounce and mode switching
        if self.cur_colour_name == "line":
            self.consecutive_line += 1
            self.consecutive_patch = 0
        else:
            self.consecutive_patch += 1
            self.consecutive_line = 0

        # Switch from LINE to PATCH quickly
        if self.mode == "line" and self.consecutive_patch >= 1:
            self.mode = "patch"
            self.new_patch = True
            self.patch_sample_count = 0
            self.patch_colour_samples = []
            rospy.loginfo("MODE SWITCH: LINE -> PATCH (%s)", self.cur_colour_name)

        # While on a new patch, collect non-line readings for majority vote
        if self.mode == "patch" and self.new_patch and self.cur_colour_name != "line":
            self.patch_sample_count += 1
            # Ignore initial readings, record a later window for majority
            if 60 < self.patch_sample_count <= 80:
                self.patch_colour_samples.append(self.cur_colour_name)

        # Switch from PATCH to LINE conservatively
        if self.mode == "patch" and self.consecutive_line >= 3:
            self.mode = "line"
            self.new_patch = False
            self.patch_sample_count = 0
            self.patch_colour_samples = []
            rospy.loginfo("MODE SWITCH: PATCH -> LINE")

        rospy.loginfo(
            "RGB: [{:.0f}, {:.0f}, {:.0f}] classified as {} | mode={} | patch_count={}".format(
                rgb[0], rgb[1], rgb[2],
                self.cur_colour_name,
                self.mode,
                self.patch_sample_count if self.new_patch else -1
            )
        )

    # Line callback: motion, Bayes, and delivery behaviour
    def line_callback(self, msg):
        actual = float(msg.data)

        # Delivery behaviour overrides other logic
        if self.in_delivery:
            self.delivery_step()
            return

        if self.mode == "patch":
            # Patch mode: go straight
            self.patch_mode()

            patch_colors = ["green", "yellow", "orange", "blue"]

            # Once enough non-line readings are collected, decide patch colour and update Bayes
            if (self.new_patch and
                    self.patch_sample_count >= 30 and
                    len(self.patch_colour_samples) > 0):

                z = self.majority_colour(self.patch_colour_samples)
                rospy.loginfo(
                    "Patch samples: %s, chosen obs=%s",
                    self.patch_colour_samples, z
                )

                if z in patch_colors:
                    u = 1  # forward
                    map_office, map_p = self.bayes_update(u, z)

                    rospy.loginfo(
                        "Bayes (patch entry): obs=%s, MAP office %d with p=%.3f",
                        z, map_office, map_p
                    )

                    belief_str = ", ".join(
                        ["{}: {:.3f}".format(self.offices[i], self.belief[i])
                         for i in range(self.N)]
                    )
                    rospy.loginfo("Belief: " + belief_str)

                    # Write belief to CSV (rounded)
                    self.belief_count += 1
                    map_p_csv = round(map_p, 2)
                    belief_csv = [round(p, 2) for p in self.belief]
                    row = [self.belief_count, z, map_office, map_p_csv] + belief_csv
                    self.csv_writer.writerow(row)
                    self.log_file.flush()

                    # Start delivery if confident and at a goal office
                    if (map_office in self.goal_offices and
                            map_office not in self.delivered_offices and
                            map_p > 0.5):
                        self.start_delivery(map_office)

                self.new_patch = False

        else:
            # Line mode: PID line following
            self.follow_line_pid(actual)

        # Publish chosen twist (unless delivery_step already did)
        self.cmd_pub.publish(self.twist)

    # Majority colour helper
    def majority_colour(self, samples):
        if not samples:
            return None

        counts = {}
        for c in samples:
            counts[c] = counts.get(c, 0) + 1

        max_count = max(counts.values())
        candidates = [c for c, n in counts.items() if n == max_count]

        if len(candidates) == 1:
            return candidates[0]

        # Tie-breaker: last seen among candidates
        for c in reversed(samples):
            if c in candidates:
                return c

        return samples[-1]

    # Motion modes
    def follow_line_pid(self, actual):
        error = self.desired - float(actual)

        P = self.gain_p * error
        self.integral += error
        I = self.gain_i * self.integral
        D = self.gain_d * (error - self.lasterror)
        self.lasterror = error

        omega = P + I + D
        omega = max(min(omega, self.omega_limit), -self.omega_limit)

        self.twist.linear.x = 0.04
        self.twist.angular.z = omega

        rospy.loginfo(
            "LINE: idx={}, err={:.1f}, omega={:.3f}".format(
                actual, error, omega)
        )

    def patch_mode(self):
        self.twist.linear.x = 0.04
        self.twist.angular.z = 0.0
        rospy.loginfo("PATCH ({}) -> going straight".format(
            self.cur_colour_name))

    # Delivery behaviour
    def start_delivery(self, office):
        if office in self.delivered_offices:
            return
        rospy.loginfo("Starting delivery at office %d", office)
        self.in_delivery = True
        self.delivery_start_time = rospy.get_time()
        self.delivered_offices.add(office)

    def delivery_step(self):
        t = rospy.get_time() - self.delivery_start_time

        if t <= 2.0:
            # forward
            self.twist.linear.x = 0.04
            self.twist.angular.z = 0.0
        elif t <= 4.0:
            # rotate left
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        elif t <= 6.0:
            # pause
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        elif t <= 8.0:
            # rotate back
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5
        else:
            rospy.loginfo("Delivery finished, resuming normal behaviour")
            self.in_delivery = False
            return

        self.cmd_pub.publish(self.twist)

    # Bayesian localization
    def normalize(self, b):
        total = sum(b)
        if total == 0:
            return b
        return [x / total for x in b]

    def transition_prob(self, u, from_i, to_i):
        eps = math.exp(-6)

        if u == 1:
            w_minus = eps
            w_same = eps
            w_plus = 0.99
        elif u == -1:
            w_minus = 0.99
            w_same = eps
            w_plus = eps
        elif u in (0, None):
            w_minus = eps
            w_same = 0.99
            w_plus = eps
        else:
            w_minus = eps
            w_same = 0.99
            w_plus = eps

        total_w = w_minus + w_same + w_plus
        p_minus = w_minus / total_w
        p_same = w_same / total_w
        p_plus = w_plus / total_w

        minus_i = (from_i - 1) % self.N
        plus_i = (from_i + 1) % self.N

        if to_i == minus_i:
            return p_minus
        elif to_i == from_i:
            return p_same
        elif to_i == plus_i:
            return p_plus
        else:
            return 0.0

    def measurement_prob(self, observed_colour, state_i):
        if observed_colour is None:
            return 1.0

        observed_colour = observed_colour.lower()
        true_colour = self.office_colors[state_i]

        if true_colour == "blue":
            table = {
                "blue":   0.90,
                "green":  0.033,
                "yellow": 0.033,
                "orange": 0.033,
            }
        elif true_colour == "green":
            table = {
                "green":  0.70,
                "blue":   0.0,
                "yellow": 0.20,
                "orange": 0.10,
            }
        elif true_colour == "yellow":
            table = {
                "yellow": 0.70,
                "orange": 0.20,
                "green":  0.10,
                "blue":   0.0,
            }
        elif true_colour == "orange":
            table = {
                "orange": 0.80,
                "yellow": 0.10,
                "green":  0.10,
                "blue":   0.0,
            }
        else:
            return 0.0

        return table.get(observed_colour, 0.0)

    def bayes_update(self, u, z):
        predicted = [0.0] * self.N
        for to_i in range(self.N):
            total = 0.0
            for from_i in range(self.N):
                total += self.transition_prob(u, from_i, to_i) * self.belief[from_i]
            predicted[to_i] = total

        updated = [0.0] * self.N
        for i in range(self.N):
            updated[i] = predicted[i] * self.measurement_prob(z, i)

        self.belief = self.normalize(updated)

        msg = Float64MultiArray()
        msg.data = self.belief
        self.belief_pub.publish(msg)

        max_p = max(self.belief)
        max_i = self.belief.index(max_p)
        map_office = self.offices[max_i]

        return map_office, max_p

    def __del__(self):
        try:
            self.log_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("color_line_follower_rgb_bayes_delivery")
    node = LineFollowerColorAware()
    rospy.spin()
