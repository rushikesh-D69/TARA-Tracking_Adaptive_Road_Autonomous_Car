#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
# Enhanced with ADAS features
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Automatic Control Client with Advanced Driver Assistance Systems (ADAS)

Keyboard Controls:
    F1          : Toggle ADAS system ON/OFF
    F2          : Toggle Adaptive Cruise Control (ACC)
    F3          : Toggle Automatic Emergency Braking (AEB)
    ESC/Ctrl+Q  : Quit

ADAS Features:
    - Forward Collision Warning (FCW)
    - Automatic Emergency Braking (AEB)
    - Adaptive Cruise Control (ACC)
    - Lane Departure Warning (LDW)
    - Blind Spot Detection (BSD)
    - Traffic Sign Recognition (TSR)
"""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import time
import weakref
from enum import Enum

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
    from pygame.locals import K_F1
    from pygame.locals import K_F2
    from pygame.locals import K_F3
    from pygame.locals import K_TAB
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_0
    from pygame.locals import K_9
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent  # pylint: disable=import-error


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            logging.warning("Actor Generation is not valid. No actor will be spawned.")
            return []
    except Exception:
        logging.warning("Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- ADAS Alert Types ---------------------------------------------------------
# ==============================================================================

class ADASAlertLevel(Enum):
    NONE = 0
    INFO = 1
    WARNING = 2
    CRITICAL = 3


 #==============================================================================
# -- Fixed Adaptive Cruise Control (ACC) --------------------------------------
# ==============================================================================

class AdaptiveCruiseControl:
    """Adaptive Cruise Control System"""
    
    def __init__(self, vehicle, target_speed=30.0, time_gap=2.0):
        self.vehicle = vehicle
        self.target_speed = target_speed  # km/h
        self.time_gap = time_gap  # seconds
        self.enabled = False
        self.min_distance = 5.0  # meters
        self.max_detection_range = 50.0  # meters
        
    def get_lead_vehicle(self, world):
        """Detect vehicle ahead in same lane"""
        try:
            vehicle_list = world.get_actors().filter('vehicle.*')
            ego_transform = self.vehicle.get_transform()
            ego_location = ego_transform.location
            ego_forward = ego_transform.get_forward_vector()
            
            closest_vehicle = None
            min_distance = self.max_detection_range
            
            for vehicle in vehicle_list:
                if vehicle.id == self.vehicle.id:
                    continue
                    
                target_location = vehicle.get_transform().location
                
                # Calculate distance properly
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                dz = target_location.z - ego_location.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance > self.max_detection_range:
                    continue
                    
                # Check if vehicle is ahead
                target_vector_x = dx / distance
                target_vector_y = dy / distance
                dot_product = ego_forward.x * target_vector_x + ego_forward.y * target_vector_y
                
                if dot_product > 0.7:  # Within ~45 degrees
                    if distance < min_distance:
                        min_distance = distance
                        closest_vehicle = vehicle
                        
            return closest_vehicle, min_distance
        except Exception as e:
            logging.debug("ACC get_lead_vehicle error: %s", e)
            return None, self.max_detection_range
    
    def calculate_target_speed(self, lead_vehicle, distance):
        """Calculate safe speed based on lead vehicle"""
        try:
            if lead_vehicle is None:
                return self.target_speed
                
            lead_velocity = lead_vehicle.get_velocity()
            lead_speed = 3.6 * math.sqrt(lead_velocity.x**2 + lead_velocity.y**2 + lead_velocity.z**2)
            
            safe_distance = lead_speed / 3.6 * self.time_gap + self.min_distance
            
            if distance < safe_distance:
                return max(0, lead_speed - 10)
            elif distance < safe_distance * 1.5:
                return lead_speed
            else:
                return min(self.target_speed, lead_speed + 5)
        except Exception as e:
            logging.debug("ACC calculate_target_speed error: %s", e)
            return self.target_speed


# ==============================================================================
# -- Fixed Forward Collision Warning (FCW) ------------------------------------
# ==============================================================================

class ForwardCollisionWarning:
    """Forward Collision Warning System"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.warning_time = 2.0
        self.critical_time = 1.0
        
    def check_collision_risk(self, world):
        """Calculate time to collision with obstacles ahead"""
        try:
            vehicle_list = world.get_actors().filter('vehicle.*')
            ego_transform = self.vehicle.get_transform()
            ego_velocity = self.vehicle.get_velocity()
            ego_speed = math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            
            if ego_speed < 1.0:
                return ADASAlertLevel.NONE, None, float('inf')
            
            ego_location = ego_transform.location
            ego_forward = ego_transform.get_forward_vector()
            
            min_ttc = float('inf')
            target_vehicle = None
            
            for vehicle in vehicle_list:
                if vehicle.id == self.vehicle.id:
                    continue
                    
                target_location = vehicle.get_transform().location
                target_velocity = vehicle.get_velocity()
                
                # Calculate distance
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                dz = target_location.z - ego_location.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance > 50.0:
                    continue
                    
                # Check if target is ahead
                rel_x = dx / distance
                rel_y = dy / distance
                dot_product = ego_forward.x * rel_x + ego_forward.y * rel_y
                
                if dot_product < 0.7:
                    continue
                    
                # Calculate relative velocity
                rel_vel_x = ego_velocity.x - target_velocity.x
                rel_vel_y = ego_velocity.y - target_velocity.y
                rel_vel_z = ego_velocity.z - target_velocity.z
                
                # Calculate closing speed
                closing_speed = ego_forward.x * rel_vel_x + ego_forward.y * rel_vel_y + ego_forward.z * rel_vel_z
                
                if closing_speed > 0.5:
                    ttc = distance / closing_speed
                    if ttc < min_ttc:
                        min_ttc = ttc
                        target_vehicle = vehicle
            
            # Determine alert level
            if min_ttc < self.critical_time:
                return ADASAlertLevel.CRITICAL, target_vehicle, min_ttc
            elif min_ttc < self.warning_time:
                return ADASAlertLevel.WARNING, target_vehicle, min_ttc
            else:
                return ADASAlertLevel.NONE, target_vehicle, min_ttc
                
        except Exception as e:
            logging.debug("FCW check_collision_risk error: %s", e)
            return ADASAlertLevel.NONE, None, float('inf')


# ==============================================================================
# -- Fixed Lane Departure Warning (LDW) ---------------------------------------
# ==============================================================================

class LaneDepartureWarning:
    """Lane Departure Warning System"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.lateral_offset_threshold = 0.5
        self.last_waypoint = None
        
    def check_lane_departure(self, world_map):
        """Check if vehicle is departing from lane"""
        try:
            vehicle_location = self.vehicle.get_transform().location
            waypoint = world_map.get_waypoint(vehicle_location)
            
            if waypoint is None:
                return ADASAlertLevel.NONE, 0.0
            
            # Calculate lateral offset
            lane_center = waypoint.transform.location
            lane_direction = waypoint.transform.get_forward_vector()
            
            # Vector to vehicle
            dx = vehicle_location.x - lane_center.x
            dy = vehicle_location.y - lane_center.y
            
            # Cross product for lateral distance
            lateral_offset = abs(dx * lane_direction.y - dy * lane_direction.x)
            
            if lateral_offset > self.lateral_offset_threshold * 1.5:
                return ADASAlertLevel.CRITICAL, lateral_offset
            elif lateral_offset > self.lateral_offset_threshold:
                return ADASAlertLevel.WARNING, lateral_offset
            
            return ADASAlertLevel.NONE, lateral_offset
            
        except Exception as e:
            logging.debug("LDW check_lane_departure error: %s", e)
            return ADASAlertLevel.NONE, 0.0


# ==============================================================================
# -- Fixed Blind Spot Detection (BSD) -----------------------------------------
# ==============================================================================

class BlindSpotDetection:
    """Blind Spot Detection System"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.detection_range = 5.0
        self.lateral_range = 3.5
        
    def check_blind_spots(self, world):
        """Detect vehicles in blind spot zones"""
        try:
            vehicle_list = world.get_actors().filter('vehicle.*')
            ego_transform = self.vehicle.get_transform()
            ego_location = ego_transform.location
            ego_forward = ego_transform.get_forward_vector()
            ego_right = ego_transform.get_right_vector()
            
            left_blind_spot = False
            right_blind_spot = False
            left_vehicles = []
            right_vehicles = []
            
            for vehicle in vehicle_list:
                if vehicle.id == self.vehicle.id:
                    continue
                    
                target_location = vehicle.get_transform().location
                
                # Calculate relative position
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                dz = target_location.z - ego_location.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance > self.detection_range:
                    continue
                
                # Calculate lateral and longitudinal distances
                lateral_dist = ego_right.x * dx + ego_right.y * dy + ego_right.z * dz
                longitudinal_dist = ego_forward.x * dx + ego_forward.y * dy + ego_forward.z * dz
                
                # Check if in blind spot zone
                if -2.0 < longitudinal_dist < self.detection_range:
                    if 1.5 < abs(lateral_dist) < self.lateral_range:
                        if lateral_dist > 0:
                            right_blind_spot = True
                            right_vehicles.append((vehicle, distance))
                        else:
                            left_blind_spot = True
                            left_vehicles.append((vehicle, distance))
            
            return left_blind_spot, right_blind_spot, left_vehicles, right_vehicles
            
        except Exception as e:
            logging.debug("BSD check_blind_spots error: %s", e)
            return False, False, [], []


# ==============================================================================
# -- Fixed Automatic Emergency Braking (AEB) ----------------------------------
# ==============================================================================

class AutomaticEmergencyBraking:
    """Automatic Emergency Braking System - Always Active"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.activation_ttc = 1.5  # More conservative - brake earlier
        self.enabled = True  # Always enabled by default - cannot be disabled for safety
        self.min_distance = 5.0  # Minimum safe distance in meters
        self.detection_range = 50.0  # Detection range in meters
        
    def check_vehicle_ahead(self, world):
        """Directly check for vehicles ahead and calculate collision risk"""
        try:
            vehicle_list = world.get_actors().filter('vehicle.*')
            ego_transform = self.vehicle.get_transform()
            ego_location = ego_transform.location
            ego_velocity = self.vehicle.get_velocity()
            ego_speed = math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            ego_forward = ego_transform.get_forward_vector()
            
            if ego_speed < 0.5:  # Too slow, no need to brake
                return False, None, float('inf')
            
            closest_vehicle = None
            min_distance = self.detection_range
            min_ttc = float('inf')
            
            for vehicle in vehicle_list:
                if vehicle.id == self.vehicle.id:
                    continue
                
                if not vehicle.is_alive:
                    continue
                
                target_location = vehicle.get_transform().location
                target_velocity = vehicle.get_velocity()
                
                # Calculate distance
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                dz = target_location.z - ego_location.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance > self.detection_range:
                    continue
                
                # Check if target is ahead
                rel_x = dx / distance if distance > 0 else 0
                rel_y = dy / distance if distance > 0 else 0
                dot_product = ego_forward.x * rel_x + ego_forward.y * rel_y
                
                if dot_product < 0.5:  # Not ahead enough
                    continue
                
                # Calculate relative velocity
                rel_vel_x = ego_velocity.x - target_velocity.x
                rel_vel_y = ego_velocity.y - target_velocity.y
                rel_vel_z = ego_velocity.z - target_velocity.z
                
                # Calculate closing speed
                closing_speed = ego_forward.x * rel_vel_x + ego_forward.y * rel_vel_y + ego_forward.z * rel_vel_z
                
                # If closing speed is positive, we're approaching
                if closing_speed > 0.1:
                    ttc = distance / closing_speed if closing_speed > 0.1 else float('inf')
                    
                    # Check if too close or TTC too low
                    if distance < self.min_distance or (ttc < self.activation_ttc and ttc > 0):
                        if distance < min_distance:
                            min_distance = distance
                            min_ttc = ttc
                            closest_vehicle = vehicle
            
            # Emergency brake if vehicle is too close or TTC is critical
            if closest_vehicle and (min_distance < self.min_distance or min_ttc < self.activation_ttc):
                return True, closest_vehicle, min_ttc
            
            return False, closest_vehicle, min_ttc
            
        except Exception as e:
            logging.debug("AEB check_vehicle_ahead error: %s", e)
            return False, None, float('inf')
        
    def should_brake(self, fcw_system, world):
        """Determine if emergency braking is needed - Always checks directly"""
        try:
            # Always check directly for vehicles ahead, regardless of FCW
            should_brake, target, ttc = self.check_vehicle_ahead(world)
            
            if should_brake and self.enabled:
                return True, ttc
            
            # Also check FCW as backup
            if fcw_system:
                alert_level, fcw_target, fcw_ttc = fcw_system.check_collision_risk(world)
                if self.enabled and alert_level == ADASAlertLevel.CRITICAL and fcw_ttc < self.activation_ttc:
                    return True, fcw_ttc
            
            return False, ttc if should_brake else float('inf')
        except Exception as e:
            logging.debug("AEB should_brake error: %s", e)
            return False, float('inf')


# ==============================================================================
# -- Traffic Sign Recognition (TSR) -------------------------------------------
# ==============================================================================

class TrafficSignRecognition:
    """Traffic Sign Recognition System"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.detection_range = 30.0
        self.current_speed_limit = None
        
    def detect_traffic_signs(self, world):
        """Detect and recognize traffic signs"""
        try:
            vehicle_location = self.vehicle.get_transform().location
            world_map = world.get_map()
            waypoint = world_map.get_waypoint(vehicle_location)
            
            if waypoint:
                # Get landmarks near current waypoint
                nearby_landmarks = waypoint.get_landmarks_of_type(
                    self.detection_range, 
                    "1000001"
                )
                
                if nearby_landmarks:
                    closest = min(nearby_landmarks, key=lambda x: x.distance)
                    self.current_speed_limit = closest.value
                    return closest.value
        except Exception as e:
            pass
        
        return self.current_speed_limit


# ==============================================================================
# -- Advanced Lane Detection System -------------------------------------------
# ==============================================================================

class AdvancedLaneDetection:
    """Advanced Lane Detection with Visual Overlays"""
    
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.lane_markings = []
        self.road_boundaries = []
        self.detection_range = 50.0
        self.lane_width = 3.5  # Standard lane width in meters
        
    def detect_lane_markings(self):
        """Detect lane markings and road boundaries"""
        try:
            vehicle_location = self.vehicle.get_transform().location
            world_map = self.world.get_map()
            waypoint = world_map.get_waypoint(vehicle_location)
            
            if not waypoint:
                return [], []
            
            # Get lane markings from waypoints
            lane_markings = []
            road_boundaries = []
            
            # Sample waypoints along the road
            current_waypoint = waypoint
            for i in range(20):  # Sample 20 waypoints ahead
                if not current_waypoint:
                    break
                    
                # Get left and right lane markings
                left_waypoint = current_waypoint.get_left_lane()
                right_waypoint = current_waypoint.get_right_lane()
                
                if left_waypoint:
                    lane_markings.append({
                        'type': 'left_lane',
                        'location': left_waypoint.transform.location,
                        'rotation': left_waypoint.transform.rotation,
                        'lane_type': left_waypoint.lane_type
                    })
                
                if right_waypoint:
                    lane_markings.append({
                        'type': 'right_lane',
                        'location': right_waypoint.transform.location,
                        'rotation': right_waypoint.transform.rotation,
                        'lane_type': right_waypoint.lane_type
                    })
                
                # Get road boundaries
                road_boundaries.append({
                    'center': current_waypoint.transform.location,
                    'left_edge': current_waypoint.transform.location + 
                               current_waypoint.transform.get_right_vector() * -self.lane_width,
                    'right_edge': current_waypoint.transform.location + 
                                current_waypoint.transform.get_right_vector() * self.lane_width
                })
                
                # Move to next waypoint
                current_waypoint = current_waypoint.next(5.0)[0] if current_waypoint.next(5.0) else None
            
            return lane_markings, road_boundaries
            
        except Exception as e:
            logging.debug("Lane detection error: %s", e)
            return [], []
    
    def get_lane_center_line(self):
        """Get the center line of the current lane"""
        try:
            vehicle_location = self.vehicle.get_transform().location
            world_map = self.world.get_map()
            waypoint = world_map.get_waypoint(vehicle_location)
            
            if not waypoint:
                return []
            
            center_line = []
            current_waypoint = waypoint
            
            for i in range(30):  # 30 waypoints ahead
                if not current_waypoint:
                    break
                    
                center_line.append({
                    'location': current_waypoint.transform.location,
                    'rotation': current_waypoint.transform.rotation,
                    'lane_type': current_waypoint.lane_type
                })
                
                current_waypoint = current_waypoint.next(3.0)[0] if current_waypoint.next(3.0) else None
            
            return center_line
            
        except Exception as e:
            logging.debug("Center line detection error: %s", e)
            return []
    
    def calculate_lane_offset(self):
        """Calculate lateral offset from lane center"""
        try:
            vehicle_location = self.vehicle.get_transform().location
            world_map = self.world.get_map()
            waypoint = world_map.get_waypoint(vehicle_location)
            
            if not waypoint:
                return 0.0, 0.0
            
            # Calculate offset from lane center
            lane_center = waypoint.transform.location
            lane_direction = waypoint.transform.get_forward_vector()
            lane_right = waypoint.transform.get_right_vector()
            
            # Vector from lane center to vehicle
            dx = vehicle_location.x - lane_center.x
            dy = vehicle_location.y - lane_center.y
            
            # Calculate lateral offset
            lateral_offset = dx * lane_right.x + dy * lane_right.y
            
            # Calculate longitudinal position
            longitudinal_position = dx * lane_direction.x + dy * lane_direction.y
            
            return lateral_offset, longitudinal_position
            
        except Exception as e:
            logging.debug("Lane offset calculation error: %s", e)
            return 0.0, 0.0


# ==============================================================================
# -- Sensor Range Visualization System ----------------------------------------
# ==============================================================================

class SensorRangeVisualization:
    """Visualize sensor detection ranges and zones"""
    
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.sensor_ranges = {
            'fcw': 50.0,      # Forward Collision Warning
            'bsd': 8.0,       # Blind Spot Detection
            'acc': 30.0,      # Adaptive Cruise Control
            'ldw': 20.0,      # Lane Departure Warning
            'tsr': 40.0       # Traffic Sign Recognition
        }
        
    def get_detection_zones(self):
        """Get detection zones for all sensors"""
        try:
            vehicle_transform = self.vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            vehicle_forward = vehicle_transform.get_forward_vector()
            vehicle_right = vehicle_transform.get_right_vector()
            
            zones = {}
            
            # FCW Zone (Forward)
            zones['fcw'] = {
                'center': vehicle_location + vehicle_forward * (self.sensor_ranges['fcw'] / 2),
                'range': self.sensor_ranges['fcw'],
                'angle': 30,  # degrees
                'color': (255, 100, 100, 50)  # Red with transparency
            }
            
            # BSD Zones (Left and Right)
            zones['bsd_left'] = {
                'center': vehicle_location + vehicle_right * -2.0,
                'range': self.sensor_ranges['bsd'],
                'angle': 45,
                'color': (100, 255, 100, 50)  # Green with transparency
            }
            
            zones['bsd_right'] = {
                'center': vehicle_location + vehicle_right * 2.0,
                'range': self.sensor_ranges['bsd'],
                'angle': 45,
                'color': (100, 255, 100, 50)  # Green with transparency
            }
            
            # ACC Zone
            zones['acc'] = {
                'center': vehicle_location + vehicle_forward * (self.sensor_ranges['acc'] / 2),
                'range': self.sensor_ranges['acc'],
                'angle': 20,
                'color': (100, 100, 255, 50)  # Blue with transparency
            }
            
            return zones
            
        except Exception as e:
            logging.debug("Sensor zone calculation error: %s", e)
            return {}


# ==============================================================================
# -- Intelligent Overtaking System --------------------------------------------
# ==============================================================================

class IntelligentOvertaking:
    """Intelligent Overtaking System with gap analysis and safe maneuvers"""
    
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.enabled = False
        self.overtaking_in_progress = False
        self.target_vehicle = None
        self.overtaking_start_time = 0
        self.max_overtaking_time = 10.0  # seconds
        self.min_gap_distance = 15.0  # meters
        self.safe_overtaking_distance = 20.0  # meters
        
    def analyze_overtaking_opportunity(self):
        """Analyze if overtaking is safe and beneficial"""
        try:
            vehicle_list = self.world.get_actors().filter('vehicle.*')
            ego_transform = self.vehicle.get_transform()
            ego_location = ego_transform.location
            ego_velocity = self.vehicle.get_velocity()
            ego_speed = math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            
            if ego_speed < 5.0:  # Too slow to overtake
                return False, None, None
                
            ego_forward = ego_transform.get_forward_vector()
            ego_right = ego_transform.get_right_vector()
            
            # Find vehicles ahead in same lane
            vehicles_ahead = []
            for vehicle in vehicle_list:
                if vehicle.id == self.vehicle.id:
                    continue
                    
                target_location = vehicle.get_transform().location
                target_velocity = vehicle.get_velocity()
                target_speed = math.sqrt(target_velocity.x**2 + target_velocity.y**2 + target_velocity.z**2)
                
                # Calculate relative position
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > 50.0:  # Too far
                    continue
                    
                # Check if vehicle is ahead
                forward_dist = ego_forward.x * dx + ego_forward.y * dy
                lateral_dist = ego_right.x * dx + ego_right.y * dy
                
                if forward_dist > 0 and abs(lateral_dist) < 2.0:  # Same lane, ahead
                    # Check if slower than ego vehicle
                    if target_speed < ego_speed - 2.0:  # At least 2 m/s slower
                        vehicles_ahead.append({
                            'vehicle': vehicle,
                            'distance': distance,
                            'forward_distance': forward_dist,
                            'lateral_distance': lateral_dist,
                            'speed': target_speed,
                            'speed_difference': ego_speed - target_speed
                        })
            
            if not vehicles_ahead:
                return False, None, None
                
            # Sort by distance (closest first)
            vehicles_ahead.sort(key=lambda x: x['forward_distance'])
            closest_vehicle = vehicles_ahead[0]
            
            # Check if overtaking is beneficial
            if closest_vehicle['speed_difference'] < 3.0:  # Not much speed advantage
                return False, None, None
                
            # Check for safe overtaking gap
            left_gap, right_gap = self._analyze_lane_gaps(ego_location, ego_forward, ego_right)
            
            # Choose best lane for overtaking
            if left_gap > right_gap and left_gap > self.min_gap_distance:
                return True, closest_vehicle, 'left'
            elif right_gap > self.min_gap_distance:
                return True, closest_vehicle, 'right'
            else:
                return False, None, None
                
        except Exception as e:
            logging.debug("Overtaking analysis error: %s", e)
            return False, None, None
    
    def _analyze_lane_gaps(self, ego_location, ego_forward, ego_right):
        """Analyze gaps in left and right lanes"""
        try:
            vehicle_list = self.world.get_actors().filter('vehicle.*')
            
            left_gap = float('inf')
            right_gap = float('inf')
            
            for vehicle in vehicle_list:
                if vehicle.id == self.vehicle.id:
                    continue
                    
                target_location = vehicle.get_transform().location
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > 30.0:  # Only check nearby vehicles
                    continue
                    
                forward_dist = ego_forward.x * dx + ego_forward.y * dy
                lateral_dist = ego_right.x * dx + ego_right.y * dy
                
                # Check left lane
                if -4.0 < lateral_dist < -1.5:  # Left lane
                    if abs(forward_dist) < 20.0:  # Within overtaking range
                        left_gap = min(left_gap, abs(forward_dist))
                        
                # Check right lane
                elif 1.5 < lateral_dist < 4.0:  # Right lane
                    if abs(forward_dist) < 20.0:  # Within overtaking range
                        right_gap = min(right_gap, abs(forward_dist))
            
            return left_gap, right_gap
            
        except Exception as e:
            logging.debug("Lane gap analysis error: %s", e)
            return 0.0, 0.0
    
    def execute_overtaking_maneuver(self, target_vehicle, direction):
        """Execute the overtaking maneuver"""
        try:
            if not self.enabled:
                return None
                
            self.overtaking_in_progress = True
            self.target_vehicle = target_vehicle
            self.overtaking_start_time = self.world.get_snapshot().timestamp.elapsed_seconds
            
            # Calculate overtaking trajectory
            ego_transform = self.vehicle.get_transform()
            ego_location = ego_transform.location
            ego_forward = ego_transform.get_forward_vector()
            ego_right = ego_transform.get_right_vector()
            
            target_location = target_vehicle['vehicle'].get_transform().location
            
            # Calculate lateral offset for overtaking
            if direction == 'left':
                lateral_offset = -3.5  # Move to left lane
            else:
                lateral_offset = 3.5   # Move to right lane
                
            # Calculate target position
            target_x = ego_location.x + ego_forward.x * 30.0 + ego_right.x * lateral_offset
            target_y = ego_location.y + ego_forward.y * 30.0 + ego_right.y * lateral_offset
            
            # Create control command for overtaking
            control = carla.VehicleControl()
            control.throttle = 0.8  # Accelerate to overtake
            control.brake = 0.0
            control.steer = lateral_offset / 10.0  # Steer towards target lane
            control.hand_brake = False
            control.manual_gear_shift = False
            control.gear = 1
            
            return control
            
        except Exception as e:
            logging.debug("Overtaking execution error: %s", e)
            return None
    
    def monitor_overtaking_progress(self):
        """Monitor and complete overtaking maneuver"""
        try:
            if not self.overtaking_in_progress or not self.target_vehicle:
                return None
                
            current_time = self.world.get_snapshot().timestamp.elapsed_seconds
            
            # Check if overtaking is taking too long
            if current_time - self.overtaking_start_time > self.max_overtaking_time:
                self._abort_overtaking()
                return None
                
            ego_transform = self.vehicle.get_transform()
            ego_location = ego_transform.location
            ego_forward = ego_transform.get_forward_vector()
            ego_right = ego_transform.get_right_vector()
            
            target_location = self.target_vehicle['vehicle'].get_transform().location
            
            # Calculate relative position
            dx = target_location.x - ego_location.x
            dy = target_location.y - ego_location.y
            forward_dist = ego_forward.x * dx + ego_forward.y * dy
            lateral_dist = ego_right.x * dx + ego_right.y * dy
            
            # Check if overtaking is complete
            if forward_dist > self.safe_overtaking_distance:
                self._complete_overtaking()
                return None
                
            # Continue overtaking maneuver
            control = carla.VehicleControl()
            control.throttle = 0.7
            control.brake = 0.0
            
            # Adjust steering based on lateral position
            if abs(lateral_dist) > 2.0:  # Need to adjust lateral position
                control.steer = -lateral_dist / 5.0
            else:
                control.steer = 0.0
                
            control.hand_brake = False
            control.manual_gear_shift = False
            control.gear = 1
            
            return control
            
        except Exception as e:
            logging.debug("Overtaking monitoring error: %s", e)
            return None
    
    def _abort_overtaking(self):
        """Abort overtaking maneuver"""
        self.overtaking_in_progress = False
        self.target_vehicle = None
        logging.info("Overtaking aborted - taking too long")
    
    def _complete_overtaking(self):
        """Complete overtaking maneuver"""
        self.overtaking_in_progress = False
        self.target_vehicle = None
        logging.info("Overtaking completed successfully")
    
    def get_overtaking_status(self):
        """Get current overtaking status"""
        return {
            'enabled': self.enabled,
            'overtaking_in_progress': self.overtaking_in_progress,
            'target_vehicle': self.target_vehicle is not None,
            'time_elapsed': (self.world.get_snapshot().timestamp.elapsed_seconds - 
                           self.overtaking_start_time) if self.overtaking_in_progress else 0
        }


# ==============================================================================
# -- ADAS Manager (Integrates all systems) ------------------------------------
# ==============================================================================

class ADASManager:
    """Central ADAS System Manager"""
    
    def __init__(self, vehicle, world, hud):
        self.vehicle = vehicle
        self.world = world
        self.hud = hud
        
        # Initialize all ADAS systems with error handling
        try:
            self.acc = AdaptiveCruiseControl(vehicle)
            logging.debug("ACC initialized")
        except Exception as e:
            logging.warning("ACC init failed: %s", e)
            self.acc = None
            
        try:
            self.fcw = ForwardCollisionWarning(vehicle)
            logging.debug("FCW initialized")
        except Exception as e:
            logging.warning("FCW init failed: %s", e)
            self.fcw = None
            
        try:
            self.ldw = LaneDepartureWarning(vehicle)
            logging.debug("LDW initialized")
        except Exception as e:
            logging.warning("LDW init failed: %s", e)
            self.ldw = None
            
        try:
            self.bsd = BlindSpotDetection(vehicle)
            logging.debug("BSD initialized")
        except Exception as e:
            logging.warning("BSD init failed: %s", e)
            self.bsd = None
            
        try:
            self.aeb = AutomaticEmergencyBraking(vehicle)
            logging.debug("AEB initialized")
        except Exception as e:
            logging.warning("AEB init failed: %s", e)
            self.aeb = None
            
        try:
            self.tsr = TrafficSignRecognition(vehicle)
            logging.debug("TSR initialized")
        except Exception as e:
            logging.warning("TSR init failed: %s", e)
            self.tsr = None
            
        try:
            self.overtaking = IntelligentOvertaking(vehicle, world)
            logging.debug("Overtaking system initialized")
        except Exception as e:
            logging.warning("Overtaking init failed: %s", e)
            self.overtaking = None
        
        self.adas_enabled = True
        self.last_notification_time = 0
        self.notification_cooldown = 2.0  # seconds
        logging.info("ADAS Manager initialized successfully")
        
    def update(self, control):
        """Update all ADAS systems and modify control if needed"""
        try:
            world_map = self.world.get_map()
            current_time = self.world.get_snapshot().timestamp.elapsed_seconds
            
            # Automatic Emergency Braking - ALWAYS ACTIVE, Works Everywhere Regardless of ADAS Status
            # This must run first and independently to prevent collisions
            if self.aeb and self.aeb.enabled:
                should_brake, brake_ttc = self.aeb.should_brake(self.fcw, self.world)
                if should_brake:
                    control.brake = 1.0
                    control.throttle = 0.0
                    control.steer = 0.0  # Keep straight when braking
                    if current_time - self.last_notification_time > 0.5:  # More frequent notifications
                        self.hud.notification('EMERGENCY BRAKE ACTIVATED! TTC: %.2fs' % brake_ttc, color=(255, 0, 0))
                        self.last_notification_time = current_time
                    # Return immediately after emergency brake - safety first
                    return control
            
            # Only run other ADAS features if ADAS is enabled
            if not self.adas_enabled:
                return control
            
            # 1. Forward Collision Warning
            if self.fcw:
                fcw_level, target, ttc = self.fcw.check_collision_risk(self.world)
                if fcw_level == ADASAlertLevel.CRITICAL:
                    if current_time - self.last_notification_time > 1.0:
                        self.hud.notification('COLLISION WARNING! TTC: %.1fs' % ttc, color=(255, 0, 0))
                        self.last_notification_time = current_time
                elif fcw_level == ADASAlertLevel.WARNING:
                    if current_time - self.last_notification_time > self.notification_cooldown:
                        self.hud.notification('Forward Collision Warning', color=(255, 165, 0))
                        self.last_notification_time = current_time
            
            # 3. Lane Departure Warning
            if self.ldw:
                ldw_level, lateral_offset = self.ldw.check_lane_departure(world_map)
                if ldw_level == ADASAlertLevel.CRITICAL:
                    if current_time - self.last_notification_time > 1.0:
                        self.hud.notification('LANE DEPARTURE! Offset: %.2fm' % lateral_offset, color=(255, 0, 0))
                elif ldw_level == ADASAlertLevel.WARNING:
                    if current_time - self.last_notification_time > self.notification_cooldown:
                        self.hud.notification('Lane Departure Warning', color=(255, 165, 0))
            
            # 4. Blind Spot Detection (silent - only shows in HUD)
            # 5. Traffic Sign Recognition (silent - only shows in HUD)
            # 6. Adaptive Cruise Control (silent - only shows in HUD)
            
            # 7. Intelligent Overtaking System
            if self.overtaking and self.overtaking.enabled:
                if self.overtaking.overtaking_in_progress:
                    # Monitor ongoing overtaking
                    overtaking_control = self.overtaking.monitor_overtaking_progress()
                    if overtaking_control:
                        control = overtaking_control
                else:
                    # Check for overtaking opportunities
                    can_overtake, target_vehicle, direction = self.overtaking.analyze_overtaking_opportunity()
                    if can_overtake:
                        overtaking_control = self.overtaking.execute_overtaking_maneuver(target_vehicle, direction)
                        if overtaking_control:
                            control = overtaking_control
                            self.hud.notification(f'Overtaking {direction} lane', color=(0, 255, 255))
            
        except Exception as e:
            logging.warning("ADAS update error: %s", e)
        
        return control
    
    def get_adas_status(self):
        """Return status dictionary for HUD display"""
        try:
            lead_vehicle, distance = self.acc.get_lead_vehicle(self.world) if self.acc else (None, None)
            left_bs, right_bs, left_veh, right_veh = self.bsd.check_blind_spots(self.world) if self.bsd else (False, False, [], [])
            fcw_level, target, ttc = self.fcw.check_collision_risk(self.world) if self.fcw else (ADASAlertLevel.NONE, None, float('inf'))
            world_map = self.world.get_map()
            ldw_level, lateral_offset = self.ldw.check_lane_departure(world_map) if self.ldw else (ADASAlertLevel.NONE, 0.0)
            speed_limit = self.tsr.current_speed_limit if self.tsr else None
            
            overtaking_status = self.overtaking.get_overtaking_status() if self.overtaking else {}
            
            return {
                'adas_enabled': self.adas_enabled,
                'acc_enabled': self.acc.enabled if self.acc else False,
                'aeb_enabled': self.aeb.enabled if self.aeb else False,
                'lead_distance': distance if lead_vehicle else None,
                'blind_spot_left': left_bs,
                'blind_spot_right': right_bs,
                'collision_warning': fcw_level != ADASAlertLevel.NONE,
                'ttc': ttc if ttc != float('inf') else None,
                'lane_departure': ldw_level != ADASAlertLevel.NONE,
                'lateral_offset': lateral_offset,
                'speed_limit': speed_limit,
                'overtaking_enabled': overtaking_status.get('enabled', False),
                'overtaking_in_progress': overtaking_status.get('overtaking_in_progress', False),
                'overtaking_time': overtaking_status.get('time_elapsed', 0)
            }
        except Exception as e:
            logging.warning("ADAS status error: %s", e)
            return {
                'adas_enabled': self.adas_enabled,
                'acc_enabled': False,
                'aeb_enabled': False,
                'lead_distance': None,
                'blind_spot_left': False,
                'blind_spot_right': False,
                'collision_warning': False,
                'ttc': None,
                'lane_departure': False,
                'lateral_offset': 0.0,
                'speed_limit': None
            }
    
    def toggle_adas(self):
        """Toggle ADAS system on/off"""
        self.adas_enabled = not self.adas_enabled
        status = "ENABLED" if self.adas_enabled else "DISABLED"
        self.hud.notification('ADAS System: %s' % status)
    
    def toggle_acc(self):
        """Toggle Adaptive Cruise Control"""
        self.acc.enabled = not self.acc.enabled
        status = "ON" if self.acc.enabled else "OFF"
        self.hud.notification('ACC: %s' % status)
    
    def toggle_aeb(self):
        """Toggle Automatic Emergency Braking (Note: AEB is always active for safety)"""
        # AEB should always be enabled for safety, but allow toggle for testing
        self.aeb.enabled = not self.aeb.enabled
        status = "ON" if self.aeb.enabled else "OFF"
        if not self.aeb.enabled:
            self.hud.notification('AEB: %s (WARNING: Safety feature disabled!)' % status, color=(255, 0, 0))
        else:
            self.hud.notification('AEB: %s' % status)
    
    def toggle_overtaking(self):
        """Toggle Intelligent Overtaking System"""
        if self.overtaking:
            self.overtaking.enabled = not self.overtaking.enabled
            status = "ON" if self.overtaking.enabled else "OFF"
            self.hud.notification('Overtaking: %s' % status)


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, hud, args):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            logging.error('RuntimeError: %s', error)
            logging.error('  The server could not send the OpenDRIVE (.xodr) file:')
            logging.error('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self.adas_manager = None
        self.scenario_manager = None
        self.sensor_visualization_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self.restart(args)
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self, args):
        """Restart the world"""
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get a random blueprint.
        blueprint_list = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                logging.error('There are no spawn points available in your map/town.')
                logging.error('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        
        # Initialize ADAS Manager with error handling
        try:
            logging.info("Initializing ADAS systems...")
            self.adas_manager = ADASManager(self.player, self.world, self.hud)
            logging.info("ADAS systems ready")
        except Exception as e:
            logging.error("ADAS initialization failed: %s", e)
            logging.warning("Continuing without ADAS features...")
            self.adas_manager = None
        
        # Initialize Scenario Manager
        try:
            logging.info("Initializing scenario manager...")
            self.scenario_manager = ScenarioManager(self, self.hud)
            logging.info("Scenario manager ready")
        except Exception as e:
            logging.warning("Scenario manager initialization failed: %s", e)
            self.scenario_manager = None
        
        # Initialize Sensor Visualization Manager
        try:
            logging.info("Initializing sensor visualization manager...")
            self.sensor_visualization_manager = SensorVisualizationManager(self.player, self.world)
            logging.info("Sensor visualization manager ready")
        except Exception as e:
            logging.warning("Sensor visualization manager initialization failed: %s", e)
            self.sensor_visualization_manager = None
        
    def next_weather(self, reverse=False):
        """Get next weather setting"""
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        """Method for every tick"""
        self.hud.tick(self, clock)
        # Sensor visualizations are updated in render method

    def render(self, display):
        """Render world"""
        # Render multi-sensor view if active
        if self.sensor_visualization_manager and self.sensor_visualization_manager.active_mode == 'multi_sensor':
            self.sensor_visualization_manager.render_multi_sensor()
        else:
            # Normal rendering
            self.camera_manager.render(display)
            self.hud.render(display)

    def destroy_sensors(self):
        """Destroy sensors"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        """Destroys all actors"""
        # Destroy sensor visualization manager first
        if self.sensor_visualization_manager:
            try:
                self.sensor_visualization_manager.destroy()
            except Exception as e:
                logging.warning("Error destroying sensor visualization manager: %s", e)
        
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world):
        self.perspective_index = 0  # Track current perspective: 0=Normal, 1=LIDAR, 2=Depth Raw
        self.perspectives = [
            (0, 'Camera RGB'),  # Normal RGB view
            (6, 'Lidar (Ray-Cast)'),  # LIDAR view
            (1, 'Camera Depth (Raw)')  # Raw depth view
        ]
        self.visualization_index = 0  # Track current visualization preset
        self.visualization_presets = [
            {
                'name': 'All Visualizations',
                'radar': True, 'speed_graph': True, 'ttc': True, 'adas_dashboard': True,
                'navigation': True, 'weather': True, 'minimap': True, 'vehicle_health': True,
                'performance': True, 'lane_overlay': True, 'sensor_zones': True, 'technical_overlay': True
            },
            {
                'name': 'Sensor Overlays',
                'radar': False, 'speed_graph': False, 'ttc': False, 'adas_dashboard': False,
                'navigation': False, 'weather': False, 'minimap': False, 'vehicle_health': False,
                'performance': False, 'lane_overlay': True, 'sensor_zones': True, 'technical_overlay': True
            },
            {
                'name': 'HUD Displays',
                'radar': True, 'speed_graph': True, 'ttc': True, 'adas_dashboard': True,
                'navigation': False, 'weather': False, 'minimap': False, 'vehicle_health': False,
                'performance': False, 'lane_overlay': False, 'sensor_zones': False, 'technical_overlay': False
            },
            {
                'name': 'Navigation & Map',
                'radar': False, 'speed_graph': False, 'ttc': False, 'adas_dashboard': False,
                'navigation': True, 'weather': True, 'minimap': True, 'vehicle_health': False,
                'performance': False, 'lane_overlay': False, 'sensor_zones': False, 'technical_overlay': False
            },
            {
                'name': 'Minimal',
                'radar': False, 'speed_graph': False, 'ttc': True, 'adas_dashboard': True,
                'navigation': False, 'weather': False, 'minimap': False, 'vehicle_health': False,
                'performance': False, 'lane_overlay': False, 'sensor_zones': False, 'technical_overlay': False
            },
            {
                'name': 'All Off',
                'radar': False, 'speed_graph': False, 'ttc': False, 'adas_dashboard': False,
                'navigation': False, 'weather': False, 'minimap': False, 'vehicle_health': False,
                'performance': False, 'lane_overlay': False, 'sensor_zones': False, 'technical_overlay': False
            }
        ]
        self.sensor_viz_mode_index = 0  # Track current sensor visualization mode
        self.sensor_viz_modes = [None, 'multi_sensor']  # None = normal, 'multi_sensor' = grid view
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)
        world.hud.notification("F1: Toggle ADAS | F2: Toggle ACC | F3: Cycle Perspectives", seconds=4.0)
        world.hud.notification("F4: Cycle Visualizations | F5: Toggle HUD | F6: Scenarios", seconds=4.0)
        world.hud.notification("F7: Toggle Multi-Sensor Grid View", seconds=4.0)
        world.hud.notification("1: Sudden Braking | 2: Overtaking | 0: End Scenario | O: Toggle Overtaking", seconds=4.0)
        world.hud.notification("TAB: Change camera | ` (backtick): Next sensor | 1-7: Select sensor (7=LIDAR)", seconds=5.0)
        world.hud.notification("L: Lane Overlay | Z: Sensor Zones | X: Technical Overlay", seconds=4.0)

    def parse_events(self, world):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_F1:
                    world.adas_manager.toggle_adas()
                elif event.key == K_F2:
                    world.adas_manager.toggle_acc()
                elif event.key == K_F3:
                    # Cycle through different perspectives: Normal -> LIDAR -> Depth Raw -> Normal
                    self.perspective_index = (self.perspective_index + 1) % len(self.perspectives)
                    sensor_index, perspective_name = self.perspectives[self.perspective_index]
                    world.camera_manager.set_sensor(sensor_index)
                    world.hud.notification(f"Perspective: {perspective_name}")
                elif event.key == pygame.K_F4:
                    # Cycle through visualization presets
                    self.visualization_index = (self.visualization_index + 1) % len(self.visualization_presets)
                    preset = self.visualization_presets[self.visualization_index]
                    
                    # Apply visualization preset
                    world.hud._show_radar = preset['radar']
                    world.hud._show_speed_graph = preset['speed_graph']
                    world.hud._show_ttc_display = preset['ttc']
                    world.hud._show_adas_dashboard = preset['adas_dashboard']
                    world.hud._show_navigation = preset['navigation']
                    world.hud._show_weather = preset['weather']
                    world.hud._show_minimap = preset['minimap']
                    world.hud._show_vehicle_health = preset['vehicle_health']
                    world.hud._show_performance_metrics = preset['performance']
                    world.hud._show_lane_overlay = preset['lane_overlay']
                    world.hud._show_sensor_zones = preset['sensor_zones']
                    world.hud._show_technical_overlay = preset['technical_overlay']
                    
                    world.hud.notification(f"Visualization: {preset['name']}", color=(0, 255, 255))
                elif event.key == pygame.K_F5:
                    self._toggle_hud_displays(world)
                elif event.key == pygame.K_F6:
                    self._toggle_scenario_menu(world)
                elif event.key == pygame.K_F7:
                    # Toggle multi-sensor grid view: Normal <-> Multi-Sensor
                    if world.sensor_visualization_manager:
                        self.sensor_viz_mode_index = (self.sensor_viz_mode_index + 1) % len(self.sensor_viz_modes)
                        mode = self.sensor_viz_modes[self.sensor_viz_mode_index]
                        new_mode = world.sensor_visualization_manager.toggle_visualization(mode)
                        
                        if new_mode == 'multi_sensor':
                            world.hud.notification("Sensor Visualization: Multi-Sensor Grid", color=(0, 255, 255))
                        else:
                            world.hud.notification("Sensor Visualization: Normal View", color=(255, 255, 255))
                    else:
                        world.hud.notification("Sensor visualization manager not available", color=(255, 0, 0))
                elif event.key == pygame.K_1:
                    self._start_scenario(world, "sudden_braking")
                elif event.key == pygame.K_2:
                    self._start_scenario(world, "overtaking")
                elif event.key == pygame.K_0:
                    self._end_scenario(world)
                elif event.key == pygame.K_r:
                    world.hud._show_radar = not world.hud._show_radar
                    world.hud.notification(f"Radar: {'ON' if world.hud._show_radar else 'OFF'}")
                elif event.key == pygame.K_s:
                    world.hud._show_speed_graph = not world.hud._show_speed_graph
                    world.hud.notification(f"Speed Graph: {'ON' if world.hud._show_speed_graph else 'OFF'}")
                elif event.key == pygame.K_t:
                    world.hud._show_ttc_display = not world.hud._show_ttc_display
                    world.hud.notification(f"TTC Display: {'ON' if world.hud._show_ttc_display else 'OFF'}")
                elif event.key == pygame.K_d:
                    world.hud._show_adas_dashboard = not world.hud._show_adas_dashboard
                    world.hud.notification(f"ADAS Dashboard: {'ON' if world.hud._show_adas_dashboard else 'OFF'}")
                elif event.key == pygame.K_n:
                    world.hud._show_navigation = not world.hud._show_navigation
                    world.hud.notification(f"Navigation: {'ON' if world.hud._show_navigation else 'OFF'}")
                elif event.key == pygame.K_w:
                    world.hud._show_weather = not world.hud._show_weather
                    world.hud.notification(f"Weather: {'ON' if world.hud._show_weather else 'OFF'}")
                elif event.key == pygame.K_m:
                    world.hud._show_minimap = not world.hud._show_minimap
                    world.hud.notification(f"Minimap: {'ON' if world.hud._show_minimap else 'OFF'}")
                elif event.key == pygame.K_v:
                    world.hud._show_vehicle_health = not world.hud._show_vehicle_health
                    world.hud.notification(f"Vehicle Health: {'ON' if world.hud._show_vehicle_health else 'OFF'}")
                elif event.key == pygame.K_p:
                    world.hud._show_performance_metrics = not world.hud._show_performance_metrics
                    world.hud.notification(f"Performance: {'ON' if world.hud._show_performance_metrics else 'OFF'}")
                elif event.key == pygame.K_l:
                    world.hud._show_lane_overlay = not world.hud._show_lane_overlay
                    world.hud.notification(f"Lane Overlay: {'ON' if world.hud._show_lane_overlay else 'OFF'}")
                elif event.key == pygame.K_z:
                    world.hud._show_sensor_zones = not world.hud._show_sensor_zones
                    world.hud.notification(f"Sensor Zones: {'ON' if world.hud._show_sensor_zones else 'OFF'}")
                elif event.key == pygame.K_x:
                    world.hud._show_technical_overlay = not world.hud._show_technical_overlay
                    world.hud.notification(f"Technical Overlay: {'ON' if world.hud._show_technical_overlay else 'OFF'}")
                elif event.key == pygame.K_o:
                    world.adas_manager.toggle_overtaking()
                elif event.key == pygame.K_TAB:
                    world.camera_manager.toggle_camera()
                    world.hud.notification("Changed camera view")
                elif event.key == pygame.K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key >= pygame.K_1 and event.key <= pygame.K_7:
                    # Direct sensor selection: 1=RGB, 2=Depth Raw, 3=Depth Gray, 4=Depth Log, 5=SemSeg Raw, 6=SemSeg CityScapes, 7=LIDAR
                    sensor_index = event.key - pygame.K_1
                    world.camera_manager.set_sensor(sensor_index)
    
    def _toggle_hud_displays(self, world):
        """Toggle all HUD displays"""
        world.hud._show_radar = not world.hud._show_radar
        world.hud._show_speed_graph = not world.hud._show_speed_graph
        world.hud._show_ttc_display = not world.hud._show_ttc_display
        world.hud._show_adas_dashboard = not world.hud._show_adas_dashboard
        world.hud._show_navigation = not world.hud._show_navigation
        world.hud._show_weather = not world.hud._show_weather
        world.hud._show_traffic_density = not world.hud._show_traffic_density
        world.hud._show_minimap = not world.hud._show_minimap
        world.hud._show_vehicle_health = not world.hud._show_vehicle_health
        world.hud._show_performance_metrics = not world.hud._show_performance_metrics
        world.hud._show_lane_overlay = not world.hud._show_lane_overlay
        world.hud._show_sensor_zones = not world.hud._show_sensor_zones
        world.hud._show_technical_overlay = not world.hud._show_technical_overlay
        world.hud.notification("All HUD displays toggled")
    
    def _toggle_scenario_menu(self, world):
        """Show scenario menu"""
        if world.scenario_manager:
            world.hud.notification("Scenarios: 1-Sudden Braking 2-Overtaking 0-End", color=(255, 255, 0))
    
    def _start_scenario(self, world, scenario_name):
        """Start a specific scenario"""
        if world.scenario_manager:
            world.scenario_manager.start_scenario(scenario_name)
    
    def _end_scenario(self, world):
        """End current scenario"""
        if world.scenario_manager:
            world.scenario_manager.end_scenario()
    
    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
# ==============================================================================
# -- Complete Enhanced HUD (Replace your existing HUD class) ------------------
# ==============================================================================

class HUD(object):
    """Enhanced HUD with ADAS visualizations"""

    def __init__(self, width, height):
        """Constructor method"""
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._font_large = pygame.font.Font(mono, 20)
        self._font_small = pygame.font.Font(mono, 10)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.world = None  # Will be set during render
        
        # History for graphs
        self.speed_history = collections.deque(maxlen=100)
        
        # Enhanced HUD options
        self._show_radar = True
        self._show_speed_graph = True
        self._show_ttc_display = True
        self._show_adas_dashboard = True
        self._show_navigation = True
        self._show_weather = True
        self._show_traffic_density = True
        self._show_minimap = True
        self._show_route_planning = True
        self._show_vehicle_health = True
        self._show_performance_metrics = True
        
        # Advanced visualization systems
        self.lane_detection = None
        self.sensor_visualization = None
        self._show_lane_overlay = True
        self._show_sensor_zones = True
        self._show_technical_overlay = True
        
        # Navigation data
        self.current_route = []
        self.destination = None
        self.next_waypoint = None
        self.route_distance = 0.0
        
        # Weather data
        self.current_weather = None
        self.weather_intensity = 0.0
        
        # Traffic density tracking
        self.traffic_density = 0.0
        self.nearby_vehicles_count = 0
        
        # Performance metrics
        self.frame_times = collections.deque(maxlen=60)
        self.memory_usage = 0.0

    def on_world_tick(self, timestamp):
        """Gets informations from the world at every tick"""
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        """HUD method for every tick"""
        self.world = world  # Store world reference for render
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        transform = world.player.get_transform()
        vel = world.player.get_velocity()
        control = world.player.get_control()
        heading = 'N' if abs(transform.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(transform.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > transform.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > transform.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (transform.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % transform.location.z,
            '']
        if isinstance(control, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', control.throttle, 0.0, 1.0),
                ('Steer:', control.steer, -1.0, 1.0),
                ('Brake:', control.brake, 0.0, 1.0),
                ('Reverse:', control.reverse),
                ('Hand brake:', control.hand_brake),
                ('Manual:', control.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)]
        elif isinstance(control, carla.WalkerControl):
            self._info_text += [
                ('Speed:', control.speed, 0.0, 5.556),
                ('Jump:', control.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]

        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']

        def dist(l):
            return math.sqrt((l.x - transform.location.x)**2 + (l.y - transform.location.y)
                             ** 2 + (l.z - transform.location.z)**2)
        vehicles = [(dist(x.get_location()), x) for x in vehicles if x.id != world.player.id]

        for d, vehicle in sorted(vehicles):
            if d > 200.0:
                break
            vehicle_type = get_actor_display_name(vehicle, truncate=22)
            self._info_text.append('% 4dm %s' % (d, vehicle_type))
        
        # Add ADAS information
        if world.adas_manager:
            try:
                adas_status = world.adas_manager.get_adas_status()
                self._info_text += [
                    '',
                    '===== ADAS STATUS =====',
                    'ADAS: %s' % ('ON' if adas_status['adas_enabled'] else 'OFF'),
                    'ACC:  %s' % ('ON' if adas_status['acc_enabled'] else 'OFF'),
                    'AEB:  %s' % ('ON' if adas_status['aeb_enabled'] else 'OFF'),
                ]
                
                if adas_status['lead_distance'] and adas_status['lead_distance'] < 50:
                    self._info_text.append('Lead Vehicle: %.1fm' % adas_status['lead_distance'])
                
                if adas_status['speed_limit']:
                    self._info_text.append('Speed Limit: %d km/h' % adas_status['speed_limit'])
                
                if adas_status['collision_warning'] and adas_status['ttc']:
                    self._info_text.append('TTC: %.2fs' % adas_status['ttc'])
                
                if adas_status['lane_departure']:
                    self._info_text.append('Lane Offset: %.2fm' % adas_status['lateral_offset'])
                
                blind_spot_status = ''
                if adas_status['blind_spot_left']:
                    blind_spot_status += 'LEFT '
                if adas_status['blind_spot_right']:
                    blind_spot_status += 'RIGHT'
                if blind_spot_status:
                    self._info_text.append('Blind Spot: %s' % blind_spot_status)
                
                if adas_status.get('overtaking_enabled', False):
                    self._info_text.append('Overtaking: ON')
                if adas_status.get('overtaking_in_progress', False):
                    self._info_text.append('Overtaking: IN PROGRESS')
            except Exception as e:
                self._info_text.append('ADAS: Error')

    def draw_radar_display(self, display, world):
        """Draw top-down radar view of nearby vehicles"""
        try:
            if not world.adas_manager:
                return
            
            radar_size = 200
            radar_pos = (self.dim[0] - radar_size - 20, 20)
            radar_range = 50.0
            
            radar_surface = pygame.Surface((radar_size, radar_size))
            radar_surface.set_alpha(180)
            radar_surface.fill((20, 20, 20))
            
            center = (radar_size // 2, radar_size // 2)
            pygame.draw.circle(radar_surface, (40, 40, 40), center, radar_size // 2, 1)
            pygame.draw.circle(radar_surface, (40, 40, 40), center, radar_size // 3, 1)
            pygame.draw.circle(radar_surface, (40, 40, 40), center, radar_size // 6, 1)
            
            pygame.draw.line(radar_surface, (40, 40, 40), (center[0], 0), (center[0], radar_size), 1)
            pygame.draw.line(radar_surface, (40, 40, 40), (0, center[1]), (radar_size, center[1]), 1)
            
            pygame.draw.circle(radar_surface, (0, 255, 0), center, 5)
            
            ego_transform = world.player.get_transform()
            ego_location = ego_transform.location
            ego_forward = ego_transform.get_forward_vector()
            ego_right = ego_transform.get_right_vector()
            
            vehicle_list = world.world.get_actors().filter('vehicle.*')
            for vehicle in vehicle_list:
                if vehicle.id == world.player.id:
                    continue
                
                target_location = vehicle.get_transform().location
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > radar_range:
                    continue
                
                forward_dist = ego_forward.x * dx + ego_forward.y * dy
                right_dist = ego_right.x * dx + ego_right.y * dy
                
                scale = (radar_size / 2) / radar_range
                x = int(center[0] + right_dist * scale)
                y = int(center[1] - forward_dist * scale)
                
                if distance < 10:
                    color = (255, 0, 0)
                elif distance < 20:
                    color = (255, 165, 0)
                else:
                    color = (255, 255, 0)
                
                if 0 <= x < radar_size and 0 <= y < radar_size:
                    pygame.draw.circle(radar_surface, color, (x, y), 3)
                    dist_text = self._font_small.render(f"{int(distance)}m", True, color)
                    radar_surface.blit(dist_text, (x + 5, y - 5))
            
            title = self._font_mono.render("RADAR VIEW", True, (255, 255, 255))
            radar_surface.blit(title, (5, 5))
            
            display.blit(radar_surface, radar_pos)
            
        except Exception as e:
            pass

    def draw_speed_graph(self, display, world):
        """Draw speed history graph"""
        try:
            vel = world.player.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            self.speed_history.append(speed)
            
            if len(self.speed_history) < 2:
                return
            
            graph_width = 300
            graph_height = 80
            graph_pos = (20, self.dim[1] - graph_height - 60)
            
            graph_surface = pygame.Surface((graph_width, graph_height))
            graph_surface.set_alpha(180)
            graph_surface.fill((20, 20, 20))
            
            for i in range(0, graph_height, 20):
                pygame.draw.line(graph_surface, (40, 40, 40), (0, i), (graph_width, i), 1)
            
            max_speed = max(self.speed_history) if self.speed_history else 100
            max_speed = max(max_speed, 50)
            
            points = []
            for i, spd in enumerate(self.speed_history):
                x = int((i / len(self.speed_history)) * graph_width)
                y = int(graph_height - (spd / max_speed) * graph_height)
                points.append((x, max(0, min(graph_height, y))))
            
            if len(points) > 1:
                pygame.draw.lines(graph_surface, (0, 255, 0), False, points, 2)
            
            title = self._font_small.render("SPEED (km/h)", True, (255, 255, 255))
            graph_surface.blit(title, (5, 5))
            
            current = self._font_mono.render(f"{int(speed)}", True, (0, 255, 0))
            graph_surface.blit(current, (graph_width - 50, 5))
            
            display.blit(graph_surface, graph_pos)
            
        except Exception as e:
            pass

    def draw_ttc_display(self, display, world):
        """Draw Time-To-Collision display"""
        try:
            if not world.adas_manager or not world.adas_manager.fcw:
                return
            
            alert_level, target, ttc = world.adas_manager.fcw.check_collision_risk(world.world)
            
            ttc_size = 150
            ttc_pos = (self.dim[0] // 2 - ttc_size // 2, 50)
            
            if alert_level != ADASAlertLevel.NONE and ttc != float('inf'):
                ttc_surface = pygame.Surface((ttc_size, ttc_size))
                ttc_surface.set_alpha(200)
                
                if alert_level == ADASAlertLevel.CRITICAL:
                    bg_color = (255, 0, 0)
                    text_color = (255, 255, 255)
                elif alert_level == ADASAlertLevel.WARNING:
                    bg_color = (255, 165, 0)
                    text_color = (0, 0, 0)
                else:
                    bg_color = (255, 255, 0)
                    text_color = (0, 0, 0)
                
                ttc_surface.fill(bg_color)
                
                ttc_text = self._font_large.render(f"{ttc:.1f}s", True, text_color)
                text_rect = ttc_text.get_rect(center=(ttc_size // 2, ttc_size // 2 - 20))
                ttc_surface.blit(ttc_text, text_rect)
                
                label = self._font_mono.render("TTC", True, text_color)
                label_rect = label.get_rect(center=(ttc_size // 2, ttc_size // 2 + 20))
                ttc_surface.blit(label, label_rect)
                
                pygame.draw.rect(ttc_surface, (255, 255, 255), (0, 0, ttc_size, ttc_size), 3)
                
                display.blit(ttc_surface, ttc_pos)
                
        except Exception as e:
            pass

    def draw_adas_dashboard(self, display, world):
        """Draw ADAS systems status dashboard"""
        try:
            if not world.adas_manager:
                return
            
            adas_status = world.adas_manager.get_adas_status()
            
            dash_width = 220
            dash_height = 180
            dash_pos = (self.dim[0] - dash_width - 20, self.dim[1] - dash_height - 20)
            
            dash_surface = pygame.Surface((dash_width, dash_height))
            dash_surface.set_alpha(200)
            dash_surface.fill((20, 20, 30))
            
            title = self._font_mono.render("ADAS STATUS", True, (255, 255, 255))
            dash_surface.blit(title, (10, 10))
            
            y_offset = 35
            line_height = 20
            
            systems = [
                ("ADAS", adas_status['adas_enabled']),
                ("ACC", adas_status['acc_enabled']),
                ("AEB", adas_status['aeb_enabled']),
                ("FCW", adas_status['collision_warning']),
                ("LDW", adas_status['lane_departure']),
                ("BSD-L", adas_status['blind_spot_left']),
                ("BSD-R", adas_status['blind_spot_right']),
                ("OVT", adas_status.get('overtaking_enabled', False)),
            ]
            
            for name, status in systems:
                color = (0, 255, 0) if status else (100, 100, 100)
                pygame.draw.circle(dash_surface, color, (20, y_offset + 8), 6)
                text = self._font_small.render(name, True, (255, 255, 255))
                dash_surface.blit(text, (35, y_offset))
                y_offset += line_height
            
            y_offset += 10
            
            if adas_status['lead_distance'] and adas_status['lead_distance'] < 50:
                dist_text = self._font_small.render(
                    f"Lead: {adas_status['lead_distance']:.1f}m", 
                    True, (255, 255, 0)
                )
                dash_surface.blit(dist_text, (10, y_offset))
            
            pygame.draw.rect(dash_surface, (100, 100, 150), (0, 0, dash_width, dash_height), 2)
            
            display.blit(dash_surface, dash_pos)
            
        except Exception as e:
            pass

    def draw_navigation_display(self, display, world):
        """Draw navigation information"""
        try:
            if not self._show_navigation:
                return
                
            nav_width = 300
            nav_height = 120
            nav_pos = (20, 20)
            
            nav_surface = pygame.Surface((nav_width, nav_height))
            nav_surface.set_alpha(180)
            nav_surface.fill((20, 30, 20))
            
            title = self._font_mono.render("NAVIGATION", True, (0, 255, 0))
            nav_surface.blit(title, (10, 10))
            
            # Current location
            transform = world.player.get_transform()
            location_text = self._font_small.render(
                f"Location: ({transform.location.x:.1f}, {transform.location.y:.1f})", 
                True, (255, 255, 255)
            )
            nav_surface.blit(location_text, (10, 35))
            
            # Speed and heading
            vel = world.player.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            speed_text = self._font_small.render(f"Speed: {speed:.1f} km/h", True, (255, 255, 255))
            nav_surface.blit(speed_text, (10, 55))
            
            heading_text = self._font_small.render(
                f"Heading: {transform.rotation.yaw:.1f}°", 
                True, (255, 255, 255)
            )
            nav_surface.blit(heading_text, (10, 75))
            
            # Route info
            if self.destination:
                route_text = self._font_small.render(
                    f"Destination: {self.route_distance:.1f}m away", 
                    True, (255, 255, 0)
                )
                nav_surface.blit(route_text, (10, 95))
            
            pygame.draw.rect(nav_surface, (0, 150, 0), (0, 0, nav_width, nav_height), 2)
            display.blit(nav_surface, nav_pos)
            
        except Exception as e:
            pass

    def draw_weather_display(self, display, world):
        """Draw weather information"""
        try:
            if not self._show_weather:
                return
                
            weather_width = 200
            weather_height = 100
            weather_pos = (self.dim[0] - weather_width - 20, 20)
            
            weather_surface = pygame.Surface((weather_width, weather_height))
            weather_surface.set_alpha(180)
            weather_surface.fill((20, 20, 30))
            
            title = self._font_mono.render("WEATHER", True, (100, 200, 255))
            weather_surface.blit(title, (10, 10))
            
            # Get current weather
            weather = world.world.get_weather()
            
            # Cloudiness
            cloud_text = self._font_small.render(
                f"Clouds: {weather.cloudiness:.1f}%", 
                True, (255, 255, 255)
            )
            weather_surface.blit(cloud_text, (10, 35))
            
            # Precipitation
            precip_text = self._font_small.render(
                f"Rain: {weather.precipitation:.1f}%", 
                True, (255, 255, 255)
            )
            weather_surface.blit(precip_text, (10, 55))
            
            # Wind
            wind_text = self._font_small.render(
                f"Wind: {weather.wind_intensity:.1f}", 
                True, (255, 255, 255)
            )
            weather_surface.blit(wind_text, (10, 75))
            
            pygame.draw.rect(weather_surface, (100, 200, 255), (0, 0, weather_width, weather_height), 2)
            display.blit(weather_surface, weather_pos)
            
        except Exception as e:
            pass

    def draw_traffic_density_display(self, display, world):
        """Draw traffic density information"""
        try:
            if not self._show_traffic_density:
                return
                
            density_width = 250
            density_height = 80
            density_pos = (20, self.dim[1] - density_height - 20)
            
            density_surface = pygame.Surface((density_width, density_height))
            density_surface.set_alpha(180)
            density_surface.fill((30, 20, 20))
            
            title = self._font_mono.render("TRAFFIC DENSITY", True, (255, 100, 100))
            density_surface.blit(title, (10, 10))
            
            # Count nearby vehicles
            vehicle_list = world.world.get_actors().filter('vehicle.*')
            ego_location = world.player.get_transform().location
            nearby_count = 0
            
            for vehicle in vehicle_list:
                if vehicle.id == world.player.id:
                    continue
                distance = ego_location.distance(vehicle.get_transform().location)
                if distance < 100.0:  # 100m radius
                    nearby_count += 1
            
            self.nearby_vehicles_count = nearby_count
            self.traffic_density = min(nearby_count / 20.0, 1.0)  # Normalize to 0-1
            
            # Density bar
            bar_width = 200
            bar_height = 20
            bar_x = 10
            bar_y = 35
            
            pygame.draw.rect(density_surface, (50, 50, 50), (bar_x, bar_y, bar_width, bar_height))
            
            # Fill based on density
            fill_width = int(bar_width * self.traffic_density)
            if self.traffic_density < 0.3:
                bar_color = (0, 255, 0)  # Green - low density
            elif self.traffic_density < 0.7:
                bar_color = (255, 165, 0)  # Orange - medium density
            else:
                bar_color = (255, 0, 0)  # Red - high density
                
            pygame.draw.rect(density_surface, bar_color, (bar_x, bar_y, fill_width, bar_height))
            
            # Text
            density_text = self._font_small.render(
                f"Vehicles: {nearby_count} | Density: {self.traffic_density:.1%}", 
                True, (255, 255, 255)
            )
            density_surface.blit(density_text, (10, 60))
            
            pygame.draw.rect(density_surface, (255, 100, 100), (0, 0, density_width, density_height), 2)
            display.blit(density_surface, density_pos)
            
        except Exception as e:
            pass

    def draw_minimap(self, display, world):
        """Draw minimap with route and traffic"""
        try:
            if not self._show_minimap:
                return
                
            minimap_size = 200
            minimap_pos = (self.dim[0] - minimap_size - 20, self.dim[1] - minimap_size - 20)
            
            minimap_surface = pygame.Surface((minimap_size, minimap_size))
            minimap_surface.set_alpha(200)
            minimap_surface.fill((20, 20, 20))
            
            # Draw grid
            for i in range(0, minimap_size, 20):
                pygame.draw.line(minimap_surface, (40, 40, 40), (i, 0), (i, minimap_size), 1)
                pygame.draw.line(minimap_surface, (40, 40, 40), (0, i), (minimap_size, i), 1)
            
            # Center point (ego vehicle)
            center = (minimap_size // 2, minimap_size // 2)
            pygame.draw.circle(minimap_surface, (0, 255, 0), center, 5)
            
            # Draw nearby vehicles
            ego_transform = world.player.get_transform()
            ego_location = ego_transform.location
            
            vehicle_list = world.world.get_actors().filter('vehicle.*')
            for vehicle in vehicle_list:
                if vehicle.id == world.player.id:
                    continue
                    
                target_location = vehicle.get_transform().location
                dx = target_location.x - ego_location.x
                dy = target_location.y - ego_location.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < 50.0:  # 50m range
                    # Convert to minimap coordinates
                    scale = minimap_size / 100.0  # 100m = full minimap
                    x = int(center[0] + dx * scale / 50.0)
                    y = int(center[1] - dy * scale / 50.0)
                    
                    if 0 <= x < minimap_size and 0 <= y < minimap_size:
                        color = (255, 0, 0) if distance < 10 else (255, 255, 0)
                        pygame.draw.circle(minimap_surface, color, (x, y), 3)
            
            # Draw route if available
            if self.current_route:
                for i, waypoint in enumerate(self.current_route[:10]):  # Show next 10 waypoints
                    dx = waypoint.location.x - ego_location.x
                    dy = waypoint.location.y - ego_location.y
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    if distance < 50.0:
                        scale = minimap_size / 100.0
                        x = int(center[0] + dx * scale / 50.0)
                        y = int(center[1] - dy * scale / 50.0)
                        
                        if 0 <= x < minimap_size and 0 <= y < minimap_size:
                            color = (0, 255, 255) if i == 0 else (100, 100, 255)
                            pygame.draw.circle(minimap_surface, color, (x, y), 2)
            
            title = self._font_small.render("MINIMAP", True, (255, 255, 255))
            minimap_surface.blit(title, (5, 5))
            
            pygame.draw.rect(minimap_surface, (100, 100, 100), (0, 0, minimap_size, minimap_size), 2)
            display.blit(minimap_surface, minimap_pos)
            
        except Exception as e:
            pass

    def draw_vehicle_health_display(self, display, world):
        """Draw vehicle health and status"""
        try:
            if not self._show_vehicle_health:
                return
                
            health_width = 200
            health_height = 120
            health_pos = (self.dim[0] - health_width - 20, self.dim[1] - health_height - 250)
            
            health_surface = pygame.Surface((health_width, health_height))
            health_surface.set_alpha(180)
            health_surface.fill((20, 20, 20))
            
            title = self._font_mono.render("VEHICLE STATUS", True, (255, 255, 0))
            health_surface.blit(title, (10, 10))
            
            # Get vehicle control
            control = world.player.get_control()
            
            # Throttle
            throttle_text = self._font_small.render(
                f"Throttle: {control.throttle:.2f}", 
                True, (255, 255, 255)
            )
            health_surface.blit(throttle_text, (10, 35))
            
            # Brake
            brake_text = self._font_small.render(
                f"Brake: {control.brake:.2f}", 
                True, (255, 255, 255)
            )
            health_surface.blit(brake_text, (10, 55))
            
            # Steering
            steer_text = self._font_small.render(
                f"Steer: {control.steer:.2f}", 
                True, (255, 255, 255)
            )
            health_surface.blit(steer_text, (10, 75))
            
            # Gear
            gear_text = self._font_small.render(
                f"Gear: {control.gear}", 
                True, (255, 255, 255)
            )
            health_surface.blit(gear_text, (10, 95))
            
            pygame.draw.rect(health_surface, (255, 255, 0), (0, 0, health_width, health_height), 2)
            display.blit(health_surface, health_pos)
            
        except Exception as e:
            pass

    def draw_performance_metrics(self, display, world):
        """Draw performance metrics"""
        try:
            if not self._show_performance_metrics:
                return
                
            perf_width = 200
            perf_height = 100
            perf_pos = (20, self.dim[1] - perf_height - 20)
            
            perf_surface = pygame.Surface((perf_width, perf_height))
            perf_surface.set_alpha(180)
            perf_surface.fill((20, 20, 20))
            
            title = self._font_mono.render("PERFORMANCE", True, (255, 100, 255))
            perf_surface.blit(title, (10, 10))
            
            # FPS
            fps_text = self._font_small.render(
                f"FPS: {self.server_fps:.1f}", 
                True, (255, 255, 255)
            )
            perf_surface.blit(fps_text, (10, 35))
            
            # Frame time
            frame_time = 1000.0 / self.server_fps if self.server_fps > 0 else 0
            frame_time_text = self._font_small.render(
                f"Frame: {frame_time:.1f}ms", 
                True, (255, 255, 255)
            )
            perf_surface.blit(frame_time_text, (10, 55))
            
            # Memory usage (optional, requires psutil)
            try:
                import psutil  # type: ignore
                memory_percent = psutil.virtual_memory().percent
                memory_text = self._font_small.render(
                    f"Memory: {memory_percent:.1f}%", 
                    True, (255, 255, 255)
                )
                perf_surface.blit(memory_text, (10, 75))
            except (ImportError, ModuleNotFoundError):
                memory_text = self._font_small.render(
                    "Memory: N/A", 
                    True, (255, 255, 255)
                )
                perf_surface.blit(memory_text, (10, 75))
            
            pygame.draw.rect(perf_surface, (255, 100, 255), (0, 0, perf_width, perf_height), 2)
            display.blit(perf_surface, perf_pos)
            
        except Exception as e:
            pass

    def draw_lane_detection_overlay(self, display, world):
        """Draw advanced lane detection overlay with lane markings and boundaries"""
        try:
            if not self._show_lane_overlay or not self.lane_detection:
                return
                
            # Initialize lane detection if not done
            if not self.lane_detection:
                self.lane_detection = AdvancedLaneDetection(world.player, world.world)
            
            # Get lane markings and road boundaries
            lane_markings, road_boundaries = self.lane_detection.detect_lane_markings()
            center_line = self.lane_detection.get_lane_center_line()
            lateral_offset, longitudinal_pos = self.lane_detection.calculate_lane_offset()
            
            # Create overlay surface
            overlay_surface = pygame.Surface(self.dim)
            overlay_surface.set_alpha(120)
            overlay_surface.fill((0, 0, 0, 0))
            
            # Draw lane center line
            if center_line:
                points = []
                for point in center_line[:20]:  # Show next 20 points
                    # Convert 3D world coordinates to 2D screen coordinates
                    screen_x = int(self.dim[0] // 2 + (point['location'].x - world.player.get_transform().location.x) * 2)
                    screen_y = int(self.dim[1] // 2 - (point['location'].y - world.player.get_transform().location.y) * 2)
                    
                    if 0 <= screen_x < self.dim[0] and 0 <= screen_y < self.dim[1]:
                        points.append((screen_x, screen_y))
                
                if len(points) > 1:
                    pygame.draw.lines(overlay_surface, (0, 255, 255), False, points, 3)
            
            # Draw road boundaries
            if road_boundaries:
                for boundary in road_boundaries[:10]:  # Show next 10 boundaries
                    # Left boundary
                    left_x = int(self.dim[0] // 2 + (boundary['left_edge'].x - world.player.get_transform().location.x) * 2)
                    left_y = int(self.dim[1] // 2 - (boundary['left_edge'].y - world.player.get_transform().location.y) * 2)
                    
                    # Right boundary
                    right_x = int(self.dim[0] // 2 + (boundary['right_edge'].x - world.player.get_transform().location.x) * 2)
                    right_y = int(self.dim[1] // 2 - (boundary['right_edge'].y - world.player.get_transform().location.y) * 2)
                    
                    if 0 <= left_x < self.dim[0] and 0 <= left_y < self.dim[1]:
                        pygame.draw.circle(overlay_surface, (255, 255, 0), (left_x, left_y), 2)
                    
                    if 0 <= right_x < self.dim[0] and 0 <= right_y < self.dim[1]:
                        pygame.draw.circle(overlay_surface, (255, 255, 0), (right_x, right_y), 2)
            
            # Draw lane offset indicator
            offset_x = self.dim[0] // 2 + int(lateral_offset * 20)  # Scale offset for visibility
            offset_y = self.dim[1] - 50
            
            # Color based on offset severity
            if abs(lateral_offset) > 1.0:
                color = (255, 0, 0)  # Red - critical
            elif abs(lateral_offset) > 0.5:
                color = (255, 165, 0)  # Orange - warning
            else:
                color = (0, 255, 0)  # Green - safe
            
            pygame.draw.circle(overlay_surface, color, (offset_x, offset_y), 8)
            
            # Draw offset text
            offset_text = self._font_small.render(f"Offset: {lateral_offset:.2f}m", True, color)
            overlay_surface.blit(offset_text, (offset_x - 30, offset_y - 20))
            
            display.blit(overlay_surface, (0, 0))
            
        except Exception as e:
            pass

    def draw_sensor_zones_overlay(self, display, world):
        """Draw sensor detection zones overlay"""
        try:
            if not self._show_sensor_zones or not self.sensor_visualization:
                return
                
            # Initialize sensor visualization if not done
            if not self.sensor_visualization:
                self.sensor_visualization = SensorRangeVisualization(world.player, world.world)
            
            # Get detection zones
            zones = self.sensor_visualization.get_detection_zones()
            
            # Create overlay surface
            overlay_surface = pygame.Surface(self.dim)
            overlay_surface.set_alpha(80)
            overlay_surface.fill((0, 0, 0, 0))
            
            # Draw sensor zones
            for zone_name, zone in zones.items():
                center = zone['center']
                range_val = zone['range']
                angle = zone['angle']
                color = zone['color'][:3]  # Remove alpha for pygame
                
                # Convert world coordinates to screen coordinates
                screen_x = int(self.dim[0] // 2 + (center.x - world.player.get_transform().location.x) * 2)
                screen_y = int(self.dim[1] // 2 - (center.y - world.player.get_transform().location.y) * 2)
                
                if 0 <= screen_x < self.dim[0] and 0 <= screen_y < self.dim[1]:
                    # Draw detection zone as circle
                    radius = int(range_val * 2)  # Scale for visibility
                    pygame.draw.circle(overlay_surface, color, (screen_x, screen_y), radius, 2)
                    
                    # Draw zone label
                    label = self._font_small.render(zone_name.upper(), True, color)
                    overlay_surface.blit(label, (screen_x - 20, screen_y - 10))
            
            display.blit(overlay_surface, (0, 0))
            
        except Exception as e:
            pass

    def draw_technical_overlay(self, display, world):
        """Draw technical overlay with measurements and indicators"""
        try:
            if not self._show_technical_overlay:
                return
                
            # Create technical overlay surface
            overlay_surface = pygame.Surface((400, 300))
            overlay_surface.set_alpha(200)
            overlay_surface.fill((20, 20, 30))
            
            # Technical header
            header = self._font_mono.render("TECHNICAL OVERLAY", True, (0, 255, 255))
            overlay_surface.blit(header, (10, 10))
            
            y_offset = 40
            
            # Vehicle dynamics
            vel = world.player.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            accel = world.player.get_acceleration()
            acceleration = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
            
            # Speed and acceleration
            speed_text = self._font_small.render(f"Speed: {speed:.1f} km/h", True, (255, 255, 255))
            overlay_surface.blit(speed_text, (10, y_offset))
            y_offset += 20
            
            accel_text = self._font_small.render(f"Acceleration: {acceleration:.2f} m/s²", True, (255, 255, 255))
            overlay_surface.blit(accel_text, (10, y_offset))
            y_offset += 20
            
            # Lane information
            if self.lane_detection:
                lateral_offset, longitudinal_pos = self.lane_detection.calculate_lane_offset()
                
                lane_text = self._font_small.render(f"Lateral Offset: {lateral_offset:.3f}m", True, (255, 255, 255))
                overlay_surface.blit(lane_text, (10, y_offset))
                y_offset += 20
                
                long_text = self._font_small.render(f"Longitudinal: {longitudinal_pos:.3f}m", True, (255, 255, 255))
                overlay_surface.blit(long_text, (10, y_offset))
                y_offset += 20
            
            # Sensor ranges
            if self.sensor_visualization:
                ranges = self.sensor_visualization.sensor_ranges
                
                for sensor, range_val in ranges.items():
                    sensor_text = self._font_small.render(f"{sensor.upper()}: {range_val:.1f}m", True, (200, 200, 200))
                    overlay_surface.blit(sensor_text, (10, y_offset))
                    y_offset += 15
            
            # ADAS status indicators
            if world.adas_manager:
                adas_status = world.adas_manager.get_adas_status()
                
                # Active systems
                active_systems = []
                if adas_status.get('acc_enabled', False):
                    active_systems.append("ACC")
                if adas_status.get('aeb_enabled', False):
                    active_systems.append("AEB")
                if adas_status.get('overtaking_enabled', False):
                    active_systems.append("OVT")
                
                if active_systems:
                    systems_text = self._font_small.render(f"Active: {', '.join(active_systems)}", True, (0, 255, 0))
                    overlay_surface.blit(systems_text, (10, y_offset))
                    y_offset += 20
            
            # Draw border
            pygame.draw.rect(overlay_surface, (0, 255, 255), (0, 0, 400, 300), 2)
            
            # Position overlay in top-right corner
            display.blit(overlay_surface, (self.dim[0] - 420, 20))
            
        except Exception as e:
            pass

    def draw_professional_hud_frame(self, display):
        """Draw professional HUD frame with technical styling"""
        try:
            # Create main HUD frame
            frame_surface = pygame.Surface((self.dim[0], 100))
            frame_surface.set_alpha(150)
            frame_surface.fill((10, 10, 20))
            
            # Draw top border with gradient effect
            for i in range(5):
                color_intensity = 50 + i * 20
                pygame.draw.line(frame_surface, (color_intensity, color_intensity, 255), 
                               (0, i), (self.dim[0], i), 1)
            
            # Draw bottom border
            pygame.draw.line(frame_surface, (0, 255, 255), (0, 95), (self.dim[0], 95), 2)
            
            # Add corner brackets
            bracket_size = 20
            # Top-left bracket
            pygame.draw.line(frame_surface, (0, 255, 255), (10, 10), (10 + bracket_size, 10), 3)
            pygame.draw.line(frame_surface, (0, 255, 255), (10, 10), (10, 10 + bracket_size), 3)
            
            # Top-right bracket
            pygame.draw.line(frame_surface, (0, 255, 255), (self.dim[0] - 10, 10), (self.dim[0] - 10 - bracket_size, 10), 3)
            pygame.draw.line(frame_surface, (0, 255, 255), (self.dim[0] - 10, 10), (self.dim[0] - 10, 10 + bracket_size), 3)
            
            # Add technical grid pattern
            for i in range(0, self.dim[0], 20):
                pygame.draw.line(frame_surface, (30, 30, 50), (i, 0), (i, 100), 1)
            
            for i in range(0, 100, 20):
                pygame.draw.line(frame_surface, (30, 30, 50), (0, i), (self.dim[0], i), 1)
            
            # Position at top of screen
            display.blit(frame_surface, (0, 0))
            
        except Exception as e:
            pass

    def toggle_info(self):
        """Toggle info on or off"""
        self._show_info = not self._show_info

    def notification(self, text, color=(255, 255, 255), seconds=2.0):
        """Notification text"""
        self._notifications.set_text(text, color, seconds=seconds)

    def error(self, text):
        """Error text"""
        self._notifications.set_text('Error: %s' % text, color=(255, 0, 0))

    def render(self, display):
        """Enhanced render method"""
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        fig = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + fig * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        
        # Draw professional HUD frame
        self.draw_professional_hud_frame(display)
        
        # Draw enhanced ADAS visualizations
        if self.world:
            # Advanced technical overlays (drawn first for background)
            if self._show_lane_overlay:
                self.draw_lane_detection_overlay(display, self.world)
            if self._show_sensor_zones:
                self.draw_sensor_zones_overlay(display, self.world)
            if self._show_technical_overlay:
                self.draw_technical_overlay(display, self.world)
            
            # Standard HUD displays
            if self._show_radar:
                self.draw_radar_display(display, self.world)
            if self._show_speed_graph:
                self.draw_speed_graph(display, self.world)
            if self._show_ttc_display:
                self.draw_ttc_display(display, self.world)
            if self._show_adas_dashboard:
                self.draw_adas_dashboard(display, self.world)
            if self._show_navigation:
                self.draw_navigation_display(display, self.world)
            if self._show_weather:
                self.draw_weather_display(display, self.world)
            if self._show_traffic_density:
                self.draw_traffic_density_display(display, self.world)
            if self._show_minimap:
                self.draw_minimap(display, self.world)
            if self._show_vehicle_health:
                self.draw_vehicle_health_display(display, self.world)
            if self._show_performance_metrics:
                self.draw_performance_metrics(display, self.world)
        
        self._notifications.render(display)
        self.help.render(display)
# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================

class FadingText(object):
    """ Class for fading text """

    def __init__(self, font, dim, pos):
        """Constructor method"""
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0.0  # Make sure this is a float
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """Set fading text"""
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = float(seconds)  # Ensure it's a float
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        """Fading text method for every tick"""
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """Render fading text method"""
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """ Helper class for text render"""

    def __init__(self, font, width, height):
        """Constructor method"""
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for i, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, i * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """Toggle on or off the render help"""
        self._render = not self._render

    def render(self, display):
        """Render help text method"""
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    """ Class for collision sensors"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        """Gets the history of collisions"""
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        """On collision method"""
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    """Class for lane invasion sensors"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        """On invasion method"""
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    """ Class for GNSS sensors"""

    def __init__(self, parent_actor):
        """Constructor method"""
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(x=1.0, z=2.8)),
                                        attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        """GNSS method"""
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    """ Class for camera management"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), attachment.Rigid),
            (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            blp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                blp.set_attribute('image_size_x', str(hud.dim[0]))
                blp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                blp.set_attribute('range', '50')
            item.append(blp)
        self.index = None

    def toggle_camera(self):
        """Activate a camera"""
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        """Set a sensor"""
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (
            force_respawn or (self.sensors[index][0] != self.sensors[self.index][0]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])

            # We need to pass the lambda a weak reference to
            # self to avoid circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        """Get the next sensor"""
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        """Toggle recording on or off"""
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        """Render method"""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- Sensor Visualization Manager ----------------------------------------------
# ==============================================================================

class SensorVisualizationManager:
    """Manages multi-sensor grid visualization"""
    
    def __init__(self, parent_actor, world):
        self._parent = parent_actor
        self.world = world
        self.active_mode = None  # None, 'multi_sensor'
        
        # Multi-sensor grid visualization
        self.multi_sensor_sensors = []
        self.multi_sensor_display = None
        self.multi_sensor_surfaces = {}
    
    def toggle_visualization(self, mode):
        """Toggle between visualization modes: None, 'multi_sensor'"""
        if mode == self.active_mode:
            # Turn off current mode
            self.disable_visualization()
            return None
        else:
            # Switch to new mode
            self.disable_visualization()
            if mode == 'multi_sensor':
                self.enable_multi_sensor_view()
            return mode
    
    def enable_multi_sensor_view(self):
        """Enable multi-sensor grid view"""
        try:
            # Create pygame display for multi-sensor view
            grid_size = [2, 3]
            window_size = [1280, 720]
            self.multi_sensor_display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
            
            # Spawn multiple sensors
            bp_library = self.world.get_blueprint_library()
            
            # Front camera
            front_cam = self._spawn_camera(bp_library, carla.Transform(
                carla.Location(x=0, z=2.4), 
                carla.Rotation(yaw=0)), [0, 0])
            
            # Left camera
            left_cam = self._spawn_camera(bp_library, carla.Transform(
                carla.Location(x=0, z=2.4), 
                carla.Rotation(yaw=-90)), [0, 1])
            
            # Right camera
            right_cam = self._spawn_camera(bp_library, carla.Transform(
                carla.Location(x=0, z=2.4), 
                carla.Rotation(yaw=90)), [0, 2])
            
            # Back camera
            back_cam = self._spawn_camera(bp_library, carla.Transform(
                carla.Location(x=0, z=2.4), 
                carla.Rotation(yaw=180)), [1, 1])
            
            # LiDAR
            lidar = self._spawn_lidar(bp_library, carla.Transform(
                carla.Location(x=0, z=2.4)), [1, 0])
            
            # Semantic LiDAR
            semantic_lidar = self._spawn_semantic_lidar(bp_library, carla.Transform(
                carla.Location(x=0, z=2.4)), [1, 2])
            
            self.active_mode = 'multi_sensor'
            logging.info("Multi-sensor grid view enabled")
            
        except Exception as e:
            logging.error(f"Failed to enable multi-sensor view: {e}")
            self.disable_visualization()
    
    def _spawn_camera(self, bp_library, transform, display_pos):
        """Spawn a camera sensor"""
        camera_bp = bp_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '426')
        camera_bp.set_attribute('image_size_y', '240')
        
        camera = self.world.spawn_actor(camera_bp, transform, attach_to=self._parent)
        
        weak_self = weakref.ref(self)
        camera.listen(lambda image: SensorVisualizationManager._camera_callback(weak_self, image, display_pos))
        
        self.multi_sensor_sensors.append(camera)
        return camera
    
    def _spawn_lidar(self, bp_library, transform, display_pos):
        """Spawn a LiDAR sensor"""
        lidar_bp = bp_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '100')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('points_per_second', '250000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        
        lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=self._parent)
        
        weak_self = weakref.ref(self)
        lidar.listen(lambda data: SensorVisualizationManager._lidar_callback(weak_self, data, display_pos))
        
        self.multi_sensor_sensors.append(lidar)
        return lidar
    
    def _spawn_semantic_lidar(self, bp_library, transform, display_pos):
        """Spawn a semantic LiDAR sensor"""
        lidar_bp = bp_library.find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('range', '100')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        
        lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=self._parent)
        
        weak_self = weakref.ref(self)
        lidar.listen(lambda data: SensorVisualizationManager._semantic_lidar_callback(weak_self, data, display_pos))
        
        self.multi_sensor_sensors.append(lidar)
        return lidar
    
    @staticmethod
    def _camera_callback(weak_self, image, display_pos):
        """Process camera image for multi-sensor view"""
        self = weak_self()
        if not self or not self.multi_sensor_display:
            return
        
        try:
            image.convert(cc.Raw)
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.multi_sensor_surfaces[tuple(display_pos)] = surface
        except Exception as e:
            logging.debug(f"Camera callback error: {e}")
    
    @staticmethod
    def _lidar_callback(weak_self, image, display_pos):
        """Process LiDAR data for multi-sensor view"""
        self = weak_self()
        if not self or not self.multi_sensor_display:
            return
        
        try:
            disp_size = [426, 240]
            lidar_range = 200.0
            
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(disp_size) / lidar_range
            lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (disp_size[0], disp_size[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            surface = pygame.surfarray.make_surface(lidar_img)
            self.multi_sensor_surfaces[tuple(display_pos)] = surface
        except Exception as e:
            logging.debug(f"LiDAR callback error: {e}")
    
    @staticmethod
    def _semantic_lidar_callback(weak_self, image, display_pos):
        """Process semantic LiDAR data for multi-sensor view"""
        self = weak_self()
        if not self or not self.multi_sensor_display:
            return
        
        try:
            disp_size = [426, 240]
            lidar_range = 200.0
            
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 6), 6))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(disp_size) / lidar_range
            lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (disp_size[0], disp_size[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            surface = pygame.surfarray.make_surface(lidar_img)
            self.multi_sensor_surfaces[tuple(display_pos)] = surface
        except Exception as e:
            logging.debug(f"Semantic LiDAR callback error: {e}")
    
    def render_multi_sensor(self):
        """Render multi-sensor grid view"""
        if not self.multi_sensor_display or self.active_mode != 'multi_sensor':
            return
        
        try:
            self.multi_sensor_display.fill((0, 0, 0))
            grid_size = [2, 3]
            display_size = [426, 240]
            
            for (row, col), surface in self.multi_sensor_surfaces.items():
                offset_x = col * display_size[0]
                offset_y = row * display_size[1]
                self.multi_sensor_display.blit(surface, (offset_x, offset_y))
            
            pygame.display.flip()
        except Exception as e:
            logging.debug(f"Multi-sensor render error: {e}")
    
    def disable_visualization(self):
        """Disable all visualizations"""
        if self.active_mode == 'multi_sensor':
            for sensor in self.multi_sensor_sensors:
                if sensor:
                    sensor.destroy()
            self.multi_sensor_sensors = []
            self.multi_sensor_surfaces = {}
            self.multi_sensor_display = None
        
        self.active_mode = None
    
    def destroy(self):
        """Clean up all resources"""
        self.disable_visualization()


# ==============================================================================
# -- Scenario Manager for ADAS Testing ----------------------------------------
# ==============================================================================

class ScenarioManager:
    """Manages test scenarios for ADAS features"""
    
    def __init__(self, world, hud):
        self.world = world
        self.hud = hud
        self.current_scenario = None
        self.scenario_vehicles = []
        self.scenario_active = False
        
    def start_scenario(self, scenario_name):
        """Start a specific test scenario"""
        try:
            if self.scenario_active:
                self.end_scenario()
                
            self.current_scenario = scenario_name
            self.scenario_active = True
            
            if scenario_name == "sudden_braking":
                self._setup_sudden_braking_scenario()
            elif scenario_name == "overtaking":
                self._setup_overtaking_scenario()
            else:
                logging.warning("Unknown scenario: %s", scenario_name)
                return False
                
            self.hud.notification(f'Scenario: {scenario_name} started', color=(0, 255, 0))
            return True
            
        except Exception as e:
            logging.error(f"Scenario start error: {e}")
            return False
    
    def _setup_sudden_braking_scenario(self):
        """Setup sudden braking scenario - spawn vehicle ahead, our vehicle stops suddenly"""
        try:
            # Automatically enable AEB (Automatic Emergency Braking) for this scenario
            if self.world.adas_manager and self.world.adas_manager.aeb:
                if not self.world.adas_manager.aeb.enabled:
                    self.world.adas_manager.aeb.enabled = True
                    self.hud.notification('AEB enabled for emergency braking scenario', color=(255, 0, 0))
            
            # Spawn vehicle directly ahead
            spawn_points = self.world.map.get_spawn_points()
            if not spawn_points:
                return
                
            ego_location = self.world.player.get_transform().location
            ego_forward = self.world.player.get_transform().get_forward_vector()
            
            # Spawn vehicle ahead at sudden close distance (10 meters) for emergency braking
            ahead_spawn = random.choice(spawn_points)
            ahead_spawn.location.x = ego_location.x + ego_forward.x * 10
            ahead_spawn.location.y = ego_location.y + ego_forward.y * 10
            ahead_spawn.location.z = ego_location.z
            
            blueprint = self.world.world.get_blueprint_library().filter('vehicle.*')[0]
            brake_vehicle = self.world.world.try_spawn_actor(blueprint, ahead_spawn)
            
            if brake_vehicle:
                self.scenario_vehicles.append(brake_vehicle)
                # Set up vehicle to brake suddenly
                self._setup_brake_vehicle_behavior(brake_vehicle)
                
            # Set up ego vehicle to stop suddenly when approaching
            self._setup_ego_sudden_brake_behavior()
                
        except Exception as e:
            logging.error("Sudden braking scenario setup error: %s", e)
    
    def _setup_brake_vehicle_behavior(self, vehicle):
        """Setup vehicle ahead to brake suddenly"""
        try:
            def brake_behavior():
                import time
                time.sleep(0.5)  # Wait only 0.5 seconds before sudden braking
                
                for i in range(100):  # 5 seconds of hard braking
                    # Check if vehicle is still alive
                    if not vehicle or not vehicle.is_alive:
                        break
                    
                    try:
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 1.0  # Full brake
                        control.steer = 0.0
                        vehicle.apply_control(control)
                    except RuntimeError:
                        break
                    except Exception as e:
                        logging.debug("Error applying brake control: %s", e)
                        break
                    
                    time.sleep(0.05)
                    
            import threading
            thread = threading.Thread(target=brake_behavior)
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            logging.error("Brake vehicle behavior setup error: %s", e)
    
    def _setup_ego_sudden_brake_behavior(self):
        """Setup ego vehicle to stop suddenly when approaching the vehicle ahead"""
        try:
            def ego_brake_behavior():
                import time
                ego_vehicle = self.world.player
                
                # Move forward initially (reduced time for more sudden emergency)
                for i in range(40):  # 2 seconds of forward movement before emergency brake
                    if not ego_vehicle or not ego_vehicle.is_alive:
                        return
                    
                    try:
                        control = carla.VehicleControl()
                        control.throttle = 0.6
                        control.steer = 0.0
                        control.brake = 0.0
                        ego_vehicle.apply_control(control)
                    except RuntimeError:
                        return
                    except Exception as e:
                        logging.debug("Error in ego brake behavior: %s", e)
                        return
                    
                    time.sleep(0.05)
                
                # Sudden stop
                for i in range(80):  # 4 seconds of hard braking
                    if not ego_vehicle or not ego_vehicle.is_alive:
                        return
                    
                    try:
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 1.0  # Full brake
                        control.steer = 0.0
                        ego_vehicle.apply_control(control)
                    except RuntimeError:
                        return
                    except Exception as e:
                        logging.debug("Error in ego brake behavior: %s", e)
                        return
                    
                    time.sleep(0.05)
                    
            import threading
            thread = threading.Thread(target=ego_brake_behavior)
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            logging.error("Ego sudden brake behavior setup error: %s", e)
    w
    def _setup_overtaking_scenario(self):
        """Setup overtaking scenario with lane detection"""
        try:
            # Automatically enable ADAS overtaking system for this scenario
            if self.world.adas_manager and self.world.adas_manager.overtaking:
                if not self.world.adas_manager.overtaking.enabled:
                    self.world.adas_manager.overtaking.enabled = True
                    self.hud.notification('Overtaking system enabled for scenario', color=(0, 255, 255))
            
            # Spawn vehicle ahead at certain distance
            spawn_points = self.world.map.get_spawn_points()
            if not spawn_points:
                return
                
            ego_location = self.world.player.get_transform().location
            ego_forward = self.world.player.get_transform().get_forward_vector()
            
            # Spawn vehicle ahead (30 meters)
            ahead_spawn = random.choice(spawn_points)
            ahead_spawn.location.x = ego_location.x + ego_forward.x * 30
            ahead_spawn.location.y = ego_location.y + ego_forward.y * 30
            ahead_spawn.location.z = ego_location.z
            
            blueprint = self.world.world.get_blueprint_library().filter('vehicle.*')[0]
            target_vehicle = self.world.world.try_spawn_actor(blueprint, ahead_spawn)
            
            if target_vehicle:
                self.scenario_vehicles.append(target_vehicle)
                # Set up slow vehicle behavior
                self._setup_slow_vehicle_behavior(target_vehicle)
                
            # Set up ego vehicle to overtake
            self._setup_ego_overtaking_behavior()
                
        except Exception as e:
            logging.error("Overtaking scenario setup error: %s", e)
    
    def _setup_slow_vehicle_behavior(self, vehicle):
        """Setup slow vehicle behavior ahead"""
        try:
            def slow_behavior():
                for i in range(300):  # 15 seconds
                    if not vehicle or not vehicle.is_alive:
                        break
                    
                    try:
                        control = carla.VehicleControl()
                        control.throttle = 0.3  # Slow speed
                        control.brake = 0.0
                        control.steer = 0.0
                        vehicle.apply_control(control)
                    except RuntimeError:
                        break
                    except Exception as e:
                        logging.debug("Error in slow behavior: %s", e)
                        break
                    
                    time.sleep(0.05)
                    
            import threading
            thread = threading.Thread(target=slow_behavior)
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            logging.error("Slow vehicle behavior setup error: %s", e)
    
    def _detect_vehicle_lane(self, vehicle):
        """Detect which lane a vehicle is in (left or right)"""
        try:
            vehicle_location = vehicle.get_transform().location
            waypoint = self.world.map.get_waypoint(vehicle_location)
            
            if not waypoint:
                return None
            
            # Get lane ID - negative means left, positive means right
            lane_id = waypoint.lane_id
            
            # Check if it's a left or right lane relative to road direction
            # In CARLA, lane_id can be used to determine position
            # We'll use a simpler approach: check relative position to road center
            
            # Get road center waypoint
            road_center = self.world.map.get_waypoint(vehicle_location)
            
            if not road_center:
                return None
            
            # Get left and right waypoints
            left_waypoint = road_center.get_left_lane()
            right_waypoint = road_center.get_right_lane()
            
            # Determine which lane we're closer to
            vehicle_pos = vehicle_location
            
            if left_waypoint:
                left_distance = vehicle_pos.distance(left_waypoint.transform.location)
            else:
                left_distance = float('inf')
            
            if right_waypoint:
                right_distance = vehicle_pos.distance(right_waypoint.transform.location)
            else:
                right_distance = float('inf')
            
            # If we're closer to left waypoint, we're in left lane
            if left_distance < right_distance and left_distance < 5.0:
                return "left"
            elif right_distance < left_distance and right_distance < 5.0:
                return "right"
            else:
                # Default: check lane ID sign
                if lane_id < 0:
                    return "left"
                else:
                    return "right"
                    
        except Exception as e:
            logging.debug("Lane detection error: %s", e)
            return None
    
    def _check_surroundings_safe(self, ego_vehicle, target_lane):
        """Check if it's safe to change to target lane"""
        try:
            ego_location = ego_vehicle.get_transform().location
            ego_waypoint = self.world.map.get_waypoint(ego_location)
            
            if not ego_waypoint:
                return False
            
            # Get waypoint for target lane
            if target_lane == "left":
                target_waypoint = ego_waypoint.get_left_lane()
            else:
                target_waypoint = ego_waypoint.get_right_lane()
            
            if not target_waypoint:
                return False
            
            # Check for vehicles in target lane within 50 meters
            nearby_vehicles = self.world.world.get_actors().filter('vehicle.*')
            
            for vehicle in nearby_vehicles:
                if vehicle.id == ego_vehicle.id:
                    continue
                
                vehicle_location = vehicle.get_transform().location
                distance = ego_location.distance(vehicle_location)
                
                if distance < 50.0:  # Check within 50 meters
                    # Check if vehicle is in target lane
                    vehicle_waypoint = self.world.map.get_waypoint(vehicle_location)
                    if vehicle_waypoint:
                        # Check if vehicle is in same lane as target waypoint
                        if abs(vehicle_waypoint.lane_id - target_waypoint.lane_id) < 2:
                            # Check if vehicle is ahead or behind
                            ego_forward = ego_vehicle.get_transform().get_forward_vector()
                            to_vehicle = vehicle_location - ego_location
                            
                            # If vehicle is ahead and close, not safe
                            if to_vehicle.dot(ego_forward) > 0 and distance < 20.0:
                                return False
                            # If vehicle is behind and very close, not safe
                            elif to_vehicle.dot(ego_forward) < 0 and distance < 10.0:
                                return False
            
            return True
            
        except Exception as e:
            logging.debug("Surroundings check error: %s", e)
            return False
    
    def _setup_ego_overtaking_behavior(self):
        """Setup ego vehicle to overtake with lane detection and return logic"""
        try:
            def ego_overtaking_behavior():
                import time
                ego_vehicle = self.world.player
                
                # Wait a bit before starting
                time.sleep(1.0)
                
                # Find the target vehicle
                target_vehicle = None
                if self.scenario_vehicles:
                    target_vehicle = self.scenario_vehicles[0]
                
                if not target_vehicle or not target_vehicle.is_alive:
                    return
                
                # Detect which lane target vehicle is in
                target_lane = self._detect_vehicle_lane(target_vehicle)
                
                if not target_lane:
                    logging.warning("Could not detect target vehicle lane")
                    return
                
                logging.info(f"Target vehicle is in {target_lane} lane")
                
                # Determine opposite lane
                opposite_lane = "right" if target_lane == "left" else "left"
                
                # Move forward to catch up
                for i in range(100):  # 5 seconds
                    if not ego_vehicle or not ego_vehicle.is_alive:
                        return
                    
                    try:
                        control = carla.VehicleControl()
                        control.throttle = 0.7
                        control.steer = 0.0
                        control.brake = 0.0
                        ego_vehicle.apply_control(control)
                    except RuntimeError:
                        return
                    except Exception as e:
                        logging.debug("Error in overtaking: %s", e)
                        return
                    
                    time.sleep(0.05)
                
                # Change to opposite lane
                steer_direction = 0.3 if opposite_lane == "right" else -0.3
                
                for i in range(80):  # 4 seconds to change lane
                    if not ego_vehicle or not ego_vehicle.is_alive:
                        return
                    
                    try:
                        control = carla.VehicleControl()
                        control.throttle = 0.8
                        control.steer = steer_direction
                        control.brake = 0.0
                        ego_vehicle.apply_control(control)
                    except RuntimeError:
                        return
                    except Exception as e:
                        logging.debug("Error in overtaking: %s", e)
                        return
                    
                    time.sleep(0.05)
                
                # Continue in opposite lane
                for i in range(100):  # 5 seconds forward
                    if not ego_vehicle or not ego_vehicle.is_alive:
                        return
                    
                    try:
                        control = carla.VehicleControl()
                        control.throttle = 0.7
                        control.steer = 0.0
                        control.brake = 0.0
                        ego_vehicle.apply_control(control)
                    except RuntimeError:
                        return
                    except Exception as e:
                        logging.debug("Error in overtaking: %s", e)
                        return
                    
                    time.sleep(0.05)
                
                # Check surroundings and return to original lane if safe
                original_lane = target_lane  # Original lane is where target was
                return_steer = -0.3 if opposite_lane == "right" else 0.3
                
                # Check if safe to return
                safe_to_return = self._check_surroundings_safe(ego_vehicle, original_lane)
                
                if safe_to_return:
                    logging.info("Safe to return to original lane")
                    # Return to original lane
                    for i in range(80):  # 4 seconds to return
                        if not ego_vehicle or not ego_vehicle.is_alive:
                            return
                        
                        try:
                            control = carla.VehicleControl()
                            control.throttle = 0.6
                            control.steer = return_steer
                            control.brake = 0.0
                            ego_vehicle.apply_control(control)
                        except RuntimeError:
                            return
                        except Exception as e:
                            logging.debug("Error in overtaking: %s", e)
                            return
                        
                        time.sleep(0.05)
                    
                    # Straighten out
                    for i in range(40):  # 2 seconds to straighten
                        if not ego_vehicle or not ego_vehicle.is_alive:
                            return
                        
                        try:
                            control = carla.VehicleControl()
                            control.throttle = 0.5
                            control.steer = 0.0
                            control.brake = 0.0
                            ego_vehicle.apply_control(control)
                        except RuntimeError:
                            return
                        except Exception as e:
                            logging.debug("Error in overtaking: %s", e)
                            return
                        
                        time.sleep(0.05)
                else:
                    logging.info("Not safe to return to original lane, staying in current lane")
                    
            import threading
            thread = threading.Thread(target=ego_overtaking_behavior)
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            logging.error("Ego overtaking behavior setup error: %s", e)
    
    def end_scenario(self):
        """End current scenario and clean up"""
        try:
            if not self.scenario_active:
                return
                
            # Destroy scenario vehicles
            for vehicle in self.scenario_vehicles:
                if vehicle and vehicle.is_alive:
                    vehicle.destroy()
                    
            self.scenario_vehicles.clear()
            self.scenario_active = False
            self.current_scenario = None
            
            self.hud.notification('Scenario ended', color=(255, 255, 0))
            
        except Exception as e:
            logging.error("Scenario end error: %s", e)


# ==============================================================================
# -- Traffic Spawner for ADAS Testing (Based on Official CARLA Script) --------
# ==============================================================================

def spawn_traffic_vehicles(client, world, traffic_manager, num_vehicles=50, safe=True):
    """
    Spawn traffic vehicles for testing ADAS features
    Based on official CARLA generate_traffic.py
    
    Args:
        client: CARLA client object
        world: CARLA world object
        traffic_manager: Traffic manager instance
        num_vehicles: Number of vehicles to spawn
        safe: If True, only spawn cars (avoid motorcycles/bikes)
    
    Returns:
        List of spawned vehicle actor IDs
    """
    vehicles_list = []
    
    try:
        # Get vehicle blueprints using proper filtering function
        blueprints = get_actor_blueprints(world, 'vehicle.*', '2')
        
        if not blueprints:
            logging.warning("Couldn't find any vehicle blueprints with the specified filters")
            return []
        
        # Only spawn 4-wheeled vehicles for safety
        if safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
            
        if not blueprints:
            logging.warning("No safe vehicle blueprints found")
            return []
        
        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        
        # Get spawn points
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if num_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif num_vehicles > number_of_spawn_points:
            logging.warning('Requested %d vehicles, but only found %d spawn points', 
                           num_vehicles, number_of_spawn_points)
            num_vehicles = number_of_spawn_points

        # Prepare batch spawn commands
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= num_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            
            blueprint.set_attribute('role_name', 'autopilot')
            
            # Spawn the cars and set their autopilot
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        # Execute batch spawn
        synchronous_master = False
        if world.get_settings().synchronous_mode:
            synchronous_master = True
            
        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        logging.info('Spawned %d traffic vehicles', len(vehicles_list))
        
        # Configure Traffic Manager for ADAS testing
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_respawn_dormant_vehicles(True)
        traffic_manager.set_boundaries_respawn_dormant_vehicles(25, 700)
        
        # Create variety in driving behaviors
        all_vehicle_actors = world.get_actors(vehicles_list)
        for i, actor in enumerate(all_vehicle_actors):
            try:
                # Mix of aggressive, normal, and cautious drivers
                if i % 3 == 0:
                    # Aggressive driver (20% faster)
                    traffic_manager.vehicle_percentage_speed_difference(actor, -20)
                    traffic_manager.distance_to_leading_vehicle(actor, 1.5)
                    traffic_manager.ignore_lights_percentage(actor, 20)
                elif i % 3 == 1:
                    # Normal driver
                    traffic_manager.vehicle_percentage_speed_difference(actor, 0)
                    traffic_manager.distance_to_leading_vehicle(actor, 2.5)
                else:
                    # Cautious driver (20% slower)
                    traffic_manager.vehicle_percentage_speed_difference(actor, 20)
                    traffic_manager.distance_to_leading_vehicle(actor, 3.5)
                
                # Enable lane changes for some vehicles
                if i % 4 == 0:
                    traffic_manager.auto_lane_change(actor, True)
                else:
                    traffic_manager.auto_lane_change(actor, False)
                
                # Update vehicle lights
                traffic_manager.update_vehicle_lights(actor, True)
            except Exception as e:
                logging.warning('Failed to configure vehicle %d: %s', actor.id, e)
                continue
        
        return vehicles_list
        
    except Exception as e:
        logging.error('Error spawning traffic vehicles: %s', e)
        return vehicles_list


# ==============================================================================
# -- Modified Game Loop with Traffic -------------------------------------------
# ==============================================================================

def game_loop(args):
    """
    Main loop of the simulation. It handles updating all the HUD information,
    ticking the agent and, if needed, the world.
    """

    pygame.init()
    pygame.font.init()
    world = None
    traffic_vehicles = []
    client = None
    traffic_manager = None
    synchronous_master = False

    try:
        if args.seed:
            random.seed(args.seed)

        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        traffic_manager = client.get_trafficmanager(args.tm_port)
        sim_world = client.get_world()

        # Handle synchronous mode properly
        settings = sim_world.get_settings()
        if args.sync:
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            traffic_manager.set_synchronous_mode(True)
            sim_world.apply_settings(settings)
        else:
            # Ensure we're not in synchronous mode if not requested
            if settings.synchronous_mode:
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                sim_world.apply_settings(settings)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        controller = KeyboardControl(world)
        
        # ====== SPAWN TRAFFIC VEHICLES ======
        logging.info("Spawning %d traffic vehicles for ADAS testing...", args.num_vehicles)
        traffic_vehicles = spawn_traffic_vehicles(
            client, 
            sim_world, 
            traffic_manager, 
            args.num_vehicles,
            safe=True
        )
        
        # Wait for vehicles to be ready
        if args.sync and synchronous_master:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()
        
        logging.info("Traffic spawned successfully: %d vehicles", len(traffic_vehicles))
        # ====================================
        
        if args.agent == "Basic":
            agent = BasicAgent(world.player, 30)
            agent.follow_speed_limits(True)
        elif args.agent == "Constant":
            agent = ConstantVelocityAgent(world.player, 30)
            ground_loc = world.world.ground_projection(world.player.get_location(), 5)
            if ground_loc:
                world.player.set_location(ground_loc.location + carla.Location(z=0.01))
            agent.follow_speed_limits(True)
        elif args.agent == "Behavior":
            agent = BehaviorAgent(world.player, behavior=args.behavior)

        # Set the agent destination
        spawn_points = world.map.get_spawn_points()
        if not spawn_points:
            logging.error("No spawn points available")
            return
        destination = random.choice(spawn_points).location
        agent.set_destination(destination)

        clock = pygame.time.Clock()

        logging.info("="*80)
        logging.info("ENHANCED ADAS TESTING SIMULATION")
        logging.info("="*80)
        logging.info("ADAS Features: ACC, AEB, FCW, LDW, BSD, TSR, OVT")
        logging.info("Total Traffic Vehicles: %d", len(traffic_vehicles))
        logging.info("="*80)

        while True:
            clock.tick()
            if args.sync and synchronous_master:
                world.world.tick()
            else:
                world.world.wait_for_tick()
            if controller.parse_events(world):
                return

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

            if agent.done():
                if args.loop:
                    agent.set_destination(random.choice(spawn_points).location)
                    world.hud.notification("Target reached", seconds=4.0)
                    logging.info("The target has been reached, searching for another target")
                else:
                    logging.info("The target has been reached, stopping the simulation")
                    break

            control = agent.run_step()
            control.manual_gear_shift = False
            
            # Apply ADAS control modifications
            if world.adas_manager:
                try:
                    control = world.adas_manager.update(control)
                except Exception as e:
                    logging.warning("ADAS update error: %s", e)
            
            world.player.apply_control(control)

    except KeyboardInterrupt:
        logging.info("Cancelled by user")
    except Exception as e:
        logging.error("Error in game loop: %s", e, exc_info=True)
    finally:
        # Restore synchronous mode settings
        if world is not None and world.world is not None:
            try:
                settings = world.world.get_settings()
                if synchronous_master:
                    settings.synchronous_mode = False
                    settings.fixed_delta_seconds = None
                    world.world.apply_settings(settings)
                if traffic_manager is not None:
                    traffic_manager.set_synchronous_mode(False)
            except Exception as e:
                logging.warning("Error restoring settings: %s", e)

            # Clean up traffic vehicles
            if traffic_vehicles and client is not None:
                try:
                    logging.info("Cleaning up traffic vehicles...")
                    client.apply_batch([carla.command.DestroyActor(x) for x in traffic_vehicles])
                    logging.info("Destroyed %d traffic vehicles", len(traffic_vehicles))
                except Exception as e:
                    logging.warning("Error cleaning up traffic vehicles: %s", e)

            try:
                world.destroy()
            except Exception as e:
                logging.warning("Error destroying world: %s", e)

        pygame.quit()


# ==============================================================================
# -- main() with traffic parameters --------------------------------------------
# ==============================================================================


def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client with ADAS')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic", "Constant"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)
    argparser.add_argument(
        '-n', '--num-vehicles',
        metavar='N',
        default=50,
        type=int,
        help='Number of traffic vehicles to spawn (default: 50)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    logging.info(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        logging.info('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
