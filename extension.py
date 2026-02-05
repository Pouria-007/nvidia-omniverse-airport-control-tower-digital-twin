"""
AIRPORT USD COMPOSER SETUP EXTENSION
Combines Control Tower Dashboard + Waypoint Navigation + RF Visualization
"""

import omni.ext
import omni.ui as ui
import omni.usd
import omni.kit.app
from pxr import Gf, UsdGeom, Usd
import math
from omni.debugdraw import get_debug_draw_interface
import omni.physx

class Airport_Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("\n" + "="*70)
        print("AIRPORT EXTENSION STARTING")
        print("="*70)
        
        # ==================== WAYPOINT CONFIGURATION ====================
        self.AIRCRAFT_PATH = "/World/Aircraft"
        self.WAYPOINT_PATHS = [
            "/World/Waypoints/Waypoint_10",
            "/World/Waypoints/Waypoint_20",
            "/World/Waypoints/Waypoint_30",
            "/World/Waypoints/Waypoint_40",
            "/World/Waypoints/Waypoint_50",
            "/World/Waypoints/Waypoint_60",
            "/World/Waypoints/Waypoint_70",
            "/World/Waypoints/Waypoint_80",
        ]
        self.AXIS_MAP = [0, 2, 1]  # Map waypoint (X,Y,Z) to aircraft (X,Z,Y)
        
        # ==================== RF VISUALIZATION CONFIGURATION ====================
        self.ANTENNAS_VIS = [
            "/World/Aircraft/Antennas/ANT_VHF_COMM_TOP",
            "/World/Aircraft/Antennas/ANT_VHF_COMM_BOTTOM",
        ]
        self.DISTANCE_NEAR = 15000.0   # Green zone
        self.DISTANCE_MED = 40000.0    # Yellow zone
        self.RAY_WIDTH_MAX = 8.0
        self.RAY_WIDTH_MIN = 2.0
        self.COLOR_GREEN = 0xFF00FF00
        self.COLOR_YELLOW = 0xFFFFFF00
        self.COLOR_RED = 0xFFFF0000
        self.COLOR_BLOCKED = 0xFFAA0000
        
        # ==================== CAMERA CONFIGURATION ====================
        self.CAMERA_PATHS = [
            "/Environment/Camera_01",
            "/Environment/Camera_02",
            "/Environment/Camera_03",
            "/Environment/Camera_04",
        ]
        
        # ==================== CONTROL TOWER CONFIGURATION ====================
        self.ANTENNA_ROOT = "/World/Aircraft/Antennas"
        self.VOLUMES = {
            "block": "/World/Volumes/RF_BLOCKING_VOLUME",
            "atten": "/World/Volumes/RF_ATTENUATION_VOLUME",
            "secure": "/World/Volumes/SECURE_ZONE_VOLUME",
        }
        self.ANTENNA_NAMES = [
            "ANT_SATCOM_PRIMARY", "ANT_GNSS_1", "ANT_GNSS_2",
            "ANT_VHF_COMM_TOP", "ANT_VHF_COMM_BOTTOM", "ANT_ATC_TRANSPONDER",
            "ANT_DME", "ANT_WEATHER_RADAR", "ANT_HF_LONG_RANGE", "ANT_ELT"
        ]
        self.ANTENNA_DESCRIPTIONS = {
            "ANT_SATCOM_PRIMARY": "Satellite Communication\nPrimary antenna for high-bandwidth satellite connectivity (Ka/Ku/L-band)\nUsed for data links, internet, and communications\nwith ground stations via satellite",
            "ANT_GNSS_1": "Global Navigation Satellite System (Primary GPS)\nReceives signals from GPS, GLONASS, Galileo satellites\nfor precise positioning and navigation",
            "ANT_GNSS_2": "Global Navigation Satellite System (Redundant GPS)\nBackup GNSS receiver for navigation redundancy\nand increased accuracy through dual-receiver configuration",
            "ANT_VHF_COMM_TOP": "VHF Communication (Top Mount)\nVery High Frequency radio for air traffic control communications\nPilot-to-controller voice communications (118-137 MHz)",
            "ANT_VHF_COMM_BOTTOM": "VHF Communication (Bottom/Ground)\nLower-mounted VHF antenna optimized for ground communications\nand tower contact during taxi and parking",
            "ANT_ATC_TRANSPONDER": "ATC Transponder\nAutomatically responds to radar interrogations from air traffic control\nTransmitting aircraft identification, altitude, and status (Mode A/C/S)",
            "ANT_DME": "Distance Measuring Equipment\nMeasures slant-range distance to ground-based DME beacons\nfor navigation, works with VOR stations for position fixing",
            "ANT_WEATHER_RADAR": "Weather Radar\nX-band forward-looking radar for detecting precipitation,\nturbulence, and weather hazards ahead of the aircraft",
            "ANT_HF_LONG_RANGE": "HF Long Range Communication\nHigh Frequency radio for long-distance communication\nover oceans and remote areas where VHF is out of range (3-30 MHz)",
            "ANT_ELT": "Emergency Locator Transmitter\nDistress beacon that transmits on 406 MHz and 121.5 MHz\nto alert search and rescue services in case of crash or emergency"
        }
        self.STATE_COLORS = {
            "ON": 0xFF00FF00,
            "DEGRADED": 0xFF00FFFF,
            "OFF": 0xFF0000FF,
            "UNKNOWN": 0xFF777777
        }
        
        # ==================== STATE ====================
        self._waypoint_data = []
        self._current_progress = 0.0
        self._waypoint_nav_window = None
        self._waypoint_progress_slider = None
        self._rf_enabled = False
        self._rf_subscription = None
        self._debug_draw = None
        self._frame_count = 0
        self._towers = []
        self._physx_scene_query = None
        self._raycast_stats = {"blocked": 0, "clear": 0}
        self._camera_buttons = []
        self._active_camera = 0
        self._ui_elements = {}
        self._control_tower_window = None
        self._control_tower_subscription = None
        self._update_frame_count = 0
        self._tower_layer_muted = True  # Track tower layer muted state
        self.TOWER_LAYER_PATH = '/home/pouria/aimodels/Projects/AircraftOperationsCenter/towers_loc_B.usd'
        
        # ==================== INITIALIZATION ====================
        if self.verify_scene():
            self.set_progress(0.0)
            self.create_waypoint_ui()
            self.start_control_tower()
            print("✓ Airport Extension Started Successfully")
            print("="*70 + "\n")
        else:
            print("✗ Scene verification failed")
            print("="*70 + "\n")
    
    # ==================== WAYPOINT FUNCTIONS ====================
    
    def verify_scene(self):
        """Verify aircraft and waypoints exist and cache their transforms."""
        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("[ERROR] No stage")
            return False
        
        aircraft_prim = stage.GetPrimAtPath(self.AIRCRAFT_PATH)
        if not aircraft_prim or not aircraft_prim.IsValid():
            print(f"[ERROR] Aircraft not found: {self.AIRCRAFT_PATH}")
            return False
        print(f"[Waypoint] ✓ Aircraft: {self.AIRCRAFT_PATH}")
        
        self._waypoint_data = []
        for wp_path in self.WAYPOINT_PATHS:
            prim = stage.GetPrimAtPath(wp_path)
            if not prim or not prim.IsValid():
                print(f"[ERROR] Missing waypoint: {wp_path}")
                return False
            
            xformable = UsdGeom.Xformable(prim)
            translation = Gf.Vec3d(0, 0, 0)
            rotation_euler = Gf.Vec3f(0, 0, 0)
            
            for xform_op in xformable.GetOrderedXformOps():
                op_type = xform_op.GetOpType()
                if op_type == UsdGeom.XformOp.TypeTranslate:
                    translation = xform_op.Get()
                elif op_type == UsdGeom.XformOp.TypeRotateXYZ:
                    rotation_euler = xform_op.Get()
                elif op_type == UsdGeom.XformOp.TypeRotateX:
                    rotation_euler[0] = xform_op.Get()
                elif op_type == UsdGeom.XformOp.TypeRotateY:
                    rotation_euler[1] = xform_op.Get()
                elif op_type == UsdGeom.XformOp.TypeRotateZ:
                    rotation_euler[2] = xform_op.Get()
            
            if translation == Gf.Vec3d(0, 0, 0):
                local_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = local_transform.ExtractTranslation()
            
            self._waypoint_data.append({
                'path': wp_path,
                'translation': translation,
                'rotation_euler': rotation_euler
            })
        
        print(f"[Waypoint] ✓ Loaded {len(self._waypoint_data)} waypoints")
        return True
    
    def interpolate_transform(self, progress):
        """Interpolate between waypoints based on progress (0.0 to 1.0)."""
        progress = max(0.0, min(1.0, progress))
        num_segments = len(self._waypoint_data) - 1
        segment_size = 1.0 / num_segments
        segment_index = int(progress / segment_size)
        if segment_index >= num_segments:
            segment_index = num_segments - 1
        
        local_blend = (progress - segment_index * segment_size) / segment_size
        local_blend = max(0.0, min(1.0, local_blend))
        
        wp_start = self._waypoint_data[segment_index]
        wp_end = self._waypoint_data[segment_index + 1]
        
        trans_start = wp_start['translation']
        trans_end = wp_end['translation']
        interp_translation = trans_start * (1.0 - local_blend) + trans_end * local_blend
        
        rot_start = wp_start['rotation_euler']
        rot_end = wp_end['rotation_euler']
        interp_rotation_waypoint = Gf.Vec3f(
            rot_start[0] * (1.0 - local_blend) + rot_end[0] * local_blend,
            rot_start[1] * (1.0 - local_blend) + rot_end[1] * local_blend,
            rot_start[2] * (1.0 - local_blend) + rot_end[2] * local_blend
        )
        
        interp_rotation = Gf.Vec3f(
            interp_rotation_waypoint[self.AXIS_MAP[0]],
            interp_rotation_waypoint[self.AXIS_MAP[1]],
            interp_rotation_waypoint[self.AXIS_MAP[2]]
        )
        
        return interp_translation, interp_rotation
    
    def update_aircraft_transform(self, progress):
        """Update aircraft transform based on progress."""
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return False
        
        aircraft_prim = stage.GetPrimAtPath(self.AIRCRAFT_PATH)
        if not aircraft_prim or not aircraft_prim.IsValid():
            return False
        
        translation, rotation_euler = self.interpolate_transform(progress)
        xformable = UsdGeom.Xformable(aircraft_prim)
        
        translate_op = None
        rotate_op = None
        
        for xform_op in xformable.GetOrderedXformOps():
            op_type = xform_op.GetOpType()
            if op_type == UsdGeom.XformOp.TypeTranslate:
                translate_op = xform_op
            elif op_type == UsdGeom.XformOp.TypeRotateXYZ:
                rotate_op = xform_op
        
        if not translate_op:
            translate_op = xformable.AddTranslateOp()
        if not rotate_op:
            rotate_op = xformable.AddRotateXYZOp()
        
        translate_op.Set(translation)
        rotate_op.Set(rotation_euler)
        
        return True
    
    def set_progress(self, progress):
        """Set aircraft position along the waypoint path."""
        self._current_progress = progress
        if self.update_aircraft_transform(progress):
            if self._waypoint_progress_slider:
                self._waypoint_progress_slider.model.set_value(progress)
            return True
        return False
    
    # ==================== RF VISUALIZATION FUNCTIONS ====================
    
    def get_all_towers(self):
        """Dynamically find all towers."""
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return []
        
        towers_parent = stage.GetPrimAtPath("/World/Towers")
        if not towers_parent or not towers_parent.IsValid():
            return []
        
        tower_paths = []
        for child in towers_parent.GetChildren():
            if child.GetName().startswith("Tower_"):
                tower_paths.append(child.GetPath().pathString)
        
        return sorted(tower_paths)
    
    def get_world_position(self, prim_path, stage):
        """Get world-space position of a prim."""
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None
        
        xformable = UsdGeom.Xformable(prim)
        world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return world_transform.ExtractTranslation()
    
    def compute_distance(self, pos1, pos2):
        """Compute Euclidean distance between two points."""
        diff = pos1 - pos2
        return math.sqrt(diff[0]**2 + diff[1]**2 + diff[2]**2)
    
    def check_line_of_sight(self, start_pos, end_pos):
        """Check if there's a clear line of sight using physics raycast."""
        if not self._physx_scene_query:
            try:
                self._physx_scene_query = omni.physx.get_physx_scene_query_interface()
            except:
                return (True, None, None)
        
        if not self._physx_scene_query:
            return (True, None, None)
        
        origin = (float(start_pos[0]), float(start_pos[1]), float(start_pos[2]))
        direction_vec = end_pos - start_pos
        distance = math.sqrt(direction_vec[0]**2 + direction_vec[1]**2 + direction_vec[2]**2)
        
        if distance > 0:
            direction = (
                float(direction_vec[0] / distance),
                float(direction_vec[1] / distance),
                float(direction_vec[2] / distance)
            )
        else:
            return (True, None, None)
        
        try:
            hit = self._physx_scene_query.raycast_closest(origin, direction, distance)
            if hit and hit["hit"]:
                hit_distance = hit.get("distance", 0)
                hit_prim_path = hit.get("rigidBody", "Unknown")
                return (False, hit_distance, hit_prim_path)
            else:
                return (True, None, None)
        except:
            return (True, None, None)
    
    def get_color_from_distance(self, distance):
        """Get color based on distance."""
        if distance <= self.DISTANCE_NEAR:
            return self.COLOR_GREEN
        elif distance <= self.DISTANCE_MED:
            return self.COLOR_YELLOW
        else:
            return self.COLOR_RED
    
    def get_state_from_distance(self, distance):
        """Map distance to antenna state."""
        if distance <= self.DISTANCE_NEAR:
            return "ON"
        elif distance <= self.DISTANCE_MED:
            return "DEGRADED"
        else:
            return "OFF"
    
    def get_thickness_from_distance(self, distance):
        """Get ray thickness based on distance."""
        if distance <= self.DISTANCE_NEAR:
            return self.RAY_WIDTH_MAX
        elif distance >= self.DISTANCE_MED * 1.5:
            return self.RAY_WIDTH_MIN
        else:
            t = (distance - self.DISTANCE_NEAR) / ((self.DISTANCE_MED * 1.5) - self.DISTANCE_NEAR)
            t = max(0.0, min(1.0, t))
            return self.RAY_WIDTH_MAX - (self.RAY_WIDTH_MAX - self.RAY_WIDTH_MIN) * t
    
    def draw_signal_ray(self, tower_pos, end_pos, distance, is_blocked=False):
        """Draw a visual ray from tower to antenna."""
        if not self._debug_draw:
            self._debug_draw = get_debug_draw_interface()
        
        if is_blocked:
            color = self.COLOR_BLOCKED
            thickness = self.RAY_WIDTH_MIN
        else:
            color = self.get_color_from_distance(distance)
            thickness = self.get_thickness_from_distance(distance)
        
        try:
            self._debug_draw.draw_line(
                tuple(tower_pos),
                color,
                thickness,
                tuple(end_pos),
                color,
                thickness
            )
        except:
            pass
    
    def update_rf_signals(self, dt):
        """Per-frame update for RF visualization with collision detection."""
        if not self._rf_enabled:
            return
        
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        
        self._frame_count += 1
        
        if self._frame_count % 60 == 0:
            self._debug_draw = get_debug_draw_interface()
            self._raycast_stats = {"blocked": 0, "clear": 0}
        
        for antenna_path in self.ANTENNAS_VIS:
            antenna_pos = self.get_world_position(antenna_path, stage)
            if not antenna_pos:
                continue
            
            closest_tower = None
            closest_distance = float('inf')
            closest_tower_pos = None
            is_signal_blocked = False
            blocking_obstacle = None
            
            for tower_path in self._towers:
                tower_pos = self.get_world_position(tower_path, stage)
                if not tower_pos:
                    continue
                
                distance = self.compute_distance(tower_pos, antenna_pos)
                is_clear, hit_distance, hit_prim = self.check_line_of_sight(tower_pos, antenna_pos)
                
                if is_clear:
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_tower = tower_path
                        closest_tower_pos = tower_pos
                        is_signal_blocked = False
                        blocking_obstacle = None
                else:
                    if closest_distance == float('inf'):
                        if distance < closest_distance or closest_distance == float('inf'):
                            closest_distance = distance
                            closest_tower = tower_path
                            closest_tower_pos = tower_pos
                            is_signal_blocked = True
                            blocking_obstacle = hit_prim
            
            if closest_tower_pos:
                if is_signal_blocked:
                    self._raycast_stats["blocked"] += 1
                else:
                    self._raycast_stats["clear"] += 1
                
                self.draw_signal_ray(closest_tower_pos, antenna_pos, closest_distance, is_blocked=is_signal_blocked)
    
    def toggle_rf_visualization(self, enabled):
        """Enable or disable RF visualization."""
        self._rf_enabled = enabled
        
        if enabled:
            self._towers = self.get_all_towers()
            
            if not self._towers:
                print("[RF] No towers found!")
                self._rf_enabled = False
                return
            
            try:
                self._physx_scene_query = omni.physx.get_physx_scene_query_interface()
                physx_status = "✓ Physics ENABLED (collision detection active)"
            except:
                self._physx_scene_query = None
                physx_status = "⚠ Physics UNAVAILABLE (no collision detection)"
            
            if not self._rf_subscription:
                stream = omni.kit.app.get_app().get_update_event_stream()
                self._rf_subscription = stream.create_subscription_to_pop(
                    self.update_rf_signals,
                    name="rf_visualization_update"
                )
            
            print(f"[RF] Visualization ENABLED ({len(self._towers)} towers)")
            print(f"[RF] {physx_status}")
        else:
            print("[RF] Visualization DISABLED")
    
    # ==================== TOWER LAYER TOGGLE FUNCTIONS ====================
    
    def toggle_tower_layer(self):
        """Toggle the visibility of the tower layer."""
        import omni.kit.commands
        
        # Toggle the muted state
        self._tower_layer_muted = not self._tower_layer_muted
        
        try:
            omni.kit.commands.execute('SetLayerMuteness',
                layer_identifier=self.TOWER_LAYER_PATH,
                muted=self._tower_layer_muted)
            
            status = "HIDDEN" if self._tower_layer_muted else "VISIBLE"
            print(f"[Tower Layer] Tower location B is now {status}")
        except Exception as e:
            print(f"[Tower Layer] Error toggling layer: {e}")
    
    # ==================== CAMERA FUNCTIONS ====================
    
    def switch_camera(self, camera_index):
        """Switch to a specific camera (1-4)."""
        if camera_index < 1 or camera_index > 4:
            print(f"[Camera] Invalid camera index: {camera_index}")
            return
        
        camera_path = self.CAMERA_PATHS[camera_index - 1]
        
        try:
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.set_active_camera(camera_path)
                self._active_camera = camera_index
                
                # Update button styles - yellow border for active camera
                for i, btn in enumerate(self._camera_buttons):
                    if i == camera_index - 1:
                        btn.set_style({"border_width": 2, "border_color": 0xFFFFFF00, "border_radius": 3})
                    else:
                        btn.set_style({"border_width": 0})
                
                print(f"[Camera] Switched to {camera_path}")
            else:
                print("[Camera] No active viewport found")
        except Exception as e:
            print(f"[Camera] Error switching camera: {e}")
    
    # ==================== CONTROL TOWER FUNCTIONS ====================
    
    def is_inside_volume(self, ant_pos, volume_prim, stage):
        """Check if antenna position is inside a volume's bounding box."""
        if not volume_prim or not volume_prim.IsValid():
            return False
        
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
        volume_bbox = bbox_cache.ComputeWorldBound(volume_prim)
        aligned_box = volume_bbox.ComputeAlignedBox()
        
        return aligned_box.Contains(ant_pos)
    
    def update_antenna_states(self, stage):
        """Update signal_state for all antennas based on volume logic."""
        if not stage:
            return
        
        block_vol = stage.GetPrimAtPath(self.VOLUMES["block"])
        atten_vol = stage.GetPrimAtPath(self.VOLUMES["atten"])
        secure_vol = stage.GetPrimAtPath(self.VOLUMES["secure"])
        
        states_changed = []
        
        for ant_name in self.ANTENNA_NAMES:
            ant_path = f"{self.ANTENNA_ROOT}/{ant_name}"
            ant_prim = stage.GetPrimAtPath(ant_path)
            
            if not ant_prim or not ant_prim.IsValid():
                continue
            
            xformable = UsdGeom.Xformable(ant_prim)
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            ant_pos = world_transform.ExtractTranslation()
            
            policy_locked = False
            if ant_prim.HasAttribute("policy_locked"):
                policy_locked = ant_prim.GetAttribute("policy_locked").Get() or False
            
            new_state = "ON"
            if self.is_inside_volume(ant_pos, block_vol, stage):
                new_state = "OFF"
            elif self.is_inside_volume(ant_pos, atten_vol, stage):
                new_state = "DEGRADED"
            elif self.is_inside_volume(ant_pos, secure_vol, stage) and policy_locked:
                new_state = "OFF"
            
            old_state = None
            if ant_prim.HasAttribute("signal_state"):
                old_state = ant_prim.GetAttribute("signal_state").Get()
                ant_prim.GetAttribute("signal_state").Set(new_state)
                
                if old_state != new_state:
                    states_changed.append(f"{ant_name}: {old_state}→{new_state}")
        
        if states_changed:
            print(f"[Control Tower] State changes: {', '.join(states_changed)}")
    
    def get_antenna_data(self, stage, ant_name):
        """Get all antenna data for dashboard display."""
        ant_path = f"{self.ANTENNA_ROOT}/{ant_name}"
        ant_prim = stage.GetPrimAtPath(ant_path)
        
        if not ant_prim or not ant_prim.IsValid():
            return None
        
        xformable = UsdGeom.Xformable(ant_prim)
        world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = world_transform.ExtractTranslation()
        
        signal_state = ant_prim.GetAttribute("signal_state").Get() if ant_prim.HasAttribute("signal_state") else "UNKNOWN"
        policy_locked = ant_prim.GetAttribute("policy_locked").Get() if ant_prim.HasAttribute("policy_locked") else False
        freq_band = ant_prim.GetAttribute("frequency_band").Get() if ant_prim.HasAttribute("frequency_band") else "N/A"
        requires_los = ant_prim.GetAttribute("requires_LOS").Get() if ant_prim.HasAttribute("requires_LOS") else False
        ant_type = ant_prim.GetAttribute("antenna:type").Get() if ant_prim.HasAttribute("antenna:type") else "UNKNOWN"
        
        block_vol = stage.GetPrimAtPath(self.VOLUMES["block"])
        atten_vol = stage.GetPrimAtPath(self.VOLUMES["atten"])
        secure_vol = stage.GetPrimAtPath(self.VOLUMES["secure"])
        
        in_block = self.is_inside_volume(pos, block_vol, stage)
        in_atten = self.is_inside_volume(pos, atten_vol, stage)
        in_secure = self.is_inside_volume(pos, secure_vol, stage)
        
        zone = "CLEAR"
        if in_block:
            zone = "RF BLOCKING"
        elif in_atten:
            zone = "RF ATTENUATION"
        elif in_secure and policy_locked:
            zone = "SECURE + LOCKED"
        elif in_secure:
            zone = "SECURE ZONE"
        
        return {
            "name": ant_name,
            "state": signal_state,
            "type": ant_type,
            "freq": freq_band,
            "los": "YES" if requires_los else "NO",
            "locked": "YES" if policy_locked else "NO",
            "zone": zone,
            "pos": f"({pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f})"
        }
    
    def update_tower_ui_data(self):
        """Update only the data in existing UI elements."""
        stage = omni.usd.get_context().get_stage()
        if not stage or not self._ui_elements:
            return
        
        for ant_name in self.ANTENNA_NAMES:
            data = self.get_antenna_data(stage, ant_name)
            if not data or ant_name not in self._ui_elements:
                continue
            
            elements = self._ui_elements[ant_name]
            state_color = self.STATE_COLORS.get(data["state"], self.STATE_COLORS["UNKNOWN"])
            locked_color = 0xFF0000FF if data["locked"] == "YES" else 0xFF00FF00
            
            zone_color = 0xFFFFFFFF
            if data["zone"] == "RF BLOCKING":
                zone_color = 0xFF0000FF
            elif data["zone"] == "RF ATTENUATION":
                zone_color = 0xFF00FFFF
            elif data["zone"] == "SECURE + LOCKED":
                zone_color = 0xFFFF00FF
            elif data["zone"] == "SECURE ZONE":
                zone_color = 0xFF00AAFF
            else:
                zone_color = 0xFF00FF00
            
            elements["circle"].style = {"background_color": state_color}
            elements["state_label"].text = data["state"]
            elements["locked_label"].text = data["locked"]
            elements["locked_label"].style = {"font_size": 14, "color": locked_color, "font_weight": "bold"}
            elements["zone_label"].text = data["zone"]
            elements["zone_label"].style = {"font_size": 13, "color": zone_color, "font_weight": "bold"}
            elements["pos_label"].text = data["pos"]
    
    def tower_master_update(self, dt):
        """Per-frame update for control tower."""
        self._update_frame_count += 1
        
        if self._update_frame_count % 60 == 0:
            print(f"[Control Tower] Update running (frame {self._update_frame_count})")
        
        stage = omni.usd.get_context().get_stage()
        if stage:
            self.update_antenna_states(stage)
            self.update_tower_ui_data()
    
    # ==================== UI BUILDERS ====================
    
    def create_waypoint_ui(self):
        """Create the waypoint navigation UI window."""
        if self._waypoint_nav_window:
            self._waypoint_nav_window.destroy()
            self._waypoint_nav_window = None
        
        self._camera_buttons = []
        self._active_camera = 0
        
        self._waypoint_nav_window = ui.Window(
            "Waypoint Navigation Control",
            width=700,
            height=270,
            flags=ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_SCROLLBAR
        )
        
        self._waypoint_nav_window.position_x = 20
        self._waypoint_nav_window.position_y = 500
        self._waypoint_nav_window.visible = True
        
        with self._waypoint_nav_window.frame:
            with ui.HStack():
                with ui.VStack(spacing=8):
                    # RF Visualization Toggle
                    with ui.HStack(height=25, spacing=10):
                        ui.Label("Enable Signal Visualization", width=200)
                        rf_checkbox = ui.CheckBox(width=20)
                        rf_checkbox.model.set_value(False)
                        rf_checkbox.model.add_value_changed_fn(lambda m: self.toggle_rf_visualization(m.get_value_as_bool()))
                    
                    # Camera Selection Label
                    with ui.HStack(height=5):
                        ui.Label("Camera Selection", width=200)
                    
                    # Camera Buttons
                    with ui.HStack(height=5, spacing=5):
                        for i in range(1, 5):
                            btn = ui.Button(f"Cam {i}", width=60, clicked_fn=lambda idx=i: self.switch_camera(idx))
                            self._camera_buttons.append(btn)
                    
                    ui.Spacer(height=2)
                    
                    # Tower Location Toggle
                    with ui.HStack(height=20):
                        ui.Label("Rewise Tower Location:", width=200)
                    
                    with ui.HStack(height=25, spacing=5):
                        ui.Button("re-design location", width=150, clicked_fn=self.toggle_tower_layer)
                    
                    ui.Spacer(height=2)
                    
                    # Title
                    ui.Label("Aircraft Waypoint Navigation", style={"font_size": 16})
                    
                    ui.Spacer(height=1)
                    
                    # Progress Slider
                    with ui.HStack(height=1):
                        ui.Label("Movement:", width=80)
                        
                        self._waypoint_progress_slider = ui.FloatSlider(
                            min=0.0,
                            max=1.0,
                            width=500
                        )
                        self._waypoint_progress_slider.model.set_value(self._current_progress)
                        self._waypoint_progress_slider.model.add_value_changed_fn(lambda m: self.set_progress(m.get_value_as_float()))
                        
                        progress_label = ui.Label(f"{self._current_progress:.2f}", width=50)
                        self._waypoint_progress_slider.model.add_value_changed_fn(lambda m: setattr(progress_label, 'text', f"{m.get_value_as_float():.2f}"))
                    
                    ui.Spacer(height=2)
                    
                    # Waypoint Buttons
                    with ui.HStack(spacing=1):
                        ui.Label("Quick Move:", width=80)
                        
                        num_waypoints = len(self.WAYPOINT_PATHS)
                        for i in range(1, num_waypoints + 1):
                            progress_val = (i - 1) / (num_waypoints - 1)
                            ui.Button(f"Loc. {i}", width=60, clicked_fn=lambda p=progress_val: self.set_progress(p))
                    
                    ui.Spacer(height=1)  # Bottom spacer
                ui.Spacer(width=20)  # Right margin
    
    def start_control_tower(self):
        """Start the control tower dashboard."""
        if self._control_tower_window:
            self._control_tower_window.destroy()
            self._control_tower_window = None
        
        self._control_tower_window = ui.Window("AIRCRAFT RF CONTROL TOWER", width=1300, height=750)
        
        self._control_tower_window.frame.set_style({
            "Tooltip": {
                "background_color": 0xFF000000,
                "color": 0xFFFFFFFF,
                "border_radius": 4,
                "padding": 12,
                "font_size": 19
            }
        })
        
        with self._control_tower_window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED
            ):
                with ui.ZStack():
                    ui.Rectangle(style={"background_color": 0xFF1A1A1A})
                    with ui.VStack(spacing=0):
                        # Title Header
                        with ui.ZStack(height=60):
                            ui.Rectangle(style={"background_color": 0xFF000000})
                            ui.Label("AIRCRAFT ANTENNA STATUS MONITOR", style={"font_size": 24, "color": 0xFFFFFFFF, "font_weight": "bold", "margin": 20})
                        
                        ui.Spacer(height=12)
                        
                        # Header Row
                        with ui.ZStack(height=42):
                            ui.Rectangle(style={"background_color": 0xFF000000})
                            with ui.HStack():
                                ui.Spacer(width=20)
                                ui.Label("ANTENNA", width=220, style={"font_weight": "bold"}).set_tooltip("Aircraft antenna identifier\nEach antenna has a unique designation\nbased on its function and location on the aircraft")
                                ui.Label("STATE", width=130, style={"font_weight": "bold"}).set_tooltip("Current signal state:\nON (operational)\nDEGRADED (partial signal)\nOFF (no signal)")
                                ui.Label("TYPE", width=160, style={"font_weight": "bold"}).set_tooltip("Antenna system type\nIndicates the communication or navigation\nsystem this antenna belongs to")
                                ui.Label("FREQUENCY", width=150, style={"font_weight": "bold"}).set_tooltip("Operating frequency band\nThe radio frequency range this antenna\ntransmits and receives on")
                                ui.Label("LOS", width=65, style={"font_weight": "bold"}).set_tooltip("Line of Sight Required\nYES: needs clear view of sky/transmitter\nNO: can work through obstacles")
                                ui.Label("LOCKED", width=80, style={"font_weight": "bold"}).set_tooltip("Policy Locked\nYES: subject to security policy,\ndisabled in secure zones\nNO: ignores security restrictions")
                                ui.Label("CURRENT ZONE", width=150, style={"font_weight": "bold"}).set_tooltip("Current RF policy zone:\nRF BLOCKING (signal blocked)\nRF ATTENUATION (signal degraded)\nSECURE+LOCKED (security zone active)\nSECURE ZONE (in security area)\nCLEAR (no restrictions)")
                                ui.Label("POSITION (X, Y, Z)", width=0, style={"font_weight": "bold"}).set_tooltip("World-space coordinates of antenna in centimeters\nUpdates in real-time as aircraft moves")
                        
                        # Antenna Data Rows
                        for idx, ant_name in enumerate(self.ANTENNA_NAMES):
                            bg = 0xFF303030 if idx % 2 == 0 else 0xFF1A1A1A
                            with ui.ZStack(height=48):
                                ui.Rectangle(style={"background_color": bg})
                                with ui.HStack():
                                    ui.Spacer(width=20)
                                    l = ui.Label(ant_name, width=220)
                                    l.set_tooltip(self.ANTENNA_DESCRIPTIONS.get(ant_name, ""))
                                    with ui.HStack(width=130, spacing=8):
                                        with ui.VStack(width=18):
                                            ui.Spacer()
                                            c = ui.Circle(width=18, height=18)
                                            ui.Spacer()
                                        s = ui.Label("ON", width=100, style={"font_weight": "bold"})
                                    t = ui.Label("TYPE", width=160)
                                    f = ui.Label("FREQ", width=150)
                                    lo = ui.Label("NO", width=65)
                                    lk = ui.Label("NO", width=80)
                                    z = ui.Label("CLEAR", width=150)
                                    p = ui.Label("(0,0,0)")
                                    self._ui_elements[ant_name] = {
                                        "circle": c,
                                        "state_label": s,
                                        "locked_label": lk,
                                        "zone_label": z,
                                        "pos_label": p
                                    }
                        
                        ui.Spacer(height=20)
                        
                        # Legend
                        with ui.ZStack(height=40):
                            ui.Rectangle(style={"background_color": 0xFF1A1A1A})
                            with ui.HStack(spacing=30):
                                ui.Spacer(width=40)
                                ui.Label("LEGEND:", style={"font_size": 14, "color": 0xFFCCCCCC, "font_weight": "bold"})
                                for label, color in [("ON", 0xFF00FF00), ("DEGRADED", 0xFF00FFFF), ("OFF", 0xFF0000FF)]:
                                    with ui.HStack(spacing=8):
                                        with ui.VStack(width=16):
                                            ui.Spacer()
                                            ui.Circle(width=16, height=16, style={"background_color": color})
                                            ui.Spacer()
                                        ui.Label(label, style={"font_size": 13, "color": 0xFFCCCCCC})
                                ui.Spacer()
        
        # Subscribe to update loop
        stream = omni.kit.app.get_app().get_update_event_stream()
        self._control_tower_subscription = stream.create_subscription_to_pop(
            self.tower_master_update,
            name="control_tower_dashboard_update"
        )
        
        print("[Control Tower] Started - Dashboard active")
    
    # ==================== SHUTDOWN ====================
    
    def on_shutdown(self):
        """Clean up on extension shutdown."""
        print("\n" + "="*70)
        print("AIRPORT EXTENSION SHUTTING DOWN")
        print("="*70)
        
        if self._rf_subscription:
            self._rf_subscription.unsubscribe()
            self._rf_subscription = None
            print("✓ RF visualization stopped")
        
        if self._control_tower_subscription:
            self._control_tower_subscription.unsubscribe()
            self._control_tower_subscription = None
            print("✓ Control tower stopped")
        
        if self._waypoint_nav_window:
            self._waypoint_nav_window.destroy()
            self._waypoint_nav_window = None
            print("✓ Waypoint window closed")
        
        if self._control_tower_window:
            self._control_tower_window.destroy()
            self._control_tower_window = None
            print("✓ Control tower window closed")
        
        print("="*70 + "\n")
