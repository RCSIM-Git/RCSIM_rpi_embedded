"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Menedżer AI do detekcji obiektów (YOLO) w czasie rzeczywistym.
AI Manager for real-time object detection (YOLO).

Obsługuje Hailo-8L (NPU) oraz tryb Mock (dla testów).
Supports Hailo-8L (NPU) and Mock mode (for testing).
"""

import json
import logging
import os
import time
from datetime import datetime
from typing import Any

import cv2
import numpy as np

# Standard HailoRT imports for RPi5/Hailo-8L
try:
    # On PC/WSL2 the module is 'hailort', but on RPi ARM64 it is shipped as 'hailo_platform'!
    from hailort import HailoStreamInterface, InferVStreams, VDevice

    HAILO_AVAILABLE = True
except ImportError:
    try:
        from hailo_platform import HailoStreamInterface, InferVStreams, VDevice

        HAILO_AVAILABLE = True
    except ImportError as e:
        import logging

        print(f"CRITICAL HAILO IMPORT ERROR: {e}")
        HAILO_AVAILABLE = False
        VDevice = HailoStreamInterface = InferVStreams = None

try:
    import cv2

    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False

# from .postprocess_yolo import draw_detections, postprocess_yolo
def postprocess_yolo(*args, **kwargs): return []
def draw_detections(img, *args, **kwargs): return img


class AIManager:
    """
    Menedżer AI obsługujący detekcję obiektów (YOLOv11n) na Hailo.
    AI Manager supporting object detection (YOLOv11n) on Hailo.
    """

    def __init__(self, logger: logging.Logger, ai_config: dict[str, Any]) -> None:
        """
        Inicjalizuje AIManager.
        Initializes the AI Manager.

        Args:
            logger (logging.Logger): Obiekt loggera. / Logger object.
            ai_config (dict[str, Any]): Konfiguracja AI. / AI configuration.
        """
        self.logger: logging.Logger = logger
        self.config: dict[str, Any] = ai_config

        ai_cfg = self.config.get("ai", {})
        self.model_path: str | None = (
            self.config.get("model_path")
            or self.config.get("hef_path")
            or ai_cfg.get("model_path")
            or ai_cfg.get("hef_path")
        )

        self.hef_path = self.model_path  # Compatibility alias
        self.model_type: str | None = None

        # TFLite usunięte z powodu braków zależności i braku użycia

        # Hailo
        self.hailo_device: VDevice | None = None
        self.hailo_vstreams: InferVStreams | None = None
        self.hailo_input_vstreams: list[Any] | None = None
        self.hailo_output_vstreams: list[Any] | None = None

        # Metadata modelu / Model metadata
        self.is_multimodal: bool = False
        self.vector_size: int = 64  # Elite State Vector v1.0
        self.image_input_name: str | None = None
        self.sensor_input_name: str | None = None

        # Debugowanie i monitorowanie zdrowia / Debug and health monitoring
        self.last_debug_info: dict[str, Any] = {
            "active": False,
            "inference_time_ms": 0.0,
            "raw_steering": 0.0,
            "raw_throttle": 0.0,
            "engine": "NONE",
            "timeout": False,
            "consecutive_timeouts": 0,
            "consecutive_failures": 0,
        }

        # Metryki zdrowia / Health metrics
        self._total_inferences: int = 0
        self._successful_inferences: int = 0
        self.last_fps = 30.0
        self._last_predict_time = time.time()
        self._timeout_count: int = 0
        self._consecutive_timeouts: int = 0
        self._max_consecutive_timeouts: int = 5  # Ostrzeżenie / Trigger warning
        self._max_consecutive_failures: int = 10  # Wyłączenie AI / Disable AI

        # Debug Snapshot Configuration
        nav_config = self.config.get("autonomous_navigation", {})
        debug_cfg = nav_config.get("debug", {})

        self.snapshot_interval = debug_cfg.get("snapshot_interval_sec", 5.0)
        self.snapshots_enabled = debug_cfg.get("enabled", False)
        self.max_snapshot_files = debug_cfg.get("max_snapshot_files", 500)
        self.save_grid_as_png = debug_cfg.get("save_grid_as_png", True)
        self.save_detections_overlay = debug_cfg.get("save_detections_overlay", True)

        self.last_snapshot_time = 0.0
        self.snapshot_dir = debug_cfg.get("snapshot_dir", "debug_logs/snapshots")
        self.json_log_path = os.path.join(
            os.path.dirname(self.snapshot_dir), "detections.jsonl"
        )

        if self.snapshots_enabled:
            os.makedirs(self.snapshot_dir, exist_ok=True)

        self.hailo_device = None
        self.hailo_vstreams = None
        self.is_initialized = False

        # Mock data for sandbox/testing
        self.use_mock = (
            not HAILO_AVAILABLE
            or not self.model_path
            or not os.path.exists(self.model_path)
        )
        self.mock_detections = []

        self.init_model()

    def load_hef(self, hef_path: str):
        """Metoda do (re)ładowania modelu HEF."""
        self.model_path = hef_path
        self.init_model()

    def init_model(self):
        """Inicjalizacja modelu Hailo (vdevice.configure) lub trybu Mock."""
        if hasattr(self, "vdevice") and self.vdevice:
            self.cleanup()

        if not HAILO_AVAILABLE:
            self.logger.warning("Hailo runtime not available → using mock mode")
            self.use_mock = True
            self.is_initialized = True
            return

        if not self.hef_path or not os.path.exists(self.hef_path):
            self.logger.error(
                f"HEF file not found: {self.hef_path}. Switching to MOCK."
            )
            self.use_mock = True
            self.is_initialized = True
            return

        try:
            try:
                from hailort import (HEF, ConfigureParams, FormatType,
                                     InferVStreams, InputVStreamParams,
                                     OutputVStreamParams)
            except ImportError:
                from hailo_platform import (HEF, ConfigureParams, FormatType,
                                            InferVStreams, InputVStreamParams,
                                            OutputVStreamParams)

            self.logger.info(f"Loading Hailo HEF model: {self.hef_path}")

            self.vdevice = VDevice()

            hef_obj = HEF(self.hef_path)

            try:
                params = ConfigureParams.create_from_hef(
                    hef_obj, interface=HailoStreamInterface.PCIe
                )
            except TypeError:
                # Jeśli starsza wersja nie przyjmuje argumentu interface, lub chce go pozycyjnie
                try:
                    params = ConfigureParams.create_from_hef(
                        hef_obj, HailoStreamInterface.PCIe
                    )
                except TypeError:
                    params = ConfigureParams.create_from_hef(hef_obj)

            self.network_group = self.vdevice.configure(hef_obj, params)[0]
            self.network_group_params = self.network_group.create_params()

            self.input_vstreams_params = InputVStreamParams.make(
                self.network_group, format_type=FormatType.UINT8
            )
            self.output_vstreams_params = OutputVStreamParams.make(
                self.network_group, format_type=FormatType.FLOAT32
            )

            # [FIX] Initialize stream info caches
            self.input_infos_cache = hef_obj.get_input_vstream_infos()
            self.output_infos_cache = hef_obj.get_output_vstream_infos()
            
            # Detect input size from the first input stream (usually the image)
            if self.input_infos_cache:
                shape = self.input_infos_cache[0].shape
                # Hailo shapes are often (H, W, C)
                self.input_size = (shape[1], shape[0]) # (W, H)

            # Detect multimodal inputs
            self.image_input_name = None
            self.sensor_input_name = None
            for info in self.input_infos_cache:
                shape = info.shape
                if len(shape) >= 3 and np.prod(shape) > 1000:
                    self.image_input_name = info.name
                else:
                    self.sensor_input_name = info.name
                    self.is_multimodal = True

            self.is_initialized = True
            self.logger.info(
                f"Hailo model loaded: {self.hef_path} | Input Size: {self.input_size} | Multimodal: {self.is_multimodal}"
            )
        except Exception as e:
            self.logger.error(
                f"Failed to load Hailo model: {e}. Switching to MOCK mode."
            )
            self.use_mock = True
            self.is_initialized = True

    def infer(self, image: np.ndarray, sensor_vector: np.ndarray | None = None) -> dict[str, Any]:
        """ Wykonuje inferencję (Obraz + Sensor Vector). """
        if not self.is_initialized:
            return {}

        if self.use_mock:
            return self._mock_infer(image)

        try:
            input_data = {}
            for info in getattr(self, "input_infos_cache", []):
                shape = info.shape
                
                if info.name == self.image_input_name:
                    # Resize i transformacja obrazu
                    if shape[-1] in (1, 3):  # NHWC
                        img = cv2.resize(image, (shape[1], shape[0]))
                        img = np.expand_dims(img, axis=0)
                    else:  # NCHW
                        img = cv2.resize(image, (shape[2], shape[1]))
                        img = img.transpose((2, 0, 1))
                        img = np.expand_dims(img, axis=0)
                    input_data[info.name] = np.ascontiguousarray(img.astype(np.uint8))
                
                elif info.name == self.sensor_input_name:
                    # Iniekcja wektora sensorycznego
                    vec = sensor_vector if sensor_vector is not None else np.zeros(self.vector_size, dtype=np.float32)
                    # Upewnienie się że rozmiar pasuje
                    if vec.shape[0] != np.prod(shape):
                        new_vec = np.zeros(np.prod(shape), dtype=np.float32)
                        new_vec[:min(len(vec), len(new_vec))] = vec[:min(len(vec), len(new_vec))]
                        vec = new_vec
                    
                    vec = vec.reshape(shape)
                    vec = np.expand_dims(vec, axis=0)
                    input_data[info.name] = np.ascontiguousarray(vec.astype(np.float32))
                
                else:
                    # Inne wejścia (fallback)
                    input_data[info.name] = np.ascontiguousarray(np.zeros(shape, dtype=np.float32))

            raw_output = self.infer_pipeline.infer(input_data)

            # Extract output tensors
            all_outputs = {}
            for name, tensor in raw_output.items():
                all_outputs[name] = tensor

            # Primary output (for compatibility)
            if hasattr(self, "output_names") and self.output_names:
                output_tensor = raw_output[self.output_names[0]]
            else:
                output_tensor = list(raw_output.values())[0]

            return {
                "raw_output": output_tensor,
                "all_outputs": all_outputs,
                "shape": output_tensor.shape,
                "mock": False,
            }
        except Exception as e:
            self.logger.error(f"Hailo Raw Inference Error: {e}")
            # Fallback to mock format if real inference fails
            num_dets = 8400
            num_classes = len(self.config.get("ai", {}).get("classes", [])) or 80
            mock_tensor = np.random.rand(1, num_dets, 4 + num_classes).astype(
                np.float32
            )
            return {"raw_output": mock_tensor, "shape": mock_tensor.shape, "mock": True}

    def infer_and_postprocess(self, frame: np.ndarray, sensor_vector: np.ndarray | None = None) -> dict[str, Any]:
        """ High-level wrapper combining inference and regression/YOLO post-processing. """
        result = self.infer(frame, sensor_vector=sensor_vector)
        if "raw_output" not in result:
            return {"detections": [], "ai_controls": None, "raw": result}

        # [NEW] V38.10: Support for End-to-End Regression (Donkey/Monaco Expert)
        ai_controls = {"steering": 0.0, "throttle": 0.0}
        detections = []

        # Case 1: Split-Head Output (Monaco Expert) - Multiple output streams
        raw = result.get("raw_output") # This might be the first stream or a dict
        
        # If infer() returned a dict of all outputs in "raw_output" (we should ensure it does)
        all_outputs = result.get("all_outputs", {})
        
        if all_outputs:
            # Try to find steering/throttle by name or shape
            for name, tensor in all_outputs.items():
                if "steer" in name.lower() or (tensor.shape[-1] == 1 and ai_controls["steering"] == 0.0):
                    ai_controls["steering"] = float(tensor.flatten()[0])
                elif "throttle" in name.lower() or (tensor.shape[-1] == 1 and ai_controls["throttle"] == 0.0):
                    # Monaco Expert uses sigmoid-like bias for throttle in PC, 
                    # but on Hailo it might be already baked or raw.
                    val = float(tensor.flatten()[0])
                    # Simple heuristic: if it's very small and we haven't set throttle yet
                    ai_controls["throttle"] = val

        # Case 2: Single output tensor (Concatenated or YOLO)
        elif isinstance(raw, np.ndarray):
            if raw.shape[-1] == 2: # [steering, throttle]
                ai_controls["steering"] = float(raw[0, 0])
                ai_controls["throttle"] = float(raw[0, 1])
            elif raw.shape[-1] == 1: # Just steering
                ai_controls["steering"] = float(raw[0, 0])

        # Optimization: Don't draw overlay here every frame.
        return {
            "detections": detections,
            "ai_controls": ai_controls,
            "raw": result,
            "inference_time_ms": self.last_debug_info.get("inference_time_ms", 0),
        }

    def predict(
        self, 
        image: np.ndarray, 
        grid: np.ndarray | None = None,
        sensor_data: dict[str, Any] | None = None,
        nav_manager: Any | None = None,
        local_planner: Any | None = None,
        can_save: bool = True
    ) -> tuple[list[dict[str, Any]], dict[str, float] | None]:
        """
        Wykrywa obiekty lub oblicza sterowanie i buduje wektor stanu AI.
        Zwraca (detections, ai_controls).
        """
        if not self.is_initialized:
            return [], None

        try:
            now = time.time()
            dt = now - self._last_predict_time
            if dt > 0:
                # EMA filtered FPS to avoid jitter
                self.last_fps = (self.last_fps * 0.9) + ((1.0 / dt) * 0.1)
            self._last_predict_time = now

            start_time = time.time()
            self._total_inferences += 1

            # 1. Budowanie wektora sensorycznego (Elite Vector 64)
            sensor_vector = self._vectorize_sensors(sensor_data, nav_manager, local_planner)

            # 2. Inferencja (Hailo / Mock)
            res = self.infer_and_postprocess(image, sensor_vector=sensor_vector)
            detections = res["detections"]
            ai_controls = res.get("ai_controls")

            self._successful_inferences += 1
            self._consecutive_failures = 0
            self.last_debug_info["inference_time_ms"] = (
                time.time() - start_time
            ) * 1000

            # Debug Snapshot & Logging - ONLY DRAW OVERLAY HERE if saving
            if self.snapshots_enabled and can_save:
                current_time = time.time()
                if current_time - self.last_snapshot_time >= self.snapshot_interval:
                    self._save_debug_snapshot(image, detections, grid=grid)
                    self.last_snapshot_time = current_time
            elif self.snapshots_enabled and not can_save:
                if self._total_inferences % 100 == 0:
                    self.logger.warning("Snapshot saving PAUSED due to low storage safety state.")

            return detections, ai_controls

        except Exception as e:
            self.logger.error(f"Prediction error: {e}")
            self._consecutive_failures += 1

            # [RECOVERY-AI] Attempt to reload HAILO if it crashes (max 3 times)
            if (
                self._consecutive_failures <= 3
                and not self.use_mock
                and self.is_initialized
            ):
                self.logger.warning(
                    f"AI consecutive failures ({self._consecutive_failures}/3). Attempting model recovery..."
                )
                try:
                    self.init_model()
                except Exception as ex:
                    self.logger.error(f"AI Recovery init failed: {ex}")
            elif self._consecutive_failures > 3 and not self.use_mock:
                self.logger.error(
                    "AI recovery failed 3 times. Switching to MOCK mode for stability."
                )
                self.use_mock = True

            return [], None

    def _vectorize_sensors(
        self, 
        sensor_data: dict[str, Any] | None, 
        nav_manager: Any | None, 
        local_planner: Any | None
    ) -> np.ndarray:
        """ Buduje znormalizowany wektor 64-elementowy. """
        vec = np.zeros(self.vector_size, dtype=np.float32)
        
        if sensor_data is None:
            return vec

        # [0-11] LiDAR Zones (12)
        lidar_data = sensor_data.get("lidar") # List of (angle, dist)
        zones = np.full(12, 5.0, dtype=np.float32) # Default 5m (bezpiecznie)
        if lidar_data:
            raw_zones = np.full(12, 10.0, dtype=np.float32)
            for angle, dist in lidar_data:
                if dist is None or dist < 100: continue
                z_idx = int(angle / 30.0) % 12
                raw_zones[z_idx] = min(raw_zones[z_idx], dist / 1000.0)
            zones = raw_zones
        vec[0:12] = zones / 10.0 # Normalizacja 0-1
        
        # [12-19] Costmap Probes (8) - Sektory 45 stopni
        if local_planner and hasattr(local_planner, 'cm'):
            vec[12:20] = self._get_costmap_probes(local_planner.cm, sensor_data.get("pose", (0,0,0)))
            
        # [20-27] Path Lookahead (8) - 4 punkty [dx, dy]
        speed = sensor_data.get("speed", 0.0)
        if nav_manager:
            vec[20:28] = self._get_path_features(nav_manager, sensor_data.get("pose", (0,0,0)), speed)

        # [28-33] State & Feedback (6)
        vec[28] = speed / 10.0 # Speed
        vec[29] = sensor_data.get("cte", 0.0) / 2.0 # Cross Track Error
        vec[30] = sensor_data.get("heading_error", 0.0) / np.pi # Heading Error
        vec[31] = sensor_data.get("target_bearing", 0.0) / np.pi # Target Bearing
        vec[32] = sensor_data.get("last_steering", 0.0) # Feedback
        vec[33] = sensor_data.get("last_throttle", 0.0) # Feedback
        
        # [34-39] IMU Dynamics (6)
        imu = sensor_data.get("imu", {})
        vec[34] = imu.get("ax", 0.0) / 19.6 # Normalizacja do 2G
        vec[35] = imu.get("ay", 0.0) / 19.6
        vec[36] = imu.get("az", 0.0) / 19.6
        vec[37] = imu.get("gx", 0.0) / 500.0 # deg/s
        vec[38] = imu.get("gy", 0.0) / 500.0
        vec[39] = imu.get("gz", 0.0) / 500.0
        
        # [40-41] Orientation
        vec[40] = imu.get("pitch", 0.0) / 90.0
        vec[41] = imu.get("roll", 0.0) / 90.0
        
        return vec

    def _get_costmap_probes(self, cm: Any, pose: tuple[float, float, float]) -> np.ndarray:
        """ Próbkuje gęstość przeszkód w 8 sektorach wokół robota. """
        rx, ry, ryaw = pose
        yaw_rad = np.deg2rad(ryaw)
        probes = np.zeros(8, dtype=np.float32)
        
        # Odległości próbkowania: 0.5m, 1.0m, 1.5m
        sample_dists = [0.5, 1.0, 1.5]
        
        for i in range(8):
            angle = yaw_rad + (i * np.pi / 4.0) # Sektory co 45 stopni
            sector_sum = 0.0
            for d in sample_dists:
                px = rx + d * np.cos(angle)
                py = ry + d * np.sin(angle)
                gx, gy = cm.world_to_grid(px, py)
                if 0 <= gx < cm.grid_size and 0 <= gy < cm.grid_size:
                    sector_sum += cm.costmap[gx, gy]
            probes[i] = sector_sum / len(sample_dists)
            
        return probes

    def _get_path_features(self, nav: Any, pose: tuple[float, float, float], speed: float) -> np.ndarray:
        """ Pobiera 4 punkty ze ścieżki (dx, dy) w układzie lokalnym robota. """
        rx, ry, ryaw = pose
        yaw_rad = np.deg2rad(ryaw)
        features = np.zeros(8, dtype=np.float32)
        
        if not nav or not nav.current_path or len(nav.current_path) == 0:
            # Fallback: punkty prosto przed robotem
            for i in range(4):
                features[i*2] = (i + 1) * 0.5 # dx
            return features

        # Adaptacyjny lookahead (zgodnie z planem)
        base_l = 1.0
        gain = 0.5
        L = base_l + speed * gain
        
        sample_steps = [0.25, 0.5, 0.75, 1.0]
        
        for i, step in enumerate(sample_steps):
            target_dist = L * step
            # Znajdź najbliższy punkt na ścieżce o dystansie target_dist
            best_pt = nav.current_path[-1] # Domniemywamy ostatni jeśli ścieżka krótka
            
            for j in range(nav.current_waypoint_idx, len(nav.current_path) - 1):
                p2 = nav.current_path[j+1]
                dist_to_robot = np.hypot(p2[0]-rx, p2[1]-ry)
                if dist_to_robot >= target_dist:
                    best_pt = p2
                    break
            
            # Transformacja do układu lokalnego (X przód, Y lewo)
            dx = best_pt[0] - rx
            dy = best_pt[1] - ry
            
            # Rotacja o -yaw
            local_x = dx * np.cos(-yaw_rad) - dy * np.sin(-yaw_rad)
            local_y = dx * np.sin(-yaw_rad) + dy * np.cos(-yaw_rad)
            
            features[i*2] = local_x / 10.0 # Normalizacja do 10m
            features[i*2 + 1] = local_y / 5.0 # Normalizacja do 5m
            
        return features

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Przygotowuje obraz do wejścia modelu Hailo.
        Dostosowuje typ danych (uint8/float32) do wymagań modelu.
        """
        if not hasattr(self, "input_size") or not self.input_size:
            self.input_size = (640, 640)

        # Resize
        img = cv2.resize(image, tuple(self.input_size))

        # Determine expected type from input stream if possible
        # Default to NHWC uint8 which is standard for Hailo RPi models
        expected_type = np.uint8
        if hasattr(self, "input_stream"):
            try:
                # Hailo RT API often expects uint8 even if model is float
                # because the normalization is baked into the HEF.
                from hailo_sdk_client import \
                    HailoStreamInterface  # dummy check
            except ImportError:
                pass

        # Most YOLO models for Hailo on RPi use uint8 NHWC
        # If your specific model was compiled differently, adjust here.
        img = img.astype(expected_type)

        # Format wejścia z konfiguracji (domyślnie NHWC dla Hailo)
        input_format = self.config.get("ai", {}).get("input_format", "NHWC").upper()

        if input_format == "NCHW":
            img = img.transpose((2, 0, 1))  # HWC -> CHW

        img = np.expand_dims(img, axis=0)  # NHWC lub NCHW

        return img

    def _mock_infer(self, frame) -> dict[str, Any]:
        """
        Symuluje wyjście modelu dla testów.
        Simulates model output for testing.
        """
        h, w = frame.shape[:2]
        # 3 przykładowe detekcje / 3 example detections
        mock_boxes = np.array(
            [
                [w * 0.3, h * 0.3, w * 0.1, h * 0.2],  # xywh
                [w * 0.6, h * 0.5, w * 0.15, h * 0.25],
                [w * 0.8, h * 0.7, w * 0.08, h * 0.12],
            ]
        )
        mock_confs = np.array([0.92, 0.78, 0.65])
        mock_classes = np.array([0, 2, 1])  # 0=pachołek, 1=osoba, 2=samochód

        num_anchors = 8400
        num_classes = len(self.config.get("ai", {}).get("classes", [])) or 3
        tensor = np.zeros((1, num_anchors, 4 + 1 + num_classes), dtype=np.float32)

        # Wypełnij tylko pierwsze 3 wiersze
        tensor[0, :3, :4] = mock_boxes
        tensor[0, :3, 4] = mock_confs
        for i in range(3):
            cls_idx = int(mock_classes[i] % num_classes)
            tensor[0, i, 5 + cls_idx] = mock_confs[i]

        return {"raw_output": tensor, "shape": tensor.shape, "mock": True}

    def _save_debug_snapshot(
        self,
        image: np.ndarray,
        detections: list[dict[str, Any]],
        grid: np.ndarray | None = None,
    ) -> str | None:
        """Saves a debug image with drawn bounding boxes and metadata."""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"snapshot_{timestamp}_{self._total_inferences}"

            # Ensure directory exists (safety)
            if not os.path.exists(self.snapshot_dir):
                os.makedirs(self.snapshot_dir, exist_ok=True)

            # 1. Save Overlay Image
            if self.save_detections_overlay:
                debug_img = image.copy()
                # Draw only when actually saving!
                debug_img = draw_detections(debug_img, detections)
                filepath = os.path.join(self.snapshot_dir, f"{base_name}.jpg")
                cv2.imwrite(filepath, debug_img)

            # 2. Save Grid as PNG
            if self.save_grid_as_png and grid is not None:
                grid_path = os.path.join(self.snapshot_dir, f"{base_name}_grid.png")
                cv2.imwrite(grid_path, grid)

            # 3. Save Detections metadata
            meta_path = os.path.join(self.snapshot_dir, f"{base_name}_meta.json")
            with open(meta_path, "w") as f:
                json.dump(
                    {
                        "timestamp": time.time(),
                        "total_inferences": self._total_inferences,
                        "detections": detections,
                    },
                    f,
                    indent=2,
                )

            # 4. Rotation / Cleanup
            self._cleanup_old_snapshots()

            return base_name
        except Exception as e:
            self.logger.error(f"Failed to save debug snapshot: {e}")
            return None

    def _cleanup_old_snapshots(self) -> None:
        """Removes old snapshots if they exceed max_snapshot_files."""
        try:
            if not os.path.exists(self.snapshot_dir):
                return

            files = [
                os.path.join(self.snapshot_dir, f)
                for f in os.listdir(self.snapshot_dir)
            ]
            # Filter to keep only files (not dirs)
            files = [f for f in files if os.path.isfile(f)]

            if len(files) > self.max_snapshot_files:
                # Sort by modification time (oldest first)
                files.sort(key=os.path.getmtime)
                to_delete = files[: len(files) - self.max_snapshot_files]
                for f in to_delete:
                    try:
                        os.remove(f)
                    except Exception as e:
                        self.logger.warning(
                            f"Could not remove old snapshot file {f}: {e}"
                        )
        except Exception as e:
            self.logger.error(f"Snapshot cleanup error: {e}")

    def _log_detections_json(
        self, detections: list[dict[str, Any]], image_filename: str
    ) -> None:
        """Logs detections to a JSONL file."""
        if not image_filename:
            return
        try:
            entry = {
                "timestamp": time.time(),
                "iso_time": datetime.now().isoformat(),
                "image_file": image_filename,
                "detections": detections,
            }
            with open(self.json_log_path, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except Exception as e:
            self.logger.error(f"Failed to log detections JSON: {e}")

    def cleanup(self):
        """Zwalnia zasoby Hailo i zamyka sesję."""
        self.is_initialized = False

        if hasattr(self, "infer_pipeline") and self.infer_pipeline:
            try:
                self.infer_pipeline.__exit__(None, None, None)
            except Exception:
                pass
            self.infer_pipeline = None

        if hasattr(self, "activated_network_group") and self.activated_network_group:
            try:
                self.activated_network_group.__exit__(None, None, None)
            except Exception:
                pass
            self.activated_network_group = None

        if hasattr(self, "hailo_vstreams") and self.hailo_vstreams:
            self.hailo_vstreams = None

        # Consistent cleanup for vdevice
        if hasattr(self, "vdevice") and self.vdevice:
            try:
                # VDevice doesn't always have a close(), but we set to None
                # to trigger garbage collection of the Hailo context
                self.vdevice = None
            except Exception:
                pass

        # Compatibility with legacy code names
        self.hailo_device = None

        self.logger.info("AI resources released.")

    def get_debug_data(self) -> dict[str, Any]:
        """Zwraca metryki wydajności i status."""
        return {
            "engine": "MOCK" if self.use_mock else "HAILO",
            "model": os.path.basename(self.model_path) if self.model_path else "NONE",
            "initialized": self.is_initialized,
            "inference_time_ms": self.last_debug_info.get("inference_time_ms", 0.0),
            "success_rate": (
                self._successful_inferences / max(1, self._total_inferences)
            )
            * 100,
        }

    def reset_health_metrics(self) -> None:
        """Resetuje liczniki zdrowia AI."""
        self._total_inferences = 0
        self._successful_inferences = 0
        self._timeout_count = 0
        self._consecutive_timeouts = 0
        self._consecutive_failures = 0
        self.logger.info("AI health metrics reset.")
