import cv2
import socket
import time
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Drone info
DRONE_IP = "192.168.1.1"
DRONE_UDP_PORT = 7099
RTSP_URL = "rtsp://192.168.1.1:7070/webcam"

# Camera switch payloads
CAMERA_1_CMD = bytes([6, 1])
CAMERA_2_CMD = bytes([6, 2])

def udp_send(raw_bytes: bytes, addr=(DRONE_IP, DRONE_UDP_PORT)):
    """Send raw bytes over UDP to the drone."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(raw_bytes, addr)
    finally:
        sock.close()

class CameraViewer:
    def __init__(self, rtsp_url=RTSP_URL):
        self.rtsp_url = rtsp_url
        self.cap = None
        self.is_drone = False
        self.open_stream()

        self.current_cam = 1
        self.window_name = "DRONE"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.WINDOW_WIDTH = 640
        self.WINDOW_HEIGHT = 480
        cv2.resizeWindow(self.window_name, self.WINDOW_WIDTH, self.WINDOW_HEIGHT)

        self.prev_time = time.time()
        self.fps = 0.0

    def open_stream(self, reopen_delay=0.3):
        """Open RTSP stream, fallback to webcam if it fails."""
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            time.sleep(reopen_delay)

        # Try drone RTSP first
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            time.sleep(0.2)
            self.cap = cv2.VideoCapture(self.rtsp_url)

        if self.cap.isOpened():
            self.is_drone = True
            print("Connected to drone RTSP stream")
        else:
            # Fallback to webcam
            print("Failed to connect to drone; falling back to webcam")
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.is_drone = False
                print("Webcam opened successfully")
            else:
                raise RuntimeError("Failed to open RTSP stream and webcam")

    def switch_camera(self):
        """Toggle between camera 1 and 2 (drone only)."""
        if not self.is_drone:
            print("Camera switching only available for drone feed")
            return

        new_cam = 2 if self.current_cam == 1 else 1
        print(f"Switching to Camera {new_cam}...")

        if new_cam == 1:
            udp_send(CAMERA_1_CMD)
        else:
            udp_send(CAMERA_2_CMD)

        try:
            self.open_stream(reopen_delay=0.7)
        except RuntimeError:
            time.sleep(0.5)
            try:
                self.open_stream(reopen_delay=0.7)
            except RuntimeError as e:
                print("Warning: failed to reopen RTSP after camera switch:", e)

        self.current_cam = new_cam

    def process_landmarks(self, hand_landmarker, mp_image, frame, cap):
        results = hand_landmarker.detect_for_video(
                mp_image,
                int(cap.get(cv2.CAP_PROP_POS_MSEC))
            )

        if results.hand_landmarks:
            h, w = frame.shape[:2]
            hand = results.hand_landmarks[0]

            x_coords = [lm.x * w for lm in hand]
            y_coords = [lm.y * h for lm in hand]
            x_min, x_max = int(min(x_coords)), int(max(x_coords))
            y_min, y_max = int(min(y_coords)), int(max(y_coords))

            return True, x_min, y_min, x_max, y_max
        else:
            return False, 0, 0, 0, 0

    def run(self):
        print("Press 'c' to switch camera, 'q' to quit.")
        reconnect_attempts = 0
        MAX_RECONNECTS = 5

        #mediapipe initialization
        base_options = python.BaseOptions(
            model_asset_path="hand_landmarker.task",
            delegate="GPU"
        )
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.34,
            min_hand_presence_confidence=0.34,
            min_tracking_confidence=0.34
        )
        hand_landmarker = vision.HandLandmarker.create_from_options(options)    

        while True:
            if self.cap is None or not self.cap.isOpened():
                reconnect_attempts += 1
                if reconnect_attempts > MAX_RECONNECTS:
                    print("Too many reconnect attempts, exiting.")
                    break
                try:
                    print(f"Stream closed; attempting reconnect {reconnect_attempts}")
                    self.open_stream()
                except RuntimeError as e:
                    print("Reconnect failed:", e)
                    time.sleep(0.8)
                    continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                reconnect_attempts += 1
                if reconnect_attempts >= 8:
                    print("No frames. Reopening stream...")
                    try:
                        self.open_stream()
                        reconnect_attempts = 0
                        continue
                    except RuntimeError as e:
                        print("Failed to reopen stream:", e)
                        time.sleep(0.5)
                        continue
                else:
                    continue
            reconnect_attempts = 0

            # Rotate if drone, flip for webcam
            if self.is_drone:
                try:
                    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                except Exception:
                    pass
            else:
                try:
                    frame = cv2.flip(frame, 1)
                except Exception:
                    pass

            # Detect hands with MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            has_hand, x_min, y_min, x_max, y_max = self.process_landmarks(hand_landmarker, mp_image, frame, self.cap)

            if has_hand:
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cx, cy = (x_min + x_max) // 2, (y_min + y_max) // 2
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Compute FPS
            current_time = time.time()
            dt = current_time - self.prev_time if (current_time - self.prev_time) > 1e-6 else 1e-6
            self.fps = 0.85 * self.fps + 0.15 * (1.0 / dt)
            self.prev_time = current_time

            # Upscale to window size while maintaining aspect ratio
            h, w = frame.shape[:2]
            aspect_ratio = w / h
            window_aspect = self.WINDOW_WIDTH / self.WINDOW_HEIGHT
            
            if aspect_ratio > window_aspect:
                new_w = self.WINDOW_WIDTH
                new_h = int(self.WINDOW_WIDTH / aspect_ratio)
            else:
                new_h = self.WINDOW_HEIGHT
                new_w = int(self.WINDOW_HEIGHT * aspect_ratio)
            
            frame_upscaled = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
            
            # Center the frame on a black background
            canvas = cv2.imread(None)
            canvas = frame_upscaled.copy() if frame_upscaled.shape[:2] == (self.WINDOW_HEIGHT, self.WINDOW_WIDTH) else cv2.copyMakeBorder(
                frame_upscaled,
                (self.WINDOW_HEIGHT - new_h) // 2,
                (self.WINDOW_HEIGHT - new_h) - (self.WINDOW_HEIGHT - new_h) // 2,
                (self.WINDOW_WIDTH - new_w) // 2,
                (self.WINDOW_WIDTH - new_w) - (self.WINDOW_WIDTH - new_w) // 2,
                cv2.BORDER_CONSTANT,
                value=(0, 0, 0)
            )
            frame_upscaled = canvas

            # Overlay info
            source = "Drone" if self.is_drone else "Webcam"
            info_text = f"Source: {source} | Camera: {self.current_cam} | Res: {frame.shape[1]}x{frame.shape[0]} | FPS: {self.fps:.1f}"

            cv2.putText(frame_upscaled, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow(self.window_name, frame_upscaled)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('c'):
                self.switch_camera()

        # Cleanup
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
        hand_landmarker.close()


if __name__ == "__main__":
    try:
        viewer = CameraViewer(RTSP_URL)
        viewer.run()
    except Exception as e:
        print("Fatal error:", e)