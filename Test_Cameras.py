import cv2
import socket
import time

# Drone info
DRONE_IP = "192.168.1.1"
DRONE_UDP_PORT = 7099
RTSP_URL = "rtsp://192.168.1.1:7070/webcam"

# Camera switch payloads (matches app's debugSend usage)
CAMERA_1_CMD = bytes([6, 1])
CAMERA_2_CMD = bytes([6, 2])

def udp_send(raw_bytes: bytes, addr=(DRONE_IP, DRONE_UDP_PORT), bind_if_needed=False):
    """Send raw bytes over UDP to the drone."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if bind_if_needed:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', 0))
        sock.sendto(raw_bytes, addr)
    finally:
        sock.close()

class DroneViewer:
    def __init__(self, rtsp_url=RTSP_URL):
        self.rtsp_url = rtsp_url
        self.cap = None
        self.open_stream()

        self.current_cam = 1
        self.window_name = "DRONE"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.WINDOW_WIDTH = 854
        self.WINDOW_HEIGHT = 480
        cv2.resizeWindow(self.window_name, self.WINDOW_WIDTH, self.WINDOW_HEIGHT)

        self.prev_time = time.time()
        self.fps = 0.0

    def open_stream(self, reopen_delay=0.3):
        """Open (or reopen) the RTSP stream."""
        # Release existing capture if any
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            time.sleep(reopen_delay)

        # Try opening with FFMPEG backend (works reliably in many setups)
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        # fallback: try default backend if FFMPEG didn't open
        if not self.cap.isOpened():
            time.sleep(0.2)
            self.cap = cv2.VideoCapture(self.rtsp_url)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open RTSP stream: {self.rtsp_url}")

    def switch_camera(self):
        """Toggle between camera 1 and 2 using the app's camera switch bytes."""
        new_cam = 2 if self.current_cam == 1 else 1
        print(f"Switching to Camera {new_cam}...")

        # send raw debugSend bytes (no prefix) — matches app: debugSend(new byte[]{6,1}) / {6,2}
        if new_cam == 1:
            udp_send(CAMERA_1_CMD)
        else:
            udp_send(CAMERA_2_CMD)

        # apps typically stop and restart their RTSP playback during camera switch.
        # re-open RTSP stream to pick up the new camera feed.
        try:
            self.open_stream(reopen_delay=0.7)
        except RuntimeError:
            # try once more
            time.sleep(0.5)
            try:
                self.open_stream(reopen_delay=0.7)
            except RuntimeError as e:
                print("Warning: failed to reopen RTSP after camera switch:", e)

        self.current_cam = new_cam

    def run(self):
        print("Press 'c' to switch camera, 'q' to quit.")
        reconnect_attempts = 0
        MAX_RECONNECTS = 5

        while True:
            if self.cap is None or not self.cap.isOpened():
                # try to reopen stream (backoff)
                reconnect_attempts += 1
                if reconnect_attempts > MAX_RECONNECTS:
                    print("Too many reconnect attempts, exiting.")
                    break
                try:
                    print("RTSP closed; attempting reconnect", reconnect_attempts)
                    self.open_stream()
                except RuntimeError as e:
                    print("Reconnect failed:", e)
                    time.sleep(0.8)
                    continue
            # read frame
            ret, frame = self.cap.read()
            if not ret or frame is None:
                # try a short wait and reconnect if repeated
                time.sleep(0.05)
                # allow a couple of consecutive misses before trying to reopen
                reconnect_attempts += 1
                if reconnect_attempts >= 8:
                    print("No frames. Reopening RTSP stream...")
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

            # rotate 90 degrees clockwise to correct orientation (as in your script)
            try:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            except Exception:
                pass

            # compute FPS
            current_time = time.time()
            dt = current_time - self.prev_time if (current_time - self.prev_time) > 1e-6 else 1e-6
            self.fps = 0.85 * self.fps + 0.15 * (1.0 / dt)  # smoothed fps
            self.prev_time = current_time

            # upscale to window size
            frame_upscaled = cv2.resize(frame, (self.WINDOW_WIDTH, self.WINDOW_HEIGHT), interpolation=cv2.INTER_CUBIC)

            # overlay info
            info_text = f"Camera: {self.current_cam} | Res: {frame.shape[1]}x{frame.shape[0]} | FPS: {self.fps:.1f}"
            cv2.putText(frame_upscaled, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            cv2.imshow(self.window_name, frame_upscaled)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('c'):
                self.switch_camera()

        # cleanup
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        viewer = DroneViewer(RTSP_URL)
        viewer.run()
    except Exception as e:
        print("Fatal error:", e)
