import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class HandLandmarkProcessor:
    def __init__(self):
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
        self.hand_landmarker = vision.HandLandmarker.create_from_options(options)

    def close(self):
        self.hand_landmarker.close()

    def process_landmarks(self, rgb_frame, frame, cap):
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        results = self.hand_landmarker.detect_for_video(
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