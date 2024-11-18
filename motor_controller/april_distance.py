import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector
import math
import os

def main():
    # Initialize the camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (2592, 1944)})
    picam2.configure(camera_config)
    picam2.start()

    # Define camera intrinsic parameters based on Arducam OV5647 specifications
    fx = 1435.0  # Focal length in pixels along x-axis
    fy = 1440.0  # Focal length in pixels along y-axis
    cx = 1296.0  # Principal point x-coordinate
    cy = 972.0   # Principal point y-coordinate
    camera_params = (fx, fy, cx, cy)

    # Define the real size of the AprilTag (in meters)
    tag_size = 0.073  # 162 mm

    # Initialize AprilTag detector with pose estimation enabled
    at_detector = Detector(
        families='tag36h11',  # Specify the tag family you're using
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    print("Starting distance measurement. Press 'Ctrl+C' to exit.")

    try:
        while True:
            # Capture a frame from the camera
            frame = picam2.capture_array()

            # Convert to grayscale as AprilTag detector expects grayscale images
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags in the image
            tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

            # Process each detected tag
            for tag in tags:
                # Extract the translation component from the pose estimation
                translation = tag.pose_t

                # The distance to the tag is the z-component of the translation vector
                distance = float(translation[2])  # Ensure distance is a scalar float

                # Draw the detected tag boundaries and ID on the frame
                corners = tag.corners.astype(int)
                for i in range(4):
                    cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1) % 4]), (0, 255, 0), 2)
                cv2.putText(frame, f"ID: {tag.tag_id}", (corners[0][0], corners[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(frame, f"Distance: {distance:.2f} m", (corners[0][0], corners[0][1] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # Print the distance to the console
                print(f"Detected tag ID {tag.tag_id} at distance: {distance:.2f} meters")

            # Save the frame to disk (replace with imshow if GUI is enabled)
            output_path = os.path.join("output", "frame.jpg")
            os.makedirs("output", exist_ok=True)
            cv2.imwrite(output_path, frame)
            print(f"Frame saved to {output_path}")

    except KeyboardInterrupt:
        print("\nDistance measurement stopped by user.")

    finally:
        picam2.stop()

if __name__ == "__main__":
    main()
