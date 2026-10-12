import sys
import time
import math

# Keep your custom environment path
sys.path.append(r"C:\Users\EPP\emio-env\Lib\site-packages")

from emioapi import EmioAPI


# ============================================================
# CONNECT TO EMIO
# ============================================================

e = EmioAPI(multiprocess_camera=False)

if e.connectToEmioDevice("COM7"):

    print("Connected to EMIO")

    # Enable native marker tracking and camera display
    e.camera.track_markers = True
    e.camera.show_frames = True

    # Read resting motor position
    original = list(e.motors.angles)

    print("Original baseline angles:", original)


    # ========================================================
    # CIRCLE PARAMETERS
    # ========================================================

    amplitude = 0.8       # radians - circle radius in motor space
    loop_duration = 10.0   # seconds for one complete circle
    time_step = 0.01      # control update interval

    start_time = time.time()

    print("Tracing circle.")
    print("Press Ctrl+C to stop.")


    try:

        while True:

            # ------------------------------------------------
            # TIME PARAMETER
            # ------------------------------------------------

            current_time = time.time() - start_time

            # 0 -> 2*pi over one circle
            t = (
                2.0
                * math.pi
                * current_time
                / loop_duration
            )


            # =================================================
            # PARAMETRIC CIRCLE
            # =================================================

            dx = amplitude * math.cos(t)
            dy = amplitude * math.sin(t)


            # =================================================
            # MOTOR COMMANDS
            # =================================================

            target = list(original)

            # Antagonistic tendon pairs
            target[0] += dx
            target[1] += dy
            target[2] -= dx
            target[3] -= dy

            e.motors.angles = target


            # =================================================
            # CAMERA UPDATE
            # =================================================

            e.camera.update()

            pixel_positions = e.camera.trackers_pos


            # =================================================
            # TRACK END EFFECTOR
            # =================================================

            if pixel_positions and len(pixel_positions) > 0:

                first_marker = pixel_positions[0]

                if len(first_marker) >= 2:

                    cX = int(first_marker[0])
                    cY = int(first_marker[1])

                    depth_map = e.camera.depth_frame

                    if depth_map is not None:

                        # Make sure pixel is inside image
                        height, width = depth_map.shape[:2]

                        if (
                            0 <= cX < width
                            and 0 <= cY < height
                        ):

                            z_depth = depth_map[cY, cX]

                            # Convert camera pixel/depth
                            # to simulation coordinates
                            sim_coords = (
                                e.camera.image_to_simulation(
                                    cX,
                                    cY,
                                    float(z_depth)
                                )
                            )

                            print(
                                f"Tip -> "
                                f"X: {sim_coords[0]:.2f}, "
                                f"Y: {sim_coords[1]:.2f}, "
                                f"Z: {sim_coords[2]:.2f}"
                            )


            # ------------------------------------------------
            # LOOP RATE
            # ------------------------------------------------

            time.sleep(time_step)


    except KeyboardInterrupt:

        print("\nCircle tracing stopped.")


    finally:

        # ====================================================
        # RETURN TO HOME POSITION
        # ====================================================

        print("Returning to original position...")

        e.motors.angles = original

        time.sleep(2.0)

        e.disconnect()

        print("EMIO disconnected cleanly.")


else:

    print("Could not connect to EMIO")
