import cv2
import numpy as np
import serial
import time
import sys
import signal

# globals so signal handler can access them
cap = None
ser = None

# Default HSV range for orange (can be adjusted in calibration)
hsv_lower = np.array([5, 150, 150])
hsv_upper = np.array([25, 255, 255])

# Default radius for the blue border circle
border_radius = 210

def find_orange_ball_center(frame, min_r=50, max_r=70):
    """Detect the orange ball center using HSV mask + HoughCircles."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    blurred = cv2.GaussianBlur(mask, (9, 9), 0)
    circles = cv2.HoughCircles(
        blurred,
        method=cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=100,
        param2=15,
        minRadius=min_r,
        maxRadius=max_r
    )
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        # Just return the first circle
        x, y, r = circles[0]
        return (x, y, r)
    return None

def cleanup(cap_obj, ser_obj):
    """Release camera, close windows, close serial (silent)."""
    try:
        if cap_obj is not None:
            cap_obj.release()
    except Exception:
        pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    try:
        if ser_obj is not None and getattr(ser_obj, "is_open", False):
            ser_obj.close()
    except Exception:
        pass

def signal_handler(sig, frame):
    """Handle signals like Ctrl+C (SIGINT) silently."""
    cleanup(cap, ser)
    sys.exit(0)

def nothing(x):
    pass

def calibrate():
    """Open calibration window with trackbars to adjust HSV range and border radius."""
    global hsv_lower, hsv_upper, border_radius

    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibration", 400, 400)

    # HSV lower
    cv2.createTrackbar("H low", "Calibration", hsv_lower[0], 179, nothing)
    cv2.createTrackbar("S low", "Calibration", hsv_lower[1], 255, nothing)
    cv2.createTrackbar("V low", "Calibration", hsv_lower[2], 255, nothing)
    # HSV upper
    cv2.createTrackbar("H high", "Calibration", hsv_upper[0], 179, nothing)
    cv2.createTrackbar("S high", "Calibration", hsv_upper[1], 255, nothing)
    cv2.createTrackbar("V high", "Calibration", hsv_upper[2], 255, nothing)
    # Border radius
    cv2.createTrackbar("Border radius", "Calibration", border_radius, 500, nothing)

    while True:
        hL = cv2.getTrackbarPos("H low", "Calibration")
        sL = cv2.getTrackbarPos("S low", "Calibration")
        vL = cv2.getTrackbarPos("V low", "Calibration")
        hH = cv2.getTrackbarPos("H high", "Calibration")
        sH = cv2.getTrackbarPos("S high", "Calibration")
        vH = cv2.getTrackbarPos("V high", "Calibration")
        rB = cv2.getTrackbarPos("Border radius", "Calibration")

        hsv_lower = np.array([hL, sL, vL])
        hsv_upper = np.array([hH, sH, vH])
        border_radius = rB

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q') or key == ord('c'):
            break

    cv2.destroyWindow("Calibration")

if __name__ == "__main__":
    serial_port = "COM8"  # or your port, e.g. "COM3"
    baud_rate = 115200

    # Open serial (silent on failure)
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)
    except Exception:
        ser = None

    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        cleanup(cap, ser)
        sys.exit(1)

    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # keep last known ball coords (or None)
        ball_coords = None
        # timing for 20 ms send interval
        last_send_time = 0.0
        send_interval = 0.02  # 20 milliseconds

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            res = find_orange_ball_center(frame, min_r=50, max_r=70)
            if res is not None:
                x, y, r = res
                ball_coords = (int(x), int(y))

                # draw detection
                cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
                cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), 3)

                # send as "x,y" every 20 ms
                current_time = time.time()
                if ser is not None and getattr(ser, "is_open", False) and (current_time - last_send_time) >= send_interval:
                    data_str = f"{int(x)},{int(y)}"
                    try:
                        ser.write(data_str.encode('ascii') + b'\n')
                        last_send_time = current_time
                    except Exception:
                        try:
                            ser.close()
                        except Exception:
                            pass
                        ser = None
            else:
                ball_coords = None

            # Draw constant circle with center (320,240) and adjustable radius
            try:
                cv2.circle(frame, (320, 240), border_radius, (255, 0, 0), 2)
            except Exception:
                pass

            # Draw red center point at (320,240)
            try:
                cv2.circle(frame, (320, 240), 3, (0, 0, 255), -1)
            except Exception:
                pass

            # Show STM connection status overlay and ball pixel coordinates
            connected = (ser is not None and getattr(ser, "is_open", False))
            status_text = "STM: Connected" if connected else "STM: Disconnected"
            if ball_coords is not None:
                ball_text = f"Ball: {ball_coords[0]},{ball_coords[1]}"
            else:
                ball_text = "Ball: Not detected"

            # prepare stacked text (two lines)
            lines = [status_text, ball_text]
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            thickness = 2
            padding_x = 6
            padding_y = 6
            line_spacing = 8  # px between lines

            # compute widths and heights for both lines
            sizes = [cv2.getTextSize(ln, font, font_scale, thickness)[0] for ln in lines]
            widths = [s[0] for s in sizes]
            heights = [s[1] for s in sizes]
            max_width = max(widths)
            total_height = sum(heights) + (len(lines) - 1) * line_spacing

            # top-left corner for rectangle background
            txt_pos = (10, 30)  # baseline of first line
            rect_x1 = txt_pos[0] - padding_x
            rect_y2 = txt_pos[1] + padding_y
            rect_y1 = txt_pos[1] - heights[0] - padding_y
            if len(lines) > 1:
                rect_y2 = rect_y2 + (total_height - heights[0])
            rect_x2 = txt_pos[0] + max_width + padding_x

            cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), -1)

            y_offset = txt_pos[1]
            for i, ln in enumerate(lines):
                if i == 0:
                    color = (0, 255, 0) if connected else (0, 0, 255)
                else:
                    color = (255, 255, 255)
                cv2.putText(frame, ln, (txt_pos[0], y_offset), font, font_scale, color, thickness, cv2.LINE_AA)
                y_offset += heights[i] + line_spacing

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # Press 'c' to calibrate
            if key == ord('c'):
                calibrate()
            # If user presses 'q' or ESC, break
            if key == ord('q') or key == 27:
                break

    except Exception:
        pass

    finally:
        cleanup(cap, ser)
