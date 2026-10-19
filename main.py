import serial
import cv2
import numpy as np

# Configure your COM port and Baudrate (must match the STM32 configuration, e.g., 9600)
try:
    ser = serial.Serial('COM12', 9600, timeout=1)
except Exception as e:
    print("❌ Could not open Serial Port. Ensure the STM32 is connected and the port is not in use by another program.")

# --- Function: Calculate the average slope and intercept ---
def average_slope_intercept(img, lines):
    lane_lines = []
    if lines is None:
        return lane_lines

    height, width = img.shape[:2]
    left_fit = []
    right_fit = []

    # Threshold for the slope to distinguish left and right lines
    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line in lines:
        try:
            x1, y1, x2, y2 = line.reshape(4)

            if x1 == x2:
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
        except:
            pass

    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(make_points(img, left_fit_average))

    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_points(img, right_fit_average))

    return lane_lines

# --- Function: Generate points from slope and intercept ---
def make_points(img, line):
    height, width = img.shape[:2]
    slope, intercept = line
    y1 = int(height)
    y2 = int(height * 0.6)

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]

# --- Function: Calculate the lane offset ---
def calculate_lane_offset(img, lines):
    height, width = img.shape[:2]
    img_center = width / 2.0
    lane_center = img_center

    if len(lines) == 2:
        left_line = lines[0][0]
        right_line = lines[1][0]
        y_val = height * 0.7
        x_left = (y_val - (np.polyfit((left_line[0], left_line[2]), (left_line[1], left_line[3]), 1))[1]) / \
                 (np.polyfit((left_line[0], left_line[2]), (left_line[1], left_line[3]), 1))[0]
        x_right = (y_val - (np.polyfit((right_line[0], right_line[2]), (right_line[1], right_line[3]), 1))[1]) / \
                  (np.polyfit((right_line[0], right_line[2]), (right_line[1], right_line[3]), 1))[0]
        lane_center = (x_left + x_right) / 2.0

    offset = lane_center - img_center
    return int(offset)

# --- Function: Apply Region of Interest (ROI) mask ---
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

# --- Function: Draw lane lines ---
def draw_lines(img, lines):
    line_image = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            try:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
            except:
                pass
    return line_image

# --- Global variables for the PD Controller ---
prev_offset = 0
Kp = 0.4
Kd = 0.2

def calculate_steering_angle(offset):
    global prev_offset
    derivative = offset - prev_offset
    steering_angle = (Kp * offset) + (Kd * derivative)
    prev_offset = offset
    # Constrain the steering angle between -45 and 45 degrees
    steering_angle = max(min(steering_angle, 45), -45)
    return int(steering_angle)

# --- Main processing function ---
def process_frame(frame):
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    roi_vertices = np.array([
        [(width * 0.1, height), (width * 0.9, height), (width // 2, int(height * 0.6))]
    ], np.int32)
    cropped_edges = region_of_interest(edges, roi_vertices)

    lines = cv2.HoughLinesP(cropped_edges, rho=2, theta=np.pi / 180, threshold=50, minLineLength=40, maxLineGap=150)

    # 1. Calculations
    averaged_lines = average_slope_intercept(frame, lines)
    line_image = draw_lines(frame, averaged_lines)
    offset = calculate_lane_offset(frame, averaged_lines)
    steering_angle = calculate_steering_angle(offset)

    # Send the steering angle to the STM32 board
    try:
       command = f"ANGLE:{steering_angle}\n"
       ser.write(command.encode('utf-8'))
    except Exception as e:
        print("❌ Error with Serial COM12:", e)

    # 2. CREATE combo_image (VERY IMPORTANT, must be done before cv2.putText)
    combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    # 3. Draw text and UI elements on combo_image
    cv2.line(combo_image, (int(width / 2), 0), (int(width / 2), height), (0, 0, 255), 2)
    cv2.putText(combo_image, f"Offset: {offset}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4, cv2.LINE_AA)
    cv2.putText(combo_image, f"Steering Angle: {steering_angle} deg", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3, cv2.LINE_AA)

    return combo_image

# --- MAIN LOOP ---
# WARNING: Ensure the video file path is correct on your computer
cap = cv2.VideoCapture("test_video.mp4")

if not cap.isOpened():
    print("❌ ERROR: OpenCV could not find the video! Check the file path and name.")
else:
    print("✅ Video loaded successfully, the window will open now...")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame = process_frame(frame)
    cv2.imshow("LKA - Lane Detection & PD Control", processed_frame)

    # Delay to control video playback speed (200ms = slow motion)
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()