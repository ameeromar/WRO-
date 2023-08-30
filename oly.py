import cv2
import numpy as np
KNOWN_OBJECT_WIDTH = 10 
FOCAL_LENGTH = 500      

def estimate_distance(object_width, focal_length, apparent_width):
    return (object_width * focal_length) / apparent_width


def simulate_cmyk_to_rgb(bgr_frame):
    cmyk_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
    simulated_rgb_frame = cv2.cvtColor(cmyk_frame, cv2.COLOR_GRAY2BGR)
    return simulated_rgb_frame

def main():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([(0, 120, 120)])
        upper_red = np.array([5, 255, 255])

        lower_green = np.array([50, 190, 44])
        upper_green = np.array([85, 255, 255])

        mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
        mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)


        frame[np.where(mask_red == 255)] = [0, 0, 255] 
        frame[np.where(mask_green == 255)] = [0, 255, 0] 
        simulated_rgb_frame = simulate_cmyk_to_rgb(frame)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > 500: 
                M = cv2.moments(contour)
                centroid_x = int(M["m10"] / M["m00"])
                
                if centroid_x < frame.shape[1] // 3:
                    location = "Left"
                elif centroid_x > 2 * frame.shape[1] // 3:
                    location = "Right"
                else:
                    location = "Center"
                distance = estimate_distance(KNOWN_OBJECT_WIDTH, FOCAL_LENGTH, area)
                distance_text = f"Distance: {distance:.2f} cm"
                cv2.putText(simulated_rgb_frame, distance_text, (centroid_x - 50, frame.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                print("Red object detected on the", location)

        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > 500:  
                M = cv2.moments(contour)
                centroid_x = int(M["m10"] / M["m00"])
                if centroid_x < frame.shape[1] // 3:
                    location1 = "Left"
                elif centroid_x > 2 * frame.shape[1] // 3:
                    location1 = "Right"
                else:
                    location1 = "Center"
                distance1 = estimate_distance(KNOWN_OBJECT_WIDTH, FOCAL_LENGTH, area)
                distance_text = f"Distance: {distance1:.2f} cm"
                cv2.putText(simulated_rgb_frame, distance_text, (centroid_x - 50, frame.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print("Green object detected on the", location1)

        cv2.imshow('Processed Frame', simulated_rgb_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()