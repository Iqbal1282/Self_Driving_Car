import time 
from drivers import Car 
from vision import Vision 

def main(): 
    car = Car()
    vision = Vision 


    # States 
    STATE = "CRUISE"


    stop_timer = 0 

    try: 
        while True: 
            # get sensor data 

            frame = vision.get_frame()
            lane_angle = vision.detect_lanes(frame) 
            light_state = vision.detect_traffic_light(frame) 
            obstacle = vision.detect_obstacle(frame) 



            # debug visualization 
            #vis = vision.visualize(frame, lane_angle, light_state, obstacle)

            #cv2.imshow("Self-Driving View", vis)
            #cv2.waitKey(1)

            if obstacle: 
                STATE = "STOP_OBSTACLE"
            elif light_state == "red": 
                STATE = "STOP_LIGHT"
            elif light_state == "green" and STATE == "STOP_LIGHT":
                STATE = "CRUISE"
            elif not obstacle and light_state != "red" and STATE in ["STOP_OBSTACLE", "STOP_LIGHT"]:
                STATE = "CRUISE"

            
            # Act based on state
            if STATE == "CRUISE":
                car.steer(lane_angle)
                car.drive(0.4)  # 40% forward speed
            elif STATE == "STOP_LIGHT":
                car.stop()
                print("Red light detected - stopping")
            elif STATE == "STOP_OBSTACLE":
                car.stop()
                print("Obstacle detected - stopping")

            time.sleep(0.05)  # 20 Hz loop

    except KeyboardInterrupt:
        print("Shutting down...")
        car.stop()
        # cv2.destroyAllWindows()

if __name__ == "__main__": 
    main()
    