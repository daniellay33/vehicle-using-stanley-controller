import carla
import math
import numpy as np
import cv2
import random 
def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class VisionStanleyController:
    def __init__(self, k=0.5, wheelbase=2.5):
        self.k = k                  
        self.wheelbase = wheelbase  
        self.prev_steer = 0.0       

    def compute_errors_from_image(self, cv_image):
        h, w, _ = cv_image.shape
        
    
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 180], dtype=np.uint8)
        upper_white = np.array([255, 50, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        edges = cv2.Canny(mask, 50, 150)
        
       
        roi_mask = np.zeros_like(edges)
        polygon = np.array([[(0, h), (int(w*0.2), int(h*0.55)), (int(w*0.8), int(h*0.55)), (w, h)]], dtype=np.int32)
        cv2.fillPoly(roi_mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        
        lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=40, minLineLength=30, maxLineGap=100)
        
        left_lines = []
        right_lines = []
        
        debug_img = cv2.cvtColor(masked_edges, cv2.COLOR_GRAY2BGR)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 == x2: continue 
                slope = (y2 - y1) / (x2 - x1)
                
                if slope < -0.3:
                    left_lines.append(line)
                    cv2.line(debug_img, (x1, y1), (x2, y2), (255, 0, 0), 2) 
                elif slope > 0.3:
                    right_lines.append(line)
                    cv2.line(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2) 

        image_center_x = w / 2.0
        
        
        left_x_bottom = 0
        right_x_bottom = w
        
        if len(left_lines) > 0:
            left_x_bottom = np.mean([line[0][0] + (h - line[0][1]) / ((line[0][3] - line[0][1]) / (line[0][2] - line[0][0]) + 1e-6) for line in left_lines])
        
        if len(right_lines) > 0:
            right_x_bottom = np.mean([line[0][0] + (h - line[0][1]) / ((line[0][3] - line[0][1]) / (line[0][2] - line[0][0]) + 1e-6) for line in right_lines])

        if len(left_lines) > 0 and len(right_lines) > 0:
            lane_center_bottom = (left_x_bottom + right_x_bottom) / 2.0
        elif len(left_lines) > 0:
            lane_center_bottom = left_x_bottom + (w * 0.4) 
        elif len(right_lines) > 0:
            lane_center_bottom = right_x_bottom - (w * 0.4) 
        else:
            lane_center_bottom = image_center_x 

        cv2.circle(debug_img, (int(lane_center_bottom), h-10), 10, (0, 255, 0), -1)

       
        pixel_error = lane_center_bottom - image_center_x
        e = pixel_error * 0.005 
        
        psi = np.arctan2(pixel_error, h * 0.8) 
        
        return e, psi, debug_img

    def run_step(self, vehicle, cv_image):
        velocity = vehicle.get_velocity()
        v = math.sqrt(velocity.x**2 + velocity.y**2)
        v = max(v, 1.0) 
        
        e, psi, processed_img = self.compute_errors_from_image(cv_image)
        
        steering_angle = psi + math.atan2(self.k * e, v)
        
        max_steer = 1.22 
        raw_steer = np.clip(steering_angle / max_steer, -1.0, 1.0)
        
        smoothed_steer = 0.5 * self.prev_steer + 0.5 * raw_steer
        self.prev_steer = smoothed_steer
        
        return smoothed_steer, processed_img


def main():
   
    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)  
    
 
    print("Loading Town04 map... this might take a few seconds. Please wait.")
    world = client.load_world('Town04')
    map = world.get_map()
    
    
    blueprint_library = world.get_blueprint_library()
    mini_bps = blueprint_library.filter('*mini*')
    if len(mini_bps) > 0:
        vehicle_bp = mini_bps[0]
    else:
        print("Mini blueprint not found, using Tesla Model 3 instead.")
        vehicle_bp = blueprint_library.filter('model3')[0]

 
    vehicle = None
    spawn_points = map.get_spawn_points()
    random.shuffle(spawn_points) 
    
    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is not None:
            print("Vehicle spawned successfully at a random point!")
            break
            
    if vehicle is None:
        raise RuntimeError("Could not spawn vehicle. Make sure the simulator isn't cluttered.")

  
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    
    camera_transform = carla.Transform(carla.Location(x=1.5, z=1.4), carla.Rotation(pitch=0.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    
    camera_data = {'image': np.zeros((600, 800, 3), dtype=np.uint8)}
    def camera_callback(image):
        array = np.copy(image.raw_data)
        camera_data['image'] = np.reshape(array, (image.height, image.width, 4))[:, :, :3]
        
    camera.listen(camera_callback)
    
    controller = VisionStanleyController(k=0.5)
    print("Vision-based Stanley Controller initialized. Starting loop...")
    
   
    spectator = world.get_spectator()
    
    try:
        while True:
            world.wait_for_tick()
            
            transform = vehicle.get_transform()
            spectator_transform = carla.Transform(
                transform.location + carla.Location(z=20.0),
                carla.Rotation(pitch=-90.0, yaw=0.0)
            )
            spectator.set_transform(spectator_transform)
            
            current_frame = camera_data['image']
            if current_frame is None or np.all(current_frame == 0):
                continue
                
            steer, processed_img = controller.run_step(vehicle, current_frame)
            
            control = carla.VehicleControl()
            control.steer = float(steer)
            control.throttle = 0.2  
            control.brake = 0.0
            vehicle.apply_control(control)
            
            cv2.imshow('Car Dashcam View', current_frame)
            cv2.imshow('Hough Lines Lane Detection', processed_img)
            
            if cv2.waitKey(1) == ord('q'):
                break
                
    finally:
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()
        print("Cleaned up successfully.")

if __name__ == '__main__':
    main()
