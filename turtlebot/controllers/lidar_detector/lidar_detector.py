from controller import Robot, Lidar
import math

class Detection:
    def __init__(self):
        # Position in 2D Cartesian coordinates
        self.x = 0.0
        self.y = 0.0
        # Polar coordinates 
        self.r = 0.0          # Distance from robot
        self.theta = 0.0      # Angle relative to robot front
        # Estimated motion 
        self.vx = 0.0         # Velocity in x-direction
        self.vy = 0.0         # Velocity in y-direction 
        self.speed = 0.0      # Total speed 
        self.direction = 0.0  # Velocity direction
        self.radius = 0.0     
        self.num_points = 0   
        # Tracking number of frames we have seen this same object
        self.age = 0
        
# One beam measurement
class LidarPoint:
    def __init__(self, x, y, r, theta, index):
        self.x = x            
        self.y = y            
        self.r = r            
        self.theta = theta    
        self.index = index    

# TRACK OBJECT
class Track:
    def __init__(self, det: Detection):
        self.det = det        # Latest detection data
        self.matched = False  

""" MAIN DETECTION MODULE
 This class handles:
   - Reading LiDAR scans
   - Converting to (x, y) points
   - Clustering points into objects
   - Tracking objects across frames
   - Computing velocity
   - Outputting clean Detection objects for other modules """

class LidarDetector:
    def __init__(self, lidar: Lidar):
        self.lidar = lidar
        # LiDAR properties from Webots
        self.horizontal_res = int(lidar.getHorizontalResolution())
        self.fov = lidar.getFov()
        self.max_range = lidar.getMaxRange()
        # Data containers
        self.points = []              # List of raw LidarPoint for current frame
        self.clusters = []            # Grouped points -> one cluster = one object
        self.tracks = []              # Objects we are tracking across frames
        self.current_detections = []  # Final processed detections per frame

        # MAIN UPDATE FUNCTION
        # dt = time difference from last frame 
    def update(self, dt: float):
        # Clear previous frame data
        self.points = []
        self.clusters = []
        self.current_detections = []
        self._read_scan()        # Read and convert LiDAR scan
        self._cluster_points()   # Cluster nearby points
        self._update_tracks(dt)

        # Debug print (enable if needed)
        """
        print("=== DEBUG: RAW DETECTIONS THIS FRAME ===")
        print("Detections this frame:", len(self.current_detections))
        for d in self.current_detections:
            print(
                f"x={d.x:.2f}, y={d.y:.2f}, r={d.r:.2f}, theta={d.theta:.2f}, "
                f"vx={d.vx:.2f}, vy={d.vy:.2f}, speed={d.speed:.2f}, "
                f"dir={d.direction:.2f}, radius={d.radius:.2f}, "
                f"points={d.num_points}, age={d.age}"
            )
        print("=========================================\n")
        """

    # Allows outside modules to access detections      
    def get_detections(self):
        return self.current_detections

      # Read LiDAR and convert to (x, y) coordinates
    def _read_scan(self):
        ranges = self.lidar.getRangeImage()
        if ranges is None:
            return

        for i in range(self.horizontal_res):
            r = ranges[i]

            # Skip invalid readings
            if math.isnan(r) or r <= 0.01 or r >= self.max_range:
                continue

            # Convert beam index to angle
            angle = -self.fov / 2.0 + (self.fov * i) / (self.horizontal_res - 1)

            # Convert polar → Cartesian
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            self.points.append(LidarPoint(x, y, r, angle, i))
    
    # Cluster nearby points into objects
    def _cluster_points(self):
        self.clusters = []
        if not self.points:
            return

        # Sort left → right according to beam index
        self.points.sort(key=lambda p: p.index)

        MAX_GAP = 0.3  # Max distance between two points to be considered same object

        current_cluster = [self.points[0]]
        for i in range(1, len(self.points)):
            prev = self.points[i - 1]
            cur = self.points[i]

            # Euclidean distance
            gap = math.dist((prev.x, prev.y), (cur.x, cur.y))

            if gap > MAX_GAP:
                # End of current cluster
                if len(current_cluster) >= 2:
                    self.clusters.append(current_cluster)
                current_cluster = []

            current_cluster.append(cur)
            
        # Add last cluster
        if len(current_cluster) >= 2:
            self.clusters.append(current_cluster)

    # Track clusters & compute velocity
    def _update_tracks(self, dt: float):
       
        for t in self.tracks:
            t.matched = False

        new_tracks = []

        for cluster in self.clusters:
            det = self._compute_cluster_detection(cluster)
            best_index = -1
            best_dist = float("inf")

            for i, t in enumerate(self.tracks):
                if t.matched:
                    continue

                dist = math.dist((det.x, det.y), (t.det.x, t.det.y))
                if dist < best_dist:
                    best_dist = dist
                    best_index = i

            MAX_ASSOC_DIST = 0.7  # Must be close to count as same object
            
            if best_index >= 0 and best_dist < MAX_ASSOC_DIST and dt > 0:
                prev = self.tracks[best_index]

                dx = det.x - prev.det.x
                dy = det.y - prev.det.y

                det.vx = dx / dt
                det.vy = dy / dt
                det.speed = math.sqrt(det.vx**2 + det.vy**2)
                det.direction = math.atan2(det.vy, det.vx)
                det.age = prev.det.age + 1
                       
                # LOW-PASS FILTER to make values stable
                ALPHA = 0.2   # smoothing factor (0.1 = smoother, 0.5 = faster)
                det.x = ALPHA * det.x + (1 - ALPHA) * prev.det.x
                det.y = ALPHA * det.y + (1 - ALPHA) * prev.det.y
                det.radius = ALPHA * det.radius + (1 - ALPHA) * prev.det.radius
                det.r = math.sqrt(det.x**2 + det.y**2)
                det.theta = math.atan2(det.y, det.x)
                               
                # Treat tiny speeds as static noise
                SPEED_EPS = 0.05
                if det.speed < SPEED_EPS:
                    det.vx = 0.0
                    det.vy = 0.0
                    det.speed = 0.0
                    det.direction = 0.0
                prev.det = det
                prev.matched = True
                new_tracks.append(prev)
  
            else:                   
                det.age = 1
                new_track = Track(det)
                new_track.matched = True
                new_tracks.append(new_track)

        self.tracks = new_tracks
        self.current_detections = [t.det for t in self.tracks]

    # Convert one cluster into a Detection object   
    def _compute_cluster_detection(self, cluster):
        d = Detection()
        n = len(cluster)
        
        # centroid
        d.x = sum(p.x for p in cluster) / n
        d.y = sum(p.y for p in cluster) / n
        
        # Convert to polar
        d.r = math.sqrt(d.x**2 + d.y**2)
        d.theta = math.atan2(d.y, d.x)
        
        # Structural properties
        d.num_points = n
        d.radius = max(math.dist((p.x, p.y), (d.x, d.y)) for p in cluster)

        return d

def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    if time_step <= 0:
        time_step = 64  # Default fallback

    # Get LiDAR and enable it
    lidar = robot.getDevice("LDS-01")
    lidar.enable(time_step)
    lidar.enablePointCloud()
    detector = LidarDetector(lidar)
    last_time = robot.getTime()
      
    # Main loop: Webots simulation
    while robot.step(time_step) != -1:
        current_time = robot.getTime()
        dt = current_time - last_time
        
        # Safety fallback if dt is invalid
        if dt <= 0:
            dt = time_step / 1000.0
            
        last_time = current_time
        detector.update(dt)
        
        # Access detections using:        
        # detections = detector.get_detections()


if __name__ == "__main__":
    main()
