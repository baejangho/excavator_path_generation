import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def check_camera_connected():
    try:
        # Create a context object. This object owns the handles to all connected realsense devices
        context = rs.context()
        # Get a list of all connected devices
        devices = context.query_devices()
        if len(devices) == 0:
            print("No RealSense device connected.")
            return False
        else:
            for device in devices:
                print(f"Device connected: {device.get_info(rs.camera_info.name)}")
            return True
    except Exception as e:
        print(f"Error: {e}")
        return False

def capture_point_cloud():
    try:
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        pipeline.start(config)

        # Skip the first frames to give the Auto-Exposure time to adjust
        for _ in range(30):
            pipeline.wait_for_frames()

        # Get frameset of depth
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            print("No frame received.")
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert depth image to a point cloud
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)

        # Convert to Open3D point cloud
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3) # xyz
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(vtx)

        # Visualize point cloud
        o3d.visualization.draw_geometries([point_cloud])

        # Stop streaming
        pipeline.stop()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if check_camera_connected():
        capture_point_cloud()
    else:
        print("Please connect a RealSense camera and try again.")
