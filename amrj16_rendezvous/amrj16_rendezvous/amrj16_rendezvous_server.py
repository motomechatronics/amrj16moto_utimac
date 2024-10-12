import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

class PlaneExtractionServer(Node):
    def __init__(self):
        super().__init__('plane_extraction_server')

        # Sottoscrivi al topic /pointcloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10)
        self.subscription  # per evitare warning inutilizzato

    def pointcloud_callback(self, msg):
        # Converti PointCloud2 in un array numpy
        point_cloud = self.pointcloud2_to_array(msg)
        
        # Crea un'istanza di Open3D PointCloud
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(point_cloud)

        # Segmentazione del piano utilizzando RANSAC
        plane_model, inliers = o3d_cloud.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000)
        
        [a, b, c, d] = plane_model
        self.get_logger().info(f"Equation of the plane: {a}x + {b}y + {c}z + {d} = 0")
        
        # Estrazione dei punti del piano
        plane_cloud = o3d_cloud.select_by_index(inliers)
        
        # Calcolo del centroide
        centroid = np.mean(np.asarray(plane_cloud.points), axis=0)
        self.get_logger().info(f"Centroid of the plane: x={centroid[0]}, y={centroid[1]}, z={centroid[2]}")

        # Calcolo della normale al piano
        normal = np.array([a, b, c])
        normal_unit = normal / np.linalg.norm(normal)
        self.get_logger().info(f"Normal of the plane: {normal_unit}")

        # Visualizza la nuvola di punti con il piano estratto
        o3d.visualization.draw_geometries([plane_cloud])

    def pointcloud2_to_array(self, cloud_msg):
        """Converti il messaggio PointCloud2 in un array numpy."""
        # Estrai i dati binari dal messaggio PointCloud2
        points = []
        for p in self.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points)

    def read_points(self, cloud, field_names=None, skip_nans=False):
        """Funzione per leggere i punti dal messaggio PointCloud2"""
        # Questa funzione converte il messaggio PointCloud2 in singoli punti
        fmt = '<fff'  # Formato per le coordinate (x, y, z)
        for i in range(cloud.height * cloud.width):
            offset = cloud.row_step * (i // cloud.width) + (i % cloud.width) * cloud.point_step
            x, y, z = struct.unpack_from(fmt, cloud.data, offset)
            if skip_nans and (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                continue
            yield (x, y, z)


def main(args=None):
    rclpy.init(args=args)
    plane_extraction_server = PlaneExtractionServer()

    try:
        rclpy.spin(plane_extraction_server)
    except KeyboardInterrupt:
        pass
    finally:
        plane_extraction_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
