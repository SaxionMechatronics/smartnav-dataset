import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# Attention: adjust this topic according to your sensor
PCL_TOPIC = "/lidar_points"

TIME_CANDS = ["t", "time", "timestamp", "ts", "stamp", "time_stamp", "offset_time"]

class PCLCheck(Node):
    def __init__(self):
        super().__init__("pcl_check")
        self.sub = self.create_subscription(PointCloud2, PCL_TOPIC, self.cb, 10)
        self.get_logger().info("Listening on " + PCL_TOPIC)

    def cb(self, msg: PointCloud2):
        names = [f.name for f in msg.fields]
        self.get_logger().info("Fields: " + ", ".join(names))

        time_field = next((n for n in names if n.lower() in TIME_CANDS), None)
        ring_field = next((n for n in names if n.lower() == "ring"), None)
        if not time_field:
            self.get_logger().warn("No per-point time field found (tried: " + ", ".join(TIME_CANDS) + ")")
            return

        read_fields = [time_field] + ([ring_field] if ring_field else [])
        gmin = gmax = None
        ring_minmax = {}

        for p in pc2.read_points(msg, field_names=read_fields, skip_nans=False):
            t = p[0]
            
            if t is None:
                continue
            try:
                t = float(t)
            except Exception:
                continue

            gmin = t if gmin is None else min(gmin, t)
            gmax = t if gmax is None else max(gmax, t)

            if ring_field:
                r = int(p[1])
                mn, mx = ring_minmax.get(r, (None, None))
                mn = t if mn is None else min(mn, t)
                mx = t if mx is None else max(mx, t)
                ring_minmax[r] = (mn, mx)

        self.get_logger().info(f"has rings: {'ring' in names}")
        self.get_logger().info(f"per-point time-stamps: {not (gmin==gmax)}")
        if ring_field and ring_minmax:
            const_rings = 0 < sum(1 for mn, mx in ring_minmax.values() if mn == mx)
            self.get_logger().info(f"per-ring time-stamps: {not const_rings}")

        self.destroy_subscription(self.sub)  
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(PCLCheck())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
