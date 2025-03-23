# subscribe_bag.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

# subscribe_bag.py
import concurrent.futures
import threading
from queue import Queue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.colors as mcolors
from message_filters import ApproximateTimeSynchronizer, Subscriber
import matplotlib.pyplot as plt  # 新增导入

class BagSubscriber(Node):
    def __init__(self):
        super().__init__('bag_subscriber')
                # 用于异步处理的线程安全队列
        self.data_queue = Queue()
        self.processed_queue = Queue()
        # 创建激光雷达数据缓存
        self.latest_scan = []
        self.latest_odom = []
        self.params = {'ang_rng': 180.0, 'ang_res': 0.5, 'unit': 100.0}
        self.grid_size = 0.2
        
        # 初始化订阅器（带时间同步）
        scan_sub = Subscriber(self, LaserScan, '/scan')
        odom_sub = Subscriber(self, Odometry, '/trajectory')


        # 初始化线程池（建议2~4个线程）
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)

        # 定时器：处理数据（可以设定较高频率）
        self.create_timer(0.5, self.check_and_process)

        # 定时器：更新图像（在主线程中刷新，避免Matplotlib线程问题）
        self.create_timer(2, self.update_plot)

        # 时间同步器（时间窗口1ms）
        self.ts = ApproximateTimeSynchronizer(
            [scan_sub, odom_sub],
            queue_size=100,
            slop=0.001
        )
        self.ts.registerCallback(self.sync_callback)
        
        # 初始化可视化参数  # 新增参数配置
        self.plot_config = {
            'point_cloud': {
                'figsize': (24, 36),
                'point_color': 'lime',
                'bg_color': 'black',
                'dpi': 100,
            },
            'grid_map': {
                'cmap': plt.cm.binary,
                'hit_threshold': 20,
                'x_range': [-10, 50],
                'y_range': [-10, 50],
                'grid_color': 'k',
                'grid_width': 0.01
            }
        }
        # 初始化matplotlib
        plt.ion()
        self.fig = plt.figure(figsize=self.plot_config['point_cloud']['figsize']) 
        gs = self.fig.add_gridspec(nrows=1, ncols=2, width_ratios=[1, 2] ) # 点云图:栅格图 = 1:2
        # 创建子图
        self.ax1 = self.fig.add_subplot(gs[0])  # 点云图占1/3宽度
        self.ax2 = self.fig.add_subplot(gs[1])  # 栅格图占2/3宽度
        
        # 预生成网格边缘坐标
        self.x_edges = np.arange(self.plot_config['grid_map']['x_range'][0], self.plot_config['grid_map']['x_range'][1] + self.grid_size, 
            self.grid_size )
        self.y_edges = np.arange(self.plot_config['grid_map']['y_range'][0],
            self.plot_config['grid_map']['y_range'][1] + self.grid_size,self.grid_size)

                # 绘制栅格
        mesh = self.ax2.pcolormesh(
            self.x_edges,
            self.y_edges,
            np.zeros((len(self.y_edges)-1, len(self.x_edges)-1),),
            cmap=self.plot_config['grid_map']['cmap'],edgecolors='gray',  linewidth=0.01,    shading='flat',   )
        # 配置栅格图显示
        self.ax2.set_xlim(self.plot_config['grid_map']['x_range'])
        self.ax2.set_ylim(self.plot_config['grid_map']['y_range'])
        
        plt.show()
        self.iter = 0
            # 定时处理数据（10Hz）
        # self.create_timer(1, self.lidar_to_world)

    def sync_callback(self, scan_msg, odom_msg):
        # 将收到的数据存入队列
        timestamp = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        scan_data = {
            'timestamp': timestamp,
            'ranges': np.array(scan_msg.ranges)
        }
        # print(np.array(scan_msg.ranges))
        # 注意：在实际使用时需要确保数据匹配，可能需要用更健壮的数据同步策略
        self.data_queue.put({
            'scan': scan_data,
            'odom': {
                'timestamp': odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9,
                'x': odom_msg.twist.twist.linear.x,
                'y': odom_msg.twist.twist.linear.y,
                'ang.z': odom_msg.twist.twist.angular.z,
            }
        })

    def check_and_process(self):
        """检查队列中是否有数据，如果有则异步处理"""
        if not self.data_queue.empty():
            # 可以批量处理，也可以一条条处理
            data = []
            while not self.data_queue.empty():
                data.append(self.data_queue.get())
            # 将数据处理任务提交到线程池中
            self.pool.submit(self.lidar_to_world, data)


    def lidar_to_world(self, data_batch):
        """在后台线程中处理激光点转换"""
        world_points = []
        for data in data_batch:
            scan = data['scan']
            pose = data['odom']
            ranges = scan['ranges']
            # 角度序列（根据激光雷达分辨率生成）
            angles = np.radians(np.arange(len(ranges)) * self.params['ang_res'])
            x_lidar = ranges / self.params['unit'] * np.cos(angles)
            y_lidar = ranges / self.params['unit'] * np.sin(angles)
            theta = pose['ang.z']
            print(x_lidar)
            rot_matrix = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta),  np.cos(theta)]
            ])
            points = np.vstack([x_lidar, y_lidar]).T
            rotated = np.dot(points, rot_matrix)
            world_x = rotated[:, 0] + pose['x']
            world_y = rotated[:, 1] + pose['y']
            world_points.extend(zip(world_x, world_y))
        # 处理后的结果保存到另一个队列，待主线程绘图调用
        if world_points:
            world_points_arr = np.array(world_points)
            grid_map, x_edges, y_edges = self.create_grid_map(world_points_arr, grid_size=self.grid_size)
            # 将处理结果推入队列
            self.processed_queue.put({
                'world_points': world_points_arr,
                'grid_map': grid_map,
                'x_edges': x_edges,
                'y_edges': y_edges
            })

    def update_plot(self):
        """在主线程中更新图像"""
        if self.processed_queue.empty():
            return
        # 获取最新处理结果
        result = self.processed_queue.get()
        self.world_points = result['world_points']
        self.grid_map = result['grid_map']
        self.x_edges = result['x_edges']
        self.y_edges = result['y_edges']
        self.plot_data()

    def create_grid_map(self, points: np.array, grid_size=0.1):
        """生成带坐标信息的栅格地图"""
        x_min, x_max = points[:,0].min(), points[:,0].max()
        y_min, y_max = points[:,1].min(), points[:,1].max()
        
        x_edges = np.arange(x_min, x_max + grid_size, grid_size)
        y_edges = np.arange(y_min, y_max + grid_size, grid_size)
        
        grid, _, _ = np.histogram2d(
            points[:,0], points[:,1],
            bins=[x_edges, y_edges]
        )
        return grid.T, x_edges, y_edges

    def plot_data(self):
        if not hasattr(self, 'world_points') or self.world_points.size == 0:
            return

        # ================= 点云图绘制 =================
        self.iter += self.world_points.shape[0]
        print(self.iter,self.iter/361)
        self.ax1.scatter(
            self.world_points[:,0], 
            self.world_points[:,1],s=1,c=self.plot_config['point_cloud']['point_color'],edgecolors='none')
        # 配置点云图
        self.ax1.set_facecolor(self.plot_config['point_cloud']['bg_color'])
        self.ax1.set_title("Real-time Point Cloud", fontsize=12, pad=10, color='white')
        
        # ================= 栅格图绘制 =================
        # 应用阈值处理
        grid_data = np.where(self.grid_map < self.plot_config['grid_map']['hit_threshold'], 0, self.grid_map)
        
        # 创建颜色标准化器
        norm = mcolors.Normalize(vmin=0, 
                               vmax=self.plot_config['grid_map']['hit_threshold'])
        
        # 绘制栅格
        mesh = self.ax2.pcolormesh(
            self.x_edges,
            self.y_edges,
            grid_data,
            cmap=self.plot_config['grid_map']['cmap'],norm=norm,edgecolors='gray',  # 临时使用醒目颜色调试
        )
        # 配置栅格图
        self.ax2.set_xlim(self.plot_config['grid_map']['x_range'])
        self.ax2.set_ylim(self.plot_config['grid_map']['y_range'])
        self.ax2.set_title(f"Occupancy Grid Map (Threshold={self.plot_config['grid_map']['hit_threshold']})", 
                          fontsize=12,
                          pad=10)
        self.ax2.set_xlabel("X Coordinate (meters)", 
                           labelpad=8,
                           fontsize=9)
        self.ax2.set_ylabel("Y Coordinate (meters)", 
                           labelpad=8,
                           fontsize=9)
        
        # 动态刷新
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = BagSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()