import rclpy
from ImageProcessing import ImageProcessing

def main(args=None):
    rclpy.init(args=args)
    Imageprocessandfollow_node = Imageprocessandfollow()
    file_path = '/home/b123/linorobot2_ws/src/duck_python/resource/filtered_duckpath.csv'
    waypoints = read_waypoints_from_csv(file_path)
    if not waypoints:
        print("導航點加載失敗，請檢查 CSV 檔案")
        Imageprocessandfollow_node.destroy_node()
        rclpy.shutdown()
        return

    navigator = WaypointFollower(waypoints)
    executor = MultiThreadedExecutor()
    executor.add_node(Imageprocessandfollow_node.egg_navigator)  # 新增 navigator
    # executor.add_node(Imageprocessandfollow_node.egg_navigator.nav2_image_surrport)  # 新增 navigator
    executor.add_node(Imageprocessandfollow_node)
    executor.add_node(navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        Imageprocessandfollow_node.stop_processing.set()
        Imageprocessandfollow_node.decode_thread.join()
        Imageprocessandfollow_node.yolo_thread.join()
        Imageprocessandfollow_node.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()  # 或是任何你想執行的函式