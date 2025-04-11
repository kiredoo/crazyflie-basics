import rclpy
from std_srvs.srv import Trigger
import yaml
from pathlib import Path


def load_cf_names_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['params']['ros__parameters']['cfs_names']

def call_trigger(node, cf_name):
    service_name = f'/{cf_name}/trigger_landing'
    client = node.create_client(Trigger, service_name)

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Waiting for service {service_name}...')

    request = Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def main():
    rclpy.init()
    node = rclpy.create_node('landing_client')

    yaml_path = Path('/home/turtlebot/Desktop/cycli_pursuit_ws2/install/cyclic_pursuit/share/cyclic_pursuit/config/cyclic.yaml')
    cfs_names = load_cf_names_from_yaml(yaml_path)

    for cf in cfs_names:
        result = call_trigger(node, cf)
        if result:
            print(f"{cf}: {result.message}")
        else:
            print(f"{cf}: Failed to receive response.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()