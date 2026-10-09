use rclrs::{CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use ros_env::{geometry_msgs, sensor_msgs};

fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("test")?;

    // ros-studio:interfaces:start
    let publisher_cmd_vel = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel")?;
    let subscriber_camera_image = node.create_subscription::<sensor_msgs::msg::Image, _>("/camera/image", move |_message| {})?;
    let _ros_interfaces = (
        publisher_cmd_vel,
        subscriber_camera_image,
    );
    // ros-studio:interfaces:end

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
