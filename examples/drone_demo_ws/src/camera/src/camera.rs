use rclrs::{CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use ros_env::sensor_msgs;

fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("camera")?;

    let publisher_camera_image =
        node.create_publisher::<sensor_msgs::msg::Image>("/camera/image")?;
    let _ros_interfaces = (publisher_camera_image,);

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
