fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default();
    let mut node = rclrs::create_node(&context, "navigation")?;

    let _subscription = node.create_subscription::<vision_msgs::msg::Detection2DArray, _>(
        "/detections",
        rclrs::QOS_PROFILE_DEFAULT,
        move |_detections| {},
    )?;

    let _publisher = node.create_publisher::<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclrs::QOS_PROFILE_DEFAULT,
    )?;

    rclrs::spin(&node)?;
    Ok(())
}
