fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default();
    let mut node = rclrs::create_node(&context, "detector")?;

    let _subscription = node.create_subscription::<sensor_msgs::msg::Image, _>(
        "/camera/image",
        rclrs::QOS_PROFILE_DEFAULT,
        move |_image| {},
    )?;

    let _publisher = node.create_publisher::<vision_msgs::msg::Detection2DArray>(
        "/detections",
        rclrs::QOS_PROFILE_DEFAULT,
    )?;

    rclrs::spin(&node)?;
    Ok(())
}
