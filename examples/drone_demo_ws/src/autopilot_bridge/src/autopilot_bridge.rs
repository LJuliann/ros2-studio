fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default();
    let mut node = rclrs::create_node(&context, "autopilot_bridge")?;

    let _subscription = node.create_subscription::<geometry_msgs::msg::Twist, _>(
        "/cmd_vel",
        rclrs::QOS_PROFILE_DEFAULT,
        move |_command| {},
    )?;

    rclrs::spin(&node)?;
    Ok(())
}
