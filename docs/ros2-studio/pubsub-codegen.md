# Pub/sub code generation (M13)

The New ROS Node wizard lists typed topics detected in the open project's Design graph. Search by topic name or message type, then select Subscribe, Publish, or both for any listed topic. Each topic shows its known publishers. The preview displays the complete new Rust source and the resulting package `Cargo.toml` before either file is written.

For selected interfaces, ROS 2 Studio generates `rclrs` 0.7 publisher/subscription calls and adds `ros-env = "0.2"` when the package does not already declare it. The generated source imports message modules from `ros_env`, which requires those interfaces to exist in the sourced ROS 2 environment at build time. The wizard accepts only message types with a complete `package::msg::Type` or `package/msg/Type` name; unresolved or ambiguous types are not offered. Typed interface generation is unavailable for `rclrs` 0.6 because its message runtime is incompatible with `ros-env` 0.2. Creating a node without interfaces remains available with both `rclrs` versions.

The generated source contains `ros-studio:interfaces:start` and `ros-studio:interfaces:end` comments to identify its initial interface block. This milestone creates a new binary only; it does not modify existing source files or implement later editing of that block. Existing files and changed manifests retain the M12 overwrite protections.
