#[allow(unused_imports, missing_docs)]
pub mod sensor_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        include!("/opt/ros2-rust-overlay/install/sensor_msgs/share/sensor_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            include!(
                "/opt/ros2-rust-overlay/install/sensor_msgs/share/sensor_msgs/rust/src/msg/rmw.rs"
            );
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        include!("/opt/ros2-rust-overlay/install/sensor_msgs/share/sensor_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            include!(
                "/opt/ros2-rust-overlay/install/sensor_msgs/share/sensor_msgs/rust/src/srv/rmw.rs"
            );
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod geometry_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::std_msgs;
        include!(
            "/opt/ros2-rust-overlay/install/geometry_msgs/share/geometry_msgs/rust/src/msg.rs"
        );
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::std_msgs;
            include!("/opt/ros2-rust-overlay/install/geometry_msgs/share/geometry_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod std_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        include!("/opt/ros2-rust-overlay/install/std_msgs/share/std_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros2-rust-overlay/install/std_msgs/share/std_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod px4_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        include!("/opt/ros2-rust-overlay/install/px4_msgs/share/px4_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros2-rust-overlay/install/px4_msgs/share/px4_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        include!("/opt/ros2-rust-overlay/install/px4_msgs/share/px4_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros2-rust-overlay/install/px4_msgs/share/px4_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod composition_interfaces {
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::rcl_interfaces;
        include!("/opt/ros/humble/share/composition_interfaces/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::rcl_interfaces;
            include!("/opt/ros/humble/share/composition_interfaces/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod statistics_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        include!("/opt/ros/humble/share/statistics_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros/humble/share/statistics_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod lifecycle_msgs {
    pub mod msg {
        include!("/opt/ros/humble/share/lifecycle_msgs/rust/src/msg.rs");
        pub mod rmw {
            include!("/opt/ros/humble/share/lifecycle_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        include!("/opt/ros/humble/share/lifecycle_msgs/rust/src/srv.rs");
        pub mod rmw {
            include!("/opt/ros/humble/share/lifecycle_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod rmw_dds_common {
    pub mod msg {
        include!("/opt/ros/humble/share/rmw_dds_common/rust/src/msg.rs");
        pub mod rmw {
            include!("/opt/ros/humble/share/rmw_dds_common/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod builtin_interfaces {
    pub mod msg {
        include!("/opt/ros/humble/share/builtin_interfaces/rust/src/msg.rs");
        pub mod rmw {
            include!("/opt/ros/humble/share/builtin_interfaces/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod nav_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/nav_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/nav_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/nav_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/nav_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod rosgraph_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::rcl_interfaces;
        include!("/opt/ros/humble/share/rosgraph_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::rcl_interfaces;
            include!("/opt/ros/humble/share/rosgraph_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod stereo_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::sensor_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/stereo_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::sensor_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/stereo_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod rcl_interfaces {
    pub mod msg {
        use crate::builtin_interfaces;
        include!("/opt/ros/humble/share/rcl_interfaces/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros/humble/share/rcl_interfaces/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        include!("/opt/ros/humble/share/rcl_interfaces/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros/humble/share/rcl_interfaces/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod visualization_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::sensor_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/visualization_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::sensor_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/visualization_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::sensor_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/visualization_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::sensor_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/visualization_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod diagnostic_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/diagnostic_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/diagnostic_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/diagnostic_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/diagnostic_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod shape_msgs {
    pub mod msg {
        use crate::geometry_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/shape_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::geometry_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/shape_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod action_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/action_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/action_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/action_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/action_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod std_srvs {
    pub mod srv {
        include!("/opt/ros/humble/share/std_srvs/rust/src/srv.rs");
        pub mod rmw {
            include!("/opt/ros/humble/share/std_srvs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod trajectory_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/trajectory_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/trajectory_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod unique_identifier_msgs {
    pub mod msg {
        include!("/opt/ros/humble/share/unique_identifier_msgs/rust/src/msg.rs");
        pub mod rmw {
            include!("/opt/ros/humble/share/unique_identifier_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod actionlib_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/actionlib_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/actionlib_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod test_msgs {
    pub mod msg {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/test_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/test_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod action {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/test_msgs/rust/src/action.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/test_msgs/rust/src/action/rmw.rs");
        }
    }
    pub mod srv {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/test_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/test_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod action_tutorials_interfaces {
    pub mod action {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/action_tutorials_interfaces/rust/src/action.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/action_tutorials_interfaces/rust/src/action/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod map_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::nav_msgs;
        use crate::sensor_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/map_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::nav_msgs;
            use crate::sensor_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/map_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::nav_msgs;
        use crate::sensor_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/map_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::nav_msgs;
            use crate::sensor_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/map_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod logging_demo {
    pub mod srv {
        include!("/opt/ros/humble/share/logging_demo/rust/src/srv.rs");
        pub mod rmw {
            include!("/opt/ros/humble/share/logging_demo/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod example_interfaces {
    pub mod msg {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/example_interfaces/rust/src/msg.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/example_interfaces/rust/src/msg/rmw.rs");
        }
    }
    pub mod action {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/example_interfaces/rust/src/action.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/example_interfaces/rust/src/action/rmw.rs");
        }
    }
    pub mod srv {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/example_interfaces/rust/src/srv.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/example_interfaces/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod pcl_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::sensor_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/pcl_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::sensor_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/pcl_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::sensor_msgs;
        use crate::std_msgs;
        include!("/opt/ros/humble/share/pcl_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::sensor_msgs;
            use crate::std_msgs;
            include!("/opt/ros/humble/share/pcl_msgs/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod pendulum_msgs {
    pub mod msg {
        use crate::builtin_interfaces;
        include!("/opt/ros/humble/share/pendulum_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros/humble/share/pendulum_msgs/rust/src/msg/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod turtlesim {
    pub mod msg {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/turtlesim/rust/src/msg.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/turtlesim/rust/src/msg/rmw.rs");
        }
    }
    pub mod action {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/turtlesim/rust/src/action.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/turtlesim/rust/src/action/rmw.rs");
        }
    }
    pub mod srv {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/turtlesim/rust/src/srv.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/turtlesim/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod rosbag2_interfaces {
    pub mod msg {
        use crate::builtin_interfaces;
        include!("/opt/ros/humble/share/rosbag2_interfaces/rust/src/msg.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros/humble/share/rosbag2_interfaces/rust/src/msg/rmw.rs");
        }
    }
    pub mod srv {
        use crate::builtin_interfaces;
        include!("/opt/ros/humble/share/rosbag2_interfaces/rust/src/srv.rs");
        pub mod rmw {
            use crate::builtin_interfaces;
            include!("/opt/ros/humble/share/rosbag2_interfaces/rust/src/srv/rmw.rs");
        }
    }
}
#[allow(unused_imports, missing_docs)]
pub mod tf2_msgs {
    pub mod msg {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/tf2_msgs/rust/src/msg.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/tf2_msgs/rust/src/msg/rmw.rs");
        }
    }
    pub mod action {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/tf2_msgs/rust/src/action.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/tf2_msgs/rust/src/action/rmw.rs");
        }
    }
    pub mod srv {
        use crate::action_msgs;
        use crate::builtin_interfaces;
        use crate::geometry_msgs;
        use crate::std_msgs;
        use crate::unique_identifier_msgs;
        include!("/opt/ros/humble/share/tf2_msgs/rust/src/srv.rs");
        pub mod rmw {
            use crate::action_msgs;
            use crate::builtin_interfaces;
            use crate::geometry_msgs;
            use crate::std_msgs;
            use crate::unique_identifier_msgs;
            include!("/opt/ros/humble/share/tf2_msgs/rust/src/srv/rmw.rs");
        }
    }
}
