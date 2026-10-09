#![forbid(unsafe_code)]

mod console_view;
mod graph_view;
mod node_wizard;
mod text_input;

use std::path::PathBuf;

use anyhow::{Context as _, Result, bail};
use gpui::{App, AppContext as _, Bounds, TitlebarOptions, WindowBounds, WindowOptions, px, size};
use graph_view::GraphView;
use ros_studio_app::{
    ExternalEditor,
    preferences::{Preferences, preferences_path},
};

fn main() -> Result<()> {
    let mut root = None;
    let mut diagnostics = false;
    let mut editor = std::env::var("ROS_STUDIO_EDITOR")
        .ok()
        .map(|name| ExternalEditor::parse(&name))
        .transpose()?;
    let mut arguments = std::env::args_os().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.to_str() {
            Some("--help" | "-h") => {
                println!(
                    "ROS 2 Studio standalone\n\nUsage: ros2-studio [--diagnostics] [--editor system|code|codium|zed|clion] [PROJECT_FOLDER]\n\n--diagnostics: log renderer and GPU information to stderr"
                );
                return Ok(());
            }
            Some("--diagnostics") => diagnostics = true,
            Some("--editor") => {
                let name = arguments
                    .next()
                    .context("--editor requires an editor name")?;
                editor = Some(ExternalEditor::parse(
                    name.to_str().context("Editor name is not UTF-8")?,
                )?);
            }
            Some("--") => {
                for argument in arguments.by_ref() {
                    if root.replace(PathBuf::from(argument)).is_some() {
                        bail!("Choose only one project folder");
                    }
                }
            }
            Some(option) if option.starts_with('-') => {
                bail!("Unknown option {option:?}. Use --help")
            }
            _ => {
                if root.replace(PathBuf::from(argument)).is_some() {
                    bail!("Choose only one project folder");
                }
            }
        }
    }

    #[cfg(target_os = "linux")]
    if gpui::guess_compositor() == "Headless" {
        bail!(
            "No graphical display detected. Launch ROS 2 Studio from your desktop session (DISPLAY or WAYLAND_DISPLAY), without ZED_HEADLESS"
        );
    }
    simplelog::WriteLogger::init(
        if diagnostics {
            simplelog::LevelFilter::Info
        } else {
            simplelog::LevelFilter::Warn
        },
        simplelog::Config::default(),
        std::io::stderr(),
    )
    .context("Cannot initialize application logging")?;
    let settings = preferences_path()
        .and_then(|path| Preferences::load(&path).map(|preferences| (path, preferences)));
    let (settings_path, mut preferences, initial_error) = match settings {
        Ok((path, preferences)) => (Some(path), preferences, None),
        Err(error) => (
            None,
            Preferences::default(),
            Some(format!("{error:#}. Preferences will not be overwritten.")),
        ),
    };
    if let Some(editor) = editor {
        preferences.editor = editor;
    }
    if std::env::var_os("ROS_STUDIO_RUNTIME").is_some() {
        preferences.environment =
            ros_studio_app::runtime_environment::RuntimePreference::from_environment()?;
    }
    gpui_platform::application().run(move |cx: &mut App| {
        cx.set_app_identity("dev.ros2studio.Standalone", "ROS 2 Studio");
        cx.on_window_closed(|cx, _| {
            if cx.windows().is_empty() {
                cx.quit();
            }
        })
        .detach();
        let bounds = Bounds::centered(None, size(px(1280.0), px(800.0)), cx);
        let result = cx.open_window(
            WindowOptions {
                window_bounds: Some(WindowBounds::Windowed(bounds)),
                titlebar: Some(TitlebarOptions {
                    title: Some("ROS 2 Studio".into()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            move |window, cx| {
                if let Some(gpu) = window.gpu_specs() {
                    if diagnostics || gpu.is_software_emulated {
                        eprintln!("ROS 2 Studio GPU: {} | driver: {} | software rendering: {}", gpu.device_name, gpu.driver_name, gpu.is_software_emulated);
                    }
                    if gpu.is_software_emulated {
                        eprintln!("Warning: the interface is rendered on the CPU, not accelerated by a hardware GPU.");
                    }
                }
                let view = cx
                    .new(|cx| GraphView::new(root, preferences, settings_path, initial_error, cx));
                let focus = view.read(cx).focus.clone();
                window.focus(&focus, cx);
                let weak = view.downgrade();
                window.on_window_should_close(cx, move |window, cx| {
                    match weak.update(cx, |view, cx| view.prepare_close(window, cx)) {
                        Ok(close) => close,
                        Err(error) => {
                            eprintln!("Cannot prepare window shutdown: {error:#}");
                            true
                        }
                    }
                });
                view
            },
        );
        if let Err(error) = result {
            eprintln!("Cannot open ROS 2 Studio: {error:#}");
            cx.quit();
        } else {
            cx.activate(true);
        }
    });
    Ok(())
}
