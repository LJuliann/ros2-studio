//! Host-side project tasks. A Makefile may already launch Docker, so these must
//! not be sent to the ROS daemon inside another container.

use std::{path::PathBuf, process::Stdio, time::Duration};

use anyhow::{Context as _, Result, bail};
use async_channel::{Receiver, Sender};
use futures::{FutureExt as _, pin_mut, select};
use ros_studio_protocol::OutputStream;

use crate::{
    runtime::Execution,
    runtime_client::{ClientCommand, RuntimeMessage, forward_output},
};

pub async fn run(
    root: PathBuf,
    execution: Execution,
    commands: Receiver<ClientCommand>,
    sender: Sender<RuntimeMessage>,
) {
    if let Err(error) = run_process(root, &execution, commands, &sender).await {
        let _ = sender
            .send(RuntimeMessage::Failed {
                operation: Some(execution.operation),
                message: format!("Host task failed: {error:#}"),
            })
            .await;
    }
}

async fn run_process(
    root: PathBuf,
    execution: &Execution,
    commands: Receiver<ClientCommand>,
    sender: &Sender<RuntimeMessage>,
) -> Result<()> {
    let (program, arguments) = execution
        .command
        .split_first()
        .context("Empty host command")?;
    let mut command = std::process::Command::new(program);
    command.args(arguments).current_dir(root).env(
        "CARGO_BUILD_JOBS",
        std::env::var("CARGO_BUILD_JOBS").unwrap_or_else(|_| "1".into()),
    );
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt as _;
        command.process_group(0);
    }
    let mut command = async_process::Command::from(command);
    let mut child = command
        .kill_on_drop(true)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .with_context(|| format!("Cannot launch {program}; install make on the host if missing"))?;
    let pid = child.id();
    let id = format!("make-{pid}");
    // Keep all setup/reporting failures inside the cleanup path below.
    let result = async {
        sender
            .send(RuntimeMessage::Started {
                id: id.clone(),
                operation: execution.operation,
            })
            .await?;
        let stdout = child.stdout.take().context("Host task has no stdout")?;
        let stderr = child.stderr.take().context("Host task has no stderr")?;
        let output = futures::future::try_join(
            forward_output(stdout, OutputStream::Stdout, sender),
            forward_output(stderr, OutputStream::Stderr, sender),
        );
        let completion = async {
            let status = child.status().fuse();
            let cancelled = commands.recv().fuse();
            pin_mut!(status, cancelled);
            let finished = select! {
                status = status => status?,
                command = cancelled => match command {
                    Ok(ClientCommand::Shutdown | ClientCommand::Stop(_)) | Err(_) => {
                        signal_group(pid, false)?;
                        let grace = async_io::Timer::after(Duration::from_secs(2)).fuse();
                        pin_mut!(grace);
                        select! {
                            status = status => status?,
                            _ = grace => {
                                signal_group(pid, true)?;
                                status.await?
                            },
                        }
                    },
                    Ok(ClientCommand::Launch(_)) => bail!("Host task is already running"),
                    Ok(ClientCommand::ConfigureHealth(_)) => bail!("Health monitoring belongs to the ROS discovery session, not a host task"),
                },
            };
            // A completed group leader may leave children holding output pipes.
            // Clean up its owned group before waiting for output EOF.
            signal_group(pid, true)?;
            Ok::<_, anyhow::Error>(finished)
        };
        Ok::<_, anyhow::Error>(futures::future::try_join(output, completion).await?.1)
    }
    .await;
    let status = match result {
        Ok(status) => status,
        Err(error) => {
            let _ = signal_group(pid, true);
            let _ = child.status().await;
            return Err(error);
        }
    };
    sender
        .send(RuntimeMessage::Exited {
            id,
            success: status.success(),
            code: status.code(),
        })
        .await?;
    Ok(())
}

fn signal_group(pid: u32, force: bool) -> Result<()> {
    #[cfg(unix)]
    {
        use nix::{
            errno::Errno,
            sys::signal::{Signal, killpg},
            unistd::Pid,
        };
        let pid = i32::try_from(pid).context("Host process ID is too large")?;
        match killpg(
            Pid::from_raw(pid),
            if force {
                Signal::SIGKILL
            } else {
                Signal::SIGINT
            },
        ) {
            Ok(()) | Err(Errno::ESRCH) => Ok(()),
            Err(error) => Err(error).context("Cannot stop host process group"),
        }
    }
    #[cfg(not(unix))]
    {
        let _ = (pid, force);
        bail!("Host make tasks currently require Unix")
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use crate::{makefile::MakeTarget, runtime::Operation};

    #[test]
    fn streams_partial_and_invalid_utf8_output_before_exit() -> Result<()> {
        let root = tempfile::tempdir()?;
        let execution = Execution {
            label: "Partial output".into(),
            command: vec![
                "sh".into(),
                "-c".into(),
                "printf 'progress: \\377'; sleep 30".into(),
            ],
            operation: Operation::Run,
        };
        let (commands, receiver) = async_channel::unbounded();
        let (sender, events) = async_channel::bounded(32);
        let control = async {
            let output = async {
                let mut id = None;
                let mut text = String::new();
                while let Ok(event) = events.recv().await {
                    match event {
                        RuntimeMessage::Started { id: started, .. } => id = Some(started),
                        RuntimeMessage::Output { text: chunk, .. } => {
                            text.push_str(&chunk);
                            if text == "progress: \u{fffd}" {
                                commands
                                    .send(ClientCommand::Stop(id.clone().context("Missing ID")?))
                                    .await?;
                            }
                        }
                        RuntimeMessage::Exited { success, .. } => {
                            assert!(!success);
                            assert_eq!(text, "progress: \u{fffd}");
                            return Ok(());
                        }
                        RuntimeMessage::Failed { message, .. } => bail!("{message}"),
                        _ => {}
                    }
                }
                bail!("Missing exit")
            }
            .fuse();
            let deadline = async_io::Timer::after(Duration::from_secs(5)).fuse();
            pin_mut!(output, deadline);
            select! { result = output => result, _ = deadline => { commands.send(ClientCommand::Shutdown).await?; bail!("Partial output was delayed or stop hung"); } }
        };
        let (_, result) = futures::executor::block_on(futures::future::join(
            run(root.path().into(), execution, receiver, sender),
            control,
        ));
        result
    }

    #[test]
    fn completed_host_tasks_clean_up_children_that_hold_output_pipes() -> Result<()> {
        let root = tempfile::tempdir()?;
        let execution = Execution {
            label: "Background child".into(),
            command: vec![
                "sh".into(),
                "-c".into(),
                "sleep 30 & echo $! > child.pid; echo ready".into(),
            ],
            operation: Operation::Run,
        };
        let (commands, receiver) = async_channel::unbounded();
        let (sender, events) = async_channel::bounded(32);
        let control = async {
            let completed = async {
                while let Ok(event) = events.recv().await {
                    match event {
                        RuntimeMessage::Exited { success, code, .. } => {
                            assert!(success);
                            assert_eq!(code, Some(0));
                            return Ok(());
                        }
                        RuntimeMessage::Failed { message, .. } => bail!("{message}"),
                        _ => {}
                    }
                }
                bail!("Missing exit")
            }
            .fuse();
            let deadline = async_io::Timer::after(Duration::from_secs(5)).fuse();
            pin_mut!(completed, deadline);
            select! { result = completed => result, _ = deadline => { commands.send(ClientCommand::Shutdown).await?; bail!("Child kept output pipes open after leader exited"); } }
        };
        let (_, result) = futures::executor::block_on(futures::future::join(
            run(root.path().into(), execution, receiver, sender),
            control,
        ));
        result?;
        #[cfg(target_os = "linux")]
        {
            let pid = std::fs::read_to_string(root.path().join("child.pid"))?;
            let path = format!("/proc/{}/stat", pid.trim());
            // Orphans may briefly remain as zombies until the system reaps them.
            for _ in 0..100 {
                match std::fs::read_to_string(&path) {
                    Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(()),
                    Ok(stat)
                        if stat
                            .rsplit_once(") ")
                            .is_some_and(|(_, rest)| rest.starts_with('Z')) =>
                    {
                        return Ok(());
                    }
                    _ => std::thread::sleep(Duration::from_millis(10)),
                }
            }
            bail!("Background child is still running");
        }
        #[cfg(not(target_os = "linux"))]
        Ok(())
    }

    #[test]
    fn runs_host_make_with_stdout_stderr_and_exit_without_a_ros_daemon() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(
            root.path().join("Makefile"),
            "test:\n\t@printf 'host-output\\n'\n\t@printf 'host-error\\n' >&2\n",
        )?;
        let execution = MakeTarget {
            makefile: "Makefile".into(),
            target: "test".into(),
        }
        .execution(root.path(), Operation::Run)?;
        let (_commands, receiver) = async_channel::unbounded();
        let (sender, events) = async_channel::unbounded();
        futures::executor::block_on(run(root.path().to_owned(), execution, receiver, sender));
        let mut output = String::new();
        let mut error = String::new();
        let mut started = false;
        let mut exited = false;
        while let Ok(event) = events.try_recv() {
            match event {
                RuntimeMessage::Started { .. } => started = true,
                RuntimeMessage::Output {
                    stream: OutputStream::Stdout,
                    text,
                } => output.push_str(&text),
                RuntimeMessage::Output {
                    stream: OutputStream::Stderr,
                    text,
                } => error.push_str(&text),
                RuntimeMessage::Exited { success, code, .. } => {
                    assert!(success);
                    assert_eq!(code, Some(0));
                    exited = true;
                }
                RuntimeMessage::Failed { message, .. } => bail!("{message}"),
                _ => {}
            }
        }
        assert!(started && exited);
        assert!(output.contains("host-output"));
        assert!(error.contains("host-error"));
        Ok(())
    }

    #[test]
    fn stop_stops_make_and_its_foreground_recipe() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(
            root.path().join("Makefile"),
            "wait:\n\t@sleep 30 & task=$$!; printf 'child:%s\\n' \"$$task\"; wait \"$$task\"\n",
        )?;
        let execution = MakeTarget {
            makefile: "Makefile".into(),
            target: "wait".into(),
        }
        .execution(root.path(), Operation::Run)?;
        let (commands, receiver) = async_channel::unbounded();
        let (sender, events) = async_channel::unbounded();
        let control = async move {
            let timeout = async_io::Timer::after(Duration::from_secs(8)).fuse();
            let stopped = async {
                let mut process_id = None;
                let mut child_started = false;
                while let Ok(event) = events.recv().await {
                    match event {
                        RuntimeMessage::Started { id, .. } => {
                            process_id = Some(id);
                        }
                        RuntimeMessage::Output { text, .. } if text.starts_with("child:") => {
                            child_started = true;
                            commands
                                .send(ClientCommand::Stop(
                                    process_id.take().context("Missing process ID")?,
                                ))
                                .await?;
                        }
                        RuntimeMessage::Exited { success, .. } => {
                            assert!(child_started);
                            assert!(!success);
                            return Ok(());
                        }
                        RuntimeMessage::Failed { message, .. } => bail!("{message}"),
                        _ => {}
                    }
                }
                bail!("Host task closed without exit event")
            }
            .fuse();
            pin_mut!(timeout, stopped);
            select! { result = stopped => result, _ = timeout => bail!("Host task did not stop") }
        };
        let (_, result) = futures::executor::block_on(futures::future::join(
            run(root.path().to_owned(), execution, receiver, sender),
            control,
        ));
        result
    }

    #[test]
    fn reports_recipe_errors_and_missing_executables() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(root.path().join("Makefile"), "fail:\n\t@exit 7\n")?;
        let mut execution = MakeTarget {
            makefile: "Makefile".into(),
            target: "fail".into(),
        }
        .execution(root.path(), Operation::Build)?;
        for missing in [false, true] {
            if missing {
                execution.command[0] = root
                    .path()
                    .join("missing-make")
                    .to_string_lossy()
                    .into_owned();
            }
            let (_commands, receiver) = async_channel::unbounded();
            let (sender, events) = async_channel::unbounded();
            futures::executor::block_on(run(
                root.path().to_owned(),
                execution.clone(),
                receiver,
                sender,
            ));
            let mut reported = false;
            while let Ok(event) = events.try_recv() {
                match event {
                    RuntimeMessage::Exited { success, code, .. } if !missing => {
                        assert!(!success);
                        assert_eq!(code, Some(2));
                        reported = true;
                    }
                    RuntimeMessage::Failed { .. } if missing => reported = true,
                    _ => {}
                }
            }
            assert!(reported);
        }
        Ok(())
    }

    #[test]
    fn closing_the_session_cancels_a_pending_host_task() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(root.path().join("Makefile"), "wait:\n\t@sleep 30\n")?;
        let execution = MakeTarget {
            makefile: "Makefile".into(),
            target: "wait".into(),
        }
        .execution(root.path(), Operation::Run)?;
        let (commands, receiver) = async_channel::unbounded();
        drop(commands);
        let (sender, events) = async_channel::unbounded();
        futures::executor::block_on(run(root.path().to_owned(), execution, receiver, sender));
        let mut exited = false;
        while let Ok(event) = events.try_recv() {
            if matches!(event, RuntimeMessage::Exited { .. }) {
                exited = true;
            }
        }
        assert!(exited);
        Ok(())
    }
}
