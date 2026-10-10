use std::{
    collections::BTreeMap,
    path::Path,
    process::{Command, Stdio},
    sync::{
        Arc, Condvar, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::SyncSender,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use anyhow::{Context as _, ensure};
use futures_lite::{AsyncRead, AsyncReadExt as _, future};
use ros_studio_protocol::{Event, OutputStream};

static PROCESS_SEQUENCE: AtomicU64 = AtomicU64::new(1);

pub(crate) struct ProcessWorker {
    id: String,
    activated: Arc<(Mutex<bool>, Condvar)>,
    running: Arc<AtomicBool>,
    handle: JoinHandle<anyhow::Result<()>>,
}

impl ProcessWorker {
    pub(crate) fn start(
        command: Vec<String>,
        env: BTreeMap<String, String>,
        working_directory: &Path,
        events: SyncSender<Event>,
    ) -> anyhow::Result<Self> {
        let (program, arguments) = command.split_first().context("launch command is empty")?;
        ensure!(!program.is_empty(), "launch executable is empty");
        ensure!(
            env.keys()
                .all(|name| !name.is_empty() && !name.contains('=')),
            "launch environment contains an invalid variable name"
        );

        let mut process = Command::new(program);
        process
            .args(arguments)
            .envs(env)
            .current_dir(working_directory);
        #[cfg(unix)]
        {
            use std::os::unix::process::CommandExt as _;
            process.process_group(0);
        }
        let mut process = smol::process::Command::from(process);
        let mut child = process
            .kill_on_drop(true)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .with_context(|| format!("failed to launch {program}"))?;
        let stdout = child
            .stdout
            .take()
            .context("launched process has no stdout")?;
        let stderr = child
            .stderr
            .take()
            .context("launched process has no stderr")?;
        let sequence = PROCESS_SEQUENCE.fetch_add(1, Ordering::Relaxed);
        let id = format!("process-{}-{sequence}", child.id());
        let activated = Arc::new((Mutex::new(false), Condvar::new()));
        let running = Arc::new(AtomicBool::new(true));

        let handle = thread::spawn({
            let id = id.clone();
            let activated = activated.clone();
            let running = running.clone();
            move || {
                wait_for_activation(&activated)?;
                let stdout_reader =
                    spawn_output_reader(stdout, id.clone(), OutputStream::Stdout, events.clone());
                let stderr_reader =
                    spawn_output_reader(stderr, id.clone(), OutputStream::Stderr, events.clone());

                let status = loop {
                    if let Some(status) = child
                        .try_status()
                        .context("failed to query launched process")?
                    {
                        // The leader may exit while its descendants still own the pipes.
                        kill_process_tree(&mut child)
                            .context("failed to clean up completed process group")?;
                        break status;
                    }
                    if !running.load(Ordering::Acquire) {
                        if let Err(kill_error) = kill_process_tree(&mut child) {
                            if let Some(status) = child
                                .try_status()
                                .context("failed to query process after stop failed")?
                            {
                                break status;
                            }
                            return Err(kill_error).context("failed to stop launched process");
                        }
                        break future::block_on(child.status())
                            .context("failed to wait for stopped process")?;
                    }
                    thread::sleep(Duration::from_millis(50));
                };

                let stdout = join_reader(stdout_reader, "stdout");
                let stderr = join_reader(stderr_reader, "stderr");
                stdout?;
                stderr?;
                events
                    .send(Event::ProcessExited {
                        id,
                        success: status.success(),
                        code: status.code(),
                    })
                    .context("failed to report process exit")?;
                Ok(())
            }
        });

        Ok(Self {
            id,
            activated,
            running,
            handle,
        })
    }

    pub(crate) fn id(&self) -> &str {
        &self.id
    }

    pub(crate) fn activate(&self) -> anyhow::Result<()> {
        let (activated, condition) = &*self.activated;
        let mut activated = activated
            .lock()
            .map_err(|_| anyhow::anyhow!("process activation lock was poisoned"))?;
        *activated = true;
        condition.notify_all();
        Ok(())
    }

    #[cfg(feature = "ros-runtime")]
    pub(crate) fn is_finished(&self) -> bool {
        self.handle.is_finished()
    }

    pub(crate) fn stop(self) -> anyhow::Result<()> {
        self.activate()?;
        self.running.store(false, Ordering::Release);
        self.handle
            .join()
            .map_err(|_| anyhow::anyhow!("launched process thread panicked"))?
    }
}

fn kill_process_tree(child: &mut smol::process::Child) -> std::io::Result<()> {
    #[cfg(unix)]
    {
        use nix::{errno::Errno, sys::signal, unistd::Pid};

        let process_id = i32::try_from(child.id())
            .map_err(|_| std::io::Error::other("process ID does not fit in i32"))?;
        match signal::killpg(Pid::from_raw(process_id), signal::Signal::SIGKILL) {
            Ok(()) | Err(Errno::ESRCH) => Ok(()),
            Err(error) => Err(std::io::Error::from_raw_os_error(error as i32)),
        }
    }
    #[cfg(not(unix))]
    {
        if child.try_status()?.is_some() {
            Ok(())
        } else {
            child.kill()
        }
    }
}

fn wait_for_activation(activated: &Arc<(Mutex<bool>, Condvar)>) -> anyhow::Result<()> {
    let (activated, condition) = &**activated;
    let mut is_activated = activated
        .lock()
        .map_err(|_| anyhow::anyhow!("process activation lock was poisoned"))?;
    while !*is_activated {
        is_activated = condition
            .wait(is_activated)
            .map_err(|_| anyhow::anyhow!("process activation lock was poisoned"))?;
    }
    Ok(())
}

fn spawn_output_reader(
    mut reader: impl AsyncRead + Unpin + Send + 'static,
    id: String,
    stream: OutputStream,
    events: SyncSender<Event>,
) -> JoinHandle<anyhow::Result<()>> {
    thread::spawn(move || {
        future::block_on(async move {
            let mut buffer = [0; 4096];
            let mut decoder = ros_studio_protocol::output::OutputDecoder::default();
            loop {
                let count = reader
                    .read(&mut buffer)
                    .await
                    .context("failed to read process output")?;
                let text = decoder.decode(&buffer[..count], count == 0);
                if !text.is_empty() {
                    events
                        .send(Event::ProcessOutput {
                            id: id.clone(),
                            stream,
                            text,
                        })
                        .context("failed to report process output")?;
                }
                if count == 0 {
                    return Ok(());
                }
            }
        })
    })
}

fn join_reader(handle: JoinHandle<anyhow::Result<()>>, stream_name: &str) -> anyhow::Result<()> {
    handle
        .join()
        .map_err(|_| anyhow::anyhow!("{stream_name} reader thread panicked"))?
}

#[cfg(test)]
mod tests {
    use std::{env, io::Write as _, sync::mpsc, time::Duration};

    use tempfile::tempdir;

    use super::*;

    const HELPER_ENV: &str = "ROS_STUDIO_PROCESS_TEST_HELPER";

    #[cfg(unix)]
    #[test]
    fn completing_the_leader_cleans_up_children_with_or_without_output_pipes() -> anyhow::Result<()>
    {
        for inherited_output in [false, true] {
            let root = tempdir()?;
            let script = if inherited_output {
                "sleep 30 & echo $! > child.pid; echo ready"
            } else {
                "sleep 30 </dev/null >/dev/null 2>&1 & echo $! > child.pid; echo ready"
            };
            let (sender, receiver) = mpsc::sync_channel(256);
            let worker = ProcessWorker::start(
                vec!["sh".into(), "-c".into(), script.into()],
                BTreeMap::new(),
                root.path(),
                sender,
            )?;
            worker.activate()?;
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            loop {
                match receiver
                    .recv_timeout(deadline.saturating_duration_since(std::time::Instant::now()))?
                {
                    Event::ProcessExited { success, code, .. } => {
                        assert!(success);
                        assert_eq!(code, Some(0));
                        break;
                    }
                    _ => {}
                }
            }
            worker.stop()?;
            #[cfg(target_os = "linux")]
            {
                let pid = std::fs::read_to_string(root.path().join("child.pid"))?;
                let path = format!("/proc/{}/stat", pid.trim());
                let deadline = std::time::Instant::now() + Duration::from_secs(1);
                loop {
                    match std::fs::read_to_string(&path) {
                        Err(error) if error.kind() == std::io::ErrorKind::NotFound => break,
                        Ok(stat)
                            if stat
                                .rsplit_once(") ")
                                .is_some_and(|(_, rest)| rest.starts_with('Z')) =>
                        {
                            break;
                        }
                        _ => {}
                    }
                    anyhow::ensure!(
                        std::time::Instant::now() < deadline,
                        "Background child is still running"
                    );
                    thread::sleep(Duration::from_millis(10));
                }
            }
        }
        Ok(())
    }

    #[cfg(unix)]
    #[test]
    fn closing_a_backpressured_output_receiver_does_not_prevent_stop() -> anyhow::Result<()> {
        let root = tempdir()?;
        let (sender, receiver) = mpsc::sync_channel(1);
        let worker = ProcessWorker::start(
            vec![
                "sh".into(),
                "-c".into(),
                "while :; do printf 'output\\n'; done".into(),
            ],
            BTreeMap::new(),
            root.path(),
            sender,
        )?;
        worker.activate()?;
        receiver.recv_timeout(Duration::from_secs(5))?;
        drop(receiver);
        // An output error is expected, but cleanup must finish and reap the leader.
        let (completed, completion) = mpsc::channel();
        thread::spawn(move || {
            let _ = completed.send(worker.stop());
        });
        assert!(completion.recv_timeout(Duration::from_secs(5))?.is_err());
        Ok(())
    }

    #[test]
    fn process_test_helper() -> anyhow::Result<()> {
        if env::var_os(HELPER_ENV).is_none() {
            return Ok(());
        }
        std::io::stdout().write_all(b"hello stdout")?;
        std::io::stdout().flush()?;
        std::io::stderr().write_all(b"hello stderr")?;
        std::io::stderr().flush()?;
        std::process::exit(7);
    }

    #[test]
    fn captures_output_and_exit_after_activation() -> anyhow::Result<()> {
        let executable = env::current_exe()?;
        let working_directory = tempdir()?;
        let (sender, receiver) = mpsc::sync_channel(256);
        let worker = ProcessWorker::start(
            vec![
                executable.to_string_lossy().into_owned(),
                "--exact".to_owned(),
                "process::tests::process_test_helper".to_owned(),
                "--nocapture".to_owned(),
            ],
            BTreeMap::from([(HELPER_ENV.to_owned(), "1".to_owned())]),
            working_directory.path(),
            sender,
        )?;
        assert!(receiver.try_recv().is_err());

        let process_id = worker.id().to_owned();
        worker.activate()?;
        let mut stdout = String::new();
        let mut stderr = String::new();
        let exit = loop {
            match receiver.recv_timeout(Duration::from_secs(5))? {
                Event::ProcessOutput { id, stream, text } => {
                    assert_eq!(id, process_id);
                    match stream {
                        OutputStream::Stdout => stdout.push_str(&text),
                        OutputStream::Stderr => stderr.push_str(&text),
                    }
                }
                Event::ProcessExited { id, success, code } => {
                    assert_eq!(id, process_id);
                    break (success, code);
                }
                _ => {}
            }
        };

        assert!(stdout.contains("hello stdout"));
        assert!(stderr.contains("hello stderr"));
        assert_eq!(exit, (false, Some(7)));
        worker.stop()?;
        Ok(())
    }

    #[test]
    fn rejects_empty_commands_and_invalid_environment_names() {
        let (sender, _) = mpsc::sync_channel(256);
        assert!(ProcessWorker::start(Vec::new(), BTreeMap::new(), Path::new("."), sender).is_err());

        let (sender, _) = mpsc::sync_channel(256);
        assert!(
            ProcessWorker::start(
                vec!["missing".to_owned()],
                BTreeMap::from([("BAD=NAME".to_owned(), "value".to_owned())]),
                Path::new("."),
                sender,
            )
            .is_err()
        );
    }
}
