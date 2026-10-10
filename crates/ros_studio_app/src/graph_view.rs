use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    process::Stdio,
    time::{Duration, Instant},
};

use anyhow::{Context as _, Result, ensure};
use async_channel::Receiver;
use gpui::{
    AppContext as _, Bounds, ClickEvent, Context, Div, MouseButton, MouseDownEvent, MouseMoveEvent,
    MouseUpEvent, PathBuilder, PathPromptOptions, Pixels, Render, ScrollDelta, ScrollWheelEvent,
    SharedString, Stateful, Task, Window, canvas, div, fill, point, prelude::*, px, rgb, size,
};
use notify::{EventKind, RecommendedWatcher, RecursiveMode, Watcher as _};
use ros_studio_app::{
    ExternalEditor, GraphSnapshot, NODE_HEIGHT, NODE_WIDTH, NodePosition, Viewport,
    health::HealthSnapshot,
    local_process,
    makefile::{self, MakeCatalog, MakeTarget},
    preferences::Preferences,
    relevant_project_path,
    runtime::{Execution, GraphMode, Operation, RunTarget, RuntimeOverlay},
    runtime_client::{ClientCommand, RuntimeMessage, SessionHandle, connect},
    runtime_environment::RuntimePreference,
};
use ros_studio_model::{EndpointKind, Project, RuntimeState, SourceLocation};
use ros_studio_protocol::OutputStream;
use walkdir::WalkDir;

use crate::console_view::{Closed, ConsoleView};
use crate::node_wizard::{NodeWizard, WizardEvent};
use crate::text_input::{Changed, TextInput};
use crate::theme;

mod health_view;

enum WatchEvent {
    Changed,
    Failed(String),
}

#[derive(Clone, Copy, Default, Eq, PartialEq)]
enum InspectorView {
    #[default]
    Selection,
    Packages,
    Nodes,
    Topics,
    Health,
}

#[derive(Clone)]
enum Drag {
    Console {
        mouse_y: f32,
        height: f32,
    },
    Canvas {
        mouse: (f32, f32),
        offset: (f32, f32),
    },
    Node {
        node_id: String,
        mouse: (f32, f32),
        position: (f32, f32),
    },
}

pub struct GraphView {
    root: Option<PathBuf>,
    restoring_project: bool,
    snapshot: Option<GraphSnapshot>,
    selected_node: Option<String>,
    editor: ExternalEditor,
    show_editors: bool,
    viewport: Viewport,
    canvas_bounds: Option<Bounds<Pixels>>,
    initial_view_pending: bool,
    drag: Option<Drag>,
    status: String,
    error: Option<String>,
    loading: bool,
    scan_task: Option<Task<()>>,
    watch_task: Option<Task<()>>,
    watcher: Option<RecommendedWatcher>,
    wizard: Option<gpui::Entity<NodeWizard>>,
    wizard_subscription: Option<gpui::Subscription>,
    design_project: Option<Project>,
    overlay: RuntimeOverlay,
    health_snapshot: HealthSnapshot,
    health_error: Option<String>,
    health_settings_error: Option<String>,
    health_topic: Option<(String, String)>,
    health_hz: gpui::Entity<TextInput>,
    health_timeout: gpui::Entity<TextInput>,
    _health_input_subscriptions: [gpui::Subscription; 2],
    _health_tick_task: Task<()>,
    mode: GraphMode,
    positions: BTreeMap<String, NodePosition>,
    preference: RuntimePreference,
    connection: Option<SessionHandle>,
    local_connection: Option<SessionHandle>,
    local_task: Option<Task<()>>,
    make_catalog: MakeCatalog,
    expanded_make_groups: BTreeSet<Option<String>>,
    show_make_panel: bool,
    make_filter: gpui::Entity<TextInput>,
    _make_filter_subscription: gpui::Subscription,
    running_make: Option<MakeTarget>,
    runtime_task: Option<Task<()>>,
    runtime_workers: Vec<Task<()>>,
    closing: bool,
    _quit_subscription: gpui::Subscription,
    connection_label: String,
    ready: bool,
    discovery: bool,
    queued: Option<Execution>,
    launch_pending: bool,
    active: Option<(String, Operation)>,
    auto_mode: bool,
    stopping: bool,
    console: gpui::Entity<ConsoleView>,
    _console_subscription: gpui::Subscription,
    console_visible: bool,
    console_height: f32,
    preferences: Preferences,
    settings_path: Option<PathBuf>,
    run_target: Option<RunTarget>,
    show_configurations: bool,
    inspector_view: InspectorView,
    context_menu: Option<gpui::Point<Pixels>>,
    pub focus: gpui::FocusHandle,
}

impl GraphView {
    pub fn new(
        root: Option<PathBuf>,
        preferences: Preferences,
        settings_path: Option<PathBuf>,
        initial_error: Option<String>,
        cx: &mut Context<Self>,
    ) -> Self {
        let restoring_project = root.is_none();
        let root = preferences.startup_project(root);
        let restoring_project = restoring_project && root.is_some();
        let console = cx.new(ConsoleView::new);
        let console_subscription = cx.subscribe(&console, |this, _, _: &Closed, cx| {
            this.console_visible = false;
            cx.notify();
        });
        let make_filter = cx.new(|cx| TextInput::new("Search Make commands…", false, cx));
        let make_filter_subscription =
            cx.subscribe(&make_filter, |_, _, _: &Changed, cx| cx.notify());
        let health_hz = cx.new(|cx| TextInput::new("Expected Hz", false, cx));
        let health_timeout = cx.new(|cx| TextInput::new("Silence timeout (seconds)", false, cx));
        let health_input_subscriptions = [
            cx.subscribe(&health_hz, |_, _, _: &Changed, cx| cx.notify()),
            cx.subscribe(&health_timeout, |_, _, _: &Changed, cx| cx.notify()),
        ];
        let health_tick_task = cx.spawn(async move |this, cx| {
            loop {
                async_io::Timer::after(Duration::from_secs(1)).await;
                if this
                    .update(cx, |this, cx| {
                        if this.discovery
                            && this.health_error.is_none()
                            && this.health_snapshot.is_stale(Instant::now())
                        {
                            this.health_error = Some(
                                "Health observer stopped reporting; measurements are unknown"
                                    .into(),
                            );
                            cx.notify();
                        }
                    })
                    .is_err()
                {
                    break;
                }
            }
        });
        let quit_subscription = cx.on_app_quit(|this, _| {
            this.connection = None;
            this.runtime_task = None;
            this.local_connection = None;
            this.local_task = None;
            let workers = std::mem::take(&mut this.runtime_workers);
            async move {
                futures::future::join_all(workers).await;
            }
        });
        let mut view = Self {
            root,
            restoring_project,
            snapshot: None,
            selected_node: None,
            editor: preferences.editor,
            show_editors: false,
            viewport: Viewport::default(),
            canvas_bounds: None,
            initial_view_pending: true,
            drag: None,
            status: "Open a ROS project folder to begin".to_owned(),
            error: None,
            loading: false,
            scan_task: None,
            watch_task: None,
            watcher: None,
            wizard: None,
            wizard_subscription: None,
            design_project: None,
            overlay: RuntimeOverlay::default(),
            health_snapshot: HealthSnapshot::default(),
            health_error: None,
            health_settings_error: None,
            health_topic: None,
            health_hz,
            health_timeout,
            _health_input_subscriptions: health_input_subscriptions,
            _health_tick_task: health_tick_task,
            mode: GraphMode::Design,
            positions: BTreeMap::new(),
            preference: preferences.environment,
            connection: None,
            local_connection: None,
            local_task: None,
            make_catalog: MakeCatalog::default(),
            expanded_make_groups: BTreeSet::new(),
            show_make_panel: false,
            make_filter,
            _make_filter_subscription: make_filter_subscription,
            running_make: None,
            runtime_task: None,
            runtime_workers: Vec::new(),
            closing: false,
            _quit_subscription: quit_subscription,
            connection_label: "Disconnected".into(),
            ready: false,
            discovery: false,
            queued: None,
            launch_pending: false,
            active: None,
            auto_mode: preferences.auto_mode,
            stopping: false,
            console,
            _console_subscription: console_subscription,
            console_visible: false,
            console_height: preferences.console_height.clamp(120.0, 800.0),
            preferences,
            settings_path,
            run_target: None,
            show_configurations: false,
            inspector_view: InspectorView::Selection,
            context_menu: None,
            focus: cx.focus_handle(),
        };
        if view.root.is_some() {
            view.refresh(cx);
        }
        view.error = initial_error;
        view
    }

    pub fn prepare_close(&mut self, window: &mut Window, cx: &mut Context<Self>) -> bool {
        if self.closing {
            return false;
        }
        if self.runtime_workers.is_empty() {
            return true;
        }
        // GPUI's app-quit grace period is shorter than ROS discovery shutdown.
        self.closing = true;
        self.connection = None;
        self.runtime_task = None;
        self.local_connection = None;
        self.local_task = None;
        let workers = std::mem::take(&mut self.runtime_workers);
        cx.spawn_in(window, async move |_, cx| {
            futures::future::join_all(workers).await;
            if let Err(error) = cx.update(|window, _| window.remove_window()) {
                eprintln!("Cannot close ROS 2 Studio window: {error:#}");
            }
        })
        .detach();
        cx.notify();
        false
    }

    fn save_preferences(&mut self, cx: &mut Context<Self>) {
        self.preferences.editor = self.editor;
        self.preferences.environment = self.preference;
        self.preferences.auto_mode = self.auto_mode;
        self.preferences.console_height = self.console_height;
        if let Some(path) = &self.settings_path {
            if let Err(error) = self.preferences.save(path) {
                self.show_error(error, cx);
            }
        }
    }

    fn set_configuration(&mut self, target: RunTarget, cx: &mut Context<Self>) {
        // Explicit file/make configurations must not be overridden by an old node selection.
        self.selected_node = match &target {
            RunTarget::Node(id) => Some(id.clone()),
            RunTarget::File(_) | RunTarget::Make(_) => None,
        };
        if let Some(project) = &self.design_project {
            self.preferences
                .configurations
                .insert(project.root_path.clone(), target.clone());
        }
        self.run_target = Some(target);
        self.show_configurations = false;
        self.save_preferences(cx);
        cx.notify();
    }

    fn choose_source(&mut self, cx: &mut Context<Self>) {
        let selection = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some("Choose source file".into()),
        });
        cx.spawn(async move |this, cx| {
            let result = async {
                if let Some(path) = selection.await??.and_then(|paths| paths.into_iter().next()) {
                    this.update(cx, |this, cx| -> Result<()> {
                        let root = this
                            .root
                            .as_ref()
                            .context("Open a project first")?
                            .canonicalize()?;
                        let path = path.canonicalize()?;
                        let relative = path
                            .strip_prefix(root)
                            .context("Choose a file inside the open project")?;
                        ensure!(
                            matches!(
                                path.extension().and_then(|value| value.to_str()),
                                Some("rs" | "py" | "c" | "cc" | "cpp" | "cxx")
                            ),
                            "Choose a Rust, Python, C, or C++ source"
                        );
                        this.set_configuration(
                            RunTarget::File(
                                relative
                                    .to_str()
                                    .context("Source path is not UTF-8")?
                                    .into(),
                            ),
                            cx,
                        );
                        Ok(())
                    })??;
                }
                anyhow::Ok(())
            }
            .await;
            if let Err(error) = result {
                if let Err(update_error) = this.update(cx, |this, cx| this.show_error(error, cx)) {
                    eprintln!("Source chooser: {update_error:#}");
                }
            }
        })
        .detach();
    }

    fn choose_project(&mut self, cx: &mut Context<Self>) {
        let prompt = cx.prompt_for_paths(PathPromptOptions {
            files: false,
            directories: true,
            multiple: false,
            prompt: Some("Open ROS project".into()),
        });
        cx.spawn(async move |this, cx| {
            let result = async {
                let selected = prompt
                    .await
                    .context("Folder chooser was closed unexpectedly")??;
                if let Some(root) = selected.and_then(|paths| paths.into_iter().next()) {
                    this.update(cx, |this, cx| {
                        this.local_connection = None;
                        this.local_task = None;
                        this.health_topic = None;
                        this.health_settings_error = None;
                        this.make_catalog = MakeCatalog::default();
                        this.expanded_make_groups.clear();
                        this.show_make_panel = false;
                        this.running_make = None;
                        this.make_filter.update(cx, |input, cx| input.clear(cx));
                        this.disconnect(cx);
                        this.design_project = None;
                        this.positions.clear();
                        this.mode = GraphMode::Design;
                        this.watcher = None;
                        this.watch_task = None;
                        this.snapshot = None;
                        this.selected_node = None;
                        this.run_target = None;
                        this.show_configurations = false;
                        this.context_menu = None;
                        this.inspector_view = InspectorView::Selection;
                        this.drag = None;
                        this.root = Some(root);
                        this.restoring_project = false;
                        this.initial_view_pending = true;
                        this.viewport = Viewport::default();
                        this.refresh(cx);
                    })?;
                }
                anyhow::Ok(())
            }
            .await;
            if let Err(error) = result {
                if let Err(update_error) = this.update(cx, |this, cx| this.show_error(error, cx)) {
                    eprintln!("Folder chooser: {update_error:#}");
                }
            }
        })
        .detach();
    }

    fn new_node(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if self.wizard.is_some() {
            return;
        }
        let Some(project) = &self.design_project else {
            self.show_error(anyhow::anyhow!("Open a ROS project first"), cx);
            return;
        };
        let project = project.clone();
        let wizard = cx.new(|cx| NodeWizard::new(project, window, cx));
        self.wizard_subscription = Some(cx.subscribe_in(
            &wizard,
            window,
            |this, _, event: &WizardEvent, window, cx| {
                window.focus(&this.focus, cx);
                this.wizard = None;
                this.wizard_subscription = None;
                if let WizardEvent::Created(source) = event {
                    this.refresh(cx);
                    this.open_source(source.clone(), cx);
                }
                cx.notify();
            },
        ));
        self.drag = None;
        self.wizard = Some(wizard);
        cx.notify();
    }

    fn refresh(&mut self, cx: &mut Context<Self>) {
        let Some(root) = self.root.clone() else {
            return;
        };
        self.loading = true;
        self.error = None;
        self.status = "Scanning project…".to_owned();
        let scan = cx.background_spawn(async move {
            let snapshot = GraphSnapshot::scan(&root)?;
            let make_targets = makefile::catalog(&snapshot.root, &snapshot.project);
            let watcher = watch_project(&snapshot.root);
            anyhow::Ok((snapshot, watcher, make_targets))
        });
        self.scan_task = Some(cx.spawn(async move |this, cx| {
            let result = scan.await;
            if let Err(error) = this.update(cx, |this, cx| {
                this.loading = false;
                match result {
                    Ok((snapshot, watcher, make_targets)) => {
                        this.restoring_project = false;
                        let first_scan = this.design_project.is_none();
                        if first_scan {
                            this.show_make_panel = make_targets
                                .as_ref()
                                .is_ok_and(|catalog| !catalog.targets.is_empty());
                            // Frame after the sidebar has taken its actual width, not
                            // against the previous project's canvas bounds.
                            this.canvas_bounds = None;
                            this.initial_view_pending = true;
                        }
                        this.apply_snapshot(snapshot, watcher, cx);
                        match make_targets {
                            Ok(catalog) => {
                                this.make_catalog = catalog;
                            }
                            Err(error) => {
                                this.make_catalog = MakeCatalog::default();
                                this.error =
                                    Some(format!("Makefile discovery unavailable: {error:#}"));
                            }
                        }
                    }
                    Err(error) if this.restoring_project => {
                        // The folder can disappear after the startup accessibility
                        // check, or scanning can encounter unreadable files. Keep
                        // the saved path for a later launch, but show the normal
                        // empty page rather than a broken restored session.
                        eprintln!("Cannot restore the last project: {error:#}");
                        this.restoring_project = false;
                        this.root = None;
                        this.status = "Open a ROS project folder to begin".into();
                    }
                    Err(error) => this.show_error(error, cx),
                }
                cx.notify();
            }) {
                eprintln!("Cannot update graph: {error:#}");
            }
        }));
        cx.notify();
    }

    fn apply_snapshot(
        &mut self,
        snapshot: GraphSnapshot,
        watcher: Result<(RecommendedWatcher, Receiver<WatchEvent>)>,
        cx: &mut Context<Self>,
    ) {
        self.status = if snapshot.project.packages.is_empty() {
            "No ROS packages found. Choose the folder containing package.xml files.".to_owned()
        } else {
            "Drag to pan · Scroll to zoom · Double-click a source node to open its code".to_owned()
        };
        self.root = Some(snapshot.root.clone());
        if self.preferences.last_project != self.root {
            self.preferences.last_project = self.root.clone();
            self.save_preferences(cx);
        }
        if self.design_project.is_none() {
            self.run_target = self
                .preferences
                .configurations
                .get(&snapshot.project.root_path)
                .cloned();
        }
        self.design_project = Some(snapshot.project);
        self.rebuild_graph(cx);
        self.drag = None;
        match watcher {
            Ok((watcher, events)) => {
                self.watcher = Some(watcher);
                self.watch_task = Some(Self::watch_events(events, cx));
            }
            Err(error) => {
                self.watcher = None;
                self.watch_task = None;
                self.error = Some(format!("Auto-refresh unavailable: {error:#}. Use Refresh."));
            }
        }
        if self.initial_view_pending {
            self.frame_project(cx);
        }
    }

    fn rebuild_graph(&mut self, cx: &mut Context<Self>) {
        let Some(design) = &self.design_project else {
            return;
        };
        if let Some(snapshot) = &self.snapshot {
            for position in &snapshot.positions {
                self.positions
                    .insert(position.node_id.clone(), position.clone());
            }
        }
        let project = self.overlay.project(design, self.mode);
        let mut snapshot = GraphSnapshot::from_project(PathBuf::from(&design.root_path), project);
        let mut previous = GraphSnapshot::from_project(snapshot.root.clone(), design.clone());
        previous.positions = self.positions.values().cloned().collect();
        snapshot.preserve_positions(&previous);
        if self
            .snapshot
            .as_ref()
            .is_none_or(|previous| previous.project.nodes.is_empty())
            && !snapshot.project.nodes.is_empty()
        {
            self.initial_view_pending = true;
        }
        self.snapshot = Some(snapshot);
        if self.initial_view_pending {
            self.frame_project(cx);
        }
        cx.notify();
    }

    fn log(&mut self, text: &str, stream: OutputStream, cx: &mut Context<Self>) {
        self.console
            .update(cx, |console, cx| console.append(text, stream, cx));
    }

    fn connect_runtime(&mut self, cx: &mut Context<Self>) {
        if self.connection.is_some() {
            return;
        }
        let Some(root) = self
            .design_project
            .as_ref()
            .map(|project| PathBuf::from(&project.root_path))
        else {
            self.show_error(anyhow::anyhow!("Open and scan a ROS project first"), cx);
            return;
        };
        let (sender, events) = async_channel::bounded(256);
        let (command_sender, commands) = async_channel::unbounded();
        self.connection = Some(SessionHandle::new(command_sender));
        self.connection_label = "Connecting…".into();
        self.console_visible = true;
        self.error = None;
        self.log("\nConnecting ROS runtime…\n", OutputStream::Stdout, cx);
        let preference = self.preference;
        self.runtime_workers.push(cx.background_spawn(async move {
            connect(root, preference, commands, sender).await;
        }));
        self.runtime_task = Some(cx.spawn(async move |this, cx| {
            while let Ok(first) = events.recv().await {
                let mut batch = vec![first];
                while batch.len() < 128 {
                    match events.try_recv() {
                        Ok(event) => batch.push(event),
                        Err(_) => break,
                    }
                }
                if let Err(error) = this.update(cx, |this, cx| {
                    for event in batch {
                        this.runtime_message(event, cx);
                    }
                    cx.notify();
                }) {
                    eprintln!("Cannot update ROS runtime view: {error:#}");
                    break;
                }
            }
        }));
        cx.notify();
    }

    fn disconnect(&mut self, cx: &mut Context<Self>) {
        self.connection = None;
        self.runtime_task = None;
        self.ready = false;
        self.discovery = false;
        self.queued = None;
        if self.local_connection.is_none() {
            self.active = None;
            self.launch_pending = false;
            self.stopping = false;
        }
        self.connection_label = "Disconnected".into();
        self.overlay = RuntimeOverlay::default();
        self.health_snapshot = HealthSnapshot::default();
        self.health_error = None;
        if self.auto_mode {
            self.mode = GraphMode::Design;
        }
        self.rebuild_graph(cx);
    }

    fn runtime_message(&mut self, event: RuntimeMessage, cx: &mut Context<Self>) {
        match event {
            RuntimeMessage::Health(report) => {
                if let Some(design) = &self.design_project {
                    let watches = self.health_watches().to_vec();
                    if self
                        .health_snapshot
                        .apply(report, &design.id, &watches, Instant::now())
                    {
                        self.health_error = None;
                    }
                }
            }
            RuntimeMessage::HealthUnavailable(message) => {
                self.health_snapshot = HealthSnapshot::default();
                self.health_error = Some(format!(
                    "Health unavailable: {message}. Rebuild the ROS daemon if needed."
                ));
            }
            RuntimeMessage::Environment(label) => {
                self.connection_label = format!("{label} · connecting…");
                self.log(&format!("{label}\n"), OutputStream::Stdout, cx);
            }
            RuntimeMessage::Output { stream, text } => self.log(&text, stream, cx),
            RuntimeMessage::Ready { discovery } => {
                self.ready = true;
                self.discovery = discovery;
                self.connection_label = if discovery {
                    "Connected · discovery active"
                } else {
                    "Connected · discovery unavailable"
                }
                .into();
                if discovery && !self.health_watches().is_empty() {
                    self.sync_health(cx);
                }
                if let Some(execution) = self.queued.take() {
                    self.send_execution(execution, cx);
                }
            }
            RuntimeMessage::Patch(patch) => {
                if let Some(design) = &self.design_project {
                    if self.overlay.apply(patch, &design.id) {
                        self.rebuild_graph(cx);
                    }
                }
            }
            RuntimeMessage::Started { id, operation } => {
                self.launch_pending = false;
                self.log(
                    &format!("{} started ({id})\n", operation.label()),
                    OutputStream::Stdout,
                    cx,
                );
                self.active = Some((id, operation));
                if operation == Operation::Run && self.auto_mode && self.discovery {
                    self.mode = GraphMode::Live;
                    self.rebuild_graph(cx);
                }
            }
            RuntimeMessage::Exited { id, success, code } => {
                if self
                    .active
                    .as_ref()
                    .is_some_and(|(active, _)| active == &id)
                {
                    let operation = self.active.take().map(|(_, operation)| operation);
                    self.launch_pending = false;
                    self.stopping = false;
                    if operation == Some(Operation::Run) && self.auto_mode {
                        self.mode = GraphMode::Design;
                        self.rebuild_graph(cx);
                    }
                }
                self.log(
                    &format!(
                        "\n{id} · {} · exit {code:?}\n",
                        if success {
                            "finished"
                        } else {
                            "failed / stopped"
                        }
                    ),
                    if success {
                        OutputStream::Stdout
                    } else {
                        OutputStream::Stderr
                    },
                    cx,
                );
            }
            RuntimeMessage::Failed { operation, message } => {
                if operation.is_some() {
                    self.launch_pending = false;
                    self.queued = None;
                }
                self.stopping = false;
                self.console_visible = true;
                self.log(&format!("{message}\n"), OutputStream::Stderr, cx);
                self.show_error(anyhow::anyhow!(message), cx);
            }
            RuntimeMessage::Disconnected(error) => {
                self.connection = None;
                self.ready = false;
                self.discovery = false;
                self.queued = None;
                if self.local_connection.is_none() {
                    self.active = None;
                    self.launch_pending = false;
                    self.stopping = false;
                }
                self.overlay = RuntimeOverlay::default();
                self.health_snapshot = HealthSnapshot::default();
                self.health_error = None;
                self.connection_label = "Disconnected".into();
                if self.auto_mode {
                    self.mode = GraphMode::Design;
                }
                self.rebuild_graph(cx);
                if let Some(error) = error {
                    self.console_visible = true;
                    self.log(&format!("{error}\n"), OutputStream::Stderr, cx);
                    self.show_error(anyhow::anyhow!(error), cx);
                }
            }
        }
    }

    fn launch(&mut self, operation: Operation, cx: &mut Context<Self>) {
        if self.active.is_some() || self.launch_pending {
            self.show_error(
                anyhow::anyhow!("Stop the current process before launching another"),
                cx,
            );
            return;
        }
        let Some(target) =
            RunTarget::for_selection(self.selected_node.as_deref(), self.run_target.as_ref())
        else {
            self.show_configurations = true;
            self.status =
                "Choose a ROS node, source file, or Makefile target, then Build/Run".into();
            cx.notify();
            return;
        };
        self.launch_target(target, operation, true, cx);
    }

    /// The Make panel launches its explicit target, independent of node selection
    /// and without changing the user's Build/Run configuration.
    fn launch_target(
        &mut self,
        target: RunTarget,
        operation: Operation,
        remember_configuration: bool,
        cx: &mut Context<Self>,
    ) {
        if self.active.is_some() || self.launch_pending {
            self.show_error(
                anyhow::anyhow!("Stop the current process before launching another"),
                cx,
            );
            return;
        }
        let result = self
            .design_project
            .as_ref()
            .context("Open a ROS project first")
            .and_then(|project| target.execution(project, operation));
        match result {
            Ok(execution) => {
                let local = matches!(target, RunTarget::Make(_));
                if remember_configuration && self.run_target.as_ref() != Some(&target) {
                    self.set_configuration(target.clone(), cx);
                }
                self.console_visible = true;
                self.error = None;
                self.launch_pending = true;
                if local {
                    if let RunTarget::Make(target) = target {
                        self.running_make = Some(target);
                    }
                    self.launch_local(execution, cx);
                } else if self.ready {
                    self.send_execution(execution, cx);
                } else {
                    self.queued = Some(execution);
                    self.connect_runtime(cx);
                }
            }
            Err(error) => self.show_error(error, cx),
        }
        cx.notify();
    }

    fn launch_local(&mut self, execution: Execution, cx: &mut Context<Self>) {
        let Some(project) = &self.design_project else {
            return;
        };
        let root = PathBuf::from(&project.root_path);
        self.log(
            &format!("\n{}\nCommand: {:?}\n", execution.label, execution.command),
            OutputStream::Stdout,
            cx,
        );
        let (sender, events) = async_channel::bounded(256);
        let (command_sender, commands) = async_channel::unbounded();
        self.local_connection = Some(SessionHandle::new(command_sender));
        self.runtime_workers.push(cx.background_spawn(async move {
            local_process::run(root, execution, commands, sender).await;
        }));
        self.local_task = Some(cx.spawn(async move |this, cx| {
            while let Ok(first) = events.recv().await {
                let mut batch = vec![first];
                while batch.len() < 128 {
                    match events.try_recv() {
                        Ok(event) => batch.push(event),
                        Err(_) => break,
                    }
                }
                if this
                    .update(cx, |this, cx| {
                        for event in batch {
                            if matches!(
                                event,
                                RuntimeMessage::Exited { .. } | RuntimeMessage::Failed { .. }
                            ) {
                                this.local_connection = None;
                                this.running_make = None;
                                if matches!(event, RuntimeMessage::Failed { .. }) {
                                    this.active = None;
                                    if this.auto_mode {
                                        this.mode = GraphMode::Design;
                                        this.rebuild_graph(cx);
                                    }
                                }
                            }
                            this.runtime_message(event, cx);
                        }
                        cx.notify();
                    })
                    .is_err()
                {
                    break;
                }
            }
        }));
    }

    fn send_execution(&mut self, execution: Execution, cx: &mut Context<Self>) {
        self.log(
            &format!("\n{}\nCommand: {:?}\n", execution.label, execution.command),
            OutputStream::Stdout,
            cx,
        );
        let result = self
            .connection
            .as_ref()
            .context("ROS runtime is disconnected")
            .and_then(|connection| connection.send(ClientCommand::Launch(execution)));
        if let Err(error) = result {
            self.launch_pending = false;
            self.show_error(error, cx);
        }
    }

    fn stop(&mut self, cx: &mut Context<Self>) {
        if self.stopping {
            return;
        }
        if self.launch_pending {
            if let Some(connection) = &self.local_connection {
                if let Err(error) = connection.send(ClientCommand::Shutdown) {
                    self.show_error(error, cx);
                } else {
                    self.stopping = true;
                    self.log("Stopping host task…\n", OutputStream::Stdout, cx);
                }
                cx.notify();
                return;
            }
            self.disconnect(cx);
            self.log(
                "Pending launch cancelled; closing runtime session.\n",
                OutputStream::Stdout,
                cx,
            );
            return;
        }
        let Some((id, _)) = &self.active else {
            return;
        };
        let result = self
            .local_connection
            .as_ref()
            .or(self.connection.as_ref())
            .context("Process session is disconnected")
            .and_then(|connection| connection.send(ClientCommand::Stop(id.clone())));
        match result {
            Ok(()) => {
                self.stopping = true;
                self.log("Stopping process…\n", OutputStream::Stdout, cx);
            }
            Err(error) => self.show_error(error, cx),
        }
        cx.notify();
    }

    /// Keep execution controls together, independent of graph/view settings.
    fn execution_toolbar(&self, cx: &mut Context<Self>) -> Div {
        let running = self.active.is_some() || self.launch_pending;
        toolbar_group()
            .child(
                button("runtime-configurations", "Configuration…")
                    .when(self.show_configurations, selected_button)
                    .on_click(cx.listener(|this, _, _, cx| {
                        this.show_configurations = !this.show_configurations;
                        cx.notify();
                    })),
            )
            .child(
                button("runtime-build", "Build")
                    .when(running, |button| button.opacity(0.5))
                    .on_click(cx.listener(|this, _, _, cx| this.launch(Operation::Build, cx))),
            )
            .child(
                button("runtime-run", "Run")
                    .bg(rgb(theme::SUCCESS_BACKGROUND))
                    .text_color(rgb(theme::SUCCESS))
                    .hover(|button| button.bg(rgb(theme::SUCCESS_HOVER)))
                    .when(running, |button| button.opacity(0.5))
                    .on_click(cx.listener(|this, _, _, cx| this.launch(Operation::Run, cx))),
            )
            .child(
                button(
                    "runtime-stop",
                    if self.stopping { "Stopping…" } else { "Stop" },
                )
                .bg(rgb(theme::ERROR_BACKGROUND))
                .text_color(rgb(theme::ERROR))
                .hover(|button| button.bg(rgb(theme::ERROR_HOVER)))
                .when(!running, |button| button.opacity(0.5))
                .on_click(cx.listener(|this, _, _, cx| this.stop(cx))),
            )
    }

    fn runtime_toolbar(&self, cx: &mut Context<Self>) -> Div {
        let health_alerts = if self.discovery {
            self.health_watches()
                .iter()
                .filter(|watch| {
                    self.health_snapshot
                        .get(&watch.topic, Instant::now())
                        .is_some_and(|report| {
                            matches!(
                                report.state,
                                ros_studio_protocol::health::TopicHealthState::Missing
                                    | ros_studio_protocol::health::TopicHealthState::Degraded
                            )
                        })
                })
                .count()
        } else {
            0
        };
        div()
            .flex()
            .flex_wrap()
            .items_center()
            .flex_shrink_0()
            .gap_3()
            .px_4()
            .py_2()
            .bg(rgb(theme::SURFACE))
            .border_t_1()
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .child(
                toolbar_group().children(
                    [GraphMode::Design, GraphMode::Live, GraphMode::Both]
                        .into_iter()
                        .enumerate()
                        .map(|(index, mode)| {
                            button(("mode", index), mode.label())
                                .when(self.mode == mode, selected_button)
                                .on_click(cx.listener(move |this, _, _, cx| {
                                    this.mode = mode;
                                    this.rebuild_graph(cx);
                                    if mode != GraphMode::Design && this.connection.is_none() {
                                        this.connect_runtime(cx);
                                    }
                                }))
                        }),
                ),
            )
            .child(
                div()
                    .flex()
                    .flex_wrap()
                    .items_center()
                    .gap_1()
                    .child(
                        button(
                            "runtime-environment",
                            format!("Env: {}", self.preference.label()),
                        )
                        .on_click(cx.listener(|this, _, _, cx| {
                            if this.connection.is_some() {
                                this.show_error(
                                    anyhow::anyhow!(
                                        "Disconnect before changing the runtime environment"
                                    ),
                                    cx,
                                );
                            } else {
                                this.preference = match this.preference {
                                    RuntimePreference::Auto => RuntimePreference::Host,
                                    RuntimePreference::Host => RuntimePreference::Docker,
                                    RuntimePreference::Docker => RuntimePreference::Auto,
                                };
                                this.error = None;
                                this.save_preferences(cx);
                                cx.notify();
                            }
                        })),
                    )
                    .child(
                        button(
                            "runtime-connect",
                            if self.connection.is_some() {
                                "Disconnect"
                            } else {
                                "Connect"
                            },
                        )
                        .on_click(cx.listener(|this, _, _, cx| {
                            if this.connection.is_some() {
                                this.disconnect(cx);
                                this.log(
                                    "Runtime session closing; its processes will be stopped.\n",
                                    OutputStream::Stdout,
                                    cx,
                                );
                            } else {
                                this.connect_runtime(cx);
                            }
                        })),
                    )
                    .child(
                        button(
                            "runtime-auto-mode",
                            if self.auto_mode {
                                "Auto: on"
                            } else {
                                "Auto: off"
                            },
                        )
                        .when(self.auto_mode, selected_button)
                        .on_click(cx.listener(|this, _, _, cx| {
                            this.auto_mode = !this.auto_mode;
                            this.save_preferences(cx);
                            cx.notify();
                        })),
                    ),
            )
            .child(
                div()
                    .flex_1()
                    .min_w(px(100.0))
                    .truncate()
                    .text_xs()
                    .text_color(rgb(theme::MUTED))
                    .child(self.connection_label.clone()),
            )
            .child(
                toolbar_group()
                    .child(
                        button(
                            "runtime-make-panel",
                            format!("Make ({})", self.make_catalog.targets.len()),
                        )
                        .when(self.show_make_panel, selected_button)
                        .on_click(cx.listener(|this, _, _, cx| {
                            this.show_make_panel = !this.show_make_panel;
                            cx.notify();
                        })),
                    )
                    .child(
                        button("runtime-console", "Console")
                            .when(self.console_visible, selected_button)
                            .on_click(cx.listener(|this, _, _, cx| {
                                this.console_visible = !this.console_visible;
                                cx.notify();
                            })),
                    )
                    .child(
                        button(
                            "runtime-health",
                            if health_alerts == 0 {
                                "Health".to_owned()
                            } else {
                                format!("Health · {health_alerts} alerts")
                            },
                        )
                        .when(
                            self.inspector_view == InspectorView::Health,
                            selected_button,
                        )
                        .when(health_alerts > 0, |button| {
                            button
                                .bg(rgb(theme::ERROR_BACKGROUND))
                                .text_color(rgb(theme::ERROR))
                        })
                        .on_click(cx.listener(|this, _, _, cx| {
                            this.inspector_view = if this.inspector_view == InspectorView::Health {
                                InspectorView::Selection
                            } else {
                                InspectorView::Health
                            };
                            cx.notify();
                        })),
                    ),
            )
            .child(
                toolbar_group()
                    .child(
                        button("zoom-out", "−")
                            .on_click(cx.listener(|this, _, _, cx| this.zoom_by(1.0 / 1.2, cx))),
                    )
                    .child(
                        button("zoom-in", "+")
                            .on_click(cx.listener(|this, _, _, cx| this.zoom_by(1.2, cx))),
                    )
                    .child(
                        button("fit", "Fit graph")
                            .on_click(cx.listener(|this, _, _, cx| this.fit(cx))),
                    ),
            )
    }

    fn make_panel(&self, cx: &mut Context<Self>) -> Div {
        let busy = self.active.is_some() || self.launch_pending;
        let query = self.make_filter.read(cx).text().to_owned();
        let image_index = self
            .make_catalog
            .targets
            .iter()
            .position(|target| target.target == "image")
            .filter(|index| self.make_catalog.targets[*index].matches_filter(&query));
        let groups = self
            .make_catalog
            .filtered_groups(&query)
            .into_iter()
            .filter_map(|mut group| {
                group
                    .target_indices
                    .retain(|index| self.make_catalog.targets[*index].target != "image");
                (!group.target_indices.is_empty()).then_some(group)
            })
            .collect::<Vec<_>>();
        let mut panel = div()
            .flex()
            .flex_col()
            .min_h_0()
            .w(px(320.0))
            .flex_shrink_0()
            .bg(rgb(theme::SURFACE))
            .border_r_1()
            .border_color(rgb(theme::BORDER))
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .p_3()
                    .flex_shrink_0()
                    .child(div().flex_1().text_lg().child("Make commands"))
                    .child(button("make-panel-close", "Close").on_click(cx.listener(
                        |this, _, _, cx| {
                            this.show_make_panel = false;
                            cx.notify();
                        },
                    ))),
            )
            .child(
                div()
                    .px_3()
                    .pb_3()
                    .flex_shrink_0()
                    .text_sm()
                    .text_color(rgb(theme::MUTED))
                    .child(if cfg!(target_os = "windows") {
                        "Inspection only on Windows · use a node/file configuration with Docker"
                    } else {
                        "Detected from this project · execution on Host"
                    }),
            )
            .child(
                div()
                    .px_3()
                    .pb_3()
                    .flex_shrink_0()
                    .child(self.make_filter.clone()),
            );
        if let Some(index) = image_index {
            panel = panel.child(
                div()
                    .px_3()
                    .pb_3()
                    .flex_shrink_0()
                    .child(self.make_row(index, busy, cx)),
            );
        }
        if self.make_catalog.targets.is_empty() {
            panel = panel.child(div().p_3().text_sm().child(if self.loading {
                "Scanning Makefiles…"
            } else {
                "No targets found in the root GNUmakefile, makefile, or Makefile."
            }));
        } else if groups.is_empty() && image_index.is_none() {
            panel = panel.child(
                div()
                    .p_3()
                    .text_sm()
                    .child("No Make commands match your search."),
            );
        }
        panel = panel.child(
            div()
                .id("make-panel-targets")
                .flex()
                .flex_col()
                .flex_1()
                .min_h_0()
                .overflow_y_scroll()
                .px_3()
                .pb_3()
                .gap_2()
                .children(groups.into_iter().enumerate().map(|(index, group)| {
                    // Search exposes matching commands even in collapsed groups.
                    let collapsed = query.trim().is_empty()
                        && !self.expanded_make_groups.contains(&group.package);
                    let label = format!(
                        "{} {} ({})",
                        if collapsed { "▸" } else { "▾" },
                        group.label(),
                        group.target_indices.len()
                    );
                    let key = group.package.clone();
                    div()
                        .flex()
                        .flex_col()
                        .gap_2()
                        .flex_shrink_0()
                        .child(
                            button(("make-panel-group", index), label)
                                .w_full()
                                .on_click(cx.listener(move |this, _, _, cx| {
                                    if !this.expanded_make_groups.remove(&key) {
                                        this.expanded_make_groups.insert(key.clone());
                                    }
                                    cx.notify();
                                })),
                        )
                        .when(!collapsed, |section| {
                            section.children(
                                group
                                    .target_indices
                                    .into_iter()
                                    .map(|index| self.make_row(index, busy, cx)),
                            )
                        })
                })),
        );
        if self.running_make.is_some() {
            panel = panel.child(
                div().p_3().flex_shrink_0().child(
                    button(
                        "make-panel-stop",
                        if self.stopping {
                            "Stopping…"
                        } else {
                            "Stop Make"
                        },
                    )
                    .on_click(cx.listener(|this, _, _, cx| this.stop(cx))),
                ),
            );
        } else if busy {
            panel = panel.child(
                div()
                    .p_3()
                    .text_sm()
                    .flex_shrink_0()
                    .child("Stop the current process before running a Make command."),
            );
        }
        panel.child(div().p_3().text_xs().flex_shrink_0().text_color(rgb(theme::WARNING))
            .child("Only run trusted commands: Make recipes can start Docker, multiple nodes, or real hardware. Closing this panel does not stop the task."))
    }

    fn make_row(&self, index: usize, busy: bool, cx: &mut Context<Self>) -> Div {
        let busy = busy || !cfg!(unix);
        let target = self.make_catalog.targets[index].clone();
        let makefile = target.makefile.clone();
        let running = self.running_make.as_ref() == Some(&target);
        let label = if target.target == "image" {
            "make image".to_owned()
        } else {
            target.target.clone()
        };
        div()
            .flex()
            .flex_col()
            .gap_1()
            .p_2()
            .rounded_md()
            .border_1()
            .border_color(if running {
                rgb(theme::ACCENT)
            } else {
                rgb(theme::BORDER)
            })
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(
                        div()
                            .flex_1()
                            .min_w_0()
                            .truncate()
                            .text_sm()
                            .font_family("monospace")
                            .child(label),
                    )
                    .child(
                        button(
                            ("make-panel-run", index),
                            if !cfg!(unix) {
                                "Unavailable"
                            } else if running {
                                "Running"
                            } else {
                                "Run"
                            },
                        )
                        .when(busy, |button| button.opacity(0.5))
                        .on_click(cx.listener(move |this, _, _, cx| {
                            if cfg!(unix) && this.active.is_none() && !this.launch_pending {
                                this.launch_target(
                                    RunTarget::Make(target.clone()),
                                    Operation::Run,
                                    false,
                                    cx,
                                );
                            }
                        })),
                    ),
            )
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::MUTED))
                    .child(makefile),
            )
    }

    fn configuration_picker(&self, cx: &mut Context<Self>) -> Div {
        let mut panel = div()
            .flex()
            .flex_col()
            .gap_2()
            .p_4()
            .bg(rgb(theme::ELEVATED))
            .child(
                div()
                    .text_lg()
                    .child("Run configuration · Shift+F10 to run · Alt+Shift+F10 to choose"),
            );
        if let Some(project) = &self.design_project {
            panel = panel.child(div().text_sm().child("ROS nodes")).child(
                div()
                    .id("configuration-list")
                    .max_h(px(220.0))
                    .overflow_y_scroll()
                    .children(project.nodes.iter().enumerate().map(|(index, node)| {
                        let target = RunTarget::Node(node.id.as_str().into());
                        button(
                            ("configuration-node", index),
                            format!("ROS node: {} · {}", node.logical_name, node.executable),
                        )
                        .when(
                            RunTarget::for_selection(
                                self.selected_node.as_deref(),
                                self.run_target.as_ref(),
                            )
                            .as_ref()
                                == Some(&target),
                            |button| button.bg(rgb(theme::SELECTED)),
                        )
                        .on_click(cx.listener(move |this, _, _, cx| {
                            this.set_configuration(target.clone(), cx)
                        }))
                    })),
            );
        }
        if cfg!(unix) && !self.make_catalog.targets.is_empty() {
            panel = panel
                .child(div().text_sm().child("Makefile targets · run on host · Build/Run executes the chosen target"))
                .child(div().text_sm().text_color(rgb(theme::WARNING))
                    .child("Only run trusted targets. Recipes may start Docker, hardware, or multiple nodes."))
                .child(div().id("configuration-make-list").flex().flex_col()
                    .max_h(px(180.0)).overflow_y_scroll()
                    .children(self.make_catalog.targets.iter().enumerate().map(|(index, make)| {
                        let target = RunTarget::Make(make.clone());
                        button(("configuration-make", index), format!("make {} · {}", make.target, make.makefile))
                            .when(RunTarget::for_selection(self.selected_node.as_deref(), self.run_target.as_ref()).as_ref() == Some(&target),
                                |button| button.bg(rgb(theme::SELECTED)))
                            .on_click(cx.listener(move |this, _, _, cx| this.set_configuration(target.clone(), cx)))
                    })));
        }
        panel.child(
            div()
                .flex()
                .gap_2()
                .child(
                    button("configuration-file", "Choose source file…")
                        .on_click(cx.listener(|this, _, _, cx| this.choose_source(cx))),
                )
                .child(
                    button("configuration-cancel", "Close").on_click(cx.listener(
                        |this, _, _, cx| {
                            this.show_configurations = false;
                            cx.notify();
                        },
                    )),
                ),
        )
    }

    fn configuration_label(&self) -> String {
        match RunTarget::for_selection(self.selected_node.as_deref(), self.run_target.as_ref()) {
            Some(RunTarget::File(path)) => format!("File: {path}"),
            Some(RunTarget::Make(target)) => {
                format!("Make: {} · {} (host)", target.target, target.makefile)
            }
            Some(RunTarget::Node(id)) => self
                .design_project
                .as_ref()
                .and_then(|project| {
                    project
                        .nodes
                        .iter()
                        .find(|node| node.id.as_str() == id.as_str())
                })
                .map(|node| format!("Node: {}", node.logical_name))
                .unwrap_or_else(|| "Configuration unavailable; choose another".into()),
            None => "No run configuration selected".into(),
        }
    }

    fn watch_events(events: Receiver<WatchEvent>, cx: &mut Context<Self>) -> Task<()> {
        cx.spawn(async move |this, cx| {
            while let Ok(event) = events.recv().await {
                let mut watcher_error = match event {
                    WatchEvent::Changed => None,
                    WatchEvent::Failed(error) => Some(error),
                };
                cx.background_executor()
                    .timer(Duration::from_millis(250))
                    .await;
                while let Ok(event) = events.try_recv() {
                    if let WatchEvent::Failed(error) = event {
                        watcher_error = Some(error);
                    }
                }
                if let Err(error) = this.update(cx, |this, cx| {
                    if let Some(error) = watcher_error {
                        this.show_error(
                            anyhow::anyhow!("File watching: {error}. Use Refresh."),
                            cx,
                        );
                    } else {
                        this.refresh(cx);
                    }
                }) {
                    eprintln!("Cannot refresh graph: {error:#}");
                    break;
                }
            }
        })
    }

    fn show_error(&mut self, error: anyhow::Error, cx: &mut Context<Self>) {
        self.error = Some(format!("{error:#}"));
        cx.notify();
    }

    fn open_source(&mut self, source: SourceLocation, cx: &mut Context<Self>) {
        let Some(snapshot) = &self.snapshot else {
            return;
        };
        let command = match self.editor.source_command(&snapshot.root, &source) {
            Ok(command) => command,
            Err(error) => {
                self.show_error(error, cx);
                return;
            }
        };
        let label = self.editor.label();
        self.error = None;
        self.status = format!("Opening {} in {label}…", source.path);
        let launch = cx.background_spawn(async move {
            let output = async_process::Command::new(&command.program)
                .args(command.arguments)
                .current_dir(command.working_directory)
                .stdin(Stdio::null())
                .output().await
                .with_context(|| format!("Cannot start {label}. Make its launcher available in PATH or choose another editor."))?;
            ensure!(output.status.success(), "{label} exited with {}: {}", output.status,
                String::from_utf8_lossy(&output.stderr).trim());
            anyhow::Ok(())
        });
        cx.spawn(async move |this, cx| {
            let result = launch.await;
            if let Err(error) = this.update(cx, |this, cx| {
                match result {
                    Ok(()) => this.status = format!("Source opened in {label}"),
                    Err(error) => this.show_error(error, cx),
                }
                cx.notify();
            }) {
                eprintln!("Editor launch: {error:#}");
            }
        })
        .detach();
        cx.notify();
    }

    fn frame_project(&mut self, cx: &mut Context<Self>) {
        if let Some((snapshot, bounds)) = self.snapshot.as_ref().zip(self.canvas_bounds) {
            if snapshot.positions.is_empty()
                || bounds.size.width <= px(0.0)
                || bounds.size.height <= px(0.0)
            {
                return;
            }
            self.viewport.open_project(
                &snapshot.positions,
                bounds.size.width.as_f32(),
                bounds.size.height.as_f32(),
            );
            self.initial_view_pending = false;
            cx.notify();
        }
    }

    fn fit(&mut self, cx: &mut Context<Self>) {
        if let Some((snapshot, bounds)) = self.snapshot.as_ref().zip(self.canvas_bounds) {
            self.viewport.fit(
                &snapshot.positions,
                bounds.size.width.as_f32(),
                bounds.size.height.as_f32(),
            );
            self.initial_view_pending = false;
            cx.notify();
        }
    }

    fn zoom_by(&mut self, factor: f32, cx: &mut Context<Self>) {
        if let Some(bounds) = self.canvas_bounds {
            self.viewport.zoom_at(
                self.viewport.scale * factor,
                bounds.size.width.as_f32() / 2.0,
                bounds.size.height.as_f32() / 2.0,
            );
            cx.notify();
        }
    }

    fn mouse_move(&mut self, event: &MouseMoveEvent, window: &mut Window, cx: &mut Context<Self>) {
        if !event.dragging() {
            if self.drag.take().is_some() {
                cx.notify();
            }
            return;
        }
        let Some(drag) = &self.drag else {
            return;
        };
        // Canvas events bubble to the root too. Handle each movement only once.
        cx.stop_propagation();
        let mouse = (event.position.x.as_f32(), event.position.y.as_f32());
        let previous_viewport = self.viewport;
        let previous_console_height = self.console_height;
        let mut node_moved = false;
        match drag {
            Drag::Console { mouse_y, height } => {
                self.console_height = (height + mouse_y - mouse.1).clamp(
                    120.0,
                    (window.viewport_size().height.as_f32() * 0.65).max(120.0),
                );
            }
            Drag::Canvas {
                mouse: start,
                offset,
            } => {
                self.viewport.x = offset.0 + mouse.0 - start.0;
                self.viewport.y = offset.1 + mouse.1 - start.1;
            }
            Drag::Node {
                node_id,
                mouse: start,
                position,
            } => {
                if let Some(layout) = self.snapshot.as_mut().and_then(|snapshot| {
                    snapshot
                        .positions
                        .iter_mut()
                        .find(|layout| &layout.node_id == node_id)
                }) {
                    let x = position.0 + (mouse.0 - start.0) / self.viewport.scale;
                    let y = position.1 + (mouse.1 - start.1) / self.viewport.scale;
                    node_moved = layout.x != x || layout.y != y;
                    layout.x = x;
                    layout.y = y;
                }
            }
        }
        if node_moved
            || self.viewport != previous_viewport
            || self.console_height != previous_console_height
        {
            cx.notify();
        }
    }

    fn mouse_up(&mut self, _: &MouseUpEvent, _: &mut Window, cx: &mut Context<Self>) {
        let drag = self.drag.take();
        if matches!(drag, Some(Drag::Console { .. })) {
            self.save_preferences(cx);
        }
        if drag.is_some() {
            cx.notify();
        }
    }

    fn scroll(&mut self, event: &ScrollWheelEvent, _: &mut Window, cx: &mut Context<Self>) {
        let Some(bounds) = self.canvas_bounds else {
            return;
        };
        let delta = match event.delta {
            ScrollDelta::Pixels(delta) => delta.y.as_f32(),
            ScrollDelta::Lines(delta) => delta.y * 10.0,
        };
        if delta == 0.0 {
            return;
        }
        let factor = (delta.clamp(-40.0, 40.0) * 0.01).exp();
        self.viewport.zoom_at(
            self.viewport.scale * factor,
            (event.position.x - bounds.origin.x).as_f32(),
            (event.position.y - bounds.origin.y).as_f32(),
        );
        cx.stop_propagation();
        cx.notify();
    }

    fn inspector(&self, cx: &mut Context<Self>) -> Div {
        if self.inspector_view == InspectorView::Health {
            return self.health_panel(cx);
        }
        let mut panel = div()
            .flex()
            .flex_col()
            .gap_3()
            .p_4()
            .w(px(280.0))
            .flex_shrink_0()
            .bg(rgb(theme::SURFACE))
            .border_l_1()
            .border_color(rgb(theme::BORDER))
            .child(div().text_lg().child("Inspector"));
        if self.inspector_view != InspectorView::Selection {
            panel = panel.child(
                button("inspector-back", "Selected node").on_click(cx.listener(
                    |this, _, _, cx| {
                        this.inspector_view = InspectorView::Selection;
                        cx.notify();
                    },
                )),
            );
            if let Some(snapshot) = &self.snapshot {
                let mut list = div()
                    .id("inspector-list")
                    .flex_1()
                    .min_h_0()
                    .overflow_y_scroll();
                match self.inspector_view {
                    InspectorView::Packages => {
                        for package in &snapshot.project.packages {
                            list = list.child(
                                div()
                                    .py_2()
                                    .border_b_1()
                                    .border_color(rgb(theme::BORDER))
                                    .child(package.name.clone())
                                    .child(div().text_xs().child(package.path.clone())),
                            );
                        }
                    }
                    InspectorView::Nodes => {
                        for (index, node) in snapshot.project.nodes.iter().enumerate() {
                            let id = node.id.as_str().to_owned();
                            list = list.child(
                                button(("inspector-node", index), node.logical_name.clone())
                                    .on_click(cx.listener(move |this, _, _, cx| {
                                        this.selected_node = Some(id.clone());
                                        this.inspector_view = InspectorView::Selection;
                                        cx.notify();
                                    })),
                            );
                        }
                    }
                    InspectorView::Topics => {
                        let topics = snapshot
                            .project
                            .nodes
                            .iter()
                            .flat_map(|node| &node.endpoints)
                            .filter(|endpoint| {
                                matches!(
                                    endpoint.kind,
                                    EndpointKind::Publisher | EndpointKind::Subscription
                                )
                            })
                            .map(|endpoint| (endpoint.name.as_str(), endpoint.type_name.as_str()))
                            .collect::<std::collections::BTreeSet<_>>();
                        for (name, type_name) in topics {
                            list = list.child(
                                div()
                                    .py_2()
                                    .border_b_1()
                                    .border_color(rgb(theme::BORDER))
                                    .child(name.to_owned())
                                    .child(div().text_xs().child(type_name.to_owned())),
                            );
                        }
                    }
                    InspectorView::Selection | InspectorView::Health => {}
                }
                return panel.child(list);
            }
        }
        let Some(node) = self.snapshot.as_ref().and_then(|snapshot| {
            snapshot
                .project
                .nodes
                .iter()
                .find(|node| Some(node.id.as_str()) == self.selected_node.as_deref())
        }) else {
            return panel.child(div().text_sm().text_color(rgb(theme::MUTED))
                .child("Select a node to inspect its interfaces. Double-click it to open the source in your editor."));
        };
        panel = panel
            .child(div().text_lg().truncate().child(node.logical_name.clone()))
            .child(
                div()
                    .text_sm()
                    .child(format!("Executable: {}", node.executable)),
            )
            .child(
                div()
                    .text_sm()
                    .child(format!("State: {:?}", node.runtime_state)),
            )
            .child(
                button("inspector-use-configuration", "Use as run configuration").on_click(
                    cx.listener({
                        let id = node.id.as_str().to_owned();
                        move |this, _, _, cx| {
                            this.set_configuration(RunTarget::Node(id.clone()), cx)
                        }
                    }),
                ),
            )
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::MUTED))
                    .child("SOURCE"),
            );
        for (index, source) in node.source_locations.iter().enumerate() {
            let source = source.clone();
            panel = panel.child(
                button(
                    ("source", index),
                    format!("{}:{}", source.path, source.line.saturating_add(1)),
                )
                .on_click(cx.listener(move |this, _, _, cx| this.open_source(source.clone(), cx))),
            );
        }
        panel
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::MUTED))
                    .child("INTERFACES"),
            )
            .child(
                div()
                    .id("interfaces")
                    .flex_1()
                    .min_h_0()
                    .overflow_y_scroll()
                    .children(node.endpoints.iter().enumerate().map(|(index, endpoint)| {
                        div()
                            .flex()
                            .flex_col()
                            .gap_1()
                            .py_2()
                            .border_b_1()
                            .border_color(rgb(theme::BORDER))
                            .child(format!("{:?} · {}", endpoint.kind, endpoint.name))
                            .child(
                                div()
                                    .text_xs()
                                    .text_color(rgb(theme::MUTED))
                                    .child(endpoint.type_name.clone()),
                            )
                            .child(
                                div()
                                    .text_xs()
                                    .text_color(rgb(theme::MUTED))
                                    .child(format!("Confidence: {:?}", endpoint.confidence)),
                            )
                            .children(endpoint.source_location.as_ref().map(|source| {
                                let source = source.clone();
                                button(
                                    ("endpoint-source", index),
                                    format!("{}:{}", source.path, source.line.saturating_add(1)),
                                )
                                .on_click(cx.listener(
                                    move |this, _, _, cx| this.open_source(source.clone(), cx),
                                ))
                            }))
                    })),
            )
    }

    fn graph_canvas(&self, cx: &mut Context<Self>) -> Stateful<Div> {
        let viewport = self.viewport;
        let graph_entity = cx.entity();
        let lines = self
            .snapshot
            .as_ref()
            .map(|snapshot| {
                let positions = snapshot
                    .positions
                    .iter()
                    .map(|position| (position.node_id.as_str(), position))
                    .collect::<BTreeMap<_, _>>();
                snapshot
                    .edges
                    .iter()
                    .filter_map(|edge| {
                        if edge.publisher_id == edge.subscriber_id {
                            return None;
                        }
                        let publisher = positions.get(edge.publisher_id.as_str())?;
                        let subscriber = positions.get(edge.subscriber_id.as_str())?;
                        let (start, end) = edge_points(publisher, subscriber);
                        Some((start, end, edge.topic.clone()))
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let paint_lines = lines.clone();
        let mut graph = div()
            .id("graph-canvas")
            .relative()
            .flex_1()
            .min_w_0()
            .min_h_0()
            .overflow_hidden()
            .bg(rgb(theme::CANVAS))
            .cursor_grab()
            .on_mouse_down(
                MouseButton::Right,
                cx.listener(|this, event: &MouseDownEvent, window, cx| {
                    window.focus(&this.focus, cx);
                    this.drag = None;
                    this.context_menu = Some(point(
                        event
                            .position
                            .x
                            .min(window.viewport_size().width - px(240.0))
                            .max(px(0.0)),
                        event
                            .position
                            .y
                            .min(window.viewport_size().height - px(180.0))
                            .max(px(0.0)),
                    ));
                    cx.stop_propagation();
                    cx.notify();
                }),
            )
            .when(self.drag.is_some(), |graph| graph.cursor_grabbing())
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(|this, event: &MouseDownEvent, _, cx| {
                    this.selected_node = None;
                    this.inspector_view = InspectorView::Selection;
                    this.drag = Some(Drag::Canvas {
                        mouse: (event.position.x.as_f32(), event.position.y.as_f32()),
                        offset: (this.viewport.x, this.viewport.y),
                    });
                    cx.notify();
                }),
            )
            .on_mouse_move(cx.listener(Self::mouse_move))
            .on_mouse_up(MouseButton::Left, cx.listener(Self::mouse_up))
            .on_mouse_up_out(MouseButton::Left, cx.listener(Self::mouse_up))
            .on_scroll_wheel(cx.listener(Self::scroll))
            .child(
                canvas(
                    move |bounds, _, cx| {
                        graph_entity.update(cx, |this, cx| {
                            this.canvas_bounds = Some(bounds);
                            if this.initial_view_pending {
                                this.frame_project(cx);
                            }
                        });
                    },
                    move |bounds, _, window, _| {
                        if let Err(error) = paint_graph(bounds, viewport, &paint_lines, window) {
                            eprintln!("Cannot paint graph: {error:#}");
                        }
                    },
                )
                .size_full(),
            );
        for (index, (start, end, topic)) in lines.iter().enumerate() {
            let (x, y) = viewport.screen_point((start.0 + end.0) / 2.0, (start.1 + end.1) / 2.0);
            let health = (self.discovery && self.mode != GraphMode::Design)
                .then(|| self.health_snapshot.get(topic, Instant::now()))
                .flatten();
            graph = graph.child(
                div()
                    .id(("edge", index))
                    .absolute()
                    .left(px(x - 100.0 * viewport.scale))
                    .top(px(y - 30.0 * viewport.scale))
                    .w(px(200.0 * viewport.scale))
                    .px(px(8.0 * viewport.scale))
                    .py(px(4.0 * viewport.scale))
                    .rounded_md()
                    .bg(rgb(theme::ELEVATED))
                    .text_size(px(14.0 * viewport.scale))
                    .text_color(rgb(health.map_or(theme::ACCENT, |report| {
                        health_view::state_color(report.state)
                    })))
                    .text_center()
                    .truncate()
                    .child(topic.clone())
                    .when_some(health, |label, report| {
                        label.child(div().text_size(px(10.0 * viewport.scale)).child(format!(
                                "{} · {} Hz",
                                report.state.label(),
                                report
                                    .observed_hz
                                    .map_or_else(|| "—".into(), |hz| format!("{hz:.1}"))
                            )))
                    }),
            );
        }
        if let Some(snapshot) = &self.snapshot {
            for (index, node) in snapshot.project.nodes.iter().enumerate() {
                let Some(position) = snapshot
                    .positions
                    .iter()
                    .find(|position| position.node_id == node.id.as_str())
                else {
                    continue;
                };
                let (x, y) = viewport.screen_point(position.x, position.y);
                let node_id = node.id.as_str().to_owned();
                let health = (self.discovery && self.mode != GraphMode::Design)
                    .then(|| {
                        self.health_snapshot.node_state(
                            &snapshot.project,
                            node,
                            self.health_watches(),
                            Instant::now(),
                        )
                    })
                    .flatten();
                let source = node.source_locations.first().cloned();
                let package = snapshot
                    .project
                    .packages
                    .iter()
                    .find(|package| package.id == node.package_id)
                    .map(|package| package.name.as_str())
                    .unwrap_or("Unknown package");
                let position = (position.x, position.y);
                let drag_node_id = node_id.clone();
                graph = graph.child(
                    div()
                        .id(("node", index))
                        .absolute()
                        .left(px(x))
                        .top(px(y))
                        .w(px(NODE_WIDTH * viewport.scale))
                        .h(px(NODE_HEIGHT * viewport.scale))
                        .p(px(12.0 * viewport.scale))
                        .flex()
                        .flex_col()
                        .gap(px(8.0 * viewport.scale))
                        .rounded_md()
                        .border_1()
                        .overflow_hidden()
                        .cursor_default()
                        .bg(rgb(if self.selected_node.as_deref() == Some(&node_id) {
                            theme::SELECTED
                        } else {
                            theme::SURFACE
                        }))
                        .border_color(rgb(if self.selected_node.as_deref() == Some(&node_id) {
                            theme::ACCENT
                        } else {
                            theme::BORDER
                        }))
                        .text_size(px(14.0 * viewport.scale))
                        .child(
                            div()
                                .flex()
                                .items_center()
                                .justify_between()
                                .gap_2()
                                .child(
                                    div()
                                        .flex_1()
                                        .min_w_0()
                                        .truncate()
                                        .child(node.logical_name.clone()),
                                )
                                .child(
                                    div()
                                        .text_size(px(10.0 * viewport.scale))
                                        .text_color(rgb(health.map_or_else(
                                            || match node.runtime_state {
                                                RuntimeState::Online => theme::SUCCESS,
                                                RuntimeState::Offline => theme::WARNING,
                                                RuntimeState::Conflict => theme::ERROR,
                                                RuntimeState::RuntimeOnly
                                                | RuntimeState::Unknown => theme::ACCENT,
                                            },
                                            health_view::state_color,
                                        )))
                                        .child(health.map_or_else(
                                            || {
                                                match node.runtime_state {
                                                    RuntimeState::Online => "ONLINE",
                                                    RuntimeState::Offline => "OFFLINE",
                                                    RuntimeState::RuntimeOnly => "RUNTIME",
                                                    RuntimeState::Conflict => "CONFLICT",
                                                    RuntimeState::Unknown => "DESIGN",
                                                }
                                                .to_owned()
                                            },
                                            |state| state.label().to_owned(),
                                        )),
                                ),
                        )
                        .child(
                            div()
                                .truncate()
                                .text_size(px(11.0 * viewport.scale))
                                .text_color(rgb(theme::MUTED))
                                .child(format!("{package} · {}", node.executable)),
                        )
                        .child(
                            div()
                                .border_t_1()
                                .border_color(rgb(theme::BORDER))
                                .pt(px(8.0 * viewport.scale))
                                .children(node.endpoints.iter().take(4).map(|endpoint| {
                                    let direction = match endpoint.kind {
                                        ros_studio_model::EndpointKind::Publisher => "OUT",
                                        ros_studio_model::EndpointKind::Subscription => "IN",
                                        _ => "API",
                                    };
                                    div()
                                        .truncate()
                                        .text_size(px(11.0 * viewport.scale))
                                        .child(format!("{direction}  {}", endpoint.name))
                                })),
                        )
                        .on_mouse_down(
                            MouseButton::Left,
                            cx.listener(move |this, event: &MouseDownEvent, window, cx| {
                                window.focus(&this.focus, cx);
                                this.selected_node = Some(drag_node_id.clone());
                                this.inspector_view = InspectorView::Selection;
                                this.context_menu = None;
                                this.drag = Some(Drag::Node {
                                    node_id: drag_node_id.clone(),
                                    mouse: (event.position.x.as_f32(), event.position.y.as_f32()),
                                    position,
                                });
                                cx.stop_propagation();
                                cx.notify();
                            }),
                        )
                        .on_click(cx.listener(move |this, event: &ClickEvent, _, cx| {
                            cx.stop_propagation();
                            if event.click_count() == 2 {
                                if let Some(source) = &source {
                                    this.open_source(source.clone(), cx);
                                } else {
                                    this.show_error(
                                        anyhow::anyhow!("This node has no source location"),
                                        cx,
                                    );
                                }
                            }
                        })),
                );
            }
            if snapshot.project.nodes.is_empty() {
                graph = graph.child(
                    div()
                        .absolute()
                        .top_8()
                        .left_8()
                        .text_color(rgb(theme::MUTED))
                        .child("No nodes detected in this folder."),
                );
            }
        } else {
            graph = graph.child(
                div()
                    .absolute()
                    .top_8()
                    .left_8()
                    .text_color(rgb(theme::MUTED))
                    .child("Open a ROS project. Your source code stays in your preferred editor."),
            );
        }
        graph
    }
}

impl Render for GraphView {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let summary = self
            .snapshot
            .as_ref()
            .map(|snapshot| snapshot.project.name.clone())
            .unwrap_or_else(|| "No project open".to_owned());
        div()
            .id("ros-studio-root")
            .track_focus(&self.focus)
            .relative()
            .flex()
            .flex_col()
            .size_full()
            .bg(rgb(theme::BACKGROUND))
            .text_color(rgb(theme::TEXT))
            .font_family("sans-serif")
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(|this, _, window, cx| {
                    if this.wizard.is_none() {
                        window.focus(&this.focus, cx);
                    }
                    if this.context_menu.take().is_some() {
                        cx.notify();
                    }
                }),
            )
            .on_key_down(cx.listener(|this, event: &gpui::KeyDownEvent, _, cx| {
                if this.wizard.is_some() {
                    return;
                }
                let key = &event.keystroke;
                if key.key == "f10" && key.modifiers.shift && key.modifiers.alt {
                    this.show_configurations = !this.show_configurations;
                } else if key.key == "f10" && key.modifiers.shift {
                    this.launch(Operation::Run, cx);
                } else if key.key == "escape" {
                    this.context_menu = None;
                    this.show_configurations = false;
                    this.show_make_panel = false;
                } else {
                    return;
                }
                cx.stop_propagation();
                cx.notify();
            }))
            .on_mouse_move(cx.listener(Self::mouse_move))
            .on_mouse_up(MouseButton::Left, cx.listener(Self::mouse_up))
            .on_mouse_up_out(MouseButton::Left, cx.listener(Self::mouse_up))
            .child(
                div()
                    .flex()
                    .flex_wrap()
                    .items_center()
                    .px_4()
                    .py_3()
                    .gap_3()
                    .flex_shrink_0()
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .gap_1()
                            .flex_1()
                            .min_w(px(220.0))
                            .child(
                                div()
                                    .text_sm()
                                    .truncate()
                                    .text_color(rgb(theme::MUTED))
                                    .child(summary),
                            ),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_wrap()
                            .items_center()
                            .gap_1()
                            .child(
                                button("open-project", "Open project…").on_click(
                                    cx.listener(|this, _, _, cx| this.choose_project(cx)),
                                ),
                            )
                            .child(
                                button(
                                    "refresh",
                                    if self.loading {
                                        "Scanning…"
                                    } else {
                                        "Refresh"
                                    },
                                )
                                .on_click(cx.listener(
                                    |this, _, _, cx| {
                                        if !this.loading {
                                            this.refresh(cx);
                                        }
                                    },
                                )),
                            )
                            .child(
                                button("new-node", "+ New node")
                                    .bg(rgb(theme::PRIMARY))
                                    .hover(|button| button.bg(rgb(theme::PRIMARY_HOVER)))
                                    .on_click(
                                        cx.listener(|this, _, window, cx| {
                                            this.new_node(window, cx)
                                        }),
                                    ),
                            )
                            .child(
                                button("editor", format!("Editor: {}", self.editor.label()))
                                    .when(self.show_editors, selected_button)
                                    .on_click(cx.listener(|this, _, _, cx| {
                                        this.show_editors = !this.show_editors;
                                        cx.notify();
                                    })),
                            ),
                    )
                    .child(self.execution_toolbar(cx)),
            )
            .when(self.show_editors, |view| {
                view.child(
                    div()
                        .flex()
                        .gap_2()
                        .px_4()
                        .pb_3()
                        .items_center()
                        .child("Open sources with:")
                        .children(ExternalEditor::ALL.into_iter().enumerate().map(
                            |(index, editor)| {
                                button(("editor-choice", index), editor.label())
                                    .when(editor == self.editor, |button| {
                                        button.bg(rgb(theme::SELECTED))
                                    })
                                    .on_click(cx.listener(move |this, _, _, cx| {
                                        this.editor = editor;
                                        this.show_editors = false;
                                        this.error = None;
                                        this.save_preferences(cx);
                                        cx.notify();
                                    }))
                            },
                        )),
                )
            })
            .child(self.runtime_toolbar(cx))
            .child(
                div()
                    .flex()
                    .flex_wrap()
                    .items_center()
                    .flex_shrink_0()
                    .gap_2()
                    .px_4()
                    .py_1()
                    .text_xs()
                    .bg(rgb(theme::SURFACE))
                    .text_color(rgb(theme::MUTED))
                    .child(
                        div()
                            .flex_1()
                            .min_w(px(220.0))
                            .truncate()
                            .child(self.configuration_label()),
                    )
                    .child(
                        div().flex().gap_1().children(
                            [
                                (
                                    format!(
                                        "{} packages",
                                        self.snapshot.as_ref().map_or(0, |snapshot| {
                                            snapshot.project.packages.len()
                                        })
                                    ),
                                    InspectorView::Packages,
                                ),
                                (
                                    format!(
                                        "{} nodes",
                                        self.snapshot
                                            .as_ref()
                                            .map_or(0, |snapshot| { snapshot.project.nodes.len() })
                                    ),
                                    InspectorView::Nodes,
                                ),
                                (
                                    format!(
                                        "{} topics",
                                        self.snapshot
                                            .as_ref()
                                            .map_or(0, GraphSnapshot::topic_count)
                                    ),
                                    InspectorView::Topics,
                                ),
                            ]
                            .into_iter()
                            .enumerate()
                            .map(|(index, (label, view))| {
                                button(("inspector-summary", index), label)
                                    .px_2()
                                    .py_0()
                                    .min_h(px(24.0))
                                    .text_xs()
                                    .text_color(rgb(theme::MUTED))
                                    .when(self.inspector_view == view, selected_button)
                                    .on_click(cx.listener(move |this, _, _, cx| {
                                        this.inspector_view = view;
                                        cx.notify();
                                    }))
                            }),
                        ),
                    ),
            )
            .when(self.show_configurations, |view| {
                view.child(self.configuration_picker(cx))
            })
            .child(
                div()
                    .flex()
                    .flex_1()
                    .min_h_0()
                    .when(self.show_make_panel, |view| view.child(self.make_panel(cx)))
                    .child(self.graph_canvas(cx))
                    .child(self.inspector(cx)),
            )
            .when(self.console_visible, |view| {
                view.child(
                    div()
                        .flex()
                        .flex_col()
                        .flex_shrink_0()
                        .h(px(self.console_height.min(
                            (window.viewport_size().height.as_f32() * 0.65).max(120.0),
                        )))
                        .child(
                            div()
                                .id("console-resize")
                                .h(px(6.0))
                                .flex_shrink_0()
                                .w_full()
                                .bg(rgb(theme::BORDER))
                                .cursor(gpui::CursorStyle::ResizeUpDown)
                                .on_mouse_down(
                                    MouseButton::Left,
                                    cx.listener(|this, event: &MouseDownEvent, _, cx| {
                                        this.drag = Some(Drag::Console {
                                            mouse_y: event.position.y.as_f32(),
                                            height: this.console_height,
                                        });
                                        cx.stop_propagation();
                                    }),
                                ),
                        )
                        .child(self.console.clone()),
                )
            })
            .child(
                div()
                    .px_4()
                    .py_2()
                    .text_xs()
                    .border_t_1()
                    .border_color(rgb(theme::BORDER))
                    .text_color(rgb(theme::MUTED))
                    .child(format!(
                        "{} · {:.0}%",
                        self.status,
                        self.viewport.scale * 100.0
                    )),
            )
            .when_some(self.error.clone(), |view, error| {
                view.child(
                    div()
                        .px_4()
                        .py_2()
                        .bg(rgb(theme::ERROR_BACKGROUND))
                        .text_sm()
                        .child(error),
                )
            })
            .when_some(self.wizard.clone(), |view, wizard| {
                view.child(
                    div()
                        .id("wizard-overlay")
                        .occlude()
                        .absolute()
                        .inset_0()
                        .size_full()
                        .bg(gpui::rgba(0x101820dd))
                        .flex()
                        .justify_center()
                        .p_6()
                        .on_mouse_down(MouseButton::Left, |_, _, cx| cx.stop_propagation())
                        .on_scroll_wheel(|_, _, cx| cx.stop_propagation())
                        .child(wizard),
                )
            })
            .when_some(self.context_menu, |view, position| {
                view.child(
                    div()
                        .id("canvas-context-menu")
                        .absolute()
                        .left(position.x)
                        .top(position.y)
                        .w(px(230.0))
                        .occlude()
                        .bg(rgb(theme::ELEVATED))
                        .rounded_md()
                        .p_2()
                        .flex()
                        .flex_col()
                        .gap_1()
                        .on_mouse_down(MouseButton::Left, |_, _, cx| cx.stop_propagation())
                        .child(
                            button("context-new-node", "New ROS node…").on_click(cx.listener(
                                |this, _, window, cx| {
                                    this.context_menu = None;
                                    this.new_node(window, cx);
                                },
                            )),
                        )
                        .child(
                            button("context-configurations", "Run configuration…").on_click(
                                cx.listener(|this, _, _, cx| {
                                    this.context_menu = None;
                                    this.show_configurations = true;
                                    cx.notify();
                                }),
                            ),
                        )
                        .child(button("context-fit", "Fit graph").on_click(cx.listener(
                            |this, _, _, cx| {
                                this.context_menu = None;
                                this.fit(cx);
                            },
                        )))
                        .child(button("context-refresh", "Refresh").on_click(cx.listener(
                            |this, _, _, cx| {
                                this.context_menu = None;
                                this.refresh(cx);
                            },
                        ))),
                )
            })
            .when(self.closing, |view| {
                view.child(
                    div()
                        .id("closing-overlay")
                        .absolute()
                        .inset_0()
                        .size_full()
                        .occlude()
                        .flex()
                        .items_center()
                        .justify_center()
                        .bg(gpui::rgba(0x101820dd))
                        .child("Closing ROS sessions and stopping their processes…"),
                )
            })
    }
}

pub(super) fn button(
    id: impl Into<gpui::ElementId>,
    label: impl Into<SharedString>,
) -> Stateful<Div> {
    div()
        .id(id)
        .px_3()
        .py_1()
        .min_h(px(32.0))
        .flex()
        .items_center()
        .rounded_md()
        .bg(rgb(theme::ELEVATED))
        .cursor_pointer()
        .text_sm()
        .text_color(rgb(theme::TEXT))
        .hover(|button| button.bg(rgb(theme::HOVER)))
        .child(label.into())
}

fn selected_button(button: Stateful<Div>) -> Stateful<Div> {
    button
        .bg(rgb(theme::SELECTED))
        .text_color(rgb(theme::ACCENT))
        .hover(|button| button.bg(rgb(theme::PRIMARY)))
}

fn toolbar_group() -> Div {
    div()
        .flex()
        .items_center()
        .flex_shrink_0()
        .gap_1()
        .p_1()
        .rounded_lg()
        .bg(rgb(theme::INPUT))
        .border_1()
        .border_color(rgb(theme::BORDER))
}

fn watch_project(root: &std::path::Path) -> Result<(RecommendedWatcher, Receiver<WatchEvent>)> {
    let (sender, receiver) = async_channel::unbounded();
    let watched_root = root.to_owned();
    let mut watcher = notify::recommended_watcher(move |result: notify::Result<notify::Event>| {
        let event = match result {
            Ok(event)
                if !matches!(event.kind, EventKind::Access(_))
                    && event
                        .paths
                        .iter()
                        .any(|path| relevant_project_path(&watched_root, path)) =>
            {
                WatchEvent::Changed
            }
            Ok(_) => return,
            Err(error) => WatchEvent::Failed(error.to_string()),
        };
        if let Err(error) = sender.try_send(event) {
            if !error.is_closed() {
                eprintln!("File watcher channel: {error}");
            }
        }
    })?;
    // Watch only source directories. Recursive root watches also register huge target/build trees.
    for entry in WalkDir::new(root)
        .into_iter()
        .filter_entry(|entry| relevant_project_path(root, entry.path()))
    {
        let entry = entry?;
        if entry.file_type().is_dir() {
            watcher
                .watch(entry.path(), RecursiveMode::NonRecursive)
                .with_context(|| format!("Cannot watch {}", entry.path().display()))?;
        }
    }
    Ok((watcher, receiver))
}

fn edge_points(publisher: &NodePosition, subscriber: &NodePosition) -> ((f32, f32), (f32, f32)) {
    let start = (
        publisher.x + NODE_WIDTH / 2.0,
        publisher.y + NODE_HEIGHT / 2.0,
    );
    let end = (
        subscriber.x + NODE_WIDTH / 2.0,
        subscriber.y + NODE_HEIGHT / 2.0,
    );
    let horizontal = end.0 - start.0;
    let vertical = end.1 - start.1;
    if horizontal.abs() >= vertical.abs() {
        let direction = horizontal.signum();
        (
            (start.0 + direction * NODE_WIDTH / 2.0, start.1),
            (end.0 - direction * NODE_WIDTH / 2.0, end.1),
        )
    } else {
        let direction = vertical.signum();
        (
            (start.0, start.1 + direction * NODE_HEIGHT / 2.0),
            (end.0, end.1 - direction * NODE_HEIGHT / 2.0),
        )
    }
}

fn paint_graph(
    bounds: Bounds<Pixels>,
    viewport: Viewport,
    lines: &[((f32, f32), (f32, f32), String)],
    window: &mut Window,
) -> Result<()> {
    if bounds.size.width <= px(0.0) || bounds.size.height <= px(0.0) {
        return Ok(());
    }
    let to_window = |(x, y): (f32, f32)| point(bounds.origin.x + px(x), bounds.origin.y + px(y));
    // Quads are batched by GPUI. Unlike one large stroked path, grid dots need
    // neither CPU tessellation nor a canvas-sized intermediate path composite.
    for dot in viewport.grid_points(bounds.size.width.as_f32(), bounds.size.height.as_f32()) {
        window.paint_quad(fill(
            Bounds::new(to_window(dot), size(px(1.5), px(1.5))),
            rgb(theme::GRID),
        ));
    }
    for &(start, end, _) in lines {
        let start = viewport.screen_point(start.0, start.1);
        let end = viewport.screen_point(end.0, end.1);
        let mut line = PathBuilder::stroke(px(2.0));
        line.move_to(to_window(start));
        line.line_to(to_window(end));
        window.paint_path(line.build()?, rgb(theme::ACCENT));
        let angle = (end.1 - start.1).atan2(end.0 - start.0);
        let length = 10.0 * viewport.scale;
        let width = 5.0 * viewport.scale;
        let base = (end.0 - length * angle.cos(), end.1 - length * angle.sin());
        let mut arrow = PathBuilder::fill();
        arrow.move_to(to_window(end));
        arrow.line_to(to_window((
            base.0 + width * angle.sin(),
            base.1 - width * angle.cos(),
        )));
        arrow.line_to(to_window((
            base.0 - width * angle.sin(),
            base.1 + width * angle.cos(),
        )));
        arrow.close();
        window.paint_path(arrow.build()?, rgb(theme::ACCENT));
    }
    Ok(())
}
