use gpui::{
    ClipboardItem, Context, EventEmitter, FocusHandle, Render, UniformListScrollHandle, Window,
    div, prelude::*, px, rgb, uniform_list,
};
use ros_studio_app::runtime::ConsoleBuffer;
use ros_studio_protocol::OutputStream;

use crate::graph_view::button;

pub struct Closed;
impl EventEmitter<Closed> for ConsoleView {}

pub struct ConsoleView {
    buffer: ConsoleBuffer,
    focus: FocusHandle,
    scroll: UniformListScrollHandle,
    follow: bool,
    selection: Option<(usize, usize)>,
    selecting: bool,
}

impl ConsoleView {
    pub fn new(cx: &mut Context<Self>) -> Self {
        Self {
            buffer: ConsoleBuffer::default(),
            focus: cx.focus_handle(),
            scroll: UniformListScrollHandle::new(),
            follow: true,
            selection: None,
            selecting: false,
        }
    }

    pub fn append(&mut self, text: &str, stream: OutputStream, cx: &mut Context<Self>) {
        let at_end = self.scroll.is_scrolled_to_end().unwrap_or(true);
        if !at_end {
            self.follow = false;
        }
        if self.buffer.append(text, stream) {
            self.selection = None;
        }
        if self.follow {
            self.scroll.scroll_to_bottom();
        }
        cx.notify();
    }

    fn copy(&self, selected_only: bool, cx: &mut Context<Self>) {
        let text = if selected_only {
            let Some((anchor, cursor)) = self.selection else {
                return;
            };
            self.buffer
                .lines
                .iter()
                .enumerate()
                .filter(|(index, _)| *index >= anchor.min(cursor) && *index <= anchor.max(cursor))
                .map(|(_, line)| line.text.as_str())
                .collect::<Vec<_>>()
                .join("\n")
        } else {
            self.buffer.text()
        };
        cx.write_to_clipboard(ClipboardItem::new_string(text));
    }
}

impl Render for ConsoleView {
    fn render(&mut self, _: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let entity = cx.entity();
        let widest = self
            .buffer
            .lines
            .iter()
            .enumerate()
            .max_by_key(|(_, line)| line.text.len())
            .map(|(index, _)| index);
        div()
            .id("console")
            .track_focus(&self.focus)
            .flex()
            .flex_col()
            .size_full()
            .bg(rgb(0x222934))
            .on_key_down(cx.listener(|this, event: &gpui::KeyDownEvent, _, cx| {
                if event.keystroke.modifiers.secondary() {
                    match event.keystroke.key.as_str() {
                        "a" => { this.selection = Some((0, this.buffer.lines.len().saturating_sub(1))); cx.notify(); }
                        "c" => this.copy(true, cx),
                        _ => return,
                    }
                    cx.stop_propagation();
                }
            }))
            .on_mouse_up(gpui::MouseButton::Left, cx.listener(|this, _, _, _| this.selecting = false))
            .on_mouse_up_out(gpui::MouseButton::Left, cx.listener(|this, _, _, _| this.selecting = false))
            .child(div().flex().items_center().gap_2().p_2().flex_shrink_0()
                .child(div().flex_1().text_sm().child("Build / Run output · drag or Shift-click to select lines · Ctrl+C to copy"))
                .child(button("console-follow", if self.follow { "Follow: on" } else { "Follow: off" }).on_click(cx.listener(|this, _, _, cx| {
                    this.follow = !this.follow;
                    if this.follow { this.scroll.scroll_to_bottom(); }
                    cx.notify();
                })))
                .child(button("console-copy", "Copy all").on_click(cx.listener(|this, _, _, cx| this.copy(false, cx))))
                .child(button("console-clear", "Clear").on_click(cx.listener(|this, _, _, cx| {
                    this.buffer = ConsoleBuffer::default(); this.selection = None; this.follow = true;
                    this.scroll.scroll_to_bottom(); cx.notify();
                })))
                .child(button("console-close", "Close").on_click(cx.listener(|_, _, _, cx| cx.emit(Closed)))))
            .child(uniform_list("console-lines", self.buffer.lines.len().max(1), move |range, _, cx| {
                entity.update(cx, |this, cx| range.map(|index| {
                    let (text, stream) = this.buffer.lines.get(index).map(|line| (line.text.clone(), line.stream)).unwrap_or_else(|| ("No output yet".into(), OutputStream::Stdout));
                    let selected = this.selection.is_some_and(|(anchor, cursor)| index >= anchor.min(cursor) && index <= anchor.max(cursor));
                    div().id(("console-line", index)).h(px(22.0)).px_3().text_sm().font_family("monospace")
                        .whitespace_nowrap()
                        .text_color(if stream == OutputStream::Stderr { rgb(0xe9bd86) } else { rgb(0xdce5f0) })
                        .when(selected, |row| row.bg(rgb(0x425c7f)))
                        .on_mouse_down(gpui::MouseButton::Left, cx.listener(move |this, event: &gpui::MouseDownEvent, window, cx| {
                            window.focus(&this.focus, cx);
                            let anchor = if event.modifiers.shift { this.selection.map_or(index, |(anchor, _)| anchor) } else { index };
                            this.selection = Some((anchor, index)); this.selecting = true; this.follow = false;
                            cx.stop_propagation(); cx.notify();
                        }))
                        .on_mouse_move(cx.listener(move |this, event: &gpui::MouseMoveEvent, _, cx| {
                            if this.selecting && event.pressed_button == Some(gpui::MouseButton::Left) {
                                let anchor = this.selection.map_or(index, |(anchor, _)| anchor);
                                this.selection = Some((anchor, index)); cx.notify();
                            }
                        }))
                        .child(text)
                }).collect::<Vec<_>>())
            }).track_scroll(&self.scroll)
                .with_width_from_item(widest)
                .with_horizontal_sizing_behavior(gpui::ListHorizontalSizingBehavior::Unconstrained)
                .flex_1().min_h_0())
    }
}
