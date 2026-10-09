use std::ops::Range;

use gpui::{
    App, Bounds, ClipboardItem, ContentMask, Context, ElementInputHandler, EntityInputHandler,
    EventEmitter, FocusHandle, Focusable, KeyDownEvent, MouseButton, Pixels, Point, Render,
    ShapedLine, SharedString, TextRun, UTF16Selection, Window, canvas, div, fill, point,
    prelude::*, px, rgb, size,
};
use ros_studio_app::{text_buffer::TextBuffer, wizard::normalize_name_input};

pub struct Changed;
impl EventEmitter<Changed> for TextInput {}

pub struct TextInput {
    buffer: TextBuffer,
    focus: FocusHandle,
    placeholder: SharedString,
    node_name: bool,
    dragging: bool,
    layout: Option<ShapedLine>,
    bounds: Option<Bounds<Pixels>>,
    scroll: Pixels,
}

impl TextInput {
    pub fn new(placeholder: &'static str, node_name: bool, cx: &mut Context<Self>) -> Self {
        Self {
            buffer: TextBuffer::default(),
            focus: cx.focus_handle(),
            placeholder: placeholder.into(),
            node_name,
            dragging: false,
            layout: None,
            bounds: None,
            scroll: px(0.0),
        }
    }

    pub fn text(&self) -> &str {
        &self.buffer.text
    }

    fn changed(&mut self, normalize: bool, cx: &mut Context<Self>) {
        if normalize && self.node_name {
            let cursor = normalize_name_input(
                self.buffer
                    .text
                    .get(..self.buffer.cursor)
                    .unwrap_or_default(),
            )
            .len();
            self.buffer.text = normalize_name_input(&self.buffer.text);
            self.buffer.select(cursor, false);
        }
        cx.emit(Changed);
        cx.notify();
    }

    fn index_at(&self, mouse: Point<Pixels>) -> usize {
        match (&self.layout, self.bounds) {
            (Some(line), Some(bounds)) if !self.buffer.text.is_empty() => {
                line.closest_index_for_x(mouse.x - bounds.left() + self.scroll)
            }
            _ => 0,
        }
    }

    fn key_down(&mut self, event: &KeyDownEvent, window: &mut Window, cx: &mut Context<Self>) {
        let key = event.keystroke.key.as_str();
        let modifiers = event.keystroke.modifiers;
        let selection = self.buffer.selection();
        if modifiers.secondary() {
            match key {
                "a" => {
                    self.buffer.anchor = 0;
                    self.buffer.cursor = self.buffer.text.len();
                }
                "c" | "x" => {
                    if !selection.is_empty() {
                        cx.write_to_clipboard(ClipboardItem::new_string(
                            self.buffer
                                .text
                                .get(selection.clone())
                                .unwrap_or_default()
                                .to_owned(),
                        ));
                        if key == "x" {
                            self.buffer.replace(selection, "");
                            self.changed(true, cx);
                        }
                    }
                }
                "v" => {
                    if let Some(text) = cx.read_from_clipboard().and_then(|item| item.text()) {
                        self.replace_text_in_range(None, &text, window, cx);
                    }
                }
                _ => return,
            }
        } else {
            match key {
                "backspace" | "delete" => {
                    let range = if selection.is_empty() {
                        if key == "backspace" {
                            self.buffer.previous(self.buffer.cursor)..self.buffer.cursor
                        } else {
                            self.buffer.cursor..self.buffer.next(self.buffer.cursor)
                        }
                    } else {
                        selection
                    };
                    self.buffer.replace(range, "");
                    self.changed(true, cx);
                }
                "left" | "right" | "home" | "end" => {
                    let offset = match key {
                        "home" => 0,
                        "end" => self.buffer.text.len(),
                        "left" if !selection.is_empty() && !modifiers.shift => selection.start,
                        "right" if !selection.is_empty() && !modifiers.shift => selection.end,
                        "left" => self.buffer.previous(self.buffer.cursor),
                        _ => self.buffer.next(self.buffer.cursor),
                    };
                    self.buffer.select(offset, modifiers.shift);
                }
                _ => return,
            }
        }
        cx.stop_propagation();
        cx.notify();
    }
}

impl Focusable for TextInput {
    fn focus_handle(&self, _: &App) -> FocusHandle {
        self.focus.clone()
    }
}

impl EntityInputHandler for TextInput {
    fn text_for_range(
        &mut self,
        range: Range<usize>,
        actual_range: &mut Option<Range<usize>>,
        _: &mut Window,
        _: &mut Context<Self>,
    ) -> Option<String> {
        let range = self.buffer.byte_range(range);
        *actual_range = Some(self.buffer.utf16_range(range.clone()));
        self.buffer.text.get(range).map(str::to_owned)
    }

    fn selected_text_range(
        &mut self,
        _: bool,
        _: &mut Window,
        _: &mut Context<Self>,
    ) -> Option<UTF16Selection> {
        Some(UTF16Selection {
            range: self.buffer.utf16_range(self.buffer.selection()),
            reversed: self.buffer.cursor < self.buffer.anchor,
        })
    }

    fn marked_text_range(&self, _: &mut Window, _: &mut Context<Self>) -> Option<Range<usize>> {
        self.buffer
            .marked
            .clone()
            .map(|range| self.buffer.utf16_range(range))
    }

    fn unmark_text(&mut self, _: &mut Window, cx: &mut Context<Self>) {
        self.buffer.marked = None;
        self.changed(true, cx);
    }

    fn replace_text_in_range(
        &mut self,
        range: Option<Range<usize>>,
        text: &str,
        _: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let range = range
            .map(|range| self.buffer.byte_range(range))
            .or_else(|| self.buffer.marked.clone())
            .unwrap_or_else(|| self.buffer.selection());
        let text = text.replace(['\n', '\r', '\t'], " ");
        self.buffer.replace(range, &text);
        self.changed(true, cx);
    }

    fn replace_and_mark_text_in_range(
        &mut self,
        range: Option<Range<usize>>,
        text: &str,
        selected: Option<Range<usize>>,
        _: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let range = range
            .map(|range| self.buffer.byte_range(range))
            .or_else(|| self.buffer.marked.clone())
            .unwrap_or_else(|| self.buffer.selection());
        let text = text.replace(['\n', '\r', '\t'], " ");
        let inserted = self.buffer.replace(range, &text);
        self.buffer.marked = (!text.is_empty()).then_some(inserted.clone());
        if let Some(selected) = selected {
            let replacement = TextBuffer {
                text,
                ..Default::default()
            };
            let selected = replacement.byte_range(selected);
            self.buffer.anchor = inserted.start + selected.start;
            self.buffer.cursor = inserted.start + selected.end;
        }
        self.changed(false, cx);
    }

    fn bounds_for_range(
        &mut self,
        range: Range<usize>,
        _: Bounds<Pixels>,
        _: &mut Window,
        _: &mut Context<Self>,
    ) -> Option<Bounds<Pixels>> {
        let line = self.layout.as_ref()?;
        let bounds = self.bounds?;
        let range = self.buffer.byte_range(range);
        Some(Bounds::from_corners(
            point(
                bounds.left() - self.scroll + line.x_for_index(range.start),
                bounds.top(),
            ),
            point(
                bounds.left() - self.scroll + line.x_for_index(range.end),
                bounds.bottom(),
            ),
        ))
    }

    fn character_index_for_point(
        &mut self,
        mouse: Point<Pixels>,
        _: &mut Window,
        _: &mut Context<Self>,
    ) -> Option<usize> {
        self.bounds?;
        Some(self.buffer.utf16_offset(self.index_at(mouse)))
    }
}

impl Render for TextInput {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let input = cx.entity();
        let paint_input = input.clone();
        div()
            .id("text-input")
            .track_focus(&self.focus)
            .w_full()
            .h_10()
            .px_3()
            .py_2()
            .bg(rgb(0x242a34))
            .border_1()
            .rounded_md()
            .overflow_hidden()
            .border_color(if self.focus.is_focused(window) {
                rgb(0x7bb8ff)
            } else {
                rgb(0x505d70)
            })
            .cursor(gpui::CursorStyle::IBeam)
            .on_key_down(cx.listener(Self::key_down))
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(|this, event: &gpui::MouseDownEvent, window, cx| {
                    window.focus(&this.focus, cx);
                    this.buffer
                        .select(this.index_at(event.position), event.modifiers.shift);
                    this.dragging = true;
                    cx.stop_propagation();
                    cx.notify();
                }),
            )
            .on_mouse_move(cx.listener(|this, event: &gpui::MouseMoveEvent, _, cx| {
                if this.dragging && event.pressed_button == Some(MouseButton::Left) {
                    this.buffer.select(this.index_at(event.position), true);
                    cx.notify();
                }
            }))
            .on_mouse_up(
                MouseButton::Left,
                cx.listener(|this, _, _, _| this.dragging = false),
            )
            .on_mouse_up_out(
                MouseButton::Left,
                cx.listener(|this, _, _, _| this.dragging = false),
            )
            .child(
                canvas(
                    move |bounds, window, cx| {
                        let input = input.read(cx);
                        let style = window.text_style();
                        let empty = input.buffer.text.is_empty();
                        let text = if empty {
                            input.placeholder.clone()
                        } else {
                            input.buffer.text.clone().into()
                        };
                        let run = TextRun {
                            len: text.len(),
                            font: style.font(),
                            color: if empty {
                                rgb(0x8390a2).into()
                            } else {
                                style.color
                            },
                            background_color: None,
                            underline: None,
                            strikethrough: None,
                        };
                        let line = window.text_system().shape_line(
                            text,
                            style.font_size.to_pixels(window.rem_size()),
                            &[run],
                            None,
                        );
                        let cursor = line.x_for_index(input.buffer.cursor);
                        let width = (bounds.size.width - px(2.0)).max(px(0.0));
                        let scroll = if cursor < input.scroll {
                            cursor
                        } else if cursor > input.scroll + width {
                            cursor - width
                        } else {
                            input.scroll
                        };
                        (line, scroll)
                    },
                    move |bounds, (line, scroll), window, cx| {
                        let input = paint_input.read(cx);
                        let focus = input.focus.clone();
                        let selection = input.buffer.selection();
                        let cursor = input.buffer.cursor;
                        window.handle_input(
                            &focus,
                            ElementInputHandler::new(bounds, paint_input.clone()),
                            cx,
                        );
                        window.with_content_mask(Some(ContentMask { bounds }), |window| {
                            let origin = point(bounds.left() - scroll, bounds.top());
                            if focus.is_focused(window) && !selection.is_empty() {
                                window.paint_quad(fill(
                                    Bounds::from_corners(
                                        point(
                                            origin.x + line.x_for_index(selection.start),
                                            bounds.top(),
                                        ),
                                        point(
                                            origin.x + line.x_for_index(selection.end),
                                            bounds.bottom(),
                                        ),
                                    ),
                                    rgb(0x425c7f),
                                ));
                            }
                            if let Err(error) = line.paint(
                                origin,
                                window.line_height(),
                                gpui::TextAlign::Left,
                                None,
                                window,
                                cx,
                            ) {
                                eprintln!("Cannot paint text input: {error:#}");
                            }
                            if focus.is_focused(window) && selection.is_empty() {
                                window.paint_quad(fill(
                                    Bounds::new(
                                        point(origin.x + line.x_for_index(cursor), bounds.top()),
                                        size(px(2.0), bounds.size.height),
                                    ),
                                    rgb(0x7bb8ff),
                                ));
                            }
                        });
                        paint_input.update(cx, |input, _| {
                            input.layout = Some(line);
                            input.bounds = Some(bounds);
                            input.scroll = scroll;
                        });
                    },
                )
                .size_full(),
            )
    }
}
