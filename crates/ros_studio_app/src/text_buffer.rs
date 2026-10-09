use std::ops::Range;

#[derive(Default)]
pub struct TextBuffer {
    pub text: String,
    pub anchor: usize,
    pub cursor: usize,
    pub marked: Option<Range<usize>>,
}

impl TextBuffer {
    pub fn selection(&self) -> Range<usize> {
        self.anchor.min(self.cursor)..self.anchor.max(self.cursor)
    }

    pub fn byte_offset(&self, utf16_offset: usize) -> usize {
        let mut offset = 0;
        for (index, character) in self.text.char_indices() {
            offset += character.len_utf16();
            if offset > utf16_offset {
                return index;
            }
        }
        self.text.len()
    }

    pub fn utf16_offset(&self, byte_offset: usize) -> usize {
        self.text
            .get(..self.boundary(byte_offset))
            .unwrap_or_default()
            .encode_utf16()
            .count()
    }

    pub fn byte_range(&self, range: Range<usize>) -> Range<usize> {
        let start = self.byte_offset(range.start);
        start..self.byte_offset(range.end).max(start)
    }

    pub fn utf16_range(&self, range: Range<usize>) -> Range<usize> {
        self.utf16_offset(range.start)..self.utf16_offset(range.end)
    }

    pub fn select(&mut self, offset: usize, extend: bool) {
        self.cursor = self.boundary(offset);
        if !extend {
            self.anchor = self.cursor;
        }
    }

    fn boundary(&self, offset: usize) -> usize {
        let mut offset = offset.min(self.text.len());
        while !self.text.is_char_boundary(offset) {
            offset -= 1;
        }
        offset
    }

    pub fn previous(&self, offset: usize) -> usize {
        self.text
            .get(..self.boundary(offset))
            .unwrap_or_default()
            .char_indices()
            .next_back()
            .map_or(0, |(index, _)| index)
    }

    pub fn next(&self, offset: usize) -> usize {
        let offset = self.boundary(offset);
        self.text
            .get(offset..)
            .and_then(|text| text.chars().next())
            .map_or(offset, |character| offset + character.len_utf8())
    }

    pub fn replace(&mut self, range: Range<usize>, text: &str) -> Range<usize> {
        let start = self.boundary(range.start);
        let end = self.boundary(range.end).max(start);
        self.text.replace_range(start..end, text);
        self.select(start + text.len(), false);
        self.marked = None;
        start..self.cursor
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handles_unicode_utf16_ranges_and_selection() {
        let mut buffer = TextBuffer {
            text: "a😀éz".into(),
            ..Default::default()
        };
        assert_eq!(buffer.byte_range(1..3), 1..5);
        assert_eq!(buffer.utf16_range(1..7), 1..4);
        assert_eq!(buffer.byte_offset(2), 1);
        assert_eq!(buffer.byte_offset(99), 8);
        buffer.select(7, false);
        buffer.select(1, true);
        assert_eq!(buffer.selection(), 1..7);
        buffer.replace(buffer.selection(), "camera");
        assert_eq!(buffer.text, "acameraz");
        assert_eq!(buffer.cursor, 7);
        assert_eq!(buffer.anchor, 7);
    }

    #[test]
    fn movement_and_out_of_range_edits_do_not_split_characters() {
        let mut buffer = TextBuffer {
            text: "é😀".into(),
            ..Default::default()
        };
        assert_eq!(buffer.next(0), 2);
        assert_eq!(buffer.next(2), 6);
        assert_eq!(buffer.previous(6), 2);
        buffer.select(3, false);
        assert_eq!(buffer.cursor, 2);
        buffer.replace(3..usize::MAX, "x");
        assert_eq!(buffer.text, "éx");
    }
}
