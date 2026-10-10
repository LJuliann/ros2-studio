//! Incremental, lossy UTF-8 decoding for bounded process-output reads.

#[derive(Default)]
pub struct OutputDecoder {
    pending: Vec<u8>,
}

impl OutputDecoder {
    /// Preserve characters split between reads; replace invalid bytes without
    /// turning a process's output into a process failure. EOF flushes a partial character.
    pub fn decode(&mut self, bytes: &[u8], eof: bool) -> String {
        self.pending.extend_from_slice(bytes);
        let mut text = String::new();
        let mut consumed = 0;
        while consumed < self.pending.len() {
            let remaining = &self.pending[consumed..];
            match std::str::from_utf8(remaining) {
                Ok(valid) => {
                    text.push_str(valid);
                    consumed = self.pending.len();
                }
                Err(error) => {
                    let valid = error.valid_up_to();
                    text.push_str(&String::from_utf8_lossy(&remaining[..valid]));
                    consumed += valid;
                    match error.error_len() {
                        Some(invalid) => {
                            text.push('\u{fffd}');
                            consumed += invalid;
                        }
                        None if eof => {
                            text.push('\u{fffd}');
                            consumed = self.pending.len();
                        }
                        None => break,
                    }
                }
            }
        }
        self.pending.drain(..consumed);
        text
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_unicode_at_every_chunk_boundary() {
        let original = "progress: café 🦀 完了";
        for boundary in 0..=original.len() {
            let mut decoder = OutputDecoder::default();
            let mut text = decoder.decode(&original.as_bytes()[..boundary], false);
            text.push_str(&decoder.decode(&original.as_bytes()[boundary..], true));
            assert_eq!(text, original);
        }
    }

    #[test]
    fn replaces_invalid_bytes_and_flushes_incomplete_eof() {
        let mut decoder = OutputDecoder::default();
        assert_eq!(decoder.decode(b"hello\xff\xe2", false), "hello\u{fffd}");
        assert_eq!(decoder.decode(b"", true), "\u{fffd}");
        assert!(decoder.pending.is_empty());
    }

    #[test]
    fn streams_without_newlines_and_retains_only_an_incomplete_character() {
        let mut decoder = OutputDecoder::default();
        for _ in 0..1000 {
            assert_eq!(decoder.decode(&[b'x'; 4096], false).len(), 4096);
            assert!(decoder.pending.len() <= 3);
        }
    }
}
