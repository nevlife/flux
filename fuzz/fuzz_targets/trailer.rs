#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| flux_core::fuzz::trailer(data));
