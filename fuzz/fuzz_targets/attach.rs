#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| {
    flux2_core::fuzz::attach(data);
});
