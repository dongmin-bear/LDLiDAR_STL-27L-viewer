//! Pacecat(Bluesea) 제조사 모듈. 모델별 하위 모듈을 묶는다.
//!
//! 폴더는 제조사/모델로 나뉘어 있고(`Pacecat/LDS-50C-E/`), Rust 식별자에 쓸 수 없는
//! 하이픈 때문에 `#[path]`로 폴더명을 그대로 두고 모듈명만 깔끔하게 노출한다.

#[path = "LDS-50C-E/mod.rs"]
pub mod lds50ce;
