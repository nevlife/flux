# flux 기여 안내

flux 코드를 고치는 사람을 위한 안내다. 사용법은 [README.md](README.md)와 `docs/`에 있다.

## 문서 예제

`docs/`의 코드 블록 가운데 언어 표시 뒤에 `doc:<id>`가 있는 블록은 테스트 코드와 같다.

| 언어 | 실행하는 테스트 |
| --- | --- |
| C++ | `flux_cpp/test/test_doc_examples.cpp` |
| Python | `flux_py/test/test_doc_examples.py` |

pre-commit의 `scripts/check_doc_examples.py`가 문서의 코드 블록이 테스트의 `[doc:<id>]` 구간과 같은지 확인한다.
