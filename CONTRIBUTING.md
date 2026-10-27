# flux 기여 안내

flux의 코드를 고치는 사람이 알아야 할 것을 둔다. 사용법은 [README.md](README.md)와 `docs/`에 있다.

## 문서 예제

`docs/`의 코드 블록 중에는 언어 표시 뒤에 `doc:<id>`가 붙은 것이 있다. 이런 블록은 시험 코드와 같다. C++ 예제는 `flux_cpp/test/test_doc_examples.cpp`에서, Python 예제는 `flux_py/test/test_doc_examples.py`에서 실제로 돈다. pre-commit의 `scripts/check_doc_examples.py`는 문서의 코드 블록이 시험의 `[doc:<id>]` 구간과 같은지 확인한다.
