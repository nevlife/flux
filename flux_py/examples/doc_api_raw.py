# Companion to docs/en/raw_api.en.md -- the path a node takes when it has no .msg. Every region
# between [doc:id] and [doc:/id] is the code the fence tagged `doc:id` in that document shows.
# Nothing here runs: the functions are never called and take everything they need as parameters.
#
# Split from doc_api.py because the two documents are separately paired in
# scripts/check_doc_examples.py: api.md carries the .msg path, this file the raw one.

import flux
import flux.ros

FP = 0


def _sink(*_args):
    pass


def doc_raw_publish(node, arr):
    # [doc:raw_py_publish]
    pub = flux.ros.Publisher(node, "img", fingerprint=FP, slot_size=16 << 20, slot_count=16)

    pub.publish(arr)
    # [doc:/raw_py_publish]
    _sink(pub.dropped)


def doc_raw_loan(pub, frame):
    # [doc:raw_py_loan]
    loan = pub.loan((480, 640, 3), dtype="uint8")
    if loan:
        loan.array[:] = frame
        loan.commit()
    # [doc:/raw_py_loan]

    # [doc:raw_py_loan_api]
    loan = pub.loan(pub.slot_size, dtype="uint8")
    held = loan.valid
    writable = loan.host_addressable
    published = loan.commit(nbytes=1024)
    loan.abort()
    # [doc:/raw_py_loan_api]
    _sink(held, writable, published)


def doc_raw_view(sub):
    # [doc:raw_py_view]
    v = sub.take()
    if v is not None:
        _sink(v.shape, v.dtype, v.nbytes)
        v = None
    # [doc:/raw_py_view]


def doc_raw_bits(pub, x, ml_dtypes):
    # [doc:raw_py_bits]
    loan = pub.loan(1024, dtype="uint16")
    if loan:
        loan.bits.view(ml_dtypes.bfloat16)[:] = x
        loan.commit()
    # [doc:/raw_py_bits]
