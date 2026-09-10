#!/usr/bin/env python3
"""Regenerate the committed .onnx test fixtures.

The fixtures are COMMITTED, not built at test time: generating them needs the
python `onnx` package, which is not a dependency of this repo and is absent
from the CI image. Building them during the test run would make the gate
host-dependent — green where onnx happens to exist, silently skipped where it
does not — which is the failure mode a shape-checking test can least afford.

They are tiny (a few hundred bytes) and change only when the contract they
exercise changes, so committing them costs nothing and keeps the gate
deterministic everywhere ONNX Runtime is available.

Run this only when a fixture needs to change:

    uv run --python 3.12 --with onnx==1.16.2 python generate_fixtures.py

Then commit the regenerated .onnx alongside the test that consumes it.
"""

import onnx
from onnx import TensorProto, helper


def two_in_two_out(path: str) -> None:
    """Two inputs, two outputs, EVERY tensor [1,2].

    The uniform shape is the point: with all four tensors identically shaped,
    shape validation provably cannot tell them apart, so only name-based
    binding can wire them correctly. sum/diff make a mis-binding visible in the
    VALUES rather than just in the metadata — swapping a and b flips the sign
    of diff, which a test can assert on.
    """
    a = helper.make_tensor_value_info("a", TensorProto.FLOAT, [1, 2])
    b = helper.make_tensor_value_info("b", TensorProto.FLOAT, [1, 2])
    s = helper.make_tensor_value_info("sum", TensorProto.FLOAT, [1, 2])
    d = helper.make_tensor_value_info("diff", TensorProto.FLOAT, [1, 2])
    graph = helper.make_graph(
        [
            helper.make_node("Add", ["a", "b"], ["sum"]),
            helper.make_node("Sub", ["a", "b"], ["diff"]),
        ],
        "two_in_two_out",
        [a, b],
        [s, d],
    )
    model = helper.make_model(
        graph,
        producer_name="rtc_inference_test",
        opset_imports=[helper.make_opsetid("", 13)],
    )
    # Pinned low: ONNX Runtime refuses an IR version newer than it knows, and
    # the fixture must load on the oldest ORT this repo supports rather than on
    # whatever the generating machine happened to have.
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, path)
    print(f"{path}: {len(model.SerializeToString())} bytes")


if __name__ == "__main__":
    two_in_two_out("two_in_two_out.onnx")
