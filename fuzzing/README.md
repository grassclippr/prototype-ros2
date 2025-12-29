# Fuzzing / Property Tests

This uses uv for dependency management.

Install dependencies:

```
uv sync
```

Run:

```
uv run pytest fuzzing/test_serial_mux_hypothesis.py
```
