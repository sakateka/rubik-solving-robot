# Rubik Scan

Experimental Rubik's Cube face scanner for the Milk-V Duo (SG2002 / CV181X).
It detects the nine visible stickers and returns their colours in 3×3 order.

The released PyTorch, ONNX and CV181X BF16 model files are described in
[`models/README.md`](models/README.md).
The engineering decisions, reproducible commands and deployment notes are in
[`PROJECT_NOTES.md`](PROJECT_NOTES.md).

## Checkout

The Milk-V Duo Buildroot SDK is a Git submodule. Clone the project with it:

```bash
git clone --recurse-submodules <repository-url>
```

For an existing checkout, fetch the SDK recorded by this repository with:

```bash
git submodule update --init --recursive
```

The submodule is pinned to a specific revision of
[`milkv-duo/duo-buildroot-sdk-v2`](https://github.com/milkv-duo/duo-buildroot-sdk-v2).
Do not update it casually: the C headers, runtime libraries and cross-build
environment must remain compatible with the Milk-V firmware used for testing.
