#!/usr/bin/env python3
"""Build DPVO CUDA extensions directly from the locked read-only submodule."""

import os
from pathlib import Path

from setuptools import setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension


SOURCE = Path(os.environ["DPVO_SOURCE"]).resolve()


def source(relative):
    return str(SOURCE / relative)


setup(
    name="aerohub-dpvo-extensions",
    version="0.1.0",
    ext_modules=[
        CUDAExtension(
            "cuda_corr",
            sources=[
                source("dpvo/altcorr/correlation.cpp"),
                source("dpvo/altcorr/correlation_kernel.cu"),
            ],
            extra_compile_args={"cxx": ["-O3"], "nvcc": ["-O3"]},
        ),
        CUDAExtension(
            "cuda_ba",
            sources=[
                source("dpvo/fastba/ba.cpp"),
                source("dpvo/fastba/ba_cuda.cu"),
                source("dpvo/fastba/block_e.cu"),
            ],
            include_dirs=["/usr/include/eigen3"],
            extra_compile_args={"cxx": ["-O3"], "nvcc": ["-O3"]},
        ),
        CUDAExtension(
            "lietorch_backends",
            sources=[
                source("dpvo/lietorch/src/lietorch.cpp"),
                source("dpvo/lietorch/src/lietorch_gpu.cu"),
                source("dpvo/lietorch/src/lietorch_cpu.cpp"),
            ],
            include_dirs=[
                source("dpvo/lietorch/include"),
                "/usr/include/eigen3",
            ],
            extra_compile_args={"cxx": ["-O3"], "nvcc": ["-O3"]},
        ),
    ],
    cmdclass={"build_ext": BuildExtension},
)
