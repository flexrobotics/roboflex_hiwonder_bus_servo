import os
import platform
import subprocess
import sys
from pathlib import Path
from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError as exc:
            raise RuntimeError("CMake must be installed to build this extension") from exc

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + extdir,
            "-DPython_EXECUTABLE=" + sys.executable,
        ]
        cfg = "Debug" if self.debug else "Release"
        build_args = ["--config", cfg]

        if platform.system() == "Windows":
            cmake_args += ["-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}".format(cfg.upper(), extdir)]
            if sys.maxsize > 2**32:
                cmake_args += ["-A", "x64"]

        self.build_temp = os.path.join(os.path.abspath(self.build_temp), "build")
        os.makedirs(self.build_temp, exist_ok=True)

        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp)
        subprocess.check_call(["cmake", "--build", "."] + build_args, cwd=self.build_temp)


long_description = (Path(__file__).parent / "README.md").read_text() if (Path(__file__).parent / "README.md").exists() else ""

setup(
    name="roboflex.hiwonder_bus_servo",
    version="0.1.6",
    description="Roboflex Hiwonder bus servo library",
    author="Colin Prepscius",
    author_email="colinprepscius@gmail.com",
    url="https://github.com/flexrobotics/roboflex_hiwonder_bus_servo",
    long_description=long_description,
    long_description_content_type="text/markdown",
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Software Development :: Embedded Systems",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3",
    ],
    keywords=["hiwonder", "servo", "robotics", "python", "c++", "roboflex"],
    license="MIT",
    python_requires=">=3.8",
    install_requires=["roboflex"],
    ext_modules=[CMakeExtension("roboflex/hiwonder_bus_servo/roboflex_hiwonder_bus_servo_ext")],
    cmdclass=dict(build_ext=CMakeBuild),
    py_modules=["__init__"],
    packages=["roboflex.hiwonder_bus_servo"],
    package_dir={"roboflex.hiwonder_bus_servo": "python"},
)
