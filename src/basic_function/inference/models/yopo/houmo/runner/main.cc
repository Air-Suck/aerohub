#include <cstdint>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "tcim/tcim_runtime.h"

namespace fs = std::filesystem;

static std::vector<char> read_file(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot read " + path.string());
  }
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

static void write_file(const fs::path& path, const tcim::Tensor& tensor) {
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output.write(static_cast<const char*>(tensor.Data()), tensor.MemSize());
  if (!output) {
    throw std::runtime_error("failed writing " + path.string());
  }
}

static void set_input(tcim::Module& module, const std::string& name,
                      const fs::path& path) {
  auto info = module.GetInputInfo(name).AsContiguous();
  auto data = read_file(path);
  if (data.size() != info.MemSize()) {
    throw std::runtime_error(name + " has " + std::to_string(data.size()) +
                             " bytes, expected " + std::to_string(info.MemSize()));
  }
  auto tensor = tcim::Tensor::CreateHostTensor(info, data.size(), data.data());
  const auto status = module.SetInput(name, tensor);
  if (status != tcim::Status::OK) {
    throw std::runtime_error("SetInput(" + name + ") failed: " +
                             std::to_string(status));
  }
}

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "Usage: yopo_hmm_runner MODEL.hmm DEPTH.f16 OBS.f16 "
                 "PREDICTIONS.f16 SCORES.f16\n";
    return 2;
  }
  try {
    const auto setup_start = std::chrono::steady_clock::now();
    tcim::Module module(argv[1]);
    if (module.GetInitStatus() != tcim::Status::OK) {
      throw std::runtime_error("LoadModel failed: " +
                               std::to_string(module.GetInitStatus()));
    }
    set_input(module, "depth", argv[2]);
    set_input(module, "obs", argv[3]);
    const auto run_start = std::chrono::steady_clock::now();
    const auto status = module.Run(true);
    if (status != tcim::Status::OK) {
      throw std::runtime_error("Run failed: " + std::to_string(status));
    }
    const auto end = std::chrono::steady_clock::now();
    write_file(argv[4], module.GetOutput("predictions"));
    write_file(argv[5], module.GetOutput("scores"));
    const auto setup_ms =
        std::chrono::duration<double, std::milli>(run_start - setup_start).count();
    const auto run_ms =
        std::chrono::duration<double, std::milli>(end - run_start).count();
    std::cout << "YOPO NPU inference passed setup_ms=" << setup_ms
              << " run_ms=" << run_ms << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
