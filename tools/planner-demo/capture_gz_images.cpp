#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

namespace fs = std::filesystem;

std::atomic<int> frame_count{0};
int max_frames = 0;
fs::path output_dir;
std::mutex write_mutex;

void OnImage(const gz::msgs::Image &_msg)
{
  const int index = frame_count.fetch_add(1);
  if (index >= max_frames)
    return;

  const auto width = static_cast<std::size_t>(_msg.width());
  const auto height = static_cast<std::size_t>(_msg.height());
  const std::size_t pixels = width * height;
  const std::string &data = _msg.data();
  if (data.size() < pixels * 3)
  {
    std::cerr << "Unsupported image payload: " << data.size() << " bytes for "
              << width << "x" << height << std::endl;
    return;
  }

  std::ostringstream name;
  name << "frame_" << std::setfill('0') << std::setw(6) << index << ".ppm";
  const fs::path temporary = output_dir / (name.str() + ".tmp");
  const fs::path final = output_dir / name.str();

  std::lock_guard<std::mutex> guard(write_mutex);
  std::ofstream file(temporary, std::ios::binary);
  file << "P6\n" << width << " " << height << "\n255\n";
  file.write(data.data(), static_cast<std::streamsize>(pixels * 3));
  file.close();
  fs::rename(temporary, final);
}

int main(int argc, char **argv)
{
  if (argc != 4)
  {
    std::cerr << "Usage: capture_gz_images TOPIC OUTPUT_DIR MAX_FRAMES" << std::endl;
    return 2;
  }

  const std::string topic = argv[1];
  output_dir = argv[2];
  max_frames = std::stoi(argv[3]);
  fs::create_directories(output_dir);

  gz::transport::Node node;
  if (!node.Subscribe(topic, OnImage))
  {
    std::cerr << "Failed to subscribe to " << topic << std::endl;
    return 1;
  }
  std::cout << "Capturing " << max_frames << " frames from " << topic << std::endl;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(10);
  while (frame_count.load() < max_frames && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const int captured = std::min(frame_count.load(), max_frames);
  std::cout << "Captured " << captured << " frames" << std::endl;
  return captured > 0 ? 0 : 1;
}
