#include <librealsense2/rs.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <map>
#include <rapidjson/writer.h>
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/document.h"
// TODO: Profile and add Intel MKL support as needed
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include <mutex>
#include <utility>
#include <set>
#include <thread>
#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/server.hpp"
#include "websocketpp/common/asio.hpp"

struct point {
  float x;
  float y;
  float z;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

std::string base_data_path() {
  std::filesystem::path base_path;
  char *xdg_path = getenv("XDG_DATA_HOME");
  if (xdg_path) {
	base_path = xdg_path;
  } else {
	base_path = getenv("HOME");
	base_path /= ".local/share";
  }
  base_path /= "footron-pointclouds";
  std::filesystem::create_directory(base_path);
  return base_path;
}

class DeviceSource {
public:
  DeviceSource(std::string serial_number, rs2::pipeline pipeline)
	  : serial_number(std::move(serial_number)), pipeline(std::move(pipeline)) {
  }
  std::string serial_number;
  rs2::pipeline pipeline;
};

std::mutex double_buffer_mutex;

class PointCloudListener {
public:
  // TODO: Unclear that I have to initialize the context
  PointCloudListener() : context() {
    point_count = 0;
	Eigen::Matrix4f matrix;
  };
  // Start this within a thread
  [[noreturn]] void run() {
	start_sources();

	while (true) {
	  // Collect the new frames from all the connected devices
	  // std::map<std::string, rs2::points *> new_frames;
	  for (const auto &[_, source] : this->sources) {
		this->process_source(source);
	  }
//	  dump_points_to_file();
	}
  };
  // This is a debug function and should not be used in production
  void dump_points_to_file() {
	std::ofstream ofile("test.txt", std::ios::out | std::ios::binary);
	uint64_t points_count = 0;
	for (const auto&[_, value] : this->clouds) {
	  points_count += value->size();
	}
	ofile << "pc";
	ofile.write((char *)&points_count, sizeof(points_count));

	// Based on
	// https://github.com/IntelRealSense/realsense-ros/blob/fc11bfcd845825f0e44b9917ecac6b109839b1f5/realsense2_camera/src/base_realsense_node.cpp#L2415

	for (const auto&[_, value] : this->clouds) {
	  for (const auto &point : *value) {
		// @vinhowe: If vertex->z == 0, then there is no data there. If u or v are outside of range, we set vertex->z = 0.
		if (point.z == 0) {
		  continue;
		}

		// TODO: not sure why we aren't just writing the struct here
		ofile.write((char *)&point.x, sizeof(point.x));
		ofile.write((char *)&point.y, sizeof(point.y));
		ofile.write((char *)&point.z, sizeof(point.z));
		ofile.write((char *)&point.r, sizeof(point.r));
		ofile.write((char *)&point.g, sizeof(point.g));
		ofile.write((char *)&point.b, sizeof(point.b));
	  }
	}

	ofile.close();
  }
  void load_transforms() {
	this->transforms_path = base_data_path() / std::filesystem::path("transforms.json");
	if (!exists(this->transforms_path)) {
	  return;
	}

	// https://rapidjson.org/md_doc_stream.html#FileReadStream
	FILE *fp = fopen(this->transforms_path.c_str(), "r");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	// TODO: Actually iterate over values
  }
  // TODO: Call this method every time we update transforms
  void save_transforms() {
	FILE *fp = fopen(this->transforms_path.c_str(), "wb"); // non-Windows use "w"

	rapidjson::Document d(rapidjson::kObjectType);
	for (auto &[key, transform] : this->transformations) {
	  rapidjson::Document::AllocatorType &allocator = d.GetAllocator();
	  rapidjson::Value array(rapidjson::kArrayType);
	  rapidjson::Value document_key(key.c_str(), allocator);
	  float *values_itr = transform.data();
	  for (int i = 0; i < transform.size(); i++, values_itr++) {
		array.PushBack(*values_itr, allocator);
	  }

	  d.AddMember(document_key, array, allocator);
	}

	char writeBuffer[65536];
	rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

	rapidjson::Writer<rapidjson::FileWriteStream> writer(os);
	d.Accept(writer);

	fclose(fp);
  }
  // Will need to call this whenever a device is added if we can't rely on all devices to be accurately reported at
  // startup--probably okay to just ignore if a device drops out if we're assuming it will launch again soon.
  void start_sources() {
	load_transforms();
	this->clouds.clear();
	this->clouds_back.clear();
	this->locks.clear();
	this->sources.clear();
	auto devices = this->context.query_devices();
	for (auto &&dev : this->context.query_devices(RS2_PRODUCT_LINE_L500)) {
	  rs2::pipeline pipe(this->context);
	  rs2::config cfg;
	  auto serial_no = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	  cfg.enable_device(serial_no);
	  // These presets are as high as I can run more than one camera on my computer without running out of "frame
	  // resources." We will need to test with our production machine's better USB controller.
	  cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	  pipe.start(cfg);
	  if (this->transformations.count(serial_no) == 0) {
		// TODO: Replace ::Random() with ::Zero()--this is just for testing
		this->transformations.insert({serial_no, Eigen::Matrix4f::Random()});
	  }
	  // TODO: We're likely overallocating 1024x768 because some of the point cloud extends beyond the RGB frame and is
	  //  clipped--we should see what number the camera consistently gives us
	  this->clouds.insert({serial_no, new std::vector<point>(1024 * 768)});
	  this->clouds_back.insert({serial_no, new std::vector<point>(1024 * 768)});
	  this->locks.insert({serial_no, std::make_unique<std::mutex>()});
	  this->sources.insert({serial_no, DeviceSource(serial_no, pipe)});
	}
	save_transforms();
  }
  // Map between serial numbers and last data for point cloud
  // TODO: Implement double buffering here
//  std::vector<point> combined_cloud;
  std::map<std::string, std::vector<point> *> clouds;
  size_t point_count;
  std::map<std::string, std::vector<point> *> clouds_back;
  std::map<std::string, std::unique_ptr<std::mutex>> locks;
private:
  void process_source(const DeviceSource &source) {
	rs2::frameset frames;

	if (!source.pipeline.poll_for_frames(&frames)) {
	  return;
	}

	rs2::pointcloud pc;
	rs2::points points;

	rs2::video_frame color = frames.get_color_frame();
	auto *color_data = (uint8_t *)color.get_data();
	const int color_data_length = color.get_data_size();

	pc.map_to(color);

	rs2::video_frame depth = frames.get_depth_frame();
	points = pc.calculate(depth);

	const rs2::vertex *vertex = points.get_vertices();
	const rs2::texture_coordinate *color_point = points.get_texture_coordinates();

	int texture_width = color.get_width();
	int texture_height = color.get_height();
	int num_colors = color.get_bytes_per_pixel();
	// TODO: Double buffer here instead
	this->clouds_back[source.serial_number]->clear();
//	auto test_matrix = Eigen::Matrix4f::Random();
	for (size_t point_idx = 0; point_idx < points.size(); point_idx++, vertex++, color_point++) {
	  float i(color_point->u);
	  float j(color_point->v);

	  // @vinhowe: If vertex->z == 0, then there is no data there. If i or j are outside [0, 1], then something is clearly
	  // wrong.
	  if (vertex->z == 0 || i < 0.f || i > 1.f || j < 0.f || j > 1.f) {
		continue;
	  }

	  int pix_x = static_cast<int>(i * static_cast<float>(texture_width));
	  int pix_y = static_cast<int>(j * static_cast<float>(texture_height));
	  int offset = (pix_y * texture_width + pix_x) * num_colors;
	  // TODO: I'm not sure why this happens--we should investigate--but it doesn't seem to cause any visual problems
	  if (offset >= color_data_length) {
		continue;
	  }

	  uint8_t *pixel_color = color_data + offset;

	  Eigen::Vector4f coords(vertex->x, vertex->y, vertex->z, 1);
//	  auto transformed_coords = test_matrix * coords;
	  this->clouds_back[source.serial_number]->push_back(
		  {
//			  transformed_coords.x(),
//			  transformed_coords.y(),
//			  transformed_coords.z(),
			  coords.x(),
			  coords.y(),
			  coords.z(),
			  pixel_color[0],
			  pixel_color[1],
			  pixel_color[2]
		  }
	  );
	}

	const std::lock_guard<std::mutex> lock(double_buffer_mutex);
	auto tmp_ptr = this->clouds[source.serial_number];
	this->clouds[source.serial_number] = this->clouds_back[source.serial_number];
	this->clouds_back[source.serial_number] = tmp_ptr;
	size_t tmp_point_count = 0;
	for (const auto&[_, value] : this->clouds) {
	  tmp_point_count += value->size();
	}
//	std::cout << "Updating point count: " << tmp_point_count << std::endl;
	this->point_count = tmp_point_count;
  }
  rs2::context context;
  std::map<std::string, DeviceSource> sources;
  std::filesystem::path transforms_path;
  // Map between serial numbers and their transformations
  std::map<std::string, Eigen::Matrix4f> transformations;
};

class CombinedPointCloudServer : public std::enable_shared_from_this<CombinedPointCloudServer> {
public:
  explicit CombinedPointCloudServer(PointCloudListener &listener)
	  : ws_server(), listener(listener), service() {
	// The most raw data we expect to get from 4 cameras with all data is ~47.19MB
	// (1024x768 * (xyz float32s + rgb bytes) * 4 cameras), so a 60MB buffer gives us some room
	output_buffer = new uint8_t[60000000];
	ws_server.init_asio(&service);
	ws_server.set_open_handler([this](auto &&PH1) { on_open(std::forward<decltype(PH1)>(PH1)); });
	ws_server.set_close_handler([this](auto &&PH1) { on_close(std::forward<decltype(PH1)>(PH1)); });

	ws_server.set_access_channels(websocketpp::log::alevel::none);

  }

  ~CombinedPointCloudServer() {
	this->stop();
  }

  void run() {
	ws_server.set_reuse_addr(true);
	ws_server.listen(9002);
	ws_server.start_accept();

	boost::asio::steady_timer listener_timer(service, boost::asio::chrono::milliseconds(30));
	listener_timer.async_wait(bind(&CombinedPointCloudServer::run_listener, this, &listener_timer));

	service.run();
  }

  void run_listener(boost::asio::steady_timer *t) {
	// TODO: See if there's a better way to let rclcpp spin independently, because this code looks like it will do more blocking than we'd like
	this->publish();
	t->expires_at(t->expiry() + boost::asio::chrono::milliseconds(30));
	t->async_wait(bind(&CombinedPointCloudServer::run_listener, this, t));
  }

  void publish() {
	const std::lock_guard<std::mutex> lock(double_buffer_mutex);
	size_t offset = 0;
	strcpy((char *)this->output_buffer + offset, "pc");
	// Length of magic chars (should probably null terminate by convention?)
	offset += 2;

	*((uint64_t *)(this->output_buffer + offset)) = listener.point_count;
	offset += sizeof(uint64_t);

	int temp_pt_count = 0;

	for (const auto&[serial_number, points] : this->listener.clouds) {
	  for (const auto &point : *points) {
		*((struct point *)(this->output_buffer + offset)) = point;
		offset += sizeof(float) * 3 + sizeof(uint8_t) * 3;
		temp_pt_count++;
	  }
	}

//	std::cout << temp_pt_count << std::endl;
//	std::cout << listener.point_count << std::endl;
//	assert(listener.point_count * (sizeof(float) * 3 + sizeof(uint8_t) * 3) == offset - 2 - sizeof(uint64_t));

	for (const auto &it : connections) {
	  ws_server.send(it,
					 this->output_buffer,
					 offset,
					 websocketpp::frame::opcode::binary);
	}
  }

  // TODO: I don't think on_{open, close} need to be open
  void on_open(websocketpp::connection_hdl hdl) {
	connections.insert(hdl);
  }

  void on_close(websocketpp::connection_hdl hdl) {
	connections.erase(hdl);
  }

  void stop() {
	ws_server.stop_listening();
  }

private:
  websocketpp::server<websocketpp::config::asio_client> ws_server;
  PointCloudListener &listener;
  std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections;
  uint8_t *output_buffer;
  boost::asio::io_service service;
};

PointCloudListener *listener;
CombinedPointCloudServer *server;

void signal_handler(int signum) {
  server->stop();
  exit(signum);
}

int main(int argc, char *argv[]) {
  signal(SIGINT, signal_handler);

  listener = new PointCloudListener();
  server = new CombinedPointCloudServer(*listener);
  std::thread cloud_listener_thread{[]() { listener->run(); }};
  std::thread server_thread{[]() { server->run(); }};
  cloud_listener_thread.join();
  server_thread.join();
  server->stop();

  return 0;
}