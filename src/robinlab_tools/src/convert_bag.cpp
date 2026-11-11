#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
static sensor_msgs::msg::PointCloud2 convert(const livox_ros_driver2::msg::CustomMsg& m)
{
	sensor_msgs::msg::PointCloud2 pc;
	pc.header = m.header;
	sensor_msgs::PointCloud2Modifier mod(pc);
	mod.setPointCloud2Fields(
			5,
			"x", 1, sensor_msgs::msg::PointField::FLOAT32,
			"y", 1, sensor_msgs::msg::PointField::FLOAT32,
			"z", 1, sensor_msgs::msg::PointField::FLOAT32,
			"intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
			"offset_time", 1, sensor_msgs::msg::PointField::UINT32);
	mod.resize(m.points.size());
	pc.is_dense = false;
	sensor_msgs::PointCloud2Iterator<float> ix(pc, "x");
	sensor_msgs::PointCloud2Iterator<float> iy(pc, "y");
	sensor_msgs::PointCloud2Iterator<float> iz(pc, "z");
	sensor_msgs::PointCloud2Iterator<float> ii(pc, "intensity");
	sensor_msgs::PointCloud2Iterator<uint32_t> it(pc,"offset_time");

	for (const auto& p : m.points)
	{
		*ix = p.x;
		*iy = p.y;
		*iz = p.z;
		*ii = static_cast<float>(p.reflectivity);
		*it = p.offset_time;
		++ix;
		++iy;
		++iz;
		++ii;
		++it;
	}
	return pc;
}

int main(int argc, char** argv)
{
	if(argc < 3)
	{
		std::fprintf(stderr, "convert_bag <bag_entrada> <bag_salida>\n");
		return 1;
	}
	const std::string in_topic = "/livox/lidar";
	const std::string out_topic = "/livox/points";

	rosbag2_storage::StorageOptions so_in, so_out;
	so_in.uri = argv[1];
	so_in.storage_id = "sqlite3";
	so_out.uri = argv[2];
	so_out.storage_id = "sqlite3";
	rosbag2_cpp::ConverterOptions co;
	co.input_serialization_format = "cdr";
	co.output_serialization_format = "cdr";

	rosbag2_cpp::Reader reader;
	reader.open(so_in, co);
	rosbag2_cpp::Writer writer;
	writer.open(so_out, co);

	for (const auto& t : reader.get_all_topics_and_types())
	{
		writer.create_topic(t);
	}
	rosbag2_storage::TopicMetadata meta;
	meta.name = out_topic;
	meta.type = "sensor_msgs/msg/PointCloud2";
	meta.serialization_format = "cdr";
	writer.create_topic(meta);

	rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> ser_in;
	rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_out;

	while (reader.has_next())
	{
		auto msg = reader.read_next();
		writer.write(msg);
		if(msg->topic_name== in_topic)
		{
			livox_ros_driver2::msg::CustomMsg cm;
			rclcpp::SerializedMessage raw(*msg->serialized_data);
			ser_in.deserialize_message (&raw, &cm);
			auto pc = convert(cm);
			auto out = std::make_shared<rclcpp::SerializedMessage>();
			ser_out.serialize_message(&pc, out.get());
			writer.write(
				out,
				out_topic,
				"sensor_msgs/msg/PointCloud2",
				rclcpp::Time(msg->time_stamp)
				);
		}
	}
	return 0;
}
		
