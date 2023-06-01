#include <carla/client/Client.h>

#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
	auto client = carla::client::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();
	client.LoadWorld("Town01");

	return 0;
}