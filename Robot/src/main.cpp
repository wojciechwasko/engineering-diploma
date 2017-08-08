#include <iostream>
#include <boost/asio.hpp>

#include "driver.hpp"
#include "server.hpp"

int main()
{
  boost::asio::io_service program_loop;
  boost::asio::use_service<SeekurJrRC::Core::Driver>(program_loop);
  boost::asio::use_service<SeekurJrRC::Core::TCPServer>(program_loop);
  std::cout << std::setprecision(10);
  std::cout << "Starting program loop." << std::endl;
  program_loop.run();
  std::cout << "Exiting program loop." << std::endl;
  return 0;
}
