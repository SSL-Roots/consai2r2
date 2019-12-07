#include <iostream>
#include <exception>
#include <stdexcept>
#include <boost/asio.hpp>

namespace asio = boost::asio;

class MulticastReceiver
{
public:
  MulticastReceiver(const std::string& ip, const int port)  :
    endpoint(asio::ip::udp::v4(), port),
    socket(io_service, endpoint)
  {
    asio::ip::address addr = asio::ip::address::from_string(ip);
    if (!addr.is_multicast())
    {
      throw std::runtime_error("excpeted a multicast address");
    }

    socket.set_option(asio::ip::multicast::join_group(addr.to_v4()));
    socket.set_option(asio::socket_base::reuse_address(true));
    socket.non_blocking(true);
  }

  size_t receive(std::vector<char>& msg)
  {
    boost::system::error_code error;
    const size_t received = socket.receive_from(asio::buffer(msg), endpoint, 0, error);
    if (error && error != asio::error::message_size)
    {
      throw boost::system::system_error(error);
      return 0;
    }
    return received;
  }

  size_t available(){
    return socket.available();
  }
private:
    asio::io_service io_service;
    asio::ip::udp::endpoint endpoint;
    asio::ip::udp::socket socket;
};