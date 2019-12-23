// Copyright (c) 2019 SSL-Roots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CONSAI2R2_RECEIVER__MULTICAST_HPP_
#define CONSAI2R2_RECEIVER__MULTICAST_HPP_

#include <boost/asio.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace asio = boost::asio;

class MulticastReceiver
{
public:
  MulticastReceiver(const std::string & ip, const int port)
  : socket_(io_service_, asio::ip::udp::v4())
  {
    asio::ip::address addr = asio::ip::address::from_string(ip);
    if (!addr.is_multicast()) {
      throw std::runtime_error("expected multicast address");
    }

    socket_.set_option(asio::socket_base::reuse_address(true));
    socket_.set_option(asio::ip::multicast::join_group(addr.to_v4()));
    socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), port));
    socket_.non_blocking(true);
  }

  size_t receive(std::vector<char> & msg)
  {
    boost::system::error_code error;
    const size_t received = socket_.receive(asio::buffer(msg), 0, error);
    if (error && error != asio::error::message_size) {
      throw boost::system::system_error(error);
      return 0;
    }
    return received;
  }

  size_t available() {return socket_.available();}

private:
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
};

#endif  // CONSAI2R2_RECEIVER__MULTICAST_HPP_
