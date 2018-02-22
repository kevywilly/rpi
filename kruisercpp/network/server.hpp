//
// async_tcp_echo_server.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using namespace std;

class session
{
public:
  session(boost::asio::io_service& io_service, void (*fn) (const string&))
    : socket_(io_service) {
    callback = fn;
  }

  tcp::socket& socket(){
    return socket_;
  }

  void start(){
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
        boost::bind(&session::handle_read, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  void handle_read(const boost::system::error_code& error, size_t bytes_transferred){
    if (!error){
      
      cout << data_ << endl;
      if(callback) {
        string payload(data_);
        callback(payload);
      }
      
      
      boost::asio::async_write(socket_,
          boost::asio::buffer(data_, bytes_transferred),
          boost::bind(&session::handle_write, this,
            boost::asio::placeholders::error));
      
    } else {
      delete this;
    }
  }

  void handle_write(const boost::system::error_code& error)
  {
    if (!error){
      std::string s(data_);
      socket_.async_read_some(boost::asio::buffer(data_, max_length),
          boost::bind(&session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
            strcpy(data_,"");
    } else {
      delete this;
    }
  }

private:
  tcp::socket socket_;
  enum { max_length = 256 };
  char data_[max_length];
  void (*callback) (const string&);
};

class server{
public:
  server(boost::asio::io_service& io_service, short port, void (*fn)(const string&))
    : io_service_(io_service), acceptor_(io_service, tcp::endpoint(tcp::v4(), port)) {
    callback = fn;
    session* new_session = new session(io_service_, callback);
    acceptor_.async_accept(new_session->socket(),
        boost::bind(&server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
  }

  void handle_accept(session* new_session, const boost::system::error_code& error){
    if (!error)
    {
      new_session->start();
      new_session = new session(io_service_, callback);
      acceptor_.async_accept(new_session->socket(),
      boost::bind(&server::handle_accept, this, new_session,
      boost::asio::placeholders::error));
    }
    else
    {
      delete new_session;
    }
  }

private:
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
  void (*callback) (const string&);
};

/*
int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: async_tcp_echo_server <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;

    using namespace std; // For atoi.
    server s(io_service, atoi(argv[1]));

    io_service.run();
    cout << "running server" << endl;
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}*/