#include <iostream>
#include <climits>

#if !(CHAR_BIT == 8)
#error "Char size is not equal to 8 bits. Will not build here!"
#endif
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include "driver.hpp"
#include "server.hpp"
#include "message.hpp"

// this will be created following from http://www.boost.org/doc/libs/1_41_0/doc/html/boost_asio/tutorial/tutdaytime7/src.html

using boost::asio::ip::tcp;
using SeekurJrRC::Core::TCPServer;
using SeekurJrRC::Core::TCPConnection;

boost::asio::io_service::id TCPServer::id;

boost::shared_ptr<TCPConnection> TCPConnection::create(boost::asio::io_service& io_service) {
  return boost::shared_ptr<TCPConnection>(new TCPConnection(io_service));
}

tcp::socket& TCPConnection::socket() {
  return _socket;
}

void TCPConnection::scheduleRead() {
  // dzięki użyciu boost::shared_array możemy z dobrym przybliżeniem
  // być pewnymi, że bufor zostanie usunięty poprawnie
  // i nie będzie żadnego wycieku pamięci
  boost::shared_array<uint8_t> read_buffer(new uint8_t[_messageLength]);
  boost::asio::async_read(
    _socket,
    boost::asio::buffer(read_buffer.get(), _messageLength),
    boost::bind(
      &TCPConnection::handleRead,
      this->shared_from_this(),
      boost::asio::placeholders::error,
      read_buffer
    )
  );  
}

void TCPConnection::start() {
  scheduleRead();
}

TCPConnection::TCPConnection(boost::asio::io_service& io_service) : _socket(io_service), _io_service(io_service) { }

void TCPConnection::handleRead(
  const boost::system::error_code& error,
  boost::shared_array<uint8_t> read_buffer
)
{
  scheduleRead();
  if (!error) {
    // przetwarzamy read_buffer i przekazujemy powstałą wiadomość driverowi
    try {
      // debug
      // for (int i = 0; i < _messageLength; ++i) {
      //   printf("%02X", read_buffer[i]);
      // }
      // std::cout << std::endl;
      
      TCPMessage message(read_buffer.get());
      // TCPMessage's constructor may throw. If it throws, the code below won't be executed!
      SeekurJrRC::Core::Driver& driver = boost::asio::use_service<SeekurJrRC::Core::Driver>(_io_service);
      driver.processMessage(message);
    } catch (const char* e) {
      std::cerr << "Instatiating TCPMessage: " << e << std::endl;
    } catch (...) {
      std::cerr << "Instatiating TCPMessage: unknown exception." << std::endl;
    }
  }
}

TCPServer::TCPServer(boost::asio::io_service& io_service)
    : _acceptor(io_service, tcp::endpoint(tcp::v4(), _connPort)), service(io_service)
{
  startAccept();
}

void TCPServer::startAccept()
{
  std::cout << "Starting SERVER, listening on port " << _connPort << std::endl;
  boost::shared_ptr<TCPConnection> new_connection =
    TCPConnection::create(_acceptor.io_service());

  _acceptor.async_accept(
    new_connection->socket(),
    boost::bind(&TCPServer::handleAccept, this, new_connection, boost::asio::placeholders::error)
  );
}

void TCPServer::handleAccept(boost::shared_ptr<TCPConnection> new_connection, const boost::system::error_code& error)
{
  if (!error) {
    new_connection->start();
    startAccept();
  }
}
