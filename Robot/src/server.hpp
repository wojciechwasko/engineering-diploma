#ifndef SERVER_HPP_
#define SERVER_HPP_

#include <string>

// this will be created following from http://www.boost.org/doc/libs/1_41_0/doc/html/boost_asio/tutorial/tutdaytime7/src.html
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include "message.hpp"

namespace SeekurJrRC {
  namespace Core {

    /**
     * 
     * Klasa reprezentująca pojedyncze połączenie do serwera; tj. jedną wiadomość z telefonu.
     */
    class TCPConnection : public boost::enable_shared_from_this<TCPConnection>
    {
    public:
      static boost::shared_ptr<TCPConnection> create(boost::asio::io_service& io_service);
      boost::asio::ip::tcp::socket& socket();
      /**
       * 
       * Metoda, której zadaniem jest zlecenie "czytania" z socket-u. Można by na upartego zrobić w handleAccept,
       * ale możliwe, że rozbudujemy protokół na komunikację dwustronną... Póki co tylko czytamy 20B w jednym pakiecie
       * i potem dalej to przekazujemy
       */
      void start();

    private:
      TCPConnection(boost::asio::io_service& io_service);
      /**
       * \param error   - czy wystąpił błąd
       * \param read_buffer - bufor, w którym mamy w formie tablicy charów wiadomość, która przyszła
       * 
       * Ta metoda zostanie wywołana, gdy boost wczyta dane z socketu. Tutaj zlecamy przetworzenie wiadomości
       * i wysyłamy update stanu do drivera. Generalnie fajnie by było, jakby docelowo driver działał we własnym wątku.
       */
      void handleRead(const boost::system::error_code& error, boost::shared_array<uint8_t> read_buffer);
      void scheduleRead();
      boost::asio::io_service& _io_service;
      boost::asio::ip::tcp::socket _socket;
      const static uint _messageLength = MESSAGE_LENGTH; // 20B
    };

    /**
     * 
     * Klasa reprezentująca serwer zarządzający przychodzącymi połączeniami
     */
    class TCPServer : public boost::asio::io_service::service
    {
    public:
      TCPServer(boost::asio::io_service& io_service);
      void shutdown_service() {};

      static boost::asio::io_service::id id;
    private:
      void startAccept();
      void handleAccept(boost::shared_ptr<TCPConnection> new_connection, const boost::system::error_code& error);

      boost::asio::ip::tcp::acceptor _acceptor;
      const static uint _connPort = 1024;
    };
  } // namespace Core
} // namespace SeekurJrRC


#endif
