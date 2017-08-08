#ifndef DRIVE_HPP_
#define DRIVE_HPP_

#include <sys/time.h> // for struct timeval

// boost
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "Aria.h"

#include "server.hpp"

namespace SeekurJrRC {
  namespace Core {
    /**
     * 
     * Klasa komunikująca się bezpośrednio z firmware'm robota; w tej klasie jest też zaimplementowany watchdog.
     * Watchdog sprawdza co określony czas, czy od ostatniego ustawienia prędkości silników nie minął ustalony wcześniej timeout;
     * jeżeli tak, wyłącza silniki
     */
    class Driver  : public boost::asio::io_service::service
    {
    public:
      /// konieczne ze względu na dziedziczenie po boost::asio::io_service::service
      static boost::asio::io_service::id id;
      /// konstruktor, ustawia timer
      explicit Driver(boost::asio::io_service& ios);
      /// pusty destruktor
      ~Driver() {};
      /**
       * 
       * Callback timera (watchdog'a). Jednocześnie po sprawdzeniu na nowo ustawia timer
       */
      void checkMotors(boost::shared_ptr<boost::asio::deadline_timer> & p_checkMotorsTimer);
      /**
       * 
       * Przyjęcie wiadomości od usługi serwera TCP i jej przetworzenie, tj. obliczenie prędkości obu stron
       * w/g odpowiedniego modelu sterowania i nadanie tych prędkości robotowi
       */
      void processMessage(TCPMessage& message);
      
    private:
      ArRobot* robot_;
      ArRobotConnector* robot_connector_;
      ArArgumentParser* robot_arg_parser_;
      /// pointer do naszego właściciela (w sumie czemu nie referencja?)
      boost::asio::io_service* p_IOService_;
      /// timeout na zatrzymanie silników
      const static long stopMotorsTimeout_ = 300; // in milliseconds
      /// odstęp czasowy kolejnych wywołań watchdog'a
      const static long stopMotorsCheckInterval_ = 50; // in milliseconds
      /// czas ostatniej zmiany prędkości silników
      static struct timeval lastMotorsUpdate_;
      /// ficzer boost::asio::io_service; przy zakończeniu programu będziemy mogli spokojnie zatrzymac robota
      void shutdown_service();
      static struct timeval nowTime_;
    };
    
    
  }
}

#endif
