#ifndef MESSAGE_HPP_
#define MESSAGE_HPP_

#define MESSAGE_LENGTH 20 // 20B

namespace SeekurJrRC {
  namespace Core {
    
    /**
     * 
     * Klasa reprezentująca pojedynczą wiadomość. Wiadomość zajmuje 20B; logicznie jest ona podzielona na pięć bloków po 4B każdy:
     * 1: drobnica
     *   - 1. bajt - START/STOP - START jeżeli wszystkie 1, w innym wypadku STOP
     *   - 2. bajt - kod typu sterowania (uint8_t) !! BIG ENDIAN !!
     *   - 3. bajt - wolny
     *   - 4. bajt - wolny
     * 2: float x w postaci bitów intowych (uint32_t)  !! BIG ENDIAN !!
     * 3: float x w postaci bitów intowych (uint32_t)  !! BIG ENDIAN !!
     * 4: float x w postaci bitów intowych (uint32_t)  !! BIG ENDIAN !!
     * 5: uint32_t CRC32  !! BIG ENDIAN !!
     * Klasa jest inicjalizowana buforem - tablicą charów, w której zawarta jest wiadomość.
     * 
     * Klasa ta w ciekawy sposób umożliwia dostęp do swoich parametrów (np. wartości wskazań akcelerometru). Zamiast dużej ilości metod
     * z rodziny set/get, stosujemy publiczne const referencje do prywatnych parametrów. W ten sposób możemy bezpośrednio uzyskać dostęp
     * read-only do wartości. Elegancka enkapsulacja.
     */
    class TCPMessage {
    public:
      /**
       * \param buffer	- bufor otrzymany poprzez TCP
       * 
       * Jedyny konstruktor wiadomości; w konstruktorze od razu zachodzi parsowanie wiadomośći. Jeżeli wiadomość
       * jest nieprawidłowa (np. suma CRC32 się nie zgadza), konstruktor rzuca wyjątek.
       */
      TCPMessage(const uint8_t* const buffer);
      
      /**
       * 
       * Pusty destruktor, nic się tu nie dzieje
       */
      ~TCPMessage() {};
      
      /// flaga ruchu robota - w wersji READ-ONLY
      const bool& startStop_;

      /// kod modelu sterowania; model sterowania określa sposób tłumaczenia wskazań akcelerometru na prędkość robota - w wersji READ-ONLY
      const uint8_t& steeringModelCode_;
      
      const float& x_;	/// wartość x-owa wskazań akcelerometru - w wersji READ-ONLY
      const float& y_;	/// wartość y-owa wskazań akcelerometru - w wersji READ-ONLY
      const float& z_;	/// wartość z-owa wskazań akcelerometru - w wersji READ-ONLY
      
    private:
      /// flaga ruchu robota - w wersji READ-WRITE
      bool rw_startStop_;
      
      /// kod modelu sterowania; model sterowania określa sposób tłumaczenia wskazań akcelerometru na prędkość robota - w wersji READ-WRITE
      uint8_t rw_steeringModelCode_;
      
      float rw_x_;	/// wartość x-owa wskazań akcelerometru - w wersji READ-WRITE
      float rw_y_;	/// wartość y-owa wskazań akcelerometru - w wersji READ-WRITE
      float rw_z_;	/// wartość z-owa wskazań akcelerometru - w wersji READ-WRITE
      
      /// Parametry rozmieszczenia wartości w wiadomości; więcej w pracy dyplomowej
      static const int offsetStartStop_ = 0;
      static const int offsetSteeringModelCode_ = 1;
      static const int offsetX_ = 4;
      static const int offsetY_ = 8;
      static const int offsetZ_ = 12;
      static const int offsetCRC32_ = 16; // 16B
      static const int messageLength_ = MESSAGE_LENGTH;
    };
  }
}

#endif
