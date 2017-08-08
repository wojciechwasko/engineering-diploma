#ifndef STEERING_MODEL_HPP_
#define STEERING_MODEL_HPP_

#include <stdint.h>
#include <utility> // for std::pair
#include <deque> // for std::deque
#include <sys/time.h> // for struct timeval
#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>

#include "utils.hpp"

#define RAD_TO_DEG 57.2957795
#define V_MAX 1200

#define BILINEAR_NO_FILT_MODEL_CODE 0
#define BILINEAR_SIMPLE_FILT_MODEL_CODE 1
#define BILINEAR_EXP_FILT_MODEL_CODE 2
#define SHEPARD_1_5_NO_FILT_MODEL_CODE 3
#define SHEPARD_1_5_SIMPLE_FILT_MODEL_CODE 4
#define SHEPARD_1_5_EXP_FILT_MODEL_CODE 5
#define SHEPARD_4_5_NO_FILT_MODEL_CODE 6
#define SHEPARD_4_5_SIMPLE_FILT_MODEL_CODE 7
#define SHEPARD_4_5_EXP_FILT_MODEL_CODE 8
#define GENETIC_1_NO_FILT_MODEL_CODE 9
#define GENETIC_1_SIMPLE_FILT_MODEL_CODE 10
#define GENETIC_1_EXP_FILT_MODEL_CODE 11
#define GENETIC_2_NO_FILT_MODEL_CODE 12
#define GENETIC_2_SIMPLE_FILT_MODEL_CODE 13
#define GENETIC_2_EXP_FILT_MODEL_CODE 14


namespace SeekurJrRC {
  namespace Core {
    typedef boost::tuple<const float, const float, const float, struct timeval> acc_tuple;

    /**
     *
     * Klasa abstrakcyjna, deklarująca wspólny interfejs dla różnych metod obliczania
     * prędkości robota na podstawie wskazań akcelerometru.
     *
     * Zależnie od tego, jakie metody zaimplementowane, prawdopodobnie ta klasa będzie się jeszcze
     * zajmowała "chomikowaniem" poprzednich wskazań akcelerometru (powiedzmy - max 10, jedynie z ostatnich 2 sekund - dane
     * są teraz z głowy strzelone), ponieważ jest widok na metody zależne nie tylko od obecnych wskazań akcelerometru
     * (np. metody z zastosowaniem filtra dolnoprzepustowego).
     *
     * Trzeba rozwiązać taki problem implementacyjny, jak zapisywanie czasu; powiedzmy, że użytkownik normalnie sterował robotem
     * (miał wciśnięty przycisk migawki), potem na chwilę puścił przycisk migawki (czyli żaden model nie był wywoływany przez dłuższy czas),
     * a następnie zaczął dalej sterować robotem. W tym momencie nie możemy jako historii użyć wartości "sprzed puszczenia migawki", bo
     * to będzie przekłamane. Najprawdopodobniej będzie to zaimplementowane jako kolejka FIFO, dla której jeżeli rozmiar będzie >= 10 (znowu:
     * z głowy), to będzie wywoływany pop i push; w przeciwnym razie tylko push. A jeżeli klasa pochodna będzie chciała się dobrać do
     * wartości "archiwalnych", to będzie sprawdzana cała kolejka, za stare zapisy będą usuwane i będzie zwracana const referencja do kolejki
     */
    class SteeringModelBase {
    public:
      /// konstruktor; tylko inicjalizacja składowych, zapisanie historii
      /// będziemy kopiowali wartości z referencji, więc jeżeli referencja nagle stałaby się
      /// "invalid", nie będzie nam to groziło.
      SteeringModelBase(const float& x, const float& y, const float& z) : current_acc_(getNormalizedAccTuple(x,y,z)) {
        // here, in the constructor, we only care about the size
        // whether the data is outdated will be decided in the getter method
        int no_of_elems_to_pop = history_.size() - max_history_length_ - 1;
        for (int i = 0; i < no_of_elems_to_pop; ++i)
          history_.pop_back();

        // history is sane now
        history_.push_front(current_acc_);
      };
      /**
       *
       * metoda, którą muszą zaimplementować klasy pochodne; jedyna metoda (poza ctorem i dtorem), która interesuje świat zewnętrzny
       */
      virtual std::pair<float, float> getSpeedValues() = 0;
    protected:
      /// zwracamy wartości, nie starsze niż max_history_time_ [msec]
      /// ze względu na relatywnie mały "rozmiar" historii, po prostu przeglądamy
      /// od końca wszystkie wpisy, usuwając wpisy zbyt stare i zwracamy tak powstały deque.
      /// 
      /// jeżeli historia miałaby kiedyś mieć więcej elementów (>50-100), można by problem rozwiązać inaczej:
      /// 1. sprawdzić, czy najnowszy element nie jest za stary, jeżeli tak to wywalić wszystko i zwrócić pustą historię
      /// 2. w przeciwnym razie oszacować indeks elementu znajdującego się na granicy łapania się, od tego elementu przejść
      ///    się nieco w odpowiednim kierunku, żeby znaleźć ten element graniczny
      /// 3. wszystkie za stare hurtowo usunąć
      const std::deque<acc_tuple>& getHistory() {
        // we actually acquire the timer only once
        // to be precise, we should do it in every step
        // however, for a relatively low number of elements, this is more effective
        struct timeval now_time;
        gettimeofday(&now_time, NULL);
        
        int size = history_.size();
        for (unsigned int i = 0; i < size; ++i) {
          // if we are here, than the deque is not empty for sure. the reason is twofold:
          //   1. if the deque was empty in the first place, the for would perform 0 iterations
          //   2. at i-th step, i-1 elements were already removed, so there are c-(i-1) elements in the deque
          //      (c = count = size - 1) which equals to 1 if (i == c)
          
          // access last element
          const acc_tuple & element = history_.back();
          if (SeekurJrRC::Utils::gTODDiffToMsec(&now_time, &element.get<3>()) > max_history_time_)
            history_.pop_back();
          else
            break; // we have reached a "good" element
        }
        // return the resulting, "sane" deque
        return history_;
      }
      
      /// convert alpha and beta values to robot speeds.
      /// vide thesis
      /// AB.first  - alpha
      /// AB.second - beta
      std::pair<float, float> speedValuesFromAB(std::pair<float,float> AB) {
        // explicit type casting, to avoid confusion
        return std::make_pair((float)V_MAX * (AB.first-AB.second), (float)V_MAX * (AB.first+AB.second));
      }
      
      /// get Phi in degrees
      float getPhiDeg(const acc_tuple& tuple) {
        return RAD_TO_DEG * getPhiRad(tuple);
      }
      
      /// get Phi in radians
      float getPhiRad(const acc_tuple& tuple) {
        const float& a_x = tuple.get<0>();
        const float& a_y = tuple.get<1>();
        const float& a_z = tuple.get<2>();
        return atan2(a_y,a_z);
      }
      
      /// get Theta in degrees
      float getThetaDeg(const acc_tuple& tuple) {
        return RAD_TO_DEG * getThetaRad(tuple);
      }
      
      /// get Theta in radians
      float getThetaRad(const acc_tuple& tuple) {
        const float& a_x = tuple.get<0>();
        const float& a_y = tuple.get<1>();
        const float& a_z = tuple.get<2>();
        float phi_r = getPhiRad(tuple);
        // return with a minus sign; all the models are implemented
        // to use theta measured the other way around.
        return -atan2(-a_x, a_y * sin(phi_r) + a_z * cos(phi_r));
      }
      
      /// get current value according to simple moving average (SMA)
      /// let's hope for RWO
      const acc_tuple getCurrentValueFilteredSimple() {
          const std::deque<acc_tuple>& history = getHistory();
          
          float f_ax = 0;
          float f_ay = 0;
          float f_az = 0;
          int count = 0;
          for (std::deque<acc_tuple>::const_iterator it = history.begin(); it != history.end(); ++it) {
              ++count;
              f_ax += (*it).get<0>();
              f_ay += (*it).get<1>();
              f_az += (*it).get<2>();
          }
          if (!count) // if history is empty (actually, should not happen), to avoid division by 0
              count = 1;
          f_ax /= (float) count;
          f_ay /= (float) count;
          f_az /= (float) count;
          
          acc_tuple ret(f_ax, f_ay, f_az);
          gettimeofday(&ret.get<3>(), NULL);
          
          return ret;
      }
      
      /// get current value according to exponentially-weighted moving average (EMA)
      /// let's hope for RWO
      const acc_tuple getCurrentValueFilteredExponential() {
          const std::deque<acc_tuple>& history = getHistory();
          
          float f_ax = 0;
          float f_ay = 0;
          float f_az = 0;

          const float alpha = 2.0 / (history.size() + 1);
          float denum = 0;
          float coeff = 1.0/(1 - alpha); // for ease of later calculations :P
          
          for (std::deque<acc_tuple>::const_iterator it = history.begin(); it != history.end(); ++it) {
              // bo:
              // begin -- najwcześniejsze próbki
              // end   -- najstarsze próbki              
              coeff *= (1-alpha);
              denum += coeff;
              
              f_ax += coeff * (*it).get<0>();
              f_ay += coeff * (*it).get<1>();
              f_az += coeff * (*it).get<2>();
          }
          if (!denum) // if history is empty, to avoid division by 0
              denum = 1;
          f_ax /= denum;
          f_ay /= denum;
          f_az /= denum;
          
          acc_tuple ret(f_ax, f_ay, f_az);
          gettimeofday(&ret.get<3>(), NULL);
          
          return ret;         
      }
      
      /// funkcje potrzebne do bezpośredniego wykorzystania danych z ewolucji
      float gp_add(float a, float b) {
        return a+b;
      }
      
      float gp_sub(float a, float b) {
        return a-b;
      }
      
      float gp_mul(float a, float b) {
        return a*b;
      }
      
      float gp_sqrt(float a) {
        return sqrt(fabs(a));
      }
      
      float gp_div(float a, float b) {
        // niestety programowanie genetyczne ma to do siebie
        if (b == 0)
          return 10000000000000.0;
        else
          return float(a)/float(b);
      }
      
      std::pair<float, float> genetic1(const acc_tuple& acc) {
        const float phi = getPhiDeg(acc);
        const float theta = getThetaDeg(acc);
        float alpha = gp_mul(gp_div(gp_sqrt(gp_add(theta, theta)), theta), gp_sqrt(gp_add(gp_sqrt(theta), gp_sqrt(theta))));
        float beta = gp_div(theta, gp_sub(gp_div(gp_sub(theta, theta), phi), gp_add(phi, phi)));
        return std::make_pair(alpha,beta);
      }
      
      std::pair<float, float> genetic2(const acc_tuple& acc) {
        const float phi = getPhiDeg(acc)/90.0;
        const float theta = getThetaDeg(acc)/90.0;
        
        // taken straight from pyevolve
        // float alpha = gp_sub(theta, gp_mul(gp_mul(gp_sub(gp_sqrt(gp_add(theta, theta)), gp_sqrt(gp_mul(theta, theta))), gp_sqrt(gp_mul(gp_sqrt(phi), phi))), theta));
        // float beta = gp_mul(gp_sub(phi, gp_sub(theta, theta)), gp_sub(gp_sqrt(theta), gp_sqrt(gp_add(gp_sqrt(gp_add(phi, phi)), theta))));
        float alpha = gp_mul(gp_sqrt(gp_sub(gp_sqrt(gp_sqrt(gp_add(theta, theta))), gp_sqrt(gp_mul(gp_mul(phi, theta), gp_sqrt(phi))))), theta);
        float beta = gp_mul(gp_mul(theta, phi), gp_sub(gp_mul(gp_sqrt(gp_mul(theta, gp_sqrt(phi))), gp_sqrt(theta)), gp_sqrt(gp_add(theta, theta))));
        return std::make_pair(alpha,beta);
      }
      
      std::pair<float, float> shepard(const acc_tuple& acc, float p) {
        const float phi = getPhiDeg(acc);
        const float theta = getThetaDeg(acc);
        float weight = 0;
        float weight_sum = 0;
        float nominator_a = 0;
        float nominator_b = 0;
        
        for (int i = 0; i < 9; ++i) {
          weight = pow(dist(phi, theta, c_arr_phi[i], c_arr_theta[i]),-p);
          weight_sum += weight;
          nominator_a += weight * c_arr_z_a[i];
          nominator_b += weight * c_arr_z_b[i];
        }
        
        float alpha = nominator_a / weight_sum;
        float beta  = nominator_b / weight_sum;
        
        return std::make_pair(alpha,beta);
      }
      
      std::pair<float, float> bilinear(const acc_tuple& acc) {
        const float phi = getPhiDeg(acc); // == x
        const float theta = getThetaDeg(acc); // == y
        
        // normally, this wouldn't be a part of the algorithm
        int i_11, i_12, i_21, i_22; // indeksy odpowiadające punktom Q_11, Q_12, Q_21, Q_22 w dokumentacji; x - pierwszy indeks, y - drugi indeks
        if (phi >= 0) {
          if (theta >= 0) {
            // top-right quarter
            i_11 = 4;
            i_12 = 7;
            i_21 = 5;
            i_22 = 8;
          } else {
            // bottom-right corner
            i_11 = 1;
            i_12 = 4;
            i_21 = 2;
            i_22 = 5;
          }
        } else {
          if (theta >= 0) {
            // top-left corner
            i_11 = 3;
            i_12 = 6;
            i_21 = 4;
            i_22 = 7;
          } else {
            // bottom-left corner
            i_11 = 0;
            i_12 = 3;
            i_21 = 1;
            i_22 = 4;
          }
        }
        
        float dx = c_arr_phi[i_22] - c_arr_phi[i_11];
        float dy = c_arr_theta[i_22]-c_arr_theta[i_11];
        float f_R1_a =  c_arr_z_a[i_11] * (c_arr_phi[i_22] - phi) / dx + c_arr_z_a[i_21] * (phi - c_arr_phi[i_11]) / dx;
        float f_R1_b =  c_arr_z_b[i_11] * (c_arr_phi[i_22] - phi) / dx + c_arr_z_b[i_21] * (phi - c_arr_phi[i_11]) / dx;
        
        float f_R2_a =  c_arr_z_a[i_12] * (c_arr_phi[i_22] - phi) / dx + c_arr_z_a[i_22] * (phi - c_arr_phi[i_11]) / dx;
        float f_R2_b =  c_arr_z_b[i_12] * (c_arr_phi[i_22] - phi) / dx + c_arr_z_b[i_22] * (phi - c_arr_phi[i_11]) / dx;
        
        float alpha = f_R1_a * (c_arr_theta[i_22] - theta) / dy + f_R2_a * (theta - c_arr_theta[i_11]) / dy;
        float beta  = f_R1_b * (c_arr_theta[i_22] - theta) / dy + f_R2_b * (theta - c_arr_theta[i_11]) / dy;
        
        return std::make_pair(alpha,beta);
      }
      
      //                                         0     1     2    3    4     5    6    7     8
      //                                      -------------------------------------------------
      static const float c_arr_phi[9];//   = { -90,    0,   90, -90,   0,   90, -90,   0,   90};
      static const float c_arr_theta[9];// = { -90,  -90,  -90,   0,   0,    0,  90,  90,   90};
      static const float c_arr_z_a[9];//   = {-0.5, -1.0, -0.5, 0.0, 0.0,  0.0, 0.5, 1.0,  0.5};
      static const float c_arr_z_b[9];//   = {-0.5,    0,  0.5, 1.0, 0.0, -1.0, 0.5, 0.0, -0.5};


      /// aktualne wartości wskazań akcelerometru
      /// alternatywnie modele pochodne mogą korzystać tylko z tego, jeżeli nie
      /// potrzebują elementów z historii
      const acc_tuple current_acc_;

    private:
      /// w tej metodzie zwracamy tuple 4-elementowe; czas ustawiamy, jeżeli setTime == true (domyślnie)
      acc_tuple getNormalizedAccTuple(const float& x, const float& y, const float& z, bool setTime = true) {
        float len = sqrt(x*x + y*y + z*z);
        float a_x = (fabs(x) < 0.8) ? 0 : x;
        float a_y = (fabs(y) < 0.8) ? 0 : y;
        float a_z = (fabs(z) < 0.8) ? 0 : z;
        acc_tuple ret(x/len, y/len, z/len);
        if (setTime)
          gettimeofday(&ret.get<3>(), NULL);
        return ret;
      };
      
      /// return sqrt((x1-x2)**2 + (y1-y2)**2)
      float dist(float x1, float y1, float x2, float y2) {
        return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
      };
      
      /// historia wartości wskazań akcelerometru
      /// uczynienie tego statycznym pozwala nam na wykorzystanie tych samych danych
      /// w różnych modelach (tj. płynne zmienianie modeli - nie będzie potrzeby zbierania danych od nowa)
      static std::deque<acc_tuple> history_;
      static const unsigned int max_history_length_ = 20;
      static const unsigned int max_history_time_ = 1000; // [msec]
    };
       
    class BilinearNoFiltModel : public SteeringModelBase {
    public:
      BilinearNoFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class BilinearSimpleFiltModel : public SteeringModelBase {
    public:
      BilinearSimpleFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class BilinearExpFiltModel : public SteeringModelBase {
    public:
      BilinearExpFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Shepard1_5NoFiltModel : public SteeringModelBase {
    public:
      Shepard1_5NoFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Shepard1_5SimpleFiltModel : public SteeringModelBase {
    public:
      Shepard1_5SimpleFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Shepard1_5ExpFiltModel : public SteeringModelBase {
    public:
      Shepard1_5ExpFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Shepard4_5NoFiltModel : public SteeringModelBase {
    public:
      Shepard4_5NoFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Shepard4_5SimpleFiltModel : public SteeringModelBase {
    public:
      Shepard4_5SimpleFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Shepard4_5ExpFiltModel : public SteeringModelBase {
    public:
      Shepard4_5ExpFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Genetic1NoFiltModel : public SteeringModelBase {
    public:
      Genetic1NoFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Genetic1SimpleFiltModel : public SteeringModelBase {
    public:
      Genetic1SimpleFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Genetic1ExpFiltModel : public SteeringModelBase {
    public:
      Genetic1ExpFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Genetic2NoFiltModel : public SteeringModelBase {
    public:
      Genetic2NoFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Genetic2SimpleFiltModel : public SteeringModelBase {
    public:
      Genetic2SimpleFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    
    class Genetic2ExpFiltModel : public SteeringModelBase {
    public:
      Genetic2ExpFiltModel(const float& x, const float& y, const float& z) : SteeringModelBase(x, y, z) {};
      std::pair<float, float> getSpeedValues();
    };
    


    /**
     * \param modelCode - kod modelu odpowiadający jednem z define'ów
     * \param x,y,z     - wartości wskazań akcelerometru
     *
     * Metoda konstruująca odpowiedni model i zwracająca do niego wskaźnik. W przypadku niepowodzenia prawdopodobnie rzucimy wyjątek.
     */
    boost::shared_ptr<SteeringModelBase> getSteeringModel(uint8_t modelCode, const float& x, const float& y, const float& z);
  }
}

#endif
