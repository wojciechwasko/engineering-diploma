#include <iostream>
#include <stdint.h>

#include <sys/time.h> // for gettimeofday
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "Aria.h"

#include "driver.hpp"
#include "server.hpp"
#include "utils.hpp"
#include "steering_model.hpp"

using SeekurJrRC::Core::Driver;
using SeekurJrRC::Core::SteeringModelBase;


boost::asio::io_service::id Driver::id;
struct timeval Driver::nowTime_;
struct timeval Driver::lastMotorsUpdate_;

struct timeval dummy;
uint64_t counter = 0;
uint64_t skip_first = 200;
float average = 0;

void Driver::checkMotors(boost::shared_ptr<boost::asio::deadline_timer>& p_checkMotorsTimer)
{
  // re-set the timer
  p_checkMotorsTimer->expires_at(p_checkMotorsTimer->expires_at() + boost::posix_time::milliseconds(stopMotorsCheckInterval_));
  p_checkMotorsTimer->async_wait(
    boost::bind(
      &Driver::checkMotors,
      this,
      p_checkMotorsTimer
    )
  );
  
//   std::cout << "Checking motors speed update interval." << std::endl;
  
  gettimeofday(&nowTime_, NULL);
  if (SeekurJrRC::Utils::gTODDiffToMsec(&nowTime_, &lastMotorsUpdate_) > stopMotorsTimeout_) {
    // stop the motors    
    std::cout << "\rTimeout reached. Stopping motors." << std::endl;
    robot_->lock();
    robot_->stop();
    robot_->unlock();
  }
}

Driver::Driver(boost::asio::io_service& ios) : service(ios), p_IOService_(&ios)
{
  Aria::init();
  robot_ = new ArRobot();
  
  int _argc = 1;
  char _argv[] = "server";
  char * _p_argv = &_argv[0];
  robot_arg_parser_ = new ArArgumentParser(&_argc, &_p_argv);
  robot_connector_ = new ArRobotConnector(robot_arg_parser_, robot_);

  if (!robot_connector_->connectRobot())
  {
    ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
    Aria::exit(1);
  }
  
  robot_->runAsync(false);
  robot_->enableMotors();
  
  boost::shared_ptr<boost::asio::deadline_timer> p_checkMotorsTimer(
    new boost::asio::deadline_timer(
      ios, 
      boost::posix_time::milliseconds(stopMotorsCheckInterval_)
    )
  );
  
  // Timer object is being passed to the handler
  p_checkMotorsTimer->async_wait(
    boost::bind(
      &Driver::checkMotors,
      this,
      p_checkMotorsTimer
    )
  );
}

void Driver::shutdown_service() {
  robot_->lock();
  robot_->stop();
  robot_->unlock();
  robot_->stopRunning();
  delete robot_;
  delete robot_arg_parser_;
  delete robot_connector_;
  Aria::shutdown();
}

void Driver::processMessage(TCPMessage& message)
{
  if (message.startStop_) {
    boost::shared_ptr<SteeringModelBase> model = SeekurJrRC::Core::getSteeringModel(message.steeringModelCode_, message.x_, message.y_, message.z_);
    if (model != NULL) {
      if (!robot_->areMotorsEnabled())
        std::cout << "Motors disabled." << std::endl;
      if (skip_first)
        skip_first--;
      else {
        gettimeofday(&dummy, NULL);
        average = (average * counter + SeekurJrRC::Utils::gTODDiffToMsec(&dummy, &lastMotorsUpdate_)) / (float)(counter+1);
        counter++;
        std::cout << "\r" << average << std::flush;
      }
      std::pair<float, float> v = model->getSpeedValues();
//       std::cout << "\rL: " << v.first << " R: " << v.second << std::flush;
//       std::cout << "Model returned: v1=" << v.first << ", v2=" << v.second << std::endl;
//       robot_->lock();
//       robot_->setVel2(v.first, v.second);
//       robot_->setVel2(50, -50);
//       robot_->setRotVel(50);
      float v_trans = 0.5 * (v.first + v.second);
      float omega = (v.second - v.first) / 20.0;
      robot_->setRotVel(omega);
      robot_->setVel(v_trans);
      robot_->unlock();
      gettimeofday(&lastMotorsUpdate_, NULL);
    }
  }
}


