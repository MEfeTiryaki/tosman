#include "tosman_state_estimator/TosmanEKF.hpp"

namespace estimator {

TosmanEKF::TosmanEKF(Model& model)
    : KulmanStateEstimatorBase(model),
      n_(6),
      m_(6),
      l_(2),
      delT_(0.01),
      time_(0.0),
      U_(),
      Xp_(),
      Pm_(),
      Pp_(),
      Xm_(),
      Q_(),
      R_(),
      F_(),
      H_(),
      K_(),
      imuSubscriberQueueSize_(0),
      rWheel_(0.15),
      lWheel_(0.5)
{

}

void TosmanEKF::initilize(ros::NodeHandle* nh)
{
  /*
   * X_k = [ x_k
   *       , y_k
   *       , theta_k
   *       , xdot_k
   *       , ydot_k
   *       , thetadot_k ]
   *
   */

  U_.setZero(l_);
  Xp_.setZero(n_);
  Xm_.setZero(n_);
  Pp_.setZero(n_, n_);
  Pm_.setZero(n_, n_);
  Eye_.setIdentity(n_, n_);
  Q_.setZero(n_, n_);
  R_.setZero(m_, m_);
  F_.setZero(n_, n_);
  H_.setZero(m_, n_);
  K_.setZero(n_, n_);
  S_.setZero(n_, n_);
  h_.setZero(m_);
  Z_.setZero(m_);

  KulmanStateEstimatorBase::initilize(nh);
  time_ = ros::Time::now().toSec();
}

void TosmanEKF::advance(double dt)
{

  getInput();
  getSensorData();
  pStep();
  mStep();

  //  positionWorldToBase_[0] = kulmanStateMsg_.;
  //  positionWorldToBase_[1] = kulmanStateMsg_.pose.pose.pos.y;
  //  positionWorldToBase_[2] = kulmanStateMsg_.pose.pose.pos.z;
  positionWorldToBase_[0] = kulmanStateMsg_.pose.pose.position.x;
  positionWorldToBase_[1] = kulmanStateMsg_.pose.pose.position.y;
  positionWorldToBase_[2] = kulmanStateMsg_.pose.pose.position.z;

  orientationWorldToBase_[0] = kulmanStateMsg_.pose.pose.orientation.w;
  orientationWorldToBase_[1] = kulmanStateMsg_.pose.pose.orientation.x;
  orientationWorldToBase_[2] = kulmanStateMsg_.pose.pose.orientation.y;
  orientationWorldToBase_[3] = kulmanStateMsg_.pose.pose.orientation.z;

  velocityWorldToBaseInWorldFrame_[0] = kulmanStateMsg_.twist.twist.linear.x;
  velocityWorldToBaseInWorldFrame_[1] = kulmanStateMsg_.twist.twist.linear.y;
  velocityWorldToBaseInWorldFrame_[2] = kulmanStateMsg_.twist.twist.linear.z;

  angularVelocityWorldToBaseInWorldFrame_[0] = kulmanStateMsg_.twist.twist.angular.x;
  angularVelocityWorldToBaseInWorldFrame_[1] = kulmanStateMsg_.twist.twist.angular.y;
  angularVelocityWorldToBaseInWorldFrame_[2] = kulmanStateMsg_.twist.twist.angular.z;

  // Kestirilen konum,yönelim ve hızları ölçülen duruma ata.
  model_.getBody().getMeasuredState().setPositionInWorldFrame(positionWorldToBase_);
  model_.getBody().getMeasuredState().setOrientationInWorldFrame(orientationWorldToBase_);
  model_.getBody().getMeasuredState().setVelocityInWorldFrame(velocityWorldToBaseInWorldFrame_);
  model_.getBody().getMeasuredState().setAngularVelocityInWorldFrame(
      angularVelocityWorldToBaseInWorldFrame_);

      /*
  std::cout << "X_real : \n" << positionWorldToBase_[0] << "," << positionWorldToBase_[1] << ","
            << 2 * acos(orientationWorldToBase_[0]) << "," << velocityWorldToBaseInWorldFrame_[0]
            << "," << angularVelocityWorldToBaseInWorldFrame_[2] << "," << std::endl;
            */
  //std::cout << "____________________________________ " << std::endl;

  publishVisualization();

  publishEstimatedState();
}

void TosmanEKF::readParameters()
{
  KulmanStateEstimatorBase::readParameters();

  getParam(*nodeHandle_, "subscribers/imu/topic", imuSubscriberName_);
  getParam(*nodeHandle_, "subscribers/imu/queue_size", imuSubscriberQueueSize_);

  // Todo : buralara cok karisik duruyor, daha guzel bir yontem bulunmali yada yordam yazilmali
  double* ptr;
  std::vector<double> P;
  if (!nodeHandle_->getParam("estimatorParameters/P", P))
    ROS_ERROR("Estimator failed to find P matrix");
  ptr = &P[0];
  Pm_.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, n_);

  //std::cout << "P_m : \n" << Pm_ << std::endl;

  std::vector<double> Q;
  if (!nodeHandle_->getParam("estimatorParameters/Q", Q))
    ROS_ERROR("Estimator failed to find Q matrix");
  ptr = &Q[0];
  Q_.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, n_);
  //std::cout << "Q : \n" << Q_ << std::endl;

  std::vector<double> R;
  if (!nodeHandle_->getParam("estimatorParameters/R", R))
    ROS_ERROR("Estimator failed to find R matrix");
  ptr = &R[0];
  R_.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, m_);
  //std::cout << "R : \n" << R_ << std::endl;

}

void TosmanEKF::initilizeSubscribers()
{
  KulmanStateEstimatorBase::initilizeSubscribers();

  imuSubscriber_ = nodeHandle_->subscribe(imuSubscriberName_, imuSubscriberQueueSize_,
                                          &TosmanEKF::getImuMsg, this);
}
void TosmanEKF::initilizePublishers()
{
  vis_pub_ = nodeHandle_->advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

void TosmanEKF::publishVisualization()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "estimator_vis";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = Xm_[0];
  marker.pose.position.y = Xm_[1];
  marker.pose.position.z = 0.1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = sin(Xm_[2] / 2);
  marker.pose.orientation.w = cos(Xm_[2] / 2);
  marker.scale.x = 0.5;
  marker.scale.y = 0.2;
  marker.scale.z = 0.3;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  vis_pub_.publish(marker);
}

void  TosmanEKF::publishEstimatedState()
{
  kulmanStateEstimatorMsg_.pose.pose.position.x = Xm_[0] ;
  kulmanStateEstimatorMsg_.pose.pose.position.y = Xm_[1] ;
  kulmanStateEstimatorMsg_.pose.pose.position.z = 0.0 ;

  kulmanStateEstimatorMsg_.pose.pose.orientation.w = cos(Xm_[2] / 2) ;
  kulmanStateEstimatorMsg_.pose.pose.orientation.x = Xm_[1] ;
  kulmanStateEstimatorMsg_.pose.pose.orientation.y = Xm_[1] ;
  kulmanStateEstimatorMsg_.pose.pose.orientation.z = sin(Xm_[2] / 2) ;

  kulmanStateEstimatorMsg_.twist.twist.linear.x = Xm_[3] ;
  kulmanStateEstimatorMsg_.twist.twist.linear.y = Xm_[4] ;
  kulmanStateEstimatorMsg_.twist.twist.linear.z = 0.0 ;

  kulmanStateEstimatorMsg_.twist.twist.angular.x = 0.0 ;
  kulmanStateEstimatorMsg_.twist.twist.angular.y = 0.0 ;
  kulmanStateEstimatorMsg_.twist.twist.angular.z = Xm_[5] ;

  KulmanStateEstimatorBase::publishEstimatedState();
}

void TosmanEKF::getImuMsg(sensor_msgs::Imu msg)
{
  imuMsg_ = msg;
}

void TosmanEKF::getSensorData()
{
  double accX = imuMsg_.linear_acceleration.x;
  double accY = imuMsg_.linear_acceleration.y;

  Z_[0] = Xm_[0] + Xm_[3]*delT_+ 0.5 * accX *delT_*delT_;
  Z_[1] = Xm_[1] + Xm_[4]*delT_+ 0.5 * accY *delT_*delT_;
  Z_[2] = 2 * acos(imuMsg_.orientation.w);
  Z_[3] = Xm_[3]+ accX *delT_;
  Z_[4] = Xm_[4]+ accY *delT_;
  Z_[5] = imuMsg_.angular_velocity.z;

}

void TosmanEKF::getInput()
{
  // XXX : Wheel hizlarini aldim ama buralar karisik yine
  U_[0] = model_.getWheel()[0]->getDesiredState().getAngularVelocityInWorldFrame()[2];
  U_[1] = model_.getWheel()[1]->getDesiredState().getAngularVelocityInWorldFrame()[2];
}
void TosmanEKF::pStep()
{
  Xp_[0] = Xm_[0] + rWheel_/2 * cos(Xm_[2]) * delT_ * ( U_[0] + U_[1] ) ;
  Xp_[1] = Xm_[1] + rWheel_/2 * sin(Xm_[2]) * delT_ * ( U_[0] + U_[1] ) ;
  Xp_[2] = Xm_[2] + rWheel_/2/lWheel_ * delT_ * ( U_[0] - U_[1] ) ;
  Xp_[3] = Xm_[3];
  Xp_[4] = Xm_[4];
  Xp_[5] = Xm_[5];

  F_.row(0) << 1, 0, 0, delT_ , 0, 0;
  F_.row(1) << 0, 1, 0, 0, delT_, 0;
  F_.row(2) << 0, 0, 1, 0, 0, delT_;
  F_.row(3) << 0, 0, rWheel_/2 * -sin(Xm_[2]) * ( U_[0] + U_[1] ), 0, 0, 0;
  F_.row(4) << 0, 0, rWheel_/2 * cos(Xm_[2]) * ( U_[0] + U_[1] ), 0, 0, 0;
  F_.row(5) << 0, 0, 0, 0, 0, rWheel_/2/lWheel_ * ( U_[0] - U_[1] );

  Pp_ = F_ * Pm_ * F_.transpose() + Q_;

  //std::cout << "U : \n" << U_.transpose() << std::endl;
  //std::cout << "F   : \n" << F_ << std::endl;
  //std::cout << "X_p : \n" << Xp_.transpose() << std::endl;
  //std::cout << "P_p : \n" << Pp_ << std::endl;

}
void TosmanEKF::mStep()
{
  h_[0] = Xp_[0];
  h_[1] = Xp_[1];
  h_[2] = Xp_[2];
  h_[3] = Xp_[3];
  h_[4] = Xp_[4];
  h_[5] = Xp_[5];

  H_.row(0) << 1, 0, 0, 0, 0, 0;
  H_.row(1) << 0, 1, 0, 0, 0, 0;
  H_.row(2) << 0, 0, 1, 0, 0, 0;
  H_.row(3) << 0, 0, 0, 1, 0, 0;
  H_.row(4) << 0, 0, 0, 0, 1, 0;
  H_.row(5) << 0, 0, 0, 0, 0, 1;

  Y_ = Z_ - h_;

  S_ = H_ * Pp_ * H_.transpose() + R_;

  K_ = Pp_ * H_.transpose() * S_.inverse();

  Xm_ = Xp_ + K_ * Y_;
  Pm_ = (Eye_ - K_ * H_) * Pp_;

  //std::cout << "Y   : \n" << Y_.transpose() << std::endl;
  //std::cout << "Z   : \n" << Z_.transpose() << std::endl;
  //std::cout << "h   : \n" << h_.transpose() << std::endl;
  //std::cout << "H   : \n" << H_ << std::endl;
  //std::cout << "S   : \n" << S_ << std::endl;
  //std::cout << "K   : \n" << K_ << std::endl;

  //std::cout << "X_m : \n" << Xm_.transpose() << std::endl;
  //std::cout << "P_m : \n" << Pm_ << std::endl;

}

}
