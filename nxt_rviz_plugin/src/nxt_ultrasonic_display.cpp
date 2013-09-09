#include "nxt_ultrasonic_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/ros_topic_property.h"
//#include "rviz/properties/property_manager.h"
//#include "rviz/common.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <rviz/ogre_helpers/shape.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace nxt_rviz_plugin
{
NXTUltrasonicDisplay::NXTUltrasonicDisplay()
  : Display()
  , messages_received_(0)
{
  color_property_ = new ColorProperty( "Color", Qt::gray, 
                                       "Color to draw the range.",
                                       this, SLOT( updateColor() ));
  alpha_property_ = new FloatProperty( "Alpha", 0.5f,
                                       "Amount of transparency to apply to the range.",
                                       this, SLOT( updateColor() ));
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<nxt_msgs::Range>() ),
                                          "nxt_msgs::Range topic to subscribe to.",
                                          this, SLOT (updateTopic() ));
}

NXTUltrasonicDisplay::~NXTUltrasonicDisplay()
{
  unsubscribe();
  clear();
  delete cone_;
  delete tf_filter_;
}

void NXTUltrasonicDisplay::onInitialize()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  float alpha = alpha_property_->getFloat();

  tf_filter_ = new tf::MessageFilter<nxt_msgs::Range>(*context_->getTFClient(), "", 10, update_nh_);
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  cone_ = new rviz::Shape(rviz::Shape::Cone, context_->getSceneManager(), scene_node_);

  scene_node_->setVisible( false );

  // setAlpha( 0.5f );
  Ogre::Vector3 scale( 0, 0, 0);
  // rviz::scaleRobotToOgre( scale );
  cone_->setScale(scale);
  cone_->setColor(color.r, color.g, color.b, alpha);

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&NXTUltrasonicDisplay::incomingMessage, this, _1));
  context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

void NXTUltrasonicDisplay::clear()
{

  messages_received_ = 0;
  setStatus( StatusProperty::Warn, "Topic", "No messages received" );
}

void NXTUltrasonicDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic_, 10);
}

void NXTUltrasonicDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void NXTUltrasonicDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void NXTUltrasonicDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void NXTUltrasonicDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
}

void NXTUltrasonicDisplay::update(float wall_dt, float ros_dt)
{
}


void NXTUltrasonicDisplay::processMessage(const nxt_msgs::Range::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(rviz::status_levels::Ok, "Topic", ss.str());
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.z = pose.position.y = 0;
  pose.position.x = msg->range/2;
  pose.orientation.x = 0.707;
  pose.orientation.z = -0.707;
  if (!context_->getFrameManager()->transform(msg->header, pose, position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  cone_->setPosition(position);
  cone_->setOrientation(orientation); 
  Ogre::Vector3 scale( sin(msg->spread_angle) * msg->range, sin(msg->spread_angle) * msg->range , msg->range);
  // rviz::scaleRobotToOgre( scale );
  cone_->setScale(scale);
  cone_->setColor(color_.r_, color_.g_, color_.b_, alpha_);

}

void NXTUltrasonicDisplay::incomingMessage(const nxt_msgs::Range::ConstPtr& msg)
{
  processMessage(msg);
}

void NXTUltrasonicDisplay::reset()
{
  Display::reset();
  clear();
}

void NXTUltrasonicDisplay::updateColor()
{
  QColor color = color_property_->getColor();
  color.setAlphaF( alpha_property_->getFloat() );
  grid_->setColor( qtToOgre( color ));
  context_->queueRender();
}

void NXTUltrasonicDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

const char* NXTUltrasonicDisplay::getDescription()
{
  return "Displays data from a nxt_msgs::Range message as a cone.";
}
} // namespace nxt_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::NXTUltrasonicDisplay, rviz::Display )
