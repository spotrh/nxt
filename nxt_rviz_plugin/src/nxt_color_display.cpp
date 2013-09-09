#include "nxt_color_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/float_property.h"
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
NXTColorDisplay::NXTColorDisplay()
  : Display()
  , messages_received_(0)
{
  display_property_ = new rviz::FloatProperty( "Display Length", 0.003f,
                                       "Display length.",
                                       this, SLOT( updateDisplayLength() ));
  alpha_property_ = new rviz::FloatProperty( "Alpha", 0.5f,
                                       "Amount of transparency to apply to the circle.",
                                       this, SLOT( updateAlpha() ));
  topic_property_ = new rviz::RosTopicProperty( "Topic", "",
                                       QString::fromStdString( ros::message_traits::datatype<nxt_msgs::Color>() ),
                                       "nxt_msgs::Color topic to subscribe to."
                                       this, SLOT (updateTopic() ));
}

NXTColorDisplay::~NXTColorDisplay()
{
  unsubscribe();
  clear();
  delete cylinder_;
  delete tf_filter_;
}

void NXTColorDisplay::onInitialize()
{
  float alpha = alpha_property_->getFloat();
  float display = display_property_->getFloat();

  tf_filter_ = new tf::MessageFilter<nxt_msgs::Color>(*context_->getTFClient(), "", 10, update_nh_);
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  cylinder_ = new rviz::Shape(rviz::Shape::Cylinder, context_->getSceneManager(), scene_node_);

  scene_node_->setVisible( false );

  // setAlpha( 0.5f );
  // setDisplayLength( 0.003f );

  Ogre::Vector3 scale( 0, 0, 0);
  // rviz::scaleRobotToOgre( scale );
  cylinder_->setScale(scale);

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&NXTColorDisplay::incomingMessage, this, _1));
  context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

void NXTColorDisplay::clear()
{

  messages_received_ = 0;
  setStatus( rviz::StatusProperty::Warn, "Topic", "No messages received" );
}

void NXTColorDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic_, 10);
}

void NXTColorDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void NXTColorDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void NXTColorDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void NXTColorDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
}

void NXTColorDisplay::update(float wall_dt, float ros_dt)
{
}


void NXTColorDisplay::processMessage(const nxt_msgs::Color::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;

  setStatus( rviz::StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.z = -0.0033;
  pose.position.y = 0;
  pose.position.x = 0.0185 + displayLength_/2;
  pose.orientation.x = 0.707;
  pose.orientation.z = -0.707;
  if (!context_->getFrameManager()->transform(msg->header, pose, position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  cylinder_->setPosition(position);
  cylinder_->setOrientation(orientation);
  Ogre::Vector3 scale( 0.0155, 0.0155, displayLength_);
  // rviz::scaleRobotToOgre( scale );
  cylinder_->setScale(scale);
  cylinder_->setColor(msg->r, msg->g, msg->b, alpha_);

}

void NXTColorDisplay::incomingMessage(const nxt_msgs::Color::ConstPtr& msg)
{
  processMessage(msg);
}

void NXTColorDisplay::reset()
{
  Display::reset();
  clear();
}

void NXTColorDisplay::updateAlpha()
{
  cylinder_->setAlpha( alpha_property_->getFloat() );
  context_->queueRender();
}

void NXTColorDisplay::updateDisplayLength()
{
  cylinder_->setDisplayLength( display_property_->getFloat() );
  context_->queueRender();
}

void NXTColorDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

} // namespace nxt_rviz_plugin    

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( nxt_rviz_plugin::NXTColorDisplay, rviz::Display )

