#ifndef NXT_COLOR_DISPLAY_H
#define NXT_COLOR_DISPLAY_H

#include "rviz/display.h"
#include "rviz/helpers/color.h"
//#include "rviz/properties/forwards.h"
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <nxt_msgs/Color.h>

#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace rviz
{
class Shape;
}


namespace Ogre
{
class SceneNode;
}

namespace nxt_rviz_plugin
{

/**
 * \class NXTColorDisplay
 * \brief Displays a nxt_msgs::Color message
 */
class NXTColorDisplay : public rviz::Display
{
public:
  NXTColorDisplay();
  virtual ~NXTColorDisplay();

  virtual void onInitialize();

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  static const char* getTypeStatic() { return "Color"; }
  virtual const char* getType() const { return getTypeStatic(); }

private Q_SLOTS:
  void updateAlpha();
  void updateDisplayLength();
  void updateTopic();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const nxt_msgs::Color::ConstPtr& msg);
  void processMessage(const nxt_msgs::Color::ConstPtr& msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  float alpha_;
  float displayLength_;

  uint32_t messages_received_;

  Ogre::SceneNode* scene_node_;
  rviz::Shape* cylinder_;      ///< Handles actually drawing the cone

  message_filters::Subscriber<nxt_msgs::Color> sub_;
  tf::MessageFilter<nxt_msgs::Color>* tf_filter_;
  nxt_msgs::Color::ConstPtr current_message_;

  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* display_property_;
  rviz::RosTopicProperty* topic_property_;
};

} // namespace nxt_rviz_plugin

#endif /* NXT_COLOR_DISPLAY_H */

