/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef NXT_ULTRASONIC_DISPLAY_H
#define NXT_ULTRASONIC_DISPLAY_H

#include "rviz/display.h"
#include "rviz/helpers/color.h"
// #include "rviz/properties/forwards.h"
#include <rviz/properties/property.h>
#include <rviz/properties/color_property.h> 
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <nxt_msgs/Range.h>

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
 * \class NXTUltrasonicDisplay
 * \brief Displays a nxt_msgs::Range message
 */
class NXTUltrasonicDisplay : public rviz::Display
{
public:
  NXTUltrasonicDisplay();
  virtual ~NXTUltrasonicDisplay();

  virtual void onInitialize();

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  static const char* getTypeStatic() { return "Range"; }
  virtual const char* getType() const { return getTypeStatic(); }

private Q_SLOTS:
  void updateColor();
  void updateTopic();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const nxt_msgs::Range::ConstPtr& msg);
  void processMessage(const nxt_msgs::Range::ConstPtr& msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  rviz::Color color_;
  float alpha_;

  uint32_t messages_received_;

  Ogre::SceneNode* scene_node_;
  rviz::Shape* cone_;      ///< Handles actually drawing the cone

  message_filters::Subscriber<nxt_msgs::Range> sub_;
  tf::MessageFilter<nxt_msgs::Range>* tf_filter_;
  nxt_msgs::Range::ConstPtr current_message_;

  rviz::ColorProperty* color_property_;
  rviz::RosTopicProperty* topic_property_;
  rviz::FloatProperty* alpha_property_;
};

} // namespace nxt_rviz_plugin

#endif /* NXT_ULTRASONIC_DISPLAY_H */

