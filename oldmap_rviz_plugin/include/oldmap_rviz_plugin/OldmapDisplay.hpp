//
// Created by wj on 22. 5. 31.
//

// setting about "Displays" panel in rviz

#ifndef OLDMAP_OLDMAPDISPLAY_HPP
#define OLDMAP_OLDMAPDISPLAY_HPP
#ifndef Q_MOC_RUN

#include <oldmap_ros/oldmap_ros.hpp>
#include <oldmap_msgs/Oldmap.h>
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>

//#include "grid_map_rviz_plugin/modified/message_filter_display.h"
#endif

namespace Ogre
{
    class SceneNode;
}

namespace rviz
{
    class FloatProperty;

    class IntProperty;

    class ColorProperty;
}

namespace heightmap_rviz_plugin
{
    class HeightmapVisual;

    class HeightmapDisplay : public rviz::MessageFilterDisplay<oldmap_msgs::Oldmap>
    {
    Q_OBJECT
    public:
        HeightmapDisplay();
        virtual ~HeightmapDisplay();

    protected:
        virtual void onInitialize();    // This is where we instantiate all the workings of the class.
        virtual void reset();            // Clear the visuals by deleting their objects.

        //  Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:
        void updateHistoryLength();
        void updateVisualization();

    private:
        // Callback for incoming ROS messages
        void processMessage(const oldmap_msgs::Oldmap::ConstPtr& msg);

        boost::mutex mMutex;
        // Storage for the list of visuals
        // circular buffer where data gets popped from the front (oldest) and pushed to the back (newest)
        boost::circular_buffer<boost::shared_ptr<HeightmapVisual>> mVisuals;

        // User-editable property variables.
//		rviz::IntProperty* mQueueSizeProperty;
        rviz::RosTopicProperty* mHeightmapTopicProperty;
        rviz::IntProperty* mHistoryLengthProperty;
        rviz::FloatProperty* mAlphaProperty;
        rviz::ColorProperty* mColorProperty;

        u_int32_t mQueueSize;
    };
}

#endif //OLDMAP_OLDMAPDISPLAY_HPP
