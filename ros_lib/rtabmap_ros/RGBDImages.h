#ifndef _ROS_rtabmap_ros_RGBDImages_h
#define _ROS_rtabmap_ros_RGBDImages_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "rtabmap_ros/RGBDImage.h"

namespace rtabmap_ros
{

  class RGBDImages : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t rgbd_images_length;
      typedef rtabmap_ros::RGBDImage _rgbd_images_type;
      _rgbd_images_type st_rgbd_images;
      _rgbd_images_type * rgbd_images;

    RGBDImages():
      header(),
      rgbd_images_length(0), rgbd_images(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->rgbd_images_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rgbd_images_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rgbd_images_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rgbd_images_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rgbd_images_length);
      for( uint32_t i = 0; i < rgbd_images_length; i++){
      offset += this->rgbd_images[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t rgbd_images_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rgbd_images_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rgbd_images_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rgbd_images_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rgbd_images_length);
      if(rgbd_images_lengthT > rgbd_images_length)
        this->rgbd_images = (rtabmap_ros::RGBDImage*)realloc(this->rgbd_images, rgbd_images_lengthT * sizeof(rtabmap_ros::RGBDImage));
      rgbd_images_length = rgbd_images_lengthT;
      for( uint32_t i = 0; i < rgbd_images_length; i++){
      offset += this->st_rgbd_images.deserialize(inbuffer + offset);
        memcpy( &(this->rgbd_images[i]), &(this->st_rgbd_images), sizeof(rtabmap_ros::RGBDImage));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/RGBDImages"; };
    const char * getMD5(){ return "aea27d3dc751e4501a1992b6fb5ad47e"; };

  };

}
#endif
