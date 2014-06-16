#ifndef VIDEREDC1394CONTROL_H
#define VIDEREDC1394CONTROL_H

#include <stdexcept>
#include <dc1394/dc1394.h>

// These defines are taken from libdc1394 to redefine how some features
// are set on Videre Stereo cameras (whitebalance, )
#define REG_CAMERA_WHITE_BALANCE            0x80CU
#define REG_CAMERA_FEATURE_HI_BASE          0x800U
#define REG_CAMERA_FEATURE_LO_BASE          0x880U

#define FEATURE_TO_VALUE_OFFSET(feature, offset)                                        \
    {                                                                                   \
        if ( (feature > DC1394_FEATURE_MAX) || (feature < DC1394_FEATURE_MIN) )         \
            return DC1394_FAILURE;                                                      \
        else if (feature < DC1394_FEATURE_ZOOM)                                         \
            offset= REG_CAMERA_FEATURE_HI_BASE+(feature - DC1394_FEATURE_MIN)*0x04U;    \
        else if (feature >= DC1394_FEATURE_CAPTURE_SIZE)                                \
            offset= REG_CAMERA_FEATURE_LO_BASE +(feature+12-DC1394_FEATURE_ZOOM)*0x04U; \
        else                                                                            \
            offset= REG_CAMERA_FEATURE_LO_BASE +(feature-DC1394_FEATURE_ZOOM)*0x04U;    \
    }

dc1394error_t
dc1394_feature_whitebalance_set_value_blind(dc1394camera_t *camera, uint32_t u_b_value, uint32_t v_r_value)
{
    uint32_t curval = ( ((u_b_value & 0xFFFUL) << 12) | (v_r_value & 0xFFFUL) );
    dc1394error_t err = dc1394_set_control_register(camera, REG_CAMERA_WHITE_BALANCE, curval);
    DC1394_ERR_RTN(err, "Could not set white balance");

    return err;
}

dc1394error_t
dc1394_feature_set_value_blind(dc1394camera_t *camera, dc1394feature_t feature, uint32_t value)
{
    uint64_t offset;
    dc1394error_t err;

    if ( (feature < DC1394_FEATURE_MIN) || (feature > DC1394_FEATURE_MAX) ) { return DC1394_INVALID_FEATURE; }

    if ((feature == DC1394_FEATURE_WHITE_BALANCE) ||
        (feature == DC1394_FEATURE_WHITE_SHADING) ||
        (feature == DC1394_FEATURE_TEMPERATURE))
    {
        err = DC1394_INVALID_FEATURE;
        DC1394_ERR_RTN(err, "You should use the specific functions to write from multiple-value features");
    }

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    err = dc1394_set_control_register(camera, offset, (value & 0xFFFUL));
    DC1394_ERR_RTN(err, "Could not set feature value");

    return err;
}

dc1394error_t
dc1394_feature_set_mode_blind(dc1394camera_t *camera, dc1394feature_t feature, dc1394feature_mode_t mode)
{
    dc1394error_t err;
    uint64_t offset;
    uint32_t value;

    if ( (feature < DC1394_FEATURE_MIN) || (feature > DC1394_FEATURE_MAX) ) { return DC1394_INVALID_FEATURE; }
    if ( (mode < DC1394_FEATURE_MODE_MIN) || (mode > DC1394_FEATURE_MODE_MAX) ) { return DC1394_INVALID_FEATURE_MODE; }

    if (feature == DC1394_FEATURE_TRIGGER) { return DC1394_INVALID_FEATURE; }

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (mode == DC1394_FEATURE_MODE_AUTO)
    {
        value = 0x01000000UL;
        err = dc1394_set_control_register(camera, offset, value);
        DC1394_ERR_RTN(err, "Could not set auto mode for feature");
    }
    else if (mode == DC1394_FEATURE_MODE_MANUAL)
    {
      value = 0;
      err = dc1394_set_control_register(camera, offset, value);
      DC1394_ERR_RTN(err, "Could not set auto mode for feature");
    }
    else
    {
        throw std::runtime_error("Unable so set mode");
    }

    return err;
}

#endif
