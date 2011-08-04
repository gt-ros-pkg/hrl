
#include "kelsey_sandbox/delay_filter.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(kelsey_sandbox, DelayFilterDouble, filters::DelayFilter<double>, filters::FilterBase<double>)
PLUGINLIB_DECLARE_CLASS(kelsey_sandbox, DelayFilterFloat, filters::DelayFilter<float>, filters::FilterBase<float>)
PLUGINLIB_DECLARE_CLASS(kelsey_sandbox, MultiChannelDelayFilterDouble, filters::MultiChannelDelayFilter<double>, filters::MultiChannelFilterBase<double>)
PLUGINLIB_DECLARE_CLASS(kelsey_sandbox, MultiChannelDelayFilterFloat, filters::MultiChannelDelayFilter<float>, filters::MultiChannelFilterBase<float>)
