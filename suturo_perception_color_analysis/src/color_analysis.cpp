#include "suturo_perception_color_analysis/color_analysis.h"

using namespace suturo_perception;

ColorAnalysis::ColorAnalysis(PipelineData::Ptr data, PipelineObject::Ptr obj) : Capability(data, obj)
{
  logger = Logger("color_analysis");
  s_lower_threshold = 0.2;
  s_upper_threshold = 0.8;
  v_lower_threshold = 0.2;
  v_upper_threshold = 0.8;
}

HSVColor
ColorAnalysis::getAverageColorHSV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  HSVColor fail_ret;
  fail_ret.h = -1;
  fail_ret.s = -1.0;
  fail_ret.v = -1.0;

  if(cloud_in->points.size() == 0) return fail_ret;

  HSVColor avg_col;
  avg_col.h = 0;
  double avg_col_h = 0.0;
  avg_col.s = 0.0;
  avg_col.v = 0.0;
  for(int i = 0; i < cloud_in->points.size(); ++i)
  {
    uint32_t rgb = *reinterpret_cast<int*>(&cloud_in->points[i].rgb);
    HSVColor hsv_col = convertRGBToHSV(rgb);

    if (!inHSVThreshold(hsv_col))
      continue;

    avg_col_h += hsv_col.h / (double)cloud_in->points.size();
    avg_col.s += hsv_col.s / (double)cloud_in->points.size();
    avg_col.v += hsv_col.v / (double)cloud_in->points.size();
  }

  avg_col.h = (uint32_t) avg_col_h;

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "getAverageColorHSVQuality()");

  return avg_col;
}

/*
 * Convert rgb color to hsv color
 * colors are in packed value format. the msb is used as debug flag, everthing != 0
 * turns on debugging
 * based on: http://en.literateprograms.org/RGB_to_HSV_color_space_conversion_%28C%29
 */
HSVColor
ColorAnalysis::convertRGBToHSV(uint32_t rgb) 
{
  HSVColor hsv;
  double r = ((uint8_t) ((rgb & 0xff0000) >> 16)) / 255.0;
  double g = ((uint8_t) ((rgb & 0xff00) >> 8)) / 255.0;
  double b = ((uint8_t) (rgb & 0xff)) / 255.0;
  //logger.logWarn((boost::format("convertRGBToHSV input: %f, %f, %f") % r % g % b).str());
  double rgb_min, rgb_max;
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(1) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  hsv.v = rgb_max;
  if (hsv.v == 0) {
      hsv.h = hsv.s = 0;
      return hsv;
  }
  /* Normalize value to 1 */
  r /= hsv.v;
  g /= hsv.v;
  b /= hsv.v;
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(2) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  hsv.s = rgb_max - rgb_min;
  if (hsv.s == 0) {
      hsv.h = 0;
      return hsv;
  }
  /* Normalize saturation to 1 */
  r = (r - rgb_min)/(rgb_max - rgb_min);
  g = (g - rgb_min)/(rgb_max - rgb_min);
  b = (b - rgb_min)/(rgb_max - rgb_min);
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(3) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  /* Compute hue */
  if (rgb_max == r) {
      int h_tmp = 0.0 + 60.0*(g - b);
      //logger.logWarn((boost::format("convertRGBToHSV hue: %f, %f, %f") % hsv.h % g % b).str());
      if (h_tmp < 0.0) {
          h_tmp += 360.0;
      }
      hsv.h = h_tmp;
      //logger.logWarn((boost::format("convertRGBToHSV hue2: %f") % hsv.h).str());
  } else if (rgb_max == g) {
      hsv.h = 120.0 + 60.0*(b - r);
      //logger.logWarn((boost::format("convertRGBToHSV hue3: %f") % hsv.h).str());
  } else /* rgb_max == b */ {
      hsv.h = 240.0 + 60.0*(r - g);
      //logger.logWarn((boost::format("convertRGBToHSV hue4: %f") % hsv.h).str());
  }
  return hsv;
}

/*
 * convert hsv to rgb color
 * taken from: http://stackoverflow.com/a/6930407
 */
uint32_t
ColorAnalysis::convertHSVToRGB(HSVColor hsv)
{
  double hh, p, q, t, ff;
  long i;
  uint8_t r,g,b;

  uint32_t h = hsv.h;
  double s = hsv.s;
  double v = hsv.v;

  if(s <= 0.0)
  {
    r = (uint8_t) v * 255;
    g = (uint8_t) v * 255;
    b = (uint8_t) v * 255;
    return (r << 16) | (g << 8) | b;
  }
  hh = h;
  if(hh >= 360.0) 
    hh = 0.0;
  hh /= 60;
  i = (long)hh;
  ff = hh - i;
  p = v * (1 - s);
  q = v * (1 - (s * ff));
  t = v * (1 - (s * (1 - ff)));

  switch(i) 
  {
  case 0:
    r = (uint8_t) (v * 255);
    g = (uint8_t) (t * 255);
    b = (uint8_t) (p * 255);
    break;
  case 1:
    r = (uint8_t) (q * 255);
    g = (uint8_t) (v * 255);
    b = (uint8_t) (p * 255);
    break;
  case 2:
    r = (uint8_t) (p * 255);
    g = (uint8_t) (v * 255);
    b = (uint8_t) (t * 255);
    break;
  case 3:
    r = (uint8_t) (p * 255);
    g = (uint8_t) (q * 255);
    b = (uint8_t) (v * 255);
    break;
  case 4:
    r = (uint8_t) (t * 255);
    g = (uint8_t) (p * 255);
    b = (uint8_t) (v * 255);
    break;
  case 5:
  default:
    r = (uint8_t) (v * 255);
    g = (uint8_t) (p * 255);
    b = (uint8_t) (q * 255);
    break;
  }
  return (r << 16) | (g << 8) | b;
}

void
ColorAnalysis::execute()
{
  // Get average color of the object
  HSVColor averageColorHSVQuality = getAverageColorHSV(pipelineObject_->get_pointCloud());

  /*
  // update perceived object
  perceivedObject.set_c_color_average_r((averageColor >> 16) & 0x0000ff);
  perceivedObject.set_c_color_average_g((averageColor >> 8)  & 0x0000ff);
  perceivedObject.set_c_color_average_b((averageColor)       & 0x0000ff);
  perceivedObject.set_c_color_average_h(averageColorHSV.h);
  perceivedObject.set_c_color_average_s(averageColorHSV.s);
  perceivedObject.set_c_color_average_v(averageColorHSV.v);
  perceivedObject.set_c_color_average_qh(averageColorHSVQuality.h);
  perceivedObject.set_c_color_average_qs(averageColorHSVQuality.s);
  perceivedObject.set_c_color_average_qv(averageColorHSVQuality.v);
  perceivedObject.set_c_hue_histogram(hueHistogram);
  perceivedObject.set_c_hue_histogram_quality(histogramQuality);
  perceivedObject.set_c_hue_histogram_image(histogram_image);
  */
}

bool
ColorAnalysis::inHSVThreshold(HSVColor col)
{
  if (col.s >= s_lower_threshold && 
      col.s <= s_upper_threshold &&
      col.v >= v_lower_threshold &&
      col.v <= v_lower_threshold)
  {
    return true;
  }
  return false;
}
