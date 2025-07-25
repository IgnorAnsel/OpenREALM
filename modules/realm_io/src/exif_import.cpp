

#include <realm_io/exif_import.h>
#include <realm_core/timer.h>

#include <realm_core/loguru.h>

namespace realm
{

io::Exiv2FrameReader::Exiv2FrameReader(const FrameTags &tags)
: m_frame_tags(tags)
{
}

std::map<std::string, bool> io::Exiv2FrameReader::probeImage(const std::string &filepath)
{
  std::map<std::string, bool> tag_existence;
  tag_existence[m_frame_tags.camera_id] = false;
  tag_existence[m_frame_tags.timestamp] = false;
  tag_existence[m_frame_tags.latitude] = false;
  tag_existence[m_frame_tags.latituderef] = false;
  tag_existence[m_frame_tags.longitude] = false;
  tag_existence[m_frame_tags.longituderef] = false;
  tag_existence[m_frame_tags.altitude] = false;
  tag_existence[m_frame_tags.heading] = false;
  tag_existence[m_frame_tags.user_comment] = false;
  Exiv2ImagePointer exif_img = Exiv2::ImageFactory::open(filepath);
  if (exif_img.get())
  {
    exif_img->readMetadata();
    Exiv2::ExifData &exif_data = exif_img->exifData();
    Exiv2::XmpData &xmp_data = exif_img->xmpData();

    tag_existence[m_frame_tags.camera_id] = probeTag(m_frame_tags.camera_id, exif_data, xmp_data);
    tag_existence[m_frame_tags.timestamp] = probeTag(m_frame_tags.timestamp, exif_data, xmp_data);
    tag_existence[m_frame_tags.latitude] = probeTag(m_frame_tags.latitude, exif_data, xmp_data);
    tag_existence[m_frame_tags.latituderef] = probeTag(m_frame_tags.latituderef, exif_data, xmp_data);
    tag_existence[m_frame_tags.longitude] = probeTag(m_frame_tags.longitude, exif_data, xmp_data);
    tag_existence[m_frame_tags.longituderef] = probeTag(m_frame_tags.longituderef, exif_data, xmp_data);
    tag_existence[m_frame_tags.altitude] = probeTag(m_frame_tags.altitude, exif_data, xmp_data);
    tag_existence[m_frame_tags.heading] = probeTag(m_frame_tags.heading, exif_data, xmp_data);
    tag_existence[m_frame_tags.user_comment] = probeTag(m_frame_tags.user_comment, exif_data, xmp_data);
  }

  return tag_existence;
}
cv::Mat io::Exiv2FrameReader::eulerToRotationMatrix(Attitude attitude) {
    double yaw   = attitude.yaw;
    double pitch = attitude.pitch;
    double roll  = attitude.roll;
    // 将角度转换为弧度
    yaw   = yaw   * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    roll  = roll  * M_PI / 180.0;

    // 计算三角函数值
    double cy = cos(yaw);
    double sy = sin(yaw);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cr = cos(roll);
    double sr = sin(roll);

    // 构建旋转矩阵
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
        cy * cp,   cy * sp * sr - sy * cr,   cy * sp * cr + sy * sr,
        sy * cp,   sy * sp * sr + cy * cr,   sy * sp * cr - cy * sr,
        -sp,       cp * sr,                  cp * cr
    );

    return R;
}
Frame::Ptr io::Exiv2FrameReader::loadFrameFromExiv2(const std::string &camera_id, const camera::Pinhole::Ptr &cam, const std::string &filepath)
{
  Exiv2ImagePointer exif_img = Exiv2::ImageFactory::open(filepath);
  if (exif_img.get())
  {
    // Read exif and xmp metadata
    exif_img->readMetadata();
    Exiv2::ExifData &exif_data = exif_img->exifData();
    Exiv2::XmpData &xmp_data = exif_img->xmpData();

    // Read image data
    cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);

    /*========== ESSENTIAL KEYS ==========*/
    uint32_t frame_id = io::extractFrameIdFromFilepath(filepath);

    std::string camera_id_set;
    if (camera_id.empty())
    {
      if (!readMetaTagCameraId(exif_data, xmp_data, &camera_id_set))
        camera_id_set = "unknown_id";
    }
    Attitude attitude{0};
    if (!readAttitude(exif_data, &attitude)) {
        attitude.yaw = 0.0;
    }
    WGSPose wgs{0};
    if (!readMetaTagLatitude(exif_data, xmp_data, &wgs.latitude))
      wgs.latitude = 0.0;

    if (!readMetaTagLongitude(exif_data, xmp_data, &wgs.longitude))
      wgs.longitude = 0.0;
else 
    if (!readMetaTagAltitude(exif_data, xmp_data, &wgs.altitude))
      wgs.altitude = 0.0;

    if (!readMetaTagHeading(exif_data, xmp_data, &wgs.heading))
      wgs.heading = 0.0;

    UTMPose utm = gis::convertToUTM(wgs);

    //LOG_F(INFO, "WGS: %f %f", wgs.latitude, wgs.longitude);
    //LOG_F(INFO, "UTM: %d %c %f %f", utm.zone, utm.band, utm.easting, utm.northing);

    /*========== OPTIONAL KEYS ==========*/
    uint64_t timestamp_val;
    if (!readMetaTagTimestamp(exif_data, xmp_data, &timestamp_val))
      timestamp_val = Timer::getCurrentTimeMilliseconds();

    return std::make_shared<Frame>(camera_id, frame_id, timestamp_val, img, utm, cam, eulerToRotationMatrix(attitude));
    //return std::make_shared<Frame>(camera_id, frame_id, timestamp_val, img, utm, cam, computeOrientationFromHeading(utm.heading));
  }
  return nullptr;
}

bool io::Exiv2FrameReader::readAttitude(Exiv2::ExifData &exif_data,
                                      Attitude* attitude)
{
    for (auto& md : exif_data) {
        std::cout << md.key() << " " << md.toString() << std::endl;
        if (md.key().find("Exif.Photo.UserComment") != std::string::npos) {
            std::string comment = md.toString();
            size_t json_start = comment.find('{');
            if (json_start == std::string::npos) {
                std::cerr << "No JSON found in UserComment" << std::endl;
                continue;
            }
            
            try {
                std::string json_str = comment.substr(json_start);
                std::cout << "Pure JSON: " << json_str << std::endl;

                nlohmann::json json = nlohmann::json::parse(json_str);
                
                attitude->yaw = json.value("yaw", 0.0);
                attitude->pitch = json.value("pitch", 0.0);
                attitude->roll = json.value("roll", 0.0);
                return true;
            } 
            catch (const std::exception& e) {
                std::cerr << "JSON parse error: " << e.what() << std::endl;
                continue;
            }
        }
    }
    return false;
}
bool io::Exiv2FrameReader::readMetaTagCameraId(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, std::string* camera_id)
{
  if (isXmpTag(m_frame_tags.camera_id))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.camera_id)) != xmp_data.end())
    {
      *camera_id = xmp_data[m_frame_tags.camera_id].toString();
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(m_frame_tags.camera_id)) != exif_data.end())
    {
      *camera_id = exif_data[m_frame_tags.camera_id].toString();
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagTimestamp(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, uint64_t* timestamp)
{
  if (isXmpTag(m_frame_tags.timestamp))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.timestamp)) != xmp_data.end())
    {
      *timestamp = std::stoul(xmp_data[m_frame_tags.timestamp].toString());
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(m_frame_tags.timestamp)) != exif_data.end())
    {
      *timestamp = std::stoul(exif_data[m_frame_tags.timestamp].toString());
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagLatitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* latitude)
{
  double latitude_dms[3];
  if (isXmpTag(m_frame_tags.latitude))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.latitude)) != xmp_data.end() &&
        xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.latituderef)) != xmp_data.end())
    {
      latitude_dms[0] = xmp_data[m_frame_tags.latitude].toFloat(0);
      latitude_dms[1] = xmp_data[m_frame_tags.latitude].toFloat(1);
      latitude_dms[2] = xmp_data[m_frame_tags.latitude].toFloat(2);
      *latitude = cvtAngleDegMinSecToDecimal(latitude_dms);

      
      if (xmp_data[m_frame_tags.latituderef].toString(0) == "S")
      {
	      *latitude *= -1.0;
      }
     
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(m_frame_tags.latitude)) != exif_data.end() &&
        exif_data.findKey(Exiv2::ExifKey(m_frame_tags.latituderef)) != exif_data.end())
    {
      latitude_dms[0] = exif_data[m_frame_tags.latitude].toFloat(0);
      latitude_dms[1] = exif_data[m_frame_tags.latitude].toFloat(1);
      latitude_dms[2] = exif_data[m_frame_tags.latitude].toFloat(2);
      *latitude = cvtAngleDegMinSecToDecimal(latitude_dms);

      if (exif_data[m_frame_tags.latituderef].toString(0) == "S")
      {
	      *latitude *= -1.0;
      }
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagLongitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* longitude)
{
  double longitude_dms[3];
  if (isXmpTag(m_frame_tags.longitude))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.longitude)) != xmp_data.end() &&
        xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.longituderef)) != xmp_data.end())
    {
      longitude_dms[0] = xmp_data[m_frame_tags.longitude].toFloat(0);
      longitude_dms[1] = xmp_data[m_frame_tags.longitude].toFloat(1);
      longitude_dms[2] = xmp_data[m_frame_tags.longitude].toFloat(2);
      *longitude = cvtAngleDegMinSecToDecimal(longitude_dms);

      if (xmp_data[m_frame_tags.longituderef].toString(0) == "W")
      {
	      *longitude *= -1.0;
      }
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(m_frame_tags.longitude)) != exif_data.end() &&
        exif_data.findKey(Exiv2::ExifKey(m_frame_tags.longituderef)) != exif_data.end())
    {
      longitude_dms[0] = exif_data[m_frame_tags.longitude].toFloat(0);
      longitude_dms[1] = exif_data[m_frame_tags.longitude].toFloat(1);
      longitude_dms[2] = exif_data[m_frame_tags.longitude].toFloat(2);
      *longitude = cvtAngleDegMinSecToDecimal(longitude_dms);

      if (exif_data[m_frame_tags.longituderef].toString(0) == "W")
      {
	      *longitude *= -1.0;
      }
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagAltitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* altitude)
{
  if (isXmpTag(m_frame_tags.altitude))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.altitude)) != xmp_data.end())
    {
      *altitude = static_cast<double>(xmp_data[m_frame_tags.altitude].toFloat());
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(m_frame_tags.altitude)) != exif_data.end())
    {
      *altitude = static_cast<double>(exif_data[m_frame_tags.altitude].toFloat());
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagHeading(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* heading)
{
  if (isXmpTag(m_frame_tags.heading))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(m_frame_tags.heading)) != xmp_data.end())
    {
      *heading = static_cast<double>(xmp_data[m_frame_tags.heading].toFloat());
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(m_frame_tags.heading)) != exif_data.end())
    {
      *heading = static_cast<double>(exif_data[m_frame_tags.heading].toFloat());
      return true;
    }
  }
  return false;
}

double io::Exiv2FrameReader::cvtAngleDegMinSecToDecimal(const double* angle)
{
  double angle_deg = angle[0];
  double angle_min = angle[1]/60;
  double angle_sec = angle[2]/3600;
  return angle_deg + angle_min + angle_sec;
}

bool io::Exiv2FrameReader::isXmpTag(const std::string &tag)
{
  std::vector<std::string> tokens = io::split(tag.c_str(), '.');
  return tokens[0] == "Xmp";
}

bool io::Exiv2FrameReader::probeTag(const std::string &tag, Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data)
{
  if (tag == m_frame_tags.user_comment) {
        for (auto& md : exif_data) {
            if (md.key().find("UserComment") != std::string::npos) {
                return true;
            }
        }
        return false;
  }
  if (isXmpTag(tag))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(tag)) != xmp_data.end())
    {
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(tag)) != exif_data.end())
    {
      return true;
    }
  }
  return false;
}

}
