#include "ArgusCamera.hpp"

#include "EGLStream/ArgusCaptureMetadata.h"
// #include "ArgusHelpers.h"
// #include "CommonOptions.h"
// #include "Argus/Error.h"
// #include "Thread.h"

#define ROUND_UP_EVEN(x) 2 * ((x + 1) / 2)

using namespace Argus;
using namespace std;

uint32_t ArgusCameraConfig::getNumChannels()
{
  return 4; // RGBA
}

size_t ArgusCameraConfig::getOutputSizeBytes()
{
  return sizeof(uint8_t) * getNumChannels() * mVideoConverterResolution[0] * mVideoConverterResolution[1];
}

ArgusCamera *ArgusCamera::createArgusCamera(const ArgusCameraConfig &config, int *info)
{
  Argus::Status status;

  std::unique_ptr<ArgusCamera> camera;
  camera.reset(new ArgusCamera());

  camera->mConfig = config;

  // setup video converter
  camera->mVideoConverter = NvVideoConverter::createVideoConverter("videoConverter");
  if (!camera->mVideoConverter) {
    if (info) {
      *info = 1; // failed to create video converter
    }
    return nullptr;
  }

  // get camera provider
  auto cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create(NULL));
  auto iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
  if (!iCameraProvider) {
    if (info) {
      *info = 2; // failed to get camera provider
    }
    return nullptr;
  }

  // get camera device
  status = iCameraProvider->getCameraDevices(&camera->devices);
  if (Argus::STATUS_OK != status) {
    if (info) {
      *info = 3; // failed to get list of camera devices
    }
    return nullptr;
  }
  std::cout << "Number of cameras: " << camera->devices.size() << std::endl;

  if (camera->mConfig.mDeviceId >= camera->devices.size()) {
    if (info) {
      *info = 4; // camera device out of range
    }
    return nullptr;
  }

  // create capture session
  camera->mCaptureSession = UniqueObj<CaptureSession>(iCameraProvider->createCaptureSession(camera->devices));
  auto iCaptureSession = interface_cast<ICaptureSession>(camera->mCaptureSession);
  if (!iCaptureSession) {
    if (info) {
      *info = 5; // failed to create capture session
    }
    return nullptr;
  }

  // create stream settings
  auto streamSettings = UniqueObj<OutputStreamSettings>(iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
  auto iEGLOutputStreamSettings = interface_cast<IEGLOutputStreamSettings>(streamSettings);
  if (!iEGLOutputStreamSettings) {
    if (info) {
      *info = 6; // failed to create stream settings
    }
    return nullptr;
  }
  std::cout << "**********************" << std::endl;

  // set stream pixel format and resolution
  iEGLOutputStreamSettings->setPixelFormat(Argus::PIXEL_FMT_YCbCr_420_888);
  auto evenResolution = Argus::Size2D<uint32_t>(
    ROUND_UP_EVEN(camera->mConfig.mStreamResolution[WIDTH_IDX]),
    ROUND_UP_EVEN(camera->mConfig.mStreamResolution[HEIGHT_IDX])
  );
  iEGLOutputStreamSettings->setResolution(evenResolution);
  iEGLOutputStreamSettings->setMetadataEnable(true);

  Argus::IOutputStreamSettings *iOutputStreamSettings = Argus::interface_cast<Argus::IOutputStreamSettings>(streamSettings);
  for(int i = 0; i < camera->devices.size();i++) {
    iOutputStreamSettings->setCameraDevice(camera->devices[i]);
    // create stream
    Argus::UniqueObj<Argus::OutputStream> *mStream = 
      new UniqueObj<OutputStream>(iCaptureSession->createOutputStream(streamSettings.get(), &status));
    camera->mStreams.push_back(mStream);

    // create frame consumer
    Argus::UniqueObj<EGLStream::FrameConsumer> *mFrameConsumer = 
      new UniqueObj<EGLStream::FrameConsumer>(EGLStream::FrameConsumer::create(camera->mStreams[i]->get()));
    camera->mFrameConsumers.push_back(mFrameConsumer);
    auto iFrameConsumer = interface_cast<EGLStream::IFrameConsumer>(*mFrameConsumer);
    if (!iFrameConsumer) {
      if (info) {
        *info = 8; // failed to create frame consumer
      }
      return nullptr;
    }
  }
  std::cout << "**********************" << std::endl;

  // start repeating capture request
  auto request = UniqueObj<Request>(iCaptureSession->createRequest());
  auto iRequest = interface_cast<IRequest>(request);
  if (!iRequest) {
    if (info) {
      *info = 9; // failed to create request
    }
    return nullptr;
  }

  for(int i = 0; i < camera->devices.size();i++) {
    // enable output stream
    iRequest->enableOutputStream(camera->mStreams[i]->get());
  }

  // configure source settings in request
  // 1. set sensor mode
  auto iCameraProperties = interface_cast<ICameraProperties>(camera->devices[0]);
  vector<SensorMode*> sensorModes;
  status = iCameraProperties->getAllSensorModes(&sensorModes);
  if (Argus::STATUS_OK != status ||
      camera->mConfig.getSensorMode() >= sensorModes.size()) {
    if (info) {
      *info = 15;
    }
    return nullptr;
  }
  auto iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
  status = iSourceSettings->setSensorMode(sensorModes[camera->mConfig.getSensorMode()]);
  if (Argus::STATUS_OK != status) {
    if (info) {
      *info = 18;
    }
    return nullptr;
  }

  // 2. set frame duration
  status = iSourceSettings->setFrameDurationRange(Argus::Range<uint64_t>(
    camera->mConfig.getFrameDurationRange()[0],
    camera->mConfig.getFrameDurationRange()[1]
  ));
  if (Argus::STATUS_OK != status) {
    if (info) {
      *info = 19;
    }
    return nullptr;
  }
  std::cout << "**********************" << std::endl;
  
  // // configure stream settings
  // auto iStreamSettings = interface_cast<IStreamSettings>(iRequest->getStreamSettings(camera->mStream.get()));
  // if (!iStreamSettings) {
  //   if (info) {
  //     *info = 16;
  //   }
  //   return nullptr;
  // }
  // // set stream resolution
  // status = iStreamSettings->setSourceClipRect(Argus::Rectangle<float>(
  //   camera->mConfig.getSourceClipRect()[0],
  //   camera->mConfig.getSourceClipRect()[1],
  //   camera->mConfig.getSourceClipRect()[2],
  //   camera->mConfig.getSourceClipRect()[3]
  // ));
  // if (Argus::STATUS_OK != status) {
  //   if (info) {
  //     *info = 17;
  //   }
  //   return nullptr;
  // }

  // start repeating capture request
  status = iCaptureSession->repeat(request.get());
  if (Argus::STATUS_OK != status) {
    if (info) {
      *info = 10; // failed to start repeating capture request
    }
    return nullptr;
  }

  std::cout << "**********************" << std::endl;
  for(int i = 0; i < camera->devices.size();i++) {
    // connect stream
    auto iStream = interface_cast<IEGLOutputStream>(*camera->mStreams[i]);
    if (!iStream) {
      if (info) {
        *info = 7; // failed to create stream
      }
      return nullptr;
    }
    iStream->waitUntilConnected();
  }
  std::cout << "**********************" << std::endl;
  // setup video converter
  uint32_t width = camera->mConfig.mVideoConverterResolution[WIDTH_IDX];
  uint32_t height = camera->mConfig.mVideoConverterResolution[HEIGHT_IDX];
  camera->mVideoConverter->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, width, height, V4L2_NV_BUFFER_LAYOUT_PITCH);
  camera->mVideoConverter->setCapturePlaneFormat(V4L2_PIX_FMT_ABGR32, width, height, V4L2_NV_BUFFER_LAYOUT_PITCH);

  if (camera->mVideoConverter->output_plane.setupPlane(V4L2_MEMORY_DMABUF, 1, false, false) < 0) {
    if (info) {
      *info = 11;
    }
    return nullptr;
  }
  if (camera->mVideoConverter->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 1, true, false) < 0) {
    if (info) {
      *info = 12;
    }
    return nullptr;
  }
  if (camera->mVideoConverter->output_plane.setStreamStatus(true) < 0) {
    if (info) {
      *info = 13;
    }
    return nullptr;
  }
  if (camera->mVideoConverter->capture_plane.setStreamStatus(true) < 0) {
    if (info) {
      *info = 14;
    }
    return nullptr;
  }

  if (info) {
    *info = 0;
  }
  std::cout << "**********************" << std::endl;
  return camera.release();
  std::cout << "**********************" << std::endl;
}

ArgusCamera::~ArgusCamera()
{
  auto iCaptureSession = interface_cast<ICaptureSession>(mCaptureSession);
  if (iCaptureSession) {
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();
  }

  for(int i = 0; i < devices.size();i++) {
    auto iStream = interface_cast<IEGLOutputStream>(*mStreams[i]);
    if (iStream) {
      iStream->disconnect();
    }
  }

  if (mVideoConverter) {
    mVideoConverter->capture_plane.deinitPlane();
    mVideoConverter->output_plane.deinitPlane();
    mVideoConverter->capture_plane.setStreamStatus(false);
    mVideoConverter->output_plane.setStreamStatus(false);
  }

  delete mVideoConverter;
}

int ArgusCamera::read(CameraInfo *camera_info)
{
  Argus::Status status;

  for(int i = 0; i < devices.size();i++) {
    std::cout << "device id: " << devices[i] << std::endl;
    auto iStream = interface_cast<IEGLOutputStream>(*mStreams[i]);
    if (!iStream) {
      return 1; // failed to create stream interface
    }

    auto iFrameConsumer = interface_cast<EGLStream::IFrameConsumer>(*mFrameConsumers[i]);
    if (!iFrameConsumer) {
      return 2; // failed to create frame consumer
    }

    auto frame = UniqueObj<EGLStream::Frame>(iFrameConsumer->acquireFrame());
    auto iFrame = interface_cast<EGLStream::IFrame>(frame);
    if (!iFrame) {
      return 3; // failed to get frame
    }

    EGLStream::IArgusCaptureMetadata *iArgusCaptureMetadata = interface_cast<EGLStream::IArgusCaptureMetadata>(frame);
    if (!iArgusCaptureMetadata)
        printf("Failed to get IArgusCaptureMetadata interface.");
    CaptureMetadata *metadata = iArgusCaptureMetadata->getMetadata();
    ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(metadata);
    if (!iMetadata)
        printf("Failed to get ICaptureMetadata interface.");
    printf("\tSensor Timestamp: %llu, LUX: %f\n",
                   static_cast<unsigned long long>(iMetadata->getSensorTimestamp()),
                   iMetadata->getSceneLux());

    auto image = iFrame->getImage();
    auto frame_id = iFrame->getNumber();
    auto time_stamp = iFrame->getTime();
    std::cout << frame_id << ", " << time_stamp << std::endl;
    auto iImage = interface_cast<EGLStream::IImage>(image);
    auto iImage2D = interface_cast<EGLStream::IImage2D>(image);
    if (!iImage) {
      return 4;
    }
    if (!iImage2D) {
      return 5;
    }

    // copy to native buffer converting resolution, color format, and layout
    int fd = -1;
    auto iNativeBuffer = interface_cast<EGLStream::NV::IImageNativeBuffer>(image);
    auto resolution = Argus::Size2D<uint32_t>(
        mConfig.mVideoConverterResolution[WIDTH_IDX],
        mConfig.mVideoConverterResolution[HEIGHT_IDX]
    );
    fd = iNativeBuffer->createNvBuffer(resolution, 
      NvBufferColorFormat::NvBufferColorFormat_YUV420,
      NvBufferLayout_Pitch);
    if (fd == -1) {
      return 6; // failed to create native buffer
    }

    NvBufferParams nativeBufferParams;
    NvBufferGetParams(fd, &nativeBufferParams);

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[3];

    // enqueue capture plane
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, 3 * sizeof(struct v4l2_plane));
    v4l2_buf.index = 0;
    v4l2_buf.m.planes = planes;
    if (mVideoConverter->capture_plane.qBuffer(v4l2_buf, NULL) < 0) {
      return 7; // failed to queue capture plane
    }

    // enqueue output plane
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, 3 * sizeof(struct v4l2_plane));
    v4l2_buf.index = 0;
    v4l2_buf.m.planes = planes;
    planes[0].m.fd = fd;
    planes[0].bytesused = 1234;
    if (mVideoConverter->output_plane.qBuffer(v4l2_buf, NULL) < 0) {
      return 8; // failed to queue output plane
    }

    NvBuffer *nativeCaptureBuffer;
    if (mVideoConverter->capture_plane.dqBuffer(v4l2_buf, &nativeCaptureBuffer, NULL, 1) < 0) {
      return 9; // failed to deqeue capture plane
    }
    mVideoConverter->output_plane.dqBuffer(v4l2_buf, NULL, NULL, 1);

    // copy to destination (pitched -> block linear)
    auto plane = nativeCaptureBuffer->planes[0];
    uint32_t rowBytes = plane.fmt.bytesperpixel * plane.fmt.width;
    uint8_t *srcData = plane.data;
    
    if(i == 0) {
      for (uint32_t i = 0; i < plane.fmt.height; i++) {
        memcpy(camera_info->left_data, srcData, rowBytes);
        camera_info->left_data += rowBytes;
        srcData += plane.fmt.stride;
      }
      camera_info->left_data -= 640*360*4;
    }
    else if(i == 1) {
      for (uint32_t i = 0; i < plane.fmt.height; i++) {
        memcpy(camera_info->right_data, srcData, rowBytes);
        camera_info->right_data += rowBytes;
        srcData += plane.fmt.stride;
      }
      camera_info->right_data -= 640*360*4;
    }

    camera_info->frame_id = iFrame->getNumber();
    camera_info->time_stamp = iFrame->getTime();

    NvBufferDestroy(fd);
  }

  return 0;
}

IArgusCamera * IArgusCamera::createArgusCamera(const ArgusCameraConfig &config, int *info)
{
  return (IArgusCamera*) ArgusCamera::createArgusCamera(config, info);
}
