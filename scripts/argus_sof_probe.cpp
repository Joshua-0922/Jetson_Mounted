// Argus probe: sensor start-of-frame (SOF) -> frame delivery latency, plus exposure / readout time.
// Used to build the perception-delay model (README, "카메라 → YOLO 인식 지연"). The camera must be free
// (stop imx219_camera first). Run outdoors to get flight-condition exposure.
//
//   g++ -O2 -std=c++17 scripts/argus_sof_probe.cpp -I/usr/src/jetson_multimedia_api/argus/include \
//       -I/usr/src/jetson_multimedia_api/include -L/usr/lib/aarch64-linux-gnu/nvidia \
//       -lnvargus_socketclient -lEGL -o /tmp/argus_sof_probe
//   /tmp/argus_sof_probe [sensor_mode=2] [frames=300]
//
// Argus getSensorTimestamp() is in CLOCK_MONOTONIC (verified: CLOCK_MONOTONIC_RAW is off by a constant).
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/ArgusCaptureMetadata.h>
#include <cstdio>
#include <ctime>
#include <vector>
#include <algorithm>
using namespace Argus;
static uint64_t mono(clockid_t c){timespec t;clock_gettime(c,&t);return t.tv_sec*1000000000ull+t.tv_nsec;}
int main(int argc,char**argv){
  int mode = argc>1?atoi(argv[1]):2, N = argc>2?atoi(argv[2]):300;
  UniqueObj<CameraProvider> prov(CameraProvider::create());
  auto *ip=interface_cast<ICameraProvider>(prov); if(!ip){puts("no provider");return 1;}
  std::vector<CameraDevice*> devs; ip->getCameraDevices(&devs); if(devs.empty()){puts("no cam");return 1;}
  auto *icp=interface_cast<ICameraProperties>(devs[0]); std::vector<SensorMode*> modes; icp->getAllSensorModes(&modes);
  auto *ism=interface_cast<ISensorMode>(modes[mode]);
  printf("sensor mode %d: %ux%u\n",mode,ism->getResolution().width(),ism->getResolution().height());
  UniqueObj<CaptureSession> ses(ip->createCaptureSession(devs[0])); auto *is=interface_cast<ICaptureSession>(ses);
  if(!is){puts("session failed (camera busy?)");return 1;}
  UniqueObj<OutputStreamSettings> ss(is->createOutputStreamSettings(STREAM_TYPE_EGL));
  auto *iss=interface_cast<IEGLOutputStreamSettings>(ss);
  iss->setPixelFormat(PIXEL_FMT_YCbCr_420_888); iss->setResolution(Size2D<uint32_t>(1280,720)); iss->setMetadataEnable(true);
  UniqueObj<OutputStream> st(is->createOutputStream(ss.get()));
  UniqueObj<EGLStream::FrameConsumer> cons(EGLStream::FrameConsumer::create(st.get()));
  auto *ic=interface_cast<EGLStream::IFrameConsumer>(cons);
  UniqueObj<Request> req(is->createRequest()); auto *ir=interface_cast<IRequest>(req);
  ir->enableOutputStream(st.get());
  auto *isrc=interface_cast<ISourceSettings>(ir->getSourceSettings()); isrc->setSensorMode(modes[mode]);
  isrc->setFrameDurationRange(Range<uint64_t>(33333333));
  is->repeat(req.get());
  std::vector<double> lat_mono, lat_raw; double exp_ms=0, ro_ms=0, fd_ms=0;
  for(int i=0;i<N;i++){
    Status s; UniqueObj<EGLStream::Frame> fr(ic->acquireFrame(2000000000ull,&s));
    uint64_t tm=mono(CLOCK_MONOTONIC), tr=mono(CLOCK_MONOTONIC_RAW);
    auto *am=interface_cast<EGLStream::IArgusCaptureMetadata>(fr); if(!am) continue;
    auto *md=interface_cast<ICaptureMetadata>(am->getMetadata()); if(!md) continue;
    uint64_t sof=md->getSensorTimestamp();
    if(i<30) continue; // AE settle
    lat_mono.push_back((double)((int64_t)(tm-sof))/1e6); lat_raw.push_back((double)((int64_t)(tr-sof))/1e6);
    exp_ms=md->getSensorExposureTime()/1e6; ro_ms=md->getFrameReadoutTime()/1e6; fd_ms=md->getFrameDuration()/1e6;
    if(i==30) printf("first: sof=%llu mono=%llu raw=%llu\n",(unsigned long long)sof,(unsigned long long)tm,(unsigned long long)tr);
  }
  is->stopRepeat(); is->waitForIdle();
  auto pr=[](const char*n,std::vector<double>v){std::sort(v.begin(),v.end());size_t k=v.size();
    printf("%s n=%zu min %.2f p50 %.2f p95 %.2f p99 %.2f max %.2f ms\n",n,k,v[0],v[k/2],v[k*95/100],v[k*99/100],v[k-1]);};
  pr("acquire - SOF (MONOTONIC)",lat_mono); pr("acquire - SOF (MONOTONIC_RAW)",lat_raw);
  printf("last frame: exposure %.2f ms, readout %.2f ms, frame duration %.2f ms\n",exp_ms,ro_ms,fd_ms);
  return 0;
}
