/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef IGNITION_RENDERING_OPTIX_OPTIXCAMERA_HH_
#define IGNITION_RENDERING_OPTIX_OPTIXCAMERA_HH_

#include <string>
#include "ignition/rendering/base/BaseCamera.hh"
#include "ignition/rendering/optix/OptixRenderTypes.hh"
#include "ignition/rendering/optix/OptixSensor.hh"

namespace ignition
{
  namespace rendering
  {
    class IGNITION_VISIBLE OptixCamera :
      public BaseCamera<OptixSensor>
    {
      protected: OptixCamera();

      public: virtual ~OptixCamera();

      public: virtual PixelFormat ImageFormat() const;

      public: virtual void SetImageFormat(PixelFormat _format);

      public: virtual math::Angle HFOV() const;

      public: virtual void SetHFOV(const math::Angle &_angle);

      public: virtual double AspectRatio() const;

      public: virtual void SetAspectRatio(double _ratio);

      public: virtual unsigned int AntiAliasing() const;

      public: virtual void SetAntiAliasing(unsigned int _aa);

      public: virtual void PreRender();

      public: virtual void Render();

      protected: virtual RenderTexturePtr RenderTexture() const;

      protected: virtual void WriteCameraToDevice();

      protected: virtual void WriteCameraToDeviceImpl();

      protected: virtual void WritePoseToDeviceImpl();

      protected: virtual void Init();

      protected: virtual void CreateRenderTexture();

      protected: virtual void CreateRenderProgram();

      protected: virtual void CreateClearProgram();

      protected: virtual void CreateErrorProgram();

      protected: optix::Program optixRenderProgram;

      protected: optix::Program optixClearProgram;

      protected: optix::Program optixErrorProgram;

      protected: OptixRenderTexturePtr renderTexture;

      protected: unsigned int imageWidth;

      protected: unsigned int imageHeight;

      protected: math::Angle xFieldOfView;

      protected: unsigned int aspectRatio;

      protected: unsigned int antiAliasing;

      protected: bool cameraDirty;

      protected: unsigned int traceId;

      protected: unsigned int clearId;

      private: static const std::string PTX_BASE_NAME;

      private: static const std::string PTX_RENDER_FUNCTION;

      private: static const std::string PTX_CLEAR_FUNCTION;

      private: friend class OptixScene;
    };
  }
}
#endif