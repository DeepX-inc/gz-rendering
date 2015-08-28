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
#include "ignition/rendering/optix/OptixMaterial.hh"
#include "ignition/rendering/optix/OptixConversions.hh"
#include "ignition/rendering/optix/OptixRayTypes.hh"
#include "ignition/rendering/optix/OptixRenderEngine.hh"
#include "ignition/rendering/optix/OptixScene.hh"
#include "ignition/rendering/optix/OptixTextureFactory.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////

const std::string OptixMaterial::PTX_FILE_BASE("OptixMaterial");

const std::string OptixMaterial::PTX_CLOSEST_HIT_FUNC("ClosestHit");

const std::string OptixMaterial::PTX_ANY_HIT_FUNC("AnyHit");

//////////////////////////////////////////////////  
OptixMaterial::OptixMaterial() :
  colorDirty(true),
  textureDirty(true),
  normalMapDirty(true),
  optixMaterial(NULL),
  optixTexture(NULL),
  optixNormalMap(NULL),
  optixEmptyTexture(NULL)
{
}

//////////////////////////////////////////////////
OptixMaterial::~OptixMaterial()
{
}

//////////////////////////////////////////////////
bool OptixMaterial::GetLightingEnabled() const
{
  return this->lightingEnabled;
}

//////////////////////////////////////////////////
void OptixMaterial::SetLightingEnabled(bool _enabled)
{
  this->lightingEnabled = _enabled;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
gazebo::common::Color OptixMaterial::GetAmbient() const
{
  return this->ambient;
}

//////////////////////////////////////////////////
void OptixMaterial::SetAmbient(const gazebo::common::Color &_color)
{
  this->ambient = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
gazebo::common::Color OptixMaterial::GetDiffuse() const
{
  return this->diffuse;
}

//////////////////////////////////////////////////
void OptixMaterial::SetDiffuse(const gazebo::common::Color &_color)
{
  this->diffuse = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
gazebo::common::Color OptixMaterial::GetSpecular() const
{
  return this->specular;
}

//////////////////////////////////////////////////
void OptixMaterial::SetSpecular(const gazebo::common::Color &_color)
{
  this->specular = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
gazebo::common::Color OptixMaterial::GetEmissive() const
{
  return this->emissive;
}

//////////////////////////////////////////////////
void OptixMaterial::SetEmissive(const gazebo::common::Color &_color)
{
  this->emissive = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
double OptixMaterial::GetShininess() const
{
  return this->shininess;
}

//////////////////////////////////////////////////
void OptixMaterial::SetShininess(double _shininess)
{
  this->shininess = _shininess;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
double OptixMaterial::GetTransparency() const
{
  return this->transparency;
}

//////////////////////////////////////////////////
void OptixMaterial::SetTransparency(double _transparency)
{
  this->transparency = std::min(std::max(_transparency, 0.0), 1.0);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
double OptixMaterial::GetReflectivity() const
{
  return this->reflectivity;
}

//////////////////////////////////////////////////
void OptixMaterial::SetReflectivity(double _reflectivity)
{
  this->reflectivity = std::min(std::max(_reflectivity, 0.0), 1.0);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
bool OptixMaterial::GetReceiveShadows() const
{
  return this->receiveShadows;
}

//////////////////////////////////////////////////
void OptixMaterial::SetReceiveShadows(bool _receiveShadows)
{
  this->receiveShadows = _receiveShadows;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
bool OptixMaterial::HasTexture() const
{
  return !this->textureName.empty();
}

//////////////////////////////////////////////////
std::string OptixMaterial::GetTexture() const
{
  return this->textureName;
}

//////////////////////////////////////////////////
void OptixMaterial::SetTexture(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearTexture();
  }
  else if (_name != this->textureName)
  {
    this->textureName = _name;
    this->textureDirty = true;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::ClearTexture()
{
  if (this->HasTexture())
  {
    this->textureName = "";
    this->textureDirty = true;
  }
}

//////////////////////////////////////////////////
bool OptixMaterial::HasNormalMap() const
{
  return !this->textureName.empty();
}

//////////////////////////////////////////////////
std::string OptixMaterial::GetNormalMap() const
{
  return this->normalMapName;
}

//////////////////////////////////////////////////
void OptixMaterial::SetNormalMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearNormalMap();
  }
  else if (_name != this->normalMapName)
  {
    this->normalMapName = _name;
    this->normalMapDirty = true;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::ClearNormalMap()
{
  if (this->HasNormalMap())
  {
    this->normalMapName = "";
    this->normalMapDirty = true;
  }
}

//////////////////////////////////////////////////
ShaderType OptixMaterial::GetShaderType() const
{
  return this->shaderType;
}

//////////////////////////////////////////////////
void OptixMaterial::SetShaderType(ShaderType _type)
{
  this->shaderType = (ShaderUtil::IsValid(_type)) ? _type : ST_PIXEL;
  // TODO: update normal space
}

//////////////////////////////////////////////////
void OptixMaterial::PreRender()
{
  this->WriteColorToDevice();
  this->WriteTextureToDevice();
  this->WriteNormalMapToDevice();
}

//////////////////////////////////////////////////
void OptixMaterial::Destroy()
{
  if (this->optixTexture)
  {
    this->optixTexture->destroy();
    this->optixTexture = 0;
  }

  if (this->optixNormalMap)
  {
    this->optixNormalMap->destroy();
    this->optixNormalMap = 0;
  }

  if (this->optixEmptyTexture)
  {
    this->optixEmptyTexture->destroy();
    this->optixEmptyTexture = 0;
  }

  if (this->optixMaterial)
  {
    this->optixMaterial->destroy();
    this->optixMaterial = 0;
  }

  BaseMaterial::Destroy();
}

//////////////////////////////////////////////////
optix::Material OptixMaterial::GetOptixMaterial() const
{
  return this->optixMaterial;
}

//////////////////////////////////////////////////
void OptixMaterial::WriteColorToDevice()
{
  if (this->colorDirty)
  {
    this->WriteColorToDeviceImpl();
    this->colorDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteTextureToDevice()
{
  if (this->textureDirty)
  {
    this->WriteTextureToDeviceImpl();
    this->textureDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteNormalMapToDevice()
{
  if (this->normalMapDirty)
  {
    this->WriteNormalMapToDeviceImpl();
    this->normalMapDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteColorToDeviceImpl()
{
  // TODO: clean up
  this->optixMaterial["diffuse"]->setFloat(this->diffuse.r, this->diffuse.g, this->diffuse.b);
  this->optixMaterial["ambient"]->setFloat(this->ambient.r, this->ambient.g, this->ambient.b);
  this->optixMaterial["specular"]->setFloat(this->specular.r, this->specular.g, this->specular.b);
  this->optixMaterial["emissive"]->setFloat(this->emissive.r, this->emissive.g, this->emissive.b);
  this->optixMaterial["reflectivity"]->setFloat(this->reflectivity);
  this->optixMaterial["transparency"]->setFloat(this->transparency);
}

//////////////////////////////////////////////////
void OptixMaterial::WriteTextureToDeviceImpl()
{
  if (this->optixTexture)
  {
    this->optixTexture->destroy();
    this->optixTexture = 0;
  }

  if (this->textureName.empty())
  {
    this->optixMaterial["texSampler"]->setTextureSampler(
        this->optixEmptyTexture);
  }
  else
  {
    OptixTextureFactory texFactory(this->scene);
    this->optixTexture = texFactory.Create(this->textureName);
    this->optixMaterial["texSampler"]->setTextureSampler(this->optixTexture);
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteNormalMapToDeviceImpl()
{
  if (this->optixNormalMap)
  {
    this->optixNormalMap->destroy();
    this->optixNormalMap = 0;
  }

  if (this->normalMapName.empty())
  {
    this->optixMaterial["normSampler"]->setTextureSampler(
        this->optixEmptyTexture);
  }
  else
  {
    OptixTextureFactory texFactory(this->scene);
    this->optixNormalMap = texFactory.Create(this->normalMapName);
    this->optixMaterial["normSampler"]->setTextureSampler(this->optixNormalMap);
  }
}

//////////////////////////////////////////////////
void OptixMaterial::Init()
{
  BaseMaterial::Init();
  optix::Context optixContext = this->scene->GetOptixContext();

  optix::Program closestHitProgram =
      this->scene->CreateOptixProgram(PTX_FILE_BASE, PTX_CLOSEST_HIT_FUNC);

  optix::Program anyHitProgram =
      this->scene->CreateOptixProgram(PTX_FILE_BASE, PTX_ANY_HIT_FUNC);

  this->optixMaterial = optixContext->createMaterial();
  optixMaterial->setClosestHitProgram(RT_RADIANCE, closestHitProgram);
  optixMaterial->setAnyHitProgram(RT_SHADOW, anyHitProgram);

  OptixTextureFactory texFactory(this->scene);
  this->optixEmptyTexture = texFactory.Create();
  this->optixMaterial["texSampler"]->setTextureSampler(this->optixEmptyTexture);

  this->Reset();
}
