// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__VISIBILITY_CONTROL_H_
#define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_EXPORT __attribute__ ((dllexport))
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_IMPORT __attribute__ ((dllimport))
  #else
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_EXPORT __declspec(dllexport)
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_IMPORT __declspec(dllimport)
  #endif
  #ifdef KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_BUILDING_DLL
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_EXPORT
  #else
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_IMPORT
  #endif
  #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC_TYPE KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC
  #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_LOCAL
#else
  #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_EXPORT __attribute__ ((visibility("default")))
  #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_IMPORT
  #if __GNUC__ >= 4
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC __attribute__ ((visibility("default")))
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC
    #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_LOCAL
  #endif
  #define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__VISIBILITY_CONTROL_H_