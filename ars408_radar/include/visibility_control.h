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

#ifndef ARS408_RADAR__VISIBILITY_CONTROL_H_
#define ARS408_RADAR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARS408_RADAR_EXPORT __attribute__ ((dllexport))
    #define ARS408_RADAR_IMPORT __attribute__ ((dllimport))
  #else
    #define ARS408_RADAR_EXPORT __declspec(dllexport)
    #define ARS408_RADAR_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARS408_RADAR_BUILDING_DLL
    #define ARS408_RADAR_PUBLIC ARS408_RADAR_EXPORT
  #else
    #define ARS408_RADAR_PUBLIC ARS408_RADAR_IMPORT
  #endif
  #define ARS408_RADAR_PUBLIC_TYPE ARS408_RADAR_PUBLIC
  #define ARS408_RADAR_LOCAL
#else
  #define ARS408_RADAR __attribute__ ((visibility("default")))
  #define ARS408_RADAR
  #if __GNUC__ >= 4
    #define ARS408_RADAR_PUBLIC __attribute__ ((visibility("default")))
    #define ARS408_RADAR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARS408_RADAR_PUBLIC
    #define ARS408_RADAR_LOCAL
  #endif
  #define ARS408_RADAR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ARS408_RADAR__VISIBILITY_CONTROL_H_
