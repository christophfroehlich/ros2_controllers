// Copyright 2023 AIT Austrian Institute of Technology
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

#ifndef TRAJECTORY_PLUGINS__VISIBILITY_CONTROL_H_
#define TRAJECTORY_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TRAJECTORY_PLUGINS_EXPORT __attribute__((dllexport))
#define TRAJECTORY_PLUGINS_IMPORT __attribute__((dllimport))
#else
#define TRAJECTORY_PLUGINS_EXPORT __declspec(dllexport)
#define TRAJECTORY_PLUGINS_IMPORT __declspec(dllimport)
#endif
#ifdef TRAJECTORY_PLUGINS_BUILDING_LIBRARY
#define TRAJECTORY_PLUGINS_PUBLIC TRAJECTORY_PLUGINS_EXPORT
#else
#define TRAJECTORY_PLUGINS_PUBLIC TRAJECTORY_PLUGINS_IMPORT
#endif
#define TRAJECTORY_PLUGINS_PUBLIC_TYPE TRAJECTORY_PLUGINS_PUBLIC
#define TRAJECTORY_PLUGINS_LOCAL
#else
#define TRAJECTORY_PLUGINS_EXPORT __attribute__((visibility("default")))
#define TRAJECTORY_PLUGINS_IMPORT
#if __GNUC__ >= 4
#define TRAJECTORY_PLUGINS_PUBLIC __attribute__((visibility("default")))
#define TRAJECTORY_PLUGINS_LOCAL __attribute__((visibility("hidden")))
#else
#define TRAJECTORY_PLUGINS_PUBLIC
#define TRAJECTORY_PLUGINS_LOCAL
#endif
#define TRAJECTORY_PLUGINS_PUBLIC_TYPE
#endif

#endif  // TRAJECTORY_PLUGINS__VISIBILITY_CONTROL_H_
