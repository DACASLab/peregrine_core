/**
 * @note C++ Primer for Python ROS2 readers
 *
 * This file follows a few recurring C++ patterns:
 * - Ownership is explicit: `std::unique_ptr` means single owner, `std::shared_ptr` means shared ownership.
 * - References (`T&`) and `const` are used to avoid unnecessary copies and make mutation intent explicit.
 * - RAII is used for resource safety: objects such as locks clean themselves up automatically at scope exit.
 * - ROS2 callbacks may run concurrently depending on executor/callback-group setup, so shared state is guarded.
 * - Templates (for example `create_subscription<MsgT>`) are compile-time type binding, not runtime reflection.
 */
/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * @defgroup message_version Message Version
 * @ingroup utils
 * This group contains helper functions to handle message versioning.
 */

#pragma once

#include <string>
#include <type_traits>

/**
 * @brief Trait to check if a message type `T` has a `MESSAGE_VERSION` constant.
 *
 * If `T` has `MESSAGE_VERSION`, `HasMessageVersion<T>::value` is `true`;
 * otherwise it is `false`.
 *
 * @tparam T The message type to check.
 */
template <typename T, typename = void>
struct HasMessageVersion : std::false_type
{
};

/// Specialization for types that have a `MESSAGE_VERSION` constant.
/// This uses SFINAE (`std::void_t<...>`) so the specialization participates only when
/// `decltype(T::MESSAGE_VERSION)` is well-formed.
template <typename T>
struct HasMessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>> : std::true_type
{
};

/**
 * @brief Retrieves the version suffix for a given message type.
 *
 * @tparam T The message type, which may or may not define a `MESSAGE_VERSION` constant.
 * @return std::string The version suffix (e.g., "_v1") or an empty string if `MESSAGE_VERSION` is `0` or undefined.
 */
template <typename T>
std::string getMessageNameVersion()
{
  // `if constexpr` is compile-time branching (C++17). The non-selected branch is not
  // instantiated at all, so referencing `T::MESSAGE_VERSION` is safe even when absent.
  if constexpr (HasMessageVersion<T>::value)
  {
    if (T::MESSAGE_VERSION == 0)
    {
      return "";
    }
    return "_v" + std::to_string(T::MESSAGE_VERSION);
  }
  else
  {
    return "";
  }
}
