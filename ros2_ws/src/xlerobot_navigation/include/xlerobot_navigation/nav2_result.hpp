#pragma once

#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

namespace xlerobot_navigation
{

template<typename Result, typename = void>
struct Nav2Result
{
  // Humble has no application error fields. Callers must still require a
  // non-null result and action status SUCCEEDED; never infer success here.
  static uint16_t error_code(const Result &) {return 0;}
  static std::string detail(const Result &) {return {};}
  static bool collision(const Result &) {return false;}
  static bool timeout(const Result &) {return false;}
};

template<typename Result>
struct Nav2Result<Result, std::void_t<decltype(std::declval<Result>().error_code)>>
{
  static uint16_t error_code(const Result & result) {return result.error_code;}
  static std::string detail(const Result & result) {return result.error_msg;}
  static bool collision(const Result & result)
  {return result.error_code == Result::COLLISION_AHEAD;}
  static bool timeout(const Result & result) {return result.error_code == Result::TIMEOUT;}
};

}  // namespace xlerobot_navigation
