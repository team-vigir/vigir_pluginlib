#include <vigir_pluginlib/type_class_traits.h>



namespace vigir_pluginlib
{
#ifdef __GNUG__
#include <cstdlib>
#include <memory>
#include <cxxabi.h>

// enable c++11 by passing the flag -std=c++11 to g++
std::string demangle(const char* name)
{
  int status = 0;

  std::unique_ptr<char, void(*)(void*)> res
  {
    abi::__cxa_demangle(name, NULL, NULL, &status),
    std::free
  };

  return (status==0) ? res.get() : name ;
}

#else

// does nothing if not g++
std::string demangle(const char* name)
{
  return name;
}

#endif
} // namespace
