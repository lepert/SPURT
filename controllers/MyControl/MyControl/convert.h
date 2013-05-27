#ifndef CONVERT_H
#define CONVERT_H
#include <string>
#include <iterator>
#include <sstream>

 namespace convert {

template <typename T>
std::string toString (const T &t) {
    std::ostringstream os;
    os << t;
    return os.str();
}


template <typename T>
T fromString (const std::string &str) {
   std::istringstream is(str);
    T t;
    is >> t;
    return t;
}

struct CommaIterator
:
  public std::iterator<std::output_iterator_tag, void, void, void, void>
{
  std::ostream *os;
  std::string comma;
  bool first;

  CommaIterator(std::ostream& os, const std::string& comma):
    os(&os), comma(comma), first(true)
  {
  }

  CommaIterator& operator++() { return *this; }
  CommaIterator& operator++(int) { return *this; }
  CommaIterator& operator*() { return *this; }
  template <class T>
  CommaIterator& operator=(const T& t) {
    if(first)
      first = false;
    else
      *os << comma;
    *os << t;
    return *this;
  }
};

}
#endif
