
extern "C" {

#include "io/debug_console.h"

  void controllerSyncCppShim();

}

#include <vector>

class CppTest {
public:
  void execute() {
    std::vector<int> list;

    debugPrint("hello from C++ ");
    list.push_back(1);
    list.push_back(2);
    list.push_back(3);
    for (int i : list) {
      debugPrinti(i);
    }
    debugPrint("\r\n");
  }
};

void controllerSyncCppShim() {
  CppTest cpp_test;
  cpp_test.execute();
}
